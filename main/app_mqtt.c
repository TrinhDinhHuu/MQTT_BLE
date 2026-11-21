/*
 * app_mqtt.c
 *
 *  Created on: 2026年11月4日
 *      Author: SKY-EN-PC01
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>

#include <time.h>

#include "app_mqtt.h"
#include "app_queue.h"
#include "app_time.h"
#include "mqtt5_manager.h"
#include "mqtt_config.h"
#include "status_led.h"
#include "gpio_control.h"
#include "cJSON.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

/* #define MQTT_COMMAND_TOPIC \
//    "skylink/esp32c3/device01/command"

// #define MQTT_EVENT_TOPIC \
//    "skylink/esp32c3/device01/event"

// #define MQTT_STATUS_TOPIC \
//    "skylink/esp32c3/device01/status"
*/
#define GPIO_MQTT_EVENT_QUEUE_LENGTH    20
#define MQTT_SCHEDULE_QUEUE_LENGTH      4
#define MQTT_SCHEDULE_SYNC_TIMEOUT_MS   10000

/*
 * Dữ liệu một GPIO event được lưu trong MQTT TX Queue.
 *
 * has_cycle_info = false:
 *     Event ON/OFF thông thường.
 *
 * has_cycle_info = true:
 *     Event có thông tin cycle_count và cycle_status.
 */
typedef struct
{
    uint8_t gpio;
    bool state;

    bool has_cycle_info;

    uint32_t cycle_count;
    uint32_t cycles_total;

    gpio_cycle_status_t cycle_status;

    /* Thông tin xác nhận Schedule đã lưu NVS */
    bool is_schedule_ack;
    uint8_t schedule_mode;

    uint16_t schedule_on_minute;
    uint16_t schedule_off_minute;

    uint32_t schedule_on_date;
    uint32_t schedule_off_date;

    int64_t saved_at_epoch;
    
    /* Thông báo Schedule thất bại */
    bool is_schedule_error;
    char schedule_error[32];    
    
} gpio_mqtt_event_t;

/*
 * Yêu cầu Schedule nhận từ MQTT.
 * Lưu nguyên payload để Worker xử lý sau khi SNTP sẵn sàng.
 */
typedef struct
{
    size_t length;
    char payload[MQTT_PAYLOAD_MAX_LEN];

} mqtt_schedule_request_t;

static QueueHandle_t s_gpio_mqtt_event_queue = NULL;

/* Queue tiếp nhận yêu cầu Schedule từ MQTT callback */
static QueueHandle_t s_mqtt_schedule_queue = NULL;

/* Quản lý Schedule Worker Task */
static TaskHandle_t s_mqtt_schedule_task_handle = NULL;

#define MQTT_WIFI_CONNECTED_BIT       BIT0
#define MQTT_RETRY_REQUEST_BIT        BIT1
#define MQTT_RECONFIGURE_REQUEST_BIT  BIT2

static EventGroupHandle_t s_mqtt_event_group = NULL;

/* Bảo vệ mqtt_handle khi TX task, callback và Supervisor cùng truy cập */
static SemaphoreHandle_t s_mqtt_mutex = NULL;

/* Đang trong quá trình thay MQTT client */
static volatile bool s_mqtt_reconfiguring = false;
/* Forward declaration */
static esp_err_t mqtt_manager_create_from_config(void);

static volatile bool s_wifi_connected = false;
static bool s_mqtt_started = false;

#define MQTT_TOPIC_MAX_LEN 128

static char MQTT_COMMAND_TOPIC[MQTT_TOPIC_MAX_LEN];
static char MQTT_EVENT_TOPIC[MQTT_TOPIC_MAX_LEN];
static char MQTT_STATUS_TOPIC[MQTT_TOPIC_MAX_LEN];

static const char *TAG = "APP_MQTT";

static mqtt5_manager_handle_t mqtt_handle = NULL;

static bool mqtt_parse_local_datetime(
    const cJSON *item,
    time_t *epoch_out,
    uint32_t *date_out,
    uint16_t *minute_out)
{
    if (!cJSON_IsString(item) ||
        item->valuestring == NULL ||
        epoch_out == NULL ||
        date_out == NULL ||
        minute_out == NULL) {
        return false;
    }

    const char *str = item->valuestring;

    /* Kiểm tra định dạng YYYY-MM-DDTHH:MM:SS */
    if (strlen(str) != 19U ||
        str[4] != '-' ||
        str[7] != '-' ||
        str[10] != 'T' ||
        str[13] != ':' ||
        str[16] != ':') {
        return false;
    }

    /* Các vị trí còn lại phải là chữ số */
    for (size_t i = 0; i < 19U; i++) {
        if (i == 4U || i == 7U ||
            i == 10U || i == 13U || i == 16U) {
            continue;
        }

        if (str[i] < '0' || str[i] > '9') {
            return false;
        }
    }

    int year =
        (str[0]-'0')*1000 +
        (str[1]-'0')*100 +
        (str[2]-'0')*10 +
        (str[3]-'0');

    int month = (str[5]-'0')*10 + (str[6]-'0');
    int day = (str[8]-'0')*10 + (str[9]-'0');
    int hour = (str[11]-'0')*10 + (str[12]-'0');
    int minute = (str[14]-'0')*10 + (str[15]-'0');
    int second = (str[17]-'0')*10 + (str[18]-'0');

    /* Schedule hiện tại chỉ có độ phân giải 1 phút */
    if (year < 2020 ||
        month < 1 || month > 12 ||
        day < 1 || day > 31 ||
        hour > 23 ||
        minute > 59 ||
        second != 0) {
        return false;
    }

    struct tm local_value = {0};

    local_value.tm_year = year - 1900;
    local_value.tm_mon = month - 1;
    local_value.tm_mday = day;
    local_value.tm_hour = hour;
    local_value.tm_min = minute;
    local_value.tm_sec = second;
    local_value.tm_isdst = -1;

    /* Chuyển thời gian local thành Unix timestamp */
    time_t converted = mktime(&local_value);

    if (converted == (time_t)-1) {
        return false;
    }

    /* Kiểm tra lại để phát hiện ngày không tồn tại */
    struct tm verify_value;

    if (localtime_r(&converted, &verify_value) == NULL ||
        verify_value.tm_year != year - 1900 ||
        verify_value.tm_mon != month - 1 ||
        verify_value.tm_mday != day ||
        verify_value.tm_hour != hour ||
        verify_value.tm_min != minute ||
        verify_value.tm_sec != second) {
        return false;
    }

    *epoch_out = converted;

    /* Ngày dạng YYYYMMDD */
    *date_out =
        (uint32_t)year * 10000U +
        (uint32_t)month * 100U +
        (uint32_t)day;

    /* Giờ chuyển thành số phút từ 00:00 */
    *minute_out = (uint16_t)(hour * 60 + minute);

    return true;
}
/**
 * @brief Phân tích giờ local theo định dạng HH:MM.
 *
 * @param item       JSON string cần phân tích.
 * @param minute_out Kết quả tính bằng phút từ 00:00.
 *
 * @return true nếu hợp lệ.
 */
static bool mqtt_parse_local_hhmm(
    const cJSON *item,
    uint16_t *minute_out)
{
    if (!cJSON_IsString(item) ||
        item->valuestring == NULL ||
        minute_out == NULL) {
        return false;
    }

    const char *str = item->valuestring;

    /* Định dạng bắt buộc HH:MM */
    if (strlen(str) != 5U || str[2] != ':') {
        return false;
    }

    /* Kiểm tra các ký tự phải là chữ số */
    if (str[0] < '0' || str[0] > '9' ||
        str[1] < '0' || str[1] > '9' ||
        str[3] < '0' || str[3] > '9' ||
        str[4] < '0' || str[4] > '9') {
        return false;
    }

    uint16_t hour =
        (uint16_t)((str[0] - '0') * 10 +
                   (str[1] - '0'));

    uint16_t minute =
        (uint16_t)((str[3] - '0') * 10 +
                   (str[4] - '0'));

    if (hour > 23U || minute > 59U) {
        return false;
    }

    *minute_out = (uint16_t)(hour * 60U + minute);

    return true;
}
/* =================================================
 * Convert UART packet to JSON
 * ================================================= */
 
#define UART_MQTT_JSON_MAX_LEN (UART_RX_MAX_LEN * 2 + 64)

static bool uart_data_to_json(
    const uart_rx_data_t *packet,
    char *output,
    size_t output_size)
{
    if (packet == NULL ||
        output == NULL ||
        packet->length > UART_RX_MAX_LEN)
    {
        return false;
    }

    int pos = snprintf(
        output,
        output_size,
        "{\"type\":\"uart_rx\",\"length\":%u,\"data\":\"",
        packet->length
    );

    if (pos < 0 || (size_t)pos >= output_size)
    {
        return false;
    }

    for (uint16_t i = 0; i < packet->length; i++)
    {
        int n = snprintf(
            output + pos,
            output_size - pos,
            "%02X",
            packet->data[i]
        );

        if (n < 0 || (size_t)n >= output_size - pos)
        {
            return false;
        }

        pos += n;
    }

    int n = snprintf(
        output + pos,
        output_size - pos,
        "\"}"
    );

    if (n < 0 || (size_t)n >= output_size - pos)
    {
        return false;
    }

    return true;
}

 /*
static bool uart_packet_to_json(
    const uart_mcu_packet_t *packet,
    char *output,
    size_t output_size)
{
    if (packet == NULL || output == NULL) {
        return false;
    }

    int pos = snprintf(
        output,
        output_size,
        "{\"command\":%u,\"length\":%u,\"data\":[",
        packet->command,
        packet->data_length
    );

    if (pos < 0 || (size_t)pos >= output_size) {
        return false;
    }

    for (uint8_t i = 0; i < packet->data_length; i++) {

        int n = snprintf(
            output + pos,
            output_size - pos,
            "%s%u",
            (i == 0) ? "" : ",",
            packet->data[i]
        );

        if (n < 0 || (size_t)n >= output_size - pos) {
            return false;
        }

        pos += n;
    }

    int n = snprintf(
        output + pos,
        output_size - pos,
        "]}"
    );

    if (n < 0 || (size_t)n >= output_size - pos) {
        return false;
    }

    return true;
}
*/
void app_mqtt_set_wifi_connected(bool connected)
{
    s_wifi_connected = connected;

    if (s_mqtt_event_group == NULL) {
        return;
    }

    if (connected) {

        xEventGroupSetBits(
            s_mqtt_event_group,
            MQTT_WIFI_CONNECTED_BIT
        );

        /* Wi-Fi recovered: request MQTT status check */
        /*
         * Wi-Fi đã có IP:
         * yêu cầu Supervisor kiểm tra và kết nối MQTT.
         */        
        if (s_mqtt_started) {
            xEventGroupSetBits(
                s_mqtt_event_group,
                MQTT_RETRY_REQUEST_BIT
            );
        }

  		ESP_LOGI(TAG, "Wi-Fi connected. MQTT retry requested.");
  		
    } else {
        /*
         * Wi-Fi mất kết nối:
         * xóa cả trạng thái Wi-Fi và yêu cầu retry đang chờ.
         */
        xEventGroupClearBits(
            s_mqtt_event_group,
            MQTT_WIFI_CONNECTED_BIT |
            MQTT_RETRY_REQUEST_BIT
        );

        ESP_LOGW(TAG, "Wi-Fi unavailable. MQTT reconnect paused.");
    }
}
/**
 * @brief Chuyển enum cycle status thành chuỗi JSON.
 */
static const char *gpio_cycle_status_to_string(
    gpio_cycle_status_t status)
{
    switch (status)
    {
        case GPIO_CYCLE_STATUS_RUNNING:
            return "running";

        case GPIO_CYCLE_STATUS_COMPLETED:
            return "completed";

        case GPIO_CYCLE_STATUS_CANCELLED:
            return "cancelled";

        case GPIO_CYCLE_STATUS_IDLE:
        default:
            return "idle";
    }
}


/**
 * @brief Hàm nội bộ đưa GPIO event vào Queue.
 */
static esp_err_t mqtt_queue_gpio_event(
    uint8_t gpio,
    bool state,
    bool has_cycle_info,
    uint32_t cycle_count,
    uint32_t cycles_total,
    gpio_cycle_status_t cycle_status)
{
    if (gpio >= 3)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_mqtt_event_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_mqtt_event_t event =
    {
        .gpio = gpio,
        .state = state,

        .has_cycle_info = has_cycle_info,

        .cycle_count = cycle_count,
        .cycles_total = cycles_total,

        .cycle_status = cycle_status
    };

    if (xQueueSend(
            s_gpio_mqtt_event_queue,
            &event,
            0
        ) != pdPASS)
    {
        ESP_LOGW(TAG, "GPIO MQTT event Queue full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}
/**
 * @brief Gửi MQTT event xác nhận lịch đã được lưu vào NVS.
 *
 * @param gpio            GPIO channel: 0, 1, 2.
 * @param mode            0 = ONCE, 1 = DAILY.
 * @param on_minute       Phút ON tính từ 00:00.
 * @param off_minute      Phút OFF tính từ 00:00.
 * @param on_date         Ngày ON dạng YYYYMMDD.
 * @param off_date        Ngày OFF dạng YYYYMMDD.
 * @param saved_at_epoch  Thời điểm lưu NVS (Unix timestamp).
 */
esp_err_t app_mqtt_publish_schedule_saved(
    uint8_t gpio,
    uint8_t mode,
    uint16_t on_minute,
    uint16_t off_minute,
    uint32_t on_date,
    uint32_t off_date,
    int64_t saved_at_epoch
);

/*
 * Đưa thông báo Schedule thất bại vào MQTT TX Queue.
 * Không publish trực tiếp từ Schedule Worker.
 */
static esp_err_t mqtt_publish_schedule_error(const char *error)
{
    if (error == NULL || s_gpio_mqtt_event_queue == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_mqtt_event_t event = {0};

    event.is_schedule_error = true;

    snprintf(
        event.schedule_error,
        sizeof(event.schedule_error),
        "%s",
        error
    );

    if (xQueueSend(
            s_gpio_mqtt_event_queue,
            &event,
            0
        ) != pdPASS)
    {
        ESP_LOGW(TAG, "Cannot queue Schedule error event");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

/**
 * @brief Gửi event GPIO ON/OFF thông thường.
 */
esp_err_t app_mqtt_publish_gpio_state(uint8_t gpio, bool state)
{
    return mqtt_queue_gpio_event(
        gpio,
        state,
        false,
        0,
        0,
        GPIO_CYCLE_STATUS_IDLE
    );
}


/**
 * @brief Gửi event GPIO kèm thông tin cycle.
 */
esp_err_t app_mqtt_publish_gpio_cycle_event(
    uint8_t gpio,
    bool state,
    uint32_t cycle_count,
    uint32_t cycles_total,
    gpio_cycle_status_t cycle_status)
{
    return mqtt_queue_gpio_event(
        gpio,
        state,
        true,
        cycle_count,
        cycles_total,
        cycle_status
    );
}
/**
 * @brief Đưa xác nhận Schedule đã lưu NVS vào MQTT TX Queue.
 */
esp_err_t app_mqtt_publish_schedule_saved(
    uint8_t gpio,
    uint8_t mode,
    uint16_t on_minute,
    uint16_t off_minute,
    uint32_t on_date,
    uint32_t off_date,
    int64_t saved_at_epoch)
{
    if (gpio >= 3 || mode > 1) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_mqtt_event_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_mqtt_event_t event = {
        .gpio = gpio,
        .is_schedule_ack = true,
        .schedule_mode = mode,
        .schedule_on_minute = on_minute,
        .schedule_off_minute = off_minute,
        .schedule_on_date = on_date,
        .schedule_off_date = off_date,
        .saved_at_epoch = saved_at_epoch
    };

    if (xQueueSend(
            s_gpio_mqtt_event_queue,
            &event,
            0
        ) != pdPASS) {

        ESP_LOGW(TAG, "Schedule ACK Queue full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}
/**
 * @brief Đưa trạng thái GPIO vào Queue để MQTT TX Task publish.

esp_err_t app_mqtt_publish_gpio_state(uint8_t gpio, bool state)
{
    if (gpio >= 3)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_mqtt_event_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_mqtt_event_t event =
    {
        .gpio = gpio,
        .state = state
    };

    if (xQueueSend(
            s_gpio_mqtt_event_queue,
            &event,
            0
        ) != pdPASS)
    {
        ESP_LOGW(TAG, "GPIO MQTT event Queue full");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(
        TAG,
        "GPIO event queued: gpio=%u, state=%s",
        (unsigned int)gpio,
        state ? "ON" : "OFF"
    );

    return ESP_OK;
}
 */
 
/**
 * @brief Chuyển duration từ JSON thành microseconds.
 *
 * time_unit = "s"  : độ phân giải 0.1 giây
 * time_unit = "ms" : độ phân giải 50 ms
 */
static bool mqtt_parse_duration_us(
    const cJSON *duration_item,
    const char *time_unit,
    uint64_t *duration_us)
{
    if (!cJSON_IsNumber(duration_item) ||
        time_unit == NULL ||
        duration_us == NULL)
    {
        return false;
    }

    double value = duration_item->valuedouble;

    if (!isfinite(value) || value <= 0.0)
    {
        return false;
    }

    if (strcmp(time_unit, "s") == 0)
    {
        /* Đổi sang đơn vị 0.1 giây */
        double scaled = value * 10.0;

        if (scaled > (double)(INT64_MAX / 100000ULL))
        {
            return false;
        }

        uint64_t steps = (uint64_t)(scaled + 0.5);

        /* Chỉ chấp nhận bội số 0.1 giây */
        if (fabs(scaled - (double)steps) > 0.000001)
        {
            return false;
        }

        *duration_us = steps * 100000ULL;
        return true;
    }

    if (strcmp(time_unit, "ms") == 0)
    {
        if (value > (double)(INT64_MAX / 1000ULL))
        {
            return false;
        }

        uint64_t milliseconds = (uint64_t)value;

        /* Phải là số nguyên */
        if ((double)milliseconds != value)
        {
            return false;
        }

        /* Độ phân giải 50 ms */
        if ((milliseconds % 50ULL) != 0)
        {
            return false;
        }

        *duration_us = milliseconds * 1000ULL;
        return true;
    }

    return false;
}


/**
 * @brief Phân tích và xử lý MQTT GPIO command.
 *
 * Hỗ trợ:
 * - on / off
 * - on / off + duration
 * - cycle
 */
static bool mqtt_handle_gpio_command(
    const char *payload,
    size_t payload_len,
    bool from_schedule_worker)
{
    if (payload == NULL || payload_len == 0)
    {
        return false;
    }

    cJSON *root = cJSON_Parse(payload);

    if (root == NULL)
    {
        ESP_LOGW(TAG, "Invalid GPIO JSON");
        return false;
    }

    cJSON *type = cJSON_GetObjectItemCaseSensitive(root, "type");

    if (!cJSON_IsString(type) ||
        type->valuestring == NULL ||
        strcmp(type->valuestring, "gpio") != 0)
    {
        cJSON_Delete(root);
        return false;
    }

    /* Đã xác định đây là GPIO command */
    cJSON *gpio = cJSON_GetObjectItemCaseSensitive(root, "gpio");
    cJSON *action = cJSON_GetObjectItemCaseSensitive(root, "action");

    if (!cJSON_IsNumber(gpio) ||
        !cJSON_IsString(action) ||
        action->valuestring == NULL)
    {
        ESP_LOGW(TAG, "Invalid GPIO command format");
        cJSON_Delete(root);
        return true;
    }

    /* Kiểm tra GPIO channel */
    double gpio_value = gpio->valuedouble;

    if (!isfinite(gpio_value) ||
        gpio_value < 0 ||
        gpio_value > 2 ||
        gpio_value != (double)(int)gpio_value)
    {
        ESP_LOGW(TAG, "Invalid GPIO channel");
        cJSON_Delete(root);
        return true;
    }

    uint8_t channel = (uint8_t)gpio_value;

    cJSON *time_unit =
        cJSON_GetObjectItemCaseSensitive(root, "time_unit");

    esp_err_t err = ESP_ERR_INVALID_ARG;
    
	/* =================================================
	 * ACTION: SCHEDULE ONCE
	 * ================================================= */
	if (strcmp(action->valuestring, "schedule_once") == 0)
	{
	    cJSON *on_time =
	        cJSON_GetObjectItemCaseSensitive(root, "on_time");
	
	    cJSON *off_time =
	        cJSON_GetObjectItemCaseSensitive(root, "off_time");
	
	    time_t on_epoch = 0;
	    time_t off_epoch = 0;
	
	    uint32_t on_date = 0;
	    uint32_t off_date = 0;
	
	    uint16_t on_minute = 0;
	    uint16_t off_minute = 0;
	
	/*
	 * Schedule phải được chuyển qua Worker để kiểm tra SNTP.
	 * Không chờ SNTP trong MQTT callback.
	 */
	if (!from_schedule_worker)
	{
	    mqtt_schedule_request_t request = {0};
	
	    if (payload_len >= sizeof(request.payload))
	    {
	        ESP_LOGW(TAG, "Schedule payload too large");
	        cJSON_Delete(root);
	        return true;
	    }
	
	    memcpy(request.payload, payload, payload_len);
	    request.payload[payload_len] = '\0';
	    request.length = payload_len;
	
	    if (s_mqtt_schedule_queue == NULL ||
	        xQueueSend(
	            s_mqtt_schedule_queue,
	            &request,
	            0
	        ) != pdPASS)
	    {
	        ESP_LOGW(TAG, "Schedule request Queue full");
	        mqtt_publish_schedule_error("QUEUE_FULL");
	    }
	
	    cJSON_Delete(root);
	    return true;
	}
	
	    /* Phân tích thời gian ON và OFF */
	    if (!mqtt_parse_local_datetime(
	            on_time,
	            &on_epoch,
	            &on_date,
	            &on_minute) ||
	
	        !mqtt_parse_local_datetime(
	            off_time,
	            &off_epoch,
	            &off_date,
	            &off_minute)) {
	
	        ESP_LOGW(TAG, "Invalid schedule_once datetime");
	
	        cJSON_Delete(root);
	        return true;
	    }
	
	    /* Kiểm tra thời gian hiện tại */
	    time_t now = time(NULL);
	
	    if (now == (time_t)-1 ||
	        on_epoch <= now ||
	        off_epoch <= on_epoch) {
	
	        ESP_LOGW(TAG,
	                 "Invalid schedule: ON must be future and OFF after ON");
	
	        cJSON_Delete(root);
	        return true;
	    }
	
	    /* Gửi lịch sang GPIO Control */
	    err = gpio_control_set_relay_schedule_once(
	        channel,
	        on_date,
	        on_minute,
	        off_date,
	        off_minute);
	} else if (strcmp(action->valuestring, "schedule_daily") == 0)    {
	/* =================================================
     * ACTION: SCHEDULE DAILY
     * ================================================= */
        /*
         * Chuyển Schedule DAILY qua Worker.
         */
        if (!from_schedule_worker)
        {
            mqtt_schedule_request_t request = {0};

            if (payload_len >= sizeof(request.payload))
            {
                ESP_LOGW(TAG, "Schedule payload too large");
                cJSON_Delete(root);
                return true;
            }

            memcpy(request.payload, payload, payload_len);
            request.payload[payload_len] = '\0';
            request.length = payload_len;

            if (s_mqtt_schedule_queue == NULL ||
                xQueueSend(
                    s_mqtt_schedule_queue,
                    &request,
                    0
                ) != pdPASS)
            {
                ESP_LOGW(TAG, "Schedule request Queue full");
                mqtt_publish_schedule_error("QUEUE_FULL");
            }

            cJSON_Delete(root);
            return true;
        }

        /* Worker đã xác nhận thời gian sẵn sàng */     
        cJSON *on_time =
            cJSON_GetObjectItemCaseSensitive(root, "on_time");

        cJSON *off_time =
            cJSON_GetObjectItemCaseSensitive(root, "off_time");

        uint16_t on_minute = 0;
        uint16_t off_minute = 0;

        /* Phân tích giờ ON */
        if (!mqtt_parse_local_hhmm(on_time, &on_minute))
        {
            ESP_LOGW(TAG, "Invalid schedule_daily on_time");

            cJSON_Delete(root);
            return true;
        }

        /* Phân tích giờ OFF */
        if (!mqtt_parse_local_hhmm(off_time, &off_minute))
        {
            ESP_LOGW(TAG, "Invalid schedule_daily off_time");

            cJSON_Delete(root);
            return true;
        }

        /* Không cho phép ON và OFF cùng thời điểm */
        if (on_minute == off_minute)
        {
            ESP_LOGW(TAG, "schedule_daily ON and OFF must be different");

            cJSON_Delete(root);
            return true;
        }

        /* Gửi cấu hình DAILY đến GPIO Control Task */
        err = gpio_control_set_relay_schedule(
            channel,
            true,
            GPIO_SCHEDULE_DAILY,
            on_minute,
            off_minute
        );

        ESP_LOGI(
            TAG,
            "Schedule DAILY: Relay=%u, ON=%02u:%02u, OFF=%02u:%02u",
            (unsigned int)(channel + 1),
            on_minute / 60U,
            on_minute % 60U,
            off_minute / 60U,
            off_minute % 60U
        );
    } else if (strcmp(action->valuestring, "cycle") == 0)    {
	/* =================================================
     * ACTION: CYCLE
     * ================================================= */
        cJSON *on_duration =
            cJSON_GetObjectItemCaseSensitive(root, "on_duration");

        cJSON *off_duration =
            cJSON_GetObjectItemCaseSensitive(root, "off_duration");

        cJSON *cycles =
            cJSON_GetObjectItemCaseSensitive(root, "cycles");

        uint64_t on_duration_us;
        uint64_t off_duration_us;

        /* Kiểm tra time_unit */
        if (!cJSON_IsString(time_unit) ||
            time_unit->valuestring == NULL)
        {
            ESP_LOGW(TAG, "Missing or invalid time_unit");
            cJSON_Delete(root);
            return true;
        }

        /* Kiểm tra ON duration */
        if (!mqtt_parse_duration_us(
                on_duration,
                time_unit->valuestring,
                &on_duration_us))
        {
            ESP_LOGW(TAG, "Invalid on_duration");
            cJSON_Delete(root);
            return true;
        }

        /* Kiểm tra OFF duration */
        if (!mqtt_parse_duration_us(
                off_duration,
                time_unit->valuestring,
                &off_duration_us))
        {
            ESP_LOGW(TAG, "Invalid off_duration");
            cJSON_Delete(root);
            return true;
        }

        /* cycles phải là số nguyên từ 0 đến UINT32_MAX */
        if (!cJSON_IsNumber(cycles) ||
            !isfinite(cycles->valuedouble) ||
            cycles->valuedouble < 0 ||
            cycles->valuedouble > (double)UINT32_MAX ||
            cycles->valuedouble !=
                (double)(uint32_t)cycles->valuedouble)
        {
            ESP_LOGW(TAG, "Invalid cycles");
            cJSON_Delete(root);
            return true;
        }

        uint32_t cycle_total = (uint32_t)cycles->valuedouble;

        /* Gửi lệnh Cycle đến GPIO Control Task */
        err = gpio_control_set_relay_cycle(
            channel,
            on_duration_us,
            off_duration_us,
            cycle_total
        );
    } else if (strcmp(action->valuestring, "on") == 0 ||
             strcmp(action->valuestring, "off") == 0)  {
	/* =================================================
     * ACTION: ON / OFF
     * ================================================= */
        bool state = (strcmp(action->valuestring, "on") == 0);

        cJSON *duration =
            cJSON_GetObjectItemCaseSensitive(root, "duration");

        if (duration != NULL)
        {
            uint64_t duration_us;

            if (!cJSON_IsString(time_unit) ||
                time_unit->valuestring == NULL ||
                !mqtt_parse_duration_us(
                    duration,
                    time_unit->valuestring,
                    &duration_us))
            {
                ESP_LOGW(TAG, "Invalid GPIO duration");
                cJSON_Delete(root);
                return true;
            }

            err = gpio_control_set_relay_duration(
                channel,
                state,
                duration_us
            );
        }
        else
        {
            err = gpio_control_set_relay(
                channel,
                state
            );
        }
    } else {
        ESP_LOGW(
            TAG,
            "Unsupported GPIO action: %s",
            action->valuestring
        );

        cJSON_Delete(root);
        return true;
    }

    /* =================================================
     * RESULT
     * ================================================= */
    if (err == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "GPIO command accepted: gpio=%u, action=%s",
            (unsigned int)channel,
            action->valuestring
        );
    }
    else
    {
        ESP_LOGW(
            TAG,
            "Cannot queue GPIO command: %s",
            esp_err_to_name(err)
        );
    }

    cJSON_Delete(root);

    return true;
}

/* =================================================
 * Schedule Worker Task
 * ================================================= */
static void mqtt_schedule_worker_task(void *arg)
{
    (void)arg;

    mqtt_schedule_request_t request;

    ESP_LOGI(TAG, "MQTT Schedule Worker started");

    while (1)
    {
        /* Chờ yêu cầu Schedule từ MQTT callback */
        if (xQueueReceive(
                s_mqtt_schedule_queue,
                &request,
                portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        /* Kiểm tra dữ liệu nhận được */
        if (request.length == 0 ||
            request.length >= sizeof(request.payload))
        {
            ESP_LOGW(TAG, "Invalid schedule request");

            mqtt_publish_schedule_error("INVALID_PAYLOAD");
            continue;
        }

        /* Đảm bảo payload kết thúc bằng NULL */
        request.payload[request.length] = '\0';

        /*
         * Kiểm tra đồng bộ thời gian.
         * Không chờ SNTP nếu thời gian đã sẵn sàng.
         */
        if (!app_time_is_ready())
        {
            ESP_LOGW(TAG, "System time not ready. Requesting SNTP sync");

            esp_err_t err = app_time_request_sync();

            if (err != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "Cannot request SNTP sync: %s",
                    esp_err_to_name(err)
                );

                mqtt_publish_schedule_error(
                    "TIME_SYNC_REQUEST_FAILED"
                );

                continue;
            }

            /* Chờ SNTP tối đa 10 giây */
            if (!app_time_wait_for_sync(
                    MQTT_SCHEDULE_SYNC_TIMEOUT_MS))
            {
                ESP_LOGE(TAG, "SNTP synchronization timeout");

                mqtt_publish_schedule_error(
                    "TIME_SYNC_TIMEOUT"
                );

                continue;
            }
        }

        /* Kiểm tra lại sau khi chờ đồng bộ */
        if (!app_time_is_ready())
        {
            ESP_LOGE(TAG, "System time is still invalid");

            mqtt_publish_schedule_error(
                "TIME_NOT_SYNCED"
            );

            continue;
        }

        ESP_LOGI(TAG, "System time synchronized");

        /*
         * Xử lý Schedule trong Worker.
         * true: không đưa yêu cầu trở lại Schedule Queue.
         */
        mqtt_handle_gpio_command(
            request.payload,
            request.length,
            true
        );
    }
}

/* =================================================
 * MQTT RX callback
 * ================================================= */
static void mqtt_event_callback(
    mqtt5_manager_handle_t handle,
    mqtt5_manager_event_t event,
    const mqtt5_manager_data_t *data,
    void *user_data)
{
    (void)user_data;

	if (s_mqtt_mutex == NULL) {
	    return;
	}
	
	if (xSemaphoreTake(
	        s_mqtt_mutex,
	        pdMS_TO_TICKS(1000)
	    ) != pdTRUE) {
	    return;
	}
	
	/* Nếu Supervisor đang thay client thì bỏ qua callback */
	if (s_mqtt_reconfiguring) {
	    xSemaphoreGive(s_mqtt_mutex);
	    return;
	}

    switch (event) {

    case MQTT5_MANAGER_CONNECTED:

        ESP_LOGI(TAG, "HiveMQ connected");

	    /* RGB LED: GREEN */
	    status_led_set_mqtt(true);
	    
	    xEventGroupClearBits(
	        s_mqtt_event_group,
	        MQTT_RETRY_REQUEST_BIT
	    );	    
	    
        mqtt5_manager_subscribe(
            handle,
            MQTT_COMMAND_TOPIC,
            1
        );

		mqtt5_manager_publish(
		    handle,
		    MQTT_STATUS_TOPIC,
		    "{\"status\":\"online\"}",
		    1,
		    true
		);

        break;

    case MQTT5_MANAGER_DISCONNECTED:

        ESP_LOGW(TAG, "HiveMQ disconnected");
        
	     /* MQTT mất kết nối: chuyển LED về trạng thái Wi-Fi-only */
	    status_led_set_mqtt(false);  
	    
	    /*
	     * Chỉ yêu cầu reconnect khi Wi-Fi vẫn có IP.
	     * Nếu Wi-Fi mất, chờ app_mqtt_set_wifi_connected(true).
	     */
	    if (s_mqtt_event_group != NULL &&
	        (xEventGroupGetBits(s_mqtt_event_group) &
	         MQTT_WIFI_CONNECTED_BIT)) {
	
	        xEventGroupSetBits(
	            s_mqtt_event_group,
	            MQTT_RETRY_REQUEST_BIT
	        );
	    }
	    
        break;

    case MQTT5_MANAGER_SUBSCRIBED:

        ESP_LOGI(TAG, "MQTT subscribe successful");
        break;

    case MQTT5_MANAGER_PUBLISHED:

        ESP_LOGI(TAG, "MQTT publish acknowledged");
        break;

    case MQTT5_MANAGER_DATA:

        if (data == NULL ||
            data->topic == NULL ||
            data->data == NULL) {
            break;
        }

        ESP_LOGI(TAG, "MQTT RX topic: %.*s",
                 data->topic_len, data->topic);

        /*
         * Only accept command topic.
         */
        if (data->topic_len != strlen(MQTT_COMMAND_TOPIC) ||
            memcmp(data->topic,
                   MQTT_COMMAND_TOPIC,
                   data->topic_len) != 0) {
            break;
        }

        if (data->data_len <= 0 ||
            data->data_len >= MQTT_PAYLOAD_MAX_LEN) {
            ESP_LOGW(TAG, "Invalid MQTT command length");
            break;
        }

        /*
         * MQTT GPIO command:
         * Xử lý riêng, không chuyển sang UART.
         */
        char payload[MQTT_PAYLOAD_MAX_LEN];

        memcpy(
            payload,
            data->data,
            data->data_len
        );

        payload[data->data_len] = '\0';

		if (mqtt_handle_gpio_command(
		        payload,
		        data->data_len,
		        false))
		{
		    break;
		}

        /*
         * Không phải GPIO command:
         * Giữ nguyên luồng MQTT -> UART hiện tại.
         */
		mqtt_command_t command = {0};
		
		/* Kiểm tra kích thước buffer đích trước khi memcpy */
		if (data->data_len >= sizeof(command.payload))
		{
		    ESP_LOGW(TAG, "MQTT->UART payload too large");
		    break;
		}
		
		memcpy(
		    command.payload,
		    data->data,
		    data->data_len
		);

        command.length = data->data_len;

        if (xQueueSend(
                mqtt_to_uart_queue,
                &command,
                0
            ) != pdPASS)
        {
            ESP_LOGW(TAG, "MQTT->UART queue full");
        }
        else
        {
            ESP_LOGI(TAG, "MQTT command queued for UART");
        }

        break;

    case MQTT5_MANAGER_ERROR:

        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
    
    xSemaphoreGive(s_mqtt_mutex);
}

/* =================================================
 * MQTT TX Task
 * ================================================= */
static void mqtt_tx_task(void *arg)
{
    (void)arg;

    //uart_mcu_packet_t packet;
    //char json_payload[MQTT_PAYLOAD_MAX_LEN];
    uart_rx_data_t packet;
    gpio_mqtt_event_t gpio_event;
    
	char json_payload[UART_MQTT_JSON_MAX_LEN];

    ESP_LOGI(TAG, "MQTT TX task started");

    while (1) {

        /*
         * Chỉ gửi dữ liệu khi MQTT đã kết nối.
         * Nếu mất kết nối, dữ liệu vẫn nằm trong Queue.
         */

		if (s_mqtt_reconfiguring) {
		    vTaskDelay(pdMS_TO_TICKS(200));
		    continue;
		}
		
		if (xSemaphoreTake(
		        s_mqtt_mutex,
		        pdMS_TO_TICKS(1000)
		    ) != pdTRUE) {
		    continue;
		}
		
		bool connected = false;
		
		if (!s_mqtt_reconfiguring && mqtt_handle != NULL) {
		    connected = mqtt5_manager_is_connected(mqtt_handle);
		}
		
		xSemaphoreGive(s_mqtt_mutex);
		
		if (!connected) {
		    vTaskDelay(pdMS_TO_TICKS(500));
		    continue;
		}

        bool is_gpio_event = false;
        bool has_packet = false;

        /*
         * Ưu tiên GPIO event.
         * GPIO event được tạo bởi cả Switch và MQTT command.
         */
        if (s_gpio_mqtt_event_queue != NULL &&
            xQueuePeek(
                s_gpio_mqtt_event_queue,
                &gpio_event,
                0
            ) == pdPASS)
        {

			int len;
			
			if (gpio_event.is_schedule_error)
			{
			    len = snprintf(
			        json_payload,
			        sizeof(json_payload),
			        "{"
			            "\"type\":\"schedule_result\","
			            "\"status\":\"failed\","
			            "\"error\":\"%s\""
			        "}",
			        gpio_event.schedule_error
			    );
			}
			else if (gpio_event.is_schedule_ack)
			{
			    char saved_at[24] = {0};
			    char on_time[24] = {0};
			    char off_time[24] = {0};
			
			    time_t saved_epoch = (time_t)gpio_event.saved_at_epoch;
			    struct tm saved_tm;
			
			    if (localtime_r(&saved_epoch, &saved_tm) == NULL) {
			        ESP_LOGE(TAG, "Cannot convert Schedule saved time");
			
			        xQueueReceive(
			            s_gpio_mqtt_event_queue,
			            &gpio_event,
			            0
			        );
			
			        continue;
			    }
			
			    strftime(
			        saved_at,
			        sizeof(saved_at),
			        "%Y-%m-%dT%H:%M:%S",
			        &saved_tm
			    );
			
			    if (gpio_event.schedule_mode == GPIO_SCHEDULE_DAILY)
			    {
			        snprintf(
			            on_time,
			            sizeof(on_time),
			            "%02u:%02u",
			            gpio_event.schedule_on_minute / 60U,
			            gpio_event.schedule_on_minute % 60U
			        );
			
			        snprintf(
			            off_time,
			            sizeof(off_time),
			            "%02u:%02u",
			            gpio_event.schedule_off_minute / 60U,
			            gpio_event.schedule_off_minute % 60U
			        );
			
			        len = snprintf(
			            json_payload,
			            sizeof(json_payload),
			            "{\"type\":\"schedule_saved\","
			            "\"gpio\":%u,"
			            "\"mode\":\"daily\","
			            "\"on_time\":\"%s\","
			            "\"off_time\":\"%s\","
			            "\"saved_at\":\"%s\"}",
			            (unsigned int)gpio_event.gpio,
			            on_time,
			            off_time,
			            saved_at
			        );
			    }
			    else
			    {
			        uint32_t on_date = gpio_event.schedule_on_date;
			        uint32_t off_date = gpio_event.schedule_off_date;
			
			        snprintf(
			            on_time,
			            sizeof(on_time),
			            "%04lu-%02lu-%02luT%02u:%02u:00",
			            (unsigned long)(on_date / 10000U),
			            (unsigned long)((on_date / 100U) % 100U),
			            (unsigned long)(on_date % 100U),
			            gpio_event.schedule_on_minute / 60U,
			            gpio_event.schedule_on_minute % 60U
			        );
			
			        snprintf(
			            off_time,
			            sizeof(off_time),
			            "%04lu-%02lu-%02luT%02u:%02u:00",
			            (unsigned long)(off_date / 10000U),
			            (unsigned long)((off_date / 100U) % 100U),
			            (unsigned long)(off_date % 100U),
			            gpio_event.schedule_off_minute / 60U,
			            gpio_event.schedule_off_minute % 60U
			        );
			
			        len = snprintf(
			            json_payload,
			            sizeof(json_payload),
			            "{\"type\":\"schedule_saved\","
			            "\"gpio\":%u,"
			            "\"mode\":\"once\","
			            "\"on_time\":\"%s\","
			            "\"off_time\":\"%s\","
			            "\"saved_at\":\"%s\"}",
			            (unsigned int)gpio_event.gpio,
			            on_time,
			            off_time,
			            saved_at
			        );
			    }
			}
			else if (gpio_event.has_cycle_info)
			{
			    // JSON có thông tin bộ đếm Cycle
			    len = snprintf(json_payload, sizeof(json_payload),
			                   "{\"type\":\"gpio\","
			                   "\"gpio\":%u,"
			                   "\"state\":\"%s\","
			                   "\"cycle_count\":%lu,"
			                   "\"cycles_total\":%lu,"
			                   "\"cycle_status\":\"%s\"}",
			                   (unsigned int)gpio_event.gpio,
			                   gpio_event.state ? "on" : "off",
			                   (unsigned long)gpio_event.cycle_count,
			                   (unsigned long)gpio_event.cycles_total,
			                   gpio_cycle_status_to_string(gpio_event.cycle_status));
			}
			else
			{
			    // JSON thông thường, giữ tương thích với sự kiện GPIO cũ
			    len = snprintf(json_payload, sizeof(json_payload),
			                   "{\"type\":\"gpio\",\"gpio\":%u,\"state\":\"%s\"}",
			                   (unsigned int)gpio_event.gpio,
			                   gpio_event.state ? "on" : "off");
			}
			
			/*/ Kiểm tra độ dài JSON
			if (len < 0 || (size_t)len >= sizeof(json_payload))
			{
			    ESP_LOGE(TAG, "GPIO JSON payload buffer too small");
			    xQueueReceive(s_gpio_mqtt_event_queue, &gpio_event, 0);
			    continue;
			}*/
			            

            if (len < 0 || len >= sizeof(json_payload))
            {
                ESP_LOGE(TAG, "GPIO JSON conversion failed");

                xQueueReceive(
                    s_gpio_mqtt_event_queue,
                    &gpio_event,
                    0
                );

                continue;
            }

            is_gpio_event = true;
            has_packet = true;
        }
        else {
            /*
             * Không có GPIO event:
             * Xử lý UART event như cơ chế hiện tại.
             */
	        if (xQueuePeek(
	                uart_to_mqtt_queue,
	                &packet,
	                pdMS_TO_TICKS(500)
	            ) != pdPASS) {
	            continue;
	        }
	
	        if (!uart_data_to_json(
			    &packet,
			    json_payload,
			    sizeof(json_payload)))
			{
	
	            ESP_LOGE(TAG, "JSON conversion failed");
	
	            xQueueReceive(
	                uart_to_mqtt_queue,
	                &packet,
	                0
	            );
	
	            continue;
	        }
	        
	        has_packet = true;
	                     		
		}
  
        if (!has_packet)
        {
            continue;
        }                

        /*
         * Publish MQTT.
         */
		int msg_id = -1;
		
		if (xSemaphoreTake(
		        s_mqtt_mutex,
		        pdMS_TO_TICKS(1000)
		    ) == pdTRUE) {
		
		    if (!s_mqtt_reconfiguring && mqtt_handle != NULL &&
		        mqtt5_manager_is_connected(mqtt_handle)) {
		
		        msg_id = mqtt5_manager_publish(
		            mqtt_handle,
		            MQTT_EVENT_TOPIC,
		            json_payload,
		            1,
		            false
		        );
		    }
		
		    xSemaphoreGive(s_mqtt_mutex);
		}
		
        /*
         * Chỉ lấy event khỏi Queue khi MQTT manager
         * đã chấp nhận yêu cầu publish.
         */		
        if (msg_id >= 0) {
            if (is_gpio_event)
            {
                xQueueReceive(
                    s_gpio_mqtt_event_queue,
                    &gpio_event,
                    0
                );
            }
            else {
	            xQueueReceive(
	                uart_to_mqtt_queue,
	                &packet,
	                0
	            );		
			}
			
            ESP_LOGI(
                TAG,
                "MQTT publish accepted, msg_id=%d",
                msg_id
            );

            ESP_LOGI(TAG, "Payload: %s", json_payload);            
            
        } else {

            ESP_LOGW(TAG, "MQTT publish failed");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
static TaskHandle_t s_mqtt_tx_task_handle = NULL;

/* =================================================
 * Start MQTT TX Task
 * ================================================= */
static esp_err_t mqtt_tx_task_start(void)
{
	if (s_mqtt_tx_task_handle != NULL) {
        return ESP_OK;
    }
    BaseType_t result = xTaskCreate(
        mqtt_tx_task,
        "mqtt_tx_task",
        4096,
        NULL,
        8,
         &s_mqtt_tx_task_handle
    );

    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}
//static TaskHandle_t s_mqtt_supervisor_task_handle = NULL;
static void mqtt_supervisor_task(void *arg)
{
    (void)arg;

    while (1) {

        /*
         * Chờ một trong các yêu cầu:
         * - MQTT reconnect
         * - MQTT reconfigure
         *
         * pdFALSE: không tự xóa bit sau khi nhận.
         * pdFALSE: chỉ cần một bit được set là thức dậy.
         */
        EventBits_t bits = xEventGroupWaitBits(
            s_mqtt_event_group,
            MQTT_RETRY_REQUEST_BIT |
            MQTT_RECONFIGURE_REQUEST_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY
        );

        /* ==========================================
         * ƯU TIÊN 1: RECONFIGURE MQTT
         * ========================================== */
        if (bits & MQTT_RECONFIGURE_REQUEST_BIT) {

            xEventGroupClearBits(
                s_mqtt_event_group,
                MQTT_RECONFIGURE_REQUEST_BIT
            );

            ESP_LOGW(TAG, "MQTT reconfiguration requested");

            s_mqtt_reconfiguring = true;

            /*
             * Chờ các thao tác TX/callback đang sử dụng
             * MQTT client hoàn tất.
             */
            if (xSemaphoreTake(
                    s_mqtt_mutex,
                    portMAX_DELAY
                ) != pdTRUE) {

                ESP_LOGE(TAG, "Cannot lock MQTT mutex");
                s_mqtt_reconfiguring = false;
                continue;
            }

            mqtt5_manager_handle_t old_handle = mqtt_handle;

            /*
             * Ngăn task khác lấy handle cũ.
             */
            mqtt_handle = NULL;

            xSemaphoreGive(s_mqtt_mutex);

            /* Dừng và giải phóng client cũ */
            if (old_handle != NULL) {

                esp_err_t err = mqtt5_manager_stop(old_handle);

                if (err != ESP_OK) {

                    ESP_LOGE(
                        TAG,
                        "Stop old MQTT client failed: %s",
                        esp_err_to_name(err)
                    );

                    /*
                     * Không destroy client nếu stop thất bại.
                     * Khôi phục handle để có thể tiếp tục sử dụng.
                     */
                    if (xSemaphoreTake(
                            s_mqtt_mutex,
                            portMAX_DELAY
                        ) == pdTRUE) {

                        mqtt_handle = old_handle;
                        xSemaphoreGive(s_mqtt_mutex);
                    }

                    s_mqtt_reconfiguring = false;

                    /* Yêu cầu thử lại sau */
                    vTaskDelay(pdMS_TO_TICKS(3000));

                    xEventGroupSetBits(
                        s_mqtt_event_group,
                        MQTT_RECONFIGURE_REQUEST_BIT
                    );

                    continue;
                }

                err = mqtt5_manager_deinit(old_handle);

                if (err != ESP_OK) {

                    ESP_LOGE(
                        TAG,
                        "Destroy old MQTT manager failed: %s",
                        esp_err_to_name(err)
                    );

                    s_mqtt_reconfiguring = false;

                    /*
                     * Không thể đảm bảo trạng thái manager cũ
                     * sau lỗi destroy. Không tạo client mới để
                     * tránh tồn tại hai MQTT client.
                     */
                    continue;
                }

                ESP_LOGI(TAG, "Old MQTT client destroyed");
            }

            /* Tạo MQTT manager mới từ cấu hình NVS hiện tại */
            esp_err_t err = mqtt_manager_create_from_config();

            if (err != ESP_OK) {

                ESP_LOGE(
                    TAG,
                    "Create new MQTT manager failed: %s",
                    esp_err_to_name(err)
                );

                s_mqtt_reconfiguring = false;

                /*
                 * Cấu hình đã được lưu trong NVS.
                 * Có thể yêu cầu Supervisor thử khởi tạo lại.
                 */
                vTaskDelay(pdMS_TO_TICKS(3000));

                xEventGroupSetBits(
                    s_mqtt_event_group,
                    MQTT_RECONFIGURE_REQUEST_BIT
                );

                continue;
            }

            ESP_LOGI(TAG, "New MQTT manager created");

            s_mqtt_reconfiguring = false;

            /*
             * Chỉ start MQTT nếu Wi-Fi đã có IP.
             */
            bits = xEventGroupGetBits(s_mqtt_event_group);

            if (bits & MQTT_WIFI_CONNECTED_BIT) {

                ESP_LOGI(TAG, "Wi-Fi available. Starting new MQTT client");

                //err = mqtt5_manager_start(mqtt_handle);
                
				if (xSemaphoreTake(
				        s_mqtt_mutex,
				        portMAX_DELAY
				    ) == pdTRUE) {
				
				    if (!s_mqtt_reconfiguring && mqtt_handle != NULL) {
				        err = mqtt5_manager_start(mqtt_handle);
				    } else {
				        err = ESP_ERR_INVALID_STATE;
				    }
				
				    xSemaphoreGive(s_mqtt_mutex);
				}                

                if (err != ESP_OK) {

                    ESP_LOGE(
                        TAG,
                        "Start new MQTT client failed: %s",
                        esp_err_to_name(err)
                    );

                    xEventGroupSetBits(
                        s_mqtt_event_group,
                        MQTT_RETRY_REQUEST_BIT
                    );
                }

            } else {

                ESP_LOGW(
                    TAG,
                    "Wi-Fi unavailable. MQTT start deferred"
                );
            }

            continue;
        }

        /* ==========================================
         * ƯU TIÊN 2: MQTT RECONNECT
         * ========================================== */


		bits = xEventGroupGetBits(s_mqtt_event_group);
		
		if ((bits & MQTT_RETRY_REQUEST_BIT) == 0) {
		    continue;
		}
		
		/*
		 * Nếu Wi-Fi chưa có IP:
		 * - Xóa retry request để tránh Supervisor chạy vòng lặp liên tục.
		 * - Chờ Wi-Fi có IP hoặc có yêu cầu reconfigure mới.
		 */
		if ((bits & MQTT_WIFI_CONNECTED_BIT) == 0) {
		
		    xEventGroupClearBits(
		        s_mqtt_event_group,
		        MQTT_RETRY_REQUEST_BIT
		    );
		
		    ESP_LOGW(
		        TAG,
		        "Wi-Fi unavailable. Waiting for Wi-Fi recovery."
		    );
		
		    EventBits_t wait_bits = xEventGroupWaitBits(
		        s_mqtt_event_group,
		        MQTT_WIFI_CONNECTED_BIT |
		        MQTT_RECONFIGURE_REQUEST_BIT,
		        pdFALSE,
		        pdFALSE,
		        portMAX_DELAY
		    );
		
		    /*
		     * Nếu Wi-Fi vừa có IP:
		     * yêu cầu Supervisor thực hiện kiểm tra MQTT lại.
		     */
		    if (wait_bits & MQTT_WIFI_CONNECTED_BIT) {
		
		        xEventGroupSetBits(
		            s_mqtt_event_group,
		            MQTT_RETRY_REQUEST_BIT
		        );
		    }
		
		    /*
		     * Nếu có yêu cầu reconfigure:
		     * quay lại đầu vòng lặp để xử lý reconfigure.
		     */
		    continue;
		}


        vTaskDelay(pdMS_TO_TICKS(3000));

        bits = xEventGroupGetBits(s_mqtt_event_group);

        if ((bits & MQTT_WIFI_CONNECTED_BIT) == 0 ||
            (bits & MQTT_RETRY_REQUEST_BIT) == 0) {
            continue;
        }

        if (xSemaphoreTake(
                s_mqtt_mutex,
                pdMS_TO_TICKS(1000)
            ) != pdTRUE) {
            continue;
        }

		esp_err_t err = ESP_ERR_INVALID_STATE;
		
		/*
		 * Kiểm tra lại Wi-Fi sau khi đã lấy mutex.
		 * Điều này tránh reconnect khi Wi-Fi vừa mất kết nối.
		 */
		EventBits_t current_bits = xEventGroupGetBits(
		    s_mqtt_event_group
		);
		if ((current_bits & MQTT_WIFI_CONNECTED_BIT) == 0) {
		
		    ESP_LOGW(
		        TAG,
		        "Wi-Fi disconnected before MQTT reconnect."
		    );
		
		    xEventGroupClearBits(
		        s_mqtt_event_group,
		        MQTT_RETRY_REQUEST_BIT
		    );
		
		} else if (!s_mqtt_reconfiguring && mqtt_handle != NULL) {

            if (mqtt5_manager_is_connected(mqtt_handle)) {

                xEventGroupClearBits(
                    s_mqtt_event_group,
                    MQTT_RETRY_REQUEST_BIT
                );

            } else {
		
		        /*
		         * Xóa retry request trước khi reconnect.
		         * Nếu reconnect thất bại sẽ yêu cầu retry lại.
		         */
		        xEventGroupClearBits(
		            s_mqtt_event_group,
		            MQTT_RETRY_REQUEST_BIT
		        );
		
		        err = mqtt5_manager_reconnect(mqtt_handle);
		    }
		}

        xSemaphoreGive(s_mqtt_mutex);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "MQTT reconnect failed: %s",
                esp_err_to_name(err)
            );

            vTaskDelay(pdMS_TO_TICKS(3000));

            xEventGroupSetBits(
                s_mqtt_event_group,
                MQTT_RETRY_REQUEST_BIT
            );
        }
    }
}
static esp_err_t mqtt_manager_create_from_config(void)
{
    if (!mqtt_config_is_ready()) {
        ESP_LOGW(TAG, "MQTT configuration is not ready");
        return ESP_ERR_INVALID_STATE;
    }

    const char *broker_uri = mqtt_config_get_broker_uri();
    const char *username = mqtt_config_get_username();
    const char *password = mqtt_config_get_password();
    const char *client_id = mqtt_config_get_client_id();

    if (broker_uri == NULL || broker_uri[0] == '\0' ||
        username == NULL || username[0] == '\0' ||
        password == NULL || password[0] == '\0' ||
        client_id == NULL || client_id[0] == '\0') {

        ESP_LOGE(TAG, "Invalid MQTT configuration");
        return ESP_ERR_INVALID_STATE;
    }

    /* Tạo topic theo username mới */
    int len;

    len = snprintf(
        MQTT_COMMAND_TOPIC,
        sizeof(MQTT_COMMAND_TOPIC),
        "%s/command",
        username
    );

    if (len < 0 || len >= sizeof(MQTT_COMMAND_TOPIC)) {
        return ESP_ERR_INVALID_SIZE;
    }

    len = snprintf(
        MQTT_EVENT_TOPIC,
        sizeof(MQTT_EVENT_TOPIC),
        "%s/event",
        username
    );

    if (len < 0 || len >= sizeof(MQTT_EVENT_TOPIC)) {
        return ESP_ERR_INVALID_SIZE;
    }

    len = snprintf(
        MQTT_STATUS_TOPIC,
        sizeof(MQTT_STATUS_TOPIC),
        "%s/status",
        username
    );

    if (len < 0 || len >= sizeof(MQTT_STATUS_TOPIC)) {
        return ESP_ERR_INVALID_SIZE;
    }

    mqtt5_manager_config_t config = {
        .broker_uri = broker_uri,
        .username = username,
        .password = password,
        .client_id = client_id,

        .keepalive = 60,

        /* Supervisor tự quản lý reconnect */
        .auto_reconnect = false,

        .will_topic = MQTT_STATUS_TOPIC,
        .will_message = "{\"status\":\"offline\"}",
        .will_qos = 1,
        .will_retain = true,

        .event_cb = mqtt_event_callback,
        .user_data = NULL
    };

    esp_err_t err = mqtt5_manager_init(
        &config,
        &mqtt_handle
    );

    if (err != ESP_OK) {
        mqtt_handle = NULL;

        ESP_LOGE(
            TAG,
            "MQTT manager init failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }

    ESP_LOGI(TAG, "MQTT manager initialized with current configuration");

    return ESP_OK;
}
/* =================================================
 * Initialize and start MQTT
 * ================================================= */
esp_err_t app_mqtt_start(void)
{
    /* Không khởi tạo task và EventGroup nhiều lần */
    if (s_mqtt_started) {
        ESP_LOGW(TAG, "MQTT application already started");
        return ESP_OK;
    }

    if (!mqtt_config_is_ready()) {
        ESP_LOGW(TAG, "MQTT configuration is not ready");
        return ESP_ERR_INVALID_STATE;
    }

    /* Tạo mutex quản lý MQTT client */
    if (s_mqtt_mutex == NULL) {

        s_mqtt_mutex = xSemaphoreCreateMutex();

        if (s_mqtt_mutex == NULL) {
            ESP_LOGE(TAG, "Cannot create MQTT mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Tạo EventGroup */
    if (s_mqtt_event_group == NULL) {

        s_mqtt_event_group = xEventGroupCreate();

        if (s_mqtt_event_group == NULL) {
            ESP_LOGE(TAG, "Cannot create MQTT EventGroup");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Khôi phục trạng thái Wi-Fi hiện tại */
    if (s_wifi_connected) {

        xEventGroupSetBits(
            s_mqtt_event_group,
            MQTT_WIFI_CONNECTED_BIT
        );
    }

    /* Khởi tạo MQTT manager từ cấu hình NVS */
    esp_err_t err = mqtt_manager_create_from_config();

    if (err != ESP_OK) {
        return err;
    }

    /* Tạo Queue lưu trạng thái GPIO cần publish */
    if (s_gpio_mqtt_event_queue == NULL)
    {
        s_gpio_mqtt_event_queue = xQueueCreate(
            GPIO_MQTT_EVENT_QUEUE_LENGTH,
            sizeof(gpio_mqtt_event_t)
        );

        if (s_gpio_mqtt_event_queue == NULL)
        {
            ESP_LOGE(TAG, "Cannot create GPIO MQTT event Queue");

            mqtt5_manager_deinit(mqtt_handle);
            mqtt_handle = NULL;

            return ESP_ERR_NO_MEM;
        }
    }
    
    /* Tạo Queue tiếp nhận Schedule request */
    if (s_mqtt_schedule_queue == NULL)
    {
        s_mqtt_schedule_queue = xQueueCreate(
            MQTT_SCHEDULE_QUEUE_LENGTH,
            sizeof(mqtt_schedule_request_t)
        );

        if (s_mqtt_schedule_queue == NULL)
        {
            ESP_LOGE(TAG, "Cannot create Schedule Queue");

            return ESP_ERR_NO_MEM;
        }
    }

    /* Tạo Schedule Worker Task một lần */
    if (s_mqtt_schedule_task_handle == NULL)
    {
        BaseType_t result = xTaskCreate(
            mqtt_schedule_worker_task,
            "mqtt_schedule",
            4096,
            NULL,
            7,
            &s_mqtt_schedule_task_handle
        );

        if (result != pdPASS)
        {
            ESP_LOGE(TAG, "Cannot create Schedule Worker");

            return ESP_FAIL;
        }
    }    
    /* Tạo MQTT TX task một lần */
    err = mqtt_tx_task_start();

    if (err != ESP_OK) {

        ESP_LOGE(TAG, "Cannot create MQTT TX task");

        if (s_gpio_mqtt_event_queue != NULL)
        {
            vQueueDelete(s_gpio_mqtt_event_queue);
            s_gpio_mqtt_event_queue = NULL;
        }
        
        mqtt5_manager_deinit(mqtt_handle);
        mqtt_handle = NULL;

        return err;
    }

    /* Tạo MQTT Supervisor một lần */
    BaseType_t result = xTaskCreate(
        mqtt_supervisor_task,
        "mqtt_supervisor",
        4096,
        NULL,
        6,
        NULL
    );

    if (result != pdPASS) {

        ESP_LOGE(TAG, "Cannot create MQTT Supervisor");

        mqtt5_manager_deinit(mqtt_handle);
        mqtt_handle = NULL;

        return ESP_FAIL;
    }

    /*
     * Đánh dấu application đã được khởi tạo.
     * Từ thời điểm này không tạo thêm TX task/Supervisor.
     */
    s_mqtt_started = true;


	/*
	 * Chỉ start MQTT nếu Wi-Fi đã có IP.
	 * Bảo vệ mqtt_handle bằng mutex.
	 */
	if (xSemaphoreTake(
	        s_mqtt_mutex,
	        pdMS_TO_TICKS(1000)
	    ) != pdTRUE) {
	
	    ESP_LOGE(TAG, "Cannot lock MQTT mutex for initial start");
	    return ESP_ERR_TIMEOUT;
	}
	
	EventBits_t bits = xEventGroupGetBits(s_mqtt_event_group);
	
	if ((bits & MQTT_WIFI_CONNECTED_BIT) &&
	    mqtt_handle != NULL) {
	
	    err = mqtt5_manager_start(mqtt_handle);
	
	} else {
	
	    err = ESP_ERR_INVALID_STATE;
	}
	
	xSemaphoreGive(s_mqtt_mutex);
	
	if (err == ESP_OK) {
	
	    ESP_LOGI(TAG, "Initial MQTT start requested");
	
	} else if ((bits & MQTT_WIFI_CONNECTED_BIT) == 0) {
	
	    ESP_LOGW(
	        TAG,
	        "Wi-Fi not connected. MQTT start deferred"
	    );
	
	} else {
	
	    ESP_LOGE(
	        TAG,
	        "Initial MQTT start failed: %s",
	        esp_err_to_name(err)
	    );
	
	    /*
	     * Để Supervisor thử reconnect khi Wi-Fi vẫn có IP.
	     */
	    xEventGroupSetBits(
	        s_mqtt_event_group,
	        MQTT_RETRY_REQUEST_BIT
	    );
	}

    return ESP_OK;
}

esp_err_t app_mqtt_request_reconfigure(void)
{
    /*
     * Nếu MQTT application chưa chạy,
     * khởi tạo lần đầu bằng cấu hình vừa lưu.
     */
    if (!s_mqtt_started) {

        /* UART configuration can be committed before BLE/Wi-Fi provisioning.
         * Do not create MQTT tasks/client until the station has an IP address.
         * app_main() starts MQTT after IP_EVENT_STA_GOT_IP. */
        if (!s_wifi_connected) {
            ESP_LOGI(
                TAG,
                "MQTT configuration saved before Wi-Fi; start deferred until IP is ready"
            );
            return ESP_OK;
        }

        ESP_LOGI(
            TAG,
            "MQTT not started and Wi-Fi is ready. Initializing with saved configuration"
        );

        return app_mqtt_start();
    }

    if (s_mqtt_event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(
        TAG,
        "Requesting MQTT client reconfiguration"
    );

    /*
     * Không restart tại đây.
     * Chỉ gửi yêu cầu để Supervisor xử lý.
     */
    xEventGroupSetBits(
        s_mqtt_event_group,
        MQTT_RECONFIGURE_REQUEST_BIT
    );

    return ESP_OK;
}
