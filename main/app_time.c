/*
 * app_time.c
 *
 *  Created on: 2026年11月12日
 *      Author: SKY-EN-PC01
 */


#include "app_time.h"

#include <time.h>
#include <stdlib.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_sntp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* EventGroup bits */
#define APP_TIME_WIFI_BIT       BIT0
#define APP_TIME_SYNCED_BIT     BIT1

/* Thời điểm hợp lệ tối thiểu: 2025-01-01 00:00:00 UTC */
#define APP_TIME_VALID_EPOCH    1735689600LL

static const char *TAG = "APP_TIME";

static EventGroupHandle_t s_time_event_group = NULL;

static bool s_sntp_initialized = false;

/* Tăng mỗi lần Wi-Fi nhận được IP */
static volatile uint32_t s_wifi_generation = 0;

static volatile uint32_t s_sync_generation = 0;
/**
 * @brief Callback được gọi khi SNTP đồng bộ thời gian.
 */
static void app_time_sync_callback(struct timeval *tv)
{
    (void)tv;

    if (s_time_event_group == NULL) {
        return;
    }

    EventBits_t bits = xEventGroupGetBits(s_time_event_group);

    /* Chỉ công nhận đồng bộ nếu Wi-Fi đang có kết nối */
    if ((bits & APP_TIME_WIFI_BIT) != 0) {

		xEventGroupSetBits(
		    s_time_event_group,
		    APP_TIME_SYNCED_BIT
		);
		
		/* Ghi nhận số lần SNTP đồng bộ thành công */
		s_sync_generation++;
		
		ESP_LOGI(TAG, "SNTP synchronized successfully. Generation=%lu", (unsigned long)s_sync_generation
		);

    } else {

        ESP_LOGW(
            TAG,
            "SNTP callback ignored: Wi-Fi is disconnected"
        );
    }
}


/**
 * @brief Khởi tạo bộ quản lý thời gian.
 */
esp_err_t app_time_init(void)
{
    if (s_time_event_group != NULL) {
        return ESP_OK;
    }

    /* Tạo EventGroup */
    s_time_event_group = xEventGroupCreate();

    if (s_time_event_group == NULL) {
        ESP_LOGE(TAG, "Cannot create time EventGroup");
        return ESP_ERR_NO_MEM;
    }

    /* Thiết lập múi giờ Việt Nam UTC+7 */
    if (setenv("TZ", "ICT-7", 1) != 0) {
        ESP_LOGE(TAG, "Cannot set timezone");
        return ESP_FAIL;
    }

    tzset();

    /* Cấu hình SNTP */
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);

    esp_sntp_setservername(
        0,
        "pool.ntp.org"
    );

    esp_sntp_set_time_sync_notification_cb(
        app_time_sync_callback
    );

    ESP_LOGI(
        TAG,
        "Time manager initialized; waiting for Wi-Fi IP"
    );

    return ESP_OK;
}


/**
 * @brief Cập nhật trạng thái Wi-Fi.
 */
void app_time_set_wifi_connected(bool connected)
{
    if (s_time_event_group == NULL) {
        return;
    }

	if (!connected) {
	
	    /*
	     * Chỉ xóa trạng thái Wi-Fi.
	     *
	     * Giữ APP_TIME_SYNCED_BIT vì System Clock
	     * vẫn tiếp tục chạy khi ESP32-C3 còn nguồn.
	     */
	    xEventGroupClearBits(
	        s_time_event_group,
	        APP_TIME_WIFI_BIT
	    );
	
	    ESP_LOGW(
	        TAG,
	        "Wi-Fi disconnected: continue using internal clock"
	    );
	
	    return;
	}

    /* Wi-Fi đã nhận IP */
    xEventGroupSetBits(
        s_time_event_group,
        APP_TIME_WIFI_BIT
    );

    s_wifi_generation++;

    if (!s_sntp_initialized) {

        /* Khởi động SNTP lần đầu */
        esp_sntp_init();

        s_sntp_initialized = true;

        ESP_LOGI(TAG, "SNTP started: pool.ntp.org");

    } else {

        /* Yêu cầu SNTP đồng bộ lại sau khi Wi-Fi reconnect */
        if (!esp_sntp_restart()) {
            ESP_LOGW(
                TAG,
                "SNTP restart requested but service is not initialized"
            );
        }

        ESP_LOGI(
            TAG,
            "SNTP restarted after Wi-Fi reconnect"
        );
    }
}


/**
 * @brief Kiểm tra thời gian có hợp lệ để chạy Schedule không.
 */
bool app_time_is_ready(void)
{
    if (s_time_event_group == NULL) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(s_time_event_group);

   /*
     * Chỉ cần thời gian đã được SNTP đồng bộ ít nhất một lần.
     * Không yêu cầu Wi-Fi phải đang kết nối.
     */
    if ((bits & APP_TIME_SYNCED_BIT) == 0) {
        return false;
    }

    time_t now = time(NULL);

    return (now >= (time_t)APP_TIME_VALID_EPOCH);
}

/**
 * @brief Yêu cầu SNTP đồng bộ thời gian lại.
 */
esp_err_t app_time_request_sync(void)
{
    if (s_time_event_group == NULL) {
        ESP_LOGE(TAG, "Time manager is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupGetBits(s_time_event_group);

    /* Không thể đồng bộ nếu Wi-Fi chưa có IP */
    if ((bits & APP_TIME_WIFI_BIT) == 0) {
        ESP_LOGW(TAG, "Cannot request SNTP: Wi-Fi has no IP");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Xóa trạng thái đồng bộ cũ.
     * Chỉ callback SNTP tiếp theo mới xác nhận đồng bộ lại.
     */
    xEventGroupClearBits(
        s_time_event_group,
        APP_TIME_SYNCED_BIT
    );

    if (!s_sntp_initialized) {

        esp_sntp_init();
        s_sntp_initialized = true;

        ESP_LOGI(TAG, "SNTP initialized for synchronization");

    } else {

        if (!esp_sntp_restart()) {
            ESP_LOGE(TAG, "SNTP restart failed");
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "SNTP synchronization requested");
    }

    return ESP_OK;
}
/**
 * @brief Chờ SNTP đồng bộ thời gian.
 */
bool app_time_wait_for_sync(uint32_t timeout_ms)
{
    if (s_time_event_group == NULL) {
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(
        s_time_event_group,
        APP_TIME_SYNCED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(timeout_ms)
    );

    if ((bits & APP_TIME_SYNCED_BIT) == 0) {
        ESP_LOGW(
            TAG,
            "SNTP synchronization timeout after %lu ms",
            (unsigned long)timeout_ms
        );

        return false;
    }

    /* Không chỉ kiểm tra bit, phải kiểm tra cả Unix timestamp */
    if (!app_time_is_ready()) {
        ESP_LOGE(TAG, "SNTP bit set but system time is invalid");
        return false;
    }

    ESP_LOGI(TAG, "System time is ready");

    return true;
}
/**
 * @brief Trả về số lần Wi-Fi nhận IP.
 */
uint32_t app_time_get_wifi_generation(void)
{
    return s_wifi_generation;
}


uint32_t app_time_get_sync_generation(void)
{
    return s_sync_generation;
}
