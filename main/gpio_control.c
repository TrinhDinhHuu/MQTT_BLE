/*
 * gpio_control.c
 *
 *  Created on: 2026年11月10日
 *      Author: SKY-EN-PC01
 */

#include "gpio_control.h"

#include "driver/gpio.h"
#include "esp_timer.h"    // Lấy thời gian monotonic microsecond

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_mqtt.h"
#include "app_time.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"
#include "app_time.h"
#include <time.h>
#include <stdio.h>

#include <stdbool.h>
#include <stdint.h>

#define GPIO_CONTROL_TASK_STACK_SIZE    4096
#define GPIO_CONTROL_TASK_PRIORITY      5

#define GPIO_SCAN_INTERVAL_MS           10
#define GPIO_DEBOUNCE_SAMPLES           5

#define GPIO_COMMAND_QUEUE_LENGTH       10
#define NVS_SCHEDULE_NAMESPACE          "relay_sched"

#define GPIO_BOOT_BUTTON       GPIO_NUM_9
#define GPIO_BOOT_HOLD_TIME_MS 3000

static gpio_boot_button_callback_t s_boot_button_callback = NULL;
/* Loại lệnh GPIO được gửi từ MQTT hoặc module khác */
typedef enum
{
    GPIO_CMD_SET = 0,       // ON/OFF thông thường
    GPIO_CMD_DURATION,     // ON/OFF có thời gian rồi đảo trạng thái
    GPIO_CMD_CYCLE,         // Chạy ON/OFF theo chu kỳ
    GPIO_CMD_SCHEDULE       // Cập nhật lịch ON/OFF hằng ngày
} gpio_command_type_t;


/* Dữ liệu lệnh truyền qua GPIO command Queue */
typedef struct
{
    gpio_command_type_t type;
    uint8_t channel;

    bool state;

    uint64_t duration_us;

    uint64_t on_duration_us;
    uint64_t off_duration_us;

    uint32_t cycles;

    bool schedule_enabled;
    gpio_schedule_mode_t schedule_mode;
    
    uint16_t schedule_on_minute;
    uint16_t schedule_off_minute;
    
    uint32_t schedule_on_date;
	uint32_t schedule_off_date;
	
} gpio_command_t;


static QueueHandle_t s_gpio_command_queue = NULL;

/*
 * Cấu hình mức điều khiển Relay.
 *
 * Relay active-high:
 *   ON  = 1
 *   OFF = 0
 *
 * Nếu sử dụng module Relay active-low:
 *   Đổi GPIO_RELAY_ON_LEVEL thành 0.
 */
#define GPIO_RELAY_ON_LEVEL             1
#define GPIO_RELAY_OFF_LEVEL            (!GPIO_RELAY_ON_LEVEL)

#define SWITCH_COUNT                    3

static const char *TAG = "GPIO_CONTROL";

/* GPIO Switch: nhấn nối chân GPIO xuống GND */
static const gpio_num_t s_switch_gpio[SWITCH_COUNT] =
{
    GPIO_NUM_3,
    GPIO_NUM_6,
    GPIO_NUM_7
};

/* GPIO Relay tương ứng với từng Switch */
static const gpio_num_t s_relay_gpio[SWITCH_COUNT] =
{
    GPIO_NUM_10,
    GPIO_NUM_18,
    GPIO_NUM_19
};

/* Trạng thái hiện tại của từng Relay */
static bool s_relay_state[SWITCH_COUNT] = { false };


/* Chế độ hoạt động timer của Relay */
typedef enum
{
    GPIO_TIMER_IDLE = 0,
    GPIO_TIMER_DURATION,
    GPIO_TIMER_CYCLE
} gpio_timer_mode_t;


/* Trạng thái timer độc lập cho từng Relay */
typedef struct
{
    gpio_timer_mode_t mode;

    bool phase_on;

    int64_t deadline_us;

    uint64_t on_duration_us;
    uint64_t off_duration_us;

    uint32_t cycle_count;
    uint32_t cycles_total;

} gpio_timer_state_t;


/* Mỗi Relay có một bộ quản lý timer riêng */
static gpio_timer_state_t s_gpio_timer[SWITCH_COUNT] = { 0 };

/* ---------------- Schedule / NVS ---------------- */
typedef struct {
    bool enabled;
    gpio_schedule_mode_t mode;
    uint16_t on_minute;
    uint16_t off_minute;
} relay_schedule_t;

typedef struct {
    uint32_t last_on_date;       /* DAILY: YYYYMMDD */
    uint32_t last_off_date;      /* DAILY: YYYYMMDD */
    uint32_t target_date;        /* ONCE: ngày bắt đầu cửa sổ lịch */
    uint32_t target_off_date;	/*ngày OFF.*/	
    bool once_on_done;           /* ONCE: mốc ON đã được ghi nhận */
    bool once_off_done;          /* ONCE: mốc OFF đã được ghi nhận */
} relay_schedule_state_t;

static relay_schedule_t s_relay_schedule[SWITCH_COUNT];
static relay_schedule_state_t s_relay_schedule_state[SWITCH_COUNT];
static bool s_schedule_ready_previous = false;
static uint32_t s_last_sync_generation = 0;
static bool s_manual_override[SWITCH_COUNT] = { false };
static bool s_schedule_reconcile_pending[SWITCH_COUNT] = { false };
static int64_t s_manual_override_minute[SWITCH_COUNT] = { 0 };
static int64_t s_last_schedule_check_minute = -1;

static void schedule_make_key(char *key, size_t size,
                              uint8_t channel, const char *suffix)
{
    (void)snprintf(key, size, "r%u_%s", (unsigned)channel, suffix);
}

/* Lưu cấu hình mới và reset tiến trình thực hiện của lịch đó. */
static esp_err_t schedule_nvs_save_config(uint8_t channel)
{
    nvs_handle_t handle;
    char key[16];
    esp_err_t err = nvs_open(NVS_SCHEDULE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    schedule_make_key(key, sizeof(key), channel, "en");
    err = nvs_set_u8(handle, key, s_relay_schedule[channel].enabled ? 1U : 0U);
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "mode");
        err = nvs_set_u8(handle, key, (uint8_t)s_relay_schedule[channel].mode);
    }
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "on");
        err = nvs_set_u16(handle, key, s_relay_schedule[channel].on_minute);
    }
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "off");
        err = nvs_set_u16(handle, key, s_relay_schedule[channel].off_minute);
    }
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "ond");
        err = nvs_set_u32(handle, key, 0U);
    }
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "offd");
        err = nvs_set_u32(handle, key, 0U);
    }
	if (err == ESP_OK) {
	    schedule_make_key(key, sizeof(key), channel, "tdate");
	
	    // Lưu ngày ON mục tiêu của lịch vào NVS.
	    err = nvs_set_u32(handle, key, s_relay_schedule_state[channel].target_date);
	}
	if (err == ESP_OK) {
	    schedule_make_key(key, sizeof(key), channel, "ofdate");	
	    err = nvs_set_u32(handle, key, s_relay_schedule_state[channel].target_off_date);
	}           
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "odone");
        err = nvs_set_u8(handle, key, 0U);
    }
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "fdone");
        err = nvs_set_u8(handle, key, 0U);
    }

    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

/* Lưu trạng thái ONCE/DAILY mà không ghi lại cấu hình giờ. */
static esp_err_t schedule_nvs_save_progress(uint8_t channel)
{
    nvs_handle_t handle;
    char key[16];
    esp_err_t err = nvs_open(NVS_SCHEDULE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    schedule_make_key(key, sizeof(key), channel, "ond");
    err = nvs_set_u32(handle, key, s_relay_schedule_state[channel].last_on_date);
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "offd");
        err = nvs_set_u32(handle, key, s_relay_schedule_state[channel].last_off_date);
    }
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "tdate");
        err = nvs_set_u32(handle, key, s_relay_schedule_state[channel].target_date);
    }
    
	if (err == ESP_OK) {
	    schedule_make_key(key, sizeof(key), channel, "ofdate");	
	    err = nvs_set_u32(handle, key, s_relay_schedule_state[channel].target_off_date);
	}   
	 
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "odone");
        err = nvs_set_u8(handle, key, s_relay_schedule_state[channel].once_on_done ? 1U : 0U);
    }
    if (err == ESP_OK) {
        schedule_make_key(key, sizeof(key), channel, "fdone");
        err = nvs_set_u8(handle, key, s_relay_schedule_state[channel].once_off_done ? 1U : 0U);
    }

    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

/* Chỉ cập nhật enabled, không reset các cờ ONCE đã lưu. */
static esp_err_t schedule_nvs_set_enabled(uint8_t channel, bool enabled)
{
    nvs_handle_t handle;
    char key[16];
    esp_err_t err = nvs_open(NVS_SCHEDULE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    schedule_make_key(key, sizeof(key), channel, "en");
    err = nvs_set_u8(handle, key, enabled ? 1U : 0U);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static void schedule_nvs_load(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_SCHEDULE_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved relay schedule; using disabled defaults");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open Schedule NVS: %s", esp_err_to_name(err));
        return;
    }

    for (uint8_t i = 0; i < SWITCH_COUNT; i++) {
        char key[16];
        uint8_t enabled = 0U, mode = (uint8_t)GPIO_SCHEDULE_DAILY;
        uint8_t on_done = 0U, off_done = 0U;
        uint16_t on_minute = 0U, off_minute = 0U;
        uint32_t last_on = 0U, last_off = 0U, target_date = 0U;
        uint32_t target_off_date = 0U;

        schedule_make_key(key, sizeof(key), i, "en"); (void)nvs_get_u8(handle, key, &enabled);
        schedule_make_key(key, sizeof(key), i, "mode"); (void)nvs_get_u8(handle, key, &mode);
        schedule_make_key(key, sizeof(key), i, "on"); (void)nvs_get_u16(handle, key, &on_minute);
        schedule_make_key(key, sizeof(key), i, "off"); (void)nvs_get_u16(handle, key, &off_minute);
        schedule_make_key(key, sizeof(key), i, "ond"); (void)nvs_get_u32(handle, key, &last_on);
        schedule_make_key(key, sizeof(key), i, "offd"); (void)nvs_get_u32(handle, key, &last_off);
        schedule_make_key(key, sizeof(key), i, "tdate"); (void)nvs_get_u32(handle, key, &target_date);
        schedule_make_key(key, sizeof(key), i, "ofdate"); (void)nvs_get_u32(handle, key, &target_off_date);
        schedule_make_key(key, sizeof(key), i, "odone"); (void)nvs_get_u8(handle, key, &on_done);
        schedule_make_key(key, sizeof(key), i, "fdone"); (void)nvs_get_u8(handle, key, &off_done);

        if ((mode <= (uint8_t)GPIO_SCHEDULE_DAILY) &&
            (on_minute < 1440U) && (off_minute < 1440U) &&
            (on_minute != off_minute)) {
            s_relay_schedule[i].enabled = (enabled != 0U);
            s_relay_schedule[i].mode = (gpio_schedule_mode_t)mode;
            s_relay_schedule[i].on_minute = on_minute;
            s_relay_schedule[i].off_minute = off_minute;
        } else {
            s_relay_schedule[i].enabled = false;
        }

        s_relay_schedule_state[i].last_on_date = last_on;
        s_relay_schedule_state[i].last_off_date = last_off;
        s_relay_schedule_state[i].target_date = target_date;
		s_relay_schedule_state[i].target_off_date = target_off_date;        
        s_relay_schedule_state[i].once_on_done = (on_done != 0U);
        s_relay_schedule_state[i].once_off_done = (off_done != 0U);
    }
    nvs_close(handle);
}

static uint32_t schedule_date_value(const struct tm *value)
{
    return (uint32_t)(value->tm_year + 1900) * 10000U +
           (uint32_t)(value->tm_mon + 1) * 100U + (uint32_t)value->tm_mday;
}

static uint32_t schedule_shift_date(const struct tm *value, int days)
{
    struct tm shifted = *value;
    shifted.tm_mday += days;
    shifted.tm_isdst = -1;
    (void)mktime(&shifted);
    return schedule_date_value(&shifted);
}
/*
static bool schedule_is_active(uint16_t minute, uint16_t on_minute, uint16_t off_minute)
{
    if (on_minute < off_minute) {
        return (minute >= on_minute) && (minute < off_minute);
    }
    return (minute >= on_minute) || (minute < off_minute);
} */

/* Chọn cửa sổ ONCE gần nhất chưa bắt đầu hoặc đang diễn ra. */
static uint32_t schedule_once_select_target_date(
    const struct tm *local_time,
    uint16_t current_minute,
    uint16_t on_minute,
    uint16_t off_minute)
{
    uint32_t today = schedule_date_value(local_time);

    if (on_minute < off_minute) {
        return (current_minute < off_minute) ? today : schedule_shift_date(local_time, 1);
    }

    /* Lịch qua đêm. */
    if (current_minute >= on_minute) return today;
    if (current_minute < off_minute) return schedule_shift_date(local_time, -1);
    return today;
}

/* Forward declarations: Schedule helper dùng các hàm được định nghĩa phía dưới. */
static void gpio_relay_set(uint8_t channel, bool state);
static void gpio_timer_cancel(uint8_t channel);

/* Chuyển YYYYMMDD thành epoch tại 00:00:00 local time. */
static bool schedule_date_to_epoch(uint32_t date, uint16_t minute, time_t *result)
{
    if (result == NULL || date == 0U || minute >= 1440U) return false;

    struct tm value = {0};
    value.tm_year = (int)(date / 10000U) - 1900;
    value.tm_mon = (int)((date / 100U) % 100U) - 1;
    value.tm_mday = (int)(date % 100U);
    value.tm_hour = (int)(minute / 60U);
    value.tm_min = (int)(minute % 60U);
    value.tm_isdst = -1;

    time_t converted = mktime(&value);
    if (converted == (time_t)-1) return false;
    *result = converted;
    return true;
}

/*
 * Xác định trạng thái DAILY hiện tại và event gần nhất cần ghi nhận.
 * Event date là ngày thực tế mà mốc ON hoặc OFF thuộc về; với lịch qua
 * đêm, ON lúc 22:00 và OFF lúc 06:00 thuộc hai ngày khác nhau.
 */
static bool schedule_daily_get_event(
    const relay_schedule_t *cfg,
    const relay_schedule_state_t *state,
    uint16_t current_minute,
    uint32_t today,
    uint32_t yesterday,
    bool *should_on,
    bool *on_event,
    uint32_t *event_date)
{
    if (cfg == NULL || state == NULL || should_on == NULL ||
        on_event == NULL || event_date == NULL) return false;

    if (cfg->on_minute < cfg->off_minute) {
        /* Lịch trong cùng ngày: ON -> OFF. */
        if (current_minute >= cfg->off_minute) {
            *should_on = false;
            *on_event = false;
            *event_date = today;
        } else if (current_minute >= cfg->on_minute) {
            *should_on = true;
            *on_event = true;
            *event_date = today;
        } else {
            *should_on = false;
            *on_event = false;
            *event_date = yesterday;
        }
    } else {
        /* Lịch qua đêm: ON hôm nay, OFF ngày kế tiếp. */
        if (current_minute >= cfg->on_minute) {
            *should_on = true;
            *on_event = true;
            *event_date = today;
        } else if (current_minute < cfg->off_minute) {
            *should_on = true;
            *on_event = true;
            *event_date = yesterday;
        } else {
            *should_on = false;
            *on_event = false;
            *event_date = today;
        }
    }

    uint32_t last_event_date = *on_event ? state->last_on_date : state->last_off_date;
    return (last_event_date != *event_date);
}

static void gpio_process_schedules(void)
{
    /* Chưa SNTP sync hoặc thời gian trước mốc 12/11/2026: không chạy Schedule. */
    if (!app_time_is_ready()) {
        s_schedule_ready_previous = false;
        return;
    }

    time_t now = time(NULL);
    struct tm local_time;
    if (localtime_r(&now, &local_time) == NULL) return;

    uint32_t sync_generation = app_time_get_sync_generation();
    bool sync_changed = (sync_generation != s_last_sync_generation);
    s_last_sync_generation = sync_generation;

    bool force_reconcile = (!s_schedule_ready_previous || sync_changed);
    s_schedule_ready_previous = true;

    int64_t epoch_minute = (int64_t)(now / 60);
    if (!force_reconcile && epoch_minute == s_last_schedule_check_minute) return;
    s_last_schedule_check_minute = epoch_minute;

    uint16_t current_minute = (uint16_t)(local_time.tm_hour * 60 + local_time.tm_min);
    uint32_t today = schedule_date_value(&local_time);
    uint32_t yesterday = schedule_shift_date(&local_time, -1);

    for (uint8_t i = 0; i < SWITCH_COUNT; i++) {
        relay_schedule_t *cfg = &s_relay_schedule[i];
        relay_schedule_state_t *state = &s_relay_schedule_state[i];
        if (!cfg->enabled) continue;

        bool pending_reconcile = s_schedule_reconcile_pending[i];
        bool reconcile = force_reconcile || pending_reconcile;
        bool should_on = false;
        bool event_due = false;
        bool on_event = false;
        uint32_t event_date = 0U;
        bool once_finished = false;

        if (cfg->mode == GPIO_SCHEDULE_ONCE) {
            /* Chọn ngày ON gần nhất khi lần đầu có thời gian hợp lệ. */
            if (state->target_date == 0U) {
                state->target_date = schedule_once_select_target_date(
                    &local_time,
                    current_minute,
                    cfg->on_minute,
                    cfg->off_minute);

                if (schedule_nvs_save_progress(i) != ESP_OK) {
                    state->target_date = 0U;
                    ESP_LOGE(TAG,
                             "Relay %u cannot save ONCE target date",
                             (unsigned)(i + 1));
                    continue;
                }
            }
			/*
            uint32_t off_date = state->target_date;
            if (cfg->off_minute <= cfg->on_minute) {
                struct tm target_tm = {0};
                target_tm.tm_year = (int)(state->target_date / 10000U) - 1900;
                target_tm.tm_mon = (int)((state->target_date / 100U) % 100U) - 1;
                target_tm.tm_mday = (int)(state->target_date % 100U);
                target_tm.tm_isdst = -1;
                (void)mktime(&target_tm);
                off_date = schedule_shift_date(&target_tm, 1);
            } */
            
			uint32_t off_date = state->target_off_date;
			
			if (off_date == 0U) {
			    /* Tương thích lịch ONCE cũ */
			    off_date = state->target_date;
			
			    if (cfg->off_minute <= cfg->on_minute) {
			        struct tm target_tm = {0};
			
			        target_tm.tm_year =
			            (int)(state->target_date / 10000U) - 1900;
			
			        target_tm.tm_mon =
			            (int)((state->target_date / 100U) % 100U) - 1;
			
			        target_tm.tm_mday =
			            (int)(state->target_date % 100U);
			
			        target_tm.tm_isdst = -1;
			
			        if (mktime(&target_tm) == (time_t)-1) {
			            continue;
			        }
			
			        off_date = schedule_shift_date(&target_tm, 1);
			    }
			}
			
            time_t on_epoch, off_epoch;
            if (!schedule_date_to_epoch(state->target_date, cfg->on_minute, &on_epoch) ||
                !schedule_date_to_epoch(off_date, cfg->off_minute, &off_epoch)) {
                ESP_LOGE(TAG, "Relay %u has invalid ONCE date/time", (unsigned)(i + 1));
                continue;
            }

            if (now >= off_epoch) {
                /* Đã qua cả cửa sổ: không phát xung ON muộn sau reboot. */
                should_on = false;
                once_finished = true;
                event_due = true; /* retry disabling if an earlier NVS write failed */
            } else if (now >= on_epoch) {
                /* Đang trong cửa sổ ON/OFF. */
                should_on = true;
                event_due = !state->once_on_done;
            } else {
                /* Chưa đến giờ ON. */
                should_on = false;
                event_due = false;
            }
        } else {
            event_due = schedule_daily_get_event(
                cfg, state, current_minute, today, yesterday,
                &should_on, &on_event, &event_date);
        }

        /*
         * Manual override: giữ trạng thái do nút nhấn đặt đến event kế tiếp.
         * Reconcile do SNTP/boot có quyền đồng bộ lại ngay.
         */
        if (s_manual_override[i] && !reconcile && !event_due) {
            continue;
        }
        if (s_manual_override[i] && (reconcile || event_due)) {
            s_manual_override[i] = false;
            ESP_LOGI(TAG, "Relay %u manual override ended", (unsigned)(i + 1));
        }

        if (!reconcile && !event_due) continue;

        /*
         * Không cắt Duration/Cycle chỉ vì MQTT vừa cập nhật cấu hình hoặc
         * SNTP vừa sync. Giữ yêu cầu reconcile pending đến khi timer kết thúc.
         * Tại event ON/OFF thực sự, Schedule có quyền ưu tiên và hủy timer.
         */
        if (s_gpio_timer[i].mode != GPIO_TIMER_IDLE && reconcile) {
            s_schedule_reconcile_pending[i] = true;
            continue;
        }

        gpio_timer_cancel(i);
        gpio_relay_set(i, should_on);
        s_schedule_reconcile_pending[i] = false;

        if (cfg->mode == GPIO_SCHEDULE_ONCE) {
            if (should_on && !state->once_on_done) {
                state->once_on_done = true;
                if (schedule_nvs_save_progress(i) != ESP_OK) {
                    state->once_on_done = false;
                    ESP_LOGE(TAG, "Relay %u cannot save ONCE ON marker", (unsigned)(i + 1));
                } else {
                    ESP_LOGI(TAG, "Relay %u ONCE ON completed", (unsigned)(i + 1));
                }
            }

            if (once_finished) {
                bool old_on_done = state->once_on_done;
                bool old_off_done = state->once_off_done;
                state->once_on_done = true;
                state->once_off_done = true;

                esp_err_t save_err = schedule_nvs_save_progress(i);
                if (save_err != ESP_OK) {
                    state->once_on_done = old_on_done;
                    state->once_off_done = old_off_done;
                    ESP_LOGE(TAG, "Relay %u cannot persist ONCE completion: %s",
                             (unsigned)(i + 1), esp_err_to_name(save_err));
                    continue;
                }

                save_err = schedule_nvs_set_enabled(i, false);
                if (save_err == ESP_OK) {
                    cfg->enabled = false;
                    ESP_LOGI(TAG, "Relay %u ONCE schedule completed and disabled",
                             (unsigned)(i + 1));
                } else {
                    /* Giữ cờ DONE trong RAM/NVS; lần kiểm tra sau sẽ retry disable. */
                    ESP_LOGE(TAG, "Relay %u cannot persist ONCE disabled state: %s",
                             (unsigned)(i + 1), esp_err_to_name(save_err));
                }
            }
        } else {
            uint32_t *stored_date = on_event ?
                &state->last_on_date : &state->last_off_date;

            if (*stored_date != event_date) {
                uint32_t old_date = *stored_date;
                *stored_date = event_date;
                if (schedule_nvs_save_progress(i) != ESP_OK) {
                    *stored_date = old_date;
                    ESP_LOGE(TAG, "Relay %u cannot save DAILY event marker",
                             (unsigned)(i + 1));
                }
            }
        }
    }
}

/* Chống khởi tạo Task nhiều lần */
static TaskHandle_t s_gpio_task_handle = NULL;

/**
 * @brief Điều khiển trạng thái Relay.
 *
 * Sau khi GPIO thay đổi thành công, gửi trạng thái mới
 * vào MQTT TX Queue.
 *
 * @param channel Kênh Relay: 0, 1, 2.
 * @param state   true = ON, false = OFF.
 */
static void gpio_relay_set(uint8_t channel, bool state)
{
    if (channel >= SWITCH_COUNT)
    {
        return;
    }

    /* Kiểm tra trạng thái trước khi thay đổi */
    bool state_changed = (s_relay_state[channel] != state);
    
    uint32_t level = state
                    ? GPIO_RELAY_ON_LEVEL
                    : GPIO_RELAY_OFF_LEVEL;

    esp_err_t err = gpio_set_level(s_relay_gpio[channel], level);

    if (err == ESP_OK)
    {
 		/* Cập nhật trạng thái nội bộ */		
        s_relay_state[channel] = state;

        ESP_LOGI(TAG,
                 "Relay %u: %s",
                 (unsigned int)(channel + 1),
                 state ? "ON" : "OFF");
        /*
         * Chỉ gửi MQTT event khi trạng thái thực sự thay đổi.
         * Không gửi event khi nhận lệnh đặt lại trạng thái cũ.
         */
        if (state_changed)
        {
            esp_err_t mqtt_err = app_mqtt_publish_gpio_state(
                channel,
                state
            );

            if (mqtt_err != ESP_OK)
            {
                ESP_LOGW(
                    TAG,
                    "Cannot queue GPIO state event: %s",
                    esp_err_to_name(mqtt_err)
                );
            }
        }                 
    }
    else
    {
        ESP_LOGE(TAG,
                 "Cannot set Relay %u: %s",
                 (unsigned int)(channel + 1),
                 esp_err_to_name(err));
    }
}

/**
 * @brief Gửi MQTT Event chứa thông tin Cycle.
 */
static void gpio_cycle_publish_event(
    uint8_t channel,
    bool state,
    uint32_t cycle_count,
    uint32_t cycles_total,
    gpio_cycle_status_t cycle_status)
{
    esp_err_t err = app_mqtt_publish_gpio_cycle_event(
        channel,
        state,
        cycle_count,
        cycles_total,
        cycle_status
    );

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "Cannot queue GPIO cycle event: %s",
                 esp_err_to_name(err));
    }
}


/**
 * @brief Thay đổi Relay và gửi Cycle Event.
 *
 * Chỉ dùng cho các lần chuyển trạng thái trong Cycle.
 */
static void gpio_relay_set_cycle(uint8_t channel, bool state)
{
    if (channel >= SWITCH_COUNT)
    {
        return;
    }

    uint32_t level = state
                   ? GPIO_RELAY_ON_LEVEL
                   : GPIO_RELAY_OFF_LEVEL;

    esp_err_t err = gpio_set_level(s_relay_gpio[channel], level);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Cannot set Relay %u: %s",
                 (unsigned int)(channel + 1),
                 esp_err_to_name(err));
        return;
    }

    s_relay_state[channel] = state;

    ESP_LOGI(TAG,
             "Relay %u: %s",
             (unsigned int)(channel + 1),
             state ? "ON" : "OFF");

    gpio_cycle_publish_event(
        channel,
        state,
        s_gpio_timer[channel].cycle_count,
        s_gpio_timer[channel].cycles_total,
        GPIO_CYCLE_STATUS_RUNNING
    );
}


/**
 * @brief Đảo trạng thái Relay.
 */
static void gpio_relay_toggle(uint8_t channel)
{
    if (channel >= SWITCH_COUNT)
    {
        return;
    }

    gpio_relay_set(channel, !s_relay_state[channel]);
}

/**
 * @brief Hủy chế độ duration/cycle của một Relay.
 *
 * Chỉ GPIO Control Task gọi hàm này.
 */
static void gpio_timer_cancel(uint8_t channel)
{
    if (channel >= SWITCH_COUNT)
    {
        return;
    }

    gpio_timer_state_t *timer = &s_gpio_timer[channel];

    /*
     * Nếu đang chạy Cycle thì gửi sự kiện CANCELLED
     * trước khi xóa thông tin bộ đếm.
     */
    if (timer->mode == GPIO_TIMER_CYCLE)
    {
        gpio_cycle_publish_event(
            channel,
            s_relay_state[channel],
            timer->cycle_count,
            timer->cycles_total,
            GPIO_CYCLE_STATUS_CANCELLED
        );

        ESP_LOGI(TAG,
                 "Relay %u cycle cancelled: %lu/%lu",
                 (unsigned int)(channel + 1),
                 (unsigned long)timer->cycle_count,
                 (unsigned long)timer->cycles_total);
    }

    /* Đưa timer về trạng thái không hoạt động */
    timer->mode = GPIO_TIMER_IDLE;

    timer->phase_on = false;
    timer->deadline_us = 0;

    timer->on_duration_us = 0;
    timer->off_duration_us = 0;

    timer->cycle_count = 0;
    timer->cycles_total = 0;
}

/**
 * @brief Kiểm tra và xử lý deadline của tất cả Relay.
 *
 * Hàm được gọi định kỳ bởi GPIO Control Task.
 * Không tạo thêm Task hoặc Software Timer.
 */
static void gpio_process_timers(void)
{
    int64_t now_us = esp_timer_get_time();

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        gpio_timer_state_t *timer = &s_gpio_timer[i];

        /* Relay này không sử dụng timer */
        if (timer->mode == GPIO_TIMER_IDLE)
        {
            continue;
        }

        /* Chưa đến deadline */
        if (now_us < timer->deadline_us)
        {
            continue;
        }

        /* -----------------------------------------
         * Xử lý DURATION
         * ----------------------------------------- */
        if (timer->mode == GPIO_TIMER_DURATION)
        {
            /*
             * Hết thời gian duy trì:
             * Đảo trạng thái Relay và kết thúc duration.
             */
            gpio_relay_set(i, !s_relay_state[i]);

            timer->mode = GPIO_TIMER_IDLE;
            timer->deadline_us = 0;

            ESP_LOGI(TAG,
                     "Relay %u duration completed",
                     (unsigned int)(i + 1));

            continue;
        }

        /* -----------------------------------------
         * Xử lý CYCLE
         * ----------------------------------------- */

		if (timer->mode == GPIO_TIMER_CYCLE)
		{
		    if (timer->phase_on)
		    {
		        /*
		         * Kết thúc pha ON:
		         * Chuyển Relay sang OFF.
		         * cycle_count chưa tăng vì pha OFF chưa hoàn thành.
		         */
		        gpio_relay_set_cycle(i, false);
		
		        timer->phase_on = false;
		
		        /*
		         * Deadline tiếp theo = deadline cũ + thời gian OFF.
		         * Giúp hạn chế sai số tích lũy.
		         */
		        timer->deadline_us +=
		            (int64_t)timer->off_duration_us;
		    }
		    else
		    {
		        /*
		         * Kết thúc pha OFF:
		         * Một chu kỳ ON/OFF đã hoàn thành.
		         */
		        if (timer->cycle_count < UINT32_MAX)
		        {
		            timer->cycle_count++;
		        }
		
		        ESP_LOGI(TAG,
		                 "Relay %u cycle completed: %lu/%lu",
		                 (unsigned int)(i + 1),
		                 (unsigned long)timer->cycle_count,
		                 (unsigned long)timer->cycles_total);
		
		        /*
		         * Kiểm tra đã hoàn thành đủ số chu kỳ chưa.
		         * cycles_total = 0 nghĩa là chạy vô hạn.
		         */
		        if ((timer->cycles_total > 0) &&
		            (timer->cycle_count >= timer->cycles_total))
		        {
		            timer->mode = GPIO_TIMER_IDLE;
		            timer->deadline_us = 0;
		
		            /*
		             * Gửi sự kiện COMPLETED.
		             * Relay vẫn OFF, nhưng cycle_count đã tăng.
		             */
		            gpio_cycle_publish_event(
		                i,
		                s_relay_state[i],
		                timer->cycle_count,
		                timer->cycles_total,
		                GPIO_CYCLE_STATUS_COMPLETED
		            );
		
		            ESP_LOGI(TAG,
		                     "Relay %u cycle sequence completed",
		                     (unsigned int)(i + 1));
		
		            continue;
		        }
		
		        /*
		         * Chưa hoàn thành hoặc chạy vô hạn:
		         * Bắt đầu pha ON tiếp theo.
		         */
		        timer->phase_on = true;
		
		        gpio_relay_set_cycle(i, true);
		
		        timer->deadline_us +=
		            (int64_t)timer->on_duration_us;
		    }
		}

    }
}
esp_err_t gpio_control_register_boot_button_callback(
    gpio_boot_button_callback_t callback)
{
    if (s_gpio_task_handle != NULL) {
        ESP_LOGE(TAG,
                 "Cannot register BOOT callback after GPIO task started");
        return ESP_ERR_INVALID_STATE;
    }

    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_boot_button_callback = callback;

    return ESP_OK;
}
/**
 * @brief Task đọc Switch và điều khiển Relay.
 *
 * Mỗi Switch tương ứng với một Relay.
 * Switch nhấn LOW, nhả HIGH.
 */

static void gpio_control_task(void *pvParameters)
{
    (void)pvParameters;
	/* Trạng thái nút BOOT GPIO9 */
	int boot_stable_level = gpio_get_level(GPIO_BOOT_BUTTON);
	int boot_candidate_level = boot_stable_level;
	
	uint8_t boot_debounce_count = 0;
	
	int64_t boot_press_start_us = 0;
	
	bool boot_long_press_handled = false;
	
	/* Nếu thiết bị khởi động khi nút đang được giữ */
	if (boot_stable_level == 0) {
	    boot_press_start_us = esp_timer_get_time();
	}
    /*
     * stable_level:
     * Trạng thái Switch đã được xác nhận ổn định.
     *
     * candidate_level:
     * Trạng thái Switch đang được kiểm tra debounce.
     *
     * debounce_count:
     * Số lần đọc liên tiếp candidate_level không thay đổi.
     */
    int stable_level[SWITCH_COUNT];
    int candidate_level[SWITCH_COUNT];

    uint8_t debounce_count[SWITCH_COUNT] = { 0 };

    /*
     * Đọc trạng thái thực tế khi Task bắt đầu.
     * Không tự kích hoạt Relay nếu Switch đang được giữ.
     */
    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        stable_level[i] = gpio_get_level(s_switch_gpio[i]);
        candidate_level[i] = stable_level[i];
    }

    ESP_LOGI(TAG, "GPIO Control Task started");
    TickType_t last_schedule_tick = xTaskGetTickCount();

    while (1)
    {

		gpio_command_t command;

        /*
         * Xử lý tất cả lệnh đang chờ trong Queue.
         * Không chờ Queue để Switch vẫn được scan định kỳ.
         */
		// Mỗi vòng lặp chỉ xử lý tối đa 5 command.
		// Tránh Queue liên tục có dữ liệu làm trì hoãn việc quét GPIO.
		for (uint8_t cmd_count = 0; cmd_count < 5; cmd_count++) {
		    
		    // Lấy command không chờ.
		    if (xQueueReceive(s_gpio_command_queue, &command, 0) != pdPASS) {
		        break;  // Queue rỗng thì chuyển sang xử lý GPIO và Timer.
		    }
		   
		    if (command.channel >= SWITCH_COUNT)
		    {
		        continue;
		    }
		
		    uint8_t channel = command.channel;		
		
		    switch (command.type)
		    {
		        case GPIO_CMD_SET:
		        {
					gpio_timer_cancel(channel);
		            /* Điều khiển ON/OFF thông thường */
		            gpio_relay_set(channel, command.state);
		            break;
		        }
		
		        case GPIO_CMD_DURATION:
		        {
					gpio_timer_cancel(channel);
		            /*
		             * Đặt trạng thái ban đầu ngay lập tức,
		             * sau đó bắt đầu đếm thời gian.
		             */
		            gpio_relay_set(channel, command.state);
		
		            gpio_timer_state_t *timer = &s_gpio_timer[channel];
		
		            timer->mode = GPIO_TIMER_DURATION;
		
		            timer->deadline_us =
		                esp_timer_get_time() +
		                (int64_t)command.duration_us;
		
		            break;
		        }
		
		        case GPIO_CMD_CYCLE:
		        {
					gpio_timer_cancel(channel);
					
		            gpio_timer_state_t *timer = &s_gpio_timer[channel];
		
		            timer->mode = GPIO_TIMER_CYCLE;
		
		            timer->phase_on = true;
		
		            timer->on_duration_us = command.on_duration_us;
		            timer->off_duration_us = command.off_duration_us;
		
		            timer->cycle_count = 0;
		            timer->cycles_total = command.cycles;
		
					gpio_relay_set_cycle(channel, true);		           
		
		            timer->deadline_us =
		                esp_timer_get_time() +
		                (int64_t)timer->on_duration_us;
		
		            ESP_LOGI(TAG,
		                     "Relay %u cycle started: ON=%llu us, OFF=%llu us, cycles=%lu",
		                     (unsigned int)(channel + 1),
		                     (unsigned long long)timer->on_duration_us,
		                     (unsigned long long)timer->off_duration_us,
		                     (unsigned long)timer->cycles_total);
		
		            break;
		        }
		
                case GPIO_CMD_SCHEDULE:
                {
                    relay_schedule_t old_schedule = s_relay_schedule[channel];
                    relay_schedule_state_t old_state = s_relay_schedule_state[channel];

                    s_relay_schedule[channel].enabled = command.schedule_enabled;
                    s_relay_schedule[channel].mode = command.schedule_mode;
                    s_relay_schedule[channel].on_minute = command.schedule_on_minute;
                    s_relay_schedule[channel].off_minute = command.schedule_off_minute;
                    s_relay_schedule_state[channel] = (relay_schedule_state_t){0};
					s_relay_schedule_state[channel].target_date = command.schedule_on_date;
					s_relay_schedule_state[channel].target_off_date = command.schedule_off_date;
					
                    esp_err_t nvs_err = schedule_nvs_save_config(channel);
                    if (nvs_err == ESP_OK) {
                        s_manual_override[channel] = false;
                        s_schedule_reconcile_pending[channel] = true;
                        s_last_schedule_check_minute = -1;
                        ESP_LOGI(TAG,
                                 "Relay %u Schedule saved: enabled=%u mode=%s ON=%02u:%02u OFF=%02u:%02u",
                                 (unsigned)(channel + 1),
                                 command.schedule_enabled ? 1U : 0U,
                                 command.schedule_mode == GPIO_SCHEDULE_ONCE ? "ONCE" : "DAILY",
                                 command.schedule_on_minute / 60U,
                                 command.schedule_on_minute % 60U,
                                 command.schedule_off_minute / 60U,
                                 command.schedule_off_minute % 60U);
                                 
						/* Lấy thời gian thực tế sau khi ghi NVS thành công */
						time_t saved_at = time(NULL);
						
						if (saved_at != (time_t)-1)
						{
						    esp_err_t mqtt_err = app_mqtt_publish_schedule_saved(
						        channel,
						        (uint8_t)command.schedule_mode,
						        command.schedule_on_minute,
						        command.schedule_off_minute,
						        command.schedule_on_date,
						        command.schedule_off_date,
						        (int64_t)saved_at
						    );
						
						    if (mqtt_err == ESP_OK)
						    {
						        ESP_LOGI(
						            TAG,
						            "Schedule ACK queued: Relay=%u, saved_at=%lld",
						            (unsigned int)(channel + 1),
						            (long long)saved_at
						        );
						    }
						    else
						    {
						        ESP_LOGW(
						            TAG,
						            "Cannot queue Schedule ACK: %s",
						            esp_err_to_name(mqtt_err)
						        );
						    }
						}
						else
						{
						    ESP_LOGW(TAG, "Cannot get current time for Schedule ACK");
						}                                 
                    } else {
                        s_relay_schedule[channel] = old_schedule;
                        s_relay_schedule_state[channel] = old_state;
                        ESP_LOGE(TAG, "Cannot save Relay %u Schedule: %s",
                                 (unsigned)(channel + 1), esp_err_to_name(nvs_err));
                    }
                    break;
                }

default:
		        {
		            ESP_LOGW(TAG, "Unknown GPIO command type");
		            break;
		        }
		    }
		}

                 
 		/* Scan Switch và debounce */                 
        for (uint8_t i = 0; i < SWITCH_COUNT; i++)
        {
            int current_level = gpio_get_level(s_switch_gpio[i]);

            /*
             * Nếu mức đọc thay đổi so với candidate:
             * Bắt đầu kiểm tra một trạng thái mới.
             */
            if (current_level != candidate_level[i])
            {
                candidate_level[i] = current_level;
                debounce_count[i] = 1;
            }
            else
            {
                /*
                 * Mức đọc giống candidate:
                 * Tăng số lần đọc liên tiếp.
                 */
                if (debounce_count[i] < GPIO_DEBOUNCE_SAMPLES)
                {
                    debounce_count[i]++;
                }
            }

            /*
             * Đủ số lần đọc liên tiếp và trạng thái candidate
             * khác trạng thái ổn định trước đó:
             * Xác nhận Switch đã thay đổi trạng thái.
             */
            if ((debounce_count[i] >= GPIO_DEBOUNCE_SAMPLES) &&
                (candidate_level[i] != stable_level[i]))
            {
                stable_level[i] = candidate_level[i];

                /*
                 * Chỉ xử lý sự kiện nhấn:
                 * HIGH -> LOW.
                 */
                if (stable_level[i] == 0)
                {
                    ESP_LOGI(TAG,
                             "Switch %u pressed",
                             (unsigned int)(i + 1));

					/*
					 * Switch được nhấn:
					 * Hủy duration/cycle đang chạy rồi đảo trạng thái Relay.
					 */
                    gpio_timer_cancel(i);
                    gpio_relay_toggle(i);
                    if (s_relay_schedule[i].enabled && app_time_is_ready()) {
                        s_manual_override[i] = true;
                        s_manual_override_minute[i] = (int64_t)(time(NULL) / 60);
                    }
                }
            }
        }
        
		/* =========================================
		 * Scan BOOT button GPIO9
		 * ========================================= */
		int boot_current_level = gpio_get_level(GPIO_BOOT_BUTTON);
		
		/* Debounce */
		if (boot_current_level != boot_candidate_level) {
		
		    boot_candidate_level = boot_current_level;
		    boot_debounce_count = 1;
		
		} else {
		
		    if (boot_debounce_count < GPIO_DEBOUNCE_SAMPLES) {
		        boot_debounce_count++;
		    }
		}
		
		/* Xác nhận trạng thái ổn định */
		if ((boot_debounce_count >= GPIO_DEBOUNCE_SAMPLES) &&
		    (boot_candidate_level != boot_stable_level)) {
		
		    boot_stable_level = boot_candidate_level;
		
		    if (boot_stable_level == 0) {
		
		        /* Bắt đầu tính thời gian nhấn */
		        boot_press_start_us = esp_timer_get_time();
		
		        boot_long_press_handled = false;
		
		        ESP_LOGI(TAG, "BOOT button pressed");
		
		    } else {
		
		        /* Nút được thả */
		        boot_press_start_us = 0;
		
		        boot_long_press_handled = false;
		
		        ESP_LOGI(TAG, "BOOT button released");
		    }
		}
		
		/* Kiểm tra giữ nút đủ 3 giây */
		if ((boot_stable_level == 0) &&
		    (boot_press_start_us > 0) &&
		    !boot_long_press_handled) {
		
		    int64_t held_time_us =
		        esp_timer_get_time() - boot_press_start_us;
		
		    if (held_time_us >=
		        ((int64_t)GPIO_BOOT_HOLD_TIME_MS * 1000)) {
		
		        boot_long_press_handled = true;
		
		        ESP_LOGW(TAG, "BOOT button held for 3 seconds");
		
		        if (s_boot_button_callback != NULL) {
		
		            s_boot_button_callback();
		
		        } else {
		
		            ESP_LOGE(TAG,
		                     "BOOT button callback is not registered");
		        }
		    }
		}
		        
        /* Giữ nguyên bộ xử lý Duration/Cycle hiện tại. */
        gpio_process_timers();

        /* Schedule chỉ cần kiểm tra mỗi giây; không ảnh hưởng debounce 10 ms. */
        TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_schedule_tick) >= pdMS_TO_TICKS(1000)) {
            last_schedule_tick = now_tick;
            gpio_process_schedules();
        }
        vTaskDelay(pdMS_TO_TICKS(GPIO_SCAN_INTERVAL_MS));
    }
}

/**
 * @brief Khởi tạo GPIO Switch và Relay.
 */
esp_err_t gpio_control_init(void)
{
    esp_err_t err;

    if (s_gpio_task_handle != NULL)
    {
        ESP_LOGW(TAG, "GPIO Control already initialized");
        return ESP_OK;
    }

    /* NVS phải được khởi tạo trong app_main trước gpio_control_init(). */
    schedule_nvs_load();

    /* -----------------------------------------------
     * 1. Khởi tạo GPIO Relay output
     * ----------------------------------------------- */

    uint64_t relay_pin_mask = 0;

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        relay_pin_mask |= (1ULL << s_relay_gpio[i]);
    }

    gpio_config_t relay_config =
    {
        .pin_bit_mask = relay_pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    err = gpio_config(&relay_config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Relay GPIO configuration failed");
        return err;
    }

    /* Đặt tất cả Relay về OFF */
    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        err = gpio_set_level(
            s_relay_gpio[i],
            GPIO_RELAY_OFF_LEVEL
        );

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Cannot initialize Relay %u",
                     (unsigned int)(i + 1));
            return err;
        }

        s_relay_state[i] = false;
    }

    /* -----------------------------------------------
     * 2. Khởi tạo GPIO Switch input
     * ----------------------------------------------- */

    uint64_t switch_pin_mask = 0;

    for (uint8_t i = 0; i < SWITCH_COUNT; i++)
    {
        switch_pin_mask |= (1ULL << s_switch_gpio[i]);
    }

    gpio_config_t switch_config =
    {
        .pin_bit_mask = switch_pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    err = gpio_config(&switch_config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Switch GPIO configuration failed");
        return err;
    }
	/* =========================================
	 * Khởi tạo BOOT button GPIO9
	 * ========================================= */
	gpio_config_t boot_button_config = {
	    .pin_bit_mask = (1ULL << GPIO_BOOT_BUTTON),
	    .mode = GPIO_MODE_INPUT,
	    .pull_up_en = GPIO_PULLUP_ENABLE,
	    .pull_down_en = GPIO_PULLDOWN_DISABLE,
	    .intr_type = GPIO_INTR_DISABLE
	};
	
	err = gpio_config(&boot_button_config);
	
	if (err != ESP_OK) {
	    ESP_LOGE(TAG, "BOOT button GPIO configuration failed");
	    return err;
	}
    /* -----------------------------------------------
     * 3. Tạo GPIO command Queue
     * ----------------------------------------------- */

    s_gpio_command_queue = xQueueCreate(
        GPIO_COMMAND_QUEUE_LENGTH,
        sizeof(gpio_command_t)
    );

    if (s_gpio_command_queue == NULL)
    {
        ESP_LOGE(TAG, "Cannot create GPIO command Queue");
        return ESP_ERR_NO_MEM;
    }

    /* -----------------------------------------------
     * 4. Tạo GPIO Control Task
     * ----------------------------------------------- */

    BaseType_t task_result = xTaskCreate(
        gpio_control_task,
        "gpio_control",
        GPIO_CONTROL_TASK_STACK_SIZE,
        NULL,
        GPIO_CONTROL_TASK_PRIORITY,
        &s_gpio_task_handle
    );

    if (task_result != pdPASS)
    {
        s_gpio_task_handle = NULL;

        if (s_gpio_command_queue != NULL)
        {
            vQueueDelete(s_gpio_command_queue);
            s_gpio_command_queue = NULL;
        }

        ESP_LOGE(TAG, "Cannot create GPIO Control Task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "GPIO Control initialized successfully");

    return ESP_OK;
}

/**
 * @brief Gửi lệnh điều khiển Relay vào GPIO Control Task.
 */
esp_err_t gpio_control_set_relay(uint8_t channel, bool state)
{
    if (channel >= SWITCH_COUNT)
    {
        ESP_LOGW(TAG, "Invalid GPIO channel: %u", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_command_queue == NULL)
    {
        ESP_LOGE(TAG, "GPIO command Queue is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

	gpio_command_t command =
	{
	    .type = GPIO_CMD_SET,
	    .channel = channel,
	    .state = state
	};

    if (xQueueSend(
            s_gpio_command_queue,
            &command,
            0
        ) != pdPASS)
    {
        ESP_LOGW(TAG, "GPIO command Queue is full");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(
        TAG,
        "GPIO command queued: channel=%u, state=%s",
        channel,
        state ? "ON" : "OFF"
    );

    return ESP_OK;
}


/**
 * @brief Gửi lệnh duration vào GPIO Control Queue.
 */
esp_err_t gpio_control_set_relay_duration(
    uint8_t channel,
    bool state,
    uint64_t duration_us)
{
    if ((channel >= SWITCH_COUNT) || (duration_us == 0))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_command_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_command_t command =
    {
        .type = GPIO_CMD_DURATION,
        .channel = channel,
        .state = state,
        .duration_us = duration_us
    };

    if (xQueueSend(s_gpio_command_queue, &command, 0) != pdPASS)
    {
        ESP_LOGW(TAG, "GPIO command Queue is full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}


/**
 * @brief Gửi lệnh cycle vào GPIO Control Queue.
 */
esp_err_t gpio_control_set_relay_cycle(
    uint8_t channel,
    uint64_t on_duration_us,
    uint64_t off_duration_us,
    uint32_t cycles)
{
    if ((channel >= SWITCH_COUNT) ||
        (on_duration_us == 0) ||
        (off_duration_us == 0))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_command_queue == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_command_t command =
    {
        .type = GPIO_CMD_CYCLE,
        .channel = channel,
        .on_duration_us = on_duration_us,
        .off_duration_us = off_duration_us,
        .cycles = cycles
    };

    if (xQueueSend(s_gpio_command_queue, &command, 0) != pdPASS)
    {
        ESP_LOGW(TAG, "GPIO command Queue is full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}





/**
 * @brief Đưa cấu hình Schedule vào GPIO Control Queue.
 *
 * ESP_OK chỉ có nghĩa Queue nhận được lệnh; việc lưu NVS được
 * thực hiện bất đồng bộ bên trong GPIO Control Task.
 */
esp_err_t gpio_control_set_relay_schedule(
    uint8_t channel,
    bool enabled,
    gpio_schedule_mode_t mode,
    uint16_t on_minute,
    uint16_t off_minute)
{
    if ((channel >= SWITCH_COUNT) ||
        (mode > GPIO_SCHEDULE_DAILY) ||
        (on_minute >= 1440U) ||
        (off_minute >= 1440U) ||
        (enabled && (on_minute == off_minute))) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_command_queue == NULL) return ESP_ERR_INVALID_STATE;

    gpio_command_t command = {
        .type = GPIO_CMD_SCHEDULE,
        .channel = channel,
        .schedule_enabled = enabled,
        .schedule_mode = mode,
        .schedule_on_minute = on_minute,
        .schedule_off_minute = off_minute
    };

    if (xQueueSend(s_gpio_command_queue, &command, 0) != pdPASS) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t gpio_control_set_relay_schedule_once(
    uint8_t channel,
    uint32_t on_date,
    uint16_t on_minute,
    uint32_t off_date,
    uint16_t off_minute)
{
    if (channel >= SWITCH_COUNT ||
        on_date == 0U ||
        off_date == 0U ||
        on_minute >= 1440U ||
        off_minute >= 1440U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_gpio_command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    time_t on_epoch;
    time_t off_epoch;

    if (!schedule_date_to_epoch(on_date, on_minute, &on_epoch) ||
        !schedule_date_to_epoch(off_date, off_minute, &off_epoch) ||
        off_epoch <= on_epoch) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_command_t command = {
        .type = GPIO_CMD_SCHEDULE,
        .channel = channel,
        .schedule_enabled = true,
        .schedule_mode = GPIO_SCHEDULE_ONCE,
        .schedule_on_minute = on_minute,
        .schedule_off_minute = off_minute,
        .schedule_on_date = on_date,
        .schedule_off_date = off_date
    };

    if (xQueueSend(s_gpio_command_queue, &command, 0) != pdPASS) {
        ESP_LOGW(TAG, "GPIO command Queue is full");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}