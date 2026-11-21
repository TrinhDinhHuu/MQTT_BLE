/*
 * app_mqtt.h
 *
 *  Created on: 2026年11月4日
 *      Author: SKY-EN-PC01
 */

#ifndef APP_MQTT_H
#define APP_MQTT_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t app_mqtt_start(void);

/* Báo cho MQTT Supervisor cấu hình lại MQTT sau khi NVS lưu thành công */
esp_err_t app_mqtt_request_reconfigure(void);

void app_mqtt_set_wifi_connected(bool connected);

/*
 * Trạng thái của chức năng cycle.
 */
typedef enum
{
    GPIO_CYCLE_STATUS_IDLE = 0,
    GPIO_CYCLE_STATUS_RUNNING,
    GPIO_CYCLE_STATUS_COMPLETED,
    GPIO_CYCLE_STATUS_CANCELLED
} gpio_cycle_status_t;


/**
 * @brief Gửi sự kiện GPIO ON/OFF thông thường.
 *
 * Dùng cho thao tác Switch hoặc lệnh ON/OFF không có cycle.
 *
 * @param gpio  Kênh logic GPIO: 0, 1, 2.
 * @param state true = ON, false = OFF.
 */
esp_err_t app_mqtt_publish_gpio_state(uint8_t gpio, bool state);


/**
 * @brief Gửi sự kiện GPIO kèm thông tin bộ đếm cycle.
 *
 * Dùng khi Relay đang chạy duration/cycle hoặc khi cycle_count thay đổi.
 *
 * @param gpio         Kênh logic GPIO: 0, 1, 2.
 * @param state        true = ON, false = OFF.
 * @param cycle_count  Số chu kỳ đã hoàn thành.
 * @param cycles_total Tổng số chu kỳ; 0 = chạy vô hạn.
 * @param cycle_status Trạng thái cycle.
 */
esp_err_t app_mqtt_publish_gpio_cycle_event(
    uint8_t gpio,
    bool state,
    uint32_t cycle_count,
    uint32_t cycles_total,
    gpio_cycle_status_t cycle_status
);
esp_err_t app_mqtt_publish_schedule_saved(
    uint8_t gpio,
    uint8_t mode,
    uint16_t on_minute,
    uint16_t off_minute,
    uint32_t on_date,
    uint32_t off_date,
    int64_t saved_at_epoch
);

#endif /* APP_MQTT_H_ */
