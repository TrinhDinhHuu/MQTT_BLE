/*
 * gpio_control.h
 *
 *  Created on: 2026年11月10日
 *      Author: SKY-EN-PC01
 */

#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief Khởi tạo GPIO Switch và Relay, sau đó tạo GPIO Control Task.
 *
 * @return ESP_OK nếu khởi tạo thành công.
 */

esp_err_t gpio_control_init(void);


/**
 * @brief Điều khiển Relay ON/OFF thông thường.
 *
 * Lệnh mới sẽ hủy duration/cycle đang chạy trên cùng Relay.
 *
 * @param channel Kênh Relay: 0, 1, 2.
 * @param state   true = ON, false = OFF.
 *
 * @return ESP_OK nếu lệnh được đưa vào Queue thành công.
 */
esp_err_t gpio_control_set_relay(uint8_t channel, bool state);


/**
 * @brief Điều khiển Relay ON/OFF trong một khoảng thời gian,
 * sau đó tự động đảo sang trạng thái ngược lại.
 *
 * Ví dụ:
 * state = true  -> ON trong duration_us, sau đó OFF.
 * state = false -> OFF trong duration_us, sau đó ON.
 *
 * Lệnh mới sẽ hủy duration/cycle đang chạy trên cùng Relay.
 *
 * @param channel     Kênh Relay: 0, 1, 2.
 * @param state       Trạng thái ban đầu.
 * @param duration_us Thời gian duy trì, đơn vị microsecond.
 *
 * @return ESP_OK nếu lệnh được đưa vào Queue thành công.
 */
esp_err_t gpio_control_set_relay_duration(
    uint8_t channel,
    bool state,
    uint64_t duration_us
);


/**
 * @brief Điều khiển Relay ON/OFF lặp lại theo chu kỳ.
 *
 * Một chu kỳ gồm:
 * ON trong on_duration_us
 * OFF trong off_duration_us
 *
 * cycle_count chỉ tăng sau khi hoàn thành cả ON và OFF.
 *
 * cycles = 0: chạy vô hạn.
 * cycles > 0: kết thúc sau khi hoàn thành đủ số chu kỳ.
 *
 * Lệnh mới sẽ hủy cycle đang chạy trên cùng Relay.
 *
 * @param channel         Kênh Relay: 0, 1, 2.
 * @param on_duration_us  Thời gian ON, đơn vị microsecond.
 * @param off_duration_us Thời gian OFF, đơn vị microsecond.
 * @param cycles          Tổng số chu kỳ; 0 = chạy vô hạn.
 *
 * @return ESP_OK nếu lệnh được đưa vào Queue thành công.
 */
esp_err_t gpio_control_set_relay_cycle(
    uint8_t channel,
    uint64_t on_duration_us,
    uint64_t off_duration_us,
    uint32_t cycles
);


/* ============================================================
 * Schedule Relay
 * ============================================================ */
typedef enum
{
    GPIO_SCHEDULE_ONCE = 0,  /* ON và OFF thực hiện một lần */
    GPIO_SCHEDULE_DAILY     /* ON và OFF thực hiện hằng ngày */
} gpio_schedule_mode_t;

/**
 * @brief Cấu hình Schedule ON/OFF cho một Relay.
 * @param channel Kênh Relay: 0, 1, 2.
 * @param enabled true = bật Schedule.
 * @param mode GPIO_SCHEDULE_ONCE hoặc GPIO_SCHEDULE_DAILY.
 * @param on_minute Giờ ON, tính từ 00:00 (0..1439).
 * @param off_minute Giờ OFF, tính từ 00:00 (0..1439).
 */
esp_err_t gpio_control_set_relay_schedule(
    uint8_t channel,
    bool enabled,
    gpio_schedule_mode_t mode,
    uint16_t on_minute,
    uint16_t off_minute
);
/**
 * @brief Cấu hình Schedule ONCE với ngày và giờ ON/OFF tuyệt đối.
 *
 * @param channel    Kênh Relay: 0, 1, 2.
 * @param on_date    Ngày ON theo định dạng YYYYMMDD.
 * @param on_minute  Phút ON tính từ 00:00 (0..1439).
 * @param off_date   Ngày OFF theo định dạng YYYYMMDD.
 * @param off_minute Phút OFF tính từ 00:00 (0..1439).
 *
 * Điều kiện: thời điểm OFF phải sau thời điểm ON.
 *
 * @return ESP_OK nếu lệnh được đưa vào Queue thành công.
 */
esp_err_t gpio_control_set_relay_schedule_once(
    uint8_t channel,
    uint32_t on_date,
    uint16_t on_minute,
    uint32_t off_date,
    uint16_t off_minute
);
/* Callback khi nhấn giữ nút BOOT GPIO9 đủ thời gian */
typedef void (*gpio_boot_button_callback_t)(void);

/* Đăng ký callback cho nút BOOT */
esp_err_t gpio_control_register_boot_button_callback(
    gpio_boot_button_callback_t callback
);

#endif /* GPIO_CONTROL_H */
