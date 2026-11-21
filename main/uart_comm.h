#ifndef UART_COMM_H
#define UART_COMM_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cấu hình UART */
#define UART_COMM_PORT_NUM      UART_NUM_1
#define UART_COMM_BAUD_RATE     115200
#define UART_COMM_TX_PIN        GPIO_NUM_4   // Chân TX (kết nối chân RX của thiết bị khác)
#define UART_COMM_RX_PIN        GPIO_NUM_5   // Chân RX (kết nối chân TX của thiết bị khác)
#define UART_COMM_BUF_SIZE      1024
#define UART_COMM_MAX_ECHO_LEN  128          // Giới hạn chuỗi echo (< 128 byte)

/* Kiểu callback nhận dữ liệu */
typedef void (*uart_rx_callback_t)(const uint8_t *data, size_t len);

/**
 * @brief Khởi tạo UART với Baudrate 115200, 8-N-1, TX=GPIO4, RX=GPIO5
 * @return esp_err_t ESP_OK nếu thành công
 */
esp_err_t uart_comm_init(void);

/**
 * @brief Truyền mảng byte qua UART
 * @param data Con trỏ vùng đệm dữ liệu cần gửi
 * @param len Độ dài dữ liệu (bytes)
 * @return Số byte đã gửi thành công, hoặc -1 nếu lỗi
 */
int uart_comm_send_bytes(const uint8_t *data, size_t len);

/**
 * @brief Truyền một chuỗi ký tự qua UART
 * @param str Chuỗi ký tự kết thúc bằng '\0'
 * @return Số byte đã gửi thành công, hoặc -1 nếu lỗi
 */
int uart_comm_send_string(const char *str);

/**
 * @brief Nhận dữ liệu từ UART (hàm đọc đồng bộ có timeout)
 * @param buf Con trỏ bộ đệm lưu dữ liệu nhận được
 * @param max_len Kích thước tối đa bộ đệm
 * @param timeout_ms Thời gian chờ tối đa (mili-giây)
 * @return Số byte thực tế nhận được, hoặc -1 nếu lỗi
 */
int uart_comm_receive_bytes(uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Đăng ký hàm callback xử lý khi có dữ liệu đến (tùy chọn)
 * @param cb Con trỏ hàm callback
 */
void uart_comm_register_rx_callback(uart_rx_callback_t cb);

/**
 * @brief Bật hoặc tắt tính năng tự động echo dữ liệu nhận được (< 128 byte)
 * @param enable true để bật (mặc định), false để tắt
 */
void uart_comm_set_echo(bool enable);

/**
 * @brief Khởi tạo FreeRTOS Task chạy ngầm tự động nhận dữ liệu UART và echo lại nếu < 128 bytes
 * @param stack_size Kích thước stack (gợi ý: 3072)
 * @param priority Mức ưu tiên task (gợi ý: 5)
 * @return esp_err_t ESP_OK nếu tạo task thành công
 */
esp_err_t uart_comm_start_rx_task(uint32_t stack_size, UBaseType_t priority);


esp_err_t uart_comm_start_task(void);

/** Wait until MCU has sent all MQTT fields and CMD_SAVE_CONFIG was committed. */
esp_err_t uart_comm_wait_config_saved(TickType_t timeout);
#ifdef __cplusplus
}
#endif

#endif // UART_COMM_H
