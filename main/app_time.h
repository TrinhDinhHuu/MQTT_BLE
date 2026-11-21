/*
 * app_time.h
 *
 *  Created on: 2026年11月12日
 *      Author: SKY-EN-PC01
 */


#ifndef APP_TIME_H
#define APP_TIME_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Khởi tạo timezone và bộ quản lý thời gian.
 *
 * Gọi một lần sau esp_netif_init().
 */
esp_err_t app_time_init(void);

/**
 * @brief Cập nhật trạng thái Wi-Fi.
 *
 * connected = false: Wi-Fi mất kết nối hoặc chưa có IP.
 * connected = true : Wi-Fi đã nhận được IP.
 */
void app_time_set_wifi_connected(bool connected);

/**
 * @brief Kiểm tra thời gian hệ thống đã được SNTP đồng bộ
 *        và có giá trị hợp lệ hay chưa.
 */
bool app_time_is_ready(void);

/**
 * @brief Yêu cầu SNTP đồng bộ thời gian lại.
 *
 * Chỉ thực hiện khi Wi-Fi đã có IP.
 *
 * @return ESP_OK nếu đã yêu cầu đồng bộ.
 */
esp_err_t app_time_request_sync(void);

/**
 * @brief Chờ SNTP đồng bộ thời gian.
 *
 * @param timeout_ms Thời gian chờ tối đa (ms).
 *
 * @return true nếu thời gian đã đồng bộ hợp lệ.
 */
bool app_time_wait_for_sync(uint32_t timeout_ms);
/**
 * @brief Lấy số lần Wi-Fi nhận IP.
 *
 * GPIO Control Task sử dụng giá trị này để phát hiện
 * Wi-Fi reconnect và thực hiện reconcile lịch.
 */
uint32_t app_time_get_wifi_generation(void);

uint32_t app_time_get_sync_generation(void);

#endif /* APP_TIME_H */
