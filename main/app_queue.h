/*
 * app_queue.h
 *
 *  Created on: 2026年11月4日
 *      Author: SKY-EN-PC01
 */


#ifndef APP_QUEUE_H
#define APP_QUEUE_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MQTT_PAYLOAD_MAX_LEN 256

#define UART_MCU_DATA_MAX_LEN 16
#define UART_RX_MAX_LEN       256

/* MQTT -> UART */
typedef struct {
    char payload[MQTT_PAYLOAD_MAX_LEN];
    uint16_t length;
} mqtt_command_t;

/* UART -> MQTT: dữ liệu UART thô */
typedef struct {
    uint16_t length;
    uint8_t data[UART_RX_MAX_LEN];
} uart_rx_data_t;

/* Packet UART đã phân tích (sử dụng sau này) */
typedef struct {
    uint16_t command;
    uint8_t data_length;
    uint8_t data[UART_MCU_DATA_MAX_LEN];
} uart_mcu_packet_t;

extern QueueHandle_t mqtt_to_uart_queue;
extern QueueHandle_t uart_to_mqtt_queue;

void app_queue_init(void);

#endif /* APP_QUEUE_H_ */
