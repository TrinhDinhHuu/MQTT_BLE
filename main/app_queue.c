/*
 * app_queue.c
 *
 *  Created on: 2026年11月4日
 *      Author: SKY-EN-PC01
 */

#include "app_queue.h"
#include "esp_log.h"

static const char *TAG = "APP_QUEUE";

QueueHandle_t mqtt_to_uart_queue = NULL;
QueueHandle_t uart_to_mqtt_queue = NULL;

void app_queue_init(void)
{
    mqtt_to_uart_queue = xQueueCreate(
        10,
        sizeof(mqtt_command_t)
    );

    uart_to_mqtt_queue = xQueueCreate(
        20,
        sizeof(uart_rx_data_t)
    );

    configASSERT(mqtt_to_uart_queue != NULL);
    configASSERT(uart_to_mqtt_queue != NULL);

    ESP_LOGI(TAG, "Queues initialized");
}




