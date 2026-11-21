/*
 * status_led.c
 *
 *  Created on: Nov 5, 2026
 *      Author: LENOVO
 */
#include "status_led.h"

#include "led_strip.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define STATUS_LED_GPIO       8
#define STATUS_LED_COUNT      1

static const char *TAG = "STATUS_LED";

static led_strip_handle_t s_led_strip = NULL;
static SemaphoreHandle_t s_led_mutex = NULL;

static bool s_wifi_connected = false;
static bool s_mqtt_connected = false;

static void status_led_update(void)
{
    if (s_led_strip == NULL) {
        return;
    }

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    /* Wi-Fi disconnected: RED */
    if (!s_wifi_connected) {
        r = 30;
    }
    /* Wi-Fi connected, MQTT connected: GREEN */
    else if (s_mqtt_connected) {
        g = 30;
    }
    /* Wi-Fi connected, MQTT disconnected: BLUE */
    else {
        b = 30;
    }

    led_strip_set_pixel(s_led_strip, 0, r, g, b);
    led_strip_refresh(s_led_strip);
}


esp_err_t status_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_GPIO,
        .max_leds = STATUS_LED_COUNT,
        .led_model = LED_MODEL_WS2812,

        // WS2812 sử dụng thứ tự màu GRB
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,

        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(
        &strip_config,
        &rmt_config,
        &s_led_strip
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB LED initialization failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    s_led_mutex = xSemaphoreCreateMutex();

    if (s_led_mutex == NULL) {
        led_strip_del(s_led_strip);
        s_led_strip = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_wifi_connected = false;
    s_mqtt_connected = false;

    status_led_update();

    ESP_LOGI(TAG, "RGB LED initialized on GPIO8");

    return ESP_OK;
}


void status_led_set_wifi(bool connected)
{
    if (s_led_mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_led_mutex, portMAX_DELAY);

    s_wifi_connected = connected;

    /* Wi-Fi lost means MQTT is also considered unavailable */
    if (!connected) {
        s_mqtt_connected = false;
    }

    status_led_update();

    xSemaphoreGive(s_led_mutex);
}

void status_led_set_mqtt(bool connected)
{
    if (s_led_mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_led_mutex, portMAX_DELAY);

    s_mqtt_connected = connected;

    status_led_update();

    xSemaphoreGive(s_led_mutex);
}

void status_led_off(void)
{
    if (s_led_mutex == NULL) {
        return;
    }

    xSemaphoreTake(s_led_mutex, portMAX_DELAY);

    led_strip_clear(s_led_strip);

    xSemaphoreGive(s_led_mutex);
}



