idf_component_register(
    SRCS
        "app_main.c"
        "app_queue.c"
        "app_mqtt.c"
        "app_time.c"
        "mqtt5_manager.c"
        "mqtt_config.c"
        "uart_comm.c"
        "status_led.c"
		"gpio_control.c"
    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        esp_event
        esp_wifi
        esp_netif
        nvs_flash
        led_strip
        mqtt
        json
        wifi_provisioning
        
)