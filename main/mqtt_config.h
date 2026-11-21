/*
 * mqtt_config.h
 *
 *  Created on: 2026年11月5日
 *      Author: SKY-EN-PC01
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define MQTT_USERNAME_MAX_LEN  29
#define MQTT_PASSWORD_MAX_LEN  128

#define MQTT_BROKER_URI_MAX_LEN  256
#define MQTT_CLIENT_ID_MAX_LEN   32

esp_err_t mqtt_config_init(void);

esp_err_t mqtt_config_set_username(
    const uint8_t *data,
    size_t length
);

esp_err_t mqtt_config_set_password(
    const uint8_t *data,
    size_t length
);

esp_err_t mqtt_config_save(void);

bool mqtt_config_is_ready(void);

const char *mqtt_config_get_username(void);
const char *mqtt_config_get_password(void);

esp_err_t mqtt_config_set_broker_uri(
    const uint8_t *data,
    size_t length
);

const char *mqtt_config_get_broker_uri(void);
const char *mqtt_config_get_client_id(void);

#endif /* MQTT_CONFIG_H_ */
