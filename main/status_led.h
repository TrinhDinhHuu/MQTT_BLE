/*
 * status_led.h
 *
 *  Created on: Nov 5, 2026
 *      Author: LENOVO
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t status_led_init(void);

void status_led_set_wifi(bool connected);

void status_led_set_mqtt(bool connected);

void status_led_off(void);

#endif /* MAIN_STATUS_LED_H_ */
