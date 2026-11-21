/*
 * mqtt5_manager.h
 *
 *  Created on: Nov 2, 2026
 *      Author: LENOVO
 */

#ifndef MAIN_MQTT5_MANAGER_H_
#define MAIN_MQTT5_MANAGER_H_


#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct mqtt5_manager *mqtt5_manager_handle_t;

/* MQTT event */
typedef enum {
    MQTT5_MANAGER_CONNECTED,
    MQTT5_MANAGER_DISCONNECTED,
    MQTT5_MANAGER_SUBSCRIBED,
    MQTT5_MANAGER_PUBLISHED,
    MQTT5_MANAGER_DATA,
    MQTT5_MANAGER_ERROR
} mqtt5_manager_event_t;

/* MQTT received data */
typedef struct {
    const char *topic;
    int topic_len;

    const char *data;
    int data_len;

    int qos;
    bool retain;
} mqtt5_manager_data_t;



/* Event callback */
typedef void (*mqtt5_manager_event_cb_t)(
    mqtt5_manager_handle_t handle,
    mqtt5_manager_event_t event,
    const mqtt5_manager_data_t *data,
    void *user_data
);

/* MQTT configuration */
typedef struct {
    const char *broker_uri;
    const char *username;
    const char *password;
    const char *client_id;

    int keepalive;
    bool auto_reconnect;

    const char *will_topic;
    const char *will_message;
    int will_qos;
    bool will_retain;

    mqtt5_manager_event_cb_t event_cb;
    void *user_data;

} mqtt5_manager_config_t;

/* Initialize MQTT5 */
esp_err_t mqtt5_manager_init(
    const mqtt5_manager_config_t *config,
    mqtt5_manager_handle_t *handle
);

/* Start MQTT */
esp_err_t mqtt5_manager_start(
    mqtt5_manager_handle_t handle
);

/* Stop MQTT */
esp_err_t mqtt5_manager_stop(
    mqtt5_manager_handle_t handle
);

/* Destroy MQTT manager và giải phóng tài nguyên */
esp_err_t mqtt5_manager_deinit(
    mqtt5_manager_handle_t handle
);

esp_err_t mqtt5_manager_reconnect(
    mqtt5_manager_handle_t handle
);

/* Publish message */
int mqtt5_manager_publish(
    mqtt5_manager_handle_t handle,
    const char *topic,
    const char *data,
    int qos,
    bool retain
);

/* Subscribe topic */
int mqtt5_manager_subscribe(
    mqtt5_manager_handle_t handle,
    const char *topic,
    int qos
);

/* Check MQTT connection */
bool mqtt5_manager_is_connected(
    mqtt5_manager_handle_t handle
);

#ifdef __cplusplus
}
#endif



#endif /* MAIN_MQTT5_MANAGER_H_ */
