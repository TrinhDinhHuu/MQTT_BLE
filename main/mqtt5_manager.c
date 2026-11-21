/*
 * mqtt5_manager.c
 *
 *  Created on: Nov 2, 2026
 *      Author: LENOVO
 */

#include <stdlib.h>
#include <string.h>

#include "mqtt5_manager.h"

#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT5_MANAGER";
#define MQTT5_RX_TOPIC_MAX_LEN    128
#define MQTT5_RX_PAYLOAD_MAX_LEN  1024


struct mqtt5_manager {

    esp_mqtt_client_handle_t client;

    mqtt5_manager_event_cb_t event_cb;
    void *user_data;

    bool connected;

    /* MQTT RX buffer */
    char rx_topic[MQTT5_RX_TOPIC_MAX_LEN];
    char rx_payload[MQTT5_RX_PAYLOAD_MAX_LEN];

    int rx_total_len;
    int rx_received_len;

    int rx_qos;
    bool rx_retain;
    bool rx_in_progress;
};

static void mqtt5_handle_data(
    mqtt5_manager_handle_t manager,
    esp_mqtt_event_handle_t event
)
{
    if (manager == NULL || event == NULL) {
        return;
    }

    /* Start of a new MQTT message */
    if (event->current_data_offset == 0) {

        manager->rx_in_progress = false;
        manager->rx_received_len = 0;
        manager->rx_total_len = event->total_data_len;

        /* Check payload length */
        if (manager->rx_total_len <= 0 ||
            manager->rx_total_len >= MQTT5_RX_PAYLOAD_MAX_LEN) {

            ESP_LOGE(TAG, "Invalid MQTT payload length: %d",
                     manager->rx_total_len);
            return;
        }

        /* Check topic */
        if (event->topic == NULL ||
            event->topic_len <= 0 ||
            event->topic_len >= MQTT5_RX_TOPIC_MAX_LEN) {

            ESP_LOGE(TAG, "Invalid MQTT topic");
            return;
        }

        memcpy(manager->rx_topic,
               event->topic,
               event->topic_len);

        manager->rx_topic[event->topic_len] = '\0';

        manager->rx_qos = event->qos;
        manager->rx_retain = event->retain;

        manager->rx_in_progress = true;
    }

    if (!manager->rx_in_progress) {
        return;
    }

    /* Validate fragment position */
    if (event->current_data_offset != manager->rx_received_len ||
        event->data_len < 0 ||
        event->current_data_offset + event->data_len >
            manager->rx_total_len ||
        (event->data_len > 0 && event->data == NULL)) {

        ESP_LOGE(TAG, "Invalid MQTT fragment");

        manager->rx_in_progress = false;
        manager->rx_received_len = 0;
        manager->rx_total_len = 0;
        return;
    }

    /* Copy fragment into manager buffer */
    if (event->data_len > 0) {

        memcpy(
            manager->rx_payload + manager->rx_received_len,
            event->data,
            event->data_len
        );

        manager->rx_received_len += event->data_len;
    }

    ESP_LOGD(TAG, "MQTT fragment: %d/%d",
             manager->rx_received_len,
             manager->rx_total_len);

    /* Wait until all fragments arrive */
    if (manager->rx_received_len < manager->rx_total_len) {
        return;
    }

    /* Complete message */
    manager->rx_payload[manager->rx_total_len] = '\0';

    ESP_LOGI(TAG, "MQTT message complete");
    ESP_LOGI(TAG, "Topic: %s", manager->rx_topic);
    ESP_LOGI(TAG, "Payload length: %d", manager->rx_total_len);

    mqtt5_manager_data_t received_data = {
        .topic = manager->rx_topic,
        .topic_len = strlen(manager->rx_topic),
        .data = manager->rx_payload,
        .data_len = manager->rx_total_len,
        .qos = manager->rx_qos,
        .retain = manager->rx_retain
    };

    /* Mark complete before notifying application */
    manager->rx_in_progress = false;

    /* Notify application exactly once */
    if (manager->event_cb != NULL) {

        manager->event_cb(
            manager,
            MQTT5_MANAGER_DATA,
            &received_data,
            manager->user_data
        );
    }

    manager->rx_received_len = 0;
    manager->rx_total_len = 0;
}

static void mqtt5_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data
)
{
    mqtt5_manager_handle_t manager =
        (mqtt5_manager_handle_t)handler_args;

    esp_mqtt_event_handle_t event =
        (esp_mqtt_event_handle_t)event_data;

    if (manager == NULL || event == NULL) {
        return;
    }

    mqtt5_manager_event_t app_event;
    bool notify = true;

    mqtt5_manager_data_t received_data = {0};

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:

        manager->connected = true;

        ESP_LOGI(TAG, "MQTT5 connected");

        app_event = MQTT5_MANAGER_CONNECTED;
        break;

    case MQTT_EVENT_DISCONNECTED:

        manager->connected = false;

        ESP_LOGW(TAG, "MQTT5 disconnected");

        app_event = MQTT5_MANAGER_DISCONNECTED;
        break;

    case MQTT_EVENT_SUBSCRIBED:

        ESP_LOGI(TAG, "Subscribe successful, msg_id=%d",
                 event->msg_id);

        app_event = MQTT5_MANAGER_SUBSCRIBED;
        break;

    case MQTT_EVENT_PUBLISHED:

        ESP_LOGI(TAG, "Publish successful, msg_id=%d",
                 event->msg_id);

        app_event = MQTT5_MANAGER_PUBLISHED;
        break;

    case MQTT_EVENT_DATA:

        ESP_LOGI(TAG, "MQTT5 data received");
        
		/*
        received_data.topic = event->topic;
        received_data.topic_len = event->topic_len;

        received_data.data = event->data;
        received_data.data_len = event->data_len;

        received_data.qos = event->qos;
        received_data.retain = event->retain;

        app_event = MQTT5_MANAGER_DATA;
        */
        
    	mqtt5_handle_data(manager, event);

    	/* Data callback is handled by mqtt5_handle_data() */
    	notify = false;
            
        break;

    case MQTT_EVENT_ERROR:

        ESP_LOGE(TAG, "MQTT5 error");

        if (event->error_handle != NULL) {

            ESP_LOGE(TAG, "MQTT error type: %d",
                     event->error_handle->error_type);

            ESP_LOGE(TAG, "MQTT5 return code: %d",
                     event->error_handle->connect_return_code);

            ESP_LOGE(TAG, "TLS error: 0x%x",
                     event->error_handle->esp_tls_last_esp_err);

            ESP_LOGE(TAG, "TLS stack error: 0x%x",
                     event->error_handle->esp_tls_stack_err);
        }

        app_event = MQTT5_MANAGER_ERROR;
        break;

    default:

        notify = false;
        break;
    }

    if (notify && manager->event_cb != NULL) {

        manager->event_cb(
            manager,
            app_event,
            (app_event == MQTT5_MANAGER_DATA)
                ? &received_data : NULL,
            manager->user_data
        );
    }
}


/* Initialize MQTT5 */
esp_err_t mqtt5_manager_init(
    const mqtt5_manager_config_t *config,
    mqtt5_manager_handle_t *handle
)
{
    if (config == NULL ||
        handle == NULL ||
        config->broker_uri == NULL) {

        return ESP_ERR_INVALID_ARG;
    }

    *handle = NULL;

    mqtt5_manager_handle_t manager =
        calloc(1, sizeof(struct mqtt5_manager));

    if (manager == NULL) {
        return ESP_ERR_NO_MEM;
    }

    manager->event_cb = config->event_cb;
    manager->user_data = config->user_data;

    esp_mqtt_client_config_t mqtt_cfg = {

        .broker.address.uri = config->broker_uri,

        /* TLS certificate verification */
        .broker.verification.crt_bundle_attach =
            esp_crt_bundle_attach,

        /* MQTT protocol version 5.0 */
        .session.protocol_ver = MQTT_PROTOCOL_V_5,

        /* Network */
        .network.disable_auto_reconnect =
            !config->auto_reconnect,

        /* Authentication */
        .credentials.username = config->username,
        .credentials.authentication.password =
            config->password,

        .credentials.client_id = config->client_id,

        /* Keep Alive */
        .session.keepalive = config->keepalive,

        /* Last Will */
        .session.last_will.topic = config->will_topic,
        .session.last_will.msg = config->will_message,

        .session.last_will.msg_len =
            config->will_message
                ? strlen(config->will_message) : 0,

        .session.last_will.qos = config->will_qos,
        .session.last_will.retain = config->will_retain,
    };

    manager->client = esp_mqtt_client_init(&mqtt_cfg);

    if (manager->client == NULL) {

        free(manager);

        ESP_LOGE(TAG, "MQTT client initialization failed");

        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        manager->client,
        ESP_EVENT_ANY_ID,
        mqtt5_event_handler,
        manager
    );

    if (err != ESP_OK) {

        esp_mqtt_client_destroy(manager->client);
        free(manager);

        return err;
    }

    *handle = manager;

    ESP_LOGI(TAG, "MQTT5 manager initialized");

    return ESP_OK;
}


/* Start MQTT */
esp_err_t mqtt5_manager_start(
    mqtt5_manager_handle_t handle
)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting MQTT5 client");

    return esp_mqtt_client_start(handle->client);
}


/* Stop MQTT */
esp_err_t mqtt5_manager_stop(
    mqtt5_manager_handle_t handle
)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->connected = false;

    ESP_LOGI(TAG, "Stopping MQTT5 client");

    return esp_mqtt_client_stop(handle->client);
}

/* Request MQTT reconnect */
esp_err_t mqtt5_manager_reconnect(
    mqtt5_manager_handle_t handle
)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->connected) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Requesting MQTT reconnect");

    return esp_mqtt_client_reconnect(handle->client);
}

/* Destroy MQTT manager */
esp_err_t mqtt5_manager_deinit(
    mqtt5_manager_handle_t handle
)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->connected = false;

    esp_err_t err = esp_mqtt_client_destroy(handle->client);

    if (err != ESP_OK) {
        return err;
    }

    free(handle);

    ESP_LOGI(TAG, "MQTT5 manager destroyed");

    return ESP_OK;
}


/* Publish message */
int mqtt5_manager_publish(
    mqtt5_manager_handle_t handle,
    const char *topic,
    const char *data,
    int qos,
    bool retain
)
{
    if (handle == NULL ||
        topic == NULL ||
        data == NULL ||
        qos < 0 || qos > 2) {

        return -1;
    }

    if (!handle->connected) {

        ESP_LOGW(TAG, "MQTT is not connected");

        return -1;
    }

    return esp_mqtt_client_publish(
        handle->client,
        topic,
        data,
        0,
        qos,
        retain
    );
}


/* Subscribe topic */
int mqtt5_manager_subscribe(
    mqtt5_manager_handle_t handle,
    const char *topic,
    int qos
)
{
    if (handle == NULL ||
        topic == NULL ||
        qos < 0 || qos > 2) {

        return -1;
    }

    if (!handle->connected) {

        ESP_LOGW(TAG, "MQTT is not connected");

        return -1;
    }

    return esp_mqtt_client_subscribe(
        handle->client,
        topic,
        qos
    );
}


/* Check connection */
bool mqtt5_manager_is_connected(
    mqtt5_manager_handle_t handle
)
{
    if (handle == NULL) {
        return false;
    }

    return handle->connected;
}




