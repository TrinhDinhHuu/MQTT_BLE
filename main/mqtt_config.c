/*
 * mqtt_config.c
 *
 *  Created on: 2026年11月5日
 *      Author: SKY-EN-PC01
 */
#include "mqtt_config.h"

#include <string.h>
#include <stdio.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_mac.h"

#define MQTT_NVS_BROKER_URI "broker_uri"
#define MQTT_NVS_NAMESPACE "mqtt_cfg"
#define MQTT_NVS_USERNAME  "username"
#define MQTT_NVS_PASSWORD  "password"

static const char *TAG = "MQTT_CONFIG";

/* RAM buffer: giữ dữ liệu trong suốt thời gian chạy */

static char s_broker_uri[MQTT_BROKER_URI_MAX_LEN + 1] = {0};
static char s_client_id[MQTT_CLIENT_ID_MAX_LEN + 1] = {0};

/* Active credentials: dữ liệu đang được sử dụng để kết nối MQTT */
static char s_username[MQTT_USERNAME_MAX_LEN + 1] = {0};
static char s_password[MQTT_PASSWORD_MAX_LEN + 1] = {0};

/* Staging credentials: dữ liệu mới nhận từ MCU qua UART */
static char s_new_broker_uri[MQTT_BROKER_URI_MAX_LEN + 1] = {0};
static char s_new_username[MQTT_USERNAME_MAX_LEN + 1] = {0};
static char s_new_password[MQTT_PASSWORD_MAX_LEN + 1] = {0};

//static bool s_username_valid = false;
//static bool s_password_valid = false;

static bool username_is_valid(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0 ||
        length > MQTT_USERNAME_MAX_LEN) {
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        uint8_t c = data[i];

        /* Chỉ nhận ASCII có thể in được */
        if (c < 0x21 || c > 0x7E) {
            return false;
        }

        /* Các ký tự không hợp lệ trong MQTT topic */
        if (c == '/' || c == '+' || c == '#') {
            return false;
        }
    }

    return true;
}

static bool password_is_valid(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0 ||
        length > MQTT_PASSWORD_MAX_LEN) {
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        if (data[i] < 0x20 || data[i] > 0x7E) {
            return false;
        }
    }

    return true;
}

static bool broker_uri_is_valid(
    const uint8_t *data,
    size_t length)
{
    size_t prefix_len = 0;

    if (data == NULL || length == 0 ||
        length > MQTT_BROKER_URI_MAX_LEN) {
        return false;
    }

    /* Xác định scheme */
    if (length > 7 &&
        memcmp(data, "mqtt://", 7) == 0) {
        prefix_len = 7;
    }
    else if (length > 8 &&
             memcmp(data, "mqtts://", 8) == 0) {
        prefix_len = 8;
    }
    else {
        return false;
    }

    /* Phải có hostname sau scheme */
    if (prefix_len >= length) {
        return false;
    }

    /* Không chấp nhận khoảng trắng hoặc ký tự điều khiển */
    for (size_t i = 0; i < length; i++) {
        if (data[i] <= 0x20 || data[i] > 0x7E) {
            return false;
        }
    }

    return true;
}
static esp_err_t mqtt_config_generate_client_id(void)
{
    uint8_t mac[6];

    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read Wi-Fi MAC: %s",
                 esp_err_to_name(err));
        return err;
    }

    int length = snprintf(
        s_client_id,
        sizeof(s_client_id),
        "ESP32C3_%02X%02X%02X%02X%02X%02X",
        mac[0], mac[1], mac[2],
        mac[3], mac[4], mac[5]
    );

    if (length < 0 ||
        length >= sizeof(s_client_id)) {
        memset(s_client_id, 0, sizeof(s_client_id));
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "MQTT Client ID generated");

    return ESP_OK;
}

esp_err_t mqtt_config_init(void)
{
    nvs_handle_t handle;
    esp_err_t err;

    memset(s_username, 0, sizeof(s_username));
    memset(s_password, 0, sizeof(s_password));
    
	memset(s_broker_uri, 0, sizeof(s_broker_uri));
	memset(s_client_id, 0, sizeof(s_client_id));
	
	memset(s_new_username, 0, sizeof(s_new_username));
	memset(s_new_password, 0, sizeof(s_new_password));
	memset(s_new_broker_uri, 0, sizeof(s_new_broker_uri));
	
	err = mqtt_config_generate_client_id();
	
	if (err != ESP_OK) {
	    return err;
	}    

    err = nvs_open(MQTT_NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "MQTT configuration not found");
        return ESP_ERR_NOT_FOUND;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open MQTT NVS: %s",
                 esp_err_to_name(err));
        return err;
    }

    size_t username_size = sizeof(s_username);
    size_t password_size = sizeof(s_password);

    err = nvs_get_str(handle, MQTT_NVS_USERNAME,
                      s_username, &username_size);

    if (err == ESP_OK) {
        err = nvs_get_str(handle, MQTT_NVS_PASSWORD,
                          s_password, &password_size);
    }

	esp_err_t broker_err = ESP_OK;
	size_t broker_uri_size = sizeof(s_broker_uri);
	
	if (err == ESP_OK) {
	    broker_err = nvs_get_str(
	        handle,
	        MQTT_NVS_BROKER_URI,
	        s_broker_uri,
	        &broker_uri_size
	    );
	}

    nvs_close(handle);

	if (err != ESP_OK ||
	    s_username[0] == '\0' ||
	    s_password[0] == '\0') {
	
	    memset(s_username, 0, sizeof(s_username));
	    memset(s_password, 0, sizeof(s_password));
	
	    ESP_LOGW(TAG, "MQTT credentials incomplete");
	    return ESP_ERR_NOT_FOUND;
	}
	
	if (!username_is_valid((const uint8_t *)s_username,
	                       strlen(s_username)) ||
	    !password_is_valid((const uint8_t *)s_password,
	                       strlen(s_password))) {
	
	    memset(s_username, 0, sizeof(s_username));
	    memset(s_password, 0, sizeof(s_password));
	
	    ESP_LOGE(TAG, "Stored MQTT credentials are invalid");
	    return ESP_ERR_INVALID_STATE;
	}
	
	/* Đồng bộ staging với dữ liệu đang lưu */
	memcpy(s_new_username, s_username, sizeof(s_username));
	memcpy(s_new_password, s_password, sizeof(s_password));
	
	/* Trường hợp firmware cũ chưa có key broker_uri */
	if (broker_err == ESP_ERR_NVS_NOT_FOUND) {
	
	    memset(s_broker_uri, 0, sizeof(s_broker_uri));
	    memset(s_new_broker_uri, 0, sizeof(s_new_broker_uri));
	
	    ESP_LOGW(TAG, "MQTT broker URI is not configured");
	    return ESP_ERR_NOT_FOUND;
	}
	
	/* Các lỗi đọc NVS khác */
	if (broker_err != ESP_OK) {
	
	    memset(s_broker_uri, 0, sizeof(s_broker_uri));
	    memset(s_new_broker_uri, 0, sizeof(s_new_broker_uri));
	
	    ESP_LOGE(TAG, "Cannot read broker URI from NVS: %s",
	             esp_err_to_name(broker_err));
	
	    return broker_err;
	}
	
	/* Kiểm tra nội dung URI */
	if (!broker_uri_is_valid(
	        (const uint8_t *)s_broker_uri,
	        strlen(s_broker_uri))) {
	
	    memset(s_broker_uri, 0, sizeof(s_broker_uri));
	    memset(s_new_broker_uri, 0, sizeof(s_new_broker_uri));
	
	    ESP_LOGE(TAG, "Stored broker URI is invalid");
	
	    return ESP_ERR_INVALID_STATE;
	}
	
	/* Đồng bộ broker URI vào staging */
	memcpy(s_new_broker_uri,
	       s_broker_uri,
	       sizeof(s_broker_uri));
	
	ESP_LOGI(TAG, "MQTT configuration loaded from NVS");
	
	return ESP_OK;
}

esp_err_t mqtt_config_set_username(
    const uint8_t *data,
    size_t length)
{
    if (!username_is_valid(data, length)) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(s_new_username, data, length);
    s_new_username[length] = '\0';

    return ESP_OK;
}

esp_err_t mqtt_config_set_password(
    const uint8_t *data,
    size_t length)
{
    if (!password_is_valid(data, length)) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(s_new_password, data, length);
    s_new_password[length] = '\0';

    return ESP_OK;
}

esp_err_t mqtt_config_set_broker_uri(
    const uint8_t *data,
    size_t length)
{
    if (!broker_uri_is_valid(data, length)) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(s_new_broker_uri, data, length);
    s_new_broker_uri[length] = '\0';

    return ESP_OK;
}

esp_err_t mqtt_config_save(void)
{
    nvs_handle_t handle;
    esp_err_t err;

	if (s_new_username[0] == '\0' ||
	    s_new_password[0] == '\0' ||
	    s_new_broker_uri[0] == '\0') {
	
	    ESP_LOGW(TAG, "MQTT configuration is incomplete");
	    return ESP_ERR_INVALID_STATE;
	}
    err = nvs_open(MQTT_NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open NVS: %s",
                 esp_err_to_name(err));
        return err;
    }

	err = nvs_set_str(handle, MQTT_NVS_USERNAME,
	                  s_new_username);
	
	if (err == ESP_OK) {
	    err = nvs_set_str(handle, MQTT_NVS_PASSWORD,
	                      s_new_password);
	}
	
	if (err == ESP_OK) {
	    err = nvs_set_str(handle, MQTT_NVS_BROKER_URI,
	                      s_new_broker_uri);
	}

    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT configuration save failed");
        return err;
    }

    /* Update active RAM credentials only after successful commit */
    memcpy(s_username, s_new_username, sizeof(s_username));
    memcpy(s_password, s_new_password, sizeof(s_password));
    memcpy(s_broker_uri, s_new_broker_uri, sizeof(s_broker_uri));

    ESP_LOGI(TAG, "MQTT credentials saved successfully");

    /* Không xóa staging.
       Giữ staging đồng bộ với cấu hình mới nhất. */
       
    /* Clear temporary credential buffers */
    //memset(s_new_username, 0, sizeof(s_new_username));
    //memset(s_new_password, 0, sizeof(s_new_password));

    return ESP_OK;
}

bool mqtt_config_is_ready(void)
{
    return (s_username[0] != '\0' &&
            s_password[0] != '\0' &&
            s_broker_uri[0] != '\0' &&
            s_client_id[0] != '\0');
}

const char *mqtt_config_get_username(void)
{
    return s_username;
}

const char *mqtt_config_get_password(void)
{
    return s_password;
}

const char *mqtt_config_get_broker_uri(void)
{
    return s_broker_uri;
}

const char *mqtt_config_get_client_id(void)
{
    return s_client_id;
}




