#include <string.h>

#include "nvs.h"

#include "setup_config.h"

#define SETUP_NVS_NAMESPACE "setup_cfg"
#define SETUP_KEY_SSID      "ssid"
#define SETUP_KEY_PASSWORD  "password"
#define SETUP_KEY_BROKER_IP "broker_ip"
#define SETUP_KEY_BROKER_PT "broker_port"
#define SETUP_KEY_OTA_BROKER_IP "ota_broker_ip"

static esp_err_t load_optional_str(nvs_handle_t handle, const char *key, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = out_len;
    esp_err_t ret = nvs_get_str(handle, key, out, &len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        out[0] = '\0';
        return ESP_OK;
    }
    return ret;
}

esp_err_t setup_config_init(void)
{
    return ESP_OK;
}

esp_err_t setup_config_load(setup_config_t *config, bool *found)
{
    if (config == NULL || found == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *found = false;
    memset(config, 0, sizeof(*config));

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(SETUP_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }

    size_t ssid_len = sizeof(config->ssid);
    size_t pass_len = sizeof(config->password);
    size_t broker_len = sizeof(config->broker_ip);

    ret = nvs_get_str(handle, SETUP_KEY_SSID, config->ssid, &ssid_len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    ret = nvs_get_str(handle, SETUP_KEY_BROKER_IP, config->broker_ip, &broker_len);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        memset(config, 0, sizeof(*config));
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    ret = nvs_get_i32(handle, SETUP_KEY_BROKER_PT, (int32_t *)&config->broker_port);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        memset(config, 0, sizeof(*config));
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    ret = load_optional_str(handle, SETUP_KEY_PASSWORD, config->password, pass_len);
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }

    ret = load_optional_str(handle, SETUP_KEY_OTA_BROKER_IP, config->ota_broker_ip,
                            sizeof(config->ota_broker_ip));
    if (ret != ESP_OK) {
        nvs_close(handle);
        return ret;
    }


    nvs_close(handle);

    if (strlen(config->ssid) == 0 || strlen(config->broker_ip) == 0 || config->broker_port <= 0) {
        memset(config, 0, sizeof(*config));
        return ESP_OK;
    }

    *found = true;
    return ESP_OK;
}

esp_err_t setup_config_save(const setup_config_t *config)
{
    if (config == NULL || strlen(config->ssid) == 0 || strlen(config->broker_ip) == 0 || config->broker_port <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(SETUP_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_str(handle, SETUP_KEY_SSID, config->ssid);
    if (ret == ESP_OK) {
        ret = nvs_set_str(handle, SETUP_KEY_PASSWORD, config->password);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_str(handle, SETUP_KEY_BROKER_IP, config->broker_ip);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(handle, SETUP_KEY_BROKER_PT, (int32_t)config->broker_port);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_str(handle, SETUP_KEY_OTA_BROKER_IP, config->ota_broker_ip);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);
    return ret;
}