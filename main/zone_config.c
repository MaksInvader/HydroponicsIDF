#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "zone_config.h"

#define ZONE_NVS_NAMESPACE "zone_cfg"
#define ZONE_KEY_ID        "zone_id"
#define ZONE_KEY_NAME      "zone_name"

static const char *TAG = "zone_config";

esp_err_t zone_config_init(void)
{
    return ESP_OK;
}

esp_err_t zone_config_load(zone_config_t *config, bool *found)
{
    if (config == NULL || found == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *found = false;
    memset(config, 0, sizeof(*config));

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(ZONE_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }

    size_t id_len = sizeof(config->zone_id);
    size_t name_len = sizeof(config->zone_name);
    ret = nvs_get_str(handle, ZONE_KEY_ID, config->zone_id, &id_len);
    if (ret == ESP_OK) {
        ret = nvs_get_str(handle, ZONE_KEY_NAME, config->zone_name, &name_len);
    }
    nvs_close(handle);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        memset(config, 0, sizeof(*config));
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }

    *found = true;
    return ESP_OK;
}

esp_err_t zone_config_save(const zone_config_t *config)
{
    if (config == NULL || strlen(config->zone_id) == 0 || strlen(config->zone_name) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(ZONE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_str(handle, ZONE_KEY_ID, config->zone_id);
    if (ret == ESP_OK) {
        ret = nvs_set_str(handle, ZONE_KEY_NAME, config->zone_name);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Zone saved: id=%s, name=%s", config->zone_id, config->zone_name);
    }
    return ret;
}