#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "sensor_calibration_nvs.h"

#define CALI_NVS_NAMESPACE "sensor_cali"

/* NVS key names — kept short; NVS key limit is 15 chars */
#define KEY_PH_SLOPE    "ph_slope"
#define KEY_PH_OFFSET   "ph_offset"
#define KEY_PH_VALID    "ph_valid"
#define KEY_PH_UPD_AT   "ph_upd_at"

#define KEY_TDS_SLOPE   "tds_slope"
#define KEY_TDS_OFFSET  "tds_offset"
#define KEY_TDS_VALID   "tds_valid"
#define KEY_TDS_UPD_AT  "tds_upd_at"

static const char *TAG = "cali_nvs";

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

/** Store a float as a uint32 blob (IEEE-754 bit pattern). */
static esp_err_t nvs_set_float(nvs_handle_t h, const char *key, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    return nvs_set_u32(h, key, bits);
}

/** Load a float from a uint32 blob. */
static esp_err_t nvs_get_float(nvs_handle_t h, const char *key, float *v)
{
    uint32_t bits = 0;
    esp_err_t ret = nvs_get_u32(h, key, &bits);
    if (ret == ESP_OK) {
        memcpy(v, &bits, sizeof(*v));
    }
    return ret;
}

/* Write one calibration_t under a fixed set of keys. */
static esp_err_t save_calibration(const char *slope_key,
                                  const char *offset_key,
                                  const char *valid_key,
                                  const char *upd_at_key,
                                  const calibration_t *c)
{
    if (c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t ret = nvs_open(CALI_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_float(h, slope_key,  c->slope);
    if (ret == ESP_OK) ret = nvs_set_float(h, offset_key, c->offset);
    if (ret == ESP_OK) ret = nvs_set_u8(h,   valid_key,  (uint8_t)c->valid);
    if (ret == ESP_OK) ret = nvs_set_u32(h,  upd_at_key, c->updated_at);
    if (ret == ESP_OK) ret = nvs_commit(h);

    nvs_close(h);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "save_calibration failed (%s): %s",
                 slope_key, esp_err_to_name(ret));
    }
    return ret;
}

/* Read one calibration_t; populates *found = false (and zeros c) if absent. */
static esp_err_t load_calibration(const char *slope_key,
                                  const char *offset_key,
                                  const char *valid_key,
                                  const char *upd_at_key,
                                  calibration_t *c,
                                  bool *found)
{
    if (c == NULL || found == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(c, 0, sizeof(*c));
    *found = false;

    nvs_handle_t h;
    esp_err_t ret = nvs_open(CALI_NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK; /* namespace not yet written — treat as not-found */
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(RO) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_get_float(h, slope_key,  &c->slope);
    if (ret == ESP_OK) ret = nvs_get_float(h, offset_key, &c->offset);

    uint8_t valid_u8 = 0;
    if (ret == ESP_OK) ret = nvs_get_u8(h, valid_key, &valid_u8);
    if (ret == ESP_OK) ret = nvs_get_u32(h, upd_at_key, &c->updated_at);

    nvs_close(h);

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        memset(c, 0, sizeof(*c));
        return ESP_OK; /* keys absent — not yet calibrated */
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "load_calibration failed (%s): %s",
                 slope_key, esp_err_to_name(ret));
        return ret;
    }

    c->valid = (bool)valid_u8;
    *found   = true;
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t sensor_calibration_nvs_save_ph(const calibration_t *c)
{
    esp_err_t ret = save_calibration(KEY_PH_SLOPE, KEY_PH_OFFSET,
                                     KEY_PH_VALID, KEY_PH_UPD_AT, c);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "pH calibration saved: slope=%.4f offset=%.4f valid=%d",
                 c->slope, c->offset, (int)c->valid);
    }
    return ret;
}

esp_err_t sensor_calibration_nvs_save_tds(const calibration_t *c)
{
    esp_err_t ret = save_calibration(KEY_TDS_SLOPE, KEY_TDS_OFFSET,
                                     KEY_TDS_VALID, KEY_TDS_UPD_AT, c);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TDS calibration saved: slope=%.4f offset=%.4f valid=%d",
                 c->slope, c->offset, (int)c->valid);
    }
    return ret;
}

esp_err_t sensor_calibration_nvs_load_ph(calibration_t *c, bool *found)
{
    return load_calibration(KEY_PH_SLOPE, KEY_PH_OFFSET,
                            KEY_PH_VALID, KEY_PH_UPD_AT, c, found);
}

esp_err_t sensor_calibration_nvs_load_tds(calibration_t *c, bool *found)
{
    return load_calibration(KEY_TDS_SLOPE, KEY_TDS_OFFSET,
                            KEY_TDS_VALID, KEY_TDS_UPD_AT, c, found);
}