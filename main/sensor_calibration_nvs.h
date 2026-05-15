#ifndef SENSOR_CALIBRATION_NVS_H
#define SENSOR_CALIBRATION_NVS_H

#include "esp_err.h"
#include "sensor_telemetry.h"   /* calibration_t */

/**
 * @file  sensor_calibration_nvs.h
 * @brief Persist and restore linear calibration coefficients (slope / offset)
 *        for the pH and TDS sensors using the ESP-IDF NVS subsystem.
 *
 * NVS namespace : "sensor_cali"
 * Keys          : "ph_slope", "ph_offset", "ph_valid", "ph_upd_at"
 *                 "tds_slope", "tds_offset", "tds_valid", "tds_upd_at"
 */

/**
 * @brief  Save pH calibration to NVS.
 * @param  c   Pointer to the populated calibration_t (must not be NULL).
 * @return ESP_OK on success, propagated NVS error otherwise.
 */
esp_err_t sensor_calibration_nvs_save_ph(const calibration_t *c);

/**
 * @brief  Save TDS calibration to NVS.
 */
esp_err_t sensor_calibration_nvs_save_tds(const calibration_t *c);

/**
 * @brief  Load pH calibration from NVS.
 *
 * @param  c      Output struct; zeroed and c->valid set false if not found.
 * @param  found  Set true if valid data was loaded, false if no entry exists.
 * @return ESP_OK even when not found (check *found); propagated error on I/O
 *         failure.
 */
esp_err_t sensor_calibration_nvs_load_ph(calibration_t *c, bool *found);

/**
 * @brief  Load TDS calibration from NVS.
 */
esp_err_t sensor_calibration_nvs_load_tds(calibration_t *c, bool *found);

/**
 * @brief  Read back pH calibration from NVS and compare to expected values.
 *
 * Used after a write to detect NVS corruption.
 *
 * @param  expected_slope   The slope value just written.
 * @param  expected_offset  The offset value just written.
 * @return ESP_OK if read-back matches; ESP_ERR_INVALID_STATE on mismatch;
 *         propagated NVS error on I/O failure.
 */
esp_err_t sensor_calibration_nvs_verify_ph(float expected_slope, float expected_offset);

/**
 * @brief  Read back TDS calibration from NVS and compare to expected values.
 */
esp_err_t sensor_calibration_nvs_verify_tds(float expected_slope, float expected_offset);

#endif /* SENSOR_CALIBRATION_NVS_H */