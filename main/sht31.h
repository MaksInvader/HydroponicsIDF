/**
 * @file  sht31.h
 * @brief I2C driver for the GY-SHT31 Temperature and Humidity sensor.
 */

#ifndef SHT31_H
#define SHT31_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Take a single-shot measurement from the SHT31.
 *
 * Temporarily locks the shared I2C_NUM_0 bus, issues the high repeatability
 * measurement command, waits for completion, and decodes the result.
 *
 * @param  out_temp      Pointer to store temperature in Celsius (can be NULL).
 * @param  out_humidity  Pointer to store relative humidity in % (can be NULL).
 *
 * @return ESP_OK on successful read and CRC check, or propagated I2C error.
 */
esp_err_t sht31_read_temp_and_humidity(float *out_temp, float *out_humidity);

#ifdef __cplusplus
}
#endif

#endif /* SHT31_H */
