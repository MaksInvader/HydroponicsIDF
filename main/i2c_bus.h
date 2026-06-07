/**
 * @file  i2c_bus.h
 * @brief Shared I2C bus mutex for I2C_NUM_0.
 *
 * Both lcd_status.c (PCF8574 LCD backpack) and sensor_telemetry.c (ADS1115)
 * share I2C_NUM_0 on SDA=GPIO16 / SCL=GPIO15.  The ESP-IDF legacy I2C driver
 * is NOT thread-safe, so every caller must hold this mutex for the full
 * duration of each I2C transaction (write, or write-then-read).
 *
 * Usage:
 *   i2c_bus_init();                          // once, at boot (idempotent)
 *
 *   esp_err_t ret = i2c_bus_lock(pdMS_TO_TICKS(500));
 *   if (ret != ESP_OK) { ... handle timeout ... }
 *   // ... i2c_master_* calls ...
 *   i2c_bus_unlock();
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Create the I2C bus mutex.
 *
 * Safe to call multiple times — subsequent calls are no-ops.
 * Must be called before the first i2c_bus_lock().
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM if mutex creation fails.
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief  Acquire exclusive access to I2C_NUM_0.
 *
 * @param  timeout_ticks  FreeRTOS tick count to wait.
 *                        Use portMAX_DELAY to wait forever.
 * @return ESP_OK          — mutex acquired, caller owns the bus.
 *         ESP_ERR_TIMEOUT — timed out, do NOT proceed with I2C.
 *         ESP_ERR_INVALID_STATE — i2c_bus_init() was never called.
 */
esp_err_t i2c_bus_lock(TickType_t timeout_ticks);

/**
 * @brief  Release exclusive access to I2C_NUM_0.
 *
 * Must only be called after a successful i2c_bus_lock().
 */
void i2c_bus_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_H */
