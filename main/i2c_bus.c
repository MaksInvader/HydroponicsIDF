/**
 * @file  i2c_bus.c
 * @brief Shared I2C bus mutex for I2C_NUM_0.
 *
 * Owns a single FreeRTOS mutex that serialises all I2C_NUM_0 transactions
 * across tasks (SensorTask → ADS1115, CommTask/SafetyTask → LCD).
 */

#include "i2c_bus.h"

#include "esp_log.h"
#include "freertos/semphr.h"

static const char *TAG = "i2c_bus";

static SemaphoreHandle_t s_i2c0_mutex = NULL;

/* -------------------------------------------------------------------------- */

esp_err_t i2c_bus_init(void)
{
    if (s_i2c0_mutex != NULL) {
        return ESP_OK; /* idempotent */
    }

    s_i2c0_mutex = xSemaphoreCreateMutex();
    if (s_i2c0_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C bus mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "I2C_NUM_0 bus mutex created");
    return ESP_OK;
}

esp_err_t i2c_bus_lock(TickType_t timeout_ticks)
{
    if (s_i2c0_mutex == NULL) {
        ESP_LOGE(TAG, "i2c_bus_lock called before i2c_bus_init");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_i2c0_mutex, timeout_ticks) != pdTRUE) {
        ESP_LOGW(TAG, "I2C bus lock timeout");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void i2c_bus_unlock(void)
{
    if (s_i2c0_mutex != NULL) {
        xSemaphoreGive(s_i2c0_mutex);
    }
}
