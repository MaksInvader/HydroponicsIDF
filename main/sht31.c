#include "sht31.h"
#include "i2c_bus.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "sht31";

#define SHT31_I2C_ADDR 0x44

esp_err_t sht31_read_temp_and_humidity(float *out_temp, float *out_humidity)
{
    esp_err_t ret;
    uint8_t cmd[2] = {0x24, 0x00}; // Single shot, high repeatability, no clock stretching

    /* 1. Send measurement command */
    ret = i2c_bus_lock(pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to lock I2C bus for write");
        return ret;
    }

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (SHT31_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(handle, cmd, sizeof(cmd), true);
    i2c_master_stop(handle);
    ret = i2c_master_cmd_begin(I2C_NUM_0, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);

    i2c_bus_unlock();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command to SHT31");
        return ret;
    }

    /* 2. Wait for measurement (SHT31 max time for high repeatability is 15ms) */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 3. Read results */
    ret = i2c_bus_lock(pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to lock I2C bus for read");
        return ret;
    }

    uint8_t data[6] = {0};
    handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (SHT31_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(handle, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(handle);
    ret = i2c_master_cmd_begin(I2C_NUM_0, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);

    i2c_bus_unlock();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from SHT31");
        return ret;
    }

    /* 4. Decode results */
    uint16_t t_raw = (data[0] << 8) | data[1];
    uint16_t h_raw = (data[3] << 8) | data[4];

    // TODO: Verify CRC8 (data[2] and data[5]) if robust error checking is needed.
    // For now, we trust the reading if I2C succeeded.

    if (out_temp) {
        *out_temp = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    }
    
    if (out_humidity) {
        *out_humidity = 100.0f * ((float)h_raw / 65535.0f);
    }

    return ESP_OK;
}
