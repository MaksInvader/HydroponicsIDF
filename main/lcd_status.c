#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lcd_status.h"

#define LCD_I2C_PORT       I2C_NUM_0
#define LCD_I2C_SDA_GPIO   15
#define LCD_I2C_SCL_GPIO   16
#define LCD_I2C_FREQ_HZ    100000

#define LCD_COLS 20
#define LCD_ROWS 4

/* PCF8574 bit mapping for common LCD backpacks */
#define LCD_BIT_RS        0x01
#define LCD_BIT_RW        0x02
#define LCD_BIT_EN        0x04
#define LCD_BIT_BACKLIGHT 0x08

static const char *TAG = "lcd_status";

static uint8_t s_lcd_addr;
static bool s_lcd_ready;
static SemaphoreHandle_t s_lcd_lock;

static esp_err_t lcd_i2c_write_byte(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((s_lcd_addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(LCD_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t lcd_write_nibble(uint8_t nibble, bool rs)
{
    uint8_t base = (uint8_t)((nibble & 0xF0) | LCD_BIT_BACKLIGHT | (rs ? LCD_BIT_RS : 0) | LCD_BIT_RW * 0);
    esp_err_t ret = lcd_i2c_write_byte((uint8_t)(base | LCD_BIT_EN));
    if (ret != ESP_OK) {
        return ret;
    }
    esp_rom_delay_us(1);
    ret = lcd_i2c_write_byte(base);
    if (ret != ESP_OK) {
        return ret;
    }
    esp_rom_delay_us(50);
    return ESP_OK;
}

static esp_err_t lcd_send_byte(uint8_t value, bool rs)
{
    esp_err_t ret = lcd_write_nibble((uint8_t)(value & 0xF0), rs);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = lcd_write_nibble((uint8_t)((value << 4) & 0xF0), rs);
    return ret;
}

static esp_err_t lcd_command(uint8_t cmd)
{
    return lcd_send_byte(cmd, false);
}

static esp_err_t lcd_data(uint8_t data)
{
    return lcd_send_byte(data, true);
}

static esp_err_t lcd_set_cursor(uint8_t row, uint8_t col)
{
    static const uint8_t row_offsets[LCD_ROWS] = {0x00, 0x40, 0x14, 0x54};
    if (row >= LCD_ROWS || col >= LCD_COLS) {
        return ESP_ERR_INVALID_ARG;
    }
    return lcd_command((uint8_t)(0x80 | (row_offsets[row] + col)));
}

static esp_err_t lcd_write_line(uint8_t row, const char *text)
{
    char buf[LCD_COLS + 1];
    size_t len = strlen(text);
    if (len > LCD_COLS) {
        len = LCD_COLS;
    }

    memset(buf, ' ', LCD_COLS);
    memcpy(buf, text, len);
    buf[LCD_COLS] = '\0';

    esp_err_t ret = lcd_set_cursor(row, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    for (int i = 0; i < LCD_COLS; i++) {
        ret = lcd_data((uint8_t)buf[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t lcd_probe_address(uint8_t addr)
{
    s_lcd_addr = addr;
    return lcd_i2c_write_byte(LCD_BIT_BACKLIGHT);
}

static void lcd_show_lines(const char *line0, const char *line1, const char *line2, const char *line3)
{
    if (!s_lcd_ready) {
        return;
    }

    if (s_lcd_lock != NULL) {
        if (xSemaphoreTake(s_lcd_lock, pdMS_TO_TICKS(250)) != pdTRUE) {
            return;
        }
    }

    lcd_write_line(0, (line0 != NULL) ? line0 : "");
    lcd_write_line(1, (line1 != NULL) ? line1 : "");
    lcd_write_line(2, (line2 != NULL) ? line2 : "");
    lcd_write_line(3, (line3 != NULL) ? line3 : "");

    if (s_lcd_lock != NULL) {
        xSemaphoreGive(s_lcd_lock);
    }
}

esp_err_t lcd_status_init(void)
{
    if (s_lcd_ready) {
        return ESP_OK;
    }

    if (s_lcd_lock == NULL) {
        s_lcd_lock = xSemaphoreCreateMutex();
        if (s_lcd_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LCD_I2C_SDA_GPIO,
        .scl_io_num = LCD_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(LCD_I2C_PORT, &conf));

    esp_err_t ret = i2c_driver_install(LCD_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = lcd_probe_address(0x27);
    if (ret != ESP_OK) {
        ret = lcd_probe_address(0x3F);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "LCD not found at 0x27 or 0x3F");
            return ret;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x30, false);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30, false);
    esp_rom_delay_us(150);
    lcd_write_nibble(0x30, false);
    lcd_write_nibble(0x20, false);

    ESP_ERROR_CHECK(lcd_command(0x28));
    ESP_ERROR_CHECK(lcd_command(0x08));
    ESP_ERROR_CHECK(lcd_command(0x01));
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_ERROR_CHECK(lcd_command(0x06));
    ESP_ERROR_CHECK(lcd_command(0x0C));

    s_lcd_ready = true;
    ESP_LOGI(TAG, "LCD initialized at I2C address 0x%02X", s_lcd_addr);
    return ESP_OK;
}

void lcd_status_show_wifi_and_broker(const char *ssid, const char *broker_ip)
{
    if (!s_lcd_ready || ssid == NULL || broker_ip == NULL) {
        return;
    }

    char line1[LCD_COLS + 1];
    char line2[LCD_COLS + 1];

    snprintf(line1, sizeof(line1), "SSID: %s", ssid);
    snprintf(line2, sizeof(line2), "Broker: %s", broker_ip);
    lcd_show_lines("WiFi + MQTT Setup", line1, line2, "Waiting MQTT...");
}

void lcd_status_show_mqtt_connected(void)
{
    if (!s_lcd_ready) {
        return;
    }

    lcd_show_lines("", "", "", "MQTT Connected");
}

void lcd_status_show_zone_overview(const char *zone_id, const char *zone_name, const char *last_actuator)
{
    if (!s_lcd_ready || zone_id == NULL || zone_name == NULL) {
        return;
    }

    char line0[LCD_COLS + 1];
    char line1[LCD_COLS + 1];
    char line2[LCD_COLS + 1];
    char line3[LCD_COLS + 1];

    snprintf(line0, sizeof(line0), "zone_id: %s", zone_id);
    snprintf(line1, sizeof(line1), "name: %s", zone_name);
    snprintf(line2, sizeof(line2), "Last Activate:");
    snprintf(line3, sizeof(line3), "%s", (last_actuator != NULL && last_actuator[0] != '\0') ? last_actuator : "None");

    lcd_show_lines(line0, line1, line2, line3);
}

void lcd_status_show_actuator_event(const char *zone_id, const char *zone_name, const char *actuator_name, const char *sensor_text, const char *state_text)
{
    if (!s_lcd_ready || zone_id == NULL || zone_name == NULL || actuator_name == NULL) {
        return;
    }

    char line0[LCD_COLS + 1];
    char line1[LCD_COLS + 1];
    char line2[LCD_COLS + 1];
    char line3[LCD_COLS + 1];

    snprintf(line0, sizeof(line0), "zone_id: %s", zone_id);
    snprintf(line1, sizeof(line1), "name: %s", zone_name);
    snprintf(line2, sizeof(line2), "%s", (sensor_text != NULL) ? sensor_text : "Sensor: N/A");
    snprintf(line3, sizeof(line3), "%s: %s", actuator_name, (state_text != NULL) ? state_text : "N/A");

    lcd_show_lines(line0, line1, line2, line3);
}