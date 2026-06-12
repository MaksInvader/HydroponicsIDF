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
#include "pin_config.h"
#include "i2c_bus.h"

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

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

/* Forward declaration */
static void lcd_stop_scroll(void);

static esp_err_t lcd_i2c_write_byte(uint8_t data)
{
    esp_err_t lock_ret = i2c_bus_lock(pdMS_TO_TICKS(500));
    if (lock_ret != ESP_OK) {
        return lock_ret;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((s_lcd_addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(PIN_LCD_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    i2c_bus_unlock();
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
    #if !ENABLE_LCD
        ESP_LOGI(TAG, "LCD disabled (ENABLE_LCD=0)");
        return ESP_OK;
    #endif

    if (s_lcd_ready) {
        return ESP_OK;
    }

    if (s_lcd_lock == NULL) {
        s_lcd_lock = xSemaphoreCreateMutex();
        if (s_lcd_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    /* Initialise the shared I2C bus mutex (idempotent — safe if ADS1115 inits first) */
    esp_err_t bus_ret = i2c_bus_init();
    if (bus_ret != ESP_OK) {
        return bus_ret;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_LCD_I2C_SDA,
        .scl_io_num = PIN_LCD_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = PIN_LCD_I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t ret = i2c_param_config(PIN_LCD_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(PIN_LCD_I2C_PORT, conf.mode, 0, 0, 0);
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

    /* Initialize LCD with error checking instead of ESP_ERROR_CHECK to avoid crashes */
    ret = lcd_command(0x28);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD command 0x28 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = lcd_command(0x08);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD command 0x08 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = lcd_command(0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD command 0x01 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    ret = lcd_command(0x06);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD command 0x06 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = lcd_command(0x0C);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD command 0x0C failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_lcd_ready = true;
    ESP_LOGI(TAG, "LCD initialized at I2C address 0x%02X", s_lcd_addr);
    return ESP_OK;
}

void lcd_status_show_wifi_and_broker(const char *ssid, const char *broker_ip)
{
    if (!s_lcd_ready || ssid == NULL || broker_ip == NULL) {
        return;
    }
    lcd_stop_scroll();
    char line1[LCD_COLS + 1];
    char line2[LCD_COLS + 1];
    snprintf(line1, sizeof(line1), "%-*.*s", LCD_COLS, LCD_COLS, ssid);
    snprintf(line2, sizeof(line2), "%-*.*s", LCD_COLS, LCD_COLS, broker_ip);
    lcd_show_lines("** CONNECTING **    ", line1, line2, "  Please wait...    ");
}

void lcd_status_show_mqtt_connected(void)
{
    if (!s_lcd_ready) return;
    lcd_stop_scroll();
    lcd_show_lines("====================",
                   "   MQTT CONNECTED   ",
                   "   System Online    ",
                   "====================");
}

void lcd_status_show_zone_overview(const char *zone_id, const char *zone_name, const char *last_actuator)
{
    if (!s_lcd_ready || zone_id == NULL || zone_name == NULL) return;
    lcd_stop_scroll();
    char line1[LCD_COLS + 1];
    char line2[LCD_COLS + 1];
    char line3[LCD_COLS + 1];
    snprintf(line1, sizeof(line1), "ID: %-*.*s", LCD_COLS - 4, LCD_COLS - 4, zone_id);
    snprintf(line2, sizeof(line2), "%-*.*s", LCD_COLS, LCD_COLS, zone_name);
    snprintf(line3, sizeof(line3), ">%-*.*s", LCD_COLS - 1, LCD_COLS - 1,
             (last_actuator != NULL && last_actuator[0] != '\0') ? last_actuator : "None");
    lcd_show_lines("---- ZONE ACTIVE ---", line1, line2, line3);
}

void lcd_status_show_actuator_event(const char *zone_id, const char *zone_name, const char *actuator_name, const char *sensor_text, const char *state_text)
{
    if (!s_lcd_ready || zone_id == NULL || zone_name == NULL || actuator_name == NULL) return;
    lcd_stop_scroll();
    char line1[LCD_COLS + 1];
    char line2[LCD_COLS + 1];
    char line3[LCD_COLS + 1];
    snprintf(line1, sizeof(line1), "%-*.*s", LCD_COLS, LCD_COLS, zone_name);
    snprintf(line2, sizeof(line2), "%-*.*s", LCD_COLS, LCD_COLS,
             (sensor_text != NULL) ? sensor_text : "");
    snprintf(line3, sizeof(line3), "[%-.*s] %s", LCD_COLS - 4, actuator_name,
             (state_text != NULL) ? state_text : "?");
    lcd_show_lines("*** ACTUATOR EVENT *", line1, line2, line3);
}

/* --------------------------------------------------------------------------
 * Scroll task — used by lcd_status_show_fault()
 * -------------------------------------------------------------------------- */

#define SCROLL_REASON_MAX   256
#define SCROLL_STEP_MS      300
#define SCROLL_GAP          4
#define SCROLL_TASK_STACK   2048
#define SCROLL_TASK_PRIO    1

static TaskHandle_t s_scroll_task   = NULL;
static char         s_scroll_reason[SCROLL_REASON_MAX];
static char         s_scroll_code[32];

static void scroll_fill_line(char *out, size_t out_len,
                              const char *padded, size_t padded_len,
                              size_t offset)
{
    for (size_t i = 0; i < out_len - 1; i++) {
        out[i] = padded[(offset + i) % padded_len];
    }
    out[out_len - 1] = '\0';
}

static void lcd_scroll_task_fn(void *arg)
{
    (void)arg;

    while (true) {
        /* Snapshot the scroll buffers under the lock so an in-place update
         * from lcd_status_show_fault() is always picked up atomically. */
        char code_snap[32];
        char reason_snap[SCROLL_REASON_MAX];
        if (xSemaphoreTake(s_lcd_lock, pdMS_TO_TICKS(300)) == pdTRUE) {
            memcpy(code_snap,   s_scroll_code,   sizeof(code_snap));
            memcpy(reason_snap, s_scroll_reason, sizeof(reason_snap));
            xSemaphoreGive(s_lcd_lock);
        } else {
            /* Couldn't get lock — skip this cycle rather than showing stale text */
            vTaskDelay(pdMS_TO_TICKS(SCROLL_STEP_MS));
            continue;
        }

        /* Rebuild the padded scroll string from the freshly-snapshotted reason */
        char padded[SCROLL_REASON_MAX + SCROLL_GAP + 1];
        size_t reason_len = strnlen(reason_snap, sizeof(reason_snap));
        memcpy(padded, reason_snap, reason_len);
        for (int i = 0; i < SCROLL_GAP; i++) padded[reason_len + i] = ' ';
        size_t padded_len = reason_len + SCROLL_GAP;
        padded[padded_len] = '\0';

        bool needs_scroll = (reason_len > (size_t)LCD_COLS);
        static size_t offset = 0;   /* persist scroll position across refreshes */

        if (xSemaphoreTake(s_lcd_lock, pdMS_TO_TICKS(300)) == pdTRUE) {
            char code_line[LCD_COLS + 1];
            snprintf(code_line, sizeof(code_line), "%-.*s", LCD_COLS, code_snap);

            if (needs_scroll) {
                char line2[LCD_COLS + 1];
                char line3[LCD_COLS + 1];
                scroll_fill_line(line2, sizeof(line2), padded, padded_len, offset);
                scroll_fill_line(line3, sizeof(line3), padded, padded_len,
                                 (offset + LCD_COLS) % padded_len);
                lcd_write_line(0, "!!!!!  FAULT  !!!!!");
                lcd_write_line(1, code_line);
                lcd_write_line(2, line2);
                lcd_write_line(3, line3);
            } else {
                char line2[LCD_COLS + 1];
                snprintf(line2, sizeof(line2), "%-.*s", LCD_COLS, reason_snap);
                lcd_write_line(0, "!!!!!  FAULT  !!!!!");
                lcd_write_line(1, code_line);
                lcd_write_line(2, line2);
                lcd_write_line(3, "");
            }
            xSemaphoreGive(s_lcd_lock);
        }

        if (needs_scroll) {
            offset = (offset + 1) % padded_len;
        }
        vTaskDelay(pdMS_TO_TICKS(SCROLL_STEP_MS));
    }
}

static void lcd_stop_scroll(void)
{
    if (s_scroll_task != NULL) {
        vTaskDelete(s_scroll_task);
        s_scroll_task = NULL;
    }
}

/* --------------------------------------------------------------------------
 * New public display functions
 * -------------------------------------------------------------------------- */

void lcd_status_show_booting(void)
{
    if (!s_lcd_ready) return;
    lcd_stop_scroll();
    lcd_show_lines("====================",
                   "    SYSTEM BOOT     ",
                   "   Please wait...   ",
                   "====================");
}

void lcd_status_show_setup(const char *ip)
{
    if (!s_lcd_ready) return;
    lcd_stop_scroll();
    char line2[LCD_COLS + 1];
    snprintf(line2, sizeof(line2), "%-*.*s", LCD_COLS, LCD_COLS, (ip != NULL) ? ip : "");
    lcd_show_lines("-- SETUP MODE ------",
                   "Connect WiFi then:  ",
                   line2,
                   "to configure        ");
}

void lcd_status_show_fault(const char *fault_code, const char *reason)
{
    if (!s_lcd_ready) return;

    /* Update the scroll buffers under the LCD lock so the scroll task
     * (if already running) picks up the new text on its next refresh cycle.
     * This means a second fault always displaces the first on-screen. */
    if (s_lcd_lock != NULL) {
        if (xSemaphoreTake(s_lcd_lock, pdMS_TO_TICKS(500)) == pdTRUE) {
            snprintf(s_scroll_code,   sizeof(s_scroll_code),   "%s",
                     (fault_code != NULL) ? fault_code : "FAULT");
            snprintf(s_scroll_reason, sizeof(s_scroll_reason), "%s",
                     (reason != NULL && reason[0] != '\0') ? reason : "Unknown reason");
            xSemaphoreGive(s_lcd_lock);
        }
    } else {
        snprintf(s_scroll_code,   sizeof(s_scroll_code),   "%s",
                 (fault_code != NULL) ? fault_code : "FAULT");
        snprintf(s_scroll_reason, sizeof(s_scroll_reason), "%s",
                 (reason != NULL && reason[0] != '\0') ? reason : "Unknown reason");
    }

    /* Only create the scroll task if one isn't already running.
     * If it's already running, updating the buffers above is enough. */
    if (s_scroll_task == NULL) {
        xTaskCreate(lcd_scroll_task_fn, "lcd_scroll", SCROLL_TASK_STACK,
                    NULL, SCROLL_TASK_PRIO, &s_scroll_task);
    }
}

void lcd_status_show_emergency(void)
{
    if (!s_lcd_ready) return;
    lcd_stop_scroll();
    lcd_show_lines("!! EMERGENCY STOP !!",
                   "!!!!!!!!!!!!!!!!!!!!",
                   "!! EMERGENCY STOP !!",
                   "!!!!!!!!!!!!!!!!!!!!");
}