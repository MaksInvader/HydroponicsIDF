#include "indicator_led.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pin_config.h"

#define FAULT_LED_DEFAULT_MS 5000U

static const char *TAG = "indicator_led";
static bool s_initialized = false;

esp_err_t indicator_led_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_CONNECTION) | (1ULL << PIN_LED_FAULT),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level((gpio_num_t)PIN_LED_CONNECTION, 0);
    gpio_set_level((gpio_num_t)PIN_LED_FAULT, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "Indicator LEDs initialised (conn=GPIO%d, fault=GPIO%d)",
             PIN_LED_CONNECTION, PIN_LED_FAULT);
    return ESP_OK;
}

void indicator_led_set_connection(bool connected)
{
    if (!s_initialized) {
        return;
    }
    gpio_set_level((gpio_num_t)PIN_LED_CONNECTION, connected ? 1 : 0);
}

void indicator_led_fault_blink(uint32_t duration_ms)
{
    if (!s_initialized) {
        return;
    }
    if (duration_ms == 0) {
        duration_ms = FAULT_LED_DEFAULT_MS;
    }
    gpio_set_level((gpio_num_t)PIN_LED_FAULT, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level((gpio_num_t)PIN_LED_FAULT, 0);
}

void indicator_led_set_fault(bool on)
{
    if (!s_initialized) {
        return;
    }
    gpio_set_level((gpio_num_t)PIN_LED_FAULT, on ? 1 : 0);
}
