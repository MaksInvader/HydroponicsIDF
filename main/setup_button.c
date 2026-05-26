/*
 * setup_button.c
 *
 * GPIO 46 — active-LOW setup / reconfiguration button.
 *
 * Behaviour
 * ─────────
 *   setup_button_wait_for_hold()  — called at boot when autostart did not
 *     succeed; blocks the calling task until the button is held for 3 s.
 *     PIN_LED_FAULT blinks every 500 ms while waiting.
 *
 *   setup_button_start_monitor()  — called after a successful autostart;
 *     spawns a lightweight background task that watches the button and
 *     triggers reconfiguration (stop runtime → start AP → start portal)
 *     when a 3-second hold is detected.
 */

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pin_config.h"
#include "runtime_tasks.h"
#include "web_portal.h"
#include "wifi_manager.h"

#define TAG "setup_btn"

/* Polling interval in ms — fine enough for a 3-second hold detection. */
#define POLL_MS          10

/* LED blink half-period while waiting for button press. */
#define BLINK_HALF_MS    500

/* Hold duration required to trigger setup mode (ms). */
#define HOLD_REQUIRED_MS 3000

/* -------------------------------------------------------------------------- */

void setup_button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_SETUP_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_LOGI(TAG, "Setup button initialised on GPIO%d", PIN_SETUP_BUTTON);
}

/* -------------------------------------------------------------------------- */

void setup_button_wait_for_hold(uint32_t hold_ms)
{
    ESP_LOGI(TAG, "Waiting for setup button hold (%lu ms)...", (unsigned long)hold_ms);

    uint32_t held_ticks  = 0;
    uint32_t blink_ticks = 0;
    int      led_state   = 0;

    /* Ensure fault LED starts LOW */
    gpio_set_direction(PIN_LED_FAULT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED_FAULT, 0);

    while (held_ticks < hold_ms) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        /* Blink LED every BLINK_HALF_MS */
        blink_ticks += POLL_MS;
        if (blink_ticks >= BLINK_HALF_MS) {
            blink_ticks = 0;
            led_state   = !led_state;
            gpio_set_level(PIN_LED_FAULT, led_state);
        }

        /* Button is active LOW */
        if (gpio_get_level(PIN_SETUP_BUTTON) == 0) {
            held_ticks += POLL_MS;
        } else {
            /* Released — reset hold counter but keep blinking */
            held_ticks = 0;
        }
    }

    /* Turn LED off when done */
    gpio_set_level(PIN_LED_FAULT, 0);
    ESP_LOGI(TAG, "Setup button hold detected — entering setup mode");
}

/* -------------------------------------------------------------------------- */

static void button_monitor_task(void *arg)
{
    (void)arg;

    uint32_t held_ticks = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        if (gpio_get_level(PIN_SETUP_BUTTON) == 0) {
            held_ticks += POLL_MS;
            if (held_ticks >= HOLD_REQUIRED_MS) {
                ESP_LOGI(TAG, "Button held — stopping runtime and entering setup mode");

                runtime_tasks_stop();

                esp_err_t ret = wifi_manager_start_ap("ESP32S3-Updater", NULL);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start setup AP: %s", esp_err_to_name(ret));
                }

                ret = web_portal_start();
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start web portal: %s", esp_err_to_name(ret));
                }

                /* Portal and restart-after-configure task take over from here */
                vTaskDelete(NULL);
                return;
            }
        } else {
            held_ticks = 0;
        }
    }
}

void setup_button_start_monitor(void)
{
    BaseType_t ret = xTaskCreate(button_monitor_task, "btn_monitor",
                                 2048, NULL, 3, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button monitor task");
    } else {
        ESP_LOGI(TAG, "Button monitor task started");
    }
}
