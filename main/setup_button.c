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

/* Number of consecutive LOW samples required before the hold timer starts.
 * Glitches shorter than DEBOUNCE_COUNT * POLL_MS (= 50 ms) are ignored.
 * Any HIGH sample resets this counter, preventing non-consecutive LOW
 * readings from accumulating toward the hold threshold. */
#define DEBOUNCE_COUNT   5

/* -------------------------------------------------------------------------- */

void setup_button_init(void)
{
if (s_setup_cfg.enable_setup_button) { // REPLACED_MACRO
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_SETUP_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_LOGI(TAG, "Setup button initialised on GPIO%d", PIN_SETUP_BUTTON);
#else
    ESP_LOGI(TAG, "Setup button disabled via pin_config.h");
} // END REPLACED_MACRO
}

/* -------------------------------------------------------------------------- */

void setup_button_wait_for_hold(uint32_t hold_ms)
{
#if !ENABLE_SETUP_BUTTON
    ESP_LOGI(TAG, "Setup button disabled — skipping hold wait");
    return;
#endif

    ESP_LOGI(TAG, "Waiting for setup button hold (%lu ms)...", (unsigned long)hold_ms);

    uint32_t held_ticks    = 0;
    uint32_t blink_ticks   = 0;
    uint32_t debounce_count = 0;   /* consecutive LOW sample counter */
    int      led_state     = 0;

    /* Ensure fault LED starts LOW */
if (s_setup_cfg.enable_indicator_leds) { // REPLACED_MACRO
    gpio_set_direction(PIN_LED_FAULT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED_FAULT, 0);
} // END REPLACED_MACRO

    while (held_ticks < hold_ms) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        /* Blink LED every BLINK_HALF_MS */
        blink_ticks += POLL_MS;
        if (blink_ticks >= BLINK_HALF_MS) {
            blink_ticks = 0;
            led_state   = !led_state;
if (s_setup_cfg.enable_indicator_leds) { // REPLACED_MACRO
            gpio_set_level(PIN_LED_FAULT, led_state);
} // END REPLACED_MACRO
        }

        /* Debounce + hold logic:
         *   - Any HIGH sample resets BOTH counters (glitch immunity).
         *   - held_ticks only starts counting once DEBOUNCE_COUNT
         *     consecutive LOW samples confirm a real press. */
        if (gpio_get_level(PIN_SETUP_BUTTON) == 0) {
            if (debounce_count < DEBOUNCE_COUNT) {
                debounce_count++;
                /* Don't count hold time until pin is debounced */
            } else {
                held_ticks += POLL_MS;
            }
        } else {
            /* Released — reset both counters */
            debounce_count = 0;
            held_ticks     = 0;
        }
    }

    /* Turn LED off when done */
if (s_setup_cfg.enable_indicator_leds) { // REPLACED_MACRO
    gpio_set_level(PIN_LED_FAULT, 0);
} // END REPLACED_MACRO
    ESP_LOGI(TAG, "Setup button hold detected — entering setup mode");
}

/* -------------------------------------------------------------------------- */

static void button_monitor_task(void *arg)
{
    (void)arg;

    /* Delay 30 seconds after boot to avoid false triggers during WiFi initialization */
    ESP_LOGI(TAG, "Button monitor starting — 30s boot delay...");
    vTaskDelay(pdMS_TO_TICKS(30000));
    ESP_LOGI(TAG, "Button monitor active");

    uint32_t held_ticks    = 0;
    uint32_t debounce_count = 0;   /* consecutive LOW sample counter */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        /* Debounce + hold logic:
         *   - Any HIGH sample resets BOTH counters.
         *   - held_ticks only accumulates after DEBOUNCE_COUNT consecutive
         *     LOW samples — isolated glitches can never reach HOLD_REQUIRED_MS. */
        if (gpio_get_level(PIN_SETUP_BUTTON) == 0) {
            if (debounce_count < DEBOUNCE_COUNT) {
                debounce_count++;
            } else {
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
            }
        } else {
            /* Released — reset both counters */
            debounce_count = 0;
            held_ticks     = 0;
        }
    }
}

void setup_button_start_monitor(void)
{
if (s_setup_cfg.enable_setup_button) { // REPLACED_MACRO
    BaseType_t ret = xTaskCreate(
        button_monitor_task,
        "setup_btn_mon",
        2048,
        NULL,
        1,
        NULL
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button monitor task");
    } else {
        ESP_LOGI(TAG, "Button monitor task started");
    }
} // END REPLACED_MACRO
}
