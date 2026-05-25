#ifndef INDICATOR_LED_H
#define INDICATOR_LED_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the connection LED and fault LED GPIOs and start the
 *        background blink task.  Both pins are driven LOW on init.
 *        Call once before using any other function in this module.
 */
esp_err_t indicator_led_init(void);

/**
 * @brief Set the connection LED state immediately (non-blocking).
 *        HIGH = broker connected, LOW = disconnected.
 */
void indicator_led_set_connection(bool connected);

/**
 * @brief Request a fault LED blink of @p duration_ms milliseconds.
 *        Non-blocking — the blink is handled by a background task.
 *        Safe to call from any task including SafetyTask.
 *        If a blink is already in progress the request is queued;
 *        if the queue is full the request is silently dropped.
 */
void indicator_led_fault_blink(uint32_t duration_ms);

/**
 * @brief Force the fault LED on or off immediately (non-blocking).
 *        Cancels any pending blink request.
 */
void indicator_led_set_fault(bool on);

#ifdef __cplusplus
}
#endif

#endif /* INDICATOR_LED_H */
