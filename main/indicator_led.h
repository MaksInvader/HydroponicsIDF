#ifndef INDICATOR_LED_H
#define INDICATOR_LED_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the connection LED and fault LED GPIOs.
 *        Both pins are driven LOW on init.
 *        Call once before using any other function in this module.
 */
esp_err_t indicator_led_init(void);

/**
 * @brief Set the connection LED state.
 *        HIGH = broker connected, LOW = disconnected.
 */
void indicator_led_set_connection(bool connected);

/**
 * @brief Turn the fault/emergency LED on for @p duration_ms milliseconds,
 *        then turn it off.  Runs in the calling task context (blocking).
 *        Safe to call from any normal task.
 */
void indicator_led_fault_blink(uint32_t duration_ms);

/**
 * @brief Force the fault LED on or off immediately (no timeout).
 */
void indicator_led_set_fault(bool on);

#ifdef __cplusplus
}
#endif

#endif /* INDICATOR_LED_H */
