/**
 * @file  ph_serial.h
 * @brief UART serial pH module driver — fallback for ADS1115 AIN1.
 *
 * Only compiled when PH_SOURCE_USE_SERIAL == 1 in pin_config.h.
 *
 * Supports any serial pH module that streams frames in the form
 *   'P' 'H' 'A' [high byte] [low byte]
 * where the 16-bit payload is treated as a pH value scaled by 100.
 *
 * The module outputs a finished, calibrated pH value — no further
 * slope/offset calibration is applied by the firmware.
 * Range validation [0.0, 14.0] is still performed after decoding.
 */

#ifndef PH_SERIAL_H
#define PH_SERIAL_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the UART port for the serial pH module.
 *
 * Configures UART_NUM_1 with the baud rate, TX/RX pins defined in
 * pin_config.h (PIN_PH_SERIAL_TX / PIN_PH_SERIAL_RX / PIN_PH_SERIAL_BAUD).
 * Safe to call multiple times — subsequent calls are no-ops if already ready.
 *
 * @return ESP_OK on success, propagated UART driver error on failure.
 */
esp_err_t ph_serial_init(void);

/**
 * @brief  Tear down the UART driver for the serial pH module.
 *
 * Safe to call even if ph_serial_init() never succeeded.
 */
void ph_serial_deinit(void);

/**
 * @brief  Returns true if ph_serial_init() completed successfully.
 */
bool ph_serial_is_ready(void);

/**
 * @brief  Read a calibrated pH value from the serial module.
 *
 * Drains the RX buffer, advances the passive frame parser, and returns the
 * latest cached frame once a complete "PHA" packet has been observed.
 *
 * @param  out_raw  Decoded 16-bit frame payload.  Stored in the snapshot
 *                  ph_raw field for MQTT publishing.
 * @param  out_ph   Parsed, calibrated pH float [0.0 – 14.0].
 *
 * @return ESP_OK            — valid reading in *out_ph / *out_raw.
 *         ESP_ERR_TIMEOUT   — no response within timeout.
 *         ESP_FAIL          — response received but could not be parsed
 *                             or value is outside [0.0, 14.0].
 *         ESP_ERR_INVALID_STATE — ph_serial_init() was never called.
 */
esp_err_t ph_serial_read(int *out_raw);

#ifdef __cplusplus
}
#endif

#endif /* PH_SERIAL_H */
