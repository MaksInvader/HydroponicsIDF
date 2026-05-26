#pragma once

#include <stdint.h>

/**
 * @brief  Initialise GPIO 46 as an active-LOW input with internal pull-up.
 *
 * Must be called once from app_main before any other setup_button function.
 */
void setup_button_init(void);

/**
 * @brief  Block until GPIO 46 is held LOW continuously for @p hold_ms.
 *
 * While waiting, PIN_LED_FAULT is blinked every 500 ms to signal that the
 * device is in "waiting for setup button" state.  The LED is left LOW when
 * this function returns.
 *
 * @param hold_ms  Required continuous hold duration in milliseconds.
 */
void setup_button_wait_for_hold(uint32_t hold_ms);

/**
 * @brief  Spawn a background FreeRTOS task that monitors GPIO 46.
 *
 * When a 3-second hold is detected the task will:
 *   1. Stop the runtime (runtime_tasks_stop).
 *   2. Start the setup AP  (wifi_manager_start_ap).
 *   3. Start the web portal (web_portal_start).
 *   4. Delete itself.
 *
 * Call this after a successful autostart so the user can reconfigure at any
 * time without power-cycling the device.
 */
void setup_button_start_monitor(void);
