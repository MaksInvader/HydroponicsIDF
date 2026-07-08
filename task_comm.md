# Comm Task Documentation

This document provides a comprehensive technical explanation of the `comm_task` — the FreeRTOS task responsible for all outbound communication (MQTT sensor publishing), LCD display updates, and bridging external server commands to the internal dosing system.

**Source file:** `main/runtime_tasks.c`  
**Task priority:** `2` (same as `sensor_task`, below dosing priority 3 and safety priority 4)  
**Stack size:** `8192` bytes  
**Interval:** every `3000 ms` (`COMM_INTERVAL_MS`)

---

## Architecture Overview

The `comm_task` acts as the system's **external interface agent**. It performs three roles each cycle:

```
Every 3 seconds:
    │
    ├─ 1. Version publish (every 60s)
    │         └─► MQTT: <zone_id>/version → "v1.2.3"
    │
    ├─ 2. Snapshot acquisition
    │         ├─ Try fresh snapshot → sensor_telemetry_get_snapshot()
    │         └─ Fallback to cached snapshot (mutex-protected)
    │
    ├─ 3. Sensor data publishing (MQTT)
    │         ├─► water_level, water_temp
    │         ├─► room_temp, humidity, vpd (if SHT31 enabled)
    │         ├─► ph_raw, ph, ph_valid
    │         └─► tds_raw, tds, tds_valid
    │
    └─ 4. LCD actuator event drain
              └─ xQueueReceive(s_lcd_event_queue) → lcd_status_show_actuator_event()
```

Inbound commands from the server travel the reverse path:
```
MQTT broker → mqtt_manager (callback thread) → on_*_command() → enqueue_command() → s_dosing_queue
```
These callbacks are registered during `runtime_tasks_start()`, not inside `comm_task` itself.

---

## 1. Task Initialization

```c
static void comm_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_version_publish = last_wake;

    runtime_safety_wdt_register();

    UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Comm task stack watermark: %u bytes free",
             (unsigned)(stack_hwm * sizeof(StackType_t)));
```

- **`last_wake`** — Captured at startup; passed to `vTaskDelayUntil()` at the end of each cycle to implement jitter-free 3-second periodic execution.

- **`last_version_publish`** — Separate timestamp tracking when the last version string was published to MQTT. Because version publishing is much less frequent than sensor publishing (`VERSION_PUBLISH_INTERVAL_MS` = 60,000ms), it needs its own independent timer.

- **`runtime_safety_wdt_register()`** — Registers with the hardware Task Watchdog Timer. The comm task must kick the WDT at the end of every iteration.

---

## 2. Version Publishing (Periodic, 60-second interval)

```c
        if (ticks_to_ms(now - last_version_publish) >= VERSION_PUBLISH_INTERVAL_MS) {
            publish_current_version(s_zone_id);
            last_version_publish = now;
        }
```

```c
static void publish_current_version(const char *zone_id)
{
    (void)zone_id;
    const esp_app_desc_t *app_desc = esp_app_get_description();
    const char *ver = (app_desc != NULL && app_desc->version[0] != '\0')
                      ? app_desc->version : "unknown";
    mqtt_manager_publish(MQTT_TOPIC_VERSION, ver, 0, /*retain=*/1);
}
```

- **`ticks_to_ms(now - last_version_publish)`** — Computes elapsed milliseconds using FreeRTOS tick arithmetic. This works correctly even across tick counter rollovers (overflow) because the subtraction is done in unsigned arithmetic.

- **`esp_app_get_description()`** — ESP-IDF API that returns a pointer to the `esp_app_desc_t` struct embedded in the firmware binary. This struct contains the version string set at compile time from `version.txt`.

- **`retain=1`** — The MQTT retain flag ensures that any new client subscribing to the version topic immediately receives the current version string without waiting for the next publish cycle.

---

## 3. Snapshot Acquisition

### Fresh Snapshot Path

```c
        if (sensor_telemetry_get_snapshot(&snap) == ESP_OK && snap.valid) {
            if (s_snapshot_mutex != NULL &&
                xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS)) == pdTRUE) {
                s_last_good_snapshot = snap;
                s_last_good_snapshot_valid = true;
                xSemaphoreGive(s_snapshot_mutex);
            } else if (s_snapshot_mutex != NULL) {
                ESP_LOGD(TAG, "Snapshot mutex busy, skipping cache update");
            }
            snapshot_available = true;
        }
```

- **`sensor_telemetry_get_snapshot(&snap)`** — Thread-safe read of the latest sensor snapshot. This function internally uses an atomic read or mutex-protected copy. Passing `&snap` (address) allows the function to populate the local struct without a return-by-value copy of the entire struct.

- **`snap.valid`** — A field within the snapshot that is `false` if no successful sensor read has ever completed. On the first boot before the sensor task has run, this prevents publishing garbage values.

- **`xSemaphoreTake(s_snapshot_mutex, timeout)`** — Attempts to acquire the mutex protecting `s_last_good_snapshot`. The timeout (`SNAPSHOT_MUTEX_WAIT_MS` = 10ms) prevents the comm task from being blocked indefinitely if the mutex is held. If the mutex cannot be acquired within 10ms, the cache update is skipped — a non-critical trade-off because the fresh data still exists in the local `snap` variable.

- **`xSemaphoreGive(s_snapshot_mutex)`** — Releases the mutex. Every `Take` must have a corresponding `Give`. Failing to `Give` would permanently block any other task trying to access the cache.

### Fallback Snapshot Path

```c
        } else {
            if (s_snapshot_mutex != NULL &&
                xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS)) == pdTRUE) {
                if (s_last_good_snapshot_valid) {
                    snap = s_last_good_snapshot;
                    snapshot_available = true;
                    ESP_LOGD(TAG, "Using last known-good sensor snapshot");
                }
                xSemaphoreGive(s_snapshot_mutex);
            }
        }
```

If `sensor_telemetry_get_snapshot()` fails (e.g., I2C bus error, sensor task not yet run), the comm task attempts to use the last known-good snapshot from the global cache. This provides resilience against transient sensor dropout:
- The dashboard continues receiving the last valid reading rather than seeing a gap.
- The safety task is unaffected because it has its own snapshot path.

The `s_last_good_snapshot` cache is written by `comm_task` and read by MQTT web handlers (e.g., the REST endpoint that serves the setup portal status page), which is why mutex protection is necessary.

---

## 4. Sensor Data Publishing

```c
static void comm_task_publish_sensors(const sensor_telemetry_snapshot_t *snap,
                                      const char *zone_id)
{
    char buf[32];

    // Water level (integer 0/1)
    snprintf(buf, sizeof(buf), "%d", snap->water_level);
    mqtt_manager_publish(sensor_telemetry_topic_water_level(), buf, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // pH (only if calibration valid)
    if (snap->ph_valid) {
        snprintf(buf, sizeof(buf), "%.2f", (double)snap->ph);
        mqtt_manager_publish(sensor_telemetry_topic_ph(), buf, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // pH validity flag (always published, with retain=1)
    mqtt_manager_publish(sensor_telemetry_topic_ph_valid(),
                         snap->ph_valid ? "true" : "false", 0, /*retain=*/1);
    // ...
}
```

- **`snprintf(buf, sizeof(buf), ...)`** — Safe string formatting into a fixed-size stack buffer. The `sizeof(buf)` bound prevents buffer overflows even if the format produces more characters than expected.

- **`(double)snap->ph`** — The `ph` field is stored as `float`, but `printf`-family functions on embedded GCC may handle `float` and `double` `%f` format differently. Explicit cast to `double` ensures the format works correctly with `%.2f`.

- **`vTaskDelay(pdMS_TO_TICKS(10))`** — A 10ms yield inserted between each MQTT publish. This prevents the MQTT library's internal buffer from being flooded with 10+ messages at once, which could cause some to be dropped. It also gives higher-priority tasks (dosing, safety) a chance to run.

- **Conditional publish for ph/tds** — If `snap->ph_valid` is `false` (no calibration loaded or sensor failed), the actual pH value is suppressed. Only the raw ADC value and the `ph_valid` flag are sent. This prevents the Node-RED dashboard from using a meaningless uncalibrated number for control decisions.

- **Raw values always published** — `ph_raw` (unsigned integer ADC count) and `tds_raw` are always published regardless of calibration validity. This allows the web portal calibration page to show live ADC values for the user to perform calibration.

- **`retain=1` on validity flags** — The `ph_valid` and `tds_valid` flags use MQTT retain so a dashboard opening mid-session immediately knows the calibration state.

All sensor topics follow the pattern:
| Field | Topic |
|---|---|
| Water level | `<zone_id>/sensor/water_level` |
| Water temperature | `<zone_id>/sensor/water_temp` |
| Room temperature | `<zone_id>/sensor/room_temp` |
| Humidity | `<zone_id>/sensor/humidity` |
| VPD | `<zone_id>/sensor/vpd` |
| pH (calibrated) | `<zone_id>/sensor/ph` |
| pH raw ADC | `<zone_id>/sensor/ph_raw` |
| pH valid flag | `<zone_id>/sensor/ph_valid` |
| TDS (calibrated) | `<zone_id>/sensor/tds` |
| TDS raw ADC | `<zone_id>/sensor/tds_raw` |
| TDS valid flag | `<zone_id>/sensor/tds_valid` |

---

## 5. LCD Actuator Event Drain

```c
if (s_setup_cfg.enable_actuators && s_setup_cfg.enable_lcd) {
        lcd_actuator_event_t ev;
        while (xQueueReceive(s_lcd_event_queue, &ev, 0) == pdTRUE) {
            char sensor_text[20] = "";
            if (snapshot_available) {
                switch (ev.channel) {
                case ACTUATOR_CHANNEL_VALVE:
                    snprintf(sensor_text, sizeof(sensor_text), "WaterLvl: %d", snap.water_level);
                    break;
                case ACTUATOR_CHANNEL_PER_NUTA:
                case ACTUATOR_CHANNEL_PER_NUTB:
                    snprintf(sensor_text, sizeof(sensor_text), "TDS: %.1f", (double)snap.tds);
                    break;
                case ACTUATOR_CHANNEL_PER_PH_UP:
                case ACTUATOR_CHANNEL_PER_PH_DOWN:
                    snprintf(sensor_text, sizeof(sensor_text), "pH: %.2f", (double)snap.ph);
                    break;
                default:
                    snprintf(sensor_text, sizeof(sensor_text), "Sensor: N/A");
                    break;
                }
            }
            lcd_status_show_actuator_event(s_zone_id, s_zone_name,
                ev.channel_name[0] != '\0' ? ev.channel_name : "?",
                sensor_text, ev.state_text);
        }
}
```

- **`xQueueReceive(..., 0)`** — The timeout `0` makes this non-blocking. The `while` loop drains the entire queue in one comm cycle. If there are no events, it exits immediately without stalling the comm task.

- **Why drain in `comm_task`, not `dosing_task`?** — The `dosing_task` runs at a higher priority and handles time-critical hardware. Making it call LCD driver functions (which involve I2C or SPI transactions) would introduce variable latency into actuator execution. Instead, `dosing_task` posts a lightweight event struct to a queue, and `comm_task` handles the display update at its lower priority.

- **Contextual sensor data** — Each actuator event gets annotated with the corresponding sensor reading for that channel (e.g., TDS for nutrient pumps, pH for pH pumps, water level for the valve). This gives the LCD display useful context: instead of just "NutA: PULSE 2000ms", it shows "NutA PULSE 2000ms | TDS: 450.3".

- **State-change filter** — Events are only posted to this queue by `dosing_task` when the actuator state genuinely changes (ON→OFF or OFF→ON), thanks to the `prev_state != new_state` check added in the LCD fix. This prevents the LCD from constantly cycling through "OFF" states.

---

## 6. Heartbeat, Watchdog, and Timing

```c
        runtime_safety_heartbeat_comm();
        runtime_safety_wdt_kick();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COMM_INTERVAL_MS));
    }
```

- **`runtime_safety_heartbeat_comm()`** — Atomically increments `s_safety.comm_heartbeat`. The safety task monitors this counter; if it stops incrementing for 15 seconds, a `SAFETY_FAULT_WDT` is triggered.

- **`runtime_safety_wdt_kick()`** — Feeds the hardware TWDT. If MQTT publishing takes unusually long (e.g., due to TCP retransmission), the 3-second `COMM_INTERVAL_MS` delay loop and the 10ms inter-message delays add up. The WDT timeout (8 seconds) must be set generously enough to accommodate the worst case.

- **`vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COMM_INTERVAL_MS))`** — Ensures a constant 3-second cycle regardless of how long publishing took. If publishing takes 500ms and one `vTaskDelay(10)` fires 12 times (120ms total overhead), the effective wait is `3000 - 620 = 2380ms`, maintaining the 3-second period.

---

## 7. Task Cleanup and Graceful Exit

```c
    runtime_safety_wdt_unregister();

    signal_task_exit(TASK_EXIT_BIT_COMM);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_comm_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}
```

- **`signal_task_exit(TASK_EXIT_BIT_COMM)`** — Sets the `TASK_EXIT_BIT_COMM` bit in the `s_task_exit_event` event group. The shutdown coordinator (`runtime_tasks_stop()`) blocks on all four bits (`DOSING | SENSOR | COMM | SAFETY`) using `xEventGroupWaitBits()`, ensuring all tasks exit cleanly before the system de-initializes MQTT and WiFi.

---

## 8. Inbound Command Handling (MQTT Subscriptions)

The comm task does **not** directly process inbound MQTT messages in its main loop. Instead, `runtime_tasks_start()` registers callback functions with the MQTT manager:

```c
// Registered before comm_task starts:
mqtt_manager_subscribe("<zone_id>/actuator/+/cmd", on_channel_command, NULL);
mqtt_manager_subscribe("<zone_id>/emergency/cmd",  on_emergency_command, NULL);
mqtt_manager_subscribe("<zone_id>/safety/clear",   on_safety_clear_command, NULL);
mqtt_manager_subscribe("<zone_id>/ota/trigger",    on_ota_trigger, NULL);
```

These callbacks execute in the **MQTT library's internal thread context**, not in `comm_task`. They are lightweight: they parse the payload, build a `dosing_command_t`, and call `enqueue_command()` to hand off to `dosing_task`. This design keeps command latency low (no waiting for the 3-second comm cycle) while keeping the comm task's main loop simple.

| MQTT Topic | Callback | Action |
|---|---|---|
| `<zone_id>/actuator/<name>/cmd` | `on_channel_command()` | Maps channel name to enum, enqueues ON/OFF/PULSE |
| `<zone_id>/emergency/cmd` | `on_emergency_command()` | Immediately calls `safety_fault_set()` with all fault bits |
| `<zone_id>/safety/clear` | `on_safety_clear_command()` | Clears faults and reboots (CLEAR) or clears specific mask (MASK:hex) |
| `<zone_id>/ota/trigger` | `on_ota_trigger()` | Kicks off OTA firmware download from broker's HTTP server |
| `<zone_id>/ota/latest_version` | `on_ota_version()` | Stores the latest available version string for comparison |
