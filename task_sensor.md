# Sensor Task Documentation

This document provides a comprehensive technical explanation of the `sensor_task` — the FreeRTOS task responsible for periodically sampling all physical sensors (pH, TDS, water temperature, water level) and making the data available to the rest of the system.

**Source files:** `main/runtime_tasks.c`, `main/sensor_telemetry.c`  
**Task priority:** `2` (SENSOR_TASK_PRIORITY)  
**Stack size:** `8192` bytes (increased from 6144 due to ADS1115 + DS18B20 + logging overhead)  
**Interval:** every `2000 ms` (`SENSOR_SAMPLE_INTERVAL_MS`)

---

## Architecture Overview

The `sensor_task` does **not** own the hardware directly. Instead, it acts as a periodic scheduler that calls into `sensor_telemetry.c` — the hardware abstraction layer that manages ADS1115 (I2C ADC), DS18B20 (1-Wire), and GPIO digital sensors. The result of each sample cycle is stored in a shared snapshot, which other tasks can read via `sensor_telemetry_get_snapshot()`.

```
sensor_task (every 2 s)
    │
    └─► sensor_telemetry_sample()           ← hardware read + rolling average
            │
            ├─► ADS1115 (I2C) → pH raw ADC → calibration → ph (float)
            ├─► ADS1115 (I2C) → TDS raw ADC → calibration → tds (float)
            ├─► DS18B20 (1-Wire) → water_temp (float)
            └─► GPIO digital → water_level (int)
            │
            └─► updates internal snapshot

Other tasks: sensor_telemetry_get_snapshot(&snap) → reads snapshot
```

---

## 1. Task Initialization

```c
static void sensor_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    runtime_safety_wdt_register();

    UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Sensor task stack watermark: %u bytes free",
             (unsigned)(stack_hwm * sizeof(StackType_t)));
```

- **`static void sensor_task(void *arg)`** — The task entry point. It is `static` to restrict its linkage to `runtime_tasks.c`, where it is passed by function pointer to `xTaskCreate()`. The `void *arg` parameter is the standard FreeRTOS task argument, unused here.

- **`(void)arg`** — Explicitly discards the argument to suppress compiler warnings about unused parameters (`-Wunused-parameter`).

- **`TickType_t last_wake = xTaskGetTickCount()`** — Records the current FreeRTOS tick count at the moment the task starts. `TickType_t` is a platform-dependent integer type (typically `uint32_t` on ESP32). This value is passed to `vTaskDelayUntil()` later to implement non-drifting periodic execution.

- **`runtime_safety_wdt_register()`** — Registers this task with the hardware Task Watchdog Timer (TWDT). After registration, the task must periodically call `runtime_safety_wdt_kick()` or the WDT will trigger a system reset. This prevents a silent hang inside `sensor_telemetry_sample()` from going undetected.

- **`uxTaskGetStackHighWaterMark(NULL)`** — FreeRTOS API that returns the minimum free stack space (in words) this task has ever had. `NULL` means "query this calling task." This is multiplied by `sizeof(StackType_t)` (4 on ESP32) to convert to bytes. Logging this at startup provides a baseline for stack usage profiling.

---

## 2. The Main Sensor Loop

```c
    uint32_t loop_count = 0;
    while (!atomic_load(&s_stop_requested)) {

        if (++loop_count % 100 == 0) {
            stack_hwm = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGD(TAG, "Sensor task stack watermark: %u bytes free",
                     (unsigned)(stack_hwm * sizeof(StackType_t)));
        }

        esp_err_t ret = sensor_telemetry_sample();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Sensor sample failed: %s", esp_err_to_name(ret));
        }
```

- **`atomic_load(&s_stop_requested)`** — Reads the global stop flag atomically (using `<stdatomic.h>`). `atomic_load` guarantees that the read is not reordered by the compiler or CPU across other memory operations, ensuring that a write to `s_stop_requested` from another task (e.g., during shutdown) is always seen correctly.

- **`loop_count % 100 == 0`** — The periodic stack watermark log runs once every 100 iterations (every 200 seconds with a 2s interval). This is a non-intrusive method to detect gradual stack consumption growth (e.g., from recursive logging or accumulating local variables) without flooding the log output.

- **`sensor_telemetry_sample()`** — The core action of the task. This function performs the full hardware read cycle:
  1. Reads pH and TDS raw ADC values from the ADS1115 over the shared I2C bus (protected by `s_i2c0_mutex`).
  2. Applies the slope-intercept linear calibration (`ph = slope * raw + offset`).
  3. Triggers a DS18B20 temperature conversion and reads the result (split across two consecutive calls to avoid blocking for 750ms).
  4. Reads the water level GPIO.
  5. Pushes all values into their respective rolling average buffers (window of 30 samples).
  6. Updates the internal snapshot struct so other tasks can access it.

- **`esp_err_to_name(ret)`** — Converts an ESP-IDF error code (e.g., `ESP_ERR_TIMEOUT`) to a human-readable string for logging. This is essential for debugging sensor communication failures.

---

## 3. Heartbeats and Precise Timing

```c
        runtime_safety_heartbeat_sensor();
        runtime_safety_wdt_kick();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS));
    }
```

- **`runtime_safety_heartbeat_sensor()`** — Atomically increments a counter (`s_safety.sensor_heartbeat`). The `safety_task` monitors this counter each cycle. If it stops incrementing for more than `SAFETY_HEARTBEAT_TIMEOUT_MS` (15 seconds), the safety task concludes that `sensor_task` has stalled and triggers a `SAFETY_FAULT_WDT` fault. This is a software heartbeat *layer* separate from the hardware WDT.

- **`runtime_safety_wdt_kick()`** — Feeds the hardware Task Watchdog Timer by calling `esp_task_wdt_reset()`. This resets the TWDT timer for this specific task. If not called within `SAFETY_WDT_TIMEOUT_SEC` (8 seconds), the TWDT will generate a `trigger_panic` and reboot the system.

- **`vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS))`** — This is the key to jitter-free periodic execution. Unlike `vTaskDelay(ms)` which delays *relative to the current moment*, `vTaskDelayUntil` delays until an *absolute future tick* based on `last_wake`, then auto-advances `last_wake` by the period. The result: if `sensor_telemetry_sample()` takes 50ms, the task still wakes up at exactly `T + 2000ms`, not `T + 50ms + 2000ms`. This guarantees a constant 2-second sample rate regardless of how long the sampling itself takes.

- **`pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS)`** — A FreeRTOS macro that converts milliseconds to scheduler ticks. With a default tick rate of 1000 Hz on ESP32, `pdMS_TO_TICKS(2000)` = 2000 ticks.

---

## 4. Relationship with the Safety Task

The safety task independently calls `sensor_telemetry_get_snapshot()` to validate readings. This means the safety subsystem does not depend on `sensor_task` to pass data — it reads directly from the same snapshot. This design choice has an important consequence: even if `sensor_task` crashes, the safety task can still detect anomalous sensor values from the last known snapshot.

```
sensor_task ──writes──► sensor snapshot ◄──reads── safety_task
                                         ◄──reads── comm_task
```

---

## 5. Task Cleanup and Graceful Exit

```c
    runtime_safety_wdt_unregister();

    signal_task_exit(TASK_EXIT_BIT_SENSOR);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_sensor_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}
```

- **`runtime_safety_wdt_unregister()`** — Calls `esp_task_wdt_delete(NULL)`, removing this task from the TWDT subscriber list. This must happen before the task is deleted; otherwise the TWDT would fire when the task stops kicking it.

- **`signal_task_exit(TASK_EXIT_BIT_SENSOR)`** — Sets a bit in the `s_task_exit_event` FreeRTOS event group. The `runtime_tasks_stop()` function blocks waiting for all four task exit bits to be set before returning, ensuring a fully synchronous shutdown.

- **`portENTER_CRITICAL` / `portEXIT_CRITICAL`** — These macros disable interrupts on the current core for the duration of the critical section. This is necessary because `s_sensor_task` (the task handle) is a shared variable that could be read simultaneously by `runtime_tasks_stop()`. Setting it to `NULL` under a critical section prevents a use-after-free scenario.

- **`vTaskDelete(NULL)`** — The FreeRTOS API to terminate the calling task. `NULL` means "delete me." After this call, the task's stack and TCB (Task Control Block) memory is returned to the heap.
