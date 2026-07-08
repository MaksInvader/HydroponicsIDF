# Dosing Task Documentation

This document provides a comprehensive technical explanation of the `dosing_task` — the FreeRTOS task responsible for executing all actuator commands (peristaltic pumps, solenoid valve, and relays) in a safe, serialized, and interlock-enforced manner.

**Source file:** `main/runtime_tasks.c`  
**Task priority:** `3` (DOSING_TASK_PRIORITY) — second highest, below SafetyTask (4)  
**Stack size:** `4096` bytes  
**Queue depth:** `64` commands (`DOSING_QUEUE_LEN`)

---

## Architecture Overview

The `dosing_task` is an **event-driven** task — it does not run on a fixed timer. Instead it blocks indefinitely on a FreeRTOS queue (`s_dosing_queue`) waiting for actuator commands to arrive. Commands are placed into this queue by MQTT message callbacks (from `comm_task`'s MQTT subscription handlers). The task then executes them one at a time, in order, while enforcing safety interlocks.

```
MQTT broker
    │
    ▼
on_channel_command() / on_zone_command()   ← MQTT callback (comm context)
    │
    └─► enqueue_command() ──► s_dosing_queue (FreeRTOS queue, depth=64)
                                    │
                                    ▼
                             dosing_task  ← blocks on xQueueReceive
                                    │
                                    ├─ safety check (safe mode?)
                                    ├─ peristaltic interlock check
                                    ├─ actuator_control_apply_state/pulse()
                                    ├─ runtime_safety_update_channel_state()
                                    ├─ dose watchdog update
                                    └─ LCD event post (if state changed)
```

---

## 1. Helper Functions: Identifying Peristaltic Channels

```c
/* Returns true if the channel is a peristaltic dosing pump. */
static bool is_peristaltic_channel(actuator_channel_t ch)
{
    return ch == ACTUATOR_CHANNEL_PER_NUTA   ||
           ch == ACTUATOR_CHANNEL_PER_NUTB   ||
           ch == ACTUATOR_CHANNEL_PER_PH_UP  ||
           ch == ACTUATOR_CHANNEL_PER_PH_DOWN;
}
```

- **`actuator_channel_t`** — An `enum` type defined in `actuator_control.h` that maps to each hardware output. Passing a numeric integer directly is avoided in favor of this typed enum to catch type mismatches at compile time.

- **Logic** — Returns `true` if the channel is any of the four chemical-injection pumps (Nutrient A, Nutrient B, pH Up, pH Down). Relay channels and the solenoid valve are **not** peristaltic, so they bypass the interlock logic.

- **`static`** — Restricts the function to `runtime_tasks.c` translation unit, allowing the compiler to inline it at all call sites since it is a pure predicate with no side effects.

---

## 2. Helper Function: Checking for Running Pumps

```c
static bool any_peristaltic_pump_running(actuator_channel_t except_channel)
{
    const actuator_channel_t per_channels[] = {
        ACTUATOR_CHANNEL_PER_NUTA,
        ACTUATOR_CHANNEL_PER_NUTB,
        ACTUATOR_CHANNEL_PER_PH_UP,
        ACTUATOR_CHANNEL_PER_PH_DOWN,
    };
    for (size_t i = 0; i < sizeof(per_channels) / sizeof(per_channels[0]); i++) {
        if (per_channels[i] == except_channel) continue;
        if (runtime_safety_get_channel_state(per_channels[i])) return true;
    }
    return false;
}
```

- **Purpose** — Enforces mutual exclusion: only one peristaltic pump may run at a time. Simultaneous operation risks overdosing and, in the case of pH Up + pH Down together, a direct acid-base chemical reaction inside the tubing.

- **`sizeof(per_channels) / sizeof(per_channels[0])`** — A compile-time safe way to compute the number of elements in a stack-allocated array. This is preferable to hard-coding `4` because if a pump is ever added to the array, the loop bound updates automatically.

- **`except_channel`** — The function excludes the channel being *requested* from the running check. Without this parameter, a request for the already-running pump would incorrectly report a conflict with itself.

- **`runtime_safety_get_channel_state(ch)`** — Reads the logical state of a channel from the `s_channel_state_on[]` shadow array inside `runtime_safety.c`, protected by `portENTER_CRITICAL`. This is the thread-safe source of truth about what is currently ON or OFF.

---

## 3. Queue Command Enqueueing

```c
static bool enqueue_command(const dosing_command_t *cmd, const char *source)
{
    if (!runtime_safety_is_gate_open(xPortInIsrContext())) {
        return false;
    }
    // ...
    BaseType_t sent = xQueueSend(s_dosing_queue, cmd, 0);

    if (sent != pdTRUE) {
        /* Queue full — flush all stale pending commands and immediately
         * enqueue the latest one so the most recent server order takes priority. */
        UBaseType_t flushed = uxQueueMessagesWaiting(s_dosing_queue);
        xQueueReset(s_dosing_queue);
        sent = xQueueSend(s_dosing_queue, cmd, 0);
    }
    return (sent == pdTRUE);
}
```

- **`runtime_safety_is_gate_open()`** — Checks the "safety gate" flag. The gate is closed (`false`) when the system enters Safe Mode. All queuing is silently rejected while the gate is closed, preventing a flood of stale commands from executing when the system recovers.

- **`xQueueSend(queue, item, timeout=0)`** — Non-blocking enqueue. If the queue is full, it returns `pdFALSE` immediately rather than blocking.

- **Queue-full strategy** — When the 64-slot queue overflows, the firmware does **not** drop the newest command. Instead it calls `xQueueReset()` to discard all 64 stale commands, then enqueues only the newest one. This "latest wins" policy ensures that the most recent server instruction (e.g., "stop all pumps") is always honored even during a burst.

---

## 4. Task Initialization

```c
static void dosing_task(void *arg)
{
    (void)arg;
    dosing_command_t cmd;

    runtime_safety_wdt_register();

    UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Dosing task stack watermark: %u bytes free",
             (unsigned)(stack_hwm * sizeof(StackType_t)));
```

- **`dosing_command_t cmd`** — Stack-allocated local variable used to receive one command at a time from the queue. Its structure contains: `actuator_channel_t channel`, `actuator_action_t action`, and `uint32_t pulse_ms`.

- **`runtime_safety_wdt_register()`** — Registers with the hardware TWDT. The dosing task must kick the watchdog in its queue receive timeout loop (200ms timeout), or else the TWDT fires.

---

## 5. The Main Event Loop

```c
    while (!atomic_load(&s_stop_requested)) {
        if (xQueueReceive(s_dosing_queue, &cmd, pdMS_TO_TICKS(DOSING_QUEUE_RECV_TIMEOUT_MS)) == pdTRUE) {

            if (runtime_safety_is_safe_mode() || atomic_load(&s_stop_requested)) {
                ESP_LOGW(TAG, "Dropping dosing command while in safe mode or stopping");
                continue;
            }

            bool prev_state = runtime_safety_get_channel_state(cmd.channel);
```

- **`xQueueReceive(queue, &cmd, timeout)`** — Blocks the task for up to `DOSING_QUEUE_RECV_TIMEOUT_MS` (200ms). If a command arrives, it is deep-copied into `cmd` and the function returns `pdTRUE`. If no command arrives within the timeout, it returns `pdFALSE` and the loop iterates, which gives the opportunity to kick the heartbeat and watchdog. This prevents the task from being stuck indefinitely if the server stops sending commands.

- **`runtime_safety_is_safe_mode()`** — Double-checks safe mode *at execution time*. Even if a command was enqueued before a fault occurred, it will be dropped here if the system transitioned to safe mode in the meantime.

- **`prev_state`** — Records the channel's logical state *before* the command is applied. This is used later to determine whether the LCD event queue should be updated (only if the state genuinely changes).

---

## 6. Peristaltic Interlock Logic

```c
            if ((cmd.action == ACTUATOR_ACTION_ON || cmd.action == ACTUATOR_ACTION_PULSE) &&
                is_peristaltic_channel(cmd.channel) &&
                any_peristaltic_pump_running(cmd.channel)) {

                bool ph_chemical_conflict =
                    (cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP   &&
                     runtime_safety_get_channel_state(ACTUATOR_CHANNEL_PER_PH_DOWN)) ||
                    (cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN &&
                     runtime_safety_get_channel_state(ACTUATOR_CHANNEL_PER_PH_UP));

                if (ph_chemical_conflict) {
                    runtime_safety_fault_set(SAFETY_FAULT_PH_INTERLOCK,
                                            "pH Up+Down simultaneous — chemical hazard");
                } else {
                    ESP_LOGW(TAG, "Peristaltic interlock: ch=%d skipped, another pump is running",
                             (int)cmd.channel);
                }
                continue;
            }
```

This block implements a **two-tier interlock**:

| Scenario | Result |
|---|---|
| Peristaltic pump command arrives while another peristaltic is ON | Command skipped (non-fatal) |
| pH Up command arrives while pH Down is ON (or vice versa) | **Safety fault triggered** (`SAFETY_FAULT_PH_INTERLOCK`) |

- The first tier is a **resource conflict** — two pumps cannot physically share the same tubing simultaneously without mixing. The command is simply dropped with a warning.
- The second tier is a **chemical hazard** — pH Up (base/KOH) and pH Down (acid/H2SO4) cannot be dispensed simultaneously as they neutralize each other inside the reservoir and can cause dangerous exothermic reactions. This is treated as a fault, locking the system into safe mode.

---

## 7. Executing Actuator Actions

```c
            esp_err_t ret = ESP_OK;
            switch (cmd.action) {
            case ACTUATOR_ACTION_ON:
                ret = actuator_control_apply_state(cmd.channel, true);
                if (ret == ESP_OK) {
                    runtime_safety_update_channel_state(cmd.channel, true);
                }
                break;
            case ACTUATOR_ACTION_OFF:
                ret = actuator_control_apply_state(cmd.channel, false);
                if (ret == ESP_OK) {
                    runtime_safety_update_channel_state(cmd.channel, false);
                }
                break;
            case ACTUATOR_ACTION_PULSE:
                runtime_safety_update_channel_state(cmd.channel, true);
                ret = actuator_control_apply_pulse(cmd.channel, cmd.pulse_ms);
                runtime_safety_update_channel_state(cmd.channel, false);
                break;
            }
```

Three actions are supported:

| Action | Behavior |
|---|---|
| `ACTUATOR_ACTION_ON` | Sets GPIO HIGH. State shadow updated to `true` on success. |
| `ACTUATOR_ACTION_OFF` | Sets GPIO LOW. State shadow updated to `false` on success. |
| `ACTUATOR_ACTION_PULSE` | Sets GPIO HIGH, blocks for `pulse_ms`, then sets GPIO LOW. State shadow updated on both edges. |

- **`actuator_control_apply_state(channel, on)`** — Calls `gpio_set_level()` on the configured GPIO pin for that channel. Returns `ESP_ERR_INVALID_ARG` if the channel has no configured pin.

- **`actuator_control_apply_pulse(channel, ms)`** — Calls `gpio_set_level(HIGH)`, then `vTaskDelay(ms)`, then `gpio_set_level(LOW)`. The dosing task blocks for the full pulse duration. No other dosing commands are processed while a PULSE is executing — this is by design and is why mutual exclusion between pumps is enforced before this point.

- **State shadow update before pulse vs. after** — For PULSE, `runtime_safety_update_channel_state(ch, true)` is called *before* the hardware action. This is intentional: the Safety Task's dose watchdog needs to see the pump as ON from the moment it starts, so it can correctly measure the ON duration.

---

## 8. LCD Event Filtering (State-Change Filter)

```c
            bool new_state = (cmd.action == ACTUATOR_ACTION_ON ||
                              cmd.action == ACTUATOR_ACTION_PULSE);
            bool state_changed = (prev_state != new_state) ||
                                 (cmd.action == ACTUATOR_ACTION_PULSE);

            if (state_changed && s_lcd_event_queue != NULL) {
                lcd_actuator_event_t ev = {0};
                // ... build ev ...
                xQueueSend(s_lcd_event_queue, &ev, 0);
            }
```

- **Why filter?** — The server frequently sends redundant `OFF` commands to all actuators. Before this filter, every such command generated an LCD event, causing the display to cycle through all channels showing "OFF" even though nothing had changed.

- **`state_changed` logic** — An LCD event is only posted if the logical state is transitioning (`false→true` or `true→false`). PULSE commands always generate an event because the pump physically activates and deactivates, which is meaningful visual feedback.

- **`xQueueSend(..., 0)`** — Non-blocking. If the 8-slot LCD event queue is full, the event is silently dropped. The display is purely informational and a dropped event is acceptable.

---

## 9. Post-Execution: Safety Notifications

```c
            runtime_safety_dose_watchdog_update(cmd.channel, cmd.action, cmd.pulse_ms);

            if (cmd.channel == ACTUATOR_CHANNEL_VALVE) {
                runtime_safety_note_valve_action(cmd.action);
            }
            if (cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP ||
                cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN) {
                runtime_safety_note_ph_dose(cmd.channel);
            }
            if (cmd.channel == ACTUATOR_CHANNEL_PER_NUTA ||
                cmd.channel == ACTUATOR_CHANNEL_PER_NUTB) {
                runtime_safety_note_tds_dose(cmd.channel);
            }
```

After a successful actuator action, the dosing task notifies the safety subsystem:

| Notification | Triggered by | Purpose |
|---|---|---|
| `runtime_safety_dose_watchdog_update()` | Any pump | Starts/stops the dose duration watchdog timer |
| `runtime_safety_note_valve_action()` | Valve ON/OFF | Arms a fill timeout — if valve stays open > 10 min with low water, fault |
| `runtime_safety_note_ph_dose()` | pH Up or Down | Arms a 90-second response timer — if pH doesn't change by ≥0.03, fault |
| `runtime_safety_note_tds_dose()` | NutA or NutB | Arms a 120-second response timer — if TDS doesn't change by ≥5 ppm, fault |

These notifications implement a closed-loop safety check: the firmware verifies that actuators actually *work* by checking if the sensor readings change as expected after an action.

---

## 10. Task Cleanup and Graceful Exit

```c
        runtime_safety_heartbeat_dosing();
        runtime_safety_wdt_kick();
    }

    runtime_safety_wdt_unregister();

    signal_task_exit(TASK_EXIT_BIT_DOSING);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_dosing_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}
```

- **`runtime_safety_heartbeat_dosing()`** — Increments the dosing heartbeat counter. Even when the queue is empty (queue receive timeout fires), the task still increments this counter each iteration, proving to the safety watchdog that the task is alive and looping.

- **Shutdown sequence** — The loop exits when `s_stop_requested` is set. The task then unregisters from the WDT, signals its exit bit, clears its handle, and deletes itself. The `runtime_tasks_stop()` function on the caller side waits for all four `TASK_EXIT_BIT_*` bits before returning.
