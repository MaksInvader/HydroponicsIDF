# Safety Task Documentation

This document provides a comprehensive technical explanation of `runtime_safety_task` — the highest-priority FreeRTOS task that continuously monitors sensor values, actuator states, and task liveness to detect and respond to dangerous operating conditions.

**Source file:** `main/runtime_safety.c`  
**Task priority:** `4` (highest of the four runtime tasks)  
**Stack size:** `6144` bytes  
**Interval:** notified every `SAFETY_INTERVAL_MS` (2000ms), or earlier if woken by a fault

---

## Architecture Overview

The safety task occupies the highest priority in the system. This means that regardless of what any other task is doing, the safety task can always preempt and run its checks. It uses a **task notification** as its primary wait mechanism, which allows it to be woken instantly from any context rather than sleeping on a fixed timer.

```
runtime_safety_task (priority 4 — highest)
    │
    ├─ Software heartbeat check (dosing, sensor, comm)
    │       └─ If any task's counter stalls → SAFETY_FAULT_WDT
    │
    ├─ Dose duration check (all pump channels)
    │       └─ If pump ON > max allowed ms → SAFETY_FAULT_DOSE_*
    │
    ├─ Water temperature check
    │       └─ OOR sustained > 1 min → SAFETY_FAULT_WATER_TEMP
    │
    ├─ pH range and rate checks
    │       ├─ OOR → SAFETY_FAULT_PH_SENSOR
    │       ├─ Critical high/low → SAFETY_FAULT_PH_UP / PH_DOWN
    │       ├─ Rising/falling too fast → SAFETY_FAULT_PH_UP / PH_DOWN
    │       └─ Frozen → SAFETY_FAULT_PH_SENSOR
    │
    ├─ TDS range and frozen checks
    │       ├─ OOR sustained > 10s → SAFETY_FAULT_TDS_SENSOR
    │       └─ Frozen → SAFETY_FAULT_TDS_SENSOR
    │
    ├─ pH dose response check (90s timer)
    │       └─ No pH change after dosing → SAFETY_FAULT_PH_UP / PH_DOWN
    │
    ├─ TDS dose response check (120s timer)
    │       └─ No TDS change after dosing → SAFETY_FAULT_DOSE_A / DOSE_B
    │
    └─ Valve fill timeout check (10 min)
            └─ Valve ON, water level still LOW → SAFETY_FAULT_VALVE
```

When any fault is detected, `safety_fault_set()` atomically sets a bit in `s_safety.faults` and calls `safety_enter_safe_state()` which kills all actuator outputs immediately.

---

## 1. The Fault Mask System

```c
// safety_config.h
typedef uint32_t safety_fault_mask_t;

#define SAFETY_FAULT_WDT          (1u << 0)
#define SAFETY_FAULT_PH_SENSOR    (1u << 1)
#define SAFETY_FAULT_TDS_SENSOR   (1u << 2)
#define SAFETY_FAULT_PH_UP        (1u << 3)
#define SAFETY_FAULT_PH_DOWN      (1u << 4)
#define SAFETY_FAULT_PH_INTERLOCK (1u << 5)
#define SAFETY_FAULT_DOSE_A       (1u << 6)
#define SAFETY_FAULT_DOSE_B       (1u << 7)
#define SAFETY_FAULT_WATER_TEMP   (1u << 8)
#define SAFETY_FAULT_VALVE        (1u << 9)
```

Faults are stored as bits in a `uint32_t` atomic variable (`s_safety.faults`). Each bit represents a distinct failure mode. This allows multiple faults to coexist simultaneously (e.g., `WDT | PH_SENSOR`) and enables selective clearing via a bitmask.

The current fault state is published to MQTT as a hex string on the `<zone_id>/safety/faults` topic, allowing the Node-RED dashboard to decode and display individual fault flags.

---

## 2. Task Initialization and Local State Variables

```c
void runtime_safety_task(void *arg)
{
    (void)arg;
    runtime_safety_wdt_register();

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_dosing_hb = atomic_load(&s_safety.dosing_heartbeat);
    uint32_t last_sensor_hb = atomic_load(&s_safety.sensor_heartbeat);
    uint32_t last_comm_hb   = atomic_load(&s_safety.comm_heartbeat);
    TickType_t dosing_last_change = last_wake;
    TickType_t sensor_last_change = last_wake;
    TickType_t comm_last_change   = last_wake;

    float last_ph = 0.0f;
    float rate_last_ph = 0.0f;
    float last_tds = 0.0f;
    bool last_ph_valid = false;
    bool last_tds_valid = false;
    int ph_frozen = 0;
    int tds_frozen = 0;
    TickType_t rate_last_ph_tick = last_wake;
```

These local variables maintain **state across iterations** — they are not global because only the safety task needs them. Key variables:

| Variable | Purpose |
|---|---|
| `last_dosing_hb`, `last_sensor_hb`, `last_comm_hb` | Last observed heartbeat counter value for each monitored task |
| `dosing_last_change`, etc. | Timestamp of the last heartbeat change, for timeout detection |
| `last_ph`, `last_tds` | Previous sample values, for delta/rate computation |
| `ph_frozen`, `tds_frozen` | Counter for consecutive "unchanged" readings |
| `rate_last_ph`, `rate_last_ph_tick` | State for computing pH rate of change over time |

---

## 3. Wait Mechanism: Task Notifications

```c
    while (s_bindings.stop_requested != NULL &&
           !atomic_load(s_bindings.stop_requested)) {

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SAFETY_INTERVAL_MS));
        TickType_t now = xTaskGetTickCount();
```

- **`ulTaskNotifyTake(pdTRUE, timeout)`** — Puts the task into a blocked state, waiting for a direct-to-task notification. `pdTRUE` clears the notification value upon exit. If no notification is received within `SAFETY_INTERVAL_MS` (2 seconds), the function returns anyway and the safety checks proceed normally.

- **Why not `vTaskDelayUntil`?** — Unlike the sensor and comm tasks, the safety task can be woken *early* via `xTaskNotifyGive()` from `safety_fault_set()` when a fault is detected. This allows the safety task to react and publish a fault immediately, without waiting up to 2 seconds for its normal cycle.

---

## 4. Software Heartbeat Monitoring

```c
        safety_check_heartbeat(s_bindings.dosing_task ? *s_bindings.dosing_task : NULL,
                               &s_safety.dosing_heartbeat,
                               &last_dosing_hb, &dosing_last_change,
                               now, "Dosing task heartbeat stalled");
        // ... same for sensor_task and comm_task ...
```

```c
static void safety_check_heartbeat(TaskHandle_t task, atomic_uint *counter,
                                   uint32_t *last_hb, TickType_t *last_change,
                                   TickType_t now, const char *reason)
{
    uint32_t hb = atomic_load(counter);
    if (hb != *last_hb) {
        *last_hb = hb;
        *last_change = now;
        return;
    }
    if (ticks_to_ms(now - *last_change) > SAFETY_HEARTBEAT_TIMEOUT_MS) {
        safety_fault_set(SAFETY_FAULT_WDT, reason);
    }
}
```

Each of the three monitored tasks (`dosing_task`, `sensor_task`, `comm_task`) increments an atomic counter at the end of every loop iteration. The safety task compares the current counter value against the last-seen value:

- If the counter **changed** → the task is alive. Reset the "last change" timestamp.
- If the counter **has not changed** for > `SAFETY_HEARTBEAT_TIMEOUT_MS` (15 seconds) → the task is stalled. Trigger `SAFETY_FAULT_WDT`.

This is a **software watchdog layer** that is separate from and complementary to the hardware TWDT. It can detect task stalls at a finer granularity (per-task level).

---

## 5. Dose Duration Watchdog

```c
static void safety_check_dose_durations(TickType_t now)
{
    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        dosing_watchdog_t *watchdog = &s_dose_watchdog[i];
        if (!watchdog->on || watchdog->on_tick == 0) continue;

        uint32_t max_dose = safety_max_dose_for_channel((actuator_channel_t)i);
        if (max_dose == 0) continue;

        uint32_t on_duration_ms = ticks_to_ms(now - watchdog->on_tick);
        if (on_duration_ms > max_dose && !atomic_load(&s_override_active)) {
            safety_fault_mask_t fault = safety_fault_for_channel((actuator_channel_t)i);
            if (fault != 0) {
                safety_fault_set(fault, "Pump on-duration exceeded max");
            }
        }
    }
}
```

This check runs every safety cycle. For each channel that is currently ON (`watchdog->on == true`), it computes how long it has been on and compares it against the maximum allowed dose duration (`safety_max_dose_for_channel()`). If exceeded, a fault is triggered. Maximum dose durations per channel (from `safety_config.h`):

| Channel | Constant | Default |
|---|---|---|
| Nutrient A | `SAFETY_MAX_DOSE_NUT_A_MS` | 30,000ms (30s) |
| Nutrient B | `SAFETY_MAX_DOSE_NUT_B_MS` | 30,000ms (30s) |
| pH Up | `SAFETY_MAX_DOSE_PH_UP_MS` | 15,000ms (15s) |
| pH Down | `SAFETY_MAX_DOSE_PH_DOWN_MS` | 15,000ms (15s) |

---

## 6. Water Temperature Monitoring

```c
            if (snap.water_temp_sensor_ok &&
                (snap.water_temp < SAFETY_TEMP_LOW ||
                 snap.water_temp > SAFETY_TEMP_HIGH)) {
                s_safety.water_temp_oor_ticks++;
                uint32_t oor_ms = (uint32_t)s_safety.water_temp_oor_ticks * SAFETY_INTERVAL_MS;
                if (oor_ms >= SAFETY_TEMP_OOR_FAULT_MS && !atomic_load(&s_override_active)) {
                    safety_fault_set(SAFETY_FAULT_WATER_TEMP, "Water temp out of range >1 min");
                }
            } else {
                s_safety.water_temp_oor_ticks = 0;
            }
```

- **`snap.water_temp_sensor_ok`** — A flag set by `sensor_telemetry.c` that is `true` only when the most recent DS18B20 read returned a valid CRC-verified reading. This prevents false faults from transient 1-Wire bus errors.

- **OOR counter** — Rather than faulting immediately on a single out-of-range sample (which could be caused by sensor noise), the firmware requires the temperature to remain out-of-range for `SAFETY_TEMP_OOR_FAULT_MS` (60 seconds) continuously before triggering a fault. The counter resets to 0 the moment any in-range sample arrives.

---

## 7. pH Safety Checks

### 7a. Range Checks

```c
                    if (!warmup_active) {
                        if (snap.ph < SAFETY_PH_MIN || snap.ph > SAFETY_PH_MAX) {
                            safety_fault_set(SAFETY_FAULT_PH_SENSOR, "pH out of range");
                        }
                        if (snap.ph > SAFETY_PH_CRITICAL_HIGH) {
                            safety_fault_set(SAFETY_FAULT_PH_UP, "pH critical high");
                        } else if (snap.ph < SAFETY_PH_CRITICAL_LOW) {
                            safety_fault_set(SAFETY_FAULT_PH_DOWN, "pH critical low");
                        }
                    }
```

- **Warmup period** — `warmup_active = ticks_to_ms(now - last_wake) < SAFETY_PH_WARMUP_MS`. For the first `SAFETY_PH_WARMUP_MS` (60,000ms = 1 minute) after startup, pH faults are suppressed. This prevents false alarms while the pH electrode polarizes after submersion.

- **Two-tier range fault** — The system distinguishes between "out of normal range" (non-critical warning, `PH_SENSOR` fault) and "critically out of range" (`PH_UP` or `PH_DOWN` fault that implies a pump is stuck ON). Critical values are closer to the extremes (e.g., pH < 3.0 or pH > 10.0).

### 7b. Rate of Change Detection

```c
                    if (last_ph_valid) {
                        float rate_dt_min = (float)ticks_to_ms(now - rate_last_ph_tick) / 60000.0f;
                        if (rate_dt_min >= 1.0f) {
                            float rate_delta = snap.ph - rate_last_ph;
                            float rate = rate_delta / rate_dt_min;
                            if (!warmup_active) {
                                if (rate > SAFETY_PH_RATE_MAX_PER_MIN)
                                    safety_fault_set(SAFETY_FAULT_PH_UP, "pH rising too fast");
                                else if (rate < -SAFETY_PH_RATE_MAX_PER_MIN)
                                    safety_fault_set(SAFETY_FAULT_PH_DOWN, "pH dropping too fast");
                            }
                            rate_last_ph = snap.ph;
                            rate_last_ph_tick = now;
                        }
                    }
```

The rate check calculates: **rate (pH/min) = ΔpH / Δt (in minutes)**. This detects a stuck-open pump before the absolute pH limit is breached. For example:
- A pH pump running for 5 minutes might push pH from 6.5 to 9.0 (Δ2.5 / 5 min = 0.5 pH/min).
- If `SAFETY_PH_RATE_MAX_PER_MIN = 0.2`, the fault triggers at 1 minute of continuous pumping, not after the pH reaches the critical range.

The `rate_dt_min >= 1.0f` guard ensures the calculation only runs once per minute, preventing noise from short-interval ADC readings from producing spuriously large rate values.

### 7c. Frozen Sensor Detection

```c
                        float delta = snap.ph - last_ph;
                        if (fabsf(delta) < SAFETY_PH_FROZEN_EPSILON) {
                            ph_frozen++;
                        } else {
                            ph_frozen = 0;
                        }
                        if (!warmup_active && ph_frozen >= SAFETY_PH_FROZEN_SAMPLES) {
                            safety_fault_set(SAFETY_FAULT_PH_SENSOR, "pH sensor frozen");
                        }
```

If the pH reading changes by less than `SAFETY_PH_FROZEN_EPSILON` (0.001 pH) for `SAFETY_PH_FROZEN_SAMPLES` (300 samples = 10 minutes) in a row, the sensor is considered frozen (disconnected, broken electrode, or clogged reference junction). `fabsf()` takes the absolute value to handle both positive and negative deltas.

---

## 8. TDS Safety Checks

```c
                if (snap.tds_valid) {
                    if (snap.tds < SAFETY_TDS_MIN || snap.tds > SAFETY_TDS_MAX) {
                        s_safety.tds_oor_ticks++;
                        uint32_t oor_ms = (uint32_t)s_safety.tds_oor_ticks * SAFETY_INTERVAL_MS;
                        if (oor_ms >= SAFETY_TDS_OOR_FAULT_MS) {
                            safety_fault_set(SAFETY_FAULT_TDS_SENSOR, "TDS out of range >10 s");
                        }
                    } else {
                        s_safety.tds_oor_ticks = 0;
                    }

                    if (last_tds_valid) {
                        if (snap.tds == last_tds) {
                            tds_frozen++;
                        } else {
                            tds_frozen = 0;
                        }
                        if (tds_frozen >= SAFETY_TDS_FROZEN_SAMPLES) {
                            safety_fault_set(SAFETY_FAULT_TDS_SENSOR, "TDS sensor frozen");
                        }
                    }
                }
```

TDS checks are similar to pH but with different parameters:
- OOR fault requires `SAFETY_TDS_OOR_FAULT_MS` (10 seconds) of sustained out-of-range (stricter than temperature).
- Frozen detection checks for exact equality (`snap.tds == last_tds`) because TDS values are low-resolution integers from the ADC, not floats. No epsilon needed.

---

## 9. Dose Response Checks

These checks verify that actuation actually produces a measurable effect on the sensor readings.

### pH Dose Response

```c
                if (ph_pending) {
                    float delta = snap.ph - ph_start;
                    if ((ph_up && delta >= SAFETY_PH_RESPONSE_MIN_DELTA) ||
                        (!ph_up && delta <= -SAFETY_PH_RESPONSE_MIN_DELTA)) {
                        // Success — cancel the timer
                        s_safety.ph_response_pending = false;
                    } else if ((int32_t)(now - ph_deadline) < 0) {
                        // Deadline not reached yet — keep waiting
                    } else {
                        // Timeout — pump may be broken or tube empty
                        safety_fault_set(ph_up ? SAFETY_FAULT_PH_UP : SAFETY_FAULT_PH_DOWN,
                                         "pH response timeout");
                        s_safety.ph_response_pending = false;
                    }
                }
```

Flow: When `dosing_task` executes a pH dose, it calls `runtime_safety_note_ph_dose()`, which records `ph_start` (current pH) and arms a deadline 90 seconds in the future. The safety task then polls:
1. If pH changes by ≥ `SAFETY_PH_RESPONSE_MIN_DELTA` (0.03 pH) in the correct direction → dosing worked, timer cancelled.
2. If the deadline passes without sufficient pH change → `SAFETY_FAULT_PH_UP` or `SAFETY_FAULT_PH_DOWN` is triggered.

The `(int32_t)(now - ph_deadline) < 0` trick handles FreeRTOS tick counter rollover safely using signed integer arithmetic. When `now < ph_deadline`, this expression is negative (deadline not yet reached).

### TDS Dose Response

Same pattern for TDS: after a nutrient dose, TDS must rise by ≥ `SAFETY_TDS_RESPONSE_MIN_DELTA` (5 ppm) within `SAFETY_TDS_RESPONSE_TIMEOUT_MS` (120 seconds), otherwise a fault is triggered.

---

## 10. Valve Fill Timeout

```c
            if (valve_waiting && valve_on_tick != 0 && snap.water_level == 0 &&
                !atomic_load(&s_override_active)) {
                if (ticks_to_ms(now - valve_on_tick) > SAFETY_FILL_TIMEOUT_MS) {
                    safety_fault_set(SAFETY_FAULT_VALVE, "Valve fill timeout");
                    s_safety.valve_waiting = false;
                }
            }
```

When the solenoid valve opens (`ACTUATOR_ACTION_ON`), `runtime_safety_note_valve_action()` records the timestamp. If the water level sensor (`snap.water_level`) remains `0` (empty) for more than `SAFETY_FILL_TIMEOUT_MS` (10 minutes) after the valve opened, it implies the water supply is cut off, the valve is broken, or the tank has a leak. A `SAFETY_FAULT_VALVE` fault is triggered.

---

## 11. The `safety_enter_safe_state()` Function

This function is called inside `safety_fault_set()` whenever a new fault is detected for the first time:

```c
static void safety_enter_safe_state(safety_fault_mask_t fault_mask)
{
    atomic_store(&s_safety.safe_mode, true);

    // Kill ALL actuator outputs immediately
    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        (void)actuator_control_apply_state((actuator_channel_t)i, false);
        runtime_safety_update_channel_state((actuator_channel_t)i, false);
    }

    flush_dosing_queue("safe mode entered");
}
```

- **`atomic_store(&s_safety.safe_mode, true)`** — Sets the global safe mode flag. All subsequent dosing commands will be rejected by `dosing_task` and all new queue entries will be rejected by `enqueue_command()`.

- **All-off actuator sweep** — Iterates through every channel and drives its GPIO LOW. This is a hardware-level forced shutdown regardless of what the server says.

- **`flush_dosing_queue()`** — Drains the dosing command queue to discard any pending commands that have not yet been executed.
