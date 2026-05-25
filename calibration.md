# Sensor Calibration — Firmware Flow

## Overview

Calibration maps a raw 12-bit ADC count (0–4095) to a physical value (pH 0–14 or TDS 0–5000 ppm) using a linear model:

```
physical_value = slope × adc_raw + offset
```

Coefficients are received over MQTT, validated, persisted to NVS, and applied every sample cycle. The system is designed so that sensors remain in an **invalid** state until calibration is confirmed — no faults fire and no calibrated values are published until then.

---

## Files Involved

| File | Responsibility |
|---|---|
| `main/sensor_telemetry.c` | ADC sampling, calibration application, MQTT callbacks, publish |
| `main/sensor_calibration_nvs.c` | NVS persistence (save, load, verify) |
| `main/sensor_calibration_nvs.h` | `calibration_t` struct, public NVS API |
| `main/sensor_telemetry.h` | `sensor_telemetry_snapshot_t` struct (exposes `ph`, `ph_raw`, `ph_valid`, etc.) |
| `main/runtime_safety.c` | Consumes snapshot; checks `ph_valid`/`tds_valid` before fault evaluation |

---

## Data Structures

### `calibration_t` (defined in `sensor_telemetry.h`, used everywhere)

```c
typedef struct {
    float    slope;       // linear coefficient
    float    offset;      // linear intercept
    bool     valid;       // true only after a confirmed write
    uint32_t updated_at;  // Unix timestamp (seconds) of last update
} calibration_t;
```

### `sensor_reading_t` (internal to `sensor_telemetry.c`)

```c
typedef struct {
    uint16_t raw;    // raw ADC count (0–4095)
    float    value;  // calibrated physical value
    bool     valid;  // true only when calibration present AND value in range
} sensor_reading_t;
```

### `sensor_telemetry_snapshot_t` (public, in `sensor_telemetry.h`)

```c
typedef struct {
    uint16_t ph_raw;     // latest raw ADC count
    float    ph;         // windowed average of calibrated pH values
    bool     ph_valid;   // true when calibration present AND value in range

    uint16_t tds_raw;
    float    tds;
    bool     tds_valid;

    float    water_temp;
    int      water_level;
    bool     valid;      // true once sensor_telemetry_init completes
    uint32_t publish_count;
} sensor_telemetry_snapshot_t;
```

---

## Boot Sequence

```
main()
  └─ sensor_telemetry_init(zone_id)
       ├─ ensure_sensor_interfaces()          // init ADC1 oneshot handle, map GPIO→channel
       ├─ build_sensor_topics(zone_id)        // build all MQTT topic strings
       ├─ reset_state_locked()                // zero history buffers, snapshot
       ├─ Load from NVS
       │    ├─ sensor_calibration_nvs_load_ph()   → s_ph_cali
       │    └─ sensor_calibration_nvs_load_tds()  → s_tds_cali
       │         • If found: validate plausible range (slope/offset bounds)
       │         • If invalid range: log warning, leave cali.valid = false
       │         • If not found: cali.valid = false (sensors stay invalid)
       ├─ mqtt_manager_add_connection_cb(on_mqtt_connected)
       ├─ if (mqtt_manager_is_connected())
       │    └─ subscribe_calibration_topics()   // non-fatal if fails
       └─ s_initialized = true                  // always reached
```

After boot, if NVS had valid calibration, sensors immediately produce valid readings on the first sample. If not, they remain invalid until calibration arrives via MQTT.

---

## Sample Cycle (every 1 second, called from `comm_task`)

```
sensor_telemetry_sample()
  ├─ Guard: return ESP_ERR_INVALID_STATE if !s_initialized
  │
  ├─ Read ADC
  │    ├─ adc_oneshot_read(s_adc1_handle, s_ph_adc_channel,  &ph_raw)
  │    └─ adc_oneshot_read(s_adc1_handle, s_tds_adc_channel, &tds_raw)
  │
  ├─ apply_adc_calibration_and_validate(&s_ph_cali,  ph_raw,  0.0, 14.0,   &ph_reading)
  ├─ apply_adc_calibration_and_validate(&s_tds_cali, tds_raw, 0.0, 5000.0, &tds_reading)
  │
  ├─ Update rolling history (SENSOR_SAMPLE_WINDOW samples)
  │    ├─ if (ph_reading.valid)  → s_ph_history[index]  = ph_reading.value
  │    └─ if (tds_reading.valid) → s_tds_history[index] = tds_reading.value
  │         • Invalid readings are NOT pushed — history stays clean
  │
  └─ Update snapshot (under s_snapshot_lock)
       ├─ s_snapshot.ph_raw   = ph_reading.raw
       ├─ s_snapshot.ph       = average_window(s_ph_history, count)
       ├─ s_snapshot.ph_valid = ph_reading.valid
       ├─ s_snapshot.tds_raw  = tds_reading.raw
       ├─ s_snapshot.tds      = average_window(s_tds_history, count)
       └─ s_snapshot.tds_valid = tds_reading.valid
```

### `apply_adc_calibration_and_validate()`

```
out->raw   = (uint16_t)adc_raw
out->value = 0.0f
out->valid = false

Gate 1: if (!cali->valid)  → return   // no calibration yet
Gate 2: computed = slope × adc_raw + offset
Gate 3: if (!isfinite(computed)) → return
Gate 4: if (computed < val_min || computed > val_max) → return

out->value = computed
out->valid = true
```

---

## MQTT Calibration Receive Flow

### Topics

| Direction | Topic | Payload |
|---|---|---|
| Inbound (set) | `<zone>/calibration/pH/set` | JSON — see below |
| Inbound (set) | `<zone>/calibration/TDS/set` | JSON — see below |
| Outbound (state) | `<zone>/calibration/pH/state` | JSON (retained) |
| Outbound (state) | `<zone>/calibration/TDS/state` | JSON (retained) |
| Outbound (valid) | `<zone>/sensor/pH/valid` | `"true"` / `"false"` (retained) |
| Outbound (valid) | `<zone>/sensor/TDS/valid` | `"true"` / `"false"` (retained) |

### Inbound Payload Format

```json
{
  "mode": "linear",
  "slope": -0.0034,
  "offset": 14.12,
  "valid": true,
  "updated_at": 1747534634
}
```

### Receive → Apply Flow

```
on_cali_ph_set() / on_cali_tds_set()
  └─ handle_cali_set(payload, len, range, nvs_save, nvs_verify, &s_ph_cali, ...)
       │
       ├─ 1. validate_cali_payload()
       │       ├─ Must be JSON object (starts with '{')
       │       ├─ "mode" field must be present and equal "linear"
       │       ├─ "slope" field must be present and parseable
       │       ├─ "offset" field must be present and parseable
       │       ├─ slope and offset must be finite (not NaN/Inf)
       │       ├─ slope must be non-zero
       │       ├─ slope within [slope_min, slope_max]
       │       └─ offset within [offset_min, offset_max]
       │            • On failure: publish error to state topic, return
       │
       ├─ 2. NVS write with retry (up to 3 attempts)
       │       └─ sensor_calibration_nvs_save_ph/tds(&new_cali)
       │            • On all attempts fail: publish error, keep old in-memory cali, return
       │
       ├─ 3. NVS read-back verification
       │       └─ sensor_calibration_nvs_verify_ph/tds(slope, offset)
       │            • Reads back from NVS, compares within 0.00005 tolerance
       │            • On mismatch: set cali_mem->valid = false, publish error, return
       │
       ├─ 4. Update in-memory calibration (under s_cali_lock)
       │       └─ *cali_mem = new_cali
       │
       └─ 5. Publish state (retained)
               ├─ <zone>/calibration/pH/state  ← JSON with slope/offset/valid/updated_at
               └─ <zone>/sensor/pH/valid        ← "true"
```

---

## NVS Persistence

### Namespace and Keys

```
NVS namespace: "sensor_cali"

pH keys:   ph_slope  (u32/float), ph_offset  (u32/float), ph_valid  (u8), ph_upd_at  (u32)
TDS keys:  tds_slope (u32/float), tds_offset (u32/float), tds_valid (u8), tds_upd_at (u32)
```

Floats are stored as their IEEE-754 bit pattern in a `uint32` key (`nvs_set_u32` / `nvs_get_u32`) to avoid NVS type limitations.

### Save Flow

```
sensor_calibration_nvs_save_ph(c)
  └─ nvs_open("sensor_cali", NVS_READWRITE)
       ├─ nvs_set_u32(h, "ph_slope",  float_bits(c->slope))
       ├─ nvs_set_u32(h, "ph_offset", float_bits(c->offset))
       ├─ nvs_set_u8 (h, "ph_valid",  c->valid)
       ├─ nvs_set_u32(h, "ph_upd_at", c->updated_at)
       ├─ nvs_commit(h)
       └─ nvs_close(h)
```

### Load Flow (at boot)

```
sensor_calibration_nvs_load_ph(c, &found)
  └─ nvs_open("sensor_cali", NVS_READONLY)
       ├─ If namespace not found → *found = false, return ESP_OK
       ├─ Read slope, offset, valid, updated_at
       ├─ If any key missing (ESP_ERR_NVS_NOT_FOUND) → *found = false, return ESP_OK
       └─ On success → *found = true, populate c
```

### Verify Flow (post-write)

```
sensor_calibration_nvs_verify_ph(expected_slope, expected_offset)
  └─ nvs_open(NVS_READONLY)
       ├─ Read back slope and offset
       └─ if |rb_slope - expected| > 0.00005 OR |rb_offset - expected| > 0.00005
              → return ESP_ERR_INVALID_STATE  (triggers error publish + cali invalidation)
```

---

## MQTT Reconnect Handling

On every MQTT connect event, `on_mqtt_connected()` runs:

```
on_mqtt_connected(connected=true)
  ├─ subscribe_calibration_topics()
  │    ├─ subscribe <zone>/calibration/pH/set  → on_cali_ph_set  (QoS 1, 3 retries)
  │    └─ subscribe <zone>/calibration/TDS/set → on_cali_tds_set (QoS 1, 3 retries)
  │
  └─ Deferred boot publish (if pending)
       ├─ if ph_pub_pending  → publish calibration/pH/state  (or valid=false if no cali)
       └─ if tds_pub_pending → publish calibration/TDS/state (or valid=false if no cali)
```

The deferred publish handles the case where MQTT was not connected when `sensor_telemetry_init` ran — the state is published on the first successful connection.

---

## Safety Integration

`runtime_safety.c` reads the snapshot every `SAFETY_INTERVAL_MS` (500 ms):

```
safety_task()
  └─ sensor_telemetry_get_snapshot(&snap)
       ├─ if (snap.ph_valid)
       │    ├─ Check snap.ph against SAFETY_PH_MIN / SAFETY_PH_MAX
       │    ├─ Check snap.ph against SAFETY_PH_CRITICAL_LOW / HIGH
       │    ├─ Check rate of change (per minute)
       │    └─ Check frozen (delta < SAFETY_PH_FROZEN_EPSILON for N samples)
       │
       └─ if (snap.tds_valid)
            ├─ Check snap.tds against SAFETY_TDS_MIN / SAFETY_TDS_MAX
            └─ Check frozen (delta < SAFETY_TDS_FROZEN_EPSILON for N samples)
```

**If `ph_valid` or `tds_valid` is false, all corresponding fault checks are skipped entirely.** This means an uncalibrated sensor never triggers a fault.

---

## Plausible Range Limits

| Parameter | pH | TDS |
|---|---|---|
| slope min | -20.0 | -100.0 |
| slope max | +20.0 | +100.0 |
| offset min | -50.0 | -500.0 |
| offset max | +50.0 | +500.0 |
| value min | 0.0 | 0.0 |
| value max | 14.0 | 5000.0 ppm |

---

## Two-Point Calibration (Computing slope/offset)

The firmware does not compute slope/offset on-device. The server (or operator tool) performs the two-point calculation and sends the result directly:

Given two calibration points `(raw1, physical1)` and `(raw2, physical2)`:

$$slope = \frac{physical_2 - physical_1}{raw_2 - raw_1}$$

$$offset = physical_1 - slope \times raw_1$$

The computed `slope` and `offset` are then sent as the MQTT JSON payload to `<zone>/calibration/pH/set` or `<zone>/calibration/TDS/set`.

---

## Published MQTT Topics Summary

| Topic | Format | Retained | When |
|---|---|---|---|
| `<zone>/sensor/pH/raw` | uint16 string | No | Every sample |
| `<zone>/sensor/pH/state` | float string | No | Every sample (suppressed if !ph_valid) |
| `<zone>/sensor/pH/valid` | `"true"` / `"false"` | Yes | On calibration change |
| `<zone>/calibration/pH/state` | JSON | Yes | On calibration change |
| `<zone>/sensor/TDS/raw` | uint16 string | No | Every sample |
| `<zone>/sensor/TDS/state` | float string | No | Every sample (suppressed if !tds_valid) |
| `<zone>/sensor/TDS/valid` | `"true"` / `"false"` | Yes | On calibration change |
| `<zone>/calibration/TDS/state` | JSON | Yes | On calibration change |
