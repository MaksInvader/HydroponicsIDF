# Firmware Codebase Audit — Website_UpdaterFreeRTOS

**Date:** 2026-05-27  
**Target:** ESP32-S3, ESP-IDF v5.5.3, FreeRTOS  
**Binary:** `Website_Updater.bin` — 1,040,624 bytes (0xFE4E0)  
**Partition headroom:** ~6,944 bytes (1%) — **critically low**

---

## 1. Flash Usage Analysis

### 1.1 Memory Section Summary

| Section | Size (bytes) | Notes |
|---|---|---|
| `.flash.text` (code) | 748,674 | Executable instructions |
| `.flash.rodata` (read-only data) | 167,128 | Strings, tables, TLS certs |
| `.appdesc` | 256 | App descriptor |
| **Total flash** | **916,058** | Of ~1,048,576 available in app partition |
| `.iram0.text` | 86,027 | Time-critical code in IRAM |
| `.dram0.data` | 23,012 | Initialized globals |
| `.dram0.bss` | 25,472 | Zero-initialized globals |
| IRAM | 16,384 / 16,384 | **100% full** |

### 1.2 Top Flash Consumers (by component)

| Rank | Library | Flash Code | Flash Data | Total |
|---|---|---|---|---|
| 1 | `libnet80211.a` (Wi-Fi stack) | 116,681 | 17,477 | **146,512** |
| 2 | `liblwip.a` (TCP/IP) | 99,881 | 3,968 | **107,942** |
| 3 | `libesp_app_format.a` (x509 cert bundle) | 447 | 107,050 | **107,507** |
| 4 | `libmbedcrypto.a` (TLS crypto) | 73,282 | 7,032 | **80,750** |
| 5 | `libwpa_supplicant.a` (WPA2) | 62,430 | 1,576 | **65,344** |
| 6 | `libpp.a` (Wi-Fi PHY) | 42,090 | 3,957 | **64,763** |
| 7 | `libc.a` (newlib C library) | 43,668 | 3,161 | **47,413** |
| 8 | `libmain.a` (application code) | 31,237 | 635 | **41,997** |
| 9 | `libesp_hw_support.a` | 25,008 | 1,722 | **36,573** |
| 10 | `libphy.a` (RF PHY) | 27,762 | 0 | **36,173** |
| 11 | `libmbedtls.a` (TLS) | 27,991 | 1,940 | **30,171** |
| 12 | `libfreertos.a` | 1,202 | 1,411 | **22,800** |
| 13 | `libmqtt.a` | 15,190 | 334 | **15,524** |

### 1.3 Root Cause of Near-Full Flash

The binary is large because of the **full TLS + Wi-Fi stack** being pulled in:

- `libesp_app_format.a` contributes **107 KB of read-only data** — this is the **x509 certificate bundle** (`x509_crt_bundle.S`) included for HTTPS/OTA. It alone accounts for ~10% of total flash.
- The Wi-Fi stack (`libnet80211` + `libpp` + `libphy` + `libwpa_supplicant`) totals **~313 KB**.
- mbedTLS (`libmbedcrypto` + `libmbedtls` + `libmbedx509`) totals **~120 KB**.
- The OTA uses plain HTTP (`http://`), not HTTPS — so the full x509 bundle is **included but never used**.
- Build optimization is `-O3` (optimize for speed), not `-Os` (optimize for size).
- IRAM is 100% full, forcing some normally-IRAM code into slower flash execution.

---

## 2. Architecture Audit

### 2.1 Task Structure

| Task | Stack | Priority | WDT | Notes |
|---|---|---|---|---|
| `dosing_task` | 4096 | 3 | Yes | Processes actuator queue |
| `sensor_task` | 6144 | 2 | Yes | DS18B20 + ADS1115 + GPIO |
| `comm_task` | 8192 | 1 | Yes | MQTT publish + LCD |
| `safety_task` | 6144 | 4 | Yes | Heartbeat + fault checks |
| `setup_button_monitor` | (unknown) | (unknown) | No | Background GPIO poll |

**Issues:**
- `comm_task` has the largest stack (8192) but lowest priority (1). If MQTT publish blocks, it starves — this is the root cause of the CommTask WDT seen in earlier sessions.
- `safety_task` at priority 4 is correct — it must preempt everything.
- No stack high-water mark logging at startup — impossible to know if any task is close to overflow without a monitor.

### 2.2 MQTT Manager

- 24 subscription slots, each holding a 128-byte topic string = **3,072 bytes** of static BSS.
- 4 connection callback slots.
- Subscription lock timeout is 100 ms — tight for a system where the MQTT event task can be busy.
- `mqtt_manager_resubscribe_all()` takes a full snapshot of all 24 slots under lock before resubscribing — correct pattern, avoids deadlock.
- `subscribe_calibration_topics_nowait()` used from `on_mqtt_connected` — correct, avoids blocking the MQTT event task.

**Issue:** Each subscription entry stores a full 128-byte topic string statically. With 24 slots this is fine, but the topic buffer in `mqtt_dispatch_snapshot_t` is also 128 bytes — allocated on the stack during dispatch. With `comm_task` stack at 8192 this is acceptable but worth noting.

### 2.3 Safety System

- `DEV_MODE = 1` is currently active — **all safety faults are suppressed**. Must be set to 0 before production deployment.
- Fault mask is `atomic_uint` — correct for cross-task access.
- Gate open/close uses `portMUX_TYPE` spinlock — correct for ISR-safe access.
- Boot fault accumulation uses `RTC_DATA_ATTR` counters — survives deep sleep and warm resets, correct.
- pH/TDS safety checks are gated on `snap.ph_valid` / `snap.tds_valid` — correct (fixed this session).
- Temperature safety checks use `snap.water_temp` which is now gated on `temp_valid` — correct (fixed this session).

**Issue:** `SAFETY_FILL_TIMEOUT_MS = 120000` (2 minutes). If the valve GPIO is stuck HIGH due to a hardware fault, the firmware will not detect it for 2 minutes. Consider reducing to 60 s for faster fault detection.

**Issue:** `SAFETY_WDT_TIMEOUT_SEC = 8` but `SAFETY_HEARTBEAT_TIMEOUT_MS = 5000`. The WDT fires at 8 s, heartbeat fault triggers at 5 s — the safety task should catch a stalled task before the WDT fires. This ordering is correct.

### 2.4 Sensor Pipeline

**Water temperature (DS18B20):**
- Deferred-read pattern: trigger at end of cycle N, read at start of cycle N+1.
- `temp_valid` gate now prevents zeros from entering history — fixed.
- No `water_temp_valid` field in snapshot — safety checks use raw `snap.water_temp` value. If DS18B20 is absent, `water_temp` stays 0.0 and will trigger `SAFETY_TEMP_LOW` fault (0 < 10.0°C). This is actually safe-fail behaviour, but should be documented.

**pH / TDS (ADS1115):**
- Single-shot I2C reads with 10 ms conversion wait per channel — 20 ms total blocking in `sensor_task` per cycle. Acceptable at 1 s sample interval.
- `ads1115_init()` calls `i2c_param_config` unconditionally — will reconfigure the I2C port even if LCD already initialised it. Both use `I2C_NUM_0`, SDA=16, SCL=15 at 100 kHz — parameters are identical so no conflict in practice, but the ordering dependency is fragile.
- ADS1115 probe failure causes `ensure_sensor_interfaces()` to return an error, which causes `sensor_telemetry_init()` to fail, which means `s_initialized` is never set, and `sensor_telemetry_sample()` always returns `ESP_ERR_INVALID_STATE`. pH/TDS will never publish. This is correct fail-safe behaviour.
- Raw value published as `uint16` but ADS1115 returns 0–32767 (15-bit effective single-ended). Calibration slope/offset stored in NVS from the old 12-bit ADC era are now invalid and must be recalibrated.

**Water level:**
- Simple `gpio_get_level` — no debounce. A noisy float switch could cause rapid state changes and spam MQTT. Consider a 2–3 sample majority vote.

### 2.5 OTA Update

- Uses plain HTTP (`http://`), not HTTPS.
- The x509 certificate bundle (`libesp_app_format.a`, 107 KB) is included in the build despite OTA being HTTP-only. This is the single largest avoidable flash consumer.
- `OTA_HTTP_BUFFER_SIZE = 1024` — small chunk size means many read iterations for a ~1 MB firmware. Increasing to 4096 or 8192 would speed up OTA significantly.
- `OTA_MAX_TOTAL_SIZE = 4 MB` — larger than the flash chip on most ESP32-S3 modules. Should be set to the actual OTA partition size.
- No version check before flashing — any `"1"` on the trigger topic starts OTA regardless of whether the new firmware is actually newer.

### 2.6 Web Portal

- HTML is stored as two large static `const char *` strings in `.rodata` — contributes to the 167 KB rodata section.
- No authentication on the `/configure` POST endpoint — anyone on the AP network can reconfigure the device.
- `url_decode` is implemented manually — correct but adds code size. ESP-IDF's `httpd_query_key_value` handles this natively.

### 2.7 Wi-Fi Manager

- `MAXIMUM_RETRY = 10` with no backoff — rapid reconnect attempts on a missing AP. Consider exponential backoff.
- AP mode uses open network (`NULL` password) — documented in the portal HTML but worth flagging as a security note.
- `s_sta_auto_reconnect_enabled` is set to `true` on first successful connection and never reset — correct for persistent reconnection.

### 2.8 NVS Layout

| Namespace | Keys | Purpose |
|---|---|---|
| `setup_cfg` | ssid, password, broker_ip, broker_port, ota_broker_ip | Wi-Fi + broker config |
| `zone_cfg` | zone_id, zone_name | Zone identity |
| `sensor_cali` | ph_slope, ph_offset, ph_valid, ph_upd_at, tds_slope, tds_offset, tds_valid, tds_upd_at | Calibration coefficients |

- Floats stored as `uint32` IEEE-754 bit patterns via `nvs_set_u32` — correct and portable.
- NVS verify functions (`sensor_calibration_nvs_verify_ph/tds`) exist but are not called after save — write-verify is not enforced.

### 2.9 Pin Configuration

| Pin | Function | Notes |
|---|---|---|
| GPIO 1 | LED connection | |
| GPIO 2 | LED fault | |
| GPIO 5 | Water level sensor | Digital input, pull-up |
| GPIO 7 | Setup button | Active-LOW, pull-up |
| GPIO 8 | DS18B20 1-Wire | |
| GPIO 12 | Valve | |
| GPIO 13–14 | Relay 1–2 | |
| GPIO 15 | ADS1115 SCL / LCD SCL | Shared |
| GPIO 16 | ADS1115 SDA / LCD SDA | Shared |
| GPIO 21 | Relay 3 | |
| GPIO 38–41 | pH Down, pH Up, NutB, NutA | |
| GPIO 45 | Relay 4 | |
| GPIO 46 | Setup button (hardcoded in main.c) | Conflicts with `PIN_SETUP_BUTTON = 7` — see below |
| GPIO 47 | Circulation pump | |

**Critical issue:** `main.c` hardcodes GPIO 46 in two `ESP_LOGI` messages, but `PIN_SETUP_BUTTON` is defined as GPIO 7 in `pin_config.h`. The actual GPIO used by `setup_button.c` is `PIN_SETUP_BUTTON = 7`. The log messages in `main.c` are misleading — they say GPIO 46 but the button is on GPIO 7.

---

## 3. Code Quality Issues

### 3.1 Bugs

| Severity | Location | Issue |
|---|---|---|
| **Fixed** | `sensor_telemetry.c` | `temp_valid` gate — zeros no longer enter history |
| **Fixed** | `sensor_telemetry.c` | `s_initialized` always set after init |
| **Fixed** | `runtime_safety.c` | pH/TDS checks gated on `ph_valid`/`tds_valid` |
| Medium | `main.c` lines 35, 50 | Hardcoded GPIO 46 in log messages; actual button is `PIN_SETUP_BUTTON = 7` |
| Medium | `sensor_telemetry.c` | No `water_temp_valid` in snapshot — absent DS18B20 silently produces 0.0°C |
| Low | `sensor_telemetry.c` | `i2c_param_config` called unconditionally in `ads1115_init` — fragile if LCD init order changes |
| Low | `actuator_control.c` | Water level GPIO has no debounce |
| Low | `ota_update.c` | `OTA_MAX_TOTAL_SIZE = 4 MB` exceeds actual OTA partition size |

### 3.2 Missing Features / TODOs

- No stack high-water mark reporting at startup or via MQTT diagnostic topic.
- No MQTT authentication (username/password) — broker is open.
- No version gate on OTA trigger — any `"1"` flashes regardless of version.
- NVS write-verify not enforced after calibration save.
- Water level debounce absent.
- `setup_button_monitor` task stack size and priority not visible in `setup_button.c` header — should be in `safety_config.h` or `pin_config.h`.

### 3.3 DEV_MODE Warning

`DEV_MODE = 1` in `safety_config.h` disables all safety faults. This must be set to `0` before any production or field deployment. There is no compile-time warning for this.

---

## 4. Flash Size Reduction Recommendations

Listed by impact, easiest first:

### 4.1 Remove x509 certificate bundle (saves ~104 KB)

The bundle is included because `esp-tls` or `esp_http_client` pulls it in by default. Since OTA uses plain HTTP, it is never used.

In `sdkconfig.defaults`, add:
```
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=n
CONFIG_ESP_TLS_USING_MBEDTLS=y
CONFIG_ESP_TLS_SERVER=n
```
Or in `menuconfig`: Component config → mbedTLS → Certificate Bundle → disable.

**Estimated saving: ~104 KB**

### 4.2 Switch optimization to -Os (saves ~50–80 KB)

The project currently builds with `-O3`. Switching to `-Os` trades a small runtime speed reduction for significantly smaller code.

In `CMakeLists.txt`:
```cmake
idf_build_set_property(COMPILE_OPTIONS "-Os" APPEND)
```
Or set `CONFIG_COMPILER_OPTIMIZATION_SIZE=y` in sdkconfig.

**Estimated saving: 50–80 KB**

### 4.3 Disable unused mbedTLS features (saves ~20–40 KB)

If HTTPS is not used anywhere, disable TLS server and reduce cipher suites:
```
CONFIG_ESP_HTTPS_SERVER_ENABLE=n
CONFIG_MBEDTLS_SSL_PROTO_DTLS=n
CONFIG_MBEDTLS_ECP_DP_SECP521R1_ENABLED=n
CONFIG_MBEDTLS_ECP_DP_BP512R1_ENABLED=n
CONFIG_MBEDTLS_ECP_DP_BP384R1_ENABLED=n
```

**Estimated saving: 20–40 KB**

### 4.4 Reduce lwIP socket count (saves ~8 KB)

`CONFIG_LWIP_MAX_SOCKETS=16` is already set. Reducing to 8 saves ~8 KB of BSS:
```
CONFIG_LWIP_MAX_SOCKETS=8
```

**Estimated saving: ~8 KB**

### 4.5 Increase OTA partition size

Rather than shrinking the binary, the partition table can be adjusted to give the app partition more space. Without a custom `partitions.csv`, ESP-IDF uses a default layout. Adding a `partitions.csv` with a larger app partition (e.g. 1.5 MB per OTA slot) removes the near-full warning entirely.

Example `partitions.csv`:
```
# Name,   Type, SubType, Offset,   Size,  Flags
nvs,      data, nvs,     0x9000,   0x6000,
otadata,  data, ota,     0xf000,   0x2000,
ota_0,    app,  ota_0,   0x11000,  0x180000,
ota_1,    app,  ota_1,   0x191000, 0x180000,
```
This gives 1.5 MB per OTA slot (current binary is ~1.02 MB, leaving ~500 KB headroom).

**Recommended: combine 4.1 + 4.2 + 4.5 for ~200 KB reduction and comfortable headroom.**

---

## 5. Summary

| Category | Status |
|---|---|
| Core functionality | Working |
| Safety system | Working (DEV_MODE=1 — disable before production) |
| Sensor pipeline | Fixed (temp gate, valid flags) |
| ADS1115 I2C | Implemented, needs hardware calibration |
| Flash headroom | **Critical — 1% free** |
| IRAM | **100% full** |
| x509 cert bundle | Included but unused — largest single saving available |
| GPIO 46 vs PIN_SETUP_BUTTON=7 | Misleading log messages in main.c |
| OTA | HTTP only, no version gate, small buffer |
| Security | Open AP, no MQTT auth, no OTA version check |
