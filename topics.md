# MQTT Topic Reference

All topics are prefixed with `<zone_id>` (e.g. `zone_0`), configured at runtime from NVS.

---

## Setup / Provisioning

| Topic | Direction | Payload | Notes |
|---|---|---|---|
| `SetUp` | **Publish** | JSON `{"zone_id":"...","name":"..."}` | Published once during provisioning to register zone with broker |
| `<zone_id>/success` | **Subscribe** | any | Broker ACK for SetUp; unsubscribed after receipt |

---

## Actuator Commands (Subscribe)

Format: `<zone_id>/<channel>/command`

| Topic | Channel |
|---|---|
| `<zone_id>/valve/command` | Main valve |
| `<zone_id>/PerNutA/command` | Nutrient A peristaltic pump |
| `<zone_id>/PerNutB/command` | Nutrient B peristaltic pump |
| `<zone_id>/PerpHUp/command` | pH Up peristaltic pump |
| `<zone_id>/PerpHDown/command` | pH Down peristaltic pump |
| `<zone_id>/relay1/command` | Relay 1 |
| `<zone_id>/relay2/command` | Relay 2 |
| `<zone_id>/relay3/command` | Relay 3 |
| `<zone_id>/relay4/command` | Relay 4 |
| `<zone_id>/CirculationPump/command` | Circulation pump (always ON, command ignored) |
| `<zone_id>/command` | Multi-channel zone command |

**Payload:** `ON` / `OFF` / `PULSE:<ms>`
Multi-channel payload: `<ChannelName> ON|OFF|PULSE <ms>`

---

## Actuator Status (Publish)

Format: `<zone_id>/<channel>/status` — QoS 1, retain 0

| Topic | Payload |
|---|---|
| `<zone_id>/valve/status` | `ON` / `OFF` |
| `<zone_id>/PerNutA/status` | `ON` / `OFF` |
| `<zone_id>/PerNutB/status` | `ON` / `OFF` |
| `<zone_id>/PerpHUp/status` | `ON` / `OFF` |
| `<zone_id>/PerpHDown/status` | `ON` / `OFF` |
| `<zone_id>/relay1/status` | `ON` / `OFF` |
| `<zone_id>/relay2/status` | `ON` / `OFF` |
| `<zone_id>/relay3/status` | `ON` / `OFF` |
| `<zone_id>/relay4/status` | `ON` / `OFF` |
| `<zone_id>/CirculationPump/status` | `ON` / `OFF` |

---

## Sensor Telemetry (Publish)

Published every 1000 ms by `comm_task`. QoS 0 unless noted.

| Topic | Payload | Retain | Notes |
|---|---|---|---|
| `<zone_id>/sensor/WaterLevel/state` | `0` or `1` | 0 | GPIO digital level |
| `<zone_id>/sensor/WaterTemp/state` | float `"%.2f"` °C | 0 | DS18B20 rolling average |
| `<zone_id>/sensor/pH/raw` | uint16 ADC counts | 0 | ADS1115 AIN0 |
| `<zone_id>/sensor/pH/state` | float `"%.2f"` | 0 | Suppressed when invalid |
| `<zone_id>/sensor/pH/valid` | `"true"` / `"false"` | **1** | Retained |
| `<zone_id>/sensor/TDS/raw` | uint16 ADC counts | 0 | ADS1115 AIN1 |
| `<zone_id>/sensor/TDS/state` | float `"%.1f"` ppm | 0 | Suppressed when invalid |
| `<zone_id>/sensor/TDS/valid` | `"true"` / `"false"` | **1** | Retained |

---

## Calibration

| Topic | Direction | Payload | QoS | Retain |
|---|---|---|---|---|
| `<zone_id>/calibration/pH/set` | **Subscribe** | JSON `{"mode":"linear","slope":f,"offset":f,"valid":true,"updated_at":u32}` | 1 | — |
| `<zone_id>/calibration/TDS/set` | **Subscribe** | JSON same schema | 1 | — |
| `<zone_id>/calibration/pH/state` | **Publish** | JSON with coefficients + `valid` + optional `reason` | 1 | **1** |
| `<zone_id>/calibration/TDS/state` | **Publish** | JSON same schema | 1 | **1** |

---

## Safety

| Topic | Direction | Payload | QoS | Retain | Notes |
|---|---|---|---|---|---|
| `<zone_id>/safety/fault` | **Publish** | JSON `{"mask":u32,"code":"F-NNN","count":n,"text":"..."}` | 1 | 0 (boot faults: 1) | Published on fault set |
| `<zone_id>/safety/clear` | **Subscribe** | `"CLEAR"` / `"ALL"` / `"MASK:<hex>"` | 1 | — | `ALL` or `CLEAR` → restart; `MASK` → clear specific faults |
| `<zone_id>/emergency` | **Subscribe** | `"1"` | 1 | — | Triggers all faults / safe mode immediately |

---

## OTA

| Topic | Direction | Payload | Notes |
|---|---|---|---|
| `<zone_id>/ota/latest_version` | **Subscribe** | version string e.g. `"1.2.3"` | Stored for logging; no version gate on trigger |
| `<zone_id>/ota/trigger` | **Subscribe** | `"1"` | Starts HTTP OTA from `http://<broker_ip>:8123/local/firmware/lorong_node.bin` |

---

## Version

| Topic | Direction | Payload | QoS | Retain | Interval |
|---|---|---|---|---|---|
| `<zone_id>/version` | **Publish** | firmware version string | 1 | **1** | On connect + every 30 s |
