# Requirements Document

## Introduction

This document specifies the redesigned calibration scheme for an ESP32 FreeRTOS-based IoT sensor system. The system manages per-zone, per-sensor-type (pH and TDS) linear calibration data persisted in NVS and communicated over MQTT. The calibration lifecycle covers boot-time restoration, runtime reception and validation of new calibration payloads, NVS persistence with integrity verification, and a sensor reading pipeline that gates calibrated output on calibration validity.

## Glossary

- **Calibration_Manager**: The firmware subsystem responsible for loading, validating, persisting, and applying calibration data for each sensor type within a zone.
- **NVS**: Non-Volatile Storage provided by ESP-IDF for persisting key-value data across reboots.
- **Sensor_Type**: One of the supported sensor categories: pH or TDS.
- **Zone_ID**: A string identifier (e.g. "zone_1") that uniquely identifies a physical growing zone.
- **Calibration_Data**: A record consisting of mode, slope, offset, valid flag, and updated_at timestamp.
- **Raw_ADC_Value**: The unprocessed digital reading from a sensor's analog-to-digital converter or GPIO level.
- **Calibrated_Value**: The physical measurement computed by applying the linear calibration formula to a Raw_ADC_Value.
- **MQTT_Broker**: The external message broker to which the device publishes telemetry and from which it receives commands.
- **Retained_Message**: An MQTT message with the retain flag set, causing the broker to store and deliver it to new subscribers immediately upon subscription.
- **Plausible_Range**: The physically meaningful bounds for calibration parameters — for pH: slope ∈ [-20.0, 20.0] and offset ∈ [-50.0, 50.0]; for TDS: slope ∈ [-100.0, 100.0] and offset ∈ [-500.0, 500.0].

## Requirements

### Requirement 1: Boot-Time Calibration Restoration

**User Story:** As a zone controller, I want calibration data to be automatically restored from NVS on boot, so that sensors resume calibrated operation without manual intervention after a power cycle.

#### Acceptance Criteria

1. WHEN the Calibration_Manager initializes for a given Zone_ID and Sensor_Type, THE Calibration_Manager SHALL attempt to read slope, offset, valid, and updated_at from NVS using a namespace keyed by Zone_ID and Sensor_Type.
2. WHEN valid Calibration_Data exists in NVS (valid flag is true and slope and offset are within the Plausible_Range for that Sensor_Type), THE Calibration_Manager SHALL apply the loaded slope and offset to the sensor reading pipeline before the first sensor reading is processed.
3. WHEN valid Calibration_Data is loaded from NVS, THE Calibration_Manager SHALL publish the full calibration state to `<zone_id>/calibration/<type>/state` as a Retained_Message with valid set to true.
4. WHEN no Calibration_Data exists in NVS for a Sensor_Type, THE Calibration_Manager SHALL publish valid equal to false to `<zone_id>/sensor/<type>/valid` as a Retained_Message.
5. WHEN no Calibration_Data exists in NVS for a Sensor_Type, THE Calibration_Manager SHALL suppress publishing to `<zone_id>/sensor/<type>/state` until valid calibration is received.
6. IF the NVS read operation fails with an error, THEN THE Calibration_Manager SHALL set the in-memory calibration to invalid, publish valid equal to false to `<zone_id>/sensor/<type>/valid` as a Retained_Message, and suppress calibrated state publishing.
7. IF Calibration_Data exists in NVS with valid flag true but slope or offset falls outside the Plausible_Range for that Sensor_Type, THEN THE Calibration_Manager SHALL treat the data as invalid, publish valid equal to false to `<zone_id>/sensor/<type>/valid` as a Retained_Message, and publish an error to `<zone_id>/calibration/<type>/state` with valid set to false and a reason field indicating out-of-range coefficients.
8. IF the MQTT connection is not yet established when calibration state publication is required, THEN THE Calibration_Manager SHALL defer publication until the MQTT connection is available and publish the retained state at that time.

### Requirement 2: Raw Sensor Reading Publication

**User Story:** As a monitoring dashboard, I want raw ADC values published on every reading regardless of calibration state, so that I can always observe the sensor hardware output.

#### Acceptance Criteria

1. WHEN a sensor reading is taken for a given Sensor_Type, THE Calibration_Manager SHALL publish the Raw_ADC_Value as a decimal integer string to `<zone_id>/sensor/<type>/raw` within the same communication cycle as the reading.
2. THE Calibration_Manager SHALL publish the Raw_ADC_Value on every reading cycle regardless of whether a valid calibration is currently loaded, whether the calibration is marked invalid, or whether NVS is in a corrupted state.
3. IF the sensor hardware read fails for a given Sensor_Type, THEN THE Calibration_Manager SHALL skip publishing to `<zone_id>/sensor/<type>/raw` for that cycle and resume publishing on the next successful read.

### Requirement 3: MQTT Calibration Subscription

**User Story:** As a calibration operator, I want the device to subscribe to calibration command topics, so that I can push new calibration coefficients remotely.

#### Acceptance Criteria

1. WHEN the Calibration_Manager initializes, THE Calibration_Manager SHALL subscribe to `<zone_id>/calibration/pH/set` with QoS 1 to receive incoming pH calibration payloads.
2. WHEN the Calibration_Manager initializes, THE Calibration_Manager SHALL subscribe to `<zone_id>/calibration/TDS/set` with QoS 1 to receive incoming TDS calibration payloads.
3. WHEN the MQTT connection is re-established after a disconnection, THE Calibration_Manager SHALL re-subscribe to both `<zone_id>/calibration/pH/set` and `<zone_id>/calibration/TDS/set` with QoS 1 within 5 seconds of reconnection.
4. IF a subscription attempt fails, THEN THE Calibration_Manager SHALL retry the subscription up to 3 times with a 2-second interval between attempts and log the failure.
5. IF all subscription retry attempts are exhausted, THEN THE Calibration_Manager SHALL publish an error message to `<zone_id>/calibration/<type>/state` with valid set to false and a reason field indicating subscription failure.

### Requirement 4: Calibration Payload Validation

**User Story:** As a system operator, I want incoming calibration payloads validated before application, so that invalid or dangerous coefficients cannot corrupt the sensor pipeline.

#### Acceptance Criteria

1. WHEN a calibration payload is received, THE Calibration_Manager SHALL attempt to parse the payload as a JSON object; IF the payload is not a valid JSON object, THEN THE Calibration_Manager SHALL reject the payload and treat it as a validation failure.
2. WHEN a calibration payload is received and parsed successfully, THE Calibration_Manager SHALL verify that the mode, slope, and offset fields are all present in the payload.
3. WHEN a calibration payload is received, THE Calibration_Manager SHALL verify that the slope and offset values are finite numbers (not NaN and not Infinity) and that the slope value is non-zero.
4. WHEN a calibration payload is received, THE Calibration_Manager SHALL verify that the mode field contains a recognized value (the value "linear" is the only recognized mode).
5. WHEN a calibration payload is received for pH, THE Calibration_Manager SHALL verify that slope falls within [-20.0, 20.0] and offset falls within [-50.0, 50.0].
6. WHEN a calibration payload is received for TDS, THE Calibration_Manager SHALL verify that slope falls within [-100.0, 100.0] and offset falls within [-500.0, 500.0].
7. IF any validation check fails, THEN THE Calibration_Manager SHALL stop validation at the first failing check and publish an error message to `<zone_id>/calibration/<type>/state` with valid set to false and a reason field identifying which check failed.
8. IF any validation check fails, THEN THE Calibration_Manager SHALL discard the payload without overwriting existing NVS Calibration_Data.

### Requirement 5: Calibration Persistence

**User Story:** As a zone controller, I want validated calibration data persisted to NVS, so that calibration survives power cycles without requiring re-calibration.

#### Acceptance Criteria

1. WHEN a calibration payload passes all validation checks, THE Calibration_Manager SHALL record the current Unix timestamp as updated_at.
2. WHEN a calibration payload passes all validation checks, THE Calibration_Manager SHALL write mode, slope, offset, valid (set to true), and updated_at to NVS under a namespace keyed by both Zone_ID and Sensor_Type as an atomic operation (all fields committed together or none committed).
3. IF the NVS write operation fails, THEN THE Calibration_Manager SHALL publish an error to `<zone_id>/calibration/<type>/state` with valid set to false and a reason field describing the write failure, and retain the previously loaded calibration in memory without modifying the in-memory valid flag.
4. IF the NVS write operation fails, THEN THE Calibration_Manager SHALL retry the write up to 2 additional times before declaring failure.

### Requirement 6: Post-Calibration State Publication

**User Story:** As a monitoring dashboard, I want the full calibration state published after a successful update, so that all subscribers have the latest coefficients.

#### Acceptance Criteria

1. WHEN a calibration write to NVS succeeds, THE Calibration_Manager SHALL publish the full calibration state to `<zone_id>/calibration/<type>/state` as a Retained_Message in the format: `{"mode":"linear","slope":<value>,"offset":<value>,"valid":true,"updated_at":<unix_timestamp>}`.
2. WHEN a calibration write to NVS succeeds, THE Calibration_Manager SHALL publish valid equal to true to `<zone_id>/sensor/<type>/valid` as a Retained_Message.
3. WHEN a calibration write to NVS succeeds, THE Calibration_Manager SHALL apply the new slope and offset to the sensor reading pipeline before the next invocation of the sensor sampling cycle, such that all subsequent Calibrated_Values use the updated coefficients.
4. IF the MQTT publish of calibration state or validity fails after a successful NVS write, THEN THE Calibration_Manager SHALL retry the publish up to 3 times with a 1-second interval between attempts, and the new calibration SHALL remain applied to the reading pipeline regardless of publish outcome.

### Requirement 7: Calibrated Value Computation

**User Story:** As a control algorithm, I want calibrated sensor values computed and published only when calibration is valid, so that downstream logic operates on trustworthy data.

#### Acceptance Criteria

1. WHILE a valid calibration is loaded for a Sensor_Type, THE Calibration_Manager SHALL compute the Calibrated_Value as (Raw_ADC_Value × slope) + offset for mode "linear".
2. IF the computed Calibrated_Value falls outside the Sensor_Type's valid physical range (pH: 0.0 to 14.0, TDS: 0.0 to 5000.0 ppm) or is non-finite, THEN THE Calibration_Manager SHALL treat the reading as invalid and suppress publishing to `<zone_id>/sensor/<type>/state` for that cycle.
3. WHILE a valid calibration is loaded for a Sensor_Type and the computed Calibrated_Value is within the valid physical range, THE Calibration_Manager SHALL publish the Calibrated_Value to `<zone_id>/sensor/<type>/state` as a non-retained message on each reading cycle.
4. WHILE no valid calibration is loaded for a Sensor_Type, THE Calibration_Manager SHALL suppress publishing to `<zone_id>/sensor/<type>/state`.
5. WHEN a sensor reading cycle completes for a Sensor_Type, THE Calibration_Manager SHALL publish the current validity state as the string "true" or "false" to `<zone_id>/sensor/<type>/valid` as a Retained_Message.

### Requirement 8: NVS Integrity and Corruption Handling

**User Story:** As a safety system, I want NVS corruption detected and reported immediately, so that stale or corrupted calibration data is never silently served to downstream consumers.

#### Acceptance Criteria

1. IF the NVS becomes corrupted or a read-back after write fails, THEN THE Calibration_Manager SHALL, before the next sensor reading is published, publish to `<zone_id>/calibration/<type>/state` as a Retained_Message with valid set to false, a reason field describing the NVS failure, and updated_at set to the last known value or 0 if no prior timestamp exists.
2. IF NVS corruption is detected, THEN THE Calibration_Manager SHALL mark the in-memory calibration as invalid, publish valid equal to false to `<zone_id>/sensor/<type>/valid` as a Retained_Message, and suppress publishing to `<zone_id>/sensor/<type>/state` until a new valid calibration is received and persisted.
3. THE Calibration_Manager SHALL continue publishing Raw_ADC_Values to `<zone_id>/sensor/<type>/raw` regardless of NVS integrity state.
4. WHEN a calibration write to NVS succeeds, THE Calibration_Manager SHALL read back the written slope and offset values and compare them to the intended values; a mismatch in either value SHALL be treated as NVS corruption.

### Requirement 9: Sensor Type Independence

**User Story:** As a multi-sensor system, I want each sensor type's calibration to be fully independent, so that a failure or update in one sensor's calibration does not affect the other.

#### Acceptance Criteria

1. THE Calibration_Manager SHALL maintain separate in-memory Calibration_Data instances (mode, slope, offset, valid flag, and updated_at) for each Sensor_Type (pH and TDS) with no shared mutable state between them.
2. WHEN Calibration_Data is updated for one Sensor_Type, THE Calibration_Manager SHALL leave the other Sensor_Type's mode, slope, offset, valid flag, and updated_at at their prior values.
3. IF one Sensor_Type's calibration becomes invalid (due to validation failure, NVS write failure, or NVS corruption), THEN THE Calibration_Manager SHALL continue applying the existing valid calibration for the other Sensor_Type and continue publishing that Sensor_Type's Calibrated_Value without interruption.
4. IF an NVS operation fails for one Sensor_Type, THEN THE Calibration_Manager SHALL not modify the in-memory Calibration_Data or the valid flag of the other Sensor_Type.

### Requirement 10: Calibration State JSON Serialization

**User Story:** As a firmware developer, I want calibration state serialized to a well-defined JSON format, so that all MQTT consumers can parse it reliably.

#### Acceptance Criteria

1. WHEN publishing a valid calibration state, THE Calibration_Manager SHALL serialize the JSON containing exactly the fields in this order: mode (string), slope (number with 4 decimal places), offset (number with 4 decimal places), valid (boolean, set to true), and updated_at (unsigned 32-bit integer).
2. WHEN publishing a valid calibration state, THE Calibration_Manager SHALL format the JSON as: `{"mode":"linear","slope":<float 4dp>,"offset":<float 4dp>,"valid":true,"updated_at":<uint32>}`.
3. WHEN publishing an invalid calibration state with an error, THE Calibration_Manager SHALL serialize the JSON containing exactly the fields: mode (string, set to "linear"), slope (number, set to 0.0), offset (number, set to 0.0), valid (boolean, set to false), updated_at (unsigned 32-bit integer, set to 0 if no prior calibration exists or the last known value otherwise), and reason (string, maximum 128 characters, describing the specific validation or persistence failure).
4. THE Calibration_Manager SHALL produce JSON payloads not exceeding 256 bytes in total length.
5. THE Calibration_Manager SHALL ensure that for all valid Calibration_Data objects, serializing to JSON and parsing back produces a Calibration_Data object where slope and offset match the original values within ±0.00005 (half unit of last displayed decimal place), and mode, valid, and updated_at fields match exactly.
