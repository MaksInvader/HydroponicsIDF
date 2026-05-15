# Implementation Plan: Calibration Scheme

## Overview

Refactor the existing scattered calibration logic into a consolidated `calibration_manager` module with a clean public API. The implementation proceeds incrementally: core types and pure functions first, then NVS persistence, MQTT integration, task wiring, and finally cleanup of the old code paths.

## Tasks

- [x] 1. Set up module structure and core types
  - [x] 1.1 Create `main/calibration_manager.h` with all public types and function declarations
    - Define `cali_sensor_type_t`, `cali_data_t`, `cali_error_t`, `cali_reading_t` enums/structs
    - Define plausible range constants (`CALI_PH_SLOPE_MIN`, etc.)
    - Define error reason string macros
    - Declare all public API functions as specified in the design
    - _Requirements: 9.1, 4.5, 4.6, 10.1, 10.2, 10.3_

  - [x] 1.2 Create `main/calibration_manager.c` with static state and skeleton implementations
    - Define static per-sensor `cali_data_t` array and `portMUX_TYPE s_cali_lock` spinlock
    - Define deferred-publish pending flags
    - Add `#include` dependencies (esp_log, cJSON, nvs, mqtt_manager, etc.)
    - Stub out all public functions with `ESP_LOGW` placeholders
    - _Requirements: 9.1_

  - [x] 1.3 Register `calibration_manager.c` in `main/CMakeLists.txt`
    - Add the new source file to the SRCS list
    - Ensure cJSON component dependency is present
    - _Requirements: (build infrastructure)_

- [ ] 2. Implement pure validation and computation functions
  - [-] 2.1 Implement `cali_mgr_validate_payload()`
    - Parse JSON with cJSON, check field presence (mode, slope, offset)
    - Verify finite/non-zero slope, mode == "linear"
    - Apply per-sensor-type range checks on slope and offset
    - Return specific `cali_error_t` code on first failure
    - Populate output `cali_data_t` only on success
    - _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8_

  - [ ]* 2.2 Write property test for payload validation — range acceptance
    - **Property 1: Range validation accepts only in-range coefficients**
    - **Validates: Requirements 1.7, 4.5, 4.6**

  - [ ]* 2.3 Write property test for payload validation — invalid rejection
    - **Property 2: Invalid payloads are always rejected**
    - **Validates: Requirements 4.1, 4.2, 4.3, 4.4**

  - [ ]* 2.4 Write property test for validation ordering
    - **Property 3: Validation stops at first failure**
    - **Validates: Requirements 4.7**

  - [-] 2.5 Implement `cali_mgr_compute()`
    - Compute `(raw * slope) + offset`
    - Check result is finite and within `[val_min, val_max]`
    - Return true/false for validity, write computed value to output pointer
    - _Requirements: 7.1, 7.2_

  - [ ]* 2.6 Write property test for linear computation
    - **Property 5: Linear computation correctness**
    - **Validates: Requirements 7.1**

- [ ] 3. Implement serialization and deserialization
  - [-] 3.1 Implement `cali_mgr_serialize()`
    - Format JSON with fixed field order: mode, slope (4dp), offset (4dp), valid, updated_at
    - Append `reason` field only when provided (error states)
    - Enforce 256-byte output limit
    - Use `snprintf` with `"%.4f"` formatting for slope/offset
    - _Requirements: 10.1, 10.2, 10.3, 10.4_

  - [-] 3.2 Implement `cali_mgr_deserialize()`
    - Parse JSON with cJSON, extract mode, slope, offset, valid, updated_at
    - Return false on parse failure or missing fields
    - _Requirements: 10.5_

  - [ ]* 3.3 Write property test for serialization round-trip
    - **Property 9: Serialization round-trip fidelity**
    - **Validates: Requirements 10.5**

  - [ ]* 3.4 Write property test for serialization format and size
    - **Property 10: Serialization format and size constraints**
    - **Validates: Requirements 10.1, 10.2, 10.3, 10.4**

- [~] 4. Checkpoint — Ensure pure function tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [ ] 5. Implement NVS persistence layer
  - [~] 5.1 Implement `sensor_calibration_nvs_save_verified()` in `main/sensor_calibration_nvs.c`
    - Write slope, offset, valid, updated_at atomically via `nvs_commit()`
    - Read back slope and offset after write
    - Compare read-back values bit-for-bit (memcmp on uint32 IEEE-754 representation)
    - Return `ESP_ERR_INVALID_CRC` on mismatch
    - Add declaration to `main/sensor_calibration_nvs.h`
    - _Requirements: 5.2, 8.4_

  - [~] 5.2 Implement NVS save with retry in `calibration_manager.c` (internal helper)
    - Call `sensor_calibration_nvs_save_verified()` up to 3 attempts
    - On exhaustion, return `CALI_ERR_NVS_WRITE` or `CALI_ERR_NVS_VERIFY`
    - _Requirements: 5.3, 5.4_

  - [~] 5.3 Implement NVS boot-time load in `cali_mgr_init()`
    - Open NVS namespace, read slope/offset/valid/updated_at for each sensor type
    - Validate loaded data against plausible ranges
    - Set in-memory state accordingly (valid or invalid)
    - Set deferred-publish pending flag
    - _Requirements: 1.1, 1.2, 1.6, 1.7_

  - [ ]* 5.4 Write property test for NVS write-then-read-back integrity
    - **Property 11: NVS write-then-read-back integrity**
    - **Validates: Requirements 8.4**

  - [ ]* 5.5 Write unit tests for NVS boot-time load scenarios
    - Test: valid data loaded successfully
    - Test: no data found (first boot)
    - Test: NVS read error
    - Test: data out of plausible range
    - _Requirements: 1.1, 1.2, 1.4, 1.6, 1.7_

- [ ] 6. Implement MQTT integration
  - [~] 6.1 Implement MQTT subscription logic in `cali_mgr_init()` and `cali_mgr_on_mqtt_connected()`
    - Subscribe to `<zone_id>/calibration/pH/set` and `<zone_id>/calibration/TDS/set` with QoS 1
    - Retry subscription up to 3 times at 2-second intervals on failure
    - Publish error to calibration state topic if all retries exhausted
    - Re-subscribe on reconnection via `cali_mgr_on_mqtt_connected()`
    - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5_

  - [~] 6.2 Implement `cali_mgr_on_set_cb()` — the MQTT calibration command callback
    - Determine sensor type from topic string
    - Call `cali_mgr_validate_payload()`; on failure, publish error state and return
    - On success: call NVS save with retry, update in-memory state under spinlock
    - Publish valid state (retained) and valid=true to sensor valid topic
    - _Requirements: 4.7, 4.8, 5.1, 5.2, 6.1, 6.2, 6.3_

  - [~] 6.3 Implement `cali_mgr_publish_reading()` and `cali_mgr_publish_deferred()`
    - `publish_reading`: publish raw always, state only if valid and in-range, valid as retained
    - `publish_deferred`: check pending flag, publish boot-time state if MQTT connected
    - Retry publish up to 3 times at 1-second intervals on failure
    - _Requirements: 1.3, 1.4, 1.5, 1.8, 2.1, 2.2, 6.4, 7.3, 7.4, 7.5_

  - [ ]* 6.4 Write property test for state publication gating
    - **Property 6: State publication gated on validity and range**
    - **Validates: Requirements 7.2, 7.3, 7.4**

  - [ ]* 6.5 Write property test for raw reading always published
    - **Property 7: Raw reading always published**
    - **Validates: Requirements 2.2, 8.3**

  - [ ]* 6.6 Write unit tests for MQTT subscription retry and error publication
    - Test: successful subscribe on first attempt
    - Test: subscribe fails then succeeds on retry
    - Test: all retries exhausted, error published
    - _Requirements: 3.4, 3.5_

- [ ] 7. Implement concurrency and sensor independence
  - [~] 7.1 Implement `cali_mgr_apply_reading()` with spinlock-protected state access
    - Copy calibration data under `portENTER_CRITICAL` / `portEXIT_CRITICAL`
    - Call `cali_mgr_compute()` with copied data outside critical section
    - Return `cali_reading_t` with raw, calibrated, valid, and raw_ok fields
    - _Requirements: 7.1, 7.2, 9.1_

  - [~] 7.2 Implement `cali_mgr_get_calibration()` and `cali_mgr_is_valid()` with spinlock
    - Thread-safe struct copy for external consumers
    - _Requirements: 9.1_

  - [ ]* 7.3 Write property test for sensor type independence
    - **Property 8: Sensor type independence**
    - **Validates: Requirements 9.1, 9.2, 9.3, 9.4**

  - [ ]* 7.4 Write unit tests for concurrent access scenarios
    - Test: calibration update during sensor read does not corrupt data
    - Test: one sensor invalidation does not affect the other
    - _Requirements: 9.2, 9.3, 9.4_

- [~] 8. Checkpoint — Ensure all module tests pass
  - Ensure all tests pass, ask the user if questions arise.

- [ ] 9. Wire into existing task architecture
  - [~] 9.1 Integrate `cali_mgr_init()` into system startup (`main.c` or `runtime_tasks.c`)
    - Call `cali_mgr_init(zone_id)` after NVS init and before task creation
    - Register `cali_mgr_on_mqtt_connected()` with the MQTT manager's connected callback
    - _Requirements: 1.1, 1.8, 3.3_

  - [~] 9.2 Integrate `cali_mgr_apply_reading()` into `sensor_task` sampling loop
    - Replace existing calibration application logic with `cali_mgr_apply_reading()` calls
    - Store `cali_reading_t` results for comm_task consumption
    - _Requirements: 7.1, 2.1_

  - [~] 9.3 Integrate `cali_mgr_publish_reading()` and `cali_mgr_publish_deferred()` into `comm_task`
    - Replace existing calibration publish logic with new API calls
    - Call `cali_mgr_publish_deferred()` on each comm_task cycle
    - _Requirements: 1.3, 2.1, 2.2, 6.1, 7.3_

  - [~] 9.4 Remove old calibration logic from `sensor_telemetry.c` and `comm_task`
    - Remove inline calibration loading, validation, and publication code
    - Keep hardware init, ADC reading, rolling average, and snapshot logic
    - Update includes and function signatures as needed
    - _Requirements: (refactoring cleanup)_

- [ ] 10. Implement error handling and NVS corruption detection
  - [~] 10.1 Implement NVS corruption detection and reporting in `cali_mgr_on_set_cb()`
    - On read-back verification failure: mark in-memory invalid, publish error state
    - Ensure raw publishing continues unaffected
    - _Requirements: 8.1, 8.2, 8.3_

  - [~] 10.2 Implement `cali_mgr_deinit()` for clean shutdown
    - Unsubscribe from MQTT topics
    - Clear in-memory state
    - _Requirements: (resource cleanup)_

  - [ ]* 10.3 Write property test for failed validation never triggers NVS write
    - **Property 4: Failed validation never triggers NVS write**
    - **Validates: Requirements 4.8**

  - [ ]* 10.4 Write unit tests for NVS corruption flow
    - Test: write succeeds but read-back mismatches → invalid state published
    - Test: raw continues publishing after corruption
    - _Requirements: 8.1, 8.2, 8.3, 8.4_

- [~] 11. Final checkpoint — Ensure all tests pass
  - Ensure all tests pass, ask the user if questions arise.

## Notes

- Tasks marked with `*` are optional and can be skipped for faster MVP
- Each task references specific requirements for traceability
- Checkpoints ensure incremental validation
- Property tests validate universal correctness properties from the design document
- Unit tests validate specific examples and edge cases
- The implementation language is C (ESP-IDF / FreeRTOS) as specified in the design
- Property-based tests use the [theft](https://github.com/silentbicycle/theft) library for C
- Host-based tests (pure functions) run on x86; integration tests run on target hardware or QEMU

## Task Dependency Graph

```json
{
  "waves": [
    { "id": 0, "tasks": ["1.1"] },
    { "id": 1, "tasks": ["1.2", "1.3"] },
    { "id": 2, "tasks": ["2.1", "2.5", "3.1", "3.2"] },
    { "id": 3, "tasks": ["2.2", "2.3", "2.4", "2.6", "3.3", "3.4"] },
    { "id": 4, "tasks": ["5.1"] },
    { "id": 5, "tasks": ["5.2", "5.3"] },
    { "id": 6, "tasks": ["5.4", "5.5", "6.1"] },
    { "id": 7, "tasks": ["6.2", "6.3"] },
    { "id": 8, "tasks": ["6.4", "6.5", "6.6", "7.1", "7.2"] },
    { "id": 9, "tasks": ["7.3", "7.4"] },
    { "id": 10, "tasks": ["9.1"] },
    { "id": 11, "tasks": ["9.2", "9.3"] },
    { "id": 12, "tasks": ["9.4", "10.1", "10.2"] },
    { "id": 13, "tasks": ["10.3", "10.4"] }
  ]
}
```
