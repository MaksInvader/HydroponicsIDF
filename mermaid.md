# Finite State Machine — Website_UpdaterFreeRTOS Firmware

## 1. System Top-Level FSM

```mermaid
stateDiagram-v2
    [*] --> BOOT

    BOOT --> SETUP_MODE : nvs_flash_init OK\nwifi_manager_start_ap()\nweb_portal_start()
    BOOT --> RUNTIME_STARTING : web_portal_try_autostart_from_nvs()\n[saved config found]

    SETUP_MODE --> RUNTIME_STARTING : POST /configure\n[valid zone_id + broker_ip]

    RUNTIME_STARTING --> RUNTIME_RUNNING : runtime_tasks_start() OK\n[gate open, tasks spawned]
    RUNTIME_STARTING --> SETUP_MODE : runtime_tasks_start() FAIL\n[cleanup_runtime_locked()]

    RUNTIME_RUNNING --> SAFE_MODE : safety_fault_set()\n[any fault triggered]
    RUNTIME_RUNNING --> RUNTIME_STOPPING : runtime_tasks_stop()

    SAFE_MODE --> RUNTIME_RUNNING : on_safety_clear_command("MASK:...")\n[runtime_tasks_clear_safety_faults()]
    SAFE_MODE --> BOOT : on_safety_clear_command("CLEAR"|"ALL")\n[esp_restart()]

    RUNTIME_STOPPING --> SETUP_MODE : cleanup_runtime_locked() done

    RUNTIME_RUNNING --> OTA_IN_PROGRESS : on_ota_trigger("1")\n[ota_update_http_start()]
    OTA_IN_PROGRESS --> BOOT : OTA success → esp_restart()
    OTA_IN_PROGRESS --> RUNTIME_RUNNING : OTA failed\n[atomic in_progress = false]
```

---

## 2. Wi-Fi Manager FSM

```mermaid
stateDiagram-v2
    [*] --> WIFI_IDLE

    WIFI_IDLE --> AP_ONLY : wifi_manager_start_ap()\n[SoftAP started, 192.168.4.1]
    WIFI_IDLE --> STA_CONNECTING : wifi_manager_connect_sta()

    AP_ONLY --> AP_STA_CONNECTING : wifi_manager_connect_sta()\n[APSTA mode]

    AP_STA_CONNECTING --> AP_STA_CONNECTED : IP_EVENT_STA_GOT_IP\n[WIFI_CONNECTED_BIT set]
    AP_STA_CONNECTING --> AP_STA_CONNECTING : WIFI_EVENT_STA_DISCONNECTED\n[retry < MAXIMUM_RETRY(10)]
    AP_STA_CONNECTING --> AP_ONLY : retry == MAXIMUM_RETRY\n[WIFI_FAIL_BIT set]

    AP_STA_CONNECTED --> AP_STA_CONNECTING : WIFI_EVENT_STA_DISCONNECTED\n[auto-reconnect enabled]

    STA_CONNECTING --> STA_CONNECTED : IP_EVENT_STA_GOT_IP
    STA_CONNECTING --> WIFI_IDLE : retry == MAXIMUM_RETRY
    STA_CONNECTED --> STA_CONNECTING : WIFI_EVENT_STA_DISCONNECTED\n[auto-reconnect]
```

---

## 3. MQTT Manager FSM

```mermaid
stateDiagram-v2
    [*] --> MQTT_DISCONNECTED

    MQTT_DISCONNECTED --> MQTT_CONNECTING : mqtt_manager_init()\n[esp_mqtt_client_start()]
    MQTT_CONNECTING --> MQTT_CONNECTED : MQTT_EVENT_CONNECTED\n[MQTT_CONNECTED_BIT set]
    MQTT_CONNECTING --> MQTT_DISCONNECTED : MQTT_EVENT_DISCONNECTED

    MQTT_CONNECTED --> MQTT_CONNECTED : mqtt_manager_subscribe()\nmqtt_manager_publish()\nmqtt_manager_resubscribe_all()
    MQTT_CONNECTED --> MQTT_DISCONNECTED : MQTT_EVENT_DISCONNECTED\n[connection_cbs notified(false)]
    MQTT_DISCONNECTED --> MQTT_CONNECTING : MQTT_EVENT_BEFORE_CONNECT\n[broker auto-reconnect]

    MQTT_CONNECTED --> MQTT_CONNECTED : MQTT_EVENT_DATA\n[dispatch to subscriber callbacks]
```

---

## 4. Runtime Tasks FSM (Worker Tasks Lifecycle)

```mermaid
stateDiagram-v2
    [*] --> TASKS_IDLE

    TASKS_IDLE --> TASKS_STARTING : runtime_tasks_start(zone_id)\n[s_starting = true]

    TASKS_STARTING --> TASKS_RUNNING : runtime_setup_resources() OK\nruntime_setup_topics() OK\nruntime_start_workers() OK\n[gate open, s_running = true]

    TASKS_STARTING --> TASKS_IDLE : any setup step FAIL\n[cleanup_runtime_locked()]

    TASKS_RUNNING --> TASKS_STOPPING : runtime_tasks_stop()\n[s_stop_requested = true, gate closed]

    TASKS_STOPPING --> TASKS_IDLE : all tasks exit\n[TASK_EXIT_ALL_BITS set]\nor timeout + force delete

    state TASKS_RUNNING {
        [*] --> DOSING_TASK
        [*] --> SENSOR_TASK
        [*] --> COMM_TASK
        [*] --> SAFETY_TASK

        DOSING_TASK : Prio 3 — processes dosing queue\nON / OFF / PULSE actuators
        SENSOR_TASK : Prio 2 — samples sensors @ 1 s
        COMM_TASK   : Prio 1 — publishes telemetry @ 1 s\npublishes version @ 30 s
        SAFETY_TASK : Prio 4 — monitors heartbeats\nchecks sensor limits
    }
```

---

## 5. Dosing Task FSM (per command)

```mermaid
stateDiagram-v2
    [*] --> WAIT_COMMAND

    WAIT_COMMAND --> CHECK_GATE : xQueueReceive(dosing_queue)\n[command received]
    WAIT_COMMAND --> HEARTBEAT : timeout (200 ms)
    HEARTBEAT --> WAIT_COMMAND : runtime_safety_heartbeat_dosing()\nwdt_kick()

    CHECK_GATE --> DROP_COMMAND : safe_mode == true\nor stop_requested == true
    CHECK_GATE --> CHECK_INTERLOCK : gate open

    DROP_COMMAND --> WAIT_COMMAND

    CHECK_INTERLOCK --> FAULT_PH_INTERLOCK : pH_UP ON while pH_DOWN ON\nor pH_DOWN ON while pH_UP ON
    CHECK_INTERLOCK --> EXECUTE_COMMAND : interlock OK

    FAULT_PH_INTERLOCK --> WAIT_COMMAND : safety_fault_set(SAFETY_FAULT_PH_INTERLOCK)

    EXECUTE_COMMAND --> ACTUATOR_ON  : action == ON
    EXECUTE_COMMAND --> ACTUATOR_OFF : action == OFF
    EXECUTE_COMMAND --> ACTUATOR_PULSE : action == PULSE

    ACTUATOR_ON --> UPDATE_SAFETY : actuator_control_apply_state(ch, true)\nruntime_safety_update_channel_state()
    ACTUATOR_OFF --> UPDATE_SAFETY : actuator_control_apply_state(ch, false)\nruntime_safety_update_channel_state()
    ACTUATOR_PULSE --> UPDATE_SAFETY : apply_pulse(ch, ms)\nstate ON → OFF

    UPDATE_SAFETY --> NOTIFY_SAFETY : dose_watchdog_update()\nnote_valve_action()\nnote_ph_dose()\nnote_tds_dose()
    NOTIFY_SAFETY --> WAIT_COMMAND
```

---

## 6. Safety Task FSM

```mermaid
stateDiagram-v2
    [*] --> SAFETY_NORMAL

    SAFETY_NORMAL --> SAFETY_NORMAL : every SAFETY_INTERVAL_MS\ncheck heartbeats\ncheck dose durations\ncheck sensor limits

    SAFETY_NORMAL --> SAFE_MODE_ACTIVE : safety_fault_set(mask)\n[any threshold exceeded]

    state SAFETY_NORMAL {
        [*] --> MONITOR_HEARTBEATS
        MONITOR_HEARTBEATS --> FAULT_WDT : dosing/sensor/comm heartbeat stalled\n> SAFETY_HEARTBEAT_TIMEOUT_MS

        [*] --> MONITOR_TEMP
        MONITOR_TEMP --> FAULT_WATER_TEMP : temp > CRITICAL or temp < LOW\nor temp frozen N samples

        [*] --> MONITOR_PH
        MONITOR_PH --> FAULT_PH_SENSOR : pH out of range\nor pH frozen\nor pH rate too fast
        MONITOR_PH --> FAULT_PH_UP : pH critical high\nor pH rising too fast\nor pH response timeout
        MONITOR_PH --> FAULT_PH_DOWN : pH critical low\nor pH dropping too fast\nor pH response timeout

        [*] --> MONITOR_TDS
        MONITOR_TDS --> FAULT_TDS_SENSOR : TDS out of range\nor TDS frozen\nor TDS response timeout

        [*] --> MONITOR_VALVE
        MONITOR_VALVE --> FAULT_VALVE : valve ON but water_level==0\nafter SAFETY_FILL_TIMEOUT_MS

        [*] --> MONITOR_DOSE_DURATION
        MONITOR_DOSE_DURATION --> FAULT_DOSE : pump ON > max allowed ms
    }

    SAFE_MODE_ACTIVE --> SAFE_MODE_ACTIVE : all actuators OFF\ngate closed\ndosing queue flushed\nfault published via MQTT

    SAFE_MODE_ACTIVE --> SAFETY_NORMAL : runtime_tasks_clear_safety_faults()\n[all faults cleared, gate re-opened]
    SAFE_MODE_ACTIVE --> [*] : esp_restart()\n["CLEAR" command received]
```

---

## 7. OTA Update FSM

```mermaid
stateDiagram-v2
    [*] --> OTA_IDLE

    OTA_IDLE --> OTA_IDLE : on_ota_latest_version()\n[store latest version string]

    OTA_IDLE --> OTA_DOWNLOADING : on_ota_trigger("1")\n[ota_update_http_start(url)]\n[atomic in_progress = true]

    OTA_DOWNLOADING --> OTA_WRITING : HTTP 200 OK\nesp_ota_begin()

    OTA_WRITING --> OTA_FINALIZING : all chunks written\nesp_ota_end() OK

    OTA_WRITING --> OTA_FAILED : HTTP read error\nor write error\nor size exceeded

    OTA_FINALIZING --> OTA_REBOOTING : esp_ota_set_boot_partition() OK
    OTA_FINALIZING --> OTA_FAILED : esp_ota_end() FAIL\nor set_boot_partition() FAIL

    OTA_REBOOTING --> [*] : vTaskDelay(500ms) → esp_restart()

    OTA_FAILED --> OTA_IDLE : atomic in_progress = false\n[task deletes itself]
```

---

## 8. Valve Debounce FSM

```mermaid
stateDiagram-v2
    [*] --> VALVE_IDLE

    VALVE_IDLE --> VALVE_IDLE : ON command\n[no prior OFF — pass through]

    VALVE_IDLE --> VALVE_DEBOUNCE_WINDOW : OFF command received\n[record last_off_tick\ncancel pending ON]

    VALVE_DEBOUNCE_WINDOW --> VALVE_DEFERRED_ON : ON arrives within VALVE_DEBOUNCE_MS (3s)\n[swallow ON, start timer]

    VALVE_DEBOUNCE_WINDOW --> VALVE_IDLE : ON arrives after VALVE_DEBOUNCE_MS\n[pass through immediately]

    VALVE_DEBOUNCE_WINDOW --> VALVE_DEBOUNCE_WINDOW : another OFF arrives\n[reset timer, cancel pending ON]

    VALVE_DEFERRED_ON --> VALVE_IDLE : timer fires, no new OFF\n[enqueue deferred ON]

    VALVE_DEFERRED_ON --> VALVE_DEBOUNCE_WINDOW : new OFF arrives before timer\n[cancel deferred ON]
```

---

## 9. Boot Fault FSM

```mermaid
stateDiagram-v2
    [*] --> CHECK_RESET_REASON

    CHECK_RESET_REASON --> RECORD_WDT_FAULT : reset reason == TASK_WDT or WDT\n[s_wdt_reset_count++\nboot_faults |= SAFETY_FAULT_WDT]

    CHECK_RESET_REASON --> RECORD_POWER_FAULT : reset reason == BROWNOUT\n[boot_faults |= SAFETY_FAULT_POWER]

    CHECK_RESET_REASON --> CHECK_COUNTERS : reset reason == POWERON or UNKNOWN

    RECORD_WDT_FAULT --> CHECK_COUNTERS
    RECORD_POWER_FAULT --> CHECK_COUNTERS

    CHECK_COUNTERS --> RECORD_POWER_FAULT : s_reset_count > SAFETY_RESET_MAX_COUNT
    CHECK_COUNTERS --> RECORD_WDT_FAULT2 : s_wdt_reset_count > SAFETY_WDT_RESET_MAX_COUNT
    CHECK_COUNTERS --> BOOT_CLEAN : counters within limits

    RECORD_WDT_FAULT2 --> BOOT_CLEAN
    BOOT_CLEAN --> [*] : runtime_tasks_start()\n[apply_boot_faults() called]

    BOOT_CLEAN --> SAFE_MODE_ON_BOOT : boot_faults != 0\n[safety_fault_set(boot_faults)]
    SAFE_MODE_ON_BOOT --> [*] : gate stays closed\nfault published via MQTT
```

---

## 10. DosingTask — Core Loop FSM

```mermaid
stateDiagram-v2
    [*] --> DOSING_WAIT

    DOSING_WAIT --> DOSING_RECV : xQueueReceive() got command
    DOSING_WAIT --> DOSING_HB   : timeout 200 ms (no command)

    DOSING_HB --> DOSING_WAIT : heartbeat_dosing()\nwdt_kick()

    DOSING_RECV --> DOSING_DROP : safe_mode == true\nor stop_requested == true
    DOSING_RECV --> DOSING_INTERLOCK_CHECK : gate open

    DOSING_DROP --> DOSING_WAIT

    DOSING_INTERLOCK_CHECK --> DOSING_FAULT_INTERLOCK : pH_UP cmd while pH_DOWN ON\nor pH_DOWN cmd while pH_UP ON
    DOSING_INTERLOCK_CHECK --> DOSING_EXEC : interlock OK

    DOSING_FAULT_INTERLOCK --> DOSING_WAIT : fault_set(PH_INTERLOCK)

    DOSING_EXEC --> DOSING_ON    : action == ON
    DOSING_EXEC --> DOSING_OFF   : action == OFF
    DOSING_EXEC --> DOSING_PULSE : action == PULSE

    DOSING_ON    --> DOSING_POST : apply_state(ch, true)\nupdate_channel_state(ch, true)
    DOSING_OFF   --> DOSING_POST : apply_state(ch, false)\nupdate_channel_state(ch, false)
    DOSING_PULSE --> DOSING_POST : apply_pulse(ch, ms)\nstate ON then wait ms then OFF

    DOSING_POST --> DOSING_WAIT : dose_watchdog_update()\nnote_valve / ph / tds action\nheartbeat_dosing()\nwdt_kick()

    DOSING_WAIT --> [*] : stop_requested == true\n[signal TASK_EXIT_BIT_DOSING\nvTaskDelete()]
```

---

## 11. SensorTask — Core Loop FSM

```mermaid
stateDiagram-v2
    [*] --> SENSOR_SAMPLE

    SENSOR_SAMPLE --> SENSOR_DS18B20_TRIGGER : sensor_telemetry_sample()\n[trigger DS18B20 conversion]

    SENSOR_DS18B20_TRIGGER --> SENSOR_ADC_READ : read pH ADC\nread TDS ADC\n[apply calibration if valid]

    SENSOR_ADC_READ --> SENSOR_WATER_LEVEL : read water level GPIO

    SENSOR_WATER_LEVEL --> SENSOR_SNAPSHOT : update rolling averages\nbuild snapshot struct

    SENSOR_SNAPSHOT --> SENSOR_HB : snapshot stored\n[valid = true if all readings OK]

    SENSOR_HB --> SENSOR_DELAY : heartbeat_sensor()\nwdt_kick()

    SENSOR_DELAY --> SENSOR_SAMPLE : vTaskDelayUntil(1000 ms)

    SENSOR_SAMPLE --> [*] : stop_requested == true\n[signal TASK_EXIT_BIT_SENSOR\nvTaskDelete()]
```

---

## 12. CommTask — Core Loop FSM

```mermaid
stateDiagram-v2
    [*] --> COMM_LOOP

    COMM_LOOP --> COMM_VERSION_CHECK : every iteration

    COMM_VERSION_CHECK --> COMM_PUBLISH_VERSION : elapsed >= 30000 ms\n[publish_current_version(zone_id)]
    COMM_VERSION_CHECK --> COMM_GET_SNAPSHOT : not yet

    COMM_PUBLISH_VERSION --> COMM_GET_SNAPSHOT

    COMM_GET_SNAPSHOT --> COMM_PUBLISH_SENSORS : snapshot valid\n[cache to s_last_good_snapshot]
    COMM_GET_SNAPSHOT --> COMM_USE_CACHED : fresh snapshot invalid\n[use s_last_good_snapshot if available]
    COMM_GET_SNAPSHOT --> COMM_SKIP_SENSORS : no snapshot at all

    COMM_USE_CACHED --> COMM_PUBLISH_SENSORS
    COMM_SKIP_SENSORS --> COMM_LCD_CHECK

    COMM_PUBLISH_SENSORS --> COMM_LCD_CHECK : publish water_level\nwater_temp\npH raw/state/valid\nTDS raw/state/valid

    COMM_LCD_CHECK --> COMM_LCD_UPDATE : last_command changed\n[topic / command / state differs]
    COMM_LCD_CHECK --> COMM_HB : no change

    COMM_LCD_UPDATE --> COMM_HB : lcd_status_show_actuator_event()\n[update s_last_lcd_rendered_cmd]

    COMM_HB --> COMM_DELAY : heartbeat_comm()\nwdt_kick()

    COMM_DELAY --> COMM_LOOP : vTaskDelayUntil(1000 ms)

    COMM_LOOP --> [*] : stop_requested == true\n[signal TASK_EXIT_BIT_COMM\nvTaskDelete()]
```

---

## 13. SafetyTask — Core Loop FSM

```mermaid
stateDiagram-v2
    [*] --> SAFETY_LOOP

    SAFETY_LOOP --> SAFETY_HB : ulTaskNotifyTake(SAFETY_INTERVAL_MS)

    SAFETY_HB --> SAFETY_CHECK_HEARTBEATS : heartbeat_safety()\nwdt_kick()

    SAFETY_CHECK_HEARTBEATS --> SAFETY_FAULT_WDT : dosing/sensor/comm counter\nunchanged > HEARTBEAT_TIMEOUT_MS
    SAFETY_CHECK_HEARTBEATS --> SAFETY_CHECK_DOSE : heartbeats OK

    SAFETY_CHECK_DOSE --> SAFETY_FAULT_DOSE : any pump ON > max_dose_ms
    SAFETY_CHECK_DOSE --> SAFETY_GET_SNAPSHOT : durations OK

    SAFETY_GET_SNAPSHOT --> SAFETY_CHECK_TEMP : snap.valid == true
    SAFETY_GET_SNAPSHOT --> SAFETY_LOOP : snap invalid — skip checks

    SAFETY_CHECK_TEMP --> SAFETY_FAULT_TEMP : temp > CRITICAL or temp < LOW\nor temp frozen >= N samples
    SAFETY_CHECK_TEMP --> SAFETY_CHECK_PH : temp OK

    SAFETY_CHECK_PH --> SAFETY_FAULT_PH : pH out of range\nor frozen\nor rate too fast\nor response timeout
    SAFETY_CHECK_PH --> SAFETY_CHECK_TDS : pH OK

    SAFETY_CHECK_TDS --> SAFETY_FAULT_TDS : TDS out of range\nor frozen\nor response timeout
    SAFETY_CHECK_TDS --> SAFETY_CHECK_VALVE : TDS OK

    SAFETY_CHECK_VALVE --> SAFETY_FAULT_VALVE : valve ON, water_level==0\nafter FILL_TIMEOUT_MS
    SAFETY_CHECK_VALVE --> SAFETY_LOOP : valve OK

    SAFETY_FAULT_WDT   --> SAFETY_LOOP : fault_set(FAULT_WDT)
    SAFETY_FAULT_DOSE  --> SAFETY_LOOP : fault_set(FAULT_DOSE_x)
    SAFETY_FAULT_TEMP  --> SAFETY_LOOP : fault_set(FAULT_WATER_TEMP)
    SAFETY_FAULT_PH    --> SAFETY_LOOP : fault_set(FAULT_PH_x)
    SAFETY_FAULT_TDS   --> SAFETY_LOOP : fault_set(FAULT_TDS_SENSOR)
    SAFETY_FAULT_VALVE --> SAFETY_LOOP : fault_set(FAULT_VALVE)

    SAFETY_LOOP --> [*] : stop_requested == true\n[signal TASK_EXIT_BIT_SAFETY\nvTaskDelete()]
```
