#include <stdio.h>
#include <string.h>
#include <math.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "indicator_led.h"
#include "lcd_status.h"
#include "mqtt_manager.h"
#include "pin_config.h"
#include "runtime_safety.h"
#include "safety_config.h"
#include "sensor_telemetry.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

typedef struct {
    TickType_t on_tick;   /* tick at which the pump last turned ON */
    bool on;              /* true while the pump is running */
} dosing_watchdog_t;

typedef struct {
    atomic_uint faults;
    atomic_uint reported_faults;
    atomic_bool safe_mode;
    atomic_uint dosing_heartbeat;
    atomic_uint sensor_heartbeat;
    atomic_uint comm_heartbeat;
    atomic_uint safety_heartbeat;
    bool wdt_enabled;
    bool ph_response_pending;
    bool ph_response_up;
    TickType_t ph_response_deadline;
    float ph_response_start;
    bool tds_response_pending;
    TickType_t tds_response_deadline;
    float tds_response_start;
    safety_fault_mask_t tds_response_fault;
    TickType_t valve_on_tick;
    bool valve_waiting;
    safety_fault_mask_t boot_faults;
} runtime_safety_state_t;

typedef struct {
    safety_fault_mask_t mask;
    const char *code;
    const char *text;
} safety_fault_info_t;

static const char *TAG = "runtime_safety";

static portMUX_TYPE s_gate_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_state_shadow_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_safety_lock = portMUX_INITIALIZER_UNLOCKED;

static runtime_safety_bindings_t s_bindings = {0};
static runtime_safety_state_t s_safety = {0};
static dosing_watchdog_t s_dose_watchdog[ACTUATOR_CHANNEL_COUNT] = {0};
static bool s_channel_state_on[ACTUATOR_CHANNEL_COUNT] = {0};
static bool s_gate_open = false;

static const safety_fault_info_t s_fault_info[] = {
    { SAFETY_FAULT_DOSE_A, "F-001", "Dose A" },
    { SAFETY_FAULT_DOSE_B, "F-002", "Dose B" },
    { SAFETY_FAULT_PH_UP, "F-003", "pH Up" },
    { SAFETY_FAULT_PH_DOWN, "F-004", "pH Down" },
    { SAFETY_FAULT_PH_INTERLOCK, "F-005", "pH Interlock" },
    { SAFETY_FAULT_VALVE, "F-006", "Valve" },
    { SAFETY_FAULT_GROWLIGHT, "F-007", "Growlight" },
    { SAFETY_FAULT_TDS_SENSOR, "F-008", "TDS Sensor" },
    { SAFETY_FAULT_PH_SENSOR, "F-009", "pH Sensor" },
    { SAFETY_FAULT_I2C_BUS, "F-010", "I2C Bus" },
    { SAFETY_FAULT_WATER_TEMP, "F-011", "Water Temp" },
    { SAFETY_FAULT_POWER, "F-012", "Power" },
    { SAFETY_FAULT_WDT, "F-013", "Watchdog" },
};

static const safety_fault_info_t *find_fault_info(safety_fault_mask_t fault)
{
    for (size_t i = 0; i < ARRAY_SIZE(s_fault_info); i++) {
        if (s_fault_info[i].mask == fault) {
            return &s_fault_info[i];
        }
    }
    return NULL;
}

static int safety_fault_count(safety_fault_mask_t mask)
{
    return __builtin_popcount((unsigned int)mask);
}

static safety_fault_mask_t safety_first_fault(safety_fault_mask_t mask)
{
    return (mask != 0) ? (1u << (unsigned)__builtin_ctz((unsigned int)mask)) : 0;
}

static uint32_t ticks_to_ms(TickType_t ticks)
{
    return (uint32_t)(ticks * (TickType_t)portTICK_PERIOD_MS);
}

static void flush_dosing_queue(const char *reason)
{
    QueueHandle_t queue = (s_bindings.dosing_queue != NULL) ? *s_bindings.dosing_queue : NULL;
    if (queue == NULL) {
        return;
    }

    UBaseType_t dropped_count = uxQueueMessagesWaiting(queue);
    if (xQueueReset(queue) == pdTRUE && dropped_count > 0) {
        ESP_LOGW(TAG, "Dropped %u dosing command(s): %s",
                 (unsigned)dropped_count,
                 reason != NULL ? reason : "unknown reason");
    }
}

static void safety_publish_faults(safety_fault_mask_t new_faults)
{
    if (new_faults == 0) {
        return;
    }

    safety_fault_mask_t first = safety_first_fault(new_faults);
    const safety_fault_info_t *info = find_fault_info(first);
    const char *code = info ? info->code : "F-000";
    const char *text = info ? info->text : "Unknown";
    int count = safety_fault_count(new_faults);

    ESP_LOGE(TAG, "Safety fault %s: %s (new=%lu)", code, text, (unsigned long)new_faults);

    if (s_bindings.safety_fault_topic != NULL && s_bindings.safety_fault_topic[0] != '\0') {
        char payload[160];
        safety_fault_mask_t mask = (safety_fault_mask_t)atomic_load(&s_safety.faults);
        int written = snprintf(payload, sizeof(payload),
                               "{\"mask\":%lu,\"code\":\"%s\",\"count\":%d,\"text\":\"%s\"}",
                               (unsigned long)mask,
                               code,
                               count,
                               text);
        if (written > 0 && written < (int)sizeof(payload)) {
            mqtt_manager_publish(s_bindings.safety_fault_topic, payload, 1, 0);
        }
    }

    if (code != NULL && s_bindings.zone_id != NULL && s_bindings.zone_id[0] != '\0') {
        char line[32];
        if (count > 1) {
            snprintf(line, sizeof(line), "FAULT %s +%d", code, count - 1);
        } else {
            snprintf(line, sizeof(line), "FAULT %s", code);
        }
        lcd_status_show_zone_overview(s_bindings.zone_id, s_bindings.zone_name, line);
    }
}

static void safety_enter_safe_state(safety_fault_mask_t new_faults)
{
    atomic_store(&s_safety.safe_mode, true);

    portENTER_CRITICAL(&s_gate_lock);
    s_gate_open = false;
    portEXIT_CRITICAL(&s_gate_lock);

    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        (void)actuator_control_apply_state((actuator_channel_t)i, false);
        runtime_safety_update_channel_state((actuator_channel_t)i, false);
    }

    flush_dosing_queue("safe mode entry");

    portENTER_CRITICAL(&s_safety_lock);
    s_safety.ph_response_pending = false;
    s_safety.tds_response_pending = false;
    s_safety.valve_waiting = false;
    portEXIT_CRITICAL(&s_safety_lock);
}

static void safety_fault_set(safety_fault_mask_t mask, const char *reason)
{
    if (mask == 0) {
        return;
    }

    safety_fault_mask_t prev = (safety_fault_mask_t)atomic_fetch_or(&s_safety.faults, (unsigned int)mask);
    safety_fault_mask_t new_faults = mask & ~prev;
    if (new_faults == 0) {
        return;
    }

    safety_enter_safe_state(new_faults);

    if (reason != NULL) {
        ESP_LOGE(TAG, "Safety fault reason: %s", reason);
    }

    /* Light the fault LED for 5 s to give a physical indication */
    indicator_led_fault_blink(5000);

    safety_fault_mask_t reported = (safety_fault_mask_t)atomic_fetch_or(&s_safety.reported_faults, (unsigned int)new_faults);
    safety_fault_mask_t to_report = new_faults & ~reported;
    safety_publish_faults(to_report);
}

static uint32_t safety_max_dose_for_channel(actuator_channel_t channel)
{
    switch (channel) {
    case ACTUATOR_CHANNEL_PER_NUTA:
        return SAFETY_MAX_DOSE_A_MS;
    case ACTUATOR_CHANNEL_PER_NUTB:
        return SAFETY_MAX_DOSE_B_MS;
    case ACTUATOR_CHANNEL_PER_PH_UP:
        return SAFETY_MAX_DOSE_PH_UP_MS;
    case ACTUATOR_CHANNEL_PER_PH_DOWN:
        return SAFETY_MAX_DOSE_PH_DOWN_MS;
    default:
        return 0;
    }
}

static safety_fault_mask_t safety_fault_for_channel(actuator_channel_t channel)
{
    switch (channel) {
    case ACTUATOR_CHANNEL_PER_NUTA:
        return SAFETY_FAULT_DOSE_A;
    case ACTUATOR_CHANNEL_PER_NUTB:
        return SAFETY_FAULT_DOSE_B;
    case ACTUATOR_CHANNEL_PER_PH_UP:
        return SAFETY_FAULT_PH_UP;
    case ACTUATOR_CHANNEL_PER_PH_DOWN:
        return SAFETY_FAULT_PH_DOWN;
    default:
        return 0;
    }
}

void runtime_safety_bind(runtime_safety_bindings_t *bindings)
{
    if (bindings == NULL) {
        memset(&s_bindings, 0, sizeof(s_bindings));
        return;
    }

    s_bindings = *bindings;
}

void runtime_safety_set_dosing_queue(QueueHandle_t queue)
{
    if (queue == NULL) {
        flush_dosing_queue("queue reset");
    }

    if (s_bindings.dosing_queue != NULL) {
        *s_bindings.dosing_queue = queue;
    }
}

void runtime_safety_reset_state(void)
{
    atomic_store(&s_safety.faults, 0);
    atomic_store(&s_safety.reported_faults, 0);
    atomic_store(&s_safety.safe_mode, false);
    s_safety.wdt_enabled = false;
    if (s_bindings.stop_requested != NULL) {
        atomic_store(s_bindings.stop_requested, false);
    }
    s_safety.ph_response_pending = false;
    s_safety.tds_response_pending = false;
    s_safety.ph_response_up = false;
    s_safety.ph_response_deadline = 0;
    s_safety.ph_response_start = 0.0f;
    s_safety.tds_response_deadline = 0;
    s_safety.tds_response_start = 0.0f;
    s_safety.tds_response_fault = 0;
    s_safety.valve_on_tick = 0;
    s_safety.valve_waiting = false;
    s_safety.boot_faults = 0;
    portENTER_CRITICAL(&s_gate_lock);
    s_gate_open = false;
    portEXIT_CRITICAL(&s_gate_lock);
    memset(s_dose_watchdog, 0, sizeof(s_dose_watchdog));
    memset(s_channel_state_on, 0, sizeof(s_channel_state_on));
}

void runtime_safety_reset_heartbeats(void)
{
    atomic_store(&s_safety.dosing_heartbeat, 0);
    atomic_store(&s_safety.sensor_heartbeat, 0);
    atomic_store(&s_safety.comm_heartbeat, 0);
    atomic_store(&s_safety.safety_heartbeat, 0);
}

void runtime_safety_set_boot_faults(safety_fault_mask_t faults)
{
    s_safety.boot_faults = faults;
}

void runtime_safety_or_boot_faults(safety_fault_mask_t faults)
{
    s_safety.boot_faults |= faults;
}

safety_fault_mask_t runtime_safety_get_boot_faults(void)
{
    return s_safety.boot_faults;
}

void runtime_safety_apply_boot_faults(void)
{
    atomic_store(&s_safety.reported_faults, 0);
    if (s_safety.boot_faults != 0) {
        safety_fault_set(s_safety.boot_faults, "Boot fault");
    }

    safety_fault_mask_t existing_faults = (safety_fault_mask_t)atomic_load(&s_safety.faults);
    if (existing_faults != 0) {
        safety_enter_safe_state(existing_faults);
        safety_publish_faults(existing_faults);
    }
}

void runtime_safety_set_gate(bool open)
{
    portENTER_CRITICAL(&s_gate_lock);
    s_gate_open = open;
    portEXIT_CRITICAL(&s_gate_lock);
    if (!open) {
        flush_dosing_queue("gate closed");
    }
}

bool runtime_safety_is_gate_open(bool isr_context)
{
    bool gate_open = false;
    if (isr_context) {
        portENTER_CRITICAL_ISR(&s_gate_lock);
        gate_open = s_gate_open;
        portEXIT_CRITICAL_ISR(&s_gate_lock);
    } else {
        portENTER_CRITICAL(&s_gate_lock);
        gate_open = s_gate_open;
        portEXIT_CRITICAL(&s_gate_lock);
    }
    return gate_open;
}

void runtime_safety_update_channel_state(actuator_channel_t channel, bool state_on)
{
    if (channel < 0 || channel >= ACTUATOR_CHANNEL_COUNT) {
        return;
    }

    portENTER_CRITICAL(&s_state_shadow_lock);
    s_channel_state_on[channel] = state_on;
    portEXIT_CRITICAL(&s_state_shadow_lock);
}

bool runtime_safety_get_channel_state(actuator_channel_t channel)
{
    if (channel < 0 || channel >= ACTUATOR_CHANNEL_COUNT) {
        return false;
    }

    bool state_on = false;
    portENTER_CRITICAL(&s_state_shadow_lock);
    state_on = s_channel_state_on[channel];
    portEXIT_CRITICAL(&s_state_shadow_lock);
    return state_on;
}

void runtime_safety_copy_channel_states(bool out_states[ACTUATOR_CHANNEL_COUNT])
{
    if (out_states == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_state_shadow_lock);
    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        out_states[i] = s_channel_state_on[i];
    }
    portEXIT_CRITICAL(&s_state_shadow_lock);
}

void runtime_safety_wdt_init(void)
{
    if (s_safety.wdt_enabled) {
        return;
    }

    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = SAFETY_WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_err_t ret = esp_task_wdt_init(&wdt_cfg);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_safety.wdt_enabled = true;
    } else {
        ESP_LOGW(TAG, "Task WDT init failed: %s", esp_err_to_name(ret));
    }
}

void runtime_safety_wdt_register(void)
{
    if (!s_safety.wdt_enabled) {
        return;
    }

    esp_err_t ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Task WDT add failed: %s", esp_err_to_name(ret));
    }
}

void runtime_safety_wdt_kick(void)
{
    if (!s_safety.wdt_enabled) {
        return;
    }

    esp_task_wdt_reset();
}

void runtime_safety_wdt_unregister(void)
{
    if (s_safety.wdt_enabled) {
        esp_task_wdt_delete(NULL);
    }
}

void runtime_safety_heartbeat_dosing(void)
{
    atomic_fetch_add(&s_safety.dosing_heartbeat, 1u);
}

void runtime_safety_heartbeat_sensor(void)
{
    atomic_fetch_add(&s_safety.sensor_heartbeat, 1u);
}

void runtime_safety_heartbeat_comm(void)
{
    atomic_fetch_add(&s_safety.comm_heartbeat, 1u);
}

void runtime_safety_heartbeat_safety(void)
{
    atomic_fetch_add(&s_safety.safety_heartbeat, 1u);
}

void runtime_safety_dose_watchdog_update(actuator_channel_t channel, actuator_action_t action, uint32_t pulse_ms)
{
    uint32_t max_dose = safety_max_dose_for_channel(channel);
    if (max_dose == 0) {
        return;
    }

    dosing_watchdog_t *watchdog = &s_dose_watchdog[channel];
    TickType_t now = xTaskGetTickCount();

    switch (action) {
    case ACTUATOR_ACTION_ON:
        if (!watchdog->on) {
            watchdog->on = true;
            watchdog->on_tick = now;
        }
        break;
    case ACTUATOR_ACTION_OFF:
        watchdog->on = false;
        watchdog->on_tick = 0;
        break;
    case ACTUATOR_ACTION_PULSE:
        /* A pulse is a self-contained ON+OFF.  Fault immediately if
         * the requested duration already exceeds the maximum. */
        if (pulse_ms > max_dose) {
            safety_fault_mask_t fault = safety_fault_for_channel(channel);
            if (fault != 0) {
                safety_fault_set(fault, "Pulse duration exceeded max");
            }
        }
        break;
    default:
        break;
    }
}

/* Called from the safety task each loop to catch pumps that have been
 * left ON continuously past their maximum allowed duration. */
static void safety_check_dose_durations(TickType_t now)
{
    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        dosing_watchdog_t *watchdog = &s_dose_watchdog[i];
        if (!watchdog->on || watchdog->on_tick == 0) {
            continue;
        }

        uint32_t max_dose = safety_max_dose_for_channel((actuator_channel_t)i);
        if (max_dose == 0) {
            continue;
        }

        uint32_t on_duration_ms = ticks_to_ms(now - watchdog->on_tick);
        if (on_duration_ms > max_dose) {
            safety_fault_mask_t fault = safety_fault_for_channel((actuator_channel_t)i);
            if (fault != 0) {
                safety_fault_set(fault, "Pump on-duration exceeded max");
            }
        }
    }
}


void runtime_safety_note_ph_dose(actuator_channel_t channel)
{
    if (!SAFETY_ENABLE_PH_TDS_CHECKS) {
        return;
    }

    sensor_telemetry_snapshot_t snap;
    if (sensor_telemetry_get_snapshot(&snap) != ESP_OK || !snap.valid) {
        return;
    }

    portENTER_CRITICAL(&s_safety_lock);
    s_safety.ph_response_pending = true;
    s_safety.ph_response_up = (channel == ACTUATOR_CHANNEL_PER_PH_UP);
    s_safety.ph_response_start = snap.ph;
    s_safety.ph_response_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SAFETY_PH_RESPONSE_TIMEOUT_MS);
    portEXIT_CRITICAL(&s_safety_lock);
}

void runtime_safety_note_tds_dose(actuator_channel_t channel)
{
    if (!SAFETY_ENABLE_PH_TDS_CHECKS) {
        return;
    }

    safety_fault_mask_t fault = safety_fault_for_channel(channel);
    if (fault == 0) {
        return;
    }

    sensor_telemetry_snapshot_t snap;
    if (sensor_telemetry_get_snapshot(&snap) != ESP_OK || !snap.valid) {
        return;
    }

    portENTER_CRITICAL(&s_safety_lock);
    s_safety.tds_response_pending = true;
    s_safety.tds_response_start = snap.tds;
    s_safety.tds_response_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SAFETY_TDS_RESPONSE_TIMEOUT_MS);
    s_safety.tds_response_fault = fault;
    portEXIT_CRITICAL(&s_safety_lock);
}

void runtime_safety_note_valve_action(actuator_action_t action)
{
    if (action != ACTUATOR_ACTION_ON && action != ACTUATOR_ACTION_OFF) {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&s_safety_lock);
    if (action == ACTUATOR_ACTION_ON) {
        s_safety.valve_on_tick = now;
        s_safety.valve_waiting = true;
    } else {
        s_safety.valve_waiting = false;
    }
    portEXIT_CRITICAL(&s_safety_lock);
}

void runtime_safety_fault_set(safety_fault_mask_t mask, const char *reason)
{
    safety_fault_set(mask, reason);
}

safety_fault_mask_t runtime_safety_get_faults(void)
{
    return (safety_fault_mask_t)atomic_load(&s_safety.faults);
}

bool runtime_safety_is_safe_mode(void)
{
    return atomic_load(&s_safety.safe_mode);
}

esp_err_t runtime_safety_clear_faults(safety_fault_mask_t mask, bool *safe_mode_cleared)
{
    safety_fault_mask_t current = (safety_fault_mask_t)atomic_load(&s_safety.faults);
    if (mask == 0) {
        mask = current;
    }

    safety_fault_mask_t new_mask = current & ~mask;
    atomic_store(&s_safety.faults, (unsigned int)new_mask);
    atomic_store(&s_safety.reported_faults, (unsigned int)new_mask);

    if (mask == current) {
        s_safety.boot_faults = 0;
    } else {
        s_safety.boot_faults &= ~mask;
    }

    if (new_mask == 0) {
        atomic_store(&s_safety.safe_mode, false);
        portENTER_CRITICAL(&s_safety_lock);
        s_safety.ph_response_pending = false;
        s_safety.tds_response_pending = false;
        s_safety.valve_waiting = false;
        portEXIT_CRITICAL(&s_safety_lock);
        if (safe_mode_cleared != NULL) {
            *safe_mode_cleared = true;
        }
    }

    return ESP_OK;
}

static void safety_check_heartbeat(TaskHandle_t task, atomic_uint *counter,
                                   uint32_t *last_hb, TickType_t *last_change,
                                   TickType_t now, const char *reason)
{
    if (task == NULL || counter == NULL || last_hb == NULL || last_change == NULL || reason == NULL) {
        return;
    }

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

void runtime_safety_task(void *arg)
{
    (void)arg;
    runtime_safety_wdt_register();

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_dosing_hb = atomic_load(&s_safety.dosing_heartbeat);
    uint32_t last_sensor_hb = atomic_load(&s_safety.sensor_heartbeat);
    uint32_t last_comm_hb = atomic_load(&s_safety.comm_heartbeat);
    TickType_t dosing_last_change = last_wake;
    TickType_t sensor_last_change = last_wake;
    TickType_t comm_last_change = last_wake;

    float last_ph = 0.0f;
    float last_tds = 0.0f;
    float last_temp = 0.0f;
    bool last_ph_valid = false;
    bool last_tds_valid = false;
    bool last_temp_valid = false;
    int ph_frozen = 0;
    int tds_frozen = 0;
    int temp_frozen = 0;
    TickType_t last_ph_tick = last_wake;

    while (s_bindings.stop_requested != NULL && !atomic_load(s_bindings.stop_requested)) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SAFETY_INTERVAL_MS));
        TickType_t now = xTaskGetTickCount();

        runtime_safety_heartbeat_safety();
        runtime_safety_wdt_kick();

        safety_check_heartbeat(s_bindings.dosing_task ? *s_bindings.dosing_task : NULL,
                               &s_safety.dosing_heartbeat,
                               &last_dosing_hb, &dosing_last_change,
                               now, "Dosing task heartbeat stalled");
        safety_check_heartbeat(s_bindings.sensor_task ? *s_bindings.sensor_task : NULL,
                               &s_safety.sensor_heartbeat,
                               &last_sensor_hb, &sensor_last_change,
                               now, "Sensor task heartbeat stalled");
        safety_check_heartbeat(s_bindings.comm_task ? *s_bindings.comm_task : NULL,
                               &s_safety.comm_heartbeat,
                               &last_comm_hb, &comm_last_change,
                               now, "Comm task heartbeat stalled");

        safety_check_dose_durations(now);

        sensor_telemetry_snapshot_t snap = {0};
        bool snap_ok = (sensor_telemetry_get_snapshot(&snap) == ESP_OK && snap.valid);

        if (snap_ok) {
            /* Water level is now managed by the auto-fill state machine above.
             * Only temperature and pH/TDS checks remain here. */

            /* Graduated temperature response:
             *   > SAFETY_TEMP_HIGH     -> warning only (log, no fault)
             *   > SAFETY_TEMP_CRITICAL -> hard fault + safe mode
             *   < SAFETY_TEMP_LOW      -> hard fault + safe mode */
            if (snap.water_temp > SAFETY_TEMP_CRITICAL ||
                snap.water_temp < SAFETY_TEMP_LOW) {
                safety_fault_set(SAFETY_FAULT_WATER_TEMP, "Water temperature out of range");
            } else if (snap.water_temp > SAFETY_TEMP_HIGH) {
                ESP_LOGW(TAG, "Water temperature elevated: %.1f C (warn threshold %.1f C)",
                         snap.water_temp, (float)SAFETY_TEMP_HIGH);
            }

            if (last_temp_valid) {
                if (fabsf(snap.water_temp - last_temp) < SAFETY_TEMP_FROZEN_EPSILON) {
                    temp_frozen++;
                } else {
                    temp_frozen = 0;
                }
                if (temp_frozen >= SAFETY_TEMP_FROZEN_SAMPLES) {
                    safety_fault_set(SAFETY_FAULT_WATER_TEMP, "Water temperature frozen");
                }
            }
            last_temp = snap.water_temp;
            last_temp_valid = true;

            if (SAFETY_ENABLE_PH_TDS_CHECKS) {
                if (snap.ph < SAFETY_PH_MIN || snap.ph > SAFETY_PH_MAX) {
                    safety_fault_set(SAFETY_FAULT_PH_SENSOR, "pH out of range");
                }

                if (snap.ph > SAFETY_PH_CRITICAL_HIGH) {
                    safety_fault_set(SAFETY_FAULT_PH_UP, "pH critical high");
                } else if (snap.ph < SAFETY_PH_CRITICAL_LOW) {
                    safety_fault_set(SAFETY_FAULT_PH_DOWN, "pH critical low");
                }

                if (last_ph_valid) {
                    float delta = snap.ph - last_ph;
                    float dt_min = (float)ticks_to_ms(now - last_ph_tick) / 60000.0f;
                    if (dt_min > 0.0f) {
                        float rate = delta / dt_min;
                        if (rate > SAFETY_PH_RATE_MAX_PER_MIN) {
                            safety_fault_set(SAFETY_FAULT_PH_UP, "pH rising too fast");
                        } else if (rate < -SAFETY_PH_RATE_MAX_PER_MIN) {
                            safety_fault_set(SAFETY_FAULT_PH_DOWN, "pH dropping too fast");
                        }
                    }

                    if (fabsf(delta) < SAFETY_PH_FROZEN_EPSILON) {
                        ph_frozen++;
                    } else {
                        ph_frozen = 0;
                    }
                    if (ph_frozen >= SAFETY_PH_FROZEN_SAMPLES) {
                        safety_fault_set(SAFETY_FAULT_PH_SENSOR, "pH sensor frozen");
                    }
                }

                last_ph = snap.ph;
                last_ph_tick = now;
                last_ph_valid = true;

                if (snap.tds < SAFETY_TDS_MIN || snap.tds > SAFETY_TDS_MAX) {
                    safety_fault_set(SAFETY_FAULT_TDS_SENSOR, "TDS out of range");
                }

                if (last_tds_valid) {
                    if (fabsf(snap.tds - last_tds) < SAFETY_TDS_FROZEN_EPSILON) {
                        tds_frozen++;
                    } else {
                        tds_frozen = 0;
                    }
                    if (tds_frozen >= SAFETY_TDS_FROZEN_SAMPLES) {
                        safety_fault_set(SAFETY_FAULT_TDS_SENSOR, "TDS sensor frozen");
                    }
                }

                last_tds = snap.tds;
                last_tds_valid = true;

                bool ph_pending = false;
                bool ph_up = false;
                TickType_t ph_deadline = 0;
                float ph_start = 0.0f;
                portENTER_CRITICAL(&s_safety_lock);
                ph_pending = s_safety.ph_response_pending;
                ph_up = s_safety.ph_response_up;
                ph_deadline = s_safety.ph_response_deadline;
                ph_start = s_safety.ph_response_start;
                portEXIT_CRITICAL(&s_safety_lock);

                if (ph_pending) {
                    float delta = snap.ph - ph_start;
                    if ((ph_up && delta >= SAFETY_PH_RESPONSE_MIN_DELTA) ||
                        (!ph_up && delta <= -SAFETY_PH_RESPONSE_MIN_DELTA)) {
                        portENTER_CRITICAL(&s_safety_lock);
                        s_safety.ph_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    } else if ((int32_t)(now - ph_deadline) < 0) {
                        /* Deadline not reached yet. */
                    } else {
                        safety_fault_set(ph_up ? SAFETY_FAULT_PH_UP : SAFETY_FAULT_PH_DOWN, "pH response timeout");
                        portENTER_CRITICAL(&s_safety_lock);
                        s_safety.ph_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    }
                }

                bool tds_pending = false;
                TickType_t tds_deadline = 0;
                float tds_start = 0.0f;
                safety_fault_mask_t tds_fault = 0;
                portENTER_CRITICAL(&s_safety_lock);
                tds_pending = s_safety.tds_response_pending;
                tds_deadline = s_safety.tds_response_deadline;
                tds_start = s_safety.tds_response_start;
                tds_fault = s_safety.tds_response_fault;
                portEXIT_CRITICAL(&s_safety_lock);

                if (tds_pending) {
                    float delta = snap.tds - tds_start;
                    if (fabsf(delta) >= SAFETY_TDS_RESPONSE_MIN_DELTA) {
                        portENTER_CRITICAL(&s_safety_lock);
                        s_safety.tds_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    } else if ((int32_t)(now - tds_deadline) < 0) {
                        /* Deadline not reached yet. */
                    } else {
                        if (tds_fault != 0) {
                            safety_fault_set(tds_fault, "TDS response timeout");
                        }
                        portENTER_CRITICAL(&s_safety_lock);
                        s_safety.tds_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    }
                }
            }

            TickType_t valve_on_tick = 0;
            bool valve_waiting = false;
            portENTER_CRITICAL(&s_safety_lock);
            valve_on_tick = s_safety.valve_on_tick;
            valve_waiting = s_safety.valve_waiting;
            portEXIT_CRITICAL(&s_safety_lock);

            if (valve_waiting && valve_on_tick != 0 && snap.water_level == 0) {
                if (ticks_to_ms(now - valve_on_tick) > SAFETY_FILL_TIMEOUT_MS) {
                    safety_fault_set(SAFETY_FAULT_VALVE, "Valve fill timeout");
                    portENTER_CRITICAL(&s_safety_lock);
                    s_safety.valve_waiting = false;
                    portEXIT_CRITICAL(&s_safety_lock);
                }
            }
        }

        last_wake = now;
    }

    runtime_safety_wdt_unregister();
}