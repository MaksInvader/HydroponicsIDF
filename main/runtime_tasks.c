#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdatomic.h>
#include <math.h>
#include <strings.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "actuator_control.h"
#include "lcd_status.h"
#include "mqtt_manager.h"
#include "ota_update.h"
#include "pin_config.h"
#include "runtime_tasks.h"
#include "safety_config.h"
#include "sensor_telemetry.h"
#include "setup_config.h"
#include "zone_config.h"

#define DOSING_TASK_STACK 4096
#define SENSOR_TASK_STACK 6144
#define COMM_TASK_STACK 8192

#define DOSING_TASK_PRIORITY 3
#define SENSOR_TASK_PRIORITY 2
#define COMM_TASK_PRIORITY 1

#define SENSOR_SAMPLE_INTERVAL_MS 1000
#define COMM_INTERVAL_MS 1000
#define DOSING_QUEUE_RECV_TIMEOUT_MS 200

#define DOSING_QUEUE_LEN 16

#define ZONE_COMMAND_BUFFER_SIZE 120
#define ZONE_COMMAND_MAX_LEN (ZONE_COMMAND_BUFFER_SIZE - 1)
#define ZONE_TOPIC_BUFFER_SIZE 96

#define OTA_HTTP_PORT_DEFAULT 8123
#define OTA_HTTP_PATH_DEFAULT "/local/firmware/lorong_node.bin"
#define OTA_VERSION_MAX_LEN 32

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define SAFE_STRCPY(dst, src) do { \
    strncpy((dst), (src), sizeof(dst) - 1); \
    (dst)[sizeof(dst) - 1] = '\0'; \
} while (0)

#define RUNTIME_MUTEX_WAIT_MS 5000
#define RUNTIME_STATE_QUERY_WAIT_MS 100
#define TASK_EXIT_WAIT_MS 5000
#define SNAPSHOT_MUTEX_WAIT_MS 100

#define TASK_EXIT_BIT_DOSING BIT0
#define TASK_EXIT_BIT_SENSOR BIT1
#define TASK_EXIT_BIT_COMM BIT2
#define TASK_EXIT_BIT_SAFETY BIT3
#define TASK_EXIT_ALL_BITS (TASK_EXIT_BIT_DOSING | TASK_EXIT_BIT_SENSOR | TASK_EXIT_BIT_COMM | TASK_EXIT_BIT_SAFETY)

typedef struct {
    actuator_channel_t channel;
    actuator_action_t action;
    uint32_t pulse_ms;
} dosing_command_t;

typedef enum {
    SAFETY_FAULT_DOSE_A = 1u << 0,
    SAFETY_FAULT_DOSE_B = 1u << 1,
    SAFETY_FAULT_PH_UP = 1u << 2,
    SAFETY_FAULT_PH_DOWN = 1u << 3,
    SAFETY_FAULT_PH_INTERLOCK = 1u << 4,
    SAFETY_FAULT_VALVE = 1u << 5,
    SAFETY_FAULT_GROWLIGHT = 1u << 6,
    SAFETY_FAULT_TDS_SENSOR = 1u << 7,
    SAFETY_FAULT_PH_SENSOR = 1u << 8,
    SAFETY_FAULT_I2C_BUS = 1u << 9,
    SAFETY_FAULT_WATER_TEMP = 1u << 10,
    SAFETY_FAULT_WATER_LEVEL = 1u << 11,
    SAFETY_FAULT_FILL_TIMEOUT = 1u << 12,
    SAFETY_FAULT_POWER = 1u << 13,
    SAFETY_FAULT_WDT = 1u << 14,
} safety_fault_t;

typedef struct {
    TickType_t window_start;
    TickType_t on_tick;
    uint32_t on_time_ms;
    bool on;
} dosing_watchdog_t;

static const char *TAG = "runtime_tasks";

static SemaphoreHandle_t s_runtime_mutex = NULL;
static SemaphoreHandle_t s_snapshot_mutex = NULL;
static portMUX_TYPE s_runtime_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_cmd_gate_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_task_handle_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_state_shadow_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_safety_lock = portMUX_INITIALIZER_UNLOCKED;
static EventGroupHandle_t s_task_exit_event = NULL;
static TaskHandle_t s_dosing_task = NULL;
static TaskHandle_t s_sensor_task = NULL;
static TaskHandle_t s_comm_task = NULL;
static TaskHandle_t s_safety_task = NULL;
static QueueHandle_t s_dosing_queue = NULL;
static bool s_running;
static bool s_starting;
static atomic_bool s_stop_requested;
static volatile bool s_gate_open;
static atomic_uint s_safety_faults;
static atomic_uint s_reported_faults;
static atomic_bool s_safe_mode;
static bool s_zone_topic_subscribed;
static bool s_channel_topics_subscribed[ACTUATOR_CHANNEL_COUNT];
static bool s_safety_topic_subscribed;
static bool s_ota_latest_subscribed;
static bool s_ota_trigger_subscribed;
static char s_zone_command_topic[ZONE_TOPIC_BUFFER_SIZE];
static char s_safety_fault_topic[ZONE_TOPIC_BUFFER_SIZE];
static char s_safety_clear_topic[ZONE_TOPIC_BUFFER_SIZE];
static char s_ota_latest_topic[ZONE_TOPIC_BUFFER_SIZE];
static char s_ota_trigger_topic[ZONE_TOPIC_BUFFER_SIZE];
static char s_ota_latest_version[OTA_VERSION_MAX_LEN];
static char s_zone_id[ZONE_ID_MAX_LEN + 1];
static char s_zone_name[ZONE_NAME_MAX_LEN + 1];
static actuator_last_command_t s_last_lcd_rendered_cmd;
static sensor_telemetry_snapshot_t s_last_good_snapshot;
static bool s_last_good_snapshot_valid;
static bool s_channel_state_on[ACTUATOR_CHANNEL_COUNT];
static dosing_watchdog_t s_dose_watchdog[ACTUATOR_CHANNEL_COUNT];
static atomic_uint s_dosing_heartbeat;
static atomic_uint s_sensor_heartbeat;
static atomic_uint s_comm_heartbeat;
static atomic_uint s_safety_heartbeat;
static atomic_bool s_water_level_low_isr;
static volatile TickType_t s_water_level_low_tick;
static bool s_isr_installed;
static bool s_wdt_enabled;
static bool s_ph_response_pending;
static bool s_ph_response_up;
static TickType_t s_ph_response_deadline;
static float s_ph_response_start;
static bool s_tds_response_pending;
static TickType_t s_tds_response_deadline;
static float s_tds_response_start;
static safety_fault_mask_t s_tds_response_fault;
static TickType_t s_valve_on_tick;
static bool s_valve_waiting;
static safety_fault_mask_t s_boot_faults;
RTC_DATA_ATTR static uint32_t s_reset_count;
RTC_DATA_ATTR static uint32_t s_wdt_reset_count;

static esp_err_t runtime_lock(void)
{
    if (s_runtime_mutex == NULL) {
        SemaphoreHandle_t created_mutex = xSemaphoreCreateMutex();
        if (created_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }

        portENTER_CRITICAL(&s_runtime_mutex_init_lock);
        if (s_runtime_mutex == NULL) {
            s_runtime_mutex = created_mutex;
            created_mutex = NULL;
        }
        portEXIT_CRITICAL(&s_runtime_mutex_init_lock);

        if (created_mutex != NULL) {
            vSemaphoreDelete(created_mutex);
        }
    }

    if (xSemaphoreTake(s_runtime_mutex, pdMS_TO_TICKS(RUNTIME_MUTEX_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void runtime_unlock(void)
{
    if (s_runtime_mutex != NULL) {
        xSemaphoreGive(s_runtime_mutex);
    }
}

static void clear_subscription_tracking(void)
{
    s_zone_topic_subscribed = false;
    memset(s_channel_topics_subscribed, 0, sizeof(s_channel_topics_subscribed));
    s_safety_topic_subscribed = false;
    s_ota_latest_subscribed = false;
    s_ota_trigger_subscribed = false;
}

static void signal_task_exit(EventBits_t bit)
{
    if (s_task_exit_event != NULL) {
        xEventGroupSetBits(s_task_exit_event, bit);
    }
}

static TaskHandle_t take_task_handle(TaskHandle_t *handle)
{
    TaskHandle_t task = NULL;

    if (handle == NULL) {
        return NULL;
    }

    portENTER_CRITICAL(&s_task_handle_lock);
    task = *handle;
    *handle = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);

    return task;
}

static void safety_heartbeat(atomic_uint *counter)
{
    if (counter != NULL) {
        atomic_fetch_add(counter, 1u);
    }
}

typedef struct {
    TaskHandle_t *handle;
    EventBits_t exit_bit;
} runtime_task_slot_t;

static const runtime_task_slot_t s_task_slots[] = {
    { &s_comm_task, TASK_EXIT_BIT_COMM },
    { &s_safety_task, TASK_EXIT_BIT_SAFETY },
    { &s_sensor_task, TASK_EXIT_BIT_SENSOR },
    { &s_dosing_task, TASK_EXIT_BIT_DOSING },
};

static const atomic_uint *const s_heartbeat_counters[] = {
    &s_dosing_heartbeat,
    &s_sensor_heartbeat,
    &s_comm_heartbeat,
    &s_safety_heartbeat,
};

static void reset_heartbeats(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(s_heartbeat_counters); i++) {
        atomic_store((atomic_uint *)s_heartbeat_counters[i], 0);
    }
}

static void update_channel_state_on(actuator_channel_t channel, bool state_on)
{
    if (channel < 0 || channel >= ACTUATOR_CHANNEL_COUNT) {
        return;
    }

    portENTER_CRITICAL(&s_state_shadow_lock);
    s_channel_state_on[channel] = state_on;
    portEXIT_CRITICAL(&s_state_shadow_lock);
}

static bool get_channel_state_on(actuator_channel_t channel)
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

typedef struct {
    safety_fault_mask_t mask;
    const char *code;
    const char *text;
} safety_fault_info_t;

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
    { SAFETY_FAULT_WATER_LEVEL, "F-012", "Water Level" },
    { SAFETY_FAULT_FILL_TIMEOUT, "F-013", "Fill Timeout" },
    { SAFETY_FAULT_POWER, "F-014", "Power" },
    { SAFETY_FAULT_WDT, "F-015", "Watchdog" },
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

    if (s_safety_fault_topic[0] != '\0') {
        char payload[160];
        safety_fault_mask_t mask = (safety_fault_mask_t)atomic_load(&s_safety_faults);
        int written = snprintf(payload, sizeof(payload),
                               "{\"mask\":%lu,\"code\":\"%s\",\"count\":%d,\"text\":\"%s\"}",
                               (unsigned long)mask,
                               code,
                               count,
                               text);
        if (written > 0 && written < (int)sizeof(payload)) {
            mqtt_manager_publish(s_safety_fault_topic, payload, 1, 0);
        }
    }

    if (code != NULL && s_zone_id[0] != '\0') {
        char line[32];
        if (count > 1) {
            snprintf(line, sizeof(line), "FAULT %s +%d", code, count - 1);
        } else {
            snprintf(line, sizeof(line), "FAULT %s", code);
        }
        lcd_status_show_zone_overview(s_zone_id, s_zone_name, line);
    }
}

static void safety_enter_safe_state(safety_fault_mask_t new_faults)
{
    (void)new_faults;
    atomic_store(&s_safe_mode, true);

    portENTER_CRITICAL(&s_cmd_gate_lock);
    s_gate_open = false;
    portEXIT_CRITICAL(&s_cmd_gate_lock);

    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        (void)actuator_control_apply_state((actuator_channel_t)i, false);
        update_channel_state_on((actuator_channel_t)i, false);
    }

    portENTER_CRITICAL(&s_safety_lock);
    s_ph_response_pending = false;
    s_tds_response_pending = false;
    s_valve_waiting = false;
    portEXIT_CRITICAL(&s_safety_lock);
}

static void safety_fault_set(safety_fault_mask_t mask, const char *reason)
{
    if (mask == 0) {
        return;
    }

    safety_fault_mask_t prev = (safety_fault_mask_t)atomic_fetch_or(&s_safety_faults, (unsigned int)mask);
    safety_fault_mask_t new_faults = mask & ~prev;
    if (new_faults == 0) {
        return;
    }

    safety_enter_safe_state(new_faults);

    if (reason != NULL) {
        ESP_LOGE(TAG, "Safety fault reason: %s", reason);
    }

    safety_fault_mask_t reported = (safety_fault_mask_t)atomic_fetch_or(&s_reported_faults, (unsigned int)new_faults);
    safety_fault_mask_t to_report = new_faults & ~reported;
    safety_publish_faults(to_report);
}

static void safety_wdt_init(void)
{
    if (s_wdt_enabled) {
        return;
    }

    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = SAFETY_WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_err_t ret = esp_task_wdt_init(&wdt_cfg);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_wdt_enabled = true;
    } else {
        ESP_LOGW(TAG, "Task WDT init failed: %s", esp_err_to_name(ret));
    }
}

static void safety_wdt_register(void)
{
    if (!s_wdt_enabled) {
        return;
    }

    esp_err_t ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Task WDT add failed: %s", esp_err_to_name(ret));
    }
}

static void safety_wdt_kick(void)
{
    if (!s_wdt_enabled) {
        return;
    }

    esp_task_wdt_reset();
}

static void safety_reset_dose_window(dosing_watchdog_t *watchdog, TickType_t now)
{
    if (watchdog == NULL) {
        return;
    }

    watchdog->window_start = now;
    watchdog->on_time_ms = 0;
    if (watchdog->on) {
        watchdog->on_tick = now;
    }
}

static uint32_t ticks_to_ms(TickType_t ticks)
{
    return (uint32_t)(ticks * (TickType_t)portTICK_PERIOD_MS);
}

static EventBits_t task_exit_bits_for_running_tasks(void)
{
    EventBits_t wait_bits = 0;

    for (size_t i = 0; i < ARRAY_SIZE(s_task_slots); i++) {
        if (*s_task_slots[i].handle != NULL) {
            wait_bits |= s_task_slots[i].exit_bit;
        }
    }

    return wait_bits;
}

static void delete_running_tasks(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(s_task_slots); i++) {
        TaskHandle_t task_handle = take_task_handle(s_task_slots[i].handle);
        if (task_handle != NULL) {
            vTaskDelete(task_handle);
        }
    }
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

static void IRAM_ATTR water_level_isr(void *arg)
{
    (void)arg;
    int level = gpio_get_level((gpio_num_t)PIN_SENSOR_WATER_LEVEL);
    if (level == 0) {
        s_water_level_low_tick = xTaskGetTickCountFromISR();
        atomic_store(&s_water_level_low_isr, true);

        BaseType_t higher_priority_task_woken = pdFALSE;
        if (s_safety_task != NULL) {
            vTaskNotifyGiveFromISR(s_safety_task, &higher_priority_task_woken);
        }
        if (higher_priority_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
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

static void dose_watchdog_update(actuator_channel_t channel, actuator_action_t action, uint32_t pulse_ms)
{
    uint32_t max_dose = safety_max_dose_for_channel(channel);
    if (max_dose == 0) {
        return;
    }

    dosing_watchdog_t *watchdog = &s_dose_watchdog[channel];
    TickType_t now = xTaskGetTickCount();
    if (watchdog->window_start == 0) {
        watchdog->window_start = now;
    }

    uint32_t window_elapsed = ticks_to_ms(now - watchdog->window_start);
    if (window_elapsed > SAFETY_DOSE_WINDOW_MS) {
        safety_reset_dose_window(watchdog, now);
    }

    switch (action) {
    case ACTUATOR_ACTION_ON:
        if (!watchdog->on) {
            watchdog->on = true;
            watchdog->on_tick = now;
        }
        break;
    case ACTUATOR_ACTION_OFF:
        if (watchdog->on) {
            watchdog->on_time_ms += ticks_to_ms(now - watchdog->on_tick);
            watchdog->on = false;
        }
        break;
    case ACTUATOR_ACTION_PULSE:
        watchdog->on_time_ms += pulse_ms;
        break;
    default:
        break;
    }

    if (watchdog->on_time_ms > max_dose) {
        safety_fault_mask_t fault = safety_fault_for_channel(channel);
        if (fault != 0) {
            safety_fault_set(fault, "Dose watchdog exceeded");
        }
    }
}

static void safety_note_ph_dose(actuator_channel_t channel)
{
    if (!SAFETY_ENABLE_PH_TDS_CHECKS) {
        return;
    }

    sensor_telemetry_snapshot_t snap;
    if (sensor_telemetry_get_snapshot(&snap) != ESP_OK || !snap.valid) {
        return;
    }

    portENTER_CRITICAL(&s_safety_lock);
    s_ph_response_pending = true;
    s_ph_response_up = (channel == ACTUATOR_CHANNEL_PER_PH_UP);
    s_ph_response_start = snap.ph;
    s_ph_response_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SAFETY_PH_RESPONSE_TIMEOUT_MS);
    portEXIT_CRITICAL(&s_safety_lock);
}

static void safety_note_tds_dose(actuator_channel_t channel)
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
    s_tds_response_pending = true;
    s_tds_response_start = snap.tds;
    s_tds_response_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SAFETY_TDS_RESPONSE_TIMEOUT_MS);
    s_tds_response_fault = fault;
    portEXIT_CRITICAL(&s_safety_lock);
}

static void build_sensor_text_for_channel(actuator_channel_t channel, const sensor_telemetry_snapshot_t *snap, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return;
    }

    if (snap == NULL || !snap->valid) {
        snprintf(out, out_len, "Sensor: N/A");
        return;
    }

    switch (channel) {
    case ACTUATOR_CHANNEL_VALVE:
        snprintf(out, out_len, "WaterLvl: %d", snap->water_level);
        break;
    case ACTUATOR_CHANNEL_PER_NUTA:
    case ACTUATOR_CHANNEL_PER_NUTB:
        snprintf(out, out_len, "TDS: %.2f", (double)snap->tds);
        break;
    case ACTUATOR_CHANNEL_PER_PH_UP:
    case ACTUATOR_CHANNEL_PER_PH_DOWN:
        snprintf(out, out_len, "pH: %.2f", (double)snap->ph);
        break;
    default:
        snprintf(out, out_len, "Sensor: N/A");
        break;
    }
}

static const char *state_text_from_last_command(const actuator_last_command_t *cmd)
{
    if (cmd == NULL) {
        return "UNKNOWN";
    }

    if (cmd->command[0] == '\0') {
        return cmd->state_on ? "ON" : "OFF";
    }

    if (strcmp(cmd->command, "N/A") == 0) {
        ESP_LOGD(TAG, "Last command text is N/A, falling back to state flag");
        return cmd->state_on ? "ON" : "OFF";
    }

    return cmd->command;
}

static bool parse_payload_is_one(const char *payload, int payload_len)
{
    if (payload == NULL || payload_len <= 0) {
        return false;
    }

    char buf[8];
    int len = payload_len;
    if (len >= (int)sizeof(buf)) {
        len = (int)sizeof(buf) - 1;
    }
    memcpy(buf, payload, (size_t)len);
    buf[len] = '\0';

    for (int i = 0; i < len; i++) {
        if (buf[i] == ' ' || buf[i] == '\r' || buf[i] == '\n' || buf[i] == '\t') {
            buf[i] = '\0';
            break;
        }
    }

    return (strcmp(buf, "1") == 0);
}

static esp_err_t write_topic(char *dst, size_t dst_len, const char *fmt,
                             const char *zone_id, const char *suffix)
{
    int written = (suffix != NULL)
        ? snprintf(dst, dst_len, fmt, zone_id, suffix)
        : snprintf(dst, dst_len, fmt, zone_id);

    if (written <= 0 || written >= (int)dst_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static esp_err_t subscribe_formatted_topic(char *dst, size_t dst_len,
                                           const char *fmt, const char *zone_id,
                                           const char *suffix,
                                           mqtt_manager_message_cb_t cb,
                                           void *ctx, bool *flag)
{
    esp_err_t ret = write_topic(dst, dst_len, fmt, zone_id, suffix);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mqtt_manager_subscribe(dst, 1, cb, ctx);
    if (ret != ESP_OK) {
        return ret;
    }

    if (flag != NULL) {
        *flag = true;
    }

    return ESP_OK;
}

typedef struct {
    bool *subscribed;
    const char *topic;
} mqtt_unsub_entry_t;

static void unsubscribe_topics(const mqtt_unsub_entry_t *entries, size_t count)
{
    if (entries == NULL) {
        return;
    }

    for (size_t i = 0; i < count; i++) {
        if (entries[i].subscribed == NULL || !*entries[i].subscribed) {
            continue;
        }

        if (entries[i].topic != NULL && entries[i].topic[0] != '\0') {
            mqtt_manager_unsubscribe(entries[i].topic);
        }
        *entries[i].subscribed = false;
    }
}

static esp_err_t channel_from_name(const char *name, actuator_channel_t *channel)
{
    if (name == NULL || channel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        const char *channel_name = actuator_control_get_channel_name((actuator_channel_t)i);
        if (channel_name != NULL && strcasecmp(name, channel_name) == 0) {
            *channel = (actuator_channel_t)i;
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

static bool parse_zone_command(const char *payload, int payload_len, dosing_command_t *out)
{
    if (payload == NULL || out == NULL || payload_len <= 0 || payload_len > ZONE_COMMAND_MAX_LEN) {
        return false;
    }

    char buf[ZONE_COMMAND_BUFFER_SIZE];
    memcpy(buf, payload, (size_t)payload_len);
    buf[payload_len] = '\0';

    char *save_ptr = NULL;
    char *token0 = strtok_r(buf, " \t\r\n", &save_ptr);
    char *token1 = strtok_r(NULL, " \t\r\n", &save_ptr);
    char *token2 = strtok_r(NULL, " \t\r\n", &save_ptr);

    if (token0 == NULL || token1 == NULL) {
        return false;
    }

    actuator_channel_t channel;
    if (channel_from_name(token0, &channel) != ESP_OK) {
        return false;
    }

    actuator_action_t action;
    uint32_t pulse_ms = 0;

    if (strcasecmp(token1, "PULSE") == 0 && token2 != NULL) {
        char pulse_buf[32];
        int written = snprintf(pulse_buf, sizeof(pulse_buf), "PULSE:%s", token2);
        if (written <= 0 || written >= (int)sizeof(pulse_buf)) {
            return false;
        }
        if (actuator_control_parse_action_payload(pulse_buf, (int)strlen(pulse_buf), &action, &pulse_ms) != ESP_OK) {
            return false;
        }
    } else {
        if (actuator_control_parse_action_payload(token1, (int)strlen(token1), &action, &pulse_ms) != ESP_OK) {
            return false;
        }
    }

    out->channel = channel;
    out->action = action;
    out->pulse_ms = pulse_ms;
    return true;
}

static bool enqueue_command(const dosing_command_t *cmd, const char *source)
{
    if (cmd == NULL || s_dosing_queue == NULL) {
        return false;
    }

    bool gate_open = false;
    if (xPortInIsrContext()) {
        portENTER_CRITICAL_ISR(&s_cmd_gate_lock);
        gate_open = s_gate_open;
        portEXIT_CRITICAL_ISR(&s_cmd_gate_lock);
    } else {
        portENTER_CRITICAL(&s_cmd_gate_lock);
        gate_open = s_gate_open;
        portEXIT_CRITICAL(&s_cmd_gate_lock);
    }

    if (!gate_open) {
        return false;
    }

    BaseType_t sent = pdFALSE;

    if (xPortInIsrContext()) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        sent = xQueueSendFromISR(s_dosing_queue, cmd, &higher_priority_task_woken);
        if (higher_priority_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    } else {
        sent = xQueueSend(s_dosing_queue, cmd, 0);
    }

    if (sent != pdTRUE) {
        ESP_LOGW(TAG, "%s: dosing queue full, dropping command", source != NULL ? source : "enqueue");
        return false;
    }

    return true;
}

static void on_channel_command(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    actuator_channel_t channel = (actuator_channel_t)(intptr_t)user_ctx;

    actuator_action_t action;
    uint32_t pulse_ms = 0;
    if (actuator_control_parse_action_payload(payload, payload_len, &action, &pulse_ms) != ESP_OK) {
        ESP_LOGW(TAG, "Invalid actuator command payload");
        return;
    }

    dosing_command_t cmd = {
        .channel = channel,
        .action = action,
        .pulse_ms = pulse_ms,
    };
    (void)enqueue_command(&cmd, "channel command");
}

static void on_zone_command(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    dosing_command_t cmd;
    if (!parse_zone_command(payload, payload_len, &cmd)) {
        ESP_LOGW(TAG, "Invalid zone command format. Expected: '<Channel> ON|OFF|PULSE <ms>'");
        return;
    }

    (void)enqueue_command(&cmd, "zone command");
}

static void on_safety_clear_command(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    if (payload == NULL || payload_len <= 0) {
        return;
    }

    char buf[32];
    int len = payload_len;
    if (len >= (int)sizeof(buf)) {
        len = (int)sizeof(buf) - 1;
    }
    memcpy(buf, payload, (size_t)len);
    buf[len] = '\0';

    for (int i = 0; i < len; i++) {
        buf[i] = (char)toupper((unsigned char)buf[i]);
    }

    if (strcmp(buf, "CLEAR") == 0 || strcmp(buf, "ALL") == 0) {
        (void)runtime_tasks_clear_safety_faults(0);
        return;
    }

    if (strncmp(buf, "MASK:", 5) == 0) {
        char *end_ptr = NULL;
        unsigned long mask = strtoul(buf + 5, &end_ptr, 16);
        if (end_ptr != buf + 5) {
            (void)runtime_tasks_clear_safety_faults((safety_fault_mask_t)mask);
        }
    }
}

static void on_ota_latest_version(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    if (payload == NULL || payload_len <= 0) {
        return;
    }

    char version[OTA_VERSION_MAX_LEN];
    int len = payload_len;
    if (len >= (int)sizeof(version)) {
        len = (int)sizeof(version) - 1;
    }
    memcpy(version, payload, (size_t)len);
    version[len] = '\0';

    while (len > 0 && (version[len - 1] == ' ' || version[len - 1] == '\r' || version[len - 1] == '\n' || version[len - 1] == '\t')) {
        version[len - 1] = '\0';
        len--;
    }

    if (version[0] == '\0') {
        return;
    }

    if (strcmp(s_ota_latest_version, version) != 0) {
        strncpy(s_ota_latest_version, version, sizeof(s_ota_latest_version) - 1);
        s_ota_latest_version[sizeof(s_ota_latest_version) - 1] = '\0';
        ESP_LOGI(TAG, "OTA latest version updated: %s", s_ota_latest_version);
    }
}

static void on_ota_trigger(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    if (!parse_payload_is_one(payload, payload_len)) {
        return;
    }

    if (ota_update_http_is_busy()) {
        ESP_LOGW(TAG, "OTA already in progress");
        return;
    }

    if (s_ota_latest_version[0] == '\0') {
        ESP_LOGW(TAG, "OTA trigger received without latest version");
        return;
    }

    const esp_app_desc_t *app_desc = esp_app_get_description();
    const char *current = (app_desc != NULL && app_desc->version[0] != '\0') ? app_desc->version : "unknown";
    if (strcmp(s_ota_latest_version, current) == 0) {
        ESP_LOGI(TAG, "OTA already up to date: %s", current);
        return;
    }

    setup_config_t setup_cfg;
    bool setup_found = false;
    if (setup_config_load(&setup_cfg, &setup_found) != ESP_OK || !setup_found) {
        ESP_LOGE(TAG, "OTA config missing");
        return;
    }

    if (setup_cfg.broker_ip[0] == '\0') {
        ESP_LOGE(TAG, "Broker IP not set");
        return;
    }

    char url[256];
    int written = snprintf(url, sizeof(url), "http://%s:%d%s",
                           setup_cfg.broker_ip,
                           OTA_HTTP_PORT_DEFAULT,
                           OTA_HTTP_PATH_DEFAULT);
    if (written <= 0 || written >= (int)sizeof(url)) {
        ESP_LOGE(TAG, "OTA URL too long");
        return;
    }

    esp_err_t ret = ota_update_http_start(url);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA start failed: %s", esp_err_to_name(ret));
    }
}

static void dosing_task(void *arg)
{
    (void)arg;
    dosing_command_t cmd;

    safety_wdt_register();

    while (!atomic_load(&s_stop_requested)) {
        if (xQueueReceive(s_dosing_queue, &cmd, pdMS_TO_TICKS(DOSING_QUEUE_RECV_TIMEOUT_MS)) == pdTRUE) {
            if ((cmd.action == ACTUATOR_ACTION_ON || cmd.action == ACTUATOR_ACTION_PULSE) &&
                ((cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP && get_channel_state_on(ACTUATOR_CHANNEL_PER_PH_DOWN)) ||
                 (cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN && get_channel_state_on(ACTUATOR_CHANNEL_PER_PH_UP)))) {
                safety_fault_set(SAFETY_FAULT_PH_INTERLOCK, "pH interlock violation");
                continue;
            }

            esp_err_t ret = ESP_OK;
            switch (cmd.action) {
            case ACTUATOR_ACTION_ON:
                ret = actuator_control_apply_state(cmd.channel, true);
                if (ret == ESP_OK) {
                    update_channel_state_on(cmd.channel, true);
                }
                break;
            case ACTUATOR_ACTION_OFF:
                ret = actuator_control_apply_state(cmd.channel, false);
                if (ret == ESP_OK) {
                    update_channel_state_on(cmd.channel, false);
                }
                break;
            case ACTUATOR_ACTION_PULSE:
                update_channel_state_on(cmd.channel, true);
                ret = actuator_control_apply_pulse(cmd.channel, cmd.pulse_ms);
                update_channel_state_on(cmd.channel, false);
                break;
            default:
                ret = ESP_ERR_INVALID_ARG;
                break;
            }

            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Dosing command failed: %s", esp_err_to_name(ret));
                continue;
            }

            dose_watchdog_update(cmd.channel, cmd.action, cmd.pulse_ms);

            if (cmd.channel == ACTUATOR_CHANNEL_VALVE) {
                TickType_t now = xTaskGetTickCount();
                portENTER_CRITICAL(&s_safety_lock);
                if (cmd.action == ACTUATOR_ACTION_ON) {
                    s_valve_on_tick = now;
                    s_valve_waiting = true;
                } else if (cmd.action == ACTUATOR_ACTION_OFF) {
                    s_valve_waiting = false;
                }
                portEXIT_CRITICAL(&s_safety_lock);
            }

            if (cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP || cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN) {
                safety_note_ph_dose(cmd.channel);
            }

            if (cmd.channel == ACTUATOR_CHANNEL_PER_NUTA || cmd.channel == ACTUATOR_CHANNEL_PER_NUTB) {
                safety_note_tds_dose(cmd.channel);
            }
        }

        safety_heartbeat(&s_dosing_heartbeat);
        safety_wdt_kick();
    }

    if (s_wdt_enabled) {
        esp_task_wdt_delete(NULL);
    }

    signal_task_exit(TASK_EXIT_BIT_DOSING);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_dosing_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}

static void sensor_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    safety_wdt_register();

    while (!atomic_load(&s_stop_requested)) {
        esp_err_t ret = sensor_telemetry_sample();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Sensor sample failed: %s", esp_err_to_name(ret));
        }

        safety_heartbeat(&s_sensor_heartbeat);
        safety_wdt_kick();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS));
    }

    if (s_wdt_enabled) {
        esp_task_wdt_delete(NULL);
    }

    signal_task_exit(TASK_EXIT_BIT_SENSOR);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_sensor_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}

static void comm_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    safety_wdt_register();

    while (!atomic_load(&s_stop_requested)) {
        sensor_telemetry_snapshot_t snap = {0};
        bool snapshot_available = false;

        if (sensor_telemetry_get_snapshot(&snap) == ESP_OK && snap.valid) {
            if (s_snapshot_mutex != NULL &&
                xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS)) == pdTRUE) {
                s_last_good_snapshot = snap;
                s_last_good_snapshot_valid = true;
                xSemaphoreGive(s_snapshot_mutex);
            } else if (s_snapshot_mutex != NULL) {
                ESP_LOGD(TAG, "Snapshot mutex busy, skipping cache update");
            }
            snapshot_available = true;
        } else {
            if (s_snapshot_mutex != NULL &&
                xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS)) == pdTRUE) {
                if (s_last_good_snapshot_valid) {
                    snap = s_last_good_snapshot;
                    snapshot_available = true;
                    ESP_LOGD(TAG, "Using last known-good sensor snapshot");
                }
                xSemaphoreGive(s_snapshot_mutex);
            } else if (s_snapshot_mutex != NULL) {
                ESP_LOGD(TAG, "Snapshot mutex busy, skipping cached snapshot");
            }
        }

        if (snapshot_available) {
            char payload_level[8];
            char payload_temp[16];
            char payload_ph[16];
            char payload_tds[16];

            snprintf(payload_level, sizeof(payload_level), "%d", snap.water_level);
            snprintf(payload_temp, sizeof(payload_temp), "%.2f", (double)snap.water_temp);
            snprintf(payload_ph, sizeof(payload_ph), "%.2f", (double)snap.ph);
            snprintf(payload_tds, sizeof(payload_tds), "%.2f", (double)snap.tds);

            mqtt_manager_publish(sensor_telemetry_topic_water_level(), payload_level, 0, 0);
            mqtt_manager_publish(sensor_telemetry_topic_water_temp(), payload_temp, 0, 0);
            mqtt_manager_publish(sensor_telemetry_topic_ph(), payload_ph, 0, 0);
            mqtt_manager_publish(sensor_telemetry_topic_tds(), payload_tds, 0, 0);
        }

        actuator_last_command_t latest_cmd;
        if (actuator_control_get_last_command(&latest_cmd) == ESP_OK) {
            if (strcmp(latest_cmd.topic, s_last_lcd_rendered_cmd.topic) != 0 ||
                strcmp(latest_cmd.command, s_last_lcd_rendered_cmd.command) != 0 ||
                latest_cmd.state_on != s_last_lcd_rendered_cmd.state_on) {
                actuator_channel_t channel;
                char sensor_text[20];

                if (channel_from_name(latest_cmd.channel, &channel) == ESP_OK) {
                    build_sensor_text_for_channel(channel, snapshot_available ? &snap : NULL, sensor_text, sizeof(sensor_text));
                } else {
                    snprintf(sensor_text, sizeof(sensor_text), "Sensor: N/A");
                }

                lcd_status_show_actuator_event(
                    s_zone_id,
                    s_zone_name,
                    latest_cmd.channel,
                    sensor_text,
                    state_text_from_last_command(&latest_cmd));

                s_last_lcd_rendered_cmd = latest_cmd;
            }
        }

        safety_heartbeat(&s_comm_heartbeat);
        safety_wdt_kick();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COMM_INTERVAL_MS));
    }

    if (s_wdt_enabled) {
        esp_task_wdt_delete(NULL);
    }

    signal_task_exit(TASK_EXIT_BIT_COMM);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_comm_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}

static void safety_task(void *arg)
{
    (void)arg;
    safety_wdt_register();

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_dosing_hb = atomic_load(&s_dosing_heartbeat);
    uint32_t last_sensor_hb = atomic_load(&s_sensor_heartbeat);
    uint32_t last_comm_hb = atomic_load(&s_comm_heartbeat);
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

    bool counters_cleared = false;

    while (!atomic_load(&s_stop_requested)) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SAFETY_INTERVAL_MS));
        TickType_t now = xTaskGetTickCount();

        safety_heartbeat(&s_safety_heartbeat);
        safety_wdt_kick();

        if (!counters_cleared && ticks_to_ms(now) > (SAFETY_RESET_WINDOW_SEC * 1000u)) {
            s_reset_count = 0;
            s_wdt_reset_count = 0;
            counters_cleared = true;
        }

        safety_check_heartbeat(s_dosing_task, &s_dosing_heartbeat,
                               &last_dosing_hb, &dosing_last_change,
                               now, "Dosing task heartbeat stalled");
        safety_check_heartbeat(s_sensor_task, &s_sensor_heartbeat,
                               &last_sensor_hb, &sensor_last_change,
                               now, "Sensor task heartbeat stalled");
        safety_check_heartbeat(s_comm_task, &s_comm_heartbeat,
                               &last_comm_hb, &comm_last_change,
                               now, "Comm task heartbeat stalled");

        if (atomic_load(&s_water_level_low_isr)) {
            if (ticks_to_ms(now - s_water_level_low_tick) >= SAFETY_LEVEL_LOW_DEBOUNCE_MS) {
                safety_fault_set(SAFETY_FAULT_WATER_LEVEL, "Water level low ISR");
            }
            atomic_store(&s_water_level_low_isr, false);
        }

        sensor_telemetry_snapshot_t snap = {0};
        bool snap_ok = (sensor_telemetry_get_snapshot(&snap) == ESP_OK && snap.valid);

        if (snap_ok) {
            if (snap.water_level == 0) {
                safety_fault_set(SAFETY_FAULT_WATER_LEVEL, "Water level low");
            }

            if (snap.water_temp > SAFETY_TEMP_CRITICAL ||
                snap.water_temp > SAFETY_TEMP_HIGH ||
                snap.water_temp < SAFETY_TEMP_LOW) {
                safety_fault_set(SAFETY_FAULT_WATER_TEMP, "Water temperature out of range");
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
                ph_pending = s_ph_response_pending;
                ph_up = s_ph_response_up;
                ph_deadline = s_ph_response_deadline;
                ph_start = s_ph_response_start;
                portEXIT_CRITICAL(&s_safety_lock);

                if (ph_pending) {
                    float delta = snap.ph - ph_start;
                    if ((ph_up && delta >= SAFETY_PH_RESPONSE_MIN_DELTA) ||
                        (!ph_up && delta <= -SAFETY_PH_RESPONSE_MIN_DELTA)) {
                        portENTER_CRITICAL(&s_safety_lock);
                        s_ph_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    } else if ((int32_t)(now - ph_deadline) < 0) {
                        /* Deadline not reached yet. */
                    } else {
                        safety_fault_set(ph_up ? SAFETY_FAULT_PH_UP : SAFETY_FAULT_PH_DOWN, "pH response timeout");
                        portENTER_CRITICAL(&s_safety_lock);
                        s_ph_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    }
                }

                bool tds_pending = false;
                TickType_t tds_deadline = 0;
                float tds_start = 0.0f;
                safety_fault_mask_t tds_fault = 0;
                portENTER_CRITICAL(&s_safety_lock);
                tds_pending = s_tds_response_pending;
                tds_deadline = s_tds_response_deadline;
                tds_start = s_tds_response_start;
                tds_fault = s_tds_response_fault;
                portEXIT_CRITICAL(&s_safety_lock);

                if (tds_pending) {
                    float delta = snap.tds - tds_start;
                    if (fabsf(delta) >= SAFETY_TDS_RESPONSE_MIN_DELTA) {
                        portENTER_CRITICAL(&s_safety_lock);
                        s_tds_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    } else if ((int32_t)(now - tds_deadline) < 0) {
                        /* Deadline not reached yet. */
                    } else {
                        if (tds_fault != 0) {
                            safety_fault_set(tds_fault, "TDS response timeout");
                        }
                        portENTER_CRITICAL(&s_safety_lock);
                        s_tds_response_pending = false;
                        portEXIT_CRITICAL(&s_safety_lock);
                    }
                }
            }

            TickType_t valve_on_tick = 0;
            bool valve_waiting = false;
            portENTER_CRITICAL(&s_safety_lock);
            valve_on_tick = s_valve_on_tick;
            valve_waiting = s_valve_waiting;
            portEXIT_CRITICAL(&s_safety_lock);

            if (valve_waiting && valve_on_tick != 0 && snap.water_level == 0) {
                if (ticks_to_ms(now - valve_on_tick) > SAFETY_FILL_TIMEOUT_MS) {
                    safety_fault_set(SAFETY_FAULT_FILL_TIMEOUT | SAFETY_FAULT_VALVE, "Valve fill timeout");
                    portENTER_CRITICAL(&s_safety_lock);
                    s_valve_waiting = false;
                    portEXIT_CRITICAL(&s_safety_lock);
                }
            }
        }

        last_wake = now;
    }

    if (s_wdt_enabled) {
        esp_task_wdt_delete(NULL);
    }

    signal_task_exit(TASK_EXIT_BIT_SAFETY);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_safety_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}

static void cleanup_runtime_locked(void)
{
    portENTER_CRITICAL(&s_cmd_gate_lock);
    s_gate_open = false;
    portEXIT_CRITICAL(&s_cmd_gate_lock);
    atomic_store(&s_stop_requested, true);

    const mqtt_unsub_entry_t unsub_entries[] = {
        { &s_zone_topic_subscribed, s_zone_command_topic },
        { &s_safety_topic_subscribed, s_safety_clear_topic },
        { &s_ota_latest_subscribed, s_ota_latest_topic },
        { &s_ota_trigger_subscribed, s_ota_trigger_topic },
    };
    unsubscribe_topics(unsub_entries, ARRAY_SIZE(unsub_entries));

    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        if (!s_channel_topics_subscribed[i]) {
            continue;
        }

        const char *topic = actuator_control_get_command_topic((actuator_channel_t)i);
        if (topic != NULL && topic[0] != '\0') {
            mqtt_manager_unsubscribe(topic);
        }
        s_channel_topics_subscribed[i] = false;
    }

    EventBits_t wait_bits = task_exit_bits_for_running_tasks();

    if (wait_bits != 0 && s_task_exit_event != NULL) {
        EventBits_t stopped_bits = xEventGroupWaitBits(
            s_task_exit_event,
            wait_bits,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(TASK_EXIT_WAIT_MS));

        if ((stopped_bits & wait_bits) != wait_bits) {
            EventBits_t missing = wait_bits & ~stopped_bits;
            ESP_LOGW(TAG, "Task stop timeout - missing bits: 0x%02lx (%s%s%s%s)",
                     (unsigned long)missing,
                     (missing & TASK_EXIT_BIT_DOSING) ? "Dosing " : "",
                     (missing & TASK_EXIT_BIT_SENSOR) ? "Sensor " : "",
                     (missing & TASK_EXIT_BIT_COMM) ? "Comm " : "",
                     (missing & TASK_EXIT_BIT_SAFETY) ? "Safety" : "");
        }
    }

    delete_running_tasks();

    if (s_dosing_queue != NULL) {
        vQueueDelete(s_dosing_queue);
        s_dosing_queue = NULL;
    }

    if (s_snapshot_mutex != NULL) {
        vSemaphoreDelete(s_snapshot_mutex);
        s_snapshot_mutex = NULL;
    }

    if (s_isr_installed) {
        gpio_isr_handler_remove((gpio_num_t)PIN_SENSOR_WATER_LEVEL);
        gpio_set_intr_type((gpio_num_t)PIN_SENSOR_WATER_LEVEL, GPIO_INTR_DISABLE);
        s_isr_installed = false;
    }

    sensor_telemetry_deinit();
    actuator_control_deinit();
    clear_subscription_tracking();
    s_zone_id[0] = '\0';
    s_zone_name[0] = '\0';
    s_zone_command_topic[0] = '\0';
    s_safety_fault_topic[0] = '\0';
    s_safety_clear_topic[0] = '\0';
    s_ota_latest_topic[0] = '\0';
    s_ota_trigger_topic[0] = '\0';
    s_ota_latest_version[0] = '\0';
    memset(&s_last_lcd_rendered_cmd, 0, sizeof(s_last_lcd_rendered_cmd));
    memset(&s_last_good_snapshot, 0, sizeof(s_last_good_snapshot));
    memset(s_channel_state_on, 0, sizeof(s_channel_state_on));
    memset(s_dose_watchdog, 0, sizeof(s_dose_watchdog));
    s_last_good_snapshot_valid = false;
    s_running = false;
    s_starting = false;
}

void runtime_tasks_record_boot_faults(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    if (reason != ESP_RST_POWERON && reason != ESP_RST_UNKNOWN) {
        if (s_reset_count < UINT32_MAX) {
            s_reset_count++;
        }
    }

    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
        if (s_wdt_reset_count < UINT32_MAX) {
            s_wdt_reset_count++;
        }
        s_boot_faults |= SAFETY_FAULT_WDT;
    }

    if (reason == ESP_RST_BROWNOUT) {
        s_boot_faults |= SAFETY_FAULT_POWER;
    }

    if (s_reset_count > SAFETY_RESET_MAX_COUNT) {
        s_boot_faults |= SAFETY_FAULT_POWER;
    }

    if (s_wdt_reset_count > SAFETY_WDT_RESET_MAX_COUNT) {
        s_boot_faults |= SAFETY_FAULT_WDT;
    }

    ESP_LOGI(TAG, "Reset reason=%d reset_count=%lu wdt_count=%lu",
             (int)reason,
             (unsigned long)s_reset_count,
             (unsigned long)s_wdt_reset_count);
}

esp_err_t runtime_tasks_start(const char *zone_id)
{
    if (zone_id == NULL || zone_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = runtime_lock();
    if (ret != ESP_OK) {
        return ret;
    }

    if (s_running || s_starting) {
        runtime_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    s_starting = true;
    atomic_store(&s_stop_requested, false);
    portENTER_CRITICAL(&s_cmd_gate_lock);
    s_gate_open = false;
    portEXIT_CRITICAL(&s_cmd_gate_lock);
    clear_subscription_tracking();
    s_ota_latest_version[0] = '\0';

    reset_heartbeats();
    atomic_store(&s_water_level_low_isr, false);
    portENTER_CRITICAL(&s_safety_lock);
    s_ph_response_pending = false;
    s_tds_response_pending = false;
    s_tds_response_fault = 0;
    s_valve_waiting = false;
    portEXIT_CRITICAL(&s_safety_lock);
    memset(s_channel_state_on, 0, sizeof(s_channel_state_on));
    memset(s_dose_watchdog, 0, sizeof(s_dose_watchdog));

    SAFE_STRCPY(s_zone_id, zone_id);
    memset(&s_last_lcd_rendered_cmd, 0, sizeof(s_last_lcd_rendered_cmd));
    memset(&s_last_good_snapshot, 0, sizeof(s_last_good_snapshot));
    s_last_good_snapshot_valid = false;

    zone_config_t zone_cfg;
    bool zone_found = false;
    esp_err_t zone_ret = zone_config_load(&zone_cfg, &zone_found);
    if (zone_ret == ESP_OK && zone_found && strcmp(zone_cfg.zone_id, zone_id) == 0) {
        SAFE_STRCPY(s_zone_name, zone_cfg.zone_name);
    } else {
        SAFE_STRCPY(s_zone_name, "Unknown");
    }

    ret = actuator_control_init(zone_id);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    ret = sensor_telemetry_init(zone_id);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    safety_wdt_init();

    s_dosing_queue = xQueueCreate(DOSING_QUEUE_LEN, sizeof(dosing_command_t));
    if (s_dosing_queue == NULL) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ESP_ERR_NO_MEM;
    }

    if (s_snapshot_mutex == NULL) {
        s_snapshot_mutex = xSemaphoreCreateMutex();
        if (s_snapshot_mutex == NULL) {
            cleanup_runtime_locked();
            runtime_unlock();
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_task_exit_event == NULL) {
        s_task_exit_event = xEventGroupCreate();
        if (s_task_exit_event == NULL) {
            cleanup_runtime_locked();
            runtime_unlock();
            return ESP_ERR_NO_MEM;
        }
    }
    xEventGroupClearBits(s_task_exit_event, TASK_EXIT_ALL_BITS);

    ret = subscribe_formatted_topic(s_zone_command_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/command", zone_id, NULL,
                                    on_zone_command, NULL, &s_zone_topic_subscribed);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    ret = write_topic(s_safety_fault_topic, ZONE_TOPIC_BUFFER_SIZE,
                      "%s/%s", zone_id, SAFETY_FAULT_TOPIC_SUFFIX);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    ret = subscribe_formatted_topic(s_safety_clear_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/%s", zone_id, SAFETY_CLEAR_TOPIC_SUFFIX,
                                    on_safety_clear_command, NULL, &s_safety_topic_subscribed);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    ret = subscribe_formatted_topic(s_ota_latest_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/ota/latest_version", zone_id, NULL,
                                    on_ota_latest_version, NULL, &s_ota_latest_subscribed);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    ret = subscribe_formatted_topic(s_ota_trigger_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/ota/trigger", zone_id, NULL,
                                    on_ota_trigger, NULL, &s_ota_trigger_subscribed);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        const char *topic = actuator_control_get_command_topic((actuator_channel_t)i);
        if (topic == NULL || topic[0] == '\0') {
            cleanup_runtime_locked();
            runtime_unlock();
            return ESP_ERR_INVALID_STATE;
        }

        ret = mqtt_manager_subscribe(topic, 1, on_channel_command, (void *)(intptr_t)i);
        if (ret != ESP_OK) {
            cleanup_runtime_locked();
            runtime_unlock();
            return ret;
        }
        s_channel_topics_subscribed[i] = true;
    }

    if (!s_isr_installed) {
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            cleanup_runtime_locked();
            runtime_unlock();
            return ret;
        }
        s_isr_installed = true;
    }

    ret = gpio_set_intr_type((gpio_num_t)PIN_SENSOR_WATER_LEVEL, GPIO_INTR_ANYEDGE);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    ret = gpio_isr_handler_add((gpio_num_t)PIN_SENSOR_WATER_LEVEL, water_level_isr, NULL);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    atomic_store(&s_reported_faults, 0);
    if (s_boot_faults != 0) {
        safety_fault_set(s_boot_faults, "Boot fault");
    }

    safety_fault_mask_t existing_faults = (safety_fault_mask_t)atomic_load(&s_safety_faults);
    if (existing_faults != 0) {
        safety_enter_safe_state(existing_faults);
        safety_publish_faults(existing_faults);
    }

    BaseType_t ok = xTaskCreate(dosing_task, "DosingTask", DOSING_TASK_STACK, NULL, DOSING_TASK_PRIORITY, &s_dosing_task);
    if (ok != pdPASS) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ESP_FAIL;
    }

    ok = xTaskCreate(sensor_task, "SensorTask", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIORITY, &s_sensor_task);
    if (ok != pdPASS) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ESP_FAIL;
    }

    ok = xTaskCreate(comm_task, "CommTask", COMM_TASK_STACK, NULL, COMM_TASK_PRIORITY, &s_comm_task);
    if (ok != pdPASS) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ESP_FAIL;
    }

    ok = xTaskCreate(safety_task, "SafetyTask", SAFETY_TASK_STACK, NULL, SAFETY_TASK_PRIORITY, &s_safety_task);
    if (ok != pdPASS) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ESP_FAIL;
    }

    if (!atomic_load(&s_safe_mode)) {
        portENTER_CRITICAL(&s_cmd_gate_lock);
        s_gate_open = true;
        portEXIT_CRITICAL(&s_cmd_gate_lock);
    }

    s_running = true;
    s_starting = false;
    atomic_store(&s_stop_requested, false);
    lcd_status_show_zone_overview(s_zone_id, s_zone_name, "None");
    ESP_LOGI(TAG, "Runtime tasks started: DosingTask(%d), SensorTask(%d), CommTask(%d), SafetyTask(%d)",
             DOSING_TASK_PRIORITY, SENSOR_TASK_PRIORITY, COMM_TASK_PRIORITY, SAFETY_TASK_PRIORITY);
    runtime_unlock();
    return ESP_OK;
}

esp_err_t runtime_tasks_stop(void)
{
    esp_err_t ret = runtime_lock();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!s_running && !s_starting) {
        runtime_unlock();
        return ESP_OK;
    }

    cleanup_runtime_locked();
    runtime_unlock();
    ESP_LOGI(TAG, "Runtime tasks stopped");
    return ESP_OK;
}

bool runtime_tasks_is_running(void)
{
    if (s_runtime_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(s_runtime_mutex, pdMS_TO_TICKS(RUNTIME_STATE_QUERY_WAIT_MS)) == pdTRUE) {
        bool running = s_running;
        xSemaphoreGive(s_runtime_mutex);
        return running;
    }

    return false;
}

safety_fault_mask_t runtime_tasks_get_safety_faults(void)
{
    return (safety_fault_mask_t)atomic_load(&s_safety_faults);
}

esp_err_t runtime_tasks_clear_safety_faults(safety_fault_mask_t mask)
{
    safety_fault_mask_t current = (safety_fault_mask_t)atomic_load(&s_safety_faults);
    if (mask == 0) {
        mask = current;
    }

    safety_fault_mask_t new_mask = current & ~mask;
    atomic_store(&s_safety_faults, (unsigned int)new_mask);
    atomic_store(&s_reported_faults, (unsigned int)new_mask);

    if (mask == current) {
        s_boot_faults = 0;
    } else {
        s_boot_faults &= ~mask;
    }

    if (new_mask == 0) {
        atomic_store(&s_safe_mode, false);
        atomic_store(&s_water_level_low_isr, false);
        portENTER_CRITICAL(&s_safety_lock);
        s_ph_response_pending = false;
        s_tds_response_pending = false;
        s_valve_waiting = false;
        portEXIT_CRITICAL(&s_safety_lock);

        if (s_running && !s_starting && !atomic_load(&s_stop_requested)) {
            portENTER_CRITICAL(&s_cmd_gate_lock);
            s_gate_open = true;
            portEXIT_CRITICAL(&s_cmd_gate_lock);
        }
    }

    return ESP_OK;
}

bool runtime_tasks_is_safe_mode(void)
{
    return atomic_load(&s_safe_mode);
}
