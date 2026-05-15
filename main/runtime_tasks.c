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
#include "indicator_led.h"
#include "lcd_status.h"
#include "mqtt_manager.h"
#include "ota_update.h"
#include "pin_config.h"
#include "runtime_safety.h"
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
#define SAFETY_TASK_PRIORITY 4

#define SENSOR_SAMPLE_INTERVAL_MS 1000
#define COMM_INTERVAL_MS 1000
#define VERSION_PUBLISH_INTERVAL_MS 30000
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


static const char *TAG = "runtime_tasks";

static SemaphoreHandle_t s_runtime_mutex = NULL;
static SemaphoreHandle_t s_snapshot_mutex = NULL;
static portMUX_TYPE s_runtime_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_task_handle_lock = portMUX_INITIALIZER_UNLOCKED;
static EventGroupHandle_t s_task_exit_event = NULL;
static TaskHandle_t s_dosing_task = NULL;
static TaskHandle_t s_sensor_task = NULL;
static TaskHandle_t s_comm_task = NULL;
static TaskHandle_t s_safety_task = NULL;
static QueueHandle_t s_dosing_queue = NULL;
static bool s_running;
static bool s_starting;
static atomic_bool s_stop_requested;
static bool s_zone_topic_subscribed;
static bool s_channel_topics_subscribed[ACTUATOR_CHANNEL_COUNT];
static bool s_safety_topic_subscribed;
static bool s_ota_latest_subscribed;
static bool s_ota_trigger_subscribed;
static bool s_emergency_subscribed;
static char s_zone_command_topic[ZONE_TOPIC_BUFFER_SIZE];
static char s_emergency_topic[ZONE_TOPIC_BUFFER_SIZE];
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
    s_emergency_subscribed = false;
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

static uint32_t ticks_to_ms(TickType_t ticks)
{
    return (uint32_t)(ticks * (TickType_t)portTICK_PERIOD_MS);
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

static void publish_current_version(const char *zone_id)
{
    if (zone_id == NULL || zone_id[0] == '\0') {
        return;
    }

    char topic[ZONE_TOPIC_BUFFER_SIZE];
    esp_err_t ret = write_topic(topic, sizeof(topic), "%s/version", zone_id, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to build version topic for zone %s", zone_id);
        return;
    }

    const esp_app_desc_t *app_desc = esp_app_get_description();
    const char *version = (app_desc != NULL && app_desc->version[0] != '\0') ? app_desc->version : "unknown";
    ret = mqtt_manager_publish(topic, version, 1, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish version %s to %s: %s", version, topic, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Published version %s to %s", version, topic);
    }
}

static void publish_actuator_states(void)
{
    bool states[ACTUATOR_CHANNEL_COUNT] = {0};

    runtime_safety_copy_channel_states(states);

    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        const char *topic = actuator_control_get_status_topic((actuator_channel_t)i);
        if (topic == NULL || topic[0] == '\0') {
            continue;
        }
        esp_err_t ret = mqtt_manager_publish(topic, states[i] ? "ON" : "OFF", 1, 0);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "Failed to republish state for %s", topic);
        }
    }
}

static void runtime_tasks_on_mqtt_connection(bool connected, void *user_ctx)
{
    (void)user_ctx;

    indicator_led_set_connection(connected);

    if (!connected) {
        return;
    }

    if (!s_running || s_starting || s_zone_id[0] == '\0') {
        return;
    }

    publish_current_version(s_zone_id);
    publish_actuator_states();
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

    if (!runtime_safety_is_gate_open(xPortInIsrContext())) {
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

static bool parse_payload_is_one(const char *payload, int payload_len);

static void on_emergency_command(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    if (!parse_payload_is_one(payload, payload_len)) {
        return;
    }

    ESP_LOGW(TAG, "Emergency command received — triggering fault");
    runtime_safety_fault_set(SAFETY_FAULT_VALVE | SAFETY_FAULT_DOSE_A | SAFETY_FAULT_DOSE_B
                             | SAFETY_FAULT_PH_UP | SAFETY_FAULT_PH_DOWN,
                             "Emergency stop");
    indicator_led_fault_blink(5000);
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

    const esp_app_desc_t *app_desc = esp_app_get_description();
    const char *current = (app_desc != NULL && app_desc->version[0] != '\0') ? app_desc->version : "unknown";
    if (s_ota_latest_version[0] != '\0') {
        ESP_LOGI(TAG, "OTA trigger received: current=%s latest=%s", current, s_ota_latest_version);
    } else {
        ESP_LOGI(TAG, "OTA trigger received: current=%s latest version not published", current);
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

    runtime_safety_wdt_register();

    while (!atomic_load(&s_stop_requested)) {
        if (xQueueReceive(s_dosing_queue, &cmd, pdMS_TO_TICKS(DOSING_QUEUE_RECV_TIMEOUT_MS)) == pdTRUE) {
            if (runtime_safety_is_safe_mode() || atomic_load(&s_stop_requested)) {
                ESP_LOGW(TAG, "Dropping dosing command while in safe mode or stopping");
                continue;
            }

            if ((cmd.action == ACTUATOR_ACTION_ON || cmd.action == ACTUATOR_ACTION_PULSE) &&
                ((cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP && runtime_safety_get_channel_state(ACTUATOR_CHANNEL_PER_PH_DOWN)) ||
                 (cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN && runtime_safety_get_channel_state(ACTUATOR_CHANNEL_PER_PH_UP)))) {
                runtime_safety_fault_set(SAFETY_FAULT_PH_INTERLOCK, "pH interlock violation");
                continue;
            }

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
            default:
                ret = ESP_ERR_INVALID_ARG;
                break;
            }

            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Dosing command failed: %s", esp_err_to_name(ret));
                continue;
            }

            runtime_safety_dose_watchdog_update(cmd.channel, cmd.action, cmd.pulse_ms);

            if (cmd.channel == ACTUATOR_CHANNEL_VALVE) {
                runtime_safety_note_valve_action(cmd.action);
            }

            if (cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP || cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN) {
                runtime_safety_note_ph_dose(cmd.channel);
            }

            if (cmd.channel == ACTUATOR_CHANNEL_PER_NUTA || cmd.channel == ACTUATOR_CHANNEL_PER_NUTB) {
                runtime_safety_note_tds_dose(cmd.channel);
            }
        }

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

static void sensor_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    runtime_safety_wdt_register();

    while (!atomic_load(&s_stop_requested)) {
        esp_err_t ret = sensor_telemetry_sample();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Sensor sample failed: %s", esp_err_to_name(ret));
        }

        runtime_safety_heartbeat_sensor();
        runtime_safety_wdt_kick();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_SAMPLE_INTERVAL_MS));
    }

    runtime_safety_wdt_unregister();

    signal_task_exit(TASK_EXIT_BIT_SENSOR);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_sensor_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}

static void comm_task_publish_sensors(const sensor_telemetry_snapshot_t *snap,
                                      const char *zone_id)
{
    (void)zone_id;
    char buf[32];

    /* ── Water level ──────────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%d", snap->water_level);
    mqtt_manager_publish(sensor_telemetry_topic_water_level(), buf, 0, 0);

    /* ── Water temperature ────────────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%.2f", (double)snap->water_temp);
    mqtt_manager_publish(sensor_telemetry_topic_water_temp(), buf, 0, 0);

    /* ── pH  raw / state / valid ──────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%u", (unsigned)snap->ph_raw);
    mqtt_manager_publish(sensor_telemetry_topic_ph_raw(), buf, 0, 0);

    /* Req 7.3/7.4: suppress pH/state when calibration is invalid */
    if (snap->ph_valid) {
        snprintf(buf, sizeof(buf), "%.2f", (double)snap->ph);
        mqtt_manager_publish(sensor_telemetry_topic_ph(), buf, 0, 0);
    }

    mqtt_manager_publish(sensor_telemetry_topic_ph_valid(),
                         snap->ph_valid ? "true" : "false", 0, /*retain=*/1);

    /* ── TDS  raw / state / valid ─────────────────────────────────────── */
    snprintf(buf, sizeof(buf), "%u", (unsigned)snap->tds_raw);
    mqtt_manager_publish(sensor_telemetry_topic_tds_raw(), buf, 0, 0);

    /* Req 7.3/7.4: suppress TDS/state when calibration is invalid */
    if (snap->tds_valid) {
        snprintf(buf, sizeof(buf), "%.1f", (double)snap->tds);
        mqtt_manager_publish(sensor_telemetry_topic_tds(), buf, 0, 0);
    }

    mqtt_manager_publish(sensor_telemetry_topic_tds_valid(),
                         snap->tds_valid ? "true" : "false", 0, /*retain=*/1);
}

static void comm_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_version_publish = last_wake;

    runtime_safety_wdt_register();

    while (!atomic_load(&s_stop_requested)) {
        TickType_t now = xTaskGetTickCount();
        sensor_telemetry_snapshot_t snap = {0};
        bool snapshot_available = false;

        if (ticks_to_ms(now - last_version_publish) >= VERSION_PUBLISH_INTERVAL_MS) {
            publish_current_version(s_zone_id);
            last_version_publish = now;
        }

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
            comm_task_publish_sensors(&snap, s_zone_id);
        }

        actuator_last_command_t latest_cmd;
        if (actuator_control_get_last_command(&latest_cmd) == ESP_OK) {
            if (strcmp(latest_cmd.topic, s_last_lcd_rendered_cmd.topic) != 0 ||
                strcmp(latest_cmd.command, s_last_lcd_rendered_cmd.command) != 0 ||
                latest_cmd.state_on != s_last_lcd_rendered_cmd.state_on) {
                actuator_channel_t channel;
                char sensor_text[20];

                if (channel_from_name(latest_cmd.channel, &channel) == ESP_OK) {
                    switch (channel) {
                    case ACTUATOR_CHANNEL_VALVE:
                        snprintf(sensor_text, sizeof(sensor_text), "WaterLvl: %d", snapshot_available ? snap.water_level : 0);
                        break;
                    case ACTUATOR_CHANNEL_PER_NUTA:
                    case ACTUATOR_CHANNEL_PER_NUTB:
                        snprintf(sensor_text, sizeof(sensor_text), "TDS: %.1f", snapshot_available ? (double)snap.tds : 0.0);
                        break;
                    case ACTUATOR_CHANNEL_PER_PH_UP:
                    case ACTUATOR_CHANNEL_PER_PH_DOWN:
                        snprintf(sensor_text, sizeof(sensor_text), "pH: %.2f", snapshot_available ? (double)snap.ph : 0.0);
                        break;
                    default:
                        snprintf(sensor_text, sizeof(sensor_text), "Sensor: N/A");
                        break;
                    }
                } else {
                    snprintf(sensor_text, sizeof(sensor_text), "Sensor: N/A");
                }

                const char *state_text = (latest_cmd.command[0] != '\0' && strcmp(latest_cmd.command, "N/A") != 0)
                    ? latest_cmd.command
                    : (latest_cmd.state_on ? "ON" : "OFF");

                lcd_status_show_actuator_event(
                    s_zone_id,
                    s_zone_name,
                    latest_cmd.channel,
                    sensor_text,
                    state_text);

                s_last_lcd_rendered_cmd = latest_cmd;
            }
        }

        runtime_safety_heartbeat_comm();
        runtime_safety_wdt_kick();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COMM_INTERVAL_MS));
    }

    runtime_safety_wdt_unregister();

    signal_task_exit(TASK_EXIT_BIT_COMM);
    portENTER_CRITICAL(&s_task_handle_lock);
    s_comm_task = NULL;
    portEXIT_CRITICAL(&s_task_handle_lock);
    vTaskDelete(NULL);
}


static void cleanup_runtime_locked(void)
{
    runtime_safety_set_gate(false);
    atomic_store(&s_stop_requested, true);

    runtime_safety_set_dosing_queue(NULL);
    (void)mqtt_manager_set_connection_cb(NULL, NULL);

    const mqtt_unsub_entry_t unsub_entries[] = {
        { &s_zone_topic_subscribed, s_zone_command_topic },
        { &s_safety_topic_subscribed, s_safety_clear_topic },
        { &s_ota_latest_subscribed, s_ota_latest_topic },
        { &s_ota_trigger_subscribed, s_ota_trigger_topic },
        { &s_emergency_subscribed, s_emergency_topic },
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

            /* Forcibly unregister any tasks that didn't exit cleanly from the
             * task WDT before vTaskDelete kills them.  If we skip this, the
             * old (soon-to-be-deleted) handle stays in the WDT subscriber
             * list and corrupts WDT state for the next runtime_tasks_start. */
            for (size_t i = 0; i < ARRAY_SIZE(s_task_slots); i++) {
                TaskHandle_t h = *s_task_slots[i].handle;
                if (h != NULL) {
                    esp_err_t wdt_ret = esp_task_wdt_delete(h);
                    if (wdt_ret != ESP_OK && wdt_ret != ESP_ERR_INVALID_ARG) {
                        ESP_LOGW(TAG, "WDT deregister for stale task failed: %s",
                                 esp_err_to_name(wdt_ret));
                    }
                }
            }
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

    runtime_safety_bind(NULL);

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
    s_emergency_topic[0] = '\0';
    s_ota_latest_version[0] = '\0';
    memset(&s_last_lcd_rendered_cmd, 0, sizeof(s_last_lcd_rendered_cmd));
    memset(&s_last_good_snapshot, 0, sizeof(s_last_good_snapshot));
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
        runtime_safety_or_boot_faults(SAFETY_FAULT_WDT);
    }

    if (reason == ESP_RST_BROWNOUT) {
        runtime_safety_or_boot_faults(SAFETY_FAULT_POWER);
    }

    if (s_reset_count > SAFETY_RESET_MAX_COUNT) {
        runtime_safety_or_boot_faults(SAFETY_FAULT_POWER);
    }

    if (s_wdt_reset_count > SAFETY_WDT_RESET_MAX_COUNT) {
        runtime_safety_or_boot_faults(SAFETY_FAULT_WDT);
    }

    ESP_LOGI(TAG, "Reset reason=%d reset_count=%lu wdt_count=%lu",
             (int)reason,
             (unsigned long)s_reset_count,
             (unsigned long)s_wdt_reset_count);
}

static void runtime_prepare_state(const char *zone_id)
{
    atomic_store(&s_stop_requested, false);
    clear_subscription_tracking();
    s_ota_latest_version[0] = '\0';

    runtime_safety_reset_state();
    runtime_safety_reset_heartbeats();

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

static esp_err_t runtime_setup_resources(const char *zone_id)
{
    esp_err_t ret = indicator_led_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "indicator_led_init failed: %s", esp_err_to_name(ret));
        /* non-fatal — continue without LEDs */
    }

    ret = actuator_control_init(zone_id);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = sensor_telemetry_init(zone_id);
    if (ret != ESP_OK) {
        return ret;
    }

    runtime_safety_wdt_init();

    s_dosing_queue = xQueueCreate(DOSING_QUEUE_LEN, sizeof(dosing_command_t));
    if (s_dosing_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (s_snapshot_mutex == NULL) {
        s_snapshot_mutex = xSemaphoreCreateMutex();
        if (s_snapshot_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_task_exit_event == NULL) {
        s_task_exit_event = xEventGroupCreate();
        if (s_task_exit_event == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

static esp_err_t runtime_setup_topics(const char *zone_id)
{
    esp_err_t ret = subscribe_formatted_topic(s_zone_command_topic, ZONE_TOPIC_BUFFER_SIZE,
                                              "%s/command", zone_id, NULL,
                                              on_zone_command, NULL, &s_zone_topic_subscribed);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = write_topic(s_safety_fault_topic, ZONE_TOPIC_BUFFER_SIZE,
                      "%s/%s", zone_id, SAFETY_FAULT_TOPIC_SUFFIX);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = subscribe_formatted_topic(s_safety_clear_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/%s", zone_id, SAFETY_CLEAR_TOPIC_SUFFIX,
                                    on_safety_clear_command, NULL, &s_safety_topic_subscribed);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = subscribe_formatted_topic(s_ota_latest_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/ota/latest_version", zone_id, NULL,
                                    on_ota_latest_version, NULL, &s_ota_latest_subscribed);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = subscribe_formatted_topic(s_ota_trigger_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/ota/trigger", zone_id, NULL,
                                    on_ota_trigger, NULL, &s_ota_trigger_subscribed);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = subscribe_formatted_topic(s_emergency_topic, ZONE_TOPIC_BUFFER_SIZE,
                                    "%s/emergency", zone_id, NULL,
                                    on_emergency_command, NULL, &s_emergency_subscribed);
    if (ret != ESP_OK) {
        return ret;
    }

    publish_current_version(zone_id);
    (void)mqtt_manager_set_connection_cb(runtime_tasks_on_mqtt_connection, NULL);
    if (mqtt_manager_is_connected()) {
        runtime_tasks_on_mqtt_connection(true, NULL);
    }

    for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
        actuator_channel_t ch = (actuator_channel_t)i;
        const char *cmd_topic = actuator_control_get_command_topic(ch);
        if (cmd_topic == NULL || cmd_topic[0] == '\0') {
            continue;
        }

        ret = mqtt_manager_subscribe(cmd_topic, 1, on_channel_command, (void *)(intptr_t)ch);
        if (ret == ESP_OK) {
            s_channel_topics_subscribed[i] = true;
        }
    }

    return ESP_OK;
}

static void runtime_apply_boot_faults(void)
{
    safety_fault_mask_t boot_faults = runtime_safety_get_boot_faults();
    if (boot_faults != 0) {
        runtime_safety_fault_set(boot_faults, "Boot fault");
    }

    safety_fault_mask_t existing_faults = runtime_safety_get_faults();
    if (existing_faults != 0) {
        runtime_safety_set_gate(false);
        
        char fault_msg[128];
        snprintf(fault_msg, sizeof(fault_msg), "Faults: 0x%08X", (unsigned int)existing_faults);
        esp_err_t ret = mqtt_manager_publish(s_safety_fault_topic, fault_msg, 1, 1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to publish boot faults: %s", esp_err_to_name(ret));
        }
    }
}

static esp_err_t runtime_start_workers(void)
{
    BaseType_t ok = xTaskCreate(dosing_task, "DosingTask", DOSING_TASK_STACK, NULL, DOSING_TASK_PRIORITY, &s_dosing_task);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    ok = xTaskCreate(sensor_task, "SensorTask", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIORITY, &s_sensor_task);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    ok = xTaskCreate(comm_task, "CommTask", COMM_TASK_STACK, NULL, COMM_TASK_PRIORITY, &s_comm_task);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    runtime_safety_bindings_t bindings = {
        .dosing_queue = &s_dosing_queue,
        .dosing_task = &s_dosing_task,
        .sensor_task = &s_sensor_task,
        .comm_task = &s_comm_task,
        .safety_task = &s_safety_task,
        .stop_requested = &s_stop_requested,
        .zone_id = s_zone_id,
        .zone_name = s_zone_name,
        .safety_fault_topic = s_safety_fault_topic,
    };

    ok = xTaskCreate(runtime_safety_task, "SafetyTask", SAFETY_TASK_STACK, &bindings, SAFETY_TASK_PRIORITY, &s_safety_task);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
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
    runtime_prepare_state(zone_id);

    ret = runtime_setup_resources(zone_id);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    ret = runtime_setup_topics(zone_id);
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    runtime_apply_boot_faults();

    ret = runtime_start_workers();
    if (ret != ESP_OK) {
        cleanup_runtime_locked();
        runtime_unlock();
        return ret;
    }

    if (!runtime_safety_is_safe_mode()) {
        runtime_safety_set_gate(true);
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
    return runtime_safety_get_faults();
}

esp_err_t runtime_tasks_clear_safety_faults(safety_fault_mask_t mask)
{
    bool safe_mode_cleared = false;
    esp_err_t ret = runtime_safety_clear_faults(mask, &safe_mode_cleared);
    if (ret != ESP_OK) {
        return ret;
    }

    if (safe_mode_cleared && s_running && !s_starting && !atomic_load(&s_stop_requested)) {
        runtime_safety_set_gate(true);
    }

    return ESP_OK;
}

bool runtime_tasks_is_safe_mode(void)
{
    return runtime_safety_is_safe_mode();
}