#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "actuator_control.h"
#include "mqtt_manager.h"
#include "pin_config.h"

#define ACTUATOR_TOPIC_LEN_MAX 96
#define ACTUATOR_MAX_PULSE_MS 10000U

typedef struct {
    const char *name;
    int gpio;
    char command_topic[ACTUATOR_TOPIC_LEN_MAX];
    char status_topic[ACTUATOR_TOPIC_LEN_MAX];
} actuator_entry_t;

static const char *TAG = "actuator_ctrl";

static const int s_gpio_list[] = {
    PIN_ACTUATOR_VALVE,
    PIN_ACTUATOR_PER_NUTA,
    PIN_ACTUATOR_PER_NUTB,
    PIN_ACTUATOR_PER_PH_UP,
    PIN_ACTUATOR_PER_PH_DOWN,
};

static actuator_entry_t s_actuators[] = {
    {.name = "valve", .gpio = PIN_ACTUATOR_VALVE},
    {.name = "PerNutA", .gpio = PIN_ACTUATOR_PER_NUTA},
    {.name = "PerNutB", .gpio = PIN_ACTUATOR_PER_NUTB},
    {.name = "PerpHUp", .gpio = PIN_ACTUATOR_PER_PH_UP},
    {.name = "PerpHDown", .gpio = PIN_ACTUATOR_PER_PH_DOWN},
};

static bool s_initialized;
static bool s_topics_bound;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static actuator_last_command_t s_last_commands[ACTUATOR_CHANNEL_COUNT];
static actuator_last_command_t s_last_command = {
    .topic = "N/A",
    .channel = "N/A",
    .command = "N/A",
    .state_on = false,
};

static bool is_valid_channel(actuator_channel_t channel)
{
    return channel >= ACTUATOR_CHANNEL_VALVE && channel <= ACTUATOR_CHANNEL_PER_PH_DOWN;
}

static const actuator_entry_t *entry_for_channel(actuator_channel_t channel)
{
    if (!is_valid_channel(channel)) {
        return NULL;
    }
    return &s_actuators[(size_t)channel];
}

static esp_err_t publish_status(const actuator_entry_t *actuator, bool is_on)
{
    if (actuator == NULL || actuator->status_topic[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }
    return mqtt_manager_publish(actuator->status_topic, is_on ? "ON" : "OFF", 1, 0);
}

static void update_last_command_locked(const actuator_entry_t *actuator, actuator_channel_t channel, bool is_on)
{
    strncpy(s_last_command.topic, actuator->command_topic, sizeof(s_last_command.topic) - 1);
    s_last_command.topic[sizeof(s_last_command.topic) - 1] = '\0';
    strncpy(s_last_command.channel, actuator->name, sizeof(s_last_command.channel) - 1);
    s_last_command.channel[sizeof(s_last_command.channel) - 1] = '\0';
    strncpy(s_last_command.command, is_on ? "ON" : "OFF", sizeof(s_last_command.command) - 1);
    s_last_command.command[sizeof(s_last_command.command) - 1] = '\0';
    s_last_command.state_on = is_on;

    size_t idx = (size_t)channel;
    if (idx < ACTUATOR_CHANNEL_COUNT) {
        strncpy(s_last_commands[idx].topic, actuator->command_topic, sizeof(s_last_commands[idx].topic) - 1);
        s_last_commands[idx].topic[sizeof(s_last_commands[idx].topic) - 1] = '\0';
        strncpy(s_last_commands[idx].channel, actuator->name, sizeof(s_last_commands[idx].channel) - 1);
        s_last_commands[idx].channel[sizeof(s_last_commands[idx].channel) - 1] = '\0';
        strncpy(s_last_commands[idx].command, is_on ? "ON" : "OFF", sizeof(s_last_commands[idx].command) - 1);
        s_last_commands[idx].command[sizeof(s_last_commands[idx].command) - 1] = '\0';
        s_last_commands[idx].state_on = is_on;
    }
}

static esp_err_t init_gpios_once(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    for (size_t i = 0; i < sizeof(s_gpio_list) / sizeof(s_gpio_list[0]); i++) {
        cfg.pin_bit_mask |= (1ULL << s_gpio_list[i]);
    }

    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    for (size_t i = 0; i < sizeof(s_gpio_list) / sizeof(s_gpio_list[0]); i++) {
        gpio_set_level((gpio_num_t)s_gpio_list[i], 0);
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t actuator_control_init(const char *zone_id)
{
    if (zone_id == NULL || strlen(zone_id) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = init_gpios_once();
    if (ret != ESP_OK) {
        return ret;
    }

    for (size_t i = 0; i < sizeof(s_actuators) / sizeof(s_actuators[0]); i++) {
        int wrote_cmd = snprintf(
            s_actuators[i].command_topic,
            sizeof(s_actuators[i].command_topic),
            "%s/%s/command",
            zone_id,
            s_actuators[i].name);

        int wrote_status = snprintf(
            s_actuators[i].status_topic,
            sizeof(s_actuators[i].status_topic),
            "%s/%s/status",
            zone_id,
            s_actuators[i].name);

        if (wrote_cmd <= 0 || wrote_cmd >= (int)sizeof(s_actuators[i].command_topic) ||
            wrote_status <= 0 || wrote_status >= (int)sizeof(s_actuators[i].status_topic)) {
            return ESP_ERR_INVALID_SIZE;
        }

        portENTER_CRITICAL(&s_state_lock);
        strncpy(s_last_commands[i].topic, s_actuators[i].command_topic, sizeof(s_last_commands[i].topic) - 1);
        s_last_commands[i].topic[sizeof(s_last_commands[i].topic) - 1] = '\0';
        strncpy(s_last_commands[i].channel, s_actuators[i].name, sizeof(s_last_commands[i].channel) - 1);
        s_last_commands[i].channel[sizeof(s_last_commands[i].channel) - 1] = '\0';
        strncpy(s_last_commands[i].command, "OFF", sizeof(s_last_commands[i].command) - 1);
        s_last_commands[i].command[sizeof(s_last_commands[i].command) - 1] = '\0';
        s_last_commands[i].state_on = false;
        portEXIT_CRITICAL(&s_state_lock);

        gpio_set_level((gpio_num_t)s_actuators[i].gpio, 0);
        ret = publish_status(&s_actuators[i], false);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Initial status publish failed for %s", s_actuators[i].name);
        }
    }

    s_topics_bound = true;
    ESP_LOGI(TAG, "Actuator control initialized for zone: %s", zone_id);
    return ESP_OK;
}

esp_err_t actuator_control_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    for (size_t i = 0; i < sizeof(s_gpio_list) / sizeof(s_gpio_list[0]); i++) {
        gpio_set_level((gpio_num_t)s_gpio_list[i], 0);
    }

    s_topics_bound = false;
    ESP_LOGI(TAG, "Actuator control deinitialized");
    return ESP_OK;
}

esp_err_t actuator_control_apply_state(actuator_channel_t channel, bool is_on)
{
    const actuator_entry_t *actuator = entry_for_channel(channel);
    if (!s_initialized || !s_topics_bound || actuator == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level((gpio_num_t)actuator->gpio, is_on ? 1 : 0);

    portENTER_CRITICAL(&s_state_lock);
    update_last_command_locked(actuator, channel, is_on);
    portEXIT_CRITICAL(&s_state_lock);

    esp_err_t ret = publish_status(actuator, is_on);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish status for %s", actuator->name);
    }

    return ESP_OK;
}

esp_err_t actuator_control_apply_pulse(actuator_channel_t channel, uint32_t pulse_ms)
{
    if (pulse_ms == 0 || pulse_ms > ACTUATOR_MAX_PULSE_MS) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = actuator_control_apply_state(channel, true);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    return actuator_control_apply_state(channel, false);
}

esp_err_t actuator_control_parse_action_payload(const char *payload, int payload_len, actuator_action_t *action, uint32_t *pulse_ms)
{
    if (payload == NULL || action == NULL || payload_len <= 0 || payload_len > 48) {
        return ESP_ERR_INVALID_ARG;
    }

    char buf[49];
    for (int i = 0; i < payload_len; i++) {
        buf[i] = (char)toupper((unsigned char)payload[i]);
    }
    buf[payload_len] = '\0';

    if (strcmp(buf, "ON") == 0) {
        *action = ACTUATOR_ACTION_ON;
        if (pulse_ms != NULL) {
            *pulse_ms = 0;
        }
        return ESP_OK;
    }

    if (strcmp(buf, "OFF") == 0) {
        *action = ACTUATOR_ACTION_OFF;
        if (pulse_ms != NULL) {
            *pulse_ms = 0;
        }
        return ESP_OK;
    }

    const char *prefix = "PULSE:";
    size_t prefix_len = strlen(prefix);
    if (strncmp(buf, prefix, prefix_len) == 0) {
        char *end_ptr = NULL;
        unsigned long parsed = strtoul(buf + prefix_len, &end_ptr, 10);
        if (end_ptr == buf + prefix_len || *end_ptr != '\0' || parsed == 0 || parsed > ACTUATOR_MAX_PULSE_MS) {
            return ESP_ERR_INVALID_ARG;
        }
        *action = ACTUATOR_ACTION_PULSE;
        if (pulse_ms != NULL) {
            *pulse_ms = (uint32_t)parsed;
        }
        return ESP_OK;
    }

    return ESP_ERR_INVALID_ARG;
}

const char *actuator_control_get_command_topic(actuator_channel_t channel)
{
    const actuator_entry_t *actuator = entry_for_channel(channel);
    if (actuator == NULL) {
        return NULL;
    }
    return actuator->command_topic;
}

const char *actuator_control_get_status_topic(actuator_channel_t channel)
{
    const actuator_entry_t *actuator = entry_for_channel(channel);
    if (actuator == NULL) {
        return NULL;
    }
    return actuator->status_topic;
}

const char *actuator_control_get_channel_name(actuator_channel_t channel)
{
    const actuator_entry_t *actuator = entry_for_channel(channel);
    if (actuator == NULL) {
        return NULL;
    }
    return actuator->name;
}

esp_err_t actuator_control_channel_from_topic(const char *topic, actuator_channel_t *channel)
{
    if (topic == NULL || channel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < sizeof(s_actuators) / sizeof(s_actuators[0]); i++) {
        if (strcmp(topic, s_actuators[i].command_topic) == 0) {
            *channel = (actuator_channel_t)i;
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t actuator_control_start(const char *zone_id)
{
    return actuator_control_init(zone_id);
}

esp_err_t actuator_control_stop(void)
{
    return actuator_control_deinit();
}

esp_err_t actuator_control_get_last_command(actuator_last_command_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_state_lock);
    *out = s_last_command;
    portEXIT_CRITICAL(&s_state_lock);
    return ESP_OK;
}

esp_err_t actuator_control_get_all_last_commands(actuator_last_command_t *out_array, size_t max_items, size_t *out_count)
{
    if (out_array == NULL || out_count == NULL || max_items == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t n = ACTUATOR_CHANNEL_COUNT;
    if (n > max_items) {
        n = max_items;
    }

    portENTER_CRITICAL(&s_state_lock);
    for (size_t i = 0; i < n; i++) {
        out_array[i] = s_last_commands[i];
    }
    portEXIT_CRITICAL(&s_state_lock);

    *out_count = n;
    return ESP_OK;
}
