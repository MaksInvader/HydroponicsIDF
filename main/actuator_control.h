#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
	char topic[96];
	char channel[24];
	char command[8];
	bool state_on;
} actuator_last_command_t;

#define ACTUATOR_CHANNEL_COUNT 5

typedef enum {
	ACTUATOR_CHANNEL_VALVE = 0,
	ACTUATOR_CHANNEL_PER_NUTA,
	ACTUATOR_CHANNEL_PER_NUTB,
	ACTUATOR_CHANNEL_PER_PH_UP,
	ACTUATOR_CHANNEL_PER_PH_DOWN,
} actuator_channel_t;

typedef enum {
	ACTUATOR_ACTION_OFF = 0,
	ACTUATOR_ACTION_ON,
	ACTUATOR_ACTION_PULSE,
} actuator_action_t;

typedef struct {
	actuator_channel_t channel;
	actuator_action_t action;
	uint32_t pulse_ms;
} actuator_command_t;

esp_err_t actuator_control_init(const char *zone_id);
esp_err_t actuator_control_deinit(void);
esp_err_t actuator_control_apply_state(actuator_channel_t channel, bool is_on);
esp_err_t actuator_control_apply_pulse(actuator_channel_t channel, uint32_t pulse_ms);
esp_err_t actuator_control_parse_action_payload(const char *payload, int payload_len, actuator_action_t *action, uint32_t *pulse_ms);
const char *actuator_control_get_command_topic(actuator_channel_t channel);
const char *actuator_control_get_status_topic(actuator_channel_t channel);
const char *actuator_control_get_channel_name(actuator_channel_t channel);
esp_err_t actuator_control_channel_from_topic(const char *topic, actuator_channel_t *channel);

esp_err_t actuator_control_start(const char *zone_id);
esp_err_t actuator_control_stop(void);
esp_err_t actuator_control_get_last_command(actuator_last_command_t *out);
esp_err_t actuator_control_get_all_last_commands(actuator_last_command_t *out_array, size_t max_items, size_t *out_count);

#endif