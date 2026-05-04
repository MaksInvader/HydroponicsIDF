#ifndef SENSOR_TELEMETRY_H
#define SENSOR_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
	bool valid;
	int water_level;
	float water_temp;
	float ph;
	float tds;
	uint32_t publish_count;
} sensor_telemetry_snapshot_t;

esp_err_t sensor_telemetry_init(const char *zone_id);
esp_err_t sensor_telemetry_deinit(void);
esp_err_t sensor_telemetry_sample(void);
const char *sensor_telemetry_topic_water_level(void);
const char *sensor_telemetry_topic_water_temp(void);
const char *sensor_telemetry_topic_ph(void);
const char *sensor_telemetry_topic_tds(void);

esp_err_t sensor_telemetry_start(const char *zone_id);
esp_err_t sensor_telemetry_stop(void);
esp_err_t sensor_telemetry_get_snapshot(sensor_telemetry_snapshot_t *out);

#endif