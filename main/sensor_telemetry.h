#ifndef SENSOR_TELEMETRY_H
#define SENSOR_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* --------------------------------------------------------------------------
 * Snapshot exposed to the rest of the firmware
 * -------------------------------------------------------------------------- */
typedef struct {
    bool valid;
    int  water_level;
    float water_temp;

    /* pH */
    uint16_t ph_raw;        /* averaged raw ADC counts                        */
    float    ph;            /* calibrated value (0–14)                        */
    bool     ph_valid;      /* calibration present AND value in sane range    */

    /* TDS */
    uint16_t tds_raw;       /* averaged raw ADC counts                        */
    float    tds;           /* calibrated value (ppm)                         */
    bool     tds_valid;     /* calibration present AND value in sane range    */

    uint32_t publish_count;
} sensor_telemetry_snapshot_t;

/* --------------------------------------------------------------------------
 * Linear calibration coefficients stored / loaded from NVS.
 *
 * physical_value = slope * raw_counts + offset
 * -------------------------------------------------------------------------- */
typedef struct {
    float    slope;
    float    offset;
    bool     valid;         /* false until at least one successful calibration */
    uint32_t updated_at;    /* Unix timestamp supplied by the calibration msg  */
} calibration_t;

/* --------------------------------------------------------------------------
 * Lifecycle
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialise sensor interfaces, load calibration from NVS, build
 *         MQTT topic strings, reset history buffers, subscribe to calibration
 *         command topics.
 *
 * @param  zone_id  NUL-terminated zone identifier (e.g. "zone_1").
 * @return ESP_OK on success.
 */
esp_err_t sensor_telemetry_init(const char *zone_id);

/**
 * @brief  Release all resources acquired by sensor_telemetry_init().
 */
esp_err_t sensor_telemetry_deinit(void);

/* Thin wrappers kept for backward compatibility with runtime_tasks.c */
esp_err_t sensor_telemetry_start(const char *zone_id);
esp_err_t sensor_telemetry_stop(void);

/* --------------------------------------------------------------------------
 * Sampling
 * -------------------------------------------------------------------------- */

/**
 * @brief  Take one set of ADC samples (multi-sample averaged), apply
 *         calibration, update the internal snapshot.
 *
 *         Call from the sensor task at the desired rate (e.g. 1 Hz).
 */
esp_err_t sensor_telemetry_sample(void);

/* --------------------------------------------------------------------------
 * Topic accessors
 *
 * The full multi-topic design is:
 *   <zone_id>/sensor/pH/raw          published uint16 ADC counts
 *   <zone_id>/sensor/pH/state        published float calibrated value
 *   <zone_id>/sensor/pH/valid        published "true" / "false"
 *   <zone_id>/sensor/TDS/raw
 *   <zone_id>/sensor/TDS/state
 *   <zone_id>/sensor/TDS/valid
 *   <zone_id>/sensor/WaterLevel/state
 *   <zone_id>/sensor/WaterTemp/state
 *
 * Calibration (subscribed, inbound):
 *   <zone_id>/calibration/pH/set
 *   <zone_id>/calibration/TDS/set
 *
 * Calibration state (published, outbound):
 *   <zone_id>/calibration/pH/state
 *   <zone_id>/calibration/TDS/state
 * -------------------------------------------------------------------------- */

/* Legacy single-topic accessors (still used by comm_task publish loop) */
const char *sensor_telemetry_topic_water_level(void);
const char *sensor_telemetry_topic_water_temp(void);
const char *sensor_telemetry_topic_ph(void);    /* → .../pH/state          */
const char *sensor_telemetry_topic_tds(void);   /* → .../TDS/state         */

/* Extended per-sensor topic accessors */
const char *sensor_telemetry_topic_ph_raw(void);
const char *sensor_telemetry_topic_ph_valid(void);
const char *sensor_telemetry_topic_tds_raw(void);
const char *sensor_telemetry_topic_tds_valid(void);

const char *sensor_telemetry_topic_cali_ph_state(void);
const char *sensor_telemetry_topic_cali_tds_state(void);

/* --------------------------------------------------------------------------
 * Snapshot query
 * -------------------------------------------------------------------------- */
esp_err_t sensor_telemetry_get_snapshot(sensor_telemetry_snapshot_t *out);

/* --------------------------------------------------------------------------
 * Calibration query helpers (for use by safety / comm tasks)
 * -------------------------------------------------------------------------- */
bool sensor_telemetry_ph_calibrated(void);
bool sensor_telemetry_tds_calibrated(void);

/**
 * @brief  Copy out the current calibration structs (e.g. to publish state).
 */
void sensor_telemetry_get_ph_calibration(calibration_t *out);
void sensor_telemetry_get_tds_calibration(calibration_t *out);

#endif /* SENSOR_TELEMETRY_H */