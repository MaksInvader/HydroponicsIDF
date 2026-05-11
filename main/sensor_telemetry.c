/*
 * sensor_telemetry.c
 *
 * Data flow
 * ─────────
 *   Water-temp: ADC raw (N samples) ──► filtered average ──► apply calibration
 *     ──► validate range ──► snapshot ──► publish raw / state / valid
 *   pH / TDS:   GPIO digital level ──► apply calibration
 *     ──► validate range ──► snapshot ──► publish raw / state / valid
 *
 * MQTT topics (published, outbound)
 * ───────────────────────────────────
 *   <zone>/sensor/pH/raw          uint16 GPIO level (0 or 1)
 *   <zone>/sensor/pH/state        float  calibrated pH
 *   <zone>/sensor/pH/valid        "true" | "false"
 *   <zone>/sensor/TDS/raw
 *   <zone>/sensor/TDS/state
 *   <zone>/sensor/TDS/valid
 *   <zone>/sensor/WaterLevel/state
 *   <zone>/sensor/WaterTemp/state
 *   <zone>/calibration/pH/state   JSON coefficients
 *   <zone>/calibration/TDS/state
 *
 * MQTT topics (subscribed, inbound)
 * ───────────────────────────────────
 *   <zone>/calibration/pH/set     JSON {"mode","slope","offset","valid","updated_at"}
 *   <zone>/calibration/TDS/set
 */

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "mqtt_manager.h"
#include "pin_config.h"
#include "sensor_calibration_nvs.h"
#include "sensor_telemetry.h"

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */

/** Number of ADC samples to average per call (water temperature only). */
#define ADC_OVERSAMPLE_COUNT    16

/** Rolling window depth for the per-sensor moving average (all sensors). */
#define SENSOR_SAMPLE_WINDOW    30

/** ESP-IDF ADC configuration for all analogue sensor channels. */
#define SENSOR_ADC_ATTEN        ADC_ATTEN_DB_12
#define SENSOR_ADC_BITWIDTH     ADC_BITWIDTH_12

/** Maximum raw ADC count (12-bit). Water-temp readings at rail are treated as faults. */
#define ADC_RAW_MAX             4095
#define ADC_RAW_STUCK_MARGIN    10  /* counts within 0 or max are "stuck"    */

/** Valid physical ranges — readings outside these mark the reading invalid. */
#define PH_VALUE_MIN            0.0f
#define PH_VALUE_MAX            14.0f
#define TDS_VALUE_MIN           0.0f
#define TDS_VALUE_MAX           5000.0f  /* ppm; adjust for your application */

/* --------------------------------------------------------------------------
 * Topic buffer sizes
 * -------------------------------------------------------------------------- */
#define TOPIC_BUF_LEN   96

/* --------------------------------------------------------------------------
 * Internal types
 * -------------------------------------------------------------------------- */

typedef struct {
    /* Published telemetry topics */
    char water_level_state[TOPIC_BUF_LEN];
    char water_temp_state[TOPIC_BUF_LEN];

    char ph_raw[TOPIC_BUF_LEN];
    char ph_state[TOPIC_BUF_LEN];
    char ph_valid[TOPIC_BUF_LEN];

    char tds_raw[TOPIC_BUF_LEN];
    char tds_state[TOPIC_BUF_LEN];
    char tds_valid[TOPIC_BUF_LEN];

    /* Calibration state publish */
    char cali_ph_state[TOPIC_BUF_LEN];
    char cali_tds_state[TOPIC_BUF_LEN];

    /* Calibration command subscribe */
    char cali_ph_set[TOPIC_BUF_LEN];
    char cali_tds_set[TOPIC_BUF_LEN];
} sensor_topics_t;

/** One sensor's averaged raw reading plus the derived physical value. */
typedef struct {
    uint16_t raw;   /* averaged ADC counts                               */
    float    value; /* calibrated physical value (pH / ppm)              */
    bool     valid; /* true only when calibration + range checks pass    */
} sensor_reading_t;

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */

static const char *TAG = "sensor_telem";

static sensor_topics_t            s_topics;
static adc_oneshot_unit_handle_t  s_adc_handle;
static adc_cali_handle_t          s_adc_cali;
static bool                       s_adc_cali_ready;
static bool                       s_initialized;

static portMUX_TYPE               s_snapshot_lock  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE               s_cali_lock      = portMUX_INITIALIZER_UNLOCKED;

static sensor_telemetry_snapshot_t s_snapshot;

/* Rolling history buffers */
static float  s_temp_history[SENSOR_SAMPLE_WINDOW];
static float  s_ph_history[SENSOR_SAMPLE_WINDOW];
static float  s_tds_history[SENSOR_SAMPLE_WINDOW];
static size_t s_history_index;
static size_t s_history_count;

/* Calibration state (protected by s_cali_lock) */
static calibration_t s_ph_cali;
static calibration_t s_tds_cali;

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void on_cali_ph_set(const char *topic, const char *payload,
                           int payload_len, void *user_ctx);
static void on_cali_tds_set(const char *topic, const char *payload,
                            int payload_len, void *user_ctx);

/* --------------------------------------------------------------------------
 * Utilities
 * -------------------------------------------------------------------------- */

static float average_window(const float *values, size_t count)
{
    if (values == NULL || count == 0) return 0.0f;
    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) sum += values[i];
    return sum / (float)count;
}

static void reset_state_locked(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(s_temp_history, 0, sizeof(s_temp_history));
    memset(s_ph_history,   0, sizeof(s_ph_history));
    memset(s_tds_history,  0, sizeof(s_tds_history));
    s_history_index = 0;
    s_history_count = 0;
}

/** Return true when the raw count looks like a stuck rail. */
static bool adc_raw_is_stuck(int raw)
{
    return (raw <= ADC_RAW_STUCK_MARGIN) ||
           (raw >= (ADC_RAW_MAX - ADC_RAW_STUCK_MARGIN));
}

/* --------------------------------------------------------------------------
 * ADC helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief  Read one ADC channel, oversampling ADC_OVERSAMPLE_COUNT times.
 *
 * @param  channel   ADC channel to read.
 * @param  out_raw   Average raw ADC count (0–4095).
 * @param  out_mv    Average voltage in mV (uses curve-fitting if available).
 * @return ESP_OK, or ESP_FAIL if the read fails.
 */
static esp_err_t adc_read_averaged(adc_channel_t channel,
                                   uint16_t *out_raw, int *out_mv)
{
    if (s_adc_handle == NULL) return ESP_ERR_INVALID_STATE;

    int32_t sum_raw = 0;
    int     failures = 0;

    for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc_handle, channel, &raw) == ESP_OK) {
            sum_raw += raw;
        } else {
            failures++;
        }
    }

    int valid_samples = ADC_OVERSAMPLE_COUNT - failures;
    if (valid_samples == 0) {
        *out_raw = 0;
        *out_mv  = 0;
        return ESP_FAIL;
    }

    int avg_raw = (int)(sum_raw / valid_samples);

    int mv = 0;
    if (s_adc_cali_ready) {
        if (adc_cali_raw_to_voltage(s_adc_cali, avg_raw, &mv) != ESP_OK) {
            mv = (avg_raw * 3300) / ADC_RAW_MAX;
        }
    } else {
        mv = (avg_raw * 3300) / ADC_RAW_MAX;
    }

    *out_raw = (uint16_t)avg_raw;
    *out_mv  = mv;
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Calibration application
 * -------------------------------------------------------------------------- */

/**
 * @brief  Apply a linear calibration to a raw ADC count and validate the result.
 *
 * Used by the water-temperature sensor (ADC-based).
 *
 * @param  cali      Calibration coefficients (may be invalid).
 * @param  raw       Averaged ADC counts.
 * @param  mv        Averaged voltage (mV).
 * @param  val_min   Minimum physically sane value.
 * @param  val_max   Maximum physically sane value.
 * @param  out       Populated reading struct.
 */
static void apply_calibration_and_validate(const calibration_t *cali,
                                           uint16_t raw, int mv,
                                           float val_min, float val_max,
                                           sensor_reading_t *out)
{
    out->raw   = raw;
    out->value = 0.0f;
    out->valid = false;

    /* Gate 1: calibration must exist */
    if (!cali->valid) return;

    /* Gate 2: raw ADC must not be stuck at a rail */
    if (adc_raw_is_stuck((int)raw)) return;

    /* Apply linear model: value = slope * raw_counts + offset */
    float computed = cali->slope * (float)raw + cali->offset;

    /* Gate 3: result must be finite and within physical bounds */
    if (!isfinite(computed))          return;
    if (computed < val_min)           return;
    if (computed > val_max)           return;

    out->value = computed;
    out->valid = true;
}

/**
 * @brief  Apply a linear calibration to a GPIO digital level and validate.
 *
 * Used by pH and TDS sensors that output a direct digital signal on a GPIO pin.
 * The linear model maps the 0/1 level to a physical value via the stored
 * slope/offset coefficients (e.g. slope=14.0, offset=0.0 maps HIGH→14 pH).
 *
 * @param  cali      Calibration coefficients (may be invalid).
 * @param  gpio_level  Raw GPIO level read via gpio_get_level() (0 or 1).
 * @param  val_min   Minimum physically sane value.
 * @param  val_max   Maximum physically sane value.
 * @param  out       Populated reading struct.
 */
static void apply_gpio_calibration_and_validate(const calibration_t *cali,
                                                int gpio_level,
                                                float val_min, float val_max,
                                                sensor_reading_t *out)
{
    out->raw   = (uint16_t)gpio_level;
    out->value = 0.0f;
    out->valid = false;

    /* Gate 1: calibration must exist */
    if (!cali->valid) return;

    /* Apply linear model: value = slope * gpio_level + offset */
    float computed = cali->slope * (float)gpio_level + cali->offset;

    /* Gate 2: result must be finite and within physical bounds */
    if (!isfinite(computed)) return;
    if (computed < val_min)  return;
    if (computed > val_max)  return;

    out->value = computed;
    out->valid = true;
}

/* --------------------------------------------------------------------------
 * Water temperature (unchanged from original implementation)
 * -------------------------------------------------------------------------- */

static float estimate_water_temp_celsius(int millivolts)
{
    /* Placeholder — replace with sensor-specific transfer function. */
    return (float)millivolts / 10.0f;
}

static esp_err_t sample_water_temp(float *temp_c)
{
    if (temp_c == NULL) return ESP_ERR_INVALID_ARG;

    uint16_t raw = 0;
    int      mv  = 0;
    if (adc_read_averaged(PIN_SENSOR_WATER_TEMP_ADC_CHANNEL, &raw, &mv) == ESP_OK) {
        *temp_c = estimate_water_temp_celsius(mv);
        return ESP_OK;
    }
    *temp_c = 0.0f;
    return ESP_FAIL;
}

/* --------------------------------------------------------------------------
 * Topic building
 * -------------------------------------------------------------------------- */

#define WRITE_TOPIC(buf, fmt, ...)                                          \
    do {                                                                    \
        int _w = snprintf((buf), sizeof(buf), (fmt), ##__VA_ARGS__);       \
        if (_w <= 0 || _w >= (int)sizeof(buf)) {                           \
            ESP_LOGE(TAG, "Topic truncated: " fmt, ##__VA_ARGS__);         \
            return ESP_ERR_INVALID_SIZE;                                    \
        }                                                                   \
    } while (0)

static esp_err_t build_sensor_topics(const char *zone_id)
{
    WRITE_TOPIC(s_topics.water_level_state, "%s/sensor/WaterLevel/state", zone_id);
    WRITE_TOPIC(s_topics.water_temp_state,  "%s/sensor/WaterTemp/state",  zone_id);

    WRITE_TOPIC(s_topics.ph_raw,   "%s/sensor/pH/raw",   zone_id);
    WRITE_TOPIC(s_topics.ph_state, "%s/sensor/pH/state", zone_id);
    WRITE_TOPIC(s_topics.ph_valid, "%s/sensor/pH/valid", zone_id);

    WRITE_TOPIC(s_topics.tds_raw,   "%s/sensor/TDS/raw",   zone_id);
    WRITE_TOPIC(s_topics.tds_state, "%s/sensor/TDS/state", zone_id);
    WRITE_TOPIC(s_topics.tds_valid, "%s/sensor/TDS/valid", zone_id);

    WRITE_TOPIC(s_topics.cali_ph_state,  "%s/calibration/pH/state",  zone_id);
    WRITE_TOPIC(s_topics.cali_tds_state, "%s/calibration/TDS/state", zone_id);

    WRITE_TOPIC(s_topics.cali_ph_set,  "%s/calibration/pH/set",  zone_id);
    WRITE_TOPIC(s_topics.cali_tds_set, "%s/calibration/TDS/set", zone_id);

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Hardware initialisation
 * -------------------------------------------------------------------------- */

static esp_err_t ensure_sensor_interfaces(void)
{
    /* Water-level digital input */
    gpio_config_t level_cfg = {
        .pin_bit_mask = (1ULL << PIN_SENSOR_WATER_LEVEL),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&level_cfg);
    if (ret != ESP_OK) return ret;

    /* pH and TDS digital inputs (direct GPIO, not ADC) */
    gpio_config_t ph_tds_cfg = {
        .pin_bit_mask = (1ULL << PIN_SENSOR_PH) | (1ULL << PIN_SENSOR_TDS),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&ph_tds_cfg);
    if (ret != ESP_OK) return ret;

    /* ADC unit (water temperature only) */
    if (s_adc_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id  = PIN_SENSOR_WATER_TEMP_ADC_UNIT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ret = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
        if (ret != ESP_OK) return ret;

        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten    = SENSOR_ADC_ATTEN,
            .bitwidth = SENSOR_ADC_BITWIDTH,
        };

        /* Configure water-temperature ADC channel only */
        ret = adc_oneshot_config_channel(s_adc_handle,
                                         PIN_SENSOR_WATER_TEMP_ADC_CHANNEL,
                                         &chan_cfg);
        if (ret != ESP_OK) return ret;

        /* Attempt hardware curve-fitting calibration */
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = PIN_SENSOR_WATER_TEMP_ADC_UNIT,
            .chan     = PIN_SENSOR_WATER_TEMP_ADC_CHANNEL,
            .atten    = SENSOR_ADC_ATTEN,
            .bitwidth = SENSOR_ADC_BITWIDTH,
        };
        s_adc_cali = NULL;
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
            s_adc_cali_ready = true;
            ESP_LOGI(TAG, "ADC curve-fitting calibration enabled");
        } else {
            s_adc_cali_ready = false;
            s_adc_cali = NULL;
            ESP_LOGW(TAG, "ADC hardware calibration unavailable; using linear fallback");
        }
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * MQTT calibration subscription
 * -------------------------------------------------------------------------- */

static esp_err_t subscribe_calibration_topics(void)
{
    esp_err_t ret = mqtt_manager_subscribe(s_topics.cali_ph_set, 1,
                                           on_cali_ph_set, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to subscribe to %s: %s",
                 s_topics.cali_ph_set, esp_err_to_name(ret));
        return ret;
    }

    ret = mqtt_manager_subscribe(s_topics.cali_tds_set, 1,
                                 on_cali_tds_set, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to subscribe to %s: %s",
                 s_topics.cali_tds_set, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Subscribed to calibration set topics");
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * JSON calibration payload publisher
 * -------------------------------------------------------------------------- */

static void publish_cali_state(const char *topic, const calibration_t *c)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"mode\":\"linear\","
             "\"slope\":%.6f,"
             "\"offset\":%.6f,"
             "\"valid\":%s,"
             "\"updated_at\":%lu}",
             (double)c->slope,
             (double)c->offset,
             c->valid ? "true" : "false",
             (unsigned long)c->updated_at);

    if (mqtt_manager_publish(topic, buf, /*qos=*/1, /*retain=*/1) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish calibration state to %s", topic);
    }
}

/* --------------------------------------------------------------------------
 * JSON calibration payload parser
 *
 * Expected format (device receives only final coefficients from Node-RED):
 * {
 *   "mode":       "linear",   // currently only "linear" is accepted
 *   "slope":      -5.72,
 *   "offset":     21.34,
 *   "valid":      true,
 *   "updated_at": 1746950000
 * }
 *
 * Uses a hand-rolled parser to avoid pulling in cJSON / nlohmann for a small
 * fixed-schema payload — replace with cJSON if available in your build.
 * -------------------------------------------------------------------------- */

/**
 * @brief  Very small JSON field extractor.
 *
 * Finds the first occurrence of "key": and parses the value as a double.
 * Returns true on success.  Not suitable for nested or complex JSON.
 */
static bool json_get_float(const char *json, const char *key, float *out)
{
    /* Build search pattern: "key": */
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(json, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t') p++;
    char *end = NULL;
    double v = strtod(p, &end);
    if (end == p) return false;
    *out = (float)v;
    return true;
}

static bool json_get_uint32(const char *json, const char *key, uint32_t *out)
{
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(json, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t') p++;
    char *end = NULL;
    unsigned long v = strtoul(p, &end, 10);
    if (end == p) return false;
    *out = (uint32_t)v;
    return true;
}

static bool json_get_bool(const char *json, const char *key, bool *out)
{
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(json, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t') p++;
    if (strncmp(p, "true",  4) == 0) { *out = true;  return true; }
    if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
    return false;
}

/**
 * @brief  Parse a JSON calibration payload into a calibration_t.
 *
 * @return true if all required fields were found and the mode is "linear".
 */
static bool parse_calibration_json(const char *payload, int payload_len,
                                   calibration_t *out)
{
    /* Work on a NUL-terminated copy */
    char buf[256];
    int copy_len = (payload_len < (int)(sizeof(buf) - 1))
                   ? payload_len : (int)(sizeof(buf) - 1);
    memcpy(buf, payload, copy_len);
    buf[copy_len] = '\0';

    /* Require mode = "linear" */
    if (strstr(buf, "\"linear\"") == NULL) {
        ESP_LOGW(TAG, "Calibration: unsupported mode (only 'linear' accepted)");
        return false;
    }

    calibration_t c = {0};
    if (!json_get_float(buf,   "slope",      &c.slope))      return false;
    if (!json_get_float(buf,   "offset",     &c.offset))     return false;
    if (!json_get_bool(buf,    "valid",      &c.valid))      return false;
    if (!json_get_uint32(buf,  "updated_at", &c.updated_at)) return false;

    *out = c;
    return true;
}

/* --------------------------------------------------------------------------
 * MQTT calibration command callbacks
 * -------------------------------------------------------------------------- */

static void on_cali_ph_set(const char *topic, const char *payload,
                           int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    calibration_t new_cali = {0};
    if (!parse_calibration_json(payload, payload_len, &new_cali)) {
        ESP_LOGE(TAG, "pH calibration: failed to parse payload");
        return;
    }

    portENTER_CRITICAL(&s_cali_lock);
    s_ph_cali = new_cali;
    portEXIT_CRITICAL(&s_cali_lock);

    if (sensor_calibration_nvs_save_ph(&new_cali) != ESP_OK) {
        ESP_LOGW(TAG, "pH calibration: NVS save failed");
    }

    ESP_LOGI(TAG, "pH calibration updated: slope=%.4f offset=%.4f valid=%d",
             new_cali.slope, new_cali.offset, (int)new_cali.valid);

    publish_cali_state(s_topics.cali_ph_state, &new_cali);
}

static void on_cali_tds_set(const char *topic, const char *payload,
                            int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    calibration_t new_cali = {0};
    if (!parse_calibration_json(payload, payload_len, &new_cali)) {
        ESP_LOGE(TAG, "TDS calibration: failed to parse payload");
        return;
    }

    portENTER_CRITICAL(&s_cali_lock);
    s_tds_cali = new_cali;
    portEXIT_CRITICAL(&s_cali_lock);

    if (sensor_calibration_nvs_save_tds(&new_cali) != ESP_OK) {
        ESP_LOGW(TAG, "TDS calibration: NVS save failed");
    }

    ESP_LOGI(TAG, "TDS calibration updated: slope=%.4f offset=%.4f valid=%d",
             new_cali.slope, new_cali.offset, (int)new_cali.valid);

    publish_cali_state(s_topics.cali_tds_state, &new_cali);
}

/* --------------------------------------------------------------------------
 * Public API — lifecycle
 * -------------------------------------------------------------------------- */

esp_err_t sensor_telemetry_init(const char *zone_id)
{
    if (zone_id == NULL || zone_id[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ensure_sensor_interfaces();
    if (ret != ESP_OK) return ret;

    ret = build_sensor_topics(zone_id);
    if (ret != ESP_OK) return ret;

    /* Reset runtime state */
    portENTER_CRITICAL(&s_snapshot_lock);
    reset_state_locked();
    portEXIT_CRITICAL(&s_snapshot_lock);

    /* Load calibration from NVS — sensors remain invalid until calibration
     * is loaded or received via MQTT */
    portENTER_CRITICAL(&s_cali_lock);
    memset(&s_ph_cali,  0, sizeof(s_ph_cali));
    memset(&s_tds_cali, 0, sizeof(s_tds_cali));
    portEXIT_CRITICAL(&s_cali_lock);

    bool found = false;
    calibration_t loaded = {0};

    ret = sensor_calibration_nvs_load_ph(&loaded, &found);
    if (ret == ESP_OK && found) {
        portENTER_CRITICAL(&s_cali_lock);
        s_ph_cali = loaded;
        portEXIT_CRITICAL(&s_cali_lock);
        ESP_LOGI(TAG, "pH calibration loaded from NVS: slope=%.4f offset=%.4f",
                 loaded.slope, loaded.offset);
    } else if (ret == ESP_OK) {
        ESP_LOGW(TAG, "No pH calibration in NVS — sensor will be invalid until calibrated");
    } else {
        ESP_LOGE(TAG, "pH NVS load error: %s", esp_err_to_name(ret));
    }

    memset(&loaded, 0, sizeof(loaded));
    found = false;
    ret = sensor_calibration_nvs_load_tds(&loaded, &found);
    if (ret == ESP_OK && found) {
        portENTER_CRITICAL(&s_cali_lock);
        s_tds_cali = loaded;
        portEXIT_CRITICAL(&s_cali_lock);
        ESP_LOGI(TAG, "TDS calibration loaded from NVS: slope=%.4f offset=%.4f",
                 loaded.slope, loaded.offset);
    } else if (ret == ESP_OK) {
        ESP_LOGW(TAG, "No TDS calibration in NVS — sensor will be invalid until calibrated");
    } else {
        ESP_LOGE(TAG, "TDS NVS load error: %s", esp_err_to_name(ret));
    }

    /* Subscribe to inbound calibration command topics */
    ret = subscribe_calibration_topics();
    if (ret != ESP_OK) return ret;

    s_initialized = true;
    ESP_LOGI(TAG, "Sensor telemetry initialized for zone: %s", zone_id);
    return ESP_OK;
}

esp_err_t sensor_telemetry_deinit(void)
{
    if (s_topics.cali_ph_set[0]  != '\0') mqtt_manager_unsubscribe(s_topics.cali_ph_set);
    if (s_topics.cali_tds_set[0] != '\0') mqtt_manager_unsubscribe(s_topics.cali_tds_set);

    if (s_adc_cali != NULL) {
        adc_cali_delete_scheme_curve_fitting(s_adc_cali);
        s_adc_cali = NULL;
    }
    s_adc_cali_ready = false;

    if (s_adc_handle != NULL) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    reset_state_locked();
    portEXIT_CRITICAL(&s_snapshot_lock);

    portENTER_CRITICAL(&s_cali_lock);
    memset(&s_ph_cali,  0, sizeof(s_ph_cali));
    memset(&s_tds_cali, 0, sizeof(s_tds_cali));
    portEXIT_CRITICAL(&s_cali_lock);

    memset(&s_topics, 0, sizeof(s_topics));

    s_initialized = false;
    ESP_LOGI(TAG, "Sensor telemetry deinitialized");
    return ESP_OK;
}

esp_err_t sensor_telemetry_start(const char *zone_id)
{
    return sensor_telemetry_init(zone_id);
}

esp_err_t sensor_telemetry_stop(void)
{
    return sensor_telemetry_deinit();
}

/* --------------------------------------------------------------------------
 * Public API — sampling
 * -------------------------------------------------------------------------- */

esp_err_t sensor_telemetry_sample(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    /* ── Water level (digital) ─────────────────────────────────────────── */
    int water_level = gpio_get_level((gpio_num_t)PIN_SENSOR_WATER_LEVEL);

    /* ── Water temperature ─────────────────────────────────────────────── */
    float temp_c = 0.0f;
    (void)sample_water_temp(&temp_c);

    /* ── pH ────────────────────────────────────────────────────────────── */
    sensor_reading_t ph_reading = {0};

    /* Take a local copy of calibration to avoid holding the lock during I/O */
    portENTER_CRITICAL(&s_cali_lock);
    calibration_t ph_cali  = s_ph_cali;
    calibration_t tds_cali = s_tds_cali;
    portEXIT_CRITICAL(&s_cali_lock);

    {
        int ph_level = gpio_get_level((gpio_num_t)PIN_SENSOR_PH);
        apply_gpio_calibration_and_validate(&ph_cali, ph_level,
                                            PH_VALUE_MIN, PH_VALUE_MAX,
                                            &ph_reading);
    }

    /* ── TDS ───────────────────────────────────────────────────────────── */
    sensor_reading_t tds_reading = {0};

    {
        int tds_level = gpio_get_level((gpio_num_t)PIN_SENSOR_TDS);
        apply_gpio_calibration_and_validate(&tds_cali, tds_level,
                                            TDS_VALUE_MIN, TDS_VALUE_MAX,
                                            &tds_reading);
    }

    /* ── Update rolling history & snapshot ────────────────────────────── */
    portENTER_CRITICAL(&s_snapshot_lock);

    s_temp_history[s_history_index] = temp_c;
    s_ph_history[s_history_index]   = ph_reading.value;
    s_tds_history[s_history_index]  = tds_reading.value;

    if (s_history_count < SENSOR_SAMPLE_WINDOW) {
        s_history_count++;
    }
    s_history_index = (s_history_index + 1U) % SENSOR_SAMPLE_WINDOW;

    s_snapshot.valid       = true;
    s_snapshot.water_level = water_level ? 1 : 0;
    s_snapshot.water_temp  = average_window(s_temp_history, s_history_count);

    /* Raw counts for pH/TDS are the GPIO level (0 or 1); for water-temp the
     * averaged ADC value.  Node-RED sees the live value and any calibration
     * fault promptly regardless of the sensor interface type. */
    s_snapshot.ph_raw   = ph_reading.raw;   /* GPIO level 0 or 1 */
    s_snapshot.ph       = average_window(s_ph_history, s_history_count);
    s_snapshot.ph_valid = ph_reading.valid;

    s_snapshot.tds_raw   = tds_reading.raw;  /* GPIO level 0 or 1 */
    s_snapshot.tds       = average_window(s_tds_history, s_history_count);
    s_snapshot.tds_valid = tds_reading.valid;

    s_snapshot.publish_count++;
    portEXIT_CRITICAL(&s_snapshot_lock);

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Public API — topic accessors
 * -------------------------------------------------------------------------- */

const char *sensor_telemetry_topic_water_level(void) { return s_topics.water_level_state; }
const char *sensor_telemetry_topic_water_temp(void)  { return s_topics.water_temp_state;  }
const char *sensor_telemetry_topic_ph(void)          { return s_topics.ph_state;           }
const char *sensor_telemetry_topic_tds(void)         { return s_topics.tds_state;          }
const char *sensor_telemetry_topic_ph_raw(void)      { return s_topics.ph_raw;             }
const char *sensor_telemetry_topic_ph_valid(void)    { return s_topics.ph_valid;           }
const char *sensor_telemetry_topic_tds_raw(void)     { return s_topics.tds_raw;            }
const char *sensor_telemetry_topic_tds_valid(void)   { return s_topics.tds_valid;          }
const char *sensor_telemetry_topic_cali_ph_state(void)  { return s_topics.cali_ph_state;  }
const char *sensor_telemetry_topic_cali_tds_state(void) { return s_topics.cali_tds_state; }

/* --------------------------------------------------------------------------
 * Public API — snapshot query
 * -------------------------------------------------------------------------- */

esp_err_t sensor_telemetry_get_snapshot(sensor_telemetry_snapshot_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_snapshot_lock);
    *out = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Public API — calibration queries
 * -------------------------------------------------------------------------- */

bool sensor_telemetry_ph_calibrated(void)
{
    portENTER_CRITICAL(&s_cali_lock);
    bool v = s_ph_cali.valid;
    portEXIT_CRITICAL(&s_cali_lock);
    return v;
}

bool sensor_telemetry_tds_calibrated(void)
{
    portENTER_CRITICAL(&s_cali_lock);
    bool v = s_tds_cali.valid;
    portEXIT_CRITICAL(&s_cali_lock);
    return v;
}

void sensor_telemetry_get_ph_calibration(calibration_t *out)
{
    if (out == NULL) return;
    portENTER_CRITICAL(&s_cali_lock);
    *out = s_ph_cali;
    portEXIT_CRITICAL(&s_cali_lock);
}

void sensor_telemetry_get_tds_calibration(calibration_t *out)
{
    if (out == NULL) return;
    portENTER_CRITICAL(&s_cali_lock);
    *out = s_tds_cali;
    portEXIT_CRITICAL(&s_cali_lock);
}