#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "pin_config.h"
#include "sensor_telemetry.h"

#define SENSOR_SAMPLE_WINDOW 30
#define WATER_TEMP_ADC_ATTEN    ADC_ATTEN_DB_12
#define WATER_TEMP_ADC_BITWIDTH ADC_BITWIDTH_12

typedef struct {
    char topic_water_level[96];
    char topic_water_temp[96];
    char topic_ph[96];
    char topic_tds[96];
} sensor_topics_t;

static const char *TAG = "sensor_telem";

static sensor_topics_t s_topics;
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_cali_ready;
static bool s_initialized;
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static sensor_telemetry_snapshot_t s_snapshot;
static float s_temp_history[SENSOR_SAMPLE_WINDOW];
static float s_ph_history[SENSOR_SAMPLE_WINDOW];
static float s_tds_history[SENSOR_SAMPLE_WINDOW];
static size_t s_history_index;
static size_t s_history_count;

static float estimate_water_temp_celsius(int millivolts)
{
    /* Placeholder transfer: replace with sensor-specific formula when calibrated sensor model is finalized. */
    return ((float)millivolts) / 10.0f;
}

static float average_window(const float *values, size_t count)
{
    if (values == NULL || count == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
        sum += values[i];
    }
    return sum / (float)count;
}

static esp_err_t ensure_sensor_interfaces(void)
{
    gpio_config_t level_cfg = {
        .pin_bit_mask = (1ULL << PIN_SENSOR_WATER_LEVEL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&level_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    if (s_adc_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id = PIN_SENSOR_WATER_TEMP_ADC_UNIT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };

        ret = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
        if (ret != ESP_OK) {
            return ret;
        }

        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = WATER_TEMP_ADC_ATTEN,
            .bitwidth = WATER_TEMP_ADC_BITWIDTH,
        };

        ret = adc_oneshot_config_channel(s_adc_handle, PIN_SENSOR_WATER_TEMP_ADC_CHANNEL, &chan_cfg);
        if (ret != ESP_OK) {
            return ret;
        }

        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = PIN_SENSOR_WATER_TEMP_ADC_UNIT,
            .chan = PIN_SENSOR_WATER_TEMP_ADC_CHANNEL,
            .atten = WATER_TEMP_ADC_ATTEN,
            .bitwidth = WATER_TEMP_ADC_BITWIDTH,
        };

        ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali);
        if (ret == ESP_OK) {
            s_adc_cali_ready = true;
            ESP_LOGI(TAG, "ADC curve-fitting calibration enabled");
        } else {
            s_adc_cali_ready = false;
            ESP_LOGW(TAG, "ADC curve-fitting calibration unavailable, fallback enabled");
        }
    }

    return ESP_OK;
}

esp_err_t sensor_telemetry_init(const char *zone_id)
{
    if (zone_id == NULL || strlen(zone_id) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ensure_sensor_interfaces();
    if (ret != ESP_OK) {
        return ret;
    }

    int w0 = snprintf(s_topics.topic_water_level, sizeof(s_topics.topic_water_level), "%s/sensor/WaterLevel/state", zone_id);
    int w1 = snprintf(s_topics.topic_water_temp, sizeof(s_topics.topic_water_temp), "%s/sensor/WaterTemp/state", zone_id);
    int w2 = snprintf(s_topics.topic_ph, sizeof(s_topics.topic_ph), "%s/sensor/pH/state", zone_id);
    int w3 = snprintf(s_topics.topic_tds, sizeof(s_topics.topic_tds), "%s/sensor/TDS/state", zone_id);

    if (w0 <= 0 || w0 >= (int)sizeof(s_topics.topic_water_level) ||
        w1 <= 0 || w1 >= (int)sizeof(s_topics.topic_water_temp) ||
        w2 <= 0 || w2 >= (int)sizeof(s_topics.topic_ph) ||
        w3 <= 0 || w3 >= (int)sizeof(s_topics.topic_tds)) {
        return ESP_ERR_INVALID_SIZE;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(s_temp_history, 0, sizeof(s_temp_history));
    memset(s_ph_history, 0, sizeof(s_ph_history));
    memset(s_tds_history, 0, sizeof(s_tds_history));
    s_history_index = 0;
    s_history_count = 0;
    portEXIT_CRITICAL(&s_snapshot_lock);

    s_initialized = true;
    ESP_LOGI(TAG, "Sensor telemetry initialized for zone: %s", zone_id);
    return ESP_OK;
}

esp_err_t sensor_telemetry_deinit(void)
{
    portENTER_CRITICAL(&s_snapshot_lock);
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(s_temp_history, 0, sizeof(s_temp_history));
    memset(s_ph_history, 0, sizeof(s_ph_history));
    memset(s_tds_history, 0, sizeof(s_tds_history));
    s_history_index = 0;
    s_history_count = 0;
    portEXIT_CRITICAL(&s_snapshot_lock);

    s_initialized = false;
    ESP_LOGI(TAG, "Sensor telemetry deinitialized");
    return ESP_OK;
}

esp_err_t sensor_telemetry_sample(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int water_level = gpio_get_level((gpio_num_t)PIN_SENSOR_WATER_LEVEL);

    int raw = 0;
    int mv = 0;
    float temp_c = 0.0f;
    if (adc_oneshot_read(s_adc_handle, PIN_SENSOR_WATER_TEMP_ADC_CHANNEL, &raw) == ESP_OK) {
        if (s_adc_cali_ready) {
            if (adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) != ESP_OK) {
                mv = 0;
            }
        } else {
            mv = (raw * 3300) / 4095;
        }
        temp_c = estimate_water_temp_celsius(mv);
    }

    float ph = 7.0f;
    float tds = 500.0f;

    portENTER_CRITICAL(&s_snapshot_lock);
    s_temp_history[s_history_index] = temp_c;
    s_ph_history[s_history_index] = ph;
    s_tds_history[s_history_index] = tds;

    if (s_history_count < SENSOR_SAMPLE_WINDOW) {
        s_history_count++;
    }

    s_history_index = (s_history_index + 1U) % SENSOR_SAMPLE_WINDOW;

    s_snapshot.valid = true;
    s_snapshot.water_level = water_level ? 1 : 0;
    s_snapshot.water_temp = average_window(s_temp_history, s_history_count);
    s_snapshot.ph = average_window(s_ph_history, s_history_count);
    s_snapshot.tds = average_window(s_tds_history, s_history_count);
    s_snapshot.publish_count++;
    portEXIT_CRITICAL(&s_snapshot_lock);

    return ESP_OK;
}

const char *sensor_telemetry_topic_water_level(void)
{
    return s_topics.topic_water_level;
}

const char *sensor_telemetry_topic_water_temp(void)
{
    return s_topics.topic_water_temp;
}

const char *sensor_telemetry_topic_ph(void)
{
    return s_topics.topic_ph;
}

const char *sensor_telemetry_topic_tds(void)
{
    return s_topics.topic_tds;
}

esp_err_t sensor_telemetry_start(const char *zone_id)
{
    return sensor_telemetry_init(zone_id);
}

esp_err_t sensor_telemetry_stop(void)
{
    return sensor_telemetry_deinit();
}

esp_err_t sensor_telemetry_get_snapshot(sensor_telemetry_snapshot_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    *out = s_snapshot;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ESP_OK;
}
