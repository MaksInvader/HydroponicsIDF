/*
 * sensor_telemetry.c
 *
 * Data flow
 * ─────────
 *   Water-temp: DS18B20 1-Wire (trigger→read across cycles) ──► rolling average
 *     ──► snapshot ──► publish state
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
#include "driver/i2c.h"
#include "esp_log.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "freertos/FreeRTOS.h"

#include "esp_timer.h"
#include "mqtt_manager.h"
#include "pin_config.h"
#include "sensor_calibration_nvs.h"
#include "sensor_telemetry.h"

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */

/** Rolling window depth for the per-sensor moving average (all sensors). */
#define SENSOR_SAMPLE_WINDOW    30

/** DS18B20 12-bit conversion time in milliseconds. */
#define DS18B20_CONVERSION_MS   750

/** Valid physical ranges — readings outside these mark the reading invalid. */
#define PH_VALUE_MIN            0.0f
#define PH_VALUE_MAX            14.0f
#define TDS_VALUE_MIN           0.0f
#define TDS_VALUE_MAX           5000.0f  /* ppm; adjust for your application */

/* --------------------------------------------------------------------------
 * Calibration plausible ranges
 * -------------------------------------------------------------------------- */
#define PH_SLOPE_MIN    (-20.0f)
#define PH_SLOPE_MAX    ( 20.0f)
#define PH_OFFSET_MIN   (-50.0f)
#define PH_OFFSET_MAX   ( 50.0f)

#define TDS_SLOPE_MIN   (-100.0f)
#define TDS_SLOPE_MAX   ( 100.0f)
#define TDS_OFFSET_MIN  (-500.0f)
#define TDS_OFFSET_MAX  ( 500.0f)

/* --------------------------------------------------------------------------
 * Calibration retry / timing constants
 * -------------------------------------------------------------------------- */
#define CALI_SUB_RETRY_COUNT     3
#define CALI_SUB_RETRY_DELAY_MS  2000
#define CALI_PUB_RETRY_COUNT     3
#define CALI_PUB_RETRY_DELAY_MS  1000
#define CALI_NVS_RETRY_COUNT     2   /* up to 2 additional attempts = 3 total */

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
    uint16_t raw;   /* raw sensor value (GPIO level for pH/TDS)          */
    float    value; /* calibrated physical value (pH / ppm)              */
    bool     valid; /* true only when calibration + range checks pass    */
} sensor_reading_t;

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */

static const char *TAG = "sensor_telem";

static sensor_topics_t            s_topics;
static onewire_bus_handle_t       s_ow_bus    = NULL;
static ds18b20_device_handle_t    s_ds18b20   = NULL;
static bool                       s_ow_ready  = false;
/* true after the first conversion trigger; read is deferred to next cycle */
static bool                       s_ow_conversion_pending = false;
static bool                       s_initialized;

/* ADS1115 I2C handle state */
static bool s_ads1115_ready = false;

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

/* Deferred publish flags — set when MQTT was not connected at boot */
static bool s_ph_pub_pending;
static bool s_tds_pub_pending;

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void on_cali_ph_set(const char *topic, const char *payload,
                           int payload_len, void *user_ctx);
static void on_cali_tds_set(const char *topic, const char *payload,
                            int payload_len, void *user_ctx);
static void on_mqtt_connected(bool connected, void *user_ctx);

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

/* --------------------------------------------------------------------------
 * DS18B20 1-Wire helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief  Initialise the 1-Wire bus and discover the DS18B20.
 *
 * Expects a single DS18B20 on PIN_SENSOR_WATER_TEMP_ONEWIRE.
 * Sets s_ow_ready = true on success.
 */
static void ow_init(void)
{
    onewire_bus_config_t bus_cfg = {
        .bus_gpio_num = PIN_SENSOR_WATER_TEMP_ONEWIRE,
    };
    onewire_bus_rmt_config_t rmt_cfg = {
        .max_rx_bytes = 10,
    };

    esp_err_t ret = onewire_new_bus_rmt(&bus_cfg, &rmt_cfg, &s_ow_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "1-Wire bus init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Enumerate — expect exactly one device */
    onewire_device_iter_handle_t iter = NULL;
    ret = onewire_new_device_iter(s_ow_bus, &iter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "1-Wire iter create failed: %s", esp_err_to_name(ret));
        return;
    }

    onewire_device_t dev;
    esp_err_t search_ret = onewire_device_iter_get_next(iter, &dev);
    onewire_del_device_iter(iter);

    if (search_ret != ESP_OK) {
        ESP_LOGE(TAG, "No DS18B20 found on 1-Wire bus");
        return;
    }

    ds18b20_config_t ds_cfg = {};
    ret = ds18b20_new_device_from_enumeration(&dev, &ds_cfg, &s_ds18b20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS18B20 device create failed: %s", esp_err_to_name(ret));
        return;
    }

    s_ow_ready = true;
    ESP_LOGI(TAG, "DS18B20 ready on GPIO%d", PIN_SENSOR_WATER_TEMP_ONEWIRE);
}

/**
 * @brief  Trigger a DS18B20 temperature conversion (non-blocking).
 *
 * Call at the END of a sampling cycle.  The result will be ready after
 * DS18B20_CONVERSION_MS (750 ms for 12-bit), well within the next
 * SENSOR_SAMPLE_INTERVAL_MS (1000 ms) cycle.
 */
static void ow_trigger_conversion(void)
{
    if (!s_ow_ready || s_ds18b20 == NULL) return;
    esp_err_t ret = ds18b20_trigger_temperature_conversion(s_ds18b20);
    if (ret == ESP_OK) {
        s_ow_conversion_pending = true;
    } else {
        ESP_LOGW(TAG, "DS18B20 conversion trigger failed: %s", esp_err_to_name(ret));
        s_ow_conversion_pending = false;
    }
}

/**
 * @brief  Read the result of the previously triggered conversion.
 *
 * Call at the START of a sampling cycle (after the previous cycle triggered).
 * Returns ESP_ERR_INVALID_STATE if no conversion was pending.
 */
static esp_err_t sample_water_temp(float *temp_c)
{
    if (temp_c == NULL) return ESP_ERR_INVALID_ARG;
    *temp_c = 0.0f;

    if (!s_ow_ready || s_ds18b20 == NULL) return ESP_ERR_INVALID_STATE;
    if (!s_ow_conversion_pending)         return ESP_ERR_INVALID_STATE;

    s_ow_conversion_pending = false;
    return ds18b20_get_temperature(s_ds18b20, temp_c);
}

/* --------------------------------------------------------------------------
 * ADS1115 I2C driver (minimal, single-shot)
 * -------------------------------------------------------------------------- */

/* ADS1115 register addresses */
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

/* Config register bits */
#define ADS1115_OS_SINGLE       (1u << 15)  /* start single conversion      */
#define ADS1115_MUX_AIN0_GND   (4u << 12)  /* AIN0 vs GND                  */
#define ADS1115_MUX_AIN1_GND   (5u << 12)  /* AIN1 vs GND                  */
#define ADS1115_PGA_4096        (1u << 9)   /* ±4.096 V FSR                 */
#define ADS1115_MODE_SINGLE     (1u << 8)   /* single-shot mode             */
#define ADS1115_DR_128SPS       (4u << 5)   /* 128 samples/s                */
#define ADS1115_COMP_DISABLE    (3u << 0)   /* disable comparator           */

#define ADS1115_CONVERSION_MS   10          /* ~8 ms at 128 SPS + margin    */

/**
 * @brief  Initialise the I2C master for the ADS1115.
 *
 * Shares I2C_NUM_0 with the LCD.  If the driver is already installed
 * (ESP_ERR_INVALID_STATE) that is treated as success.
 */
static esp_err_t ads1115_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_ADS1115_I2C_SDA,
        .scl_io_num       = PIN_ADS1115_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags        = 0,
    };
    esp_err_t ret = i2c_param_config(PIN_ADS1115_I2C_PORT, &conf);
    if (ret != ESP_OK) return ret;

    ret = i2c_driver_install(PIN_ADS1115_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE || ret == ESP_FAIL) {
        /* Legacy i2c driver returns ESP_FAIL (not ESP_ERR_INVALID_STATE)
         * when the driver is already installed on this port. Treat both as
         * "already installed" and continue — the driver is usable. */
        ret = ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Probe: write config register with a benign value */
    uint16_t cfg = ADS1115_MUX_AIN0_GND | ADS1115_PGA_4096 |
                   ADS1115_MODE_SINGLE   | ADS1115_DR_128SPS |
                   ADS1115_COMP_DISABLE;
    uint8_t buf[3] = {
        ADS1115_REG_CONFIG,
        (uint8_t)(cfg >> 8),
        (uint8_t)(cfg & 0xFF),
    };
    ret = i2c_master_write_to_device(PIN_ADS1115_I2C_PORT, PIN_ADS1115_I2C_ADDR,
                                     buf, sizeof(buf), pdMS_TO_TICKS(50));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 not found at 0x%02X: %s",
                 PIN_ADS1115_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }

    s_ads1115_ready = true;
    ESP_LOGI(TAG, "ADS1115 ready at I2C addr 0x%02X", PIN_ADS1115_I2C_ADDR);
    return ESP_OK;
}

/**
 * @brief  Perform a single-shot conversion on the given mux setting.
 *
 * @param  mux_bits  ADS1115_MUX_AIN0_GND or ADS1115_MUX_AIN1_GND
 * @param  out_raw   Raw 16-bit signed result (0–32767 for single-ended).
 * @return ESP_OK on success.
 */
static esp_err_t ads1115_read_channel(uint16_t mux_bits, int *out_raw)
{
    if (!s_ads1115_ready || out_raw == NULL) return ESP_ERR_INVALID_STATE;

    /* Write config: start single-shot conversion on requested channel */
    uint16_t cfg = ADS1115_OS_SINGLE | mux_bits | ADS1115_PGA_4096 |
                   ADS1115_MODE_SINGLE | ADS1115_DR_128SPS |
                   ADS1115_COMP_DISABLE;
    uint8_t wr[3] = {
        ADS1115_REG_CONFIG,
        (uint8_t)(cfg >> 8),
        (uint8_t)(cfg & 0xFF),
    };
    esp_err_t ret = i2c_master_write_to_device(PIN_ADS1115_I2C_PORT,
                                               PIN_ADS1115_I2C_ADDR,
                                               wr, sizeof(wr),
                                               pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return ret;

    /* Wait for conversion to complete */
    vTaskDelay(pdMS_TO_TICKS(ADS1115_CONVERSION_MS));

    /* Point to conversion register */
    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t rd[2] = {0};
    ret = i2c_master_write_read_device(PIN_ADS1115_I2C_PORT,
                                       PIN_ADS1115_I2C_ADDR,
                                       &reg, 1, rd, 2,
                                       pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return ret;

    int16_t raw = (int16_t)((rd[0] << 8) | rd[1]);
    /* Single-ended: clamp negatives to 0 */
    *out_raw = (raw < 0) ? 0 : (int)raw;
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Calibration application
 * -------------------------------------------------------------------------- */

/**
 * @brief  Apply a linear calibration to an ADC raw count and validate.
 *
 * The linear model maps the raw ADC count to a physical value via the stored
 * slope/offset coefficients: value = slope * raw + offset.
 *
 * @param  cali      Calibration coefficients (may be invalid).
 * @param  adc_raw   Raw ADC reading (0–4095 for 12-bit).
 * @param  val_min   Minimum physically sane value.
 * @param  val_max   Maximum physically sane value.
 * @param  out       Populated reading struct.
 */
static void apply_adc_calibration_and_validate(const calibration_t *cali,
                                               int adc_raw,
                                               float val_min, float val_max,
                                               sensor_reading_t *out)
{
    out->raw   = (uint16_t)adc_raw;
    out->value = 0.0f;
    out->valid = false;

    /* Gate 1: calibration must exist */
    if (!cali->valid) return;

    /* Apply linear model: value = slope * adc_raw + offset */
    float computed = cali->slope * (float)adc_raw + cali->offset;

    /* Gate 2: result must be finite and within physical bounds */
    if (!isfinite(computed)) return;
    if (computed < val_min)  return;
    if (computed > val_max)  return;

    out->value = computed;
    out->valid = true;
}

/* --------------------------------------------------------------------------
 * Water temperature — stale ADC stub removed; DS18B20 implementation above.
 * -------------------------------------------------------------------------- */

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

    /* pH and TDS — ADS1115 16-bit ADC over I2C */
    ret = ads1115_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* DS18B20 1-Wire (water temperature) */
    ow_init();

    /* Trigger the first conversion immediately so the very first
     * sample_water_temp() call one cycle later has a result ready. */
    ow_trigger_conversion();

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * MQTT calibration subscription
 * -------------------------------------------------------------------------- */

static esp_err_t subscribe_one_with_retry(const char *topic,
                                          mqtt_manager_message_cb_t cb,
                                          const char *err_state_topic);

static esp_err_t subscribe_calibration_topics(void)
{
    esp_err_t ret = subscribe_one_with_retry(s_topics.cali_ph_set,
                                             on_cali_ph_set,
                                             s_topics.cali_ph_state);
    if (ret != ESP_OK) return ret;

    return subscribe_one_with_retry(s_topics.cali_tds_set,
                                    on_cali_tds_set,
                                    s_topics.cali_tds_state);
}

/**
 * Single-attempt (no vTaskDelay) subscribe — safe to call from the MQTT
 * event task where blocking is forbidden.
 */
static void subscribe_calibration_topics_nowait(void)
{
    esp_err_t ret = mqtt_manager_subscribe(s_topics.cali_ph_set, 1,
                                           on_cali_ph_set, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "pH cali subscribe failed on connect: %s — will retry next reconnect",
                 esp_err_to_name(ret));
    }
    ret = mqtt_manager_subscribe(s_topics.cali_tds_set, 1,
                                 on_cali_tds_set, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TDS cali subscribe failed on connect: %s — will retry next reconnect",
                 esp_err_to_name(ret));
    }
}

/* --------------------------------------------------------------------------
 * JSON calibration payload publisher
 * -------------------------------------------------------------------------- */

/** Publish with retain, retrying up to CALI_PUB_RETRY_COUNT times. */
static esp_err_t publish_retained_with_retry(const char *topic, const char *payload)
{
    for (int attempt = 0; attempt <= CALI_PUB_RETRY_COUNT; attempt++) {
        if (mqtt_manager_publish(topic, payload, /*qos=*/1, /*retain=*/1) == ESP_OK) {
            return ESP_OK;
        }
        if (attempt < CALI_PUB_RETRY_COUNT) {
            ESP_LOGW(TAG, "Publish to %s failed, retry %d/%d",
                     topic, attempt + 1, CALI_PUB_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(CALI_PUB_RETRY_DELAY_MS));
        }
    }
    ESP_LOGE(TAG, "Publish to %s failed after %d attempts", topic, CALI_PUB_RETRY_COUNT + 1);
    return ESP_FAIL;
}

/**
 * Publish a valid calibration state (4 dp, field order per Req 10).
 * Also publishes valid=true to the sensor/<type>/valid topic.
 */
static void publish_cali_state(const char *state_topic,
                               const char *valid_topic,
                               const calibration_t *c)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"mode\":\"linear\","
             "\"slope\":%.4f,"
             "\"offset\":%.4f,"
             "\"valid\":true,"
             "\"updated_at\":%lu}",
             (double)c->slope,
             (double)c->offset,
             (unsigned long)c->updated_at);

    publish_retained_with_retry(state_topic, buf);
    publish_retained_with_retry(valid_topic, "true");
}

/**
 * Publish an error calibration state (valid=false, reason field, Req 10.3).
 * updated_at is taken from the current in-memory calibration (0 if never set).
 */
static void publish_cali_error(const char *state_topic,
                               const char *valid_topic,
                               uint32_t last_updated_at,
                               const char *reason)
{
    char buf[256];
    /* Truncate reason to 128 chars as required */
    char safe_reason[129];
    strncpy(safe_reason, reason, 128);
    safe_reason[128] = '\0';

    snprintf(buf, sizeof(buf),
             "{\"mode\":\"linear\","
             "\"slope\":0.0000,"
             "\"offset\":0.0000,"
             "\"valid\":false,"
             "\"updated_at\":%lu,"
             "\"reason\":\"%s\"}",
             (unsigned long)last_updated_at,
             safe_reason);

    publish_retained_with_retry(state_topic, buf);
    publish_retained_with_retry(valid_topic, "false");
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

/* --------------------------------------------------------------------------
 * Calibration plausible-range descriptor
 * -------------------------------------------------------------------------- */

typedef struct {
    float slope_min, slope_max;
    float offset_min, offset_max;
} cali_plausible_range_t;

static const cali_plausible_range_t k_ph_range = {
    .slope_min  = PH_SLOPE_MIN,  .slope_max  = PH_SLOPE_MAX,
    .offset_min = PH_OFFSET_MIN, .offset_max = PH_OFFSET_MAX,
};

static const cali_plausible_range_t k_tds_range = {
    .slope_min  = TDS_SLOPE_MIN,  .slope_max  = TDS_SLOPE_MAX,
    .offset_min = TDS_OFFSET_MIN, .offset_max = TDS_OFFSET_MAX,
};

/**
 * @brief  Validate a raw MQTT calibration payload against all Req-4 rules.
 *
 * Stops at the first failing check and writes a human-readable reason into
 * @p reason_out (max 128 chars including NUL).
 *
 * @return true if all checks pass; false with reason_out populated otherwise.
 */
static bool validate_cali_payload(const char *payload, int payload_len,
                                  const cali_plausible_range_t *range,
                                  calibration_t *out,
                                  char *reason_out)
{
    /* Work on a NUL-terminated copy */
    char buf[256];
    int copy_len = (payload_len < (int)(sizeof(buf) - 1))
                   ? payload_len : (int)(sizeof(buf) - 1);
    memcpy(buf, payload, copy_len);
    buf[copy_len] = '\0';

    /* Req 4.1 — must be parseable JSON object (heuristic: starts with '{') */
    const char *p = buf;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != '{') {
        snprintf(reason_out, 128, "payload is not a JSON object");
        return false;
    }

    /* Req 4.4 — mode must be "linear" */
    if (strstr(buf, "\"mode\"") == NULL) {
        snprintf(reason_out, 128, "missing field: mode");
        return false;
    }
    if (strstr(buf, "\"linear\"") == NULL) {
        snprintf(reason_out, 128, "unsupported mode (only linear accepted)");
        return false;
    }

    /* Req 4.2 — slope must be present */
    float slope = 0.0f;
    if (!json_get_float(buf, "slope", &slope)) {
        snprintf(reason_out, 128, "missing field: slope");
        return false;
    }

    /* Req 4.2 — offset must be present */
    float offset = 0.0f;
    if (!json_get_float(buf, "offset", &offset)) {
        snprintf(reason_out, 128, "missing field: offset");
        return false;
    }

    /* Req 4.3 — slope and offset must be finite */
    if (!isfinite(slope)) {
        snprintf(reason_out, 128, "slope is not finite");
        return false;
    }
    if (!isfinite(offset)) {
        snprintf(reason_out, 128, "offset is not finite");
        return false;
    }

    /* Req 4.3 — slope must be non-zero */
    if (slope == 0.0f) {
        snprintf(reason_out, 128, "slope must be non-zero");
        return false;
    }

    /* Req 4.5/4.6 — plausible range check */
    if (slope < range->slope_min || slope > range->slope_max) {
        snprintf(reason_out, 128, "slope %.4f out of range [%.4f, %.4f]",
                 (double)slope, (double)range->slope_min, (double)range->slope_max);
        return false;
    }
    if (offset < range->offset_min || offset > range->offset_max) {
        snprintf(reason_out, 128, "offset %.4f out of range [%.4f, %.4f]",
                 (double)offset, (double)range->offset_min, (double)range->offset_max);
        return false;
    }

    out->slope      = slope;
    out->offset     = offset;
    out->valid      = true;
    out->updated_at = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    return true;
}

/**
 * @brief  Orchestrate validate → NVS write (with retry) → readback → in-memory update.
 *
 * @param  payload        Raw MQTT payload bytes.
 * @param  payload_len    Length of payload.
 * @param  range          Plausible-range descriptor for this sensor type.
 * @param  nvs_save       Function pointer to sensor_calibration_nvs_save_ph/tds.
 * @param  nvs_verify     Function pointer to sensor_calibration_nvs_verify_ph/tds.
 * @param  cali_mem       Pointer to the in-memory calibration_t to update on success.
 * @param  state_topic    Topic for publishing calibration state / errors.
 * @param  valid_topic    Topic for publishing sensor valid flag.
 * @param  type_name      Human-readable type name for log messages.
 */
static void handle_cali_set(const char *payload, int payload_len,
                             const cali_plausible_range_t *range,
                             esp_err_t (*nvs_save)(const calibration_t *),
                             esp_err_t (*nvs_verify)(float, float),
                             calibration_t *cali_mem,
                             const char *state_topic,
                             const char *valid_topic,
                             const char *type_name)
{
    /* Snapshot current updated_at for error messages before any change */
    portENTER_CRITICAL(&s_cali_lock);
    uint32_t prev_updated_at = cali_mem->updated_at;
    portEXIT_CRITICAL(&s_cali_lock);

    /* --- Req 4: Validate --- */
    char reason[128] = {0};
    calibration_t new_cali = {0};
    if (!validate_cali_payload(payload, payload_len, range, &new_cali, reason)) {
        ESP_LOGW(TAG, "%s calibration rejected: %s", type_name, reason);
        publish_cali_error(state_topic, valid_topic, prev_updated_at, reason);
        return;
    }

    /* --- Req 5: NVS write with retry --- */
    esp_err_t save_ret = ESP_FAIL;
    for (int attempt = 0; attempt <= CALI_NVS_RETRY_COUNT; attempt++) {
        save_ret = nvs_save(&new_cali);
        if (save_ret == ESP_OK) break;
        ESP_LOGW(TAG, "%s NVS save failed (attempt %d/%d): %s",
                 type_name, attempt + 1, CALI_NVS_RETRY_COUNT + 1,
                 esp_err_to_name(save_ret));
    }

    if (save_ret != ESP_OK) {
        snprintf(reason, sizeof(reason), "NVS write failed: %s",
                 esp_err_to_name(save_ret));
        publish_cali_error(state_topic, valid_topic, prev_updated_at, reason);
        /* Req 5.3: keep old in-memory calibration unchanged */
        return;
    }

    /* --- Req 8.4: Read-back verification --- */
    esp_err_t verify_ret = nvs_verify(new_cali.slope, new_cali.offset);
    if (verify_ret != ESP_OK) {
        /* Req 8.1/8.2: treat as corruption — invalidate in-memory */
        portENTER_CRITICAL(&s_cali_lock);
        cali_mem->valid = false;
        portEXIT_CRITICAL(&s_cali_lock);
        snprintf(reason, sizeof(reason), "NVS read-back mismatch after write");
        publish_cali_error(state_topic, valid_topic, prev_updated_at, reason);
        return;
    }

    /* --- Success: update in-memory AFTER confirmed NVS write --- */
    portENTER_CRITICAL(&s_cali_lock);
    *cali_mem = new_cali;
    portEXIT_CRITICAL(&s_cali_lock);

    ESP_LOGI(TAG, "%s calibration updated: slope=%.4f offset=%.4f updated_at=%lu",
             type_name, (double)new_cali.slope, (double)new_cali.offset,
             (unsigned long)new_cali.updated_at);

    /* --- Req 6: Publish state + valid flag --- */
    publish_cali_state(state_topic, valid_topic, &new_cali);
}

/* --------------------------------------------------------------------------
 * MQTT calibration command callbacks
 * -------------------------------------------------------------------------- */

static void on_cali_ph_set(const char *topic, const char *payload,
                           int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;
    handle_cali_set(payload, payload_len,
                    &k_ph_range,
                    sensor_calibration_nvs_save_ph,
                    sensor_calibration_nvs_verify_ph,
                    &s_ph_cali,
                    s_topics.cali_ph_state,
                    s_topics.ph_valid,
                    "pH");
}

static void on_cali_tds_set(const char *topic, const char *payload,
                            int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;
    handle_cali_set(payload, payload_len,
                    &k_tds_range,
                    sensor_calibration_nvs_save_tds,
                    sensor_calibration_nvs_verify_tds,
                    &s_tds_cali,
                    s_topics.cali_tds_state,
                    s_topics.tds_valid,
                    "TDS");
}

/* --------------------------------------------------------------------------
 * Subscription helper with retry (Req 3.4/3.5)
 * -------------------------------------------------------------------------- */

static esp_err_t subscribe_one_with_retry(const char *topic,
                                          mqtt_manager_message_cb_t cb,
                                          const char *err_state_topic)
{
    for (int attempt = 0; attempt <= CALI_SUB_RETRY_COUNT; attempt++) {
        esp_err_t ret = mqtt_manager_subscribe(topic, 1, cb, NULL);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Subscribed to %s", topic);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Subscribe to %s failed (attempt %d/%d): %s",
                 topic, attempt + 1, CALI_SUB_RETRY_COUNT + 1,
                 esp_err_to_name(ret));
        if (attempt < CALI_SUB_RETRY_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(CALI_SUB_RETRY_DELAY_MS));
        }
    }

    /* Req 3.5: publish error after all retries exhausted */
    char reason[128];
    snprintf(reason, sizeof(reason), "subscription failed for %s", topic);
    /* Determine which valid topic to use based on which state topic this is */
    bool is_ph = (strcmp(err_state_topic, s_topics.cali_ph_state) == 0);
    const char *valid_topic = is_ph ? s_topics.ph_valid : s_topics.tds_valid;
    portENTER_CRITICAL(&s_cali_lock);
    uint32_t upd = is_ph ? s_ph_cali.updated_at : s_tds_cali.updated_at;
    portEXIT_CRITICAL(&s_cali_lock);
    publish_cali_error(err_state_topic, valid_topic, upd, reason);
    return ESP_FAIL;
}

/* --------------------------------------------------------------------------
 * MQTT reconnect callback (Req 3.3 re-subscribe + Req 1.8 deferred publish)
 * -------------------------------------------------------------------------- */

static void on_mqtt_connected(bool connected, void *user_ctx)
{
    (void)user_ctx;
    if (!connected) return;

    /* Re-subscribe using the no-wait variant — this callback runs on the
     * MQTT event task and must never call vTaskDelay. A single attempt is
     * sufficient because the broker just confirmed the connection. */
    subscribe_calibration_topics_nowait();

    /* Deferred publish for boot-time calibration state (Req 1.8) */
    portENTER_CRITICAL(&s_cali_lock);
    bool ph_pending  = s_ph_pub_pending;
    bool tds_pending = s_tds_pub_pending;
    calibration_t ph_snap  = s_ph_cali;
    calibration_t tds_snap = s_tds_cali;
    s_ph_pub_pending  = false;
    s_tds_pub_pending = false;
    portEXIT_CRITICAL(&s_cali_lock);

    if (ph_pending) {
        if (ph_snap.valid) {
            publish_cali_state(s_topics.cali_ph_state, s_topics.ph_valid, &ph_snap);
        } else {
            publish_retained_with_retry(s_topics.ph_valid, "false");
        }
    }
    if (tds_pending) {
        if (tds_snap.valid) {
            publish_cali_state(s_topics.cali_tds_state, s_topics.tds_valid, &tds_snap);
        } else {
            publish_retained_with_retry(s_topics.tds_valid, "false");
        }
    }
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
    memset(&s_ph_cali,   0, sizeof(s_ph_cali));
    memset(&s_tds_cali,  0, sizeof(s_tds_cali));
    s_ph_pub_pending  = false;
    s_tds_pub_pending = false;
    portEXIT_CRITICAL(&s_cali_lock);

    /* Helper: load one sensor's calibration with plausible-range check */
    struct {
        esp_err_t (*load)(calibration_t *, bool *);
        calibration_t        *mem;
        bool                 *pub_pending;
        const char           *state_topic;
        const char           *valid_topic;
        const cali_plausible_range_t *range;
        const char           *name;
    } sensors[] = {
        { sensor_calibration_nvs_load_ph,  &s_ph_cali,  &s_ph_pub_pending,
          s_topics.cali_ph_state,  s_topics.ph_valid,  &k_ph_range,  "pH"  },
        { sensor_calibration_nvs_load_tds, &s_tds_cali, &s_tds_pub_pending,
          s_topics.cali_tds_state, s_topics.tds_valid, &k_tds_range, "TDS" },
    };

    /* Register MQTT reconnect callback before checking connection state so a
     * connect event that fires during the NVS load loop is not missed (Req 1.8).
     * Uses add_connection_cb so it does not overwrite runtime_tasks' callback. */
    mqtt_manager_add_connection_cb(on_mqtt_connected, NULL);

    bool mqtt_up = mqtt_manager_is_connected();

    for (int i = 0; i < 2; i++) {
        bool found = false;
        calibration_t loaded = {0};
        esp_err_t load_ret = sensors[i].load(&loaded, &found);

        if (load_ret != ESP_OK) {
            /* Req 1.6: NVS read error — mark invalid */
            ESP_LOGE(TAG, "%s NVS load error: %s",
                     sensors[i].name, esp_err_to_name(load_ret));
            if (mqtt_up) {
                publish_retained_with_retry(sensors[i].valid_topic, "false");
            } else {
                *sensors[i].pub_pending = true;
            }
            continue;
        }

        if (!found) {
            /* Req 1.4: no entry in NVS */
            ESP_LOGW(TAG, "No %s calibration in NVS — sensor invalid until calibrated",
                     sensors[i].name);
            if (mqtt_up) {
                publish_retained_with_retry(sensors[i].valid_topic, "false");
            } else {
                *sensors[i].pub_pending = true;
            }
            continue;
        }

        /* Req 1.7: plausible-range check on loaded data */
        const cali_plausible_range_t *r = sensors[i].range;
        bool in_range = (loaded.slope  >= r->slope_min  && loaded.slope  <= r->slope_max &&
                         loaded.offset >= r->offset_min && loaded.offset <= r->offset_max);

        if (!loaded.valid || !in_range) {
            ESP_LOGW(TAG, "%s NVS calibration out of plausible range or invalid flag — discarding",
                     sensors[i].name);
            char reason[128];
            if (!loaded.valid) {
                snprintf(reason, sizeof(reason), "NVS valid flag is false");
            } else {
                snprintf(reason, sizeof(reason),
                         "coefficients out of plausible range: slope=%.4f offset=%.4f",
                         (double)loaded.slope, (double)loaded.offset);
            }
            if (mqtt_up) {
                publish_cali_error(sensors[i].state_topic, sensors[i].valid_topic,
                                   loaded.updated_at, reason);
            } else {
                *sensors[i].pub_pending = true;
                /* Store the invalid-flagged copy so deferred publish has updated_at */
                loaded.valid = false;
                portENTER_CRITICAL(&s_cali_lock);
                *sensors[i].mem = loaded;
                portEXIT_CRITICAL(&s_cali_lock);
            }
            continue;
        }

        /* Req 1.2: valid calibration — apply to pipeline */
        portENTER_CRITICAL(&s_cali_lock);
        *sensors[i].mem = loaded;
        portEXIT_CRITICAL(&s_cali_lock);
        ESP_LOGI(TAG, "%s calibration loaded from NVS: slope=%.4f offset=%.4f",
                 sensors[i].name, (double)loaded.slope, (double)loaded.offset);

        /* Req 1.3: publish state */
        if (mqtt_up) {
            publish_cali_state(sensors[i].state_topic, sensors[i].valid_topic, &loaded);
        } else {
            *sensors[i].pub_pending = true;
        }
    }

    /* Subscribe to inbound calibration command topics.
     * If MQTT is not connected yet this will fail — that is non-fatal because
     * on_mqtt_connected() calls subscribe_calibration_topics() on every
     * connect event, so the subscription will be established then. */
    if (mqtt_manager_is_connected()) {
        ret = subscribe_calibration_topics();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Calibration subscribe failed at init (%s) — will retry on reconnect",
                     esp_err_to_name(ret));
        }
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Sensor telemetry initialized for zone: %s", zone_id);
    return ESP_OK;
}

esp_err_t sensor_telemetry_deinit(void)
{
    mqtt_manager_remove_connection_cb(on_mqtt_connected);
    if (s_topics.cali_ph_set[0]  != '\0') mqtt_manager_unsubscribe(s_topics.cali_ph_set);
    if (s_topics.cali_tds_set[0] != '\0') mqtt_manager_unsubscribe(s_topics.cali_tds_set);

    s_ads1115_ready = false;

    if (s_ds18b20 != NULL) {
        ds18b20_del_device(s_ds18b20);
        s_ds18b20 = NULL;
    }

    if (s_ow_bus != NULL) {
        onewire_bus_del(s_ow_bus);
        s_ow_bus = NULL;
    }

    s_ow_ready              = false;
    s_ow_conversion_pending = false;

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

    /* ── Water temperature (DS18B20 1-Wire, trigger-then-read pattern) ── */
    /* Read the result of the conversion triggered at the end of the         */
    /* previous cycle, then immediately trigger the next conversion so it    */
    /* is ready by the time this function is called again (~1000 ms later).  */
    float temp_c = 0.0f;
    (void)sample_water_temp(&temp_c);
    ow_trigger_conversion();

    /* ── pH ────────────────────────────────────────────────────────────── */
    sensor_reading_t ph_reading = {0};

    /* Take a local copy of calibration to avoid holding the lock during I/O */
    portENTER_CRITICAL(&s_cali_lock);
    calibration_t ph_cali  = s_ph_cali;
    calibration_t tds_cali = s_tds_cali;
    portEXIT_CRITICAL(&s_cali_lock);

    {
        int ph_raw = 0;
        esp_err_t adc_ret = ads1115_read_channel(ADS1115_MUX_AIN0_GND, &ph_raw);
        if (adc_ret != ESP_OK) {
            ESP_LOGW(TAG, "pH ADS1115 read failed: %s", esp_err_to_name(adc_ret));
            ph_raw = 0;
        }
        apply_adc_calibration_and_validate(&ph_cali, ph_raw,
                                           PH_VALUE_MIN, PH_VALUE_MAX,
                                           &ph_reading);
    }

    /* ── TDS ───────────────────────────────────────────────────────────── */
    sensor_reading_t tds_reading = {0};

    {
        int tds_raw = 0;
        esp_err_t adc_ret = ads1115_read_channel(ADS1115_MUX_AIN1_GND, &tds_raw);
        if (adc_ret != ESP_OK) {
            ESP_LOGW(TAG, "TDS ADS1115 read failed: %s", esp_err_to_name(adc_ret));
            tds_raw = 0;
        }
        apply_adc_calibration_and_validate(&tds_cali, tds_raw,
                                           TDS_VALUE_MIN, TDS_VALUE_MAX,
                                           &tds_reading);
    }

    /* ── Update rolling history & snapshot ────────────────────────────── */
    portENTER_CRITICAL(&s_snapshot_lock);

    s_temp_history[s_history_index] = temp_c;
    /* Only record calibrated values; skip invalid readings so history
     * averages are never polluted with uncalibrated zeros. */
    if (ph_reading.valid) {
        s_ph_history[s_history_index]  = ph_reading.value;
    }
    if (tds_reading.valid) {
        s_tds_history[s_history_index] = tds_reading.value;
    }

    if (s_history_count < SENSOR_SAMPLE_WINDOW) {
        s_history_count++;
    }
    s_history_index = (s_history_index + 1U) % SENSOR_SAMPLE_WINDOW;

    s_snapshot.valid       = true;
    s_snapshot.water_level = water_level ? 1 : 0;
    s_snapshot.water_temp  = average_window(s_temp_history, s_history_count);

    /* Raw counts for pH/TDS are the GPIO level (0 or 1). Water-temp is read
     * via DS18B20 1-Wire and has no raw count in the snapshot. */
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