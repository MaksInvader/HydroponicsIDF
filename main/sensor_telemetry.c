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
#include "i2c_bus.h"
#if ENABLE_SHT31
#include "sht31.h"
#endif

#if (PH_SOURCE_USE_SERIAL == 1)
#include "ph_serial.h"
#endif

/* Water-level debounce: a HIGH→LOW transition is only committed after the
 * level has been continuously LOW for this many milliseconds — but ONLY
 * when the LOW follows a short HIGH (bounce). A sustained clean LOW
 * (HIGH lasted longer than WATER_LEVEL_BOUNCE_WINDOW_MS) commits immediately. */
#define WATER_LEVEL_DEBOUNCE_LOW_MS  5000

/* A HIGH→LOW transition is considered a bounce only if the preceding HIGH
 * lasted less than this many milliseconds.  Any HIGH longer than this is
 * treated as a stable level, so the subsequent LOW commits immediately. */
#define WATER_LEVEL_BOUNCE_WINDOW_MS  200

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */

/** Rolling window depth for the per-sensor moving average (all sensors). */
#define SENSOR_SAMPLE_WINDOW    30

/** Consecutive DS18B20 read failures before water_temp_sensor_ok is cleared. */
#define SENSOR_TEMP_FAIL_THRESHOLD  100

/** DS18B20 12-bit conversion time in milliseconds. */
#define DS18B20_CONVERSION_MS   750

/** Consecutive 1-Wire failures before triggering bus reinit (backoff strategy).
 * Prevents aggressive reinit attempts that may stress the bus further. */
#define OW_REINIT_AFTER_N_FAILURES  5

/** After the sensor has been declared dead (fail count >= SENSOR_TEMP_FAIL_THRESHOLD),
 * attempt a full bus reinit every this many samples to self-recover without
 * requiring a device restart (~5 min at 2-second sample rate). */
#define OW_DEAD_RETRY_INTERVAL      150

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
    char room_temp_state[TOPIC_BUF_LEN];
    char humidity_state[TOPIC_BUF_LEN];

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
static int                        s_temp_fail_count = 0;  /* consecutive DS18B20 read failures */
static int                        s_ow_reinit_fail_count = 0;  /* consecutive failures before reinit */
static bool                       s_temp_read_fresh = false; /* true only if last read succeeded */
static int                        s_ow_dead_retry_count = 0;  /* samples elapsed since sensor declared dead */

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
static size_t s_temp_history_count;
static size_t s_ph_history_count;
static size_t s_tds_history_count;

/* Calibration state (protected by s_cali_lock) */
static calibration_t s_ph_cali;
static calibration_t s_tds_cali;

/* Deferred publish flags — set when MQTT was not connected at boot */
static bool s_ph_pub_pending;
static bool s_tds_pub_pending;

/* Water-level debounce state */
static esp_timer_handle_t s_wl_debounce_timer  = NULL;
static portMUX_TYPE       s_wl_debounce_lock   = portMUX_INITIALIZER_UNLOCKED;
static int                s_wl_debounced_level = 1;   /* last committed level */
static bool               s_wl_low_pending     = false;
static TickType_t         s_wl_last_high_tick  = 0;   /* tick when level last went/stayed HIGH */

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

static void wl_debounce_timer_cb(void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&s_wl_debounce_lock);
    bool commit = s_wl_low_pending;
    if (commit) {
        s_wl_debounced_level = 0;
        s_wl_low_pending     = false;
    }
    portEXIT_CRITICAL(&s_wl_debounce_lock);
    if (commit) {
        ESP_LOGI(TAG, "WaterLevel debounce elapsed — committing LOW");
    }
}

static void wl_debounce_init(void)
{
    if (s_wl_debounce_timer != NULL) return;
    esp_timer_create_args_t args = {
        .callback        = wl_debounce_timer_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "wl_debounce",
    };
    esp_timer_create(&args, &s_wl_debounce_timer);
}

static void wl_debounce_deinit(void)
{
    if (s_wl_debounce_timer == NULL) return;
    esp_timer_stop(s_wl_debounce_timer);
    esp_timer_delete(s_wl_debounce_timer);
    s_wl_debounce_timer = NULL;
    portENTER_CRITICAL(&s_wl_debounce_lock);
    s_wl_low_pending     = false;
    s_wl_debounced_level = 1;
    s_wl_last_high_tick  = xTaskGetTickCount();
    portEXIT_CRITICAL(&s_wl_debounce_lock);
}

/* Returns the debounced water level.
 *
 * HIGH (1) — committed immediately; records the tick; cancels any pending LOW timer.
 *
 * LOW  (0) — two cases:
 *   Stable LOW: the preceding HIGH lasted > WATER_LEVEL_BOUNCE_WINDOW_MS.
 *               Committed immediately — no timer needed.
 *   Bouncy LOW: the preceding HIGH lasted ≤ WATER_LEVEL_BOUNCE_WINDOW_MS.
 *               Deferred for WATER_LEVEL_DEBOUNCE_LOW_MS; reports HIGH until
 *               the timer fires (same as the original behaviour). */
static int wl_debounce_filter(int raw_level)
{
    if (raw_level == 1) {
        portENTER_CRITICAL(&s_wl_debounce_lock);
        bool was_pending     = s_wl_low_pending;
        s_wl_low_pending     = false;
        s_wl_debounced_level = 1;
        s_wl_last_high_tick  = xTaskGetTickCount();
        portEXIT_CRITICAL(&s_wl_debounce_lock);
        if (was_pending && s_wl_debounce_timer != NULL) {
            esp_timer_stop(s_wl_debounce_timer);
        }
        return 1;
    }

    /* raw_level == 0 */
    portENTER_CRITICAL(&s_wl_debounce_lock);
    int  current         = s_wl_debounced_level;
    bool already_pending = s_wl_low_pending;
    TickType_t last_high = s_wl_last_high_tick;
    portEXIT_CRITICAL(&s_wl_debounce_lock);

    if (current == 0) {
        return 0; /* already committed LOW */
    }

    /* Determine whether the preceding HIGH was stable or a bounce. */
    uint32_t high_duration_ms = (uint32_t)((xTaskGetTickCount() - last_high) * (TickType_t)portTICK_PERIOD_MS);
    bool is_bounce = (high_duration_ms <= WATER_LEVEL_BOUNCE_WINDOW_MS);

    if (!is_bounce) {
        /* Stable HIGH→LOW: commit immediately, no timer needed. */
        portENTER_CRITICAL(&s_wl_debounce_lock);
        s_wl_low_pending = true;
        s_wl_debounced_level = 0;
        s_wl_low_pending     = false;
        portEXIT_CRITICAL(&s_wl_debounce_lock);
        ESP_LOGI(TAG, "WaterLevel HIGH→LOW (stable, %lu ms HIGH) — committing immediately",
                 (unsigned long)high_duration_ms);
        return 0;
    }

    /* Bouncy HIGH→LOW: start debounce timer if not already running */
    if (!already_pending) {
        portENTER_CRITICAL(&s_wl_debounce_lock);
        s_wl_low_pending = true;
        portEXIT_CRITICAL(&s_wl_debounce_lock);
        if (s_wl_debounce_timer != NULL) {
            esp_timer_stop(s_wl_debounce_timer);
            esp_timer_start_once(s_wl_debounce_timer,
                                 (uint64_t)WATER_LEVEL_DEBOUNCE_LOW_MS * 1000ULL);
        }
        ESP_LOGI(TAG, "WaterLevel HIGH→LOW (bounce, %lu ms HIGH) — deferring for %d ms",
                 (unsigned long)high_duration_ms, WATER_LEVEL_DEBOUNCE_LOW_MS);
    }

    return 1; /* report HIGH until timer fires */
}

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
    s_history_index      = 0;
    s_temp_history_count = 0;
    s_ph_history_count   = 0;
    s_tds_history_count  = 0;
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
 * @brief  Tear down the 1-Wire bus and DS18B20 device handles.
 * Safe to call even if init never completed — checks each handle before deleting.
 */
static void ow_deinit(void)
{
    s_ow_ready              = false;
    s_ow_conversion_pending = false;

    if (s_ds18b20 != NULL) {
        ds18b20_del_device(s_ds18b20);
        s_ds18b20 = NULL;
    }
    if (s_ow_bus != NULL) {
        onewire_bus_del(s_ow_bus);
        s_ow_bus = NULL;
    }
}

/**
 * @brief  Tear down and re-initialise the 1-Wire bus after a read error.
 * A short delay lets the RMT peripheral fully release its resources before
 * the new bus handle is created.
 */
static void ow_reinit(void)
{
    ESP_LOGW(TAG, "DS18B20 reinit — tearing down 1-Wire bus");
    ow_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Reset ALL failure counters unconditionally BEFORE attempting init.
     * This prevents a reinit storm: without this, a failed reinit leaves
     * s_ow_reinit_fail_count >= OW_REINIT_AFTER_N_FAILURES, so the very
     * next trigger failure immediately calls ow_reinit() again instead of
     * allowing the 5-failure backoff window to apply fresh. */
    s_ow_reinit_fail_count = 0;
    s_temp_fail_count      = 0;
    s_ow_dead_retry_count  = 0;

    ow_init();
    if (s_ow_ready) {
        /* Trigger a fresh conversion so the next sample cycle has a result */
        esp_err_t ret = ds18b20_trigger_temperature_conversion(s_ds18b20);
        if (ret == ESP_OK) {
            s_ow_conversion_pending = true;
            ESP_LOGI(TAG, "DS18B20 reinit succeeded");
        } else {
            ESP_LOGE(TAG, "DS18B20 reinit: trigger failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "DS18B20 reinit failed — will retry in %d samples",
                 OW_DEAD_RETRY_INTERVAL);
    }
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
        s_ow_reinit_fail_count = 0;  /* success — reset backoff counter */
    } else {
        s_ow_conversion_pending = false;
        s_ow_reinit_fail_count++;
        if (s_ow_reinit_fail_count >= OW_REINIT_AFTER_N_FAILURES) {
            ESP_LOGW(TAG, "DS18B20 conversion trigger failed %d times: %s — reinitialising",
                     s_ow_reinit_fail_count, esp_err_to_name(ret));
            ow_reinit();
        } else {
            ESP_LOGD(TAG, "DS18B20 conversion trigger failed (%d/%d): %s — will retry",
                     s_ow_reinit_fail_count, OW_REINIT_AFTER_N_FAILURES,
                     esp_err_to_name(ret));
        }
    }
}

/**
 * @brief  Read the result of the previously triggered conversion.
 *
 * Call at the START of a sampling cycle (after the previous cycle triggered).
 * Returns ESP_ERR_INVALID_STATE if no conversion was pending.
 * On read failure, triggers a bus reinit and returns ESP_FAIL so the caller
 * falls back to the rolling average for this cycle.
 */
static esp_err_t sample_water_temp(float *temp_c)
{
    if (temp_c == NULL) return ESP_ERR_INVALID_ARG;
    *temp_c = 0.0f;

    if (!s_ow_ready || s_ds18b20 == NULL) return ESP_ERR_INVALID_STATE;
    if (!s_ow_conversion_pending)         return ESP_ERR_INVALID_STATE;

    s_ow_conversion_pending = false;
    esp_err_t ret = ds18b20_get_temperature(s_ds18b20, temp_c);
    if (ret != ESP_OK) {
        s_ow_reinit_fail_count++;
        if (s_ow_reinit_fail_count >= OW_REINIT_AFTER_N_FAILURES) {
            ESP_LOGW(TAG, "DS18B20 read failed %d times (%s) — reinitialising",
                     s_ow_reinit_fail_count, esp_err_to_name(ret));
            ow_reinit();
        } else {
            ESP_LOGD(TAG, "DS18B20 read failed (%d/%d): %s — will retry",
                     s_ow_reinit_fail_count, OW_REINIT_AFTER_N_FAILURES,
                     esp_err_to_name(ret));
        }
        return ESP_FAIL; /* caller uses last-known-good from rolling average */
    }
    s_ow_reinit_fail_count = 0;  /* success — reset backoff counter */
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * ADS1115 I2C driver (minimal, single-shot)
 * -------------------------------------------------------------------------- */

/* ADS1115 register addresses */
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

/* Config register bits */
#define ADS1115_OS_SINGLE       (1u << 15)  /* start single conversion      */
#define ADS1115_MUX_AIN0_GND    (4u << 12)  /* AIN0 vs GND — unused         */
#define ADS1115_MUX_AIN1_GND    (5u << 12)  /* AIN1 vs GND — TDS            */
#define ADS1115_PGA_4096        (1u << 9)   /* ±4.096 V FSR                 */
#define ADS1115_MODE_SINGLE     (1u << 8)   /* single-shot mode             */
#define ADS1115_DR_128SPS       (4u << 5)   /* 128 samples/s (DR[2:0]=100)  */
#define ADS1115_COMP_DISABLE    (3u << 0)   /* disable comparator           */

#define ADS1115_CONVERSION_MS   80      /* ~8 ms at 128 SPS + margin    */

/* Number of ADC samples taken per pH reading — median of these is used */
#define PH_MEDIAN_SAMPLES  5

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
        .master.clk_speed = 50000,
        .clk_flags        = 0,
    };
    /* Ensure the shared bus mutex exists before any I2C access */
    i2c_bus_init();

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
    if (i2c_bus_lock(pdMS_TO_TICKS(500)) != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 init: I2C bus lock timeout");
        return ESP_ERR_TIMEOUT;
    }
    ret = i2c_master_write_to_device(PIN_ADS1115_I2C_PORT, PIN_ADS1115_I2C_ADDR,
                                     buf, sizeof(buf), pdMS_TO_TICKS(50));
    i2c_bus_unlock();
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
 * @brief  Tear down the I2C driver for the ADS1115.
 * Safe to call even if init never completed.
 */
static void ads1115_deinit(void)
{
    s_ads1115_ready = false;
    /* Do NOT delete the I2C driver — it is shared with the LCD.
     * The LCD owns the driver lifetime; we only mark ourselves not ready. */
}

/**
 * @brief  Tear down and re-initialise the ADS1115 I2C driver after a read error.
 * A short delay lets the I2C bus settle before reinitialising.
 */
static void ads1115_reinit(void)
{
    ESP_LOGW(TAG, "ADS1115 reinit — resetting I2C driver");
    ads1115_deinit();
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t ret = ads1115_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADS1115 reinit succeeded");
    } else {
        ESP_LOGE(TAG, "ADS1115 reinit failed: %s — will retry next cycle", esp_err_to_name(ret));
    }
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

    /* --- Phase 1: write config register (start conversion) --- */
    if (i2c_bus_lock(pdMS_TO_TICKS(500)) != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 read: I2C bus lock timeout (write phase)");
        return ESP_ERR_TIMEOUT;
    }

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
    i2c_bus_unlock();

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 write failed (%s) — reinitialising", esp_err_to_name(ret));
        ads1115_reinit();
        return ESP_FAIL;
    }

    /* --- Phase 2: wait for conversion — mutex released, bus is free --- */
    vTaskDelay(pdMS_TO_TICKS(ADS1115_CONVERSION_MS));

    /* --- Phase 3: read conversion result --- */
    if (i2c_bus_lock(pdMS_TO_TICKS(500)) != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 read: I2C bus lock timeout (read phase)");
        return ESP_ERR_TIMEOUT;
    }

    /* i2c_master_write_read_device sends the register address byte before
     * reading, so the ADS1115 register pointer is always explicitly set to
     * ADS1115_REG_CONVERSION regardless of what other transactions occurred
     * during the conversion wait. */
    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t rd[2] = {0};
    ret = i2c_master_write_read_device(PIN_ADS1115_I2C_PORT,
                                       PIN_ADS1115_I2C_ADDR,
                                       &reg, 1, rd, 2,
                                       pdMS_TO_TICKS(50));
    i2c_bus_unlock();

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADS1115 read failed (%s) — reinitialising", esp_err_to_name(ret));
        ads1115_reinit();
        return ESP_FAIL;
    }

    int16_t raw = (int16_t)((rd[0] << 8) | rd[1]);
    /* Single-ended: clamp negatives to 0 */
    *out_raw = (raw < 0) ? 0 : (int)raw;
    return ESP_OK;
}

#if (PH_SOURCE_USE_SERIAL == 2)
/**
 * @brief  Read PH_MEDIAN_SAMPLES pH ADC values and return the median.
 *
 * Filters out spike noise from the pH analog front-end.
 * Returns ESP_FAIL immediately if any individual read fails.
 */
static esp_err_t ads1115_read_ph_median(int *out_raw)
{
    if (out_raw == NULL) return ESP_ERR_INVALID_ARG;

    int samples[PH_MEDIAN_SAMPLES];
    for (int i = 0; i < PH_MEDIAN_SAMPLES; i++) {
        esp_err_t ret = ads1115_read_channel(ADS1115_MUX_AIN1_GND, &samples[i]);
        if (ret != ESP_OK) return ret;
    }

    /* Insertion sort — tiny fixed array, no need for qsort */
    for (int i = 1; i < PH_MEDIAN_SAMPLES; i++) {
        int key = samples[i];
        int j   = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    *out_raw = samples[PH_MEDIAN_SAMPLES / 2];  /* middle element = median */
    return ESP_OK;
}
#endif /* PH_SOURCE_USE_SERIAL == 2 */

/* --------------------------------------------------------------------------
 * TDS median read — always compiled (TDS always uses ADS1115 AIN1)
 * -------------------------------------------------------------------------- */

/* Number of ADC samples taken per TDS reading — median is used to reject
 * impulse noise from the conductivity probe and pump switching interference. */
#define TDS_MEDIAN_SAMPLES 3

/**
 * @brief  Read TDS_MEDIAN_SAMPLES TDS ADC values and return the median.
 *
 * Mirrors ads1115_read_ph_median() but uses ADS1115_MUX_AIN1_GND (TDS channel).
 * Returns ESP_FAIL on the first failing individual read — caller treats the
 * entire batch as failed so no partial data enters the history.
 */
#if 0
static esp_err_t ads1115_read_tds_median(int *out_raw)
{
    if (out_raw == NULL) return ESP_ERR_INVALID_ARG;

    int samples[TDS_MEDIAN_SAMPLES];
    for (int i = 0; i < TDS_MEDIAN_SAMPLES; i++) {
        esp_err_t ret = ads1115_read_channel(ADS1115_MUX_AIN1_GND, &samples[i]);
        if (ret != ESP_OK) return ret;
    }

    /* Insertion sort — fixed-size array, no qsort needed */
    for (int i = 1; i < TDS_MEDIAN_SAMPLES; i++) {
        int key = samples[i];
        int j   = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    *out_raw = samples[TDS_MEDIAN_SAMPLES / 2];  /* middle element = median */
    return ESP_OK;
}
#endif


/* --------------------------------------------------------------------------
 * Calibration application
 * -------------------------------------------------------------------------- */

#if (PH_SOURCE_USE_SERIAL == 2)
/**
 * @brief  Apply a linear calibration to an ADC raw count and validate.
 *
 * The linear model maps the raw ADC count to a physical value via the stored
 * slope/offset coefficients: value = slope * raw + offset.
 *
 * Used only by the ADS1115 pH path (PH_SOURCE_USE_SERIAL == 2).
 * Serial pH and TDS do inline calibration for their specific needs.
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
#endif /* PH_SOURCE_USE_SERIAL == 2 */

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
    WRITE_TOPIC(s_topics.room_temp_state,   "%s/sensor/RoomTemperature/state", zone_id);
    WRITE_TOPIC(s_topics.humidity_state,    "%s/sensor/Humidity/state",   zone_id);

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

    #if (ENABLE_TDS_SENSOR || (PH_SOURCE_USE_SERIAL == 2))
    /* pH and TDS — ADS1115 16-bit ADC over I2C */
    ret = ads1115_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    #endif

    #if (PH_SOURCE_USE_SERIAL == 1)
        /* Serial pH module — fatal if init fails because this mode was explicitly chosen */
        ret = ph_serial_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "pH serial init failed: %s — cannot continue in serial mode",
                     esp_err_to_name(ret));
            return ret;
        }
    #endif

    #if ENABLE_WATER_TEMP
    /* DS18B20 1-Wire (water temperature) */
    ow_init();

    /* Trigger the first conversion immediately so the very first
     * sample_water_temp() call one cycle later has a result ready. */
    ow_trigger_conversion();
    #endif

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

    /* Must be a JSON object (heuristic: starts with '{') */
    const char *p = buf;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != '{') {
        snprintf(reason_out, 128, "payload is not a JSON object");
        return false;
    }

    /* ── Extract two calibration points ─────────────────────────────────── */
    float actual_1 = 0.0f, actual_2 = 0.0f;
    float raw_1 = 0.0f, raw_2 = 0.0f;

    if (!json_get_float(buf, "actual_1", &actual_1)) {
        snprintf(reason_out, 128, "missing field: actual_1");
        return false;
    }
    if (!json_get_float(buf, "raw_1", &raw_1)) {
        snprintf(reason_out, 128, "missing field: raw_1");
        return false;
    }
    if (!json_get_float(buf, "actual_2", &actual_2)) {
        snprintf(reason_out, 128, "missing field: actual_2");
        return false;
    }
    if (!json_get_float(buf, "raw_2", &raw_2)) {
        snprintf(reason_out, 128, "missing field: raw_2");
        return false;
    }

    /* All values must be finite */
    if (!isfinite(actual_1) || !isfinite(raw_1) ||
        !isfinite(actual_2) || !isfinite(raw_2)) {
        snprintf(reason_out, 128, "calibration point values must be finite");
        return false;
    }

    /* raw_1 and raw_2 must differ — otherwise slope is undefined */
    float raw_diff = raw_2 - raw_1;
    if (raw_diff == 0.0f) {
        snprintf(reason_out, 128, "raw_1 and raw_2 must differ (division by zero)");
        return false;
    }

    /* ── Compute linear coefficients: value = slope * raw + offset ────── */
    float slope  = (actual_2 - actual_1) / raw_diff;
    float offset = actual_1 - slope * raw_1;

    /* Computed slope and offset must be finite */
    if (!isfinite(slope)) {
        snprintf(reason_out, 128, "computed slope is not finite");
        return false;
    }
    if (!isfinite(offset)) {
        snprintf(reason_out, 128, "computed offset is not finite");
        return false;
    }

    /* Slope must be non-zero */
    if (slope == 0.0f) {
        snprintf(reason_out, 128, "computed slope is zero (actual_1 == actual_2)");
        return false;
    }

    /* Plausible range check on computed coefficients */
    if (slope < range->slope_min || slope > range->slope_max) {
        snprintf(reason_out, 128, "computed slope %.4f out of range [%.4f, %.4f]",
                 (double)slope, (double)range->slope_min, (double)range->slope_max);
        return false;
    }
    if (offset < range->offset_min || offset > range->offset_max) {
        snprintf(reason_out, 128, "computed offset %.4f out of range [%.4f, %.4f]",
                 (double)offset, (double)range->offset_min, (double)range->offset_max);
        return false;
    }

    out->slope      = slope;
    out->offset     = offset;
    out->valid      = true;
    /* Prefer the timestamp supplied by the sender (Unix epoch seconds).
     * Fall back to a boot-relative counter only when the field is absent. */
    float upd_f = 0.0f;
    if (json_get_float(buf, "updated_at", &upd_f) && upd_f > 0.0f) {
        out->updated_at = (uint32_t)upd_f;
    } else {
        out->updated_at = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    }

    ESP_LOGD(TAG, "Cali points: (%.0f,%.0f)(%.0f,%.0f) → s=%.4f o=%.4f",
             (double)raw_1, (double)actual_1, (double)raw_2, (double)actual_2,
             (double)slope, (double)offset);
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
        ESP_LOGW(TAG, "%s cali rejected: %s", type_name, reason);
        publish_cali_error(state_topic, valid_topic, prev_updated_at, reason);
        return;
    }

    /* --- Req 5: NVS write with retry --- */
    esp_err_t save_ret = ESP_FAIL;
    for (int attempt = 0; attempt <= CALI_NVS_RETRY_COUNT; attempt++) {
        save_ret = nvs_save(&new_cali);
        if (save_ret == ESP_OK) break;
        ESP_LOGD(TAG, "%s NVS save retry %d: %s",
                 type_name, attempt + 1, esp_err_to_name(save_ret));
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

    ESP_LOGI(TAG, "%s cali set: s=%.4f o=%.4f",
             type_name, (double)new_cali.slope, (double)new_cali.offset);

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
            ESP_LOGD(TAG, "Sub OK: %s", topic);
            return ESP_OK;
        }
        ESP_LOGD(TAG, "Sub retry %d %s: %s",
                 attempt + 1, topic, esp_err_to_name(ret));
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
            ESP_LOGD(TAG, "No %s cali in NVS", sensors[i].name);
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
            ESP_LOGW(TAG, "%s NVS cali invalid/OOR — discarding", sensors[i].name);
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
        ESP_LOGD(TAG, "%s NVS cali: s=%.4f o=%.4f",
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
            ESP_LOGD(TAG, "Cali sub deferred: %s", esp_err_to_name(ret));
        }
    }

    s_initialized = true;
    #if ENABLE_WATER_LEVEL
    wl_debounce_init();
    #endif
    ESP_LOGI(TAG, "Sensor telemetry initialized for zone: %s", zone_id);
    return ESP_OK;
}

esp_err_t sensor_telemetry_deinit(void)
{
    mqtt_manager_remove_connection_cb(on_mqtt_connected);
    if (s_topics.cali_ph_set[0]  != '\0') mqtt_manager_unsubscribe(s_topics.cali_ph_set);
    if (s_topics.cali_tds_set[0] != '\0') mqtt_manager_unsubscribe(s_topics.cali_tds_set);

    #if (ENABLE_TDS_SENSOR || (PH_SOURCE_USE_SERIAL == 2))
    ads1115_deinit();
    #endif
    #if ENABLE_WATER_TEMP
    ow_deinit();
    #endif

    #if (PH_SOURCE_USE_SERIAL == 1)
        ph_serial_deinit();
    #endif

    portENTER_CRITICAL(&s_snapshot_lock);
    reset_state_locked();
    portEXIT_CRITICAL(&s_snapshot_lock);

    portENTER_CRITICAL(&s_cali_lock);
    memset(&s_ph_cali,  0, sizeof(s_ph_cali));
    memset(&s_tds_cali, 0, sizeof(s_tds_cali));
    portEXIT_CRITICAL(&s_cali_lock);

    memset(&s_topics, 0, sizeof(s_topics));

    s_initialized = false;
    #if ENABLE_WATER_LEVEL
    wl_debounce_deinit();
    #endif
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
    #if ENABLE_WATER_LEVEL
    int water_level = wl_debounce_filter(gpio_get_level((gpio_num_t)PIN_SENSOR_WATER_LEVEL));
    #else
    int water_level = 0;
    #endif

    /* ── Water temperature (DS18B20 1-Wire, trigger-then-read pattern) ── */
    #if ENABLE_WATER_TEMP
    /* Read the result of the conversion triggered at the end of the         */
    /* previous cycle, then immediately trigger the next conversion so it    */
    /* is ready by the time this function is called again (~1000 ms later).  */
    float temp_c = 0.0f;
    bool temp_valid = (sample_water_temp(&temp_c) == ESP_OK);
    ow_trigger_conversion();

    /* Track consecutive failures to expose sensor health in the snapshot */
    s_temp_read_fresh = temp_valid;
    if (temp_valid) {
        s_temp_fail_count     = 0;
        s_ow_dead_retry_count = 0;
    } else {
        if (s_temp_fail_count < SENSOR_TEMP_FAIL_THRESHOLD) {
            s_temp_fail_count++;
        } else {
            /* Sensor has been continuously dead for SENSOR_TEMP_FAIL_THRESHOLD
             * samples. Attempt a full bus reinit every OW_DEAD_RETRY_INTERVAL
             * samples to self-recover from permanent hardware glitches without
             * requiring a device restart. */
            s_ow_dead_retry_count++;
            if (s_ow_dead_retry_count >= OW_DEAD_RETRY_INTERVAL) {
                ESP_LOGW(TAG, "DS18B20 dead for %d samples — attempting recovery reinit",
                         SENSOR_TEMP_FAIL_THRESHOLD + s_ow_dead_retry_count);
                ow_reinit();
            }
        }
    }
    #else
    float temp_c = 0.0f;
    bool temp_valid = false;
    #endif

    /* Track raw read success independently of calibration validity.
     * These flags gate snap.valid so /raw topics publish even when
     * calibration doesn't exist yet — essential for calibration workflow. */
    bool temp_raw_ok = temp_valid;
    bool ph_raw_ok = false;
    bool tds_raw_ok = false;

    /* ── pH ────────────────────────────────────────────────────────────── */
    sensor_reading_t ph_reading = {0};

    /* Take a local copy of calibration to avoid holding the lock during I/O */
    portENTER_CRITICAL(&s_cali_lock);
    #if (PH_SOURCE_USE_SERIAL == 1) || (PH_SOURCE_USE_SERIAL == 2)
    calibration_t ph_cali  = s_ph_cali;
    #endif
    calibration_t tds_cali = s_tds_cali;
    portEXIT_CRITICAL(&s_cali_lock);

    #if (PH_SOURCE_USE_SERIAL == 1)
        {
            int   serial_raw = 0;
            esp_err_t serial_ret = ph_serial_read(&serial_raw);
            if (serial_ret == ESP_OK) {
                ph_reading.raw = (uint16_t)serial_raw;
                ph_raw_ok = true;
                /* Gate on calibration: serial module outputs a raw ADC
                 * value but we still require a valid calibration entry before
                 * marking the reading valid so safety checks are not triggered
                 * on an uncalibrated sensor. */
                if (ph_cali.valid) {
                    float calibrated = ph_cali.slope * (float)serial_raw + ph_cali.offset;
                    if (isfinite(calibrated) &&
                        calibrated >= PH_VALUE_MIN &&
                        calibrated <= PH_VALUE_MAX) {
                        ph_reading.value = calibrated;
                        ph_reading.valid = true;
                    }
                }
            } else {
                ESP_LOGW(TAG, "pH serial read failed: %s", esp_err_to_name(serial_ret));
            }
        }
    #elif (PH_SOURCE_USE_SERIAL == 2)
        {
            /* ADS1115 path — AIN1 for pH only; TDS on AIN0 below is untouched */
            int ph_raw = 0;
            esp_err_t adc_ret = ads1115_read_ph_median(&ph_raw);
            if (adc_ret != ESP_OK) {
                ESP_LOGW(TAG, "pH ADS1115 read failed: %s", esp_err_to_name(adc_ret));
                ph_raw = 0;
            } else {
                ph_raw_ok = true;
            }
            apply_adc_calibration_and_validate(&ph_cali, ph_raw,
                                               PH_VALUE_MIN, PH_VALUE_MAX,
                                               &ph_reading);
        }
    #endif

    /* ── TDS ───────────────────────────────────────────────────────────── */
    sensor_reading_t tds_reading = {0};

    #if ENABLE_TDS_SENSOR
    {
        int tds_raw = 0;
        /* Median filter temporarily disabled for trial — single sample. */
        esp_err_t adc_ret = ads1115_read_channel(ADS1115_MUX_AIN1_GND, &tds_raw);
        if (adc_ret != ESP_OK) {
            ESP_LOGW(TAG, "TDS ADS1115 read failed: %s", esp_err_to_name(adc_ret));
            /* Do NOT fall through to calibration — tds_reading stays zeroed.
             * tds_raw_ok remains false so neither the history nor the snapshot
             * valid flag are updated from this failed cycle. */
        } else {
            tds_raw_ok = true;

            /* Store raw ADC count only on a successful read so that a failed
             * read never publishes raw=0 as if it were a real measurement. */
            tds_reading.raw = (uint16_t)tds_raw;

            /* Temperature compensation: normalise raw signal to 25 °C equivalent.
             * comp_raw = raw / (1.0 + 0.02 * (T - 25))
             * Falls back to 25 °C (coeff = 1.0, no-op) when temp is unavailable. */
            float tds_temp_c     = temp_valid ? temp_c : 25.0f;
            float tds_comp_coeff = 1.0f + 0.02f * (tds_temp_c - 25.0f);
            if (tds_comp_coeff < 0.01f) tds_comp_coeff = 0.01f; /* guard div-by-zero */
            float tds_comp_raw   = (float)tds_raw / tds_comp_coeff;

            if (tds_cali.valid) {
                float tds_computed = tds_cali.slope * tds_comp_raw + tds_cali.offset;
                if (isfinite(tds_computed) &&
                    tds_computed >= TDS_VALUE_MIN &&
                    tds_computed <= TDS_VALUE_MAX) {
                    tds_reading.value = tds_computed;
                    tds_reading.valid = true;
                }
            }
        }
    }
    #endif

    /* ── Environment (SHT31) ───────────────────────────────────────────── */
    float room_temp = 0.0f;
    float humidity = 0.0f;
#if ENABLE_SHT31
    bool  env_valid = (sht31_read_temp_and_humidity(&room_temp, &humidity) == ESP_OK);
#else
    bool  env_valid = false;
#endif

    /* ── Update rolling history & snapshot ────────────────────────────── */
    portENTER_CRITICAL(&s_snapshot_lock);

    /* Only record valid readings so history averages are never polluted
     * with zeros from failed reads or absent calibration.
     * Each sensor has its own count so average_window() divides only by
     * the number of slots actually written — prevents startup zero-pollution
     * from dragging the average below range thresholds. */
    if (temp_valid) {
        s_temp_history[s_history_index] = temp_c;
        if (s_temp_history_count < SENSOR_SAMPLE_WINDOW) s_temp_history_count++;
    }
    if (ph_reading.valid) {
        s_ph_history[s_history_index] = ph_reading.value;
        if (s_ph_history_count < SENSOR_SAMPLE_WINDOW) s_ph_history_count++;
    }
    if (tds_reading.valid) {
        s_tds_history[s_history_index] = tds_reading.value;
        if (s_tds_history_count < SENSOR_SAMPLE_WINDOW) s_tds_history_count++;
    }

    s_history_index = (s_history_index + 1U) % SENSOR_SAMPLE_WINDOW;

    /* Snapshot is valid as soon as ANY raw read succeeds, regardless of
     * calibration state. This enables /raw publishing before calibration
     * exists — essential for server-side calibration workflow. */
    s_snapshot.valid = temp_raw_ok || ph_raw_ok || tds_raw_ok || env_valid;
    s_snapshot.water_level          = water_level ? 1 : 0;
    s_snapshot.water_temp           = average_window(s_temp_history, s_temp_history_count);
#if ENABLE_WATER_TEMP
    s_snapshot.water_temp_sensor_ok = s_temp_read_fresh;  /* only true when last read was valid */
    s_snapshot.water_temp_sensor_dead = (s_temp_fail_count >= SENSOR_TEMP_FAIL_THRESHOLD);
#else
    s_snapshot.water_temp_sensor_ok = false;   /* sensor disabled — suppress safety checks */
    s_snapshot.water_temp_sensor_dead = false;
#endif

    s_snapshot.room_temp       = room_temp;
    s_snapshot.room_temp_valid = env_valid;
    s_snapshot.humidity        = humidity;
    s_snapshot.humidity_valid  = env_valid;

    /* Raw counts for pH/TDS are ADS1115 16-bit counts (0–32767).
     * Water-temp is read via DS18B20 1-Wire and has no raw count. */
    s_snapshot.ph_raw   = ph_reading.raw;
    s_snapshot.ph       = average_window(s_ph_history, s_ph_history_count);
    s_snapshot.ph_valid = ph_reading.valid;

    s_snapshot.tds_raw   = tds_reading.raw;
    s_snapshot.tds       = average_window(s_tds_history, s_tds_history_count);
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
const char *sensor_telemetry_topic_room_temp(void)   { return s_topics.room_temp_state;   }
const char *sensor_telemetry_topic_humidity(void)    { return s_topics.humidity_state;    }
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