/**
 * @file  ph_serial.c
 * @brief UART serial pH module driver — passive frame parser.
 *
 * Only used when PH_SOURCE_USE_SERIAL == 1 in pin_config.h.
 *
 * Protocol implemented here:
 *   Header:  'P' 'H' 'A'
 *   Data:    2 bytes, big-endian
 *
 * The driver never transmits. It continuously drains RX bytes, advances a
 * small state machine, caches the latest decoded 16-bit frame, and exposes
 * both the raw frame value and a convenience float derived from it.
 */

#include "ph_serial.h"
#include "pin_config.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include <string.h>

#if (PH_SOURCE_USE_SERIAL == 1)

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

static const char *TAG = "ph_serial";

/** RX ring-buffer size (bytes). Larger buffer enables efficient DMA transfers. */
#define PH_SERIAL_RX_BUF            512

/** Bytes to read at a time while draining the UART RX buffer. */
#define PH_SERIAL_DRAIN_CHUNK        5

/* --------------------------------------------------------------------------
 * Parser state
 * -------------------------------------------------------------------------- */

typedef enum {
    PH_WAITING_HEADER_1 = 0,
    PH_WAITING_HEADER_2,
    PH_WAITING_HEADER_3,
    PH_WAITING_DATA_HIGH,
    PH_WAITING_DATA_LOW,
} ph_parser_state_t;

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */

static bool s_ready = false;
static bool s_synchronized = false;  /* true after first valid frame received */
static TaskHandle_t s_task_handle = NULL;  /* background task handle */
static ph_parser_state_t s_parser_state = PH_WAITING_HEADER_1;
static uint8_t           s_data_high_byte = 0;
static bool              s_has_reading = false;
static int               s_latest_raw = 0;
static float             s_latest_ph = 0.0f;

static float raw_to_ph(int raw)
{
    return (float)raw / 100.0f;
}

static void parser_reset(void)
{
    s_parser_state = PH_WAITING_HEADER_1;
    s_data_high_byte = 0;
}

static void parser_feed_byte(uint8_t byte)
{
    switch (s_parser_state) {
    case PH_WAITING_HEADER_1:
        if (byte == 'P') {
            s_parser_state = PH_WAITING_HEADER_2;
        }
        break;

    case PH_WAITING_HEADER_2:
        if (byte == 'H') {
            s_parser_state = PH_WAITING_HEADER_3;
        } else if (byte == 'P') {
            s_parser_state = PH_WAITING_HEADER_2;
        } else {
            ESP_LOGW(TAG, "Frame error: unexpected 0x%02X in HEADER_2, reset", byte);
            s_parser_state = PH_WAITING_HEADER_1;
        }
        break;

    case PH_WAITING_HEADER_3:
        if (byte == 'A') {
            s_parser_state = PH_WAITING_DATA_HIGH;
        } else if (byte == 'P') {
            s_parser_state = PH_WAITING_HEADER_2;
        } else {
            ESP_LOGW(TAG, "Frame error: unexpected 0x%02X in HEADER_3, reset", byte);
            s_parser_state = PH_WAITING_HEADER_1;
        }
        break;

    case PH_WAITING_DATA_HIGH:
        s_data_high_byte = byte;
        s_parser_state = PH_WAITING_DATA_LOW;
        break;

    case PH_WAITING_DATA_LOW: {
        int raw = (int)(((uint16_t)s_data_high_byte << 8) | (uint16_t)byte);
        float ph = raw_to_ph(raw);
        ESP_LOGI(TAG, "Data low byte: 0x%02X, complete frame: raw=%d ph=%.2f", byte, raw, (double)ph);
        if (ph >= 0.0f && ph <= 14.0f) {
            s_latest_raw = raw;
            s_latest_ph = ph;
            s_has_reading = true;
            
            /* Mark as synchronized on first valid frame */
            if (!s_synchronized) {
                s_synchronized = true;
                ESP_LOGI(TAG, "✓ SYNCHRONIZED with transmitter on first valid frame");
            }
            
            ESP_LOGI(TAG, "pH frame: raw=%d ph=%.2f", raw, (double)ph);
            gpio_set_level(PIN_PH_SERIAL_DEBUG_LED, !gpio_get_level(PIN_PH_SERIAL_DEBUG_LED));
        } else {
            ESP_LOGW(TAG, "✗ pH serial frame out of range: raw=%d ph=%.2f", raw, (double)ph);
        }
        s_parser_state = PH_WAITING_HEADER_1;
        break;
    }
    }
}

static void drain_uart_rx(void)
{
    uint8_t chunk[PH_SERIAL_DRAIN_CHUNK];

    for (;;) {
        int len = uart_read_bytes(PIN_PH_SERIAL_UART, chunk, sizeof(chunk), 0);
        if (len <= 0) {
            break;
        }
        for (int i = 0; i < len; i++) {
            parser_feed_byte(chunk[i]);
        }
    }
}

/* --------------------------------------------------------------------------
 * Background task
 * -------------------------------------------------------------------------- */

static void ph_serial_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "pH serial background task started");
    
    while (s_ready) {
        drain_uart_rx();
        vTaskDelay(pdMS_TO_TICKS(100));  /* Check every 100ms */
    }
    
    ESP_LOGI(TAG, "pH serial background task stopped");
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ph_serial_init(void)
{
    if (s_ready) return ESP_OK;  /* idempotent */

    uart_config_t cfg = {
        .baud_rate           = PIN_PH_SERIAL_BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(PIN_PH_SERIAL_UART, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(PIN_PH_SERIAL_UART,
                       PIN_PH_SERIAL_TX,
                       PIN_PH_SERIAL_RX,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Pull-up on RX to prevent floating-pin noise when transmitter is disconnected */
    gpio_pullup_en(PIN_PH_SERIAL_RX);

    ret = uart_driver_install(PIN_PH_SERIAL_UART,
                              PH_SERIAL_RX_BUF * 2,
                              0,       /* TX buffer — 0 = blocking */
                              0,       /* event queue size */
                              NULL,    /* event queue handle */
                              ESP_INTR_FLAG_IRAM);  /* IRAM interrupt for DMA performance */
    if (ret == ESP_ERR_INVALID_STATE) {
        /* Driver already installed — treat as success */
        ret = ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize debug LED */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << PIN_PH_SERIAL_DEBUG_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);
    gpio_set_level(PIN_PH_SERIAL_DEBUG_LED, 0);

    s_ready = true;
    parser_reset();
    
    /* Create background task to continuously read UART */
    BaseType_t task_ret = xTaskCreate(
        ph_serial_task,
        "ph_serial",
        4096,
        NULL,
        5,
        &s_task_handle
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create pH serial task");
        s_ready = false;
        uart_driver_delete(PIN_PH_SERIAL_UART);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "pH serial ready on UART%d TX=GPIO%d RX=GPIO%d baud=%d (debug LED=GPIO%d)",
             PIN_PH_SERIAL_UART, PIN_PH_SERIAL_TX,
             PIN_PH_SERIAL_RX,   PIN_PH_SERIAL_BAUD,
             PIN_PH_SERIAL_DEBUG_LED);
    return ESP_OK;
}

void ph_serial_deinit(void)
{
    if (!s_ready) return;
    
    /* Stop background task first */
    s_ready = false;  /* Signal task to exit */
    if (s_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));  /* Wait for task to exit */
        if (s_task_handle != NULL) {
            vTaskDelete(s_task_handle);
            s_task_handle = NULL;
        }
    }
    
    uart_driver_delete(PIN_PH_SERIAL_UART);
    gpio_set_level(PIN_PH_SERIAL_DEBUG_LED, 0);
    gpio_reset_pin(PIN_PH_SERIAL_DEBUG_LED);
    s_ready = false;
    s_synchronized = false;
    s_has_reading = false;
    parser_reset();
    ESP_LOGI(TAG, "pH serial deinitialized");
}

bool ph_serial_is_ready(void)
{
    return s_ready;
}

esp_err_t ph_serial_read(int *out_raw, float *out_ph)
{
    if (!s_ready)              return ESP_ERR_INVALID_STATE;
    if (!out_raw || !out_ph)   return ESP_ERR_INVALID_ARG;

    if (!s_has_reading) {
        return ESP_ERR_TIMEOUT;
    }

    *out_raw = s_latest_raw;
    *out_ph = s_latest_ph;
    return ESP_OK;
}

#endif /* PH_SOURCE_USE_SERIAL == 1 */
