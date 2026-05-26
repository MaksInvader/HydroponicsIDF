#include "indicator_led.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "pin_config.h"

#define FAULT_LED_DEFAULT_MS  5000U
#define BLINK_QUEUE_LEN       4
#define BLINK_TASK_STACK      1536
#define BLINK_TASK_PRIORITY   1     /* low priority — purely cosmetic */

/* Special duration value that tells the blink task to turn the LED off
 * immediately and discard any queued blink. */
#define BLINK_CANCEL          0xFFFFFFFFU

static const char *TAG = "indicator_led";

static bool           s_initialized = false;
static QueueHandle_t  s_blink_queue = NULL;
static TaskHandle_t   s_blink_task  = NULL;

/* ── Background blink task ───────────────────────────────────────────────── */

static void blink_task(void *arg)
{
    (void)arg;
    uint32_t duration_ms;

    while (true) {
        /* Block indefinitely until a blink request arrives. */
        if (xQueueReceive(s_blink_queue, &duration_ms, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (duration_ms == BLINK_CANCEL) {
            gpio_set_level((gpio_num_t)PIN_LED_FAULT, 0);
            gpio_set_level((gpio_num_t)PIN_RESERVE_BINARY, 0);
            /* Drain any further queued requests. */
            while (xQueueReceive(s_blink_queue, &duration_ms, 0) == pdTRUE) {}
            continue;
        }

        gpio_set_level((gpio_num_t)PIN_LED_FAULT, 1);
        gpio_set_level((gpio_num_t)PIN_RESERVE_BINARY, 1);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        gpio_set_level((gpio_num_t)PIN_LED_FAULT, 0);
        gpio_set_level((gpio_num_t)PIN_RESERVE_BINARY, 0);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t indicator_led_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_CONNECTION) | (1ULL << PIN_LED_FAULT)
                      | (1ULL << PIN_RESERVE_BINARY),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level((gpio_num_t)PIN_LED_CONNECTION, 0);
    gpio_set_level((gpio_num_t)PIN_LED_FAULT, 0);
    gpio_set_level((gpio_num_t)PIN_RESERVE_BINARY, 0);

    s_blink_queue = xQueueCreate(BLINK_QUEUE_LEN, sizeof(uint32_t));
    if (s_blink_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create blink queue");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreate(blink_task, "led_blink", BLINK_TASK_STACK,
                                NULL, BLINK_TASK_PRIORITY, &s_blink_task);
    if (ok != pdPASS) {
        vQueueDelete(s_blink_queue);
        s_blink_queue = NULL;
        ESP_LOGE(TAG, "Failed to create blink task");
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Indicator LEDs initialised (conn=GPIO%d, fault=GPIO%d)",
             PIN_LED_CONNECTION, PIN_LED_FAULT);
    return ESP_OK;
}

void indicator_led_set_connection(bool connected)
{
    if (!s_initialized) {
        return;
    }
    gpio_set_level((gpio_num_t)PIN_LED_CONNECTION, connected ? 1 : 0);
}

void indicator_led_fault_blink(uint32_t duration_ms)
{
    if (!s_initialized || s_blink_queue == NULL) {
        return;
    }
    if (duration_ms == 0 || duration_ms == BLINK_CANCEL) {
        duration_ms = FAULT_LED_DEFAULT_MS;
    }
    /* Non-blocking send — drop silently if queue is full. */
    xQueueSend(s_blink_queue, &duration_ms, 0);
}

void indicator_led_set_fault(bool on)
{
    if (!s_initialized || s_blink_queue == NULL) {
        return;
    }
    if (on) {
        /* Drive immediately from this task context. */
        gpio_set_level((gpio_num_t)PIN_LED_FAULT, 1);
        gpio_set_level((gpio_num_t)PIN_RESERVE_BINARY, 1);
    } else {
        /* Send cancel so the blink task also turns it off if it's mid-blink. */
        uint32_t cancel = BLINK_CANCEL;
        xQueueSend(s_blink_queue, &cancel, 0);
    }
}
