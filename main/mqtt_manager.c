#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "mqtt_manager.h"

#define MQTT_CONNECTED_BIT        BIT0
#define MQTT_SETUP_SUCCESS_BIT    BIT1
#define MQTT_MAX_SUBSCRIPTIONS    12
#define MQTT_DEFAULT_SUB_QOS      1
#define MQTT_SUB_LOCK_TIMEOUT_MS  100

static const char *TAG = "mqtt_manager";

typedef struct {
    bool in_use;
    char topic[128];
    mqtt_manager_message_cb_t cb;
    void *user_ctx;
} mqtt_subscription_entry_t;

static esp_mqtt_client_handle_t s_client;
static EventGroupHandle_t       s_event_group;
static char                     s_success_topic[96];
static mqtt_subscription_entry_t s_subscriptions[MQTT_MAX_SUBSCRIPTIONS];
static SemaphoreHandle_t        s_subscriptions_lock;
static uint32_t                 s_subscriptions_lock_timeout_count;

typedef struct {
    char topic[128];
    mqtt_manager_message_cb_t cb;
    void *user_ctx;
} mqtt_dispatch_snapshot_t;

/* ── lock helpers ────────────────────────────────────────────────────────── */
static bool subscriptions_lock_take(const char *caller)
{
    if (s_subscriptions_lock == NULL) {
        ESP_LOGE(TAG, "Subscription lock is not initialized (%s)", caller);
        return false;
    }

    if (xSemaphoreTake(s_subscriptions_lock, pdMS_TO_TICKS(MQTT_SUB_LOCK_TIMEOUT_MS)) != pdTRUE) {
        s_subscriptions_lock_timeout_count++;
        ESP_LOGE(TAG, "Subscription lock timeout in %s (count=%lu)",
                 caller, (unsigned long)s_subscriptions_lock_timeout_count);
        return false;
    }
    return true;
}

static void subscriptions_lock_give(void)
{
    if (s_subscriptions_lock != NULL) {
        xSemaphoreGive(s_subscriptions_lock);
    }
}

/* ── public helpers ──────────────────────────────────────────────────────── */
bool mqtt_manager_is_connected(void)
{
    if (s_event_group == NULL) return false;
    return (xEventGroupGetBits(s_event_group) & MQTT_CONNECTED_BIT) != 0;
}

/* ── internal dispatch ───────────────────────────────────────────────────── */
static bool topic_matches(const char *event_topic, int event_topic_len,
                          const char *registered_topic)
{
    size_t reg_len = strlen(registered_topic);
    return (event_topic_len == (int)reg_len) &&
           (strncmp(event_topic, registered_topic, reg_len) == 0);
}

static void dispatch_subscribed_message(const char *topic, int topic_len,
                                        const char *payload, int payload_len)
{
    mqtt_dispatch_snapshot_t snapshot[MQTT_MAX_SUBSCRIPTIONS];
    size_t snapshot_count = 0;

    if (!subscriptions_lock_take("dispatch_subscribed_message")) return;

    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++) {
        if (!s_subscriptions[i].in_use || s_subscriptions[i].cb == NULL) continue;

        if (topic_matches(topic, topic_len, s_subscriptions[i].topic) &&
            snapshot_count < MQTT_MAX_SUBSCRIPTIONS) {
            strncpy(snapshot[snapshot_count].topic, s_subscriptions[i].topic,
                    sizeof(snapshot[snapshot_count].topic) - 1);
            snapshot[snapshot_count].topic[sizeof(snapshot[snapshot_count].topic) - 1] = '\0';
            snapshot[snapshot_count].cb       = s_subscriptions[i].cb;
            snapshot[snapshot_count].user_ctx = s_subscriptions[i].user_ctx;
            snapshot_count++;
        }
    }

    subscriptions_lock_give();

    for (size_t i = 0; i < snapshot_count; i++) {
        if (snapshot[i].cb != NULL) {
            snapshot[i].cb(snapshot[i].topic, payload, payload_len, snapshot[i].user_ctx);
        }
    }
}

/* ── MQTT event handler ──────────────────────────────────────────────────── */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        xEventGroupSetBits(s_event_group, MQTT_CONNECTED_BIT);

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        xEventGroupClearBits(s_event_group, MQTT_CONNECTED_BIT);
        break;

    case MQTT_EVENT_ERROR:
        xEventGroupClearBits(s_event_group, MQTT_CONNECTED_BIT);
        if (event->error_handle != NULL) {
            ESP_LOGE(TAG,
                     "MQTT event error: type=%d, tls=0x%x, stack=0x%x, sock_errno=%d",
                     event->error_handle->error_type,
                     event->error_handle->esp_tls_last_esp_err,
                     event->error_handle->esp_tls_stack_err,
                     event->error_handle->esp_transport_sock_errno);
        } else {
            ESP_LOGE(TAG, "MQTT event error without details");
        }
        break;

    case MQTT_EVENT_DATA: {
        if (event->topic_len == 0) break; /* fragmented topic header — skip */

        /* ── setup success topic ── */
        size_t expected_len = strlen(s_success_topic);
        if (expected_len > 0 &&
            event->topic_len == (int)expected_len &&
            strncmp(event->topic, s_success_topic, event->topic_len) == 0) {
            ESP_LOGI(TAG, "Received success on: %s", s_success_topic);
            xEventGroupSetBits(s_event_group, MQTT_SETUP_SUCCESS_BIT);
        }

        /* ── normal subscriber dispatch ── */
        dispatch_subscribed_message(event->topic, event->topic_len,
                                    event->data, event->data_len);
        break;
    }

    default:
        break;
    }
}

/* ── init / deinit ───────────────────────────────────────────────────────── */
esp_err_t mqtt_manager_init(const char *broker_ip, int broker_port)
{
    if (broker_ip == NULL || strlen(broker_ip) == 0 ||
        broker_port <= 0 || broker_port > 65535) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_event_group == NULL) {
        s_event_group = xEventGroupCreate();
        if (s_event_group == NULL) return ESP_ERR_NO_MEM;
    }

    if (s_subscriptions_lock == NULL) {
        s_subscriptions_lock = xSemaphoreCreateMutex();
        if (s_subscriptions_lock == NULL) return ESP_ERR_NO_MEM;
    }

    if (s_client != NULL) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }

    if (!subscriptions_lock_take("mqtt_manager_init")) return ESP_ERR_TIMEOUT;
    memset(s_subscriptions, 0, sizeof(s_subscriptions));
    subscriptions_lock_give();

    s_subscriptions_lock_timeout_count = 0;
    s_success_topic[0]    = '\0';

    char broker_uri[128];
    int written = snprintf(broker_uri, sizeof(broker_uri),
                           "mqtt://%s:%d", broker_ip, broker_port);
    if (written <= 0 || written >= (int)sizeof(broker_uri)) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri            = broker_uri,
        .network.timeout_ms            = 10000,
        .network.reconnect_timeout_ms  = 5000,
        .network.disable_auto_reconnect = false,
        .session.keepalive             = 30,
    };

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_client == NULL) return ESP_FAIL;

    esp_err_t ret = esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                                   mqtt_event_handler, NULL);
    if (ret != ESP_OK) return ret;

    ret = esp_mqtt_client_start(s_client);
    if (ret != ESP_OK) return ret;

    EventBits_t bits = xEventGroupWaitBits(s_event_group, MQTT_CONNECTED_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(10000));
    if ((bits & MQTT_CONNECTED_BIT) == 0) {
        ESP_LOGE(TAG, "MQTT connect timeout");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t mqtt_manager_deinit(void)
{
    esp_err_t status = ESP_OK;

    if (s_client != NULL) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }

    if (s_subscriptions_lock != NULL) {
        if (!subscriptions_lock_take("mqtt_manager_deinit")) {
            status = ESP_ERR_TIMEOUT;
        } else {
            memset(s_subscriptions, 0, sizeof(s_subscriptions));
            subscriptions_lock_give();
        }
    } else {
        memset(s_subscriptions, 0, sizeof(s_subscriptions));
    }

    s_success_topic[0]    = '\0';

    if (s_event_group != NULL) {
        xEventGroupClearBits(s_event_group, MQTT_CONNECTED_BIT | MQTT_SETUP_SUCCESS_BIT);
    }

    ESP_LOGI(TAG, "MQTT manager deinitialized");
    return status;
}

/* ── remaining public API (unchanged) ───────────────────────────────────── */

esp_err_t mqtt_manager_publish_setup_and_wait(const char *zone_id,
                                               const char *payload,
                                               int timeout_ms)
{
    if (s_client == NULL || zone_id == NULL || strlen(zone_id) == 0 ||
        payload == NULL || strlen(payload) == 0 || timeout_ms <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int written = snprintf(s_success_topic, sizeof(s_success_topic),
                           "%s/success", zone_id);
    if (written <= 0 || written >= (int)sizeof(s_success_topic)) {
        return ESP_ERR_INVALID_SIZE;
    }

    int sub_id = esp_mqtt_client_subscribe(s_client, s_success_topic,
                                           MQTT_DEFAULT_SUB_QOS);
    if (sub_id < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to success topic: %s", s_success_topic);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Subscribed to topic: %s", s_success_topic);

    xEventGroupClearBits(s_event_group, MQTT_SETUP_SUCCESS_BIT);

    int msg_id = esp_mqtt_client_publish(s_client, "SetUp", payload, 0,
                                         MQTT_DEFAULT_SUB_QOS, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish SetUp payload");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Published SetUp message id=%d", msg_id);

    EventBits_t bits = xEventGroupWaitBits(s_event_group,
                                           MQTT_SETUP_SUCCESS_BIT,
                                           pdTRUE, pdTRUE,
                                           pdMS_TO_TICKS(timeout_ms));
    if ((bits & MQTT_SETUP_SUCCESS_BIT) == 0) {
        ESP_LOGE(TAG, "Timeout waiting for %s", s_success_topic);
        return ESP_ERR_TIMEOUT;
    }

    esp_mqtt_client_unsubscribe(s_client, s_success_topic);
    ESP_LOGI(TAG, "%s confirmed", s_success_topic);
    return ESP_OK;
}

esp_err_t mqtt_manager_publish(const char *topic, const char *payload,
                                int qos, int retain)
{
    if (s_client == NULL || topic == NULL || strlen(topic) == 0 ||
        payload == NULL || qos < 0 || qos > 2 ||
        (retain != 0 && retain != 1)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!mqtt_manager_is_connected()) return ESP_ERR_INVALID_STATE;

    int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
    return (msg_id < 0) ? ESP_FAIL : ESP_OK;
}

esp_err_t mqtt_manager_subscribe(const char *topic, int qos,
                                  mqtt_manager_message_cb_t cb, void *user_ctx)
{
    if (s_client == NULL || topic == NULL || strlen(topic) == 0 ||
        cb == NULL || qos < 0 || qos > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!subscriptions_lock_take("mqtt_manager_subscribe")) return ESP_ERR_TIMEOUT;

    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++) {
        if (s_subscriptions[i].in_use &&
            strcmp(s_subscriptions[i].topic, topic) == 0) {
            s_subscriptions[i].cb       = cb;
            s_subscriptions[i].user_ctx = user_ctx;
            subscriptions_lock_give();
            return ESP_OK;
        }
    }

    subscriptions_lock_give();

    int sub_id = esp_mqtt_client_subscribe(s_client, topic, qos);
    if (sub_id < 0) return ESP_FAIL;

    if (!subscriptions_lock_take("mqtt_manager_subscribe_store")) {
        esp_mqtt_client_unsubscribe(s_client, topic);
        return ESP_ERR_TIMEOUT;
    }

    /* Double-check after re-acquiring lock */
    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++) {
        if (s_subscriptions[i].in_use &&
            strcmp(s_subscriptions[i].topic, topic) == 0) {
            s_subscriptions[i].cb       = cb;
            s_subscriptions[i].user_ctx = user_ctx;
            subscriptions_lock_give();
            return ESP_OK;
        }
    }

    int free_index = -1;
    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++) {
        if (!s_subscriptions[i].in_use) { free_index = i; break; }
    }

    if (free_index < 0) {
        subscriptions_lock_give();
        esp_mqtt_client_unsubscribe(s_client, topic);
        return ESP_ERR_NO_MEM;
    }

    s_subscriptions[free_index].in_use   = true;
    s_subscriptions[free_index].cb       = cb;
    s_subscriptions[free_index].user_ctx = user_ctx;
    strncpy(s_subscriptions[free_index].topic, topic,
            sizeof(s_subscriptions[free_index].topic) - 1);
    s_subscriptions[free_index].topic[sizeof(s_subscriptions[free_index].topic) - 1] = '\0';
    subscriptions_lock_give();

    ESP_LOGI(TAG, "Subscribed: %s", topic);
    return ESP_OK;
}

esp_err_t mqtt_manager_unsubscribe(const char *topic)
{
    if (s_client == NULL || topic == NULL || strlen(topic) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!subscriptions_lock_take("mqtt_manager_unsubscribe")) return ESP_ERR_TIMEOUT;

    bool found = false;
    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++) {
        if (s_subscriptions[i].in_use &&
            strcmp(s_subscriptions[i].topic, topic) == 0) {
            memset(&s_subscriptions[i], 0, sizeof(s_subscriptions[i]));
            found = true;
            break;
        }
    }

    subscriptions_lock_give();

    if (!found) return ESP_ERR_NOT_FOUND;

    int unsub_id = esp_mqtt_client_unsubscribe(s_client, topic);
    if (unsub_id < 0) {
        ESP_LOGW(TAG, "Broker unsubscribe failed for topic: %s", topic);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Unsubscribed: %s", topic);
    return ESP_OK;
}

uint32_t mqtt_manager_get_lock_timeout_count(void)
{
    return s_subscriptions_lock_timeout_count;
}