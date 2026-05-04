#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

typedef void (*mqtt_manager_message_cb_t)(const char *topic, const char *payload, int payload_len, void *user_ctx);

esp_err_t mqtt_manager_init(const char *broker_ip, int broker_port);
esp_err_t mqtt_manager_deinit(void);
esp_err_t mqtt_manager_publish_setup_and_wait(const char *zone_id, const char *payload, int timeout_ms);
esp_err_t mqtt_manager_publish(const char *topic, const char *payload, int qos, int retain);
esp_err_t mqtt_manager_subscribe(const char *topic, int qos, mqtt_manager_message_cb_t cb, void *user_ctx);
esp_err_t mqtt_manager_unsubscribe(const char *topic);
uint32_t mqtt_manager_get_lock_timeout_count(void);
bool mqtt_manager_is_connected(void);


#endif