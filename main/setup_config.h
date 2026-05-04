#ifndef SETUP_CONFIG_H
#define SETUP_CONFIG_H

#include <stdbool.h>

#include "esp_err.h"

#define SETUP_WIFI_SSID_MAX_LEN      32
#define SETUP_WIFI_PASSWORD_MAX_LEN  63
#define SETUP_BROKER_IP_MAX_LEN      63
#define SETUP_OTA_BROKER_IP_MAX_LEN  63

typedef struct {
    char ssid[SETUP_WIFI_SSID_MAX_LEN + 1];
    char password[SETUP_WIFI_PASSWORD_MAX_LEN + 1];
    char broker_ip[SETUP_BROKER_IP_MAX_LEN + 1];
    int broker_port;
    char ota_broker_ip[SETUP_OTA_BROKER_IP_MAX_LEN + 1];
} setup_config_t;

esp_err_t setup_config_init(void);
esp_err_t setup_config_load(setup_config_t *config, bool *found);
esp_err_t setup_config_save(const setup_config_t *config);

#endif