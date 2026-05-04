#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

void wifi_manager_init(void);
esp_err_t wifi_manager_start_ap(const char *ap_ssid, const char *ap_password);
esp_err_t wifi_manager_connect_sta(const char *ssid, const char *password, int timeout_ms);
bool wifi_manager_is_sta_connected(void);

#endif