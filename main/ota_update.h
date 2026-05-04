#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * ota_update_http_start()
 *
 * Start an HTTP OTA download in a background task.
 * The URL should be a full http://... path to the firmware .bin file.
 *
 * Returns ESP_OK if the OTA task is started.
 * Returns ESP_ERR_INVALID_STATE if an OTA is already running.
 */
esp_err_t ota_update_http_start(const char *url);

/* Returns true while an HTTP OTA is in progress. */
bool ota_update_http_is_busy(void);
