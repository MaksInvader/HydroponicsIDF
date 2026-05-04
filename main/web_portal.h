#ifndef WEB_PORTAL_H
#define WEB_PORTAL_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t web_portal_start(void);
esp_err_t web_portal_try_autostart_from_nvs(bool *started);

#endif