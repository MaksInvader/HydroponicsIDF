#ifndef ZONE_CONFIG_H
#define ZONE_CONFIG_H

#include <stdbool.h>

#include "esp_err.h"

#define ZONE_ID_MAX_LEN   32
#define ZONE_NAME_MAX_LEN 64

typedef struct {
    char zone_id[ZONE_ID_MAX_LEN + 1];
    char zone_name[ZONE_NAME_MAX_LEN + 1];
} zone_config_t;

esp_err_t zone_config_init(void);
esp_err_t zone_config_load(zone_config_t *config, bool *found);
esp_err_t zone_config_save(const zone_config_t *config);

#endif