#ifndef SYSLOG_H
#define SYSLOG_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t syslog_init(const char *host, uint16_t port);

#endif /* SYSLOG_H */
