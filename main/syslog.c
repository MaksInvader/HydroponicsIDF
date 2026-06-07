#include "syslog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "syslog";

static int s_sock = -1;
static struct sockaddr_in s_dest_addr;
static int (*s_prev_vprintf)(const char *fmt, va_list ap) = NULL;

static int syslog_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    va_list ap_copy;
    va_copy(ap_copy, ap);

    int len = vsnprintf(buf, sizeof(buf), fmt, ap_copy);
    va_end(ap_copy);

    if (s_prev_vprintf != NULL) {
        s_prev_vprintf(fmt, ap);
    }

    if (s_sock >= 0 && len > 0) {
        if (len >= (int)sizeof(buf)) {
            len = (int)sizeof(buf) - 1;
        }
        sendto(s_sock, buf, len, 0, (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
    }

    return len;
}

esp_err_t syslog_init(const char *host, uint16_t port)
{
    if (host == NULL || host[0] == '\0') {
        ESP_LOGE(TAG, "syslog_init: host is NULL or empty");
        return ESP_ERR_INVALID_ARG;
    }

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return ESP_FAIL;
    }

    memset(&s_dest_addr, 0, sizeof(s_dest_addr));
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port = htons(port);
    if (inet_aton(host, &s_dest_addr.sin_addr) == 0) {
        ESP_LOGE(TAG, "syslog_init: invalid host address '%s'", host);
        close(s_sock);
        s_sock = -1;
        return ESP_ERR_INVALID_ARG;
    }

    s_prev_vprintf = esp_log_set_vprintf(syslog_vprintf);
    ESP_LOGI(TAG, "Syslog UDP unicast enabled → %s:%u", host, (unsigned)port);

    return ESP_OK;
}
