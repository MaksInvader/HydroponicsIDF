#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ota_update.h"

static const char *TAG = "ota_update";

#define OTA_HTTP_TASK_STACK      8192
#define OTA_HTTP_TASK_PRIORITY   5
#define OTA_HTTP_BUFFER_SIZE     1024
#define OTA_HTTP_TIMEOUT_MS      10000
#define OTA_MAX_TOTAL_SIZE       (4 * 1024 * 1024)
#define OTA_URL_MAX_LEN          256

typedef struct {
    atomic_bool in_progress;
    char url[OTA_URL_MAX_LEN];
} ota_http_state_t;

static ota_http_state_t s_ota = {
    .in_progress = false,
    .url = {0},
};

static esp_err_t ota_http_download(const char *url)
{
    if (url == NULL || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
        .buffer_size = OTA_HTTP_BUFFER_SIZE,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int64_t content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP status %d", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    if (content_length > 0 && content_length > OTA_MAX_TOTAL_SIZE) {
        ESP_LOGE(TAG, "Firmware too large: %lld bytes", (long long)content_length);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_INVALID_SIZE;
    }

    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition available");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_ota_handle_t update_handle = 0;
    bool ota_started = false;

    err = esp_ota_begin(partition,
                        (content_length > 0) ? (size_t)content_length : OTA_SIZE_UNKNOWN,
                        &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return err;
    }
    ota_started = true;

    uint8_t buffer[OTA_HTTP_BUFFER_SIZE];
    size_t total = 0;

    while (true) {
        int read = esp_http_client_read(client, (char *)buffer, sizeof(buffer));
        if (read < 0) {
            ESP_LOGE(TAG, "HTTP read failed");
            err = ESP_FAIL;
            break;
        }
        if (read == 0) {
            err = ESP_OK;
            break;
        }

        if (total + (size_t)read > OTA_MAX_TOTAL_SIZE) {
            ESP_LOGE(TAG, "Firmware exceeds max size");
            err = ESP_ERR_INVALID_SIZE;
            break;
        }

        err = esp_ota_write(update_handle, buffer, read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            break;
        }
        total += (size_t)read;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        if (ota_started) {
            esp_ota_abort(update_handle);
        }
        return err;
    }

    if (total == 0) {
        if (ota_started) {
            esp_ota_abort(update_handle);
        }
        ESP_LOGE(TAG, "No data received from HTTP server");
        return ESP_FAIL;
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ota_set_boot_partition(partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "OTA complete: %zu bytes", total);
    return ESP_OK;
}

static void ota_http_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "HTTP OTA start: %s", s_ota.url);
    esp_err_t err = ota_http_download(s_ota.url);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP OTA success. Rebooting...");
        atomic_store(&s_ota.in_progress, false);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "HTTP OTA failed: %s", esp_err_to_name(err));
        atomic_store(&s_ota.in_progress, false);
    }

    vTaskDelete(NULL);
}

esp_err_t ota_update_http_start(const char *url)
{
    if (url == NULL || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    if (atomic_load(&s_ota.in_progress)) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t url_len = strlen(url);
    if (url_len >= sizeof(s_ota.url)) {
        return ESP_ERR_INVALID_SIZE;
    }

    strncpy(s_ota.url, url, sizeof(s_ota.url) - 1);
    s_ota.url[sizeof(s_ota.url) - 1] = '\0';
    atomic_store(&s_ota.in_progress, true);

    BaseType_t ok = xTaskCreate(ota_http_task,
                                "OtaHttp",
                                OTA_HTTP_TASK_STACK,
                                NULL,
                                OTA_HTTP_TASK_PRIORITY,
                                NULL);
    if (ok != pdPASS) {
        atomic_store(&s_ota.in_progress, false);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool ota_update_http_is_busy(void)
{
    return atomic_load(&s_ota.in_progress);
}
