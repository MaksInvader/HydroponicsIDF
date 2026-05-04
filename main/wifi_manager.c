#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "wifi_manager.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define MAXIMUM_RETRY 10

static const char *TAG = "wifi_manager";

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;
static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;
static bool s_connect_sequence_active;
static bool s_sta_auto_reconnect_enabled;
static bool s_event_handlers_registered;
static esp_event_handler_instance_t s_instance_any_id;
static esp_event_handler_instance_t s_instance_got_ip;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (s_connect_sequence_active || s_sta_auto_reconnect_enabled) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_event_group != NULL) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }

        if (s_connect_sequence_active && s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to AP (%d/%d)", s_retry_num, MAXIMUM_RETRY);
        } else if (s_connect_sequence_active) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        } else if (s_sta_auto_reconnect_enabled) {
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Auto-reconnect request failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGW(TAG, "STA disconnected, requesting reconnect");
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
        s_sta_auto_reconnect_enabled = true;
    }
}

static esp_err_t ensure_wifi_event_handlers_registered(void)
{
    if (s_event_handlers_registered) {
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &s_instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        &s_instance_got_ip));

    s_event_handlers_registered = true;
    return ESP_OK;
}

void wifi_manager_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (s_sta_netif == NULL) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
    }
    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        ESP_ERROR_CHECK(s_wifi_event_group != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    }

    ESP_ERROR_CHECK(ensure_wifi_event_handlers_registered());
}

esp_err_t wifi_manager_start_ap(const char *ap_ssid, const char *ap_password)
{
    wifi_config_t wifi_config = {0};

    size_t ssid_len = strlen(ap_ssid);
    if (ssid_len > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(wifi_config.ap.ssid, ap_ssid, ssid_len);
    wifi_config.ap.ssid_len = (uint8_t)ssid_len;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.channel = 1;

    if (ap_password != NULL && strlen(ap_password) >= 8) {
        strncpy((char *)wifi_config.ap.password, ap_password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started. SSID: %s", ap_ssid);
    return ESP_OK;
}

esp_err_t wifi_manager_connect_sta(const char *ssid, const char *password, int timeout_ms)
{
    wifi_ap_record_t current_ap;
    wifi_config_t wifi_config = {0};

    if (ssid == NULL || strlen(ssid) == 0 || strlen(ssid) > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);

    if (password != NULL && strlen(password) > 0) {
        if (strlen(password) > 63) {
            return ESP_ERR_INVALID_ARG;
        }
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }

    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t reg_ret = ensure_wifi_event_handlers_registered();
    if (reg_ret != ESP_OK) {
        return reg_ret;
    }

    s_retry_num = 0;
    s_connect_sequence_active = false;
    s_sta_auto_reconnect_enabled = false;

    if (esp_wifi_sta_get_ap_info(&current_ap) == ESP_OK) {
        ESP_LOGI(TAG, "STA already connected. Disconnecting before switching AP");
        ESP_ERROR_CHECK(esp_wifi_disconnect());
    }

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    s_connect_sequence_active = true;
    ESP_ERROR_CHECK(esp_wifi_connect());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_ms));

    s_connect_sequence_active = false;

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to target AP");
        s_sta_auto_reconnect_enabled = true;
        return ESP_OK;
    }

    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to target AP");
        s_sta_auto_reconnect_enabled = false;
        return ESP_FAIL;
    }

    ESP_LOGE(TAG, "Timeout while waiting for AP connection");
    s_sta_auto_reconnect_enabled = false;
    return ESP_ERR_TIMEOUT;
}

bool wifi_manager_is_sta_connected(void)
{
    if (s_wifi_event_group == NULL) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}