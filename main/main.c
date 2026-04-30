#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#define AP_SSID "ESP32S3-Setup"
#define AP_PASS "12345678"
#define AP_CHANNEL 1
#define AP_MAX_CONN 4

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_STA_RETRY      10

static const char *TAG = "wifi_portal";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_sta_has_credentials = false;
static int s_zone_id = 1;
static bool s_zone_published = false;
static httpd_handle_t s_server = NULL;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

static const char *INDEX_HTML =
	"<!doctype html><html><head><meta charset='utf-8'>"
	"<meta name='viewport' content='width=device-width,initial-scale=1'>"
	"<title>ESP32 Wi-Fi Setup</title>"
	"<style>"
	"body{font-family:Arial,sans-serif;background:#f6f8fb;color:#1a1a1a;padding:24px;}"
	".card{max-width:420px;margin:32px auto;background:#fff;border-radius:12px;"
	"box-shadow:0 6px 24px rgba(0,0,0,.12);padding:20px;}"
	"h1{font-size:1.2rem;margin:0 0 12px;}"
	"label{display:block;margin:10px 0 6px;font-weight:600;}"
	"input{width:100%;padding:10px;border:1px solid #c8ced8;border-radius:8px;}"
	"button{margin-top:14px;width:100%;padding:10px;border:none;border-radius:8px;"
	"background:#0066cc;color:white;font-weight:700;cursor:pointer;}"
	"p{font-size:.92rem;color:#444;}"
	"</style></head><body><div class='card'>"
	"<h1>ESP32-S3 Wi-Fi Setup</h1>"
	"<p>Connect this device to your home Wi-Fi.</p>"
	"<form method='POST' action='/connect'>"
	"<label>Wi-Fi SSID</label><input name='ssid' required maxlength='32'>"
	"<label>Zone ID</label><input name='zone_id' type='number' min='1' value='1' required>"
	"<label>Password</label><input name='password' type='password' maxlength='64'>"
	"<p>Leave password blank for open Wi-Fi networks.</p>"
	"<button type='submit'>Connect</button></form></div></body></html>";

static void mqtt_event_handler(void *handler_args,
							   esp_event_base_t base,
							   int32_t event_id,
							   void *event_data)
{
	(void)handler_args;
	(void)base;
	(void)event_data;
	if (event_id == MQTT_EVENT_CONNECTED && !s_zone_published) {
		char payload[64] = {0};
		snprintf(payload, sizeof(payload), "{\"ZoneID\":%d}", s_zone_id);
		int msg_id = esp_mqtt_client_publish(s_mqtt_client, "SetUp", payload, 0, 1, 0);
		if (msg_id >= 0) {
			s_zone_published = true;
			ESP_LOGI(TAG, "MQTT published to topic SetUp: %s", payload);
		} else {
			ESP_LOGE(TAG, "MQTT publish failed");
		}
	}
}

static void mqtt_publish_zone_id(void)
{
	if (s_zone_published) {
		return;
	}

	if (s_mqtt_client == NULL) {
		const esp_mqtt_client_config_t mqtt_cfg = {
			.broker.address.uri = "mqtt://172.20.10.3:1883",
		};
		s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
		esp_mqtt_client_register_event(s_mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
		esp_mqtt_client_start(s_mqtt_client);
		ESP_LOGI(TAG, "MQTT client started: mqtt://172.20.10.3:1883");
	}
}

static void wifi_event_handler(void *arg,
							   esp_event_base_t event_base,
							   int32_t event_id,
							   void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		if (s_sta_has_credentials) {
			esp_wifi_connect();
		}
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (!s_sta_has_credentials) {
			return;
		}
		if (s_retry_num < MAX_STA_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGW(TAG, "Retrying STA connection (%d/%d)", s_retry_num, MAX_STA_RETRY);
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
			ESP_LOGE(TAG, "STA failed to connect");
		}
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

		if (s_server != NULL) {
			httpd_stop(s_server);
			s_server = NULL;
			ESP_LOGI(TAG, "Web server stopped after STA connection");
		}

		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_LOGI(TAG, "SoftAP disabled, running STA only");

		mqtt_publish_zone_id();
	}
}

static void url_decode(char *dst, const char *src)
{
	char a, b;
	while (*src) {
		if ((*src == '%') && ((a = src[1]) && (b = src[2])) &&
			(((a >= '0' && a <= '9') || (a >= 'a' && a <= 'f') || (a >= 'A' && a <= 'F')) &&
			 ((b >= '0' && b <= '9') || (b >= 'a' && b <= 'f') || (b >= 'A' && b <= 'F')))) {
			if (a >= 'a') {
				a -= 'a' - 'A';
			}
			if (a >= 'A') {
				a -= ('A' - 10);
			} else {
				a -= '0';
			}
			if (b >= 'a') {
				b -= 'a' - 'A';
			}
			if (b >= 'A') {
				b -= ('A' - 10);
			} else {
				b -= '0';
			}
			*dst++ = (char)(16 * a + b);
			src += 3;
		} else if (*src == '+') {
			*dst++ = ' ';
			src++;
		} else {
			*dst++ = *src++;
		}
	}
	*dst = '\0';
}

static void parse_form_value(const char *body, const char *key, char *out, size_t out_len)
{
	char pattern[20] = {0};
	snprintf(pattern, sizeof(pattern), "%s=", key);
	const char *start = strstr(body, pattern);
	if (!start) {
		out[0] = '\0';
		return;
	}

	start += strlen(pattern);
	const char *end = strchr(start, '&');
	size_t raw_len = end ? (size_t)(end - start) : strlen(start);
	if (raw_len >= out_len) {
		raw_len = out_len - 1;
	}

	char tmp[128] = {0};
	if (raw_len >= sizeof(tmp)) {
		raw_len = sizeof(tmp) - 1;
	}
	memcpy(tmp, start, raw_len);
	tmp[raw_len] = '\0';
	url_decode(out, tmp);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static void wifi_connect_with_credentials(const char *ssid, const char *password)
{
	wifi_config_t sta_cfg = {0};
	strlcpy((char *)sta_cfg.sta.ssid, ssid, sizeof(sta_cfg.sta.ssid));
	strlcpy((char *)sta_cfg.sta.password, password, sizeof(sta_cfg.sta.password));
	sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
	sta_cfg.sta.pmf_cfg.capable = true;
	sta_cfg.sta.pmf_cfg.required = false;

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
	s_sta_has_credentials = true;
	s_retry_num = 0;
	xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
	esp_wifi_disconnect();
	ESP_ERROR_CHECK(esp_wifi_connect());
	ESP_LOGI(TAG, "Trying to connect STA to SSID: %s", ssid);
}

static esp_err_t connect_post_handler(httpd_req_t *req)
{
	if (req->content_len <= 0 || req->content_len > 256) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
	}

	char body[257] = {0};
	int total_received = 0;
	while (total_received < req->content_len) {
		int received = httpd_req_recv(req, body + total_received, req->content_len - total_received);
		if (received <= 0) {
			return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read body");
		}
		total_received += received;
	}
	body[total_received] = '\0';

	char ssid[33] = {0};
	char password[65] = {0};
	char zone_id_str[12] = {0};
	parse_form_value(body, "ssid", ssid, sizeof(ssid));
	parse_form_value(body, "zone_id", zone_id_str, sizeof(zone_id_str));
	parse_form_value(body, "password", password, sizeof(password));

	if (strlen(ssid) == 0) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
	}

	char *endptr = NULL;
	long zone = strtol(zone_id_str, &endptr, 10);
	if (zone_id_str[0] == '\0' || endptr == zone_id_str || *endptr != '\0' || zone <= 0) {
		return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Valid zone_id required");
	}
	s_zone_id = (int)zone;
	s_zone_published = false;

	wifi_connect_with_credentials(ssid, password);

	const char *resp =
		"<html><body><h2>Connecting...</h2>"
		"<p>ESP32-S3 is trying to connect to your Wi-Fi network.</p>"
		"<p>Check serial logs for status.</p>"
		"</body></html>";
	httpd_resp_set_type(req, "text/html");
	return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_webserver(void)
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.server_port = 80;

	httpd_handle_t server = NULL;
	if (httpd_start(&server, &config) == ESP_OK) {
		httpd_uri_t root_uri = {
			.uri = "/",
			.method = HTTP_GET,
			.handler = root_get_handler,
			.user_ctx = NULL};
		httpd_register_uri_handler(server, &root_uri);

		httpd_uri_t connect_uri = {
			.uri = "/connect",
			.method = HTTP_POST,
			.handler = connect_post_handler,
			.user_ctx = NULL};
		httpd_register_uri_handler(server, &connect_uri);

		ESP_LOGI(TAG, "Web server started");
		s_server = server;
		return server;
	}

	ESP_LOGE(TAG, "Error starting web server");
	return NULL;
}

static void wifi_init_apsta(void)
{
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(
		IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

	wifi_config_t ap_config = {
		.ap = {
			.channel = AP_CHANNEL,
			.max_connection = AP_MAX_CONN,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK,
		},
	};

	strlcpy((char *)ap_config.ap.ssid, AP_SSID, sizeof(ap_config.ap.ssid));
	strlcpy((char *)ap_config.ap.password, AP_PASS, sizeof(ap_config.ap.password));
	ap_config.ap.ssid_len = strlen(AP_SSID);

	if (strlen(AP_PASS) < 8) {
		ap_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "SoftAP started: SSID=%s PASS=%s", AP_SSID, AP_PASS);
	ESP_LOGI(TAG, "Open http://192.168.4.1 in your browser");
}

void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	wifi_init_apsta();
	start_webserver();
}
