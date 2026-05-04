#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "actuator_control.h"
#include "lcd_status.h"
#include "mqtt_manager.h"
#include "runtime_tasks.h"
#include "sensor_telemetry.h"
#include "setup_config.h"
#include "web_portal.h"
#include "wifi_manager.h"
#include "zone_config.h"

static const char *TAG = "web_portal";

static const char *INDEX_HTML_HEAD =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32S3 Updater</title>"
    "<style>body{font-family:Arial,sans-serif;background:#f3f4f6;padding:20px;}"
    ".card{max-width:560px;margin:auto;background:#fff;padding:20px;border-radius:12px;box-shadow:0 8px 24px rgba(0,0,0,.08);}"
    "label{display:block;margin-top:12px;font-weight:600;}input{width:100%;padding:10px;margin-top:6px;border:1px solid #cbd5e1;border-radius:8px;}"
    ".hint{margin-top:8px;padding:10px;background:#e2e8f0;border-radius:8px;color:#0f172a;font-size:14px;}"
    "button{margin-top:16px;width:100%;padding:11px;border:0;background:#0f766e;color:#fff;border-radius:8px;font-weight:700;}"
    "small{display:block;margin-top:8px;color:#475569;}</style></head><body><div class='card'>"
    "<h2>ESP32S3 Wi-Fi + OTA Setup</h2>"
    "<div class='hint'>Current Zone: ";

static const char *INDEX_HTML_TAIL =
    "</div>"
    "<form method='POST' action='/configure'>"
    "<label>Wi-Fi SSID</label><input name='ssid' required maxlength='32'>"
    "<label>Wi-Fi Password (optional)</label><input name='password' type='password' maxlength='63'>"
    "<label>New Zone ID (optional, reassign device)</label><input name='zone_id' maxlength='32' placeholder='zone_N'>"
    "<label>New Zone Name (optional, reassign device)</label><input name='zone_name' maxlength='64' placeholder='Lorong Zone N'>"
    "<label>MQTT Broker IP</label><input name='broker_ip' required placeholder='192.168.1.20'>"
    "<label>MQTT Broker Port</label><input name='broker_port' type='number' min='1' max='65535' value='1883'>"
    "<button type='submit'>Apply / Reassign Zone and Run Setup</button></form>"
    "<small><a href='/debug'>Open Runtime Debug Page</a></small>"
    "<small>Leave New Zone fields empty to keep the current zone.</small>"
    "<small>OTA firmware is served by the MQTT broker host.</small>"
    "<small>Device AP: ESP32S3-Updater (open network by default)</small>"
    "</div></body></html>";

static const char *DEBUG_HTML_HEAD =
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32S3 Debug</title>"
    "<style>body{font-family:Arial,sans-serif;background:#eef2f7;padding:20px;}"
    ".card{max-width:640px;margin:auto;background:#fff;padding:20px;border-radius:12px;box-shadow:0 8px 24px rgba(0,0,0,.08);}"
    "h3{margin-top:14px;}pre{background:#0f172a;color:#e2e8f0;padding:12px;border-radius:8px;overflow:auto;}a{color:#0f766e;}"
    "</style></head><body><div class='card'><h2>Runtime Debug</h2>"
    "<p><b>Current Zone:</b> <span id='zone'>loading...</span></p>"
    "<h3>All Topics</h3><pre id='topics'>loading...</pre>"
    "<h3>Last Commands (All Command Topics)</h3><pre id='lastcmds'>loading...</pre>"
    "<h3>Last Sensor Publish</h3><pre id='sensor'>loading...</pre>"
    "<h3>MQTT Health</h3><pre id='mqtthealth'>loading...</pre>";

static const char *DEBUG_HTML_TAIL =
    "<p><a href='/'>Back to Setup</a></p>"
    "<script>"
    "async function refreshDebug(){"
    "try{"
    "const r=await fetch('/debug/data',{cache:'no-store'});"
    "if(!r.ok){throw new Error('HTTP '+r.status);}"
    "const d=await r.json();"
    "document.getElementById('zone').textContent=d.zone;"
    "document.getElementById('topics').textContent=d.topics.join('\\n');"
    "document.getElementById('lastcmds').textContent=d.lastCommands.map(c=>"
    "'Topic: '+c.topic+'\\n'+"
    "'Channel: '+c.channel+'\\n'+"
    "'Command: '+c.command+'\\n'+"
    "'State: '+c.state"
    ").join('\\n\\n');"
    "document.getElementById('sensor').textContent="
    "'WaterLevel: '+d.sensor.waterLevel+'\\n'+"
    "'WaterTemp: '+d.sensor.waterTemp+'\\n'+"
    "'pH: '+d.sensor.ph+'\\n'+"
    "'TDS: '+d.sensor.tds+'\\n'+"
    "'PublishCount: '+d.sensor.publishCount;"
    "document.getElementById('mqtthealth').textContent="
    "'SubscriptionsLockTimeoutCount: '+d.mqttHealth.subscriptionsLockTimeoutCount;"
    "}catch(e){"
    "document.getElementById('topics').textContent='Failed to fetch debug data';"
    "}"
    "}"
    "refreshDebug();setInterval(refreshDebug,1000);"
    "</script></div></body></html>";

static char from_hex(char c)
{
    if (c >= '0' && c <= '9') {
        return (char)(c - '0');
    }
    if (c >= 'a' && c <= 'f') {
        return (char)(10 + c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return (char)(10 + c - 'A');
    }
    return 0;
}

static void url_decode(char *dst, const char *src, size_t dst_len)
{
    size_t src_len = strlen(src);
    size_t di = 0;
    for (size_t si = 0; si < src_len && di + 1 < dst_len; si++) {
        if (src[si] == '+') {
            dst[di++] = ' ';
            continue;
        }
        if (src[si] == '%' && si + 2 < src_len && isxdigit((unsigned char)src[si + 1]) && isxdigit((unsigned char)src[si + 2])) {
            dst[di++] = (char)((from_hex(src[si + 1]) << 4) | from_hex(src[si + 2]));
            si += 2;
            continue;
        }
        dst[di++] = src[si];
    }
    dst[di] = '\0';
}

static void get_form_value(const char *body, const char *key, char *out, size_t out_len)
{
    out[0] = '\0';

    const char *start = strstr(body, key);
    if (start == NULL) {
        return;
    }

    start += strlen(key);
    const char *end = strchr(start, '&');
    if (end == NULL) {
        end = start + strlen(start);
    }

    size_t raw_len = (size_t)(end - start);
    if (raw_len == 0) {
        return;
    }

    char raw[128];
    if (raw_len >= sizeof(raw)) {
        raw_len = sizeof(raw) - 1;
    }
    memcpy(raw, start, raw_len);
    raw[raw_len] = '\0';

    url_decode(out, raw, out_len);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    zone_config_t zone_cfg;
    bool zone_found = false;
    const char *zone_text = "Not assigned";
    char zone_buf[96];

    esp_err_t zone_ret = zone_config_load(&zone_cfg, &zone_found);
    if (zone_ret == ESP_OK && zone_found) {
        snprintf(zone_buf, sizeof(zone_buf), "%.32s (%.40s)", zone_cfg.zone_id, zone_cfg.zone_name);
        zone_text = zone_buf;
    }

    httpd_resp_set_type(req, "text/html");

    esp_err_t ret = httpd_resp_sendstr_chunk(req, INDEX_HTML_HEAD);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = httpd_resp_sendstr_chunk(req, zone_text);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = httpd_resp_sendstr_chunk(req, INDEX_HTML_TAIL);
    if (ret != ESP_OK) {
        return ret;
    }

    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t configure_post_handler(httpd_req_t *req)
{
    char *body = NULL;
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 1536) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Invalid request body", HTTPD_RESP_USE_STRLEN);
    }

    body = (char *)malloc((size_t)total_len + 1);
    if (body == NULL) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Out of memory", HTTPD_RESP_USE_STRLEN);
    }

    int remaining = total_len;
    int offset = 0;
    while (remaining > 0) {
        int received = httpd_req_recv(req, body + offset, remaining);
        if (received <= 0) {
            free(body);
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "Failed to read request body", HTTPD_RESP_USE_STRLEN);
        }
        offset += received;
        remaining -= received;
    }
    body[offset] = '\0';

    char ssid[33];
    char password[64];
    char zone_id[33];
    char zone_name[65];
    char broker_ip[64];
    char broker_port_text[8];
    char setup_payload[160];
    zone_config_t saved_zone;
    zone_config_t active_zone;
    setup_config_t setup_cfg;
    bool zone_found = false;
    bool has_new_zone_id;
    bool has_new_zone_name;
    bool zone_reassigned = false;

    get_form_value(body, "ssid=", ssid, sizeof(ssid));
    get_form_value(body, "password=", password, sizeof(password));
    get_form_value(body, "zone_id=", zone_id, sizeof(zone_id));
    get_form_value(body, "zone_name=", zone_name, sizeof(zone_name));
    get_form_value(body, "broker_ip=", broker_ip, sizeof(broker_ip));
    get_form_value(body, "broker_port=", broker_port_text, sizeof(broker_port_text));

    if (strlen(ssid) == 0 || strlen(broker_ip) == 0) {
        free(body);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "ssid and broker_ip are required", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t zone_ret = zone_config_load(&saved_zone, &zone_found);
    if (zone_ret != ESP_OK) {
        free(body);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Failed to load current zone", HTTPD_RESP_USE_STRLEN);
    }

    has_new_zone_id = strlen(zone_id) > 0;
    has_new_zone_name = strlen(zone_name) > 0;

    if (has_new_zone_id != has_new_zone_name) {
        free(body);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "zone_id and zone_name must both be provided when reassigning", HTTPD_RESP_USE_STRLEN);
    }

    if (has_new_zone_id) {
        strncpy(active_zone.zone_id, zone_id, sizeof(active_zone.zone_id) - 1);
        active_zone.zone_id[sizeof(active_zone.zone_id) - 1] = '\0';
        strncpy(active_zone.zone_name, zone_name, sizeof(active_zone.zone_name) - 1);
        active_zone.zone_name[sizeof(active_zone.zone_name) - 1] = '\0';

        esp_err_t save_ret = zone_config_save(&active_zone);
        if (save_ret != ESP_OK) {
            free(body);
            httpd_resp_set_status(req, "500 Internal Server Error");
            return httpd_resp_send(req, "Failed to save new zone assignment", HTTPD_RESP_USE_STRLEN);
        }
        zone_reassigned = true;
    } else {
        if (!zone_found) {
            free(body);
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_send(req, "No current zone found. Provide zone_id and zone_name to assign one.", HTTPD_RESP_USE_STRLEN);
        }
        active_zone = saved_zone;
    }

    int payload_written = snprintf(
        setup_payload,
        sizeof(setup_payload),
        "{\"zone_id\":\"%s\",\"name\":\"%s\"}",
        active_zone.zone_id,
        active_zone.zone_name);

    if (payload_written <= 0 || payload_written >= (int)sizeof(setup_payload)) {
        free(body);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "Payload is too large", HTTPD_RESP_USE_STRLEN);
    }

    int broker_port = atoi(broker_port_text);
    if (broker_port <= 0) {
        broker_port = 1883;
    }

    ESP_LOGI(TAG, "Request: ssid=%s, broker=%s:%d, payload=%s", ssid, broker_ip, broker_port, setup_payload);

    runtime_tasks_stop();
    mqtt_manager_deinit();

    esp_err_t conn_ret = wifi_manager_connect_sta(ssid, password, 20000);
    if (conn_ret != ESP_OK) {
        free(body);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Failed to connect to target Wi-Fi", HTTPD_RESP_USE_STRLEN);
    }

    lcd_status_show_wifi_and_broker(ssid, broker_ip);

    esp_err_t mqtt_init_ret = mqtt_manager_init(broker_ip, broker_port);
    if (mqtt_init_ret != ESP_OK) {
        free(body);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Connected to Wi-Fi, but failed to connect MQTT broker", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t mqtt_pub_ret = mqtt_manager_publish_setup_and_wait(active_zone.zone_id, setup_payload, 15000);
    if (mqtt_pub_ret != ESP_OK) {
        free(body);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "MQTT SetUp publish failed or zone_id/Success timeout", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t runtime_ret = runtime_tasks_start(active_zone.zone_id);
    if (runtime_ret != ESP_OK) {
        free(body);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Runtime tasks setup failed", HTTPD_RESP_USE_STRLEN);
    }

    memset(&setup_cfg, 0, sizeof(setup_cfg));
    strncpy(setup_cfg.ssid, ssid, sizeof(setup_cfg.ssid) - 1);
    setup_cfg.ssid[sizeof(setup_cfg.ssid) - 1] = '\0';
    strncpy(setup_cfg.password, password, sizeof(setup_cfg.password) - 1);
    setup_cfg.password[sizeof(setup_cfg.password) - 1] = '\0';
    strncpy(setup_cfg.broker_ip, broker_ip, sizeof(setup_cfg.broker_ip) - 1);
    setup_cfg.broker_ip[sizeof(setup_cfg.broker_ip) - 1] = '\0';
    setup_cfg.broker_port = broker_port;

    esp_err_t save_setup_ret = setup_config_save(&setup_cfg);
    if (save_setup_ret != ESP_OK) {
        ESP_LOGW(TAG, "Runtime started, but failed to persist setup profile: %s", esp_err_to_name(save_setup_ret));
    }

    lcd_status_show_zone_overview(active_zone.zone_id, active_zone.zone_name, "None");

    free(body);
    if (zone_reassigned) {
        return httpd_resp_send(
            req,
            "Zone reassigned successfully. Wi-Fi connected. SetUp published and zone_id/Success received. OTA is triggered via HTTP using MQTT commands.",
            HTTPD_RESP_USE_STRLEN);
    }
    return httpd_resp_send(
        req,
        "Wi-Fi connected. SetUp published and zone_id/Success received. OTA is triggered via HTTP using MQTT commands.",
        HTTPD_RESP_USE_STRLEN);
}

static esp_err_t debug_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    ESP_ERROR_CHECK(httpd_resp_sendstr_chunk(req, DEBUG_HTML_HEAD));
    ESP_ERROR_CHECK(httpd_resp_sendstr_chunk(req, DEBUG_HTML_TAIL));
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t debug_data_get_handler(httpd_req_t *req)
{
    zone_config_t zone_cfg;
    bool zone_found = false;
    const char *zone_id = "<zone_id>";
    const char *zone_name = "Not assigned";
    char zone_text[96];

    actuator_last_command_t last_cmds[ACTUATOR_CHANNEL_COUNT];
    size_t last_cmd_count = 0;
    sensor_telemetry_snapshot_t snap;
    uint32_t subs_lock_timeout_count = mqtt_manager_get_lock_timeout_count();
    char commands_json[900];
    char json[1900];

    esp_err_t zone_ret = zone_config_load(&zone_cfg, &zone_found);
    if (zone_ret == ESP_OK && zone_found) {
        zone_id = zone_cfg.zone_id;
        zone_name = zone_cfg.zone_name;
    }

    actuator_control_get_all_last_commands(last_cmds, ACTUATOR_CHANNEL_COUNT, &last_cmd_count);
    sensor_telemetry_get_snapshot(&snap);
    commands_json[0] = '\0';

    size_t offset = 0;
    for (size_t i = 0; i < last_cmd_count; i++) {
        int written_cmd = snprintf(
            commands_json + offset,
            sizeof(commands_json) - offset,
            "%s{\"topic\":\"%s\",\"channel\":\"%s\",\"command\":\"%s\",\"state\":\"%s\"}",
            (i == 0) ? "" : ",",
            last_cmds[i].topic,
            last_cmds[i].channel,
            last_cmds[i].command,
            last_cmds[i].state_on ? "ON" : "OFF");

        if (written_cmd <= 0 || (size_t)written_cmd >= (sizeof(commands_json) - offset)) {
            httpd_resp_set_status(req, "500 Internal Server Error");
            return httpd_resp_send(req, "Failed to render command list", HTTPD_RESP_USE_STRLEN);
        }

        offset += (size_t)written_cmd;
    }

    snprintf(zone_text, sizeof(zone_text), "%.32s (%.40s)", zone_id, zone_name);

    int written = snprintf(
        json,
        sizeof(json),
        "{"
        "\"zone\":\"%s\","
        "\"topics\":["
        "\"SetUp\","
        "\"%s/success\","
        "\"%s/sensor/WaterLevel/state\","
        "\"%s/sensor/WaterTemp/state\","
        "\"%s/sensor/pH/state\","
        "\"%s/sensor/TDS/state\","
        "\"%s/valve/command\","
        "\"%s/valve/status\","
        "\"%s/PerNutA/command\","
        "\"%s/PerNutA/status\","
        "\"%s/PerNutB/command\","
        "\"%s/PerNutB/status\","
        "\"%s/PerpHUp/command\","
        "\"%s/PerpHUp/status\","
        "\"%s/PerpHDown/command\","
        "\"%s/PerpHDown/status\","
        "\"%s/ota/latest_version\","
        "\"%s/ota/trigger\"],"
        "\"lastCommands\":[%s],"
        "\"sensor\":{"
        "\"waterLevel\":%d,"
        "\"waterTemp\":%.2f,"
        "\"ph\":%.2f,"
        "\"tds\":%.2f,"
        "\"publishCount\":%lu"
        "},"
        "\"mqttHealth\":{"
        "\"subscriptionsLockTimeoutCount\":%lu"
        "}"
        "}",
        zone_text,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        zone_id,
        commands_json,
        snap.valid ? snap.water_level : 0,
        (double)(snap.valid ? snap.water_temp : 0.0f),
        (double)(snap.valid ? snap.ph : 0.0f),
        (double)(snap.valid ? snap.tds : 0.0f),
        (unsigned long)(snap.valid ? snap.publish_count : 0),
        (unsigned long)subs_lock_timeout_count);

    if (written <= 0 || written >= (int)sizeof(json)) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Failed to render debug JSON", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

esp_err_t web_portal_try_autostart_from_nvs(bool *started)
{
    zone_config_t zone_cfg;
    setup_config_t setup_cfg;
    bool zone_found = false;
    bool setup_found = false;
    char setup_payload[160];

    if (started != NULL) {
        *started = false;
    }

    esp_err_t ret = zone_config_load(&zone_cfg, &zone_found);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = setup_config_load(&setup_cfg, &setup_found);
    if (ret != ESP_OK) {
        return ret;
    }

    if (!zone_found || !setup_found) {
        ESP_LOGI(TAG, "Autostart skipped: missing saved zone or setup profile");
        return ESP_OK;
    }

    int payload_written = snprintf(
        setup_payload,
        sizeof(setup_payload),
        "{\"zone_id\":\"%s\",\"name\":\"%s\"}",
        zone_cfg.zone_id,
        zone_cfg.zone_name);

    if (payload_written <= 0 || payload_written >= (int)sizeof(setup_payload)) {
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Autostart: trying saved Wi-Fi '%s' and zone '%s'", setup_cfg.ssid, zone_cfg.zone_id);

    ret = wifi_manager_connect_sta(setup_cfg.ssid, setup_cfg.password, 20000);
    if (ret != ESP_OK) {
        return ret;
    }

    lcd_status_show_wifi_and_broker(setup_cfg.ssid, setup_cfg.broker_ip);

    ret = mqtt_manager_init(setup_cfg.broker_ip, setup_cfg.broker_port);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mqtt_manager_publish_setup_and_wait(zone_cfg.zone_id, setup_payload, 15000);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = runtime_tasks_start(zone_cfg.zone_id);
    if (ret != ESP_OK) {
        return ret;
    }

    lcd_status_show_zone_overview(zone_cfg.zone_id, zone_cfg.zone_name, "None");

    if (started != NULL) {
        *started = true;
    }

    ESP_LOGI(TAG, "Autostart completed successfully");
    return ESP_OK;
}

esp_err_t web_portal_start(void)
{
    ESP_ERROR_CHECK(zone_config_init());
    ESP_ERROR_CHECK(setup_config_init());

    esp_err_t lcd_ret = lcd_status_init();
    if (lcd_ret != ESP_OK) {
        ESP_LOGW(TAG, "LCD init failed, continuing without LCD: %s", esp_err_to_name(lcd_ret));
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 7;   // ← was 10, max allowed with current LWIP_MAX_SOCKETS=10
    config.lru_purge_enable = true;
    config.recv_wait_timeout  = 3;
    config.send_wait_timeout  = 3;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t configure_uri = {
        .uri = "/configure",
        .method = HTTP_POST,
        .handler = configure_post_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t debug_uri = {
        .uri = "/debug",
        .method = HTTP_GET,
        .handler = debug_get_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t debug_data_uri = {
        .uri = "/debug/data",
        .method = HTTP_GET,
        .handler = debug_data_get_handler,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &configure_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &debug_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &debug_data_uri));
    ESP_LOGI(TAG, "Web portal running on http://192.168.4.1/");
    return ESP_OK;
}