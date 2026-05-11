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

static bool json_escape_into(const char *src, char *dst, size_t dst_len)
{
    if (dst == NULL || dst_len == 0) {
        return false;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return true;
    }

    size_t di = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p != '\0'; p++) {
        unsigned char c = *p;
        if (c == '"' || c == '\\') {
            if (di + 2 >= dst_len) {
                return false;
            }
            dst[di++] = '\\';
            dst[di++] = (char)c;
            continue;
        }

        if (c <= 0x1F) {
            if (di + 6 >= dst_len) {
                return false;
            }
            int written = snprintf(dst + di, dst_len - di, "\\u%04x", (unsigned)c);
            if (written != 6) {
                return false;
            }
            di += 6;
            continue;
        }

        if (di + 1 >= dst_len) {
            return false;
        }
        dst[di++] = (char)c;
    }

    dst[di] = '\0';
    return true;
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

typedef struct {
    char ssid[33];
    char password[64];
    char zone_id[33];
    char zone_name[65];
    char broker_ip[64];
    int broker_port;
} portal_request_t;

typedef enum {
    PORTAL_SETUP_OK = 0,
    PORTAL_SETUP_WIFI_FAILED,
    PORTAL_SETUP_MQTT_INIT_FAILED,
    PORTAL_SETUP_MQTT_PUB_FAILED,
    PORTAL_SETUP_RUNTIME_FAILED,
} portal_setup_result_t;

static esp_err_t send_http_error(httpd_req_t *req, const char *status, const char *message)
{
    httpd_resp_set_status(req, status);
    return httpd_resp_send(req, message, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t read_request_body(httpd_req_t *req, char **out_body)
{
    if (out_body == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_body = NULL;

    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 1536) {
        return ESP_ERR_INVALID_SIZE;
    }

    char *body = (char *)malloc((size_t)total_len + 1);
    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }

    int remaining = total_len;
    int offset = 0;
    while (remaining > 0) {
        int received = httpd_req_recv(req, body + offset, remaining);
        if (received <= 0) {
            free(body);
            return ESP_FAIL;
        }
        offset += received;
        remaining -= received;
    }
    body[offset] = '\0';

    *out_body = body;
    return ESP_OK;
}

static esp_err_t parse_portal_request(const char *body, portal_request_t *out)
{
    if (body == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));

    char broker_port_text[8];
    broker_port_text[0] = '\0';

    get_form_value(body, "ssid=", out->ssid, sizeof(out->ssid));
    get_form_value(body, "password=", out->password, sizeof(out->password));
    get_form_value(body, "zone_id=", out->zone_id, sizeof(out->zone_id));
    get_form_value(body, "zone_name=", out->zone_name, sizeof(out->zone_name));
    get_form_value(body, "broker_ip=", out->broker_ip, sizeof(out->broker_ip));
    get_form_value(body, "broker_port=", broker_port_text, sizeof(broker_port_text));

    int broker_port = atoi(broker_port_text);
    if (broker_port <= 0) {
        broker_port = 1883;
    }
    out->broker_port = broker_port;

    return ESP_OK;
}

static esp_err_t resolve_zone_assignment(const portal_request_t *req, zone_config_t *active_zone,
                                         bool *zone_reassigned, bool *save_failed)
{
    if (req == NULL || active_zone == NULL || zone_reassigned == NULL || save_failed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *save_failed = false;

    zone_config_t saved_zone;
    bool zone_found = false;

    esp_err_t zone_ret = zone_config_load(&saved_zone, &zone_found);
    if (zone_ret != ESP_OK) {
        return zone_ret;
    }

    bool has_new_zone_id = strlen(req->zone_id) > 0;
    bool has_new_zone_name = strlen(req->zone_name) > 0;

    if (has_new_zone_id != has_new_zone_name) {
        return ESP_ERR_INVALID_ARG;
    }

    if (has_new_zone_id) {
        strncpy(active_zone->zone_id, req->zone_id, sizeof(active_zone->zone_id) - 1);
        active_zone->zone_id[sizeof(active_zone->zone_id) - 1] = '\0';
        strncpy(active_zone->zone_name, req->zone_name, sizeof(active_zone->zone_name) - 1);
        active_zone->zone_name[sizeof(active_zone->zone_name) - 1] = '\0';

        esp_err_t save_ret = zone_config_save(active_zone);
        if (save_ret != ESP_OK) {
            *save_failed = true;
            return save_ret;
        }

        *zone_reassigned = true;
        return ESP_OK;
    }

    if (!zone_found) {
        return ESP_ERR_NOT_FOUND;
    }

    *active_zone = saved_zone;
    *zone_reassigned = false;
    return ESP_OK;
}

static portal_setup_result_t start_runtime_with_request(const portal_request_t *req, const zone_config_t *zone,
                                                        const char *setup_payload)
{
    if (req == NULL || zone == NULL || setup_payload == NULL) {
        return PORTAL_SETUP_RUNTIME_FAILED;
    }

    runtime_tasks_stop();
    mqtt_manager_deinit();

    esp_err_t conn_ret = wifi_manager_connect_sta(req->ssid, req->password, 20000);
    if (conn_ret != ESP_OK) {
        return PORTAL_SETUP_WIFI_FAILED;
    }

    lcd_status_show_wifi_and_broker(req->ssid, req->broker_ip);

    esp_err_t mqtt_init_ret = mqtt_manager_init(req->broker_ip, req->broker_port);
    if (mqtt_init_ret != ESP_OK) {
        return PORTAL_SETUP_MQTT_INIT_FAILED;
    }

    esp_err_t mqtt_pub_ret = mqtt_manager_publish_setup_and_wait(zone->zone_id, setup_payload, 15000);
    if (mqtt_pub_ret != ESP_OK) {
        return PORTAL_SETUP_MQTT_PUB_FAILED;
    }

    esp_err_t runtime_ret = runtime_tasks_start(zone->zone_id);
    if (runtime_ret != ESP_OK) {
        return PORTAL_SETUP_RUNTIME_FAILED;
    }

    lcd_status_show_zone_overview(zone->zone_id, zone->zone_name, "None");
    return PORTAL_SETUP_OK;
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
    portal_request_t request;
    char setup_payload[160];
    zone_config_t active_zone;
    setup_config_t setup_cfg;
    bool zone_reassigned = false;

    esp_err_t body_ret = read_request_body(req, &body);
    if (body_ret == ESP_ERR_INVALID_SIZE) {
        return send_http_error(req, "400 Bad Request", "Invalid request body");
    }
    if (body_ret == ESP_ERR_NO_MEM) {
        return send_http_error(req, "500 Internal Server Error", "Out of memory");
    }
    if (body_ret != ESP_OK) {
        return send_http_error(req, "400 Bad Request", "Failed to read request body");
    }

    esp_err_t parse_ret = parse_portal_request(body, &request);
    if (parse_ret != ESP_OK) {
        free(body);
        return send_http_error(req, "400 Bad Request", "Invalid request body");
    }

    if (strlen(request.ssid) == 0 || strlen(request.broker_ip) == 0) {
        free(body);
        return send_http_error(req, "400 Bad Request", "ssid and broker_ip are required");
    }

    bool save_failed = false;
    esp_err_t zone_ret = resolve_zone_assignment(&request, &active_zone, &zone_reassigned, &save_failed);
    if (zone_ret == ESP_ERR_INVALID_ARG) {
        free(body);
        return send_http_error(req, "400 Bad Request", "zone_id and zone_name must both be provided when reassigning");
    }
    if (zone_ret == ESP_ERR_NOT_FOUND) {
        free(body);
        return send_http_error(req, "400 Bad Request", "No current zone found. Provide zone_id and zone_name to assign one.");
    }
    if (zone_ret != ESP_OK && save_failed) {
        free(body);
        return send_http_error(req, "500 Internal Server Error", "Failed to save new zone assignment");
    }
    if (zone_ret != ESP_OK) {
        free(body);
        return send_http_error(req, "500 Internal Server Error", "Failed to load current zone");
    }

    int payload_written = snprintf(
        setup_payload,
        sizeof(setup_payload),
        "{\"zone_id\":\"%s\",\"name\":\"%s\"}",
        active_zone.zone_id,
        active_zone.zone_name);

    if (payload_written <= 0 || payload_written >= (int)sizeof(setup_payload)) {
        free(body);
        return send_http_error(req, "400 Bad Request", "Payload is too large");
    }

    ESP_LOGI(TAG, "Request: ssid=%s, broker=%s:%d, payload=%s",
             request.ssid, request.broker_ip, request.broker_port, setup_payload);

    portal_setup_result_t run_ret = start_runtime_with_request(&request, &active_zone, setup_payload);
    if (run_ret == PORTAL_SETUP_WIFI_FAILED) {
        free(body);
        return send_http_error(req, "500 Internal Server Error", "Failed to connect to target Wi-Fi");
    }
    if (run_ret == PORTAL_SETUP_MQTT_INIT_FAILED) {
        free(body);
        return send_http_error(req, "500 Internal Server Error", "Connected to Wi-Fi, but failed to connect MQTT broker");
    }
    if (run_ret == PORTAL_SETUP_MQTT_PUB_FAILED) {
        free(body);
        return send_http_error(req, "500 Internal Server Error", "MQTT SetUp publish failed or zone_id/Success timeout");
    }
    if (run_ret == PORTAL_SETUP_RUNTIME_FAILED) {
        free(body);
        return send_http_error(req, "500 Internal Server Error", "Runtime tasks setup failed");
    }

    memset(&setup_cfg, 0, sizeof(setup_cfg));
    strncpy(setup_cfg.ssid, request.ssid, sizeof(setup_cfg.ssid) - 1);
    setup_cfg.ssid[sizeof(setup_cfg.ssid) - 1] = '\0';
    strncpy(setup_cfg.password, request.password, sizeof(setup_cfg.password) - 1);
    setup_cfg.password[sizeof(setup_cfg.password) - 1] = '\0';
    strncpy(setup_cfg.broker_ip, request.broker_ip, sizeof(setup_cfg.broker_ip) - 1);
    setup_cfg.broker_ip[sizeof(setup_cfg.broker_ip) - 1] = '\0';
    setup_cfg.broker_port = request.broker_port;

    esp_err_t save_setup_ret = setup_config_save(&setup_cfg);
    if (save_setup_ret != ESP_OK) {
        ESP_LOGW(TAG, "Runtime started, but failed to persist setup profile: %s", esp_err_to_name(save_setup_ret));
    }

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
    char zone_text_escaped[600];
    char zone_id_escaped[200];

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

    if (!json_escape_into(zone_id, zone_id_escaped, sizeof(zone_id_escaped))) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Failed to render zone id", HTTPD_RESP_USE_STRLEN);
    }

    size_t offset = 0;
    for (size_t i = 0; i < last_cmd_count; i++) {
        char topic_escaped[600];
        char channel_escaped[160];
        char command_escaped[64];

        if (!json_escape_into(last_cmds[i].topic, topic_escaped, sizeof(topic_escaped)) ||
            !json_escape_into(last_cmds[i].channel, channel_escaped, sizeof(channel_escaped)) ||
            !json_escape_into(last_cmds[i].command, command_escaped, sizeof(command_escaped))) {
            httpd_resp_set_status(req, "500 Internal Server Error");
            return httpd_resp_send(req, "Failed to escape command list", HTTPD_RESP_USE_STRLEN);
        }

        int written_cmd = snprintf(
            commands_json + offset,
            sizeof(commands_json) - offset,
            "%s{\"topic\":\"%s\",\"channel\":\"%s\",\"command\":\"%s\",\"state\":\"%s\"}",
            (i == 0) ? "" : ",",
            topic_escaped,
            channel_escaped,
            command_escaped,
            last_cmds[i].state_on ? "ON" : "OFF");

        if (written_cmd <= 0 || (size_t)written_cmd >= (sizeof(commands_json) - offset)) {
            httpd_resp_set_status(req, "500 Internal Server Error");
            return httpd_resp_send(req, "Failed to render command list", HTTPD_RESP_USE_STRLEN);
        }

        offset += (size_t)written_cmd;
    }

    snprintf(zone_text, sizeof(zone_text), "%.32s (%.40s)", zone_id, zone_name);
    if (!json_escape_into(zone_text, zone_text_escaped, sizeof(zone_text_escaped))) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "Failed to render zone text", HTTPD_RESP_USE_STRLEN);
    }

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
        zone_text_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
        zone_id_escaped,
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

    config.stack_size = 8192;

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