# Actuator Activation Flow — MQTT Command to GPIO

This document traces the complete path from an incoming MQTT message to a physical GPIO state change, covering every layer of code involved.

---

## 1. Channels and GPIO Mapping

Nine actuator channels are defined in [main/actuator_control.h](main/actuator_control.h) and their GPIO pins in [main/pin_config.h](main/pin_config.h).

```c
// main/actuator_control.h
#define ACTUATOR_CHANNEL_COUNT 9

typedef enum {
    ACTUATOR_CHANNEL_VALVE = 0,
    ACTUATOR_CHANNEL_PER_NUTA,
    ACTUATOR_CHANNEL_PER_NUTB,
    ACTUATOR_CHANNEL_PER_PH_UP,
    ACTUATOR_CHANNEL_PER_PH_DOWN,
    ACTUATOR_CHANNEL_RELAY_1,
    ACTUATOR_CHANNEL_RELAY_2,
    ACTUATOR_CHANNEL_RELAY_3,
    ACTUATOR_CHANNEL_RELAY_4,
} actuator_channel_t;
```

```c
// main/pin_config.h
#define PIN_ACTUATOR_VALVE        12
#define PIN_ACTUATOR_PER_NUTA     13
#define PIN_ACTUATOR_PER_NUTB     14
#define PIN_ACTUATOR_PER_PH_UP    17
#define PIN_ACTUATOR_PER_PH_DOWN  18

/* Relay output pins (generic, server-controlled) */
#define PIN_RELAY_1               19
#define PIN_RELAY_2               20
#define PIN_RELAY_3               21
#define PIN_RELAY_4               38

/* Status / indicator LEDs */
#define PIN_LED_CONNECTION        39   /* HIGH = broker connected */
#define PIN_LED_FAULT             40   /* HIGH = fault or emergency active */

/* Reserved binary output */
#define PIN_RESERVE_BINARY        41
```

Each actuator channel maps to an `actuator_entry_t` that holds the channel name, GPIO number, and its two MQTT topics (command and status):

```c
// main/actuator_control.c
static actuator_entry_t s_actuators[] = {
    {.name = "valve",     .gpio = PIN_ACTUATOR_VALVE},
    {.name = "PerNutA",   .gpio = PIN_ACTUATOR_PER_NUTA},
    {.name = "PerNutB",   .gpio = PIN_ACTUATOR_PER_NUTB},
    {.name = "PerpHUp",   .gpio = PIN_ACTUATOR_PER_PH_UP},
    {.name = "PerpHDown", .gpio = PIN_ACTUATOR_PER_PH_DOWN},
    {.name = "relay1",    .gpio = PIN_RELAY_1},
    {.name = "relay2",    .gpio = PIN_RELAY_2},
    {.name = "relay3",    .gpio = PIN_RELAY_3},
    {.name = "relay4",    .gpio = PIN_RELAY_4},
};
```

The indicator LEDs and reserve binary pin are **not** actuator channels — they are managed separately by [main/indicator_led.c](main/indicator_led.c).

---

## 2. Initialization

### 2a. GPIO Configuration

`actuator_control_init()` is called from `runtime_setup_resources()` during startup. It configures all actuator pins as push-pull outputs and drives them LOW:

```c
// main/actuator_control.c — init_gpios_once()
gpio_config_t cfg = {
    .pin_bit_mask = 0,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

for (size_t i = 0; i < sizeof(s_gpio_list) / sizeof(s_gpio_list[0]); i++) {
    cfg.pin_bit_mask |= (1ULL << s_gpio_list[i]);
}

esp_err_t ret = gpio_config(&cfg);
// ...
for (size_t i = 0; i < sizeof(s_gpio_list) / sizeof(s_gpio_list[0]); i++) {
    gpio_set_level((gpio_num_t)s_gpio_list[i], 0);
}
```

### 2b. Topic Binding

`bind_actuator_topics()` builds the MQTT command and status topic strings for each channel using the zone ID:

```c
// main/actuator_control.c — bind_actuator_topics()
snprintf(
    s_actuators[i].command_topic,
    sizeof(s_actuators[i].command_topic),
    "%s/%s/command",
    zone_id,
    s_actuators[i].name);

snprintf(
    s_actuators[i].status_topic,
    sizeof(s_actuators[i].status_topic),
    "%s/%s/status",
    zone_id,
    s_actuators[i].name);
```

For example, with `zone_id = "greenhouse1"` and channel `valve`, the topics become:
- Command: `greenhouse1/valve/command`
- Status:  `greenhouse1/valve/status`

Each channel is also initialized to OFF and its status published to MQTT:

```c
// main/actuator_control.c — bind_actuator_topics()
gpio_set_level((gpio_num_t)s_actuators[i].gpio, 0);
esp_err_t ret = publish_status(&s_actuators[i], false);
```

### 2c. MQTT Subscription

After topic binding, `runtime_setup_topics()` subscribes each channel's command topic to the `on_channel_command` callback. The channel enum value is passed as `user_ctx` so the callback knows which channel received the message:

```c
// main/runtime_tasks.c — runtime_setup_topics()
for (int i = 0; i < ACTUATOR_CHANNEL_COUNT; i++) {
    actuator_channel_t ch = (actuator_channel_t)i;
    const char *cmd_topic = actuator_control_get_command_topic(ch);
    if (cmd_topic == NULL || cmd_topic[0] == '\0') {
        continue;
    }

    ret = mqtt_manager_subscribe(cmd_topic, 1, on_channel_command, (void *)(intptr_t)ch);
    if (ret == ESP_OK) {
        s_channel_topics_subscribed[i] = true;
    }
}
```

A zone-level wildcard topic (`{zone_id}/command`) is also subscribed for multi-channel commands:

```c
// main/runtime_tasks.c — runtime_setup_topics()
esp_err_t ret = subscribe_formatted_topic(s_zone_command_topic, ZONE_TOPIC_BUFFER_SIZE,
                                          "%s/command", zone_id, NULL,
                                          on_zone_command, NULL, &s_zone_topic_subscribed);
```

---

## 3. MQTT Message Arrival

When the broker delivers a message, `mqtt_manager` dispatches it to all matching subscribers via `dispatch_subscribed_message()`:

```c
// main/mqtt_manager.c — dispatch_subscribed_message()
for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++) {
    if (!s_subscriptions[i].in_use || s_subscriptions[i].cb == NULL) continue;

    if (topic_matches(topic, topic_len, s_subscriptions[i].topic) &&
        snapshot_count < MQTT_MAX_SUBSCRIPTIONS) {
        // copy to snapshot ...
        snapshot_count++;
    }
}
// ...
for (size_t i = 0; i < snapshot_count; i++) {
    if (snapshot[i].cb != NULL) {
        snapshot[i].cb(snapshot[i].topic, payload, payload_len, snapshot[i].user_ctx);
    }
}
```

Topic matching is exact string comparison — no wildcards:

```c
// main/mqtt_manager.c — topic_matches()
static bool topic_matches(const char *event_topic, int event_topic_len,
                          const char *registered_topic)
{
    size_t reg_len = strlen(registered_topic);
    return (event_topic_len == (int)reg_len) &&
           (strncmp(event_topic, registered_topic, reg_len) == 0);
}
```

---

## 4. Command Callbacks

### 4a. Per-Channel Command (`on_channel_command`)

Handles messages on `{zone_id}/{channel_name}/command`. The channel is recovered from `user_ctx`:

```c
// main/runtime_tasks.c — on_channel_command()
static void on_channel_command(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    actuator_channel_t channel = (actuator_channel_t)(intptr_t)user_ctx;

    actuator_action_t action;
    uint32_t pulse_ms = 0;
    if (actuator_control_parse_action_payload(payload, payload_len, &action, &pulse_ms) != ESP_OK) {
        ESP_LOGW(TAG, "Invalid actuator command payload");
        return;
    }

    dosing_command_t cmd = {
        .channel = channel,
        .action = action,
        .pulse_ms = pulse_ms,
    };
    (void)enqueue_command(&cmd, "channel command");
}
```

### 4b. Zone-Level Command (`on_zone_command`)

Handles messages on `{zone_id}/command`. The payload format is `<ChannelName> ON|OFF|PULSE <ms>`:

```c
// main/runtime_tasks.c — on_zone_command()
static void on_zone_command(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    (void)topic;
    (void)user_ctx;

    dosing_command_t cmd;
    if (!parse_zone_command(payload, payload_len, &cmd)) {
        ESP_LOGW(TAG, "Invalid zone command format. Expected: '<Channel> ON|OFF|PULSE <ms>'");
        return;
    }

    (void)enqueue_command(&cmd, "zone command");
}
```

`parse_zone_command()` tokenizes the payload and resolves the channel name:

```c
// main/runtime_tasks.c — parse_zone_command()
char *token0 = strtok_r(buf, " \t\r\n", &save_ptr);  // channel name
char *token1 = strtok_r(NULL, " \t\r\n", &save_ptr); // action
char *token2 = strtok_r(NULL, " \t\r\n", &save_ptr); // pulse ms (optional)

if (channel_from_name(token0, &channel) != ESP_OK) {
    return false;
}
```

### 4c. Payload Parsing

`actuator_control_parse_action_payload()` accepts `ON`, `OFF`, or `PULSE:<ms>` (case-insensitive, max 10 000 ms):

```c
// main/actuator_control.c — actuator_control_parse_action_payload()
if (strcmp(buf, "ON") == 0) {
    *action = ACTUATOR_ACTION_ON;
    return ESP_OK;
}

if (strcmp(buf, "OFF") == 0) {
    *action = ACTUATOR_ACTION_OFF;
    return ESP_OK;
}

const char *prefix = "PULSE:";
if (strncmp(buf, prefix, prefix_len) == 0) {
    unsigned long parsed = strtoul(buf + prefix_len, &end_ptr, 10);
    if (end_ptr == buf + prefix_len || *end_ptr != '\0' ||
        parsed == 0 || parsed > ACTUATOR_MAX_PULSE_MS) {
        return ESP_ERR_INVALID_ARG;
    }
    *action = ACTUATOR_ACTION_PULSE;
    *pulse_ms = (uint32_t)parsed;
    return ESP_OK;
}
```

---

## 5. Safety Gate Check and Queue Enqueue

Before the command reaches the queue, `enqueue_command()` checks the safety gate. If the gate is closed (safe mode active), the command is silently dropped:

```c
// main/runtime_tasks.c — enqueue_command()
if (!runtime_safety_is_gate_open(xPortInIsrContext())) {
    return false;
}

// ...
sent = xQueueSend(s_dosing_queue, cmd, 0);

if (sent != pdTRUE) {
    ESP_LOGW(TAG, "%s: dosing queue full, dropping command", source != NULL ? source : "enqueue");
    return false;
}
```

The queue holds up to 16 commands (`DOSING_QUEUE_LEN 16`), each a `dosing_command_t`:

```c
// main/runtime_tasks.c
typedef struct {
    actuator_channel_t channel;
    actuator_action_t action;
    uint32_t pulse_ms;
} dosing_command_t;
```

---

## 6. Dosing Task — Command Execution

`dosing_task` runs at priority 3 and blocks on the queue with a 200 ms timeout. On receipt it performs additional safety checks before executing:

```c
// main/runtime_tasks.c — dosing_task()
while (!atomic_load(&s_stop_requested)) {
    if (xQueueReceive(s_dosing_queue, &cmd, pdMS_TO_TICKS(DOSING_QUEUE_RECV_TIMEOUT_MS)) == pdTRUE) {

        // Drop if safe mode entered between enqueue and dequeue
        if (runtime_safety_is_safe_mode() || atomic_load(&s_stop_requested)) {
            ESP_LOGW(TAG, "Dropping dosing command while in safe mode or stopping");
            continue;
        }

        // pH interlock: prevent simultaneous pH Up + pH Down
        if ((cmd.action == ACTUATOR_ACTION_ON || cmd.action == ACTUATOR_ACTION_PULSE) &&
            ((cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP   && runtime_safety_get_channel_state(ACTUATOR_CHANNEL_PER_PH_DOWN)) ||
             (cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN && runtime_safety_get_channel_state(ACTUATOR_CHANNEL_PER_PH_UP)))) {
            runtime_safety_fault_set(SAFETY_FAULT_PH_INTERLOCK, "pH interlock violation");
            continue;
        }

        // Execute
        switch (cmd.action) {
        case ACTUATOR_ACTION_ON:
            ret = actuator_control_apply_state(cmd.channel, true);
            if (ret == ESP_OK) runtime_safety_update_channel_state(cmd.channel, true);
            break;
        case ACTUATOR_ACTION_OFF:
            ret = actuator_control_apply_state(cmd.channel, false);
            if (ret == ESP_OK) runtime_safety_update_channel_state(cmd.channel, false);
            break;
        case ACTUATOR_ACTION_PULSE:
            runtime_safety_update_channel_state(cmd.channel, true);
            ret = actuator_control_apply_pulse(cmd.channel, cmd.pulse_ms);
            runtime_safety_update_channel_state(cmd.channel, false);
            break;
        }
    }
}
```

---

## 7. GPIO Drive and Status Publish

### ON / OFF

`actuator_control_apply_state()` drives the GPIO and publishes the new state to the status topic:

```c
// main/actuator_control.c — actuator_control_apply_state()
esp_err_t actuator_control_apply_state(actuator_channel_t channel, bool is_on)
{
    const actuator_entry_t *actuator = entry_for_channel(channel);
    if (!s_initialized || !s_topics_bound || actuator == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level((gpio_num_t)actuator->gpio, is_on ? 1 : 0);

    portENTER_CRITICAL(&s_state_lock);
    update_last_command_locked(actuator, channel, is_on);
    portEXIT_CRITICAL(&s_state_lock);

    esp_err_t ret = publish_status(actuator, is_on);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to publish status for %s", actuator->name);
    }

    return ESP_OK;
}
```

### PULSE

`actuator_control_apply_pulse()` turns the channel ON, blocks the dosing task for the requested duration, then turns it OFF:

```c
// main/actuator_control.c — actuator_control_apply_pulse()
esp_err_t actuator_control_apply_pulse(actuator_channel_t channel, uint32_t pulse_ms)
{
    if (pulse_ms == 0 || pulse_ms > ACTUATOR_MAX_PULSE_MS) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = actuator_control_apply_state(channel, true);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    return actuator_control_apply_state(channel, false);
}
```

Maximum pulse duration is 10 000 ms (`ACTUATOR_MAX_PULSE_MS`). Because `vTaskDelay` is called inside the dosing task, no other dosing commands are processed during a pulse.

### Status Publish

After every state change, the status is published to `{zone_id}/{channel_name}/status` at QoS 1, non-retained:

```c
// main/actuator_control.c — publish_status()
static esp_err_t publish_status(const actuator_entry_t *actuator, bool is_on)
{
    if (actuator == NULL || actuator->status_topic[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }
    return mqtt_manager_publish(actuator->status_topic, is_on ? "ON" : "OFF", 1, 0);
}
```

---

## 8. Post-Execution Safety Bookkeeping

After a successful command, the dosing task notifies the safety subsystem so it can enforce dose-duration limits and interlock rules:

```c
// main/runtime_tasks.c — dosing_task()
runtime_safety_dose_watchdog_update(cmd.channel, cmd.action, cmd.pulse_ms);

if (cmd.channel == ACTUATOR_CHANNEL_VALVE) {
    runtime_safety_note_valve_action(cmd.action);
}

if (cmd.channel == ACTUATOR_CHANNEL_PER_PH_UP || cmd.channel == ACTUATOR_CHANNEL_PER_PH_DOWN) {
    runtime_safety_note_ph_dose(cmd.channel);
}

if (cmd.channel == ACTUATOR_CHANNEL_PER_NUTA || cmd.channel == ACTUATOR_CHANNEL_PER_NUTB) {
    runtime_safety_note_tds_dose(cmd.channel);
}
```

---

## 9. End-to-End Summary

```
MQTT broker
    │
    │  {zone_id}/{channel}/command  payload: "ON" | "OFF" | "PULSE:<ms>"
    │  {zone_id}/command            payload: "<ChannelName> ON|OFF|PULSE <ms>"
    ▼
mqtt_manager — dispatch_subscribed_message()
    │  exact topic match
    ▼
on_channel_command()  /  on_zone_command()
    │  parse payload → dosing_command_t
    ▼
enqueue_command()
    │  safety gate check → xQueueSend()
    ▼
dosing_task  (priority 3, stack 4096)
    │  xQueueReceive()
    │  safe-mode check
    │  pH interlock check
    ▼
actuator_control_apply_state()  /  actuator_control_apply_pulse()
    │  gpio_set_level()
    │  publish_status() → {zone_id}/{channel}/status  "ON" | "OFF"
    ▼
Physical GPIO pin (HIGH = actuator energised, LOW = off)
```

---

---

## 11. Emergency Stop

Subscribed during `runtime_setup_topics()` alongside the other control topics:

```c
// main/runtime_tasks.c — runtime_setup_topics()
ret = subscribe_formatted_topic(s_emergency_topic, ZONE_TOPIC_BUFFER_SIZE,
                                "%s/emergency", zone_id, NULL,
                                on_emergency_command, NULL, &s_emergency_subscribed);
```

The callback fires when payload `"1"` is received. It sets a broad fault mask covering all dosing channels and the valve, which closes the safety gate and turns off all actuators:

```c
// main/runtime_tasks.c — on_emergency_command()
static void on_emergency_command(const char *topic, const char *payload, int payload_len, void *user_ctx)
{
    if (!parse_payload_is_one(payload, payload_len)) {
        return;
    }

    runtime_safety_fault_set(SAFETY_FAULT_VALVE | SAFETY_FAULT_DOSE_A | SAFETY_FAULT_DOSE_B
                             | SAFETY_FAULT_PH_UP | SAFETY_FAULT_PH_DOWN,
                             "Emergency stop");
    indicator_led_fault_blink(5000);
}
```

The emergency state persists until faults are explicitly cleared via `{zone_id}/safety/clear`. Any payload other than `"1"` is ignored.

---

## 12. Indicator LEDs

Managed by [main/indicator_led.c](main/indicator_led.c), initialised inside `runtime_setup_resources()` before actuators.

| Pin | Name | Behaviour |
|---|---|---|
| GPIO39 | `PIN_LED_CONNECTION` | HIGH while MQTT broker is connected |
| GPIO40 | `PIN_LED_FAULT` | Pulses HIGH for 5 s on any new fault or emergency |

The connection LED is driven directly from the MQTT connection callback:

```c
// main/runtime_tasks.c — runtime_tasks_on_mqtt_connection()
indicator_led_set_connection(connected);
```

The fault LED is triggered inside `safety_fault_set()` in [main/runtime_safety.c](main/runtime_safety.c) whenever new (previously unreported) fault bits are set:

```c
// main/runtime_safety.c — safety_fault_set()
indicator_led_fault_blink(5000);  /* blocks calling task for 5 s */
```

This means the fault LED fires for **every** fault source: dose watchdog, pH interlock, sensor out-of-range, valve timeout, WDT, brownout, and emergency stop.

| Topic | Payload | Effect |
|---|---|---|
| `{zone}/valve/command` | `ON` | Open valve (GPIO HIGH) |
| `{zone}/valve/command` | `OFF` | Close valve (GPIO LOW) |
| `{zone}/PerNutA/command` | `PULSE:3000` | Nutrient A pump on for 3 s |
| `{zone}/PerpHUp/command` | `PULSE:1500` | pH Up pump on for 1.5 s |
| `{zone}/relay1/command` | `ON` | Relay 1 energised (GPIO HIGH) |
| `{zone}/relay1/command` | `OFF` | Relay 1 off (GPIO LOW) |
| `{zone}/relay2/command` | `PULSE:5000` | Relay 2 on for 5 s |
| `{zone}/relay3/command` | `ON` | Relay 3 energised |
| `{zone}/relay4/command` | `OFF` | Relay 4 off |
| `{zone}/command` | `valve ON` | Zone-level: open valve |
| `{zone}/command` | `PerNutB PULSE 2000` | Zone-level: Nutrient B pulse 2 s |
| `{zone}/command` | `relay1 ON` | Zone-level: relay 1 on |
| `{zone}/emergency` | `1` | Emergency stop — all actuators off, fault LED 5 s |

Payload matching is case-insensitive. Maximum pulse is 60 000 ms. Simultaneous pH Up + pH Down is blocked by the interlock and raises `SAFETY_FAULT_PH_INTERLOCK`. Relays follow the same ON/OFF/PULSE semantics as all other channels and are subject to the same safety gate.
