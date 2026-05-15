#ifndef RUNTIME_SAFETY_H
#define RUNTIME_SAFETY_H

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "actuator_control.h"
#include "runtime_tasks.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_FAULT_DOSE_A       = 1u << 0,
    SAFETY_FAULT_DOSE_B       = 1u << 1,
    SAFETY_FAULT_PH_UP        = 1u << 2,
    SAFETY_FAULT_PH_DOWN      = 1u << 3,
    SAFETY_FAULT_PH_INTERLOCK = 1u << 4,
    SAFETY_FAULT_VALVE        = 1u << 5,
    SAFETY_FAULT_GROWLIGHT    = 1u << 6,
    SAFETY_FAULT_TDS_SENSOR   = 1u << 7,
    SAFETY_FAULT_PH_SENSOR    = 1u << 8,
    SAFETY_FAULT_I2C_BUS      = 1u << 9,
    SAFETY_FAULT_WATER_TEMP   = 1u << 10,
    SAFETY_FAULT_POWER        = 1u << 11,
    SAFETY_FAULT_WDT          = 1u << 12,
} safety_fault_t;

typedef struct {
    QueueHandle_t *dosing_queue;
    TaskHandle_t *dosing_task;
    TaskHandle_t *sensor_task;
    TaskHandle_t *comm_task;
    TaskHandle_t *safety_task;
    atomic_bool *stop_requested;
    const char *zone_id;
    const char *zone_name;
    const char *safety_fault_topic;
} runtime_safety_bindings_t;

void runtime_safety_bind(runtime_safety_bindings_t *bindings);
void runtime_safety_set_dosing_queue(QueueHandle_t queue);

void runtime_safety_reset_state(void);
void runtime_safety_reset_heartbeats(void);

void runtime_safety_set_boot_faults(safety_fault_mask_t faults);
void runtime_safety_or_boot_faults(safety_fault_mask_t faults);
safety_fault_mask_t runtime_safety_get_boot_faults(void);
void runtime_safety_apply_boot_faults(void);

void runtime_safety_set_gate(bool open);
bool runtime_safety_is_gate_open(bool isr_context);

void runtime_safety_update_channel_state(actuator_channel_t channel, bool state_on);
bool runtime_safety_get_channel_state(actuator_channel_t channel);
void runtime_safety_copy_channel_states(bool out_states[ACTUATOR_CHANNEL_COUNT]);

void runtime_safety_wdt_init(void);
void runtime_safety_wdt_register(void);
void runtime_safety_wdt_kick(void);
void runtime_safety_wdt_unregister(void);

void runtime_safety_heartbeat_dosing(void);
void runtime_safety_heartbeat_sensor(void);
void runtime_safety_heartbeat_comm(void);
void runtime_safety_heartbeat_safety(void);

void runtime_safety_dose_watchdog_update(actuator_channel_t channel, actuator_action_t action, uint32_t pulse_ms);
void runtime_safety_note_ph_dose(actuator_channel_t channel);
void runtime_safety_note_tds_dose(actuator_channel_t channel);
void runtime_safety_note_valve_action(actuator_action_t action);

void runtime_safety_fault_set(safety_fault_mask_t mask, const char *reason);
safety_fault_mask_t runtime_safety_get_faults(void);
bool runtime_safety_is_safe_mode(void);
esp_err_t runtime_safety_clear_faults(safety_fault_mask_t mask, bool *safe_mode_cleared);

void runtime_safety_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif
