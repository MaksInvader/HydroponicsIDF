#ifndef RUNTIME_TASKS_H
#define RUNTIME_TASKS_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef uint32_t safety_fault_mask_t;

esp_err_t runtime_tasks_start(const char *zone_id);
esp_err_t runtime_tasks_stop(void);
bool runtime_tasks_is_running(void);
void runtime_tasks_record_boot_faults(void);
safety_fault_mask_t runtime_tasks_get_safety_faults(void);
esp_err_t runtime_tasks_clear_safety_faults(safety_fault_mask_t mask);
bool runtime_tasks_is_safe_mode(void);

#endif