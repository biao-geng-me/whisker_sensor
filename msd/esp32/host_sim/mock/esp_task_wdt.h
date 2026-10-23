// Task watchdog (host_sim: not modelled).
#pragma once
#include "Arduino.h"
typedef int esp_err_t;
static inline esp_err_t esp_task_wdt_add(TaskHandle_t) { return 0; }
static inline esp_err_t esp_task_wdt_delete(TaskHandle_t) { return 0; }
static inline esp_err_t esp_task_wdt_reset() { return 0; }
