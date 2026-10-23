// Task watchdog (sim.cpp): tracks which tasks are subscribed; a reset from a
// task that isn't logs "task not found" like ESP-IDF (bench r3) and is counted.
#pragma once
#include "Arduino.h"
typedef int esp_err_t;
esp_err_t esp_task_wdt_add(TaskHandle_t t);
esp_err_t esp_task_wdt_delete(TaskHandle_t t);
esp_err_t esp_task_wdt_reset();
