// ESP-IDF logging: sim.cpp routes a component's log line through the hook set
// here, or (no hook) writes it raw into the UART stream as ESP-IDF does.
#pragma once
#include <cstdarg>
typedef int (*vprintf_like_t)(const char *, va_list);
vprintf_like_t esp_log_set_vprintf(vprintf_like_t f);
