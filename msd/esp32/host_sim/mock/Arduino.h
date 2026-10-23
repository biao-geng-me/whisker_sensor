// Minimal Arduino-ESP32 API for compiling whisker_daq.ino on a PC (host_sim).
// Everything that touches hardware or time is implemented in sim.cpp against
// a virtual clock and a model of the ADS131M04.
#pragma once
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using std::max;
using std::min;

#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define FALLING 0x02
#define digitalPinToInterrupt(p) (p)

// FreeRTOS / ESP-IDF task API (sim.cpp: tasks are threads on two virtual cores)
typedef void *TaskHandle_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY 0xFFFFFFFFu
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))     // 1 kHz tick
BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, uint32_t stack, void *arg,
                                   UBaseType_t prio, TaskHandle_t *h, BaseType_t core);
void vTaskDelete(TaskHandle_t h);
void vTaskDelay(TickType_t ticks);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks);
void xTaskNotifyGive(TaskHandle_t h);
bool disableCore0WDT();
typedef int portMUX_TYPE;                       // one runner at a time: no locking needed
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL_SAFE(m) ((void)(m))
#define portEXIT_CRITICAL_SAFE(m) ((void)(m))

uint32_t micros();
uint32_t millis();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
void attachInterrupt(uint8_t pin, void (*fn)(void), int mode);
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t res);
bool ledcWrite(uint8_t pin, uint32_t duty);
bool ledcDetach(uint8_t pin);
uint32_t ledcReadFreq(uint8_t pin);

class HardwareSerial {
  public:
    void begin(unsigned long);
    void setDebugOutput(bool);
    size_t setTxBufferSize(size_t n);
    int available();
    int read();
    int availableForWrite();
    size_t write(const uint8_t *d, size_t n);
    size_t write(uint8_t b) { return write(&b, 1); }
    size_t print(const char *s) { return write((const uint8_t *)s, strlen(s)); }
    size_t println(const char *s = "") { return print(s) + print("\r\n"); }
    size_t printf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
    void flush();
};
extern HardwareSerial Serial;
