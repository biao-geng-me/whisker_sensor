#pragma once
#include "Arduino.h"
#define MSBFIRST 1
#define SPI_MODE1 1
#define VSPI 3
struct spi_t { int dummy; };
class SPISettings {
  public:
    SPISettings(uint32_t hz, uint8_t, uint8_t) : hz(hz) {}
    uint32_t hz;
};
class SPIClass {
  public:
    explicit SPIClass(int) {}
    void begin(int sck, int miso, int mosi, int ss);
    void beginTransaction(const SPISettings &s);
    void endTransaction() {}
    uint8_t transfer(uint8_t b);
    void transferBytes(const uint8_t *out, uint8_t *in, uint32_t n);
    void writeBytes(const uint8_t *out, uint32_t n) { transferBytes(out, nullptr, n); }
    spi_t *bus() { return &bus_; }
  private:
    spi_t bus_{};
};
void spiTransferBytesNL(spi_t *spi, const void *out, uint8_t *in, uint32_t n);
uint32_t spiGetClockDiv(spi_t *spi);
uint32_t spiClockDivToFrequency(spi_t *spi, uint32_t div);
