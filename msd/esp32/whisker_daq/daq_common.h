// daq_common.h — what whisker_daq.ino and the locked reader (lk_reader.cpp)
// share: pins, ADC registers and commands, scan geometry, frame format, and the
// state and helpers both sides use. Definitions live in whisker_daq.ino.
#pragma once
#include <Arduino.h>
#include <SPI.h>

// ── Serial ───────────────────────────────────────────────────────────────────
#define SERIAL_BAUD   921600UL    // CP2102 handles this; 115200 caps at ~50 scans/s
#define SERIAL_TXBUF  4096      // 250 scans/s x 118 B = 30 kB/s in wide mode

// ── Pins ─────────────────────────────────────────────────────────────────────
#define PIN_SCLK   18
#define PIN_MISO   19
#define PIN_MOSI   23
#define PIN_CS     21
#define PIN_DRDY    4
#define PIN_CLKIN  16

#define PIN_SELC_1 27
#define PIN_SELB_1 26
#define PIN_SELA_1 25
#define PIN_SELA_2 32
#define PIN_SELB_2 33
#define PIN_SELC_2 14

// OPTIONAL hardware reset — requires the U4 pin-11 rework (PIN11_REWORK.md).
// Leave at 0 while pin 11 is still tied to 3.3 V.
#define PIN_RESET      17
#define HAVE_HW_RESET  0

// Safety: the boot probe checks whether any select line is still jumpered to
// GND before driving it. Set to 0 only if you deliberately want to bypass it.
#define PROBE_SELECTS  1

// ── Clocking ─────────────────────────────────────────────────────────────────
#define CLKIN_HZ   800000UL   // 0.8 MHz on GPIO16 via LEDC
// One ADC read is 18 bytes = 144 SCLK. It must finish inside one 320 us sample
// period or every read waits an extra period: 200 kHz (720 us/read) gave ~33
// scans/s. 600 kHz = 240 us/read. The old bring-up rule "SCLK must stay below
// CLKIN" is not in the datasheet: SBAS890D 6.6 only asks tc(SC) >= 40 ns
// (SCLK <= 25 MHz at DVDD 2.7-3.6 V); SPI is asynchronous to CLKIN. What does
// matter is finishing a read before the next conversion lands (8.5.1.5: the
// DRDY pulse is blocked when a conversion completes during a read). Watch the
// server's adc-crc / adc-stale / recov counters after raising it.
#define SPI_HZ     600000UL
// fMOD = CLKIN/2 = 400 kHz; OSR 128 -> fDATA = 3.125 kSPS -> 320 us/sample.
#define FDATA_SPS  (CLKIN_HZ / 2 / 128)

// ── ADS131M04 commands / registers ───────────────────────────────────────────
#define CMD_NULL    0x0000
#define CMD_RESET   0x0011
#define CMD_UNLOCK  0x0655
#define CMD_LOCK    0x0555
#define CMD_RREG(a) (0xA000 | ((a) << 7))
#define CMD_WREG(a) (0x6000 | ((a) << 7))
#define REG_ID      0x00
#define REG_MODE    0x02
#define REG_CLOCK   0x03
#define REG_GAIN1   0x04   // the only GAIN register on the M04; 0x05 is reserved
// CLOCK: reserved[15:12]=1111 (preserve), CH_EN[11:8]=1111, OSR[4:2]=000 (128),
// PWR[1:0]=10 (high-resolution) -> 0xFF02
#define CLOCK_ALL_CH_OSR128  0xFF02
#define CLOCK_MASK  0x0FFF
#define CLOCK_OSR_MASK  0x001C   // OSR[2:0] at bits 4:2: 128 << code, 7 = 16384
#define OSR_NORMAL  128
// MODE (reset 0x0510): RESET bit 10 = 0 (clears STATUS RESET), WLENGTH 01b
// (24 bit), TIMEOUT 1, DRDY_SEL 00, DRDY_HiZ 0, DRDY_FMT 0: the defaults with
// the reset flag cleared (SBAS890D Table 8-16).
#define MODE_CLEAR_RESET  0x0110
#define GAIN_ALL_1  0x0000
// STATUS (reg 0x01, returned as word 0 of every data frame, MSB-aligned in the
// 24-bit word): bits 3:0 = DRDY3..DRDY0, "1b = New data are available" per
// channel (SBAS890D Table 8-15). All four channels are enabled, so a frame
// with a fresh conversion has all four set.
#define STATUS_DRDY_ALL  0x000F
// STATUS bit 10 RESET: "1b = Reset occurred" (Table 8-15). It stays set until
// MODE bit 10 is written 0, which initADC does, so seeing it later means the
// ADC reset behind our back (late power-up, brown-out, spark) and is back at
// its defaults (OSR 1024: the scan drops from ~76 to 12.2 Hz, CRCs stay fine).
#define STATUS_RESET     0x0400

#define VREF        1.2f
#define ADC_FS      8388608.0f   // 2^23
// INA351ABS with GS (pin 1) tied to GND in the MSD schematic -> gain 10. The
// inherited "~141" doesn't match this part. Text-mode display only.
#define INA_GAIN    10.0f

// ── Scan geometry ────────────────────────────────────────────────────────────
#define NUM_SEL         8
#define NUM_ADC_CH      4
#define NUM_OUT         32
#define NUM_WHISKER     16
#define SETTLE_US       250   // MUX + INA351 analog settle after a select change
#define DISCARD_SAMPLES 3     // sinc3 filter samples to drop after that

// (ADC channel, select) -> OUT number.  Rows: CH0..CH3.
static const uint8_t OUT_OF[NUM_ADC_CH][NUM_SEL] = {
  /* CH0 = COM_X2 */ {32, 30, 28, 26, 31, 29, 27, 25},
  /* CH1 = COM_Y2 */ {18, 20, 22, 24, 23, 21, 19, 17},
  /* CH2 = COM_X1 */ { 5,  7,  3,  1,  2,  4,  6,  8},
  /* CH3 = COM_Y1 */ {15, 13, 11,  9, 10, 12, 16, 14},
};

// CONN index within a J header (1..16) -> physical header pin.
// Every J header has its odd column transposed in pairs; this is that fix.
static const uint8_t CONN_TO_JPIN[17] =
  {0, 3, 1, 2, 4, 7, 5, 6, 8, 11, 9, 10, 12, 15, 13, 14, 16};

// ── Binary frame format (mirror of whisker_link/esp32_frame.py) ─────────────
#define FRAME_SYNC0   0xA5
#define FRAME_SYNC1   0x5A
#define FT_SCAN       0x01
#define FT_TEXT       0x02
#define FT_INFO       0x03
#define FT_HEALTH     0x04   // 1 Hz reader health in the locked scan modes
#define PROTO_VERSION 4      // 2: INFO + osr, settle_us, discard_samples; 3: + sel_mask;
                             // 4: + mode, spi_hz, n_conv, avg per select; HEALTH frame
#define FLAG_RECOVERED 0x01   // an ADC halt was recovered during this sweep
#define FLAG_CRC_RETRY 0x02   // an ADC SPI read failed CRC and was re-read
#define FLAG_STALE     0x04   // a read carried no new conversion and was dropped
#define FLAG_REINIT    0x08   // the ADC had reset (STATUS RESET); re-initialised,
                              // sweep restarted
#define FLAG_SYNC      0x10   // locked scan: conversions missed / mux switched late /
                              // resynced to the conversion grid in this scan
#define FLAG_EMPTY     0x20   // locked scan: a slot had no usable conversion (its
                              // outputs repeat the previous value)

// A channel with nothing plugged in leaves the INA351 inputs floating, its
// output slams to a rail, and the ADC clips at a constant +/- full scale.
#define SAT_CODE   8380000L           // 2^23 = 8388608; anything near it is clipped
#define SEL_MASK_ALL  0xFF
#define SKIP_CODE     8388607L       // +full scale (a skipped select's outputs)
// SBAS890D 6.6: tc(SC) >= 40 ns, i.e. SCLK <= 25 MHz. Past ~8 MHz the jumper
// wiring, not the chip, sets the limit; 'p' measures it.
#define SPI_MAX_HZ  20000000L

// A temporary ADC config of the diagnostics (r, D, p); see fastCfgApply.
struct FastCfg {
    long     clkin, osr, spi;     // requested
    uint32_t clkActual, spiActual;
    uint16_t clockRb;
    float    fdata, periodUs;
    uint32_t timeoutMs;           // waitDRDY timeout: 2 periods + slack
};

// ── Shared state (defined in whisker_daq.ino) ───────────────────────────────
extern SPIClass vspi;
extern float    outV  [NUM_OUT + 1];   // latest voltage, indexed by OUT number
extern long     outRaw[NUM_OUT + 1];   // latest raw 24-bit code
extern uint16_t g_clockRb;
extern uint32_t g_halts, g_crcFails, g_scans, g_staleReads, g_adcReinits;
extern uint32_t g_scanT0, g_scanDur;
extern bool     g_scanRecovered, g_scanCrcRetry, g_scanStale, g_scanReinit;
extern uint8_t  g_selMask;
extern uint32_t g_clkinNow;
extern char     g_mode;                // 'B' = binary stream, else a text mode
extern volatile uint32_t g_drdyEdges, g_drdyEdgeT;   // DRDY ISR: count, time of the latest
extern uint32_t g_drdyTaken;           // edges already claimed by a read
extern bool     g_respPending;         // next frame's word 0 is a command response
extern uint16_t g_crcTab[256];

// ── Shared helpers (defined in whisker_daq.ino) ─────────────────────────────
void send_frame(uint8_t type, const uint8_t *payload, uint8_t len);
void send_scan();
void send_info();
void logf(const char *fmt, ...);
uint32_t txqUsed();
uint32_t txqWrite(uint32_t maxBytes, bool paced = false);
void txqFlushBlocking();
int  osrCode(long osr);
bool fastCfgApply(long clkin, long osr, long spi, FastCfg *c);
void restoreNormalCfg();
void spiFlush();
void setClkin(uint32_t hz);

// Forget edges nobody read (text-mode pauses, re-init). Data still sitting in
// the ADC's FIFO keeps DRDY low, so waitDRDY still finds it.
static inline void drdyResync() { g_drdyTaken = g_drdyEdges; }
// Table-driven CRC-16-CCITT (crcTabInit fills the table).
static inline uint16_t crc16_fast(uint16_t crc, const uint8_t *d, int n) {
    while (n--) crc = (uint16_t)(crc << 8) ^ g_crcTab[((crc >> 8) ^ *d++) & 0xFF];
    return crc;
}
static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
