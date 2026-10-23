/*
 * ============================================================================
 *  whisker_daq.ino — 32-channel whisker array DAQ (16 whiskers x 2 gauges)
 *  MSD Seal Whisker Project · RIT · Flow Physics and Modeling Lab
 * ----------------------------------------------------------------------------
 *  Rewrite of whisker_reader.ino (Rev 2026-08-04-c) for the BlueROV2 payload.
 *  ADC driver, MUX mapping and halt recovery are unchanged from that file;
 *  what's new is the BINARY STREAM mode used by rpi/daq_server.py:
 *    - every scan carries the ESP32 micros() timestamp, so it can be aligned
 *      with the autopilot IMU on the host
 *    - raw 24-bit ADC codes, not mV-from-baseline: zeroing is done host side
 *      and nothing is thrown away
 *    - CRC-protected frames; log text is framed too, so it can't corrupt data
 *    - 921600 baud, so the serial link no longer caps the scan rate
 *  Frame layout: whisker_link/esp32_frame.py (keep the two in sync).
 *
 *  Board : ESP32 (ESP-WROOM-32, 38-pin)   Arduino ESP32 core 3.x
 *  ADC   : ADS131M04IPWR (4-ch, 24-bit delta-sigma, NO internal oscillator)
 *  MUX   : 2x CD4097BM (dual 8:1)     Amp : INA351ABS (gain 10, GS to GND)
 * ============================================================================
 *
 *  SIGNAL CHAIN
 *    whisker gauge (350 ohm quarter bridge, 3.3 V excitation)
 *    -> 2 connector pins -> INA351ABS (diff, gain 10) -> OUTn
 *    -> CD4097 channel -> COM_Xn/COM_Yn -> ADS131M04 AINxP -> SPI -> ESP32
 *    whisker W (1..16):  gauge A = OUT(2W-1),  gauge B = OUT(2W)
 *    OUT n:              CONN(2n-1) and CONN(2n)
 *
 *  P1 (ADC/SPI)                     P2 (MUX selects)
 *    1 SCLK      -> GPIO18 (pin14)    1 SELC_1 -> GPIO27
 *    2 DRDY      -> GPIO4  (pin13)    2 SELB_1 -> GPIO26
 *    3 CS        -> GPIO21 (pin12)    3 SELA_1 -> GPIO25
 *    4 DOUT/MISO -> GPIO19 (pin15)    4 SELA_2 -> GPIO32
 *    5 DIN/MOSI  -> GPIO23 (pin16)    5 SELB_2 -> GPIO33
 *    6 CLKIN     -> GPIO16 (pin17)    6 SELC_2 -> GPIO14
 *  PCB GND <-> ESP32 GND is REQUIRED.
 *
 *  SERIAL COMMANDS (single characters)
 *    B  BINARY STREAM — framed scans for daq_server.py (sends INFO first)
 *    g  grid view (default after boot)    x  array profile    s  table
 *    b  re-capture baseline               v  deviation <-> absolute volts
 *    m  connector/channel map             c  calibration      n  noise floor
 *    [ ]  halve / double the graph scale  0  auto-scale
 *    d<sel>  mux settling diagnostic: 'd' then a target select digit 0-7.
 *            Per ADC channel, switches N times from a select where that
 *            channel is railed/far off to <sel>, reads every conversion for
 *            ~5 ms (no discards) and prints the averaged mV vs time since
 *            the switch. Shows the true analog + sinc3 settling floor.
 *    r<sel> [n] [osr] [clkin_hz] [spi_hz]  then ';' or newline
 *            RAW BURST: holds mux select <sel> and records every conversion
 *            of the 4 ADC channels (default n 8192, OSR 128, CLKIN 0.8 MHz,
 *            SPI 600 kHz; OSR 128..16384, CLKIN up to 8 MHz), then dumps
 *            "#BURST ..." header, one "t_us flags c0 c1 c2 c3" line per
 *            conversion and "#END lines= crc=". Restores the normal config.
 *            Host side: msd/esp32/burst_spectrum.py. Details at burst_capture.
 *    D<from> <to> [clkin] [osr] [spi] [reps] [window_us] [dither] [dwell_us] [jitter_us];
 *            FINE MUX SETTLE: per rep, dwell on <from>, switch to <to> and
 *            read every conversion for window_us (defaults 8 MHz, OSR 256,
 *            SPI 4 MHz, 64 reps, 2000 us, dither 4, dwell 3000 us + random
 *            0..1000 us per rep); dumps all
 *            of them. Host: msd/esp32/adc_diag.py settle. See settle_fine.
 *    p[clkin] [osr] [secs] [spi_hz ...];
 *            SPI SPEED SWEEP: per SPI clock, read every conversion for <secs>
 *            and report frames, CRC errors, stale/overrun/edgeless reads and
 *            the read time (defaults 8 MHz, OSR 256, 2 s, 0.6-8 MHz).
 *            Host: msd/esp32/adc_diag.py spi. See spi_sweep.
 *    S<digits>;  scan only these mux selects, e.g. S0123457; ("S;" = all 8,
 *            also the boot default). Skipped outputs read as railed/open.
 *    M<0|1|2> [settle_us] [osr] [period_us] [spi_hz];   SCAN MODE (boot default M1)
 *            M0 legacy ~76 Hz scan (below). M1 quiet: locked scan, CLKIN
 *            8 MHz, OSR 512, SPI 8 MHz, 125 conversions = 16 ms, 62.5 Hz.
 *            M2 wide: OSR 128, SPI 12 MHz, 125 conversions = 4 ms, 250 Hz.
 *            0 or left out = default (settle 200 us). "M1 200 256;" = quiet
 *            at OSR 256 (250 conversions); "M1 200 256 8000;" = quiet at
 *            125 Hz (quiet periods: multiples of 8000 us, N = period / T must
 *            be whole, so 8000 needs OSR <= 256). Locked modes send a HEALTH
 *            frame each ~1 s. See the "Locked scan" section. Forgotten on reboot.
 *  r, D and p restore the normal config (0.8 MHz, OSR 128, SPI_HZ) after,
 *  and put a locked mode back if one was on.
 *  Any text-mode command leaves binary mode. Serial Monitor: set the line
 *  ending to "No line ending" and the baud to SERIAL_BAUD.
 *
 *  SCAN TIMING
 *    Each select: SETTLE_US, then DISCARD_SAMPLES + 1 conversions that END
 *    after the settle: the kept one's sinc3 window (3 x 320 us) starts after
 *    the settle. ~250 + 160 (avg wait) + 3 x 320 + 240 (last read) ~ 1.6
 *    ms/select, 8 selects ~ 13 ms -> ~75-80 scans/s (estimate; read dur_us
 *    off the stream). Every conversion is read (also during the
 *    settle) so the ADC's 2-deep output FIFO never backs up; a read that
 *    carries no new conversion is counted as "stale" (see readValid). The
 *    four ADC channels of one select are simultaneous; successive selects are
 *    ~1.6 ms apart, so gauges within one scan are NOT sampled at the same
 *    instant (see OUT_OF for the order). t_us in the frame is the sweep start.
 *    That is the legacy scan (M0). The locked scans (M1/M2) time everything
 *    from the ADC conversion grid instead: see "Locked scan".
 * ============================================================================
 */

#include <SPI.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include "soc/gpio_reg.h"
#include "soc/soc.h"

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

// ── State ────────────────────────────────────────────────────────────────────
SPIClass vspi(VSPI);

static float outV    [NUM_OUT + 1];   // latest voltage, indexed by OUT number
static long  outRaw  [NUM_OUT + 1];   // latest raw 24-bit code
static float outBase [NUM_OUT + 1];
static float outNoise[NUM_OUT + 1];

// A channel with nothing plugged in leaves the INA351 inputs floating, its
// output slams to a rail, and the ADC clips at a constant +/- full scale.
#define SAT_CODE   8380000L           // 2^23 = 8388608; anything near it is clipped

static uint16_t g_id = 0, g_clockRb = 0, g_modeRb = 0;
static uint32_t g_halts = 0, g_crcFails = 0, g_unrecovered = 0, g_scans = 0;
static uint32_t g_recByLevel[4] = {0, 0, 0, 0};
static uint32_t g_staleReads = 0;      // reads whose STATUS DRDY bits said "no new data"
static uint32_t g_adcReinits = 0;      // re-inits after STATUS RESET was seen set
static uint32_t g_reinitMs   = 0;      // millis() of the last one
static bool     g_reinitArmed = false; // set once boot init is done
static bool     g_absolute = false;   // 'v' toggles
static char     g_mode     = 'g';     // 'g' grid, 'x' profile, 's' table, 'c' cal, 'n' noise, 'B' binary
static uint32_t g_scanT0   = 0;       // micros() at the start of the last scan
static uint32_t g_scanDur  = 0;
static bool     g_scanRecovered = false;
static bool     g_scanCrcRetry  = false;
static bool     g_scanStale     = false;
static bool     g_scanReinit    = false;
// Active mux selects (bit s = select s is scanned); 'S' sets it. Outputs of a
// skipped select hold SKIP_CODE, i.e. read as railed ("open") in the SCAN
// railmask, so old decoders never take them for data. INFO carries the mask.
#define SEL_MASK_ALL  0xFF
#define SKIP_CODE     8388607L       // +full scale
static uint8_t  g_selMask = SEL_MASK_ALL;
// CLKIN and SPI clock currently in effect (the diagnostics change them).
static uint32_t g_clkinNow = CLKIN_HZ;
static uint32_t g_spiNow   = SPI_HZ;

// A temporary ADC config of the diagnostics (r, D, p); see fastCfgApply.
struct FastCfg {
    long     clkin, osr, spi;     // requested
    uint32_t clkActual, spiActual;
    uint16_t clockRb;
    float    fdata, periodUs;
    uint32_t timeoutMs;           // waitDRDY timeout: 2 periods + slack
};

// ── Locked scan state (modes M1 quiet / M2 wide; see the "Locked scan" section)
#define LK_MAX_N  1000
struct LockHealth {                 // one ~1 s window, sent as a HEALTH frame
    uint32_t t0Us, conv, overrun, missed, edgeless, crc, stale, lateSw;
    uint32_t txDrop, empty, resync, reinit, recov, latN, readN;
    uint64_t busyUs, latSum, readSum;
    uint16_t latMax, readMax, txMax;
};
struct LockState {
    uint8_t  mode;                  // 0 = legacy 76 Hz scan, 1 quiet, 2 wide
    uint16_t osr, N, settleUs, T;   // T = conversion period, us
    uint32_t spi, scanUs, spiActual;
    uint8_t  nslot, discard;
    uint8_t  slotSel[NUM_SEL], avgPerSel[NUM_SEL];
    uint16_t slotStart[NUM_SEL], slotLen[NUM_SEL];
    uint8_t  posSlot[LK_MAX_N];
    bool     haveRef, waitBoundary, streaming, scanDone, pendingSend;
    uint32_t tRef;                  // DRDY time of conversion pos 0 of the current scan
    int32_t  lastK;                 // pos of the last conversion read (-1: none yet)
    uint8_t  curSel;
    uint32_t tSw, lastEdgeT, lastActT;
    int64_t  acc[NUM_SEL][NUM_ADC_CH];
    uint16_t cnt[NUM_SEL];
    bool     taint[NUM_SEL];
    uint8_t  scanFlags;             // flags of the scan being taken
    uint8_t  outFlags;              // flags of the last finished scan (send_scan)
    LockHealth h;
};
static LockState g_lk = {};

// One conversion as read by readConv: DRDY edge time, codes, BF_* flags.
struct Conv {
    uint32_t t;
    long     ch[4];
    uint8_t  f;
    uint8_t  missed;
    uint16_t readUs, latUs;
};

// DRDY edge bookkeeping. Each falling edge is one new conversion; the ISR
// counts them and reads claim them one at a time, so no read can ever reuse
// a conversion that was already read (the old level test on DRDY could).
static volatile uint32_t g_drdyEdges = 0;   // falling edges seen by the ISR
static volatile uint32_t g_drdyEdgeT = 0;   // micros() of the latest edge
static uint32_t g_drdyTaken  = 0;           // edges already claimed by a read
static bool     g_respPending = false;      // next frame's word 0 is a command
                                            // response, not STATUS
// Time first, then the count: a reader that sees the new count also sees its time.
static void IRAM_ATTR drdyISR() { g_drdyEdgeT = micros(); g_drdyEdges = g_drdyEdges + 1; }

// Graph geometry.
#define BAR_H        6
#define COL_W        3
#define AXIS_W       8
#define SCALE_MIN  2.0f
static float g_scale    = SCALE_MIN;
static bool  g_autoScale = true;

inline void cs_low()  { digitalWrite(PIN_CS, LOW);  }
inline void cs_high() { digitalWrite(PIN_CS, HIGH); }

static bool initADC(uint16_t clockReg = CLOCK_ALL_CH_OSR128);
static bool readValid(long ch[4], uint32_t budget_ms, uint32_t *idx = nullptr,
                      uint32_t *tConv = nullptr);

// ─────────────────────────────────────────────────────────────────────────────
//  Framing + logging
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t *d, int n) {
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// Table-driven CRC-16-CCITT for the frames (same result as above, ~8x faster).
static uint16_t g_crcTab[256];
static void crcTabInit() {
    for (int i = 0; i < 256; i++) {
        uint16_t c = (uint16_t)(i << 8);
        for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
        g_crcTab[i] = c;
    }
}
static inline uint16_t crc16_fast(uint16_t crc, const uint8_t *d, int n) {
    while (n--) crc = (uint16_t)(crc << 8) ^ g_crcTab[((crc >> 8) ^ *d++) & 0xFF];
    return crc;
}

// TX queue for the locked scan: whole frames go in, the reader writes them to
// Serial in small chunks only where a chunk can't delay the next ADC read.
// Everything framed goes through it while g_lk.streaming, so frames never
// interleave. A frame that doesn't fit is dropped whole (counted).
#define TXQ_SIZE  4096
static uint8_t  g_txq[TXQ_SIZE];
static uint32_t g_txqHead = 0, g_txqTail = 0;          // free-running byte counters
static inline uint32_t txqUsed() { return g_txqHead - g_txqTail; }
static bool txqPush(const uint8_t *d, uint32_t n) {
    if (TXQ_SIZE - txqUsed() < n) return false;
    for (uint32_t i = 0; i < n; i++) g_txq[(g_txqHead + i) % TXQ_SIZE] = d[i];
    g_txqHead += n;
    return true;
}
// Writes up to maxBytes of queued data (one contiguous piece); returns bytes written.
static uint32_t txqWrite(uint32_t maxBytes) {
    uint32_t used = txqUsed();
    if (!used) return 0;
    uint32_t off = g_txqTail % TXQ_SIZE;
    uint32_t n = min(min(used, maxBytes), (uint32_t)(TXQ_SIZE - off));
    int room = Serial.availableForWrite();
    if (room <= 0) return 0;
    if ((uint32_t)room < n) n = room;
    n = Serial.write(g_txq + off, n);
    g_txqTail += n;
    return n;
}
static void txqFlushBlocking() {
    while (txqUsed()) { if (!txqWrite(256)) delay(1); }
}

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len) {
    uint8_t f[4 + 255 + 2];
    f[0] = FRAME_SYNC0; f[1] = FRAME_SYNC1; f[2] = type; f[3] = len;
    memcpy(f + 4, payload, len);
    uint16_t crc = crc16_fast(0xFFFF, f + 2, 2 + len);
    f[4 + len] = (uint8_t)(crc & 0xFF);
    f[5 + len] = (uint8_t)(crc >> 8);
    if (g_lk.streaming) {
        if (!txqPush(f, 6 + len)) g_lk.h.txDrop++;
    } else {
        Serial.write(f, 6 + len);
    }
}

// printf-style log that stays readable in text modes and stays framed in
// binary mode, so the host parser never sees stray text mid-stream.
static void logf(const char *fmt, ...) {
    char buf[240];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    if (g_mode == 'B') {
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) n--;
        send_frame(FT_TEXT, (const uint8_t *)buf, (uint8_t)n);
    } else {
        Serial.print(buf);
    }
}

static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

// INFO describes the scan in effect: the legacy one (mode 0) or a locked mode.
static void send_info() {
    bool lk = g_lk.mode != 0;
    uint8_t p[39];
    p[0] = PROTO_VERSION;
    p[1] = NUM_OUT;
    put_u16(p + 2, g_id);
    put_u16(p + 4, g_clockRb);
    put_u32(p + 6, g_clkinNow);
    put_u32(p + 10, lk ? g_clkinNow / 2 / g_lk.osr : FDATA_SPS);
    put_u32(p + 14, SERIAL_BAUD);
    put_u16(p + 18, lk ? g_lk.osr : OSR_NORMAL);
    put_u16(p + 20, lk ? g_lk.settleUs : SETTLE_US);
    p[22] = lk ? g_lk.discard : DISCARD_SAMPLES;
    p[23] = g_selMask;
    // version 4
    p[24] = g_lk.mode;
    put_u32(p + 25, lk ? g_lk.spiActual : SPI_HZ);
    put_u16(p + 29, lk ? g_lk.N : 0);
    for (int s = 0; s < NUM_SEL; s++)        // conversions averaged per select
        p[31 + s] = ((g_selMask >> s) & 1) ? (lk ? g_lk.avgPerSel[s] : 1) : 0;
    send_frame(FT_INFO, p, sizeof(p));
}

static inline bool isRailed(uint8_t o) { return labs(outRaw[o]) >= SAT_CODE; }

static void send_scan() {
    // seq u32, t_us u32, dur_us u16, railmask u32, flags u8, nch u8, 32 x i24
    uint8_t p[16 + 3 * NUM_OUT];
    uint32_t rail = 0;
    for (int o = 1; o <= NUM_OUT; o++) if (isRailed(o)) rail |= (1UL << (o - 1));
    put_u32(p + 0, g_scans);
    put_u32(p + 4, g_scanT0);
    put_u16(p + 8, g_scanDur > 0xFFFF ? 0xFFFF : (uint16_t)g_scanDur);
    put_u32(p + 10, rail);
    p[14] = (g_scanRecovered ? FLAG_RECOVERED : 0) | (g_scanCrcRetry ? FLAG_CRC_RETRY : 0) |
            (g_scanStale ? FLAG_STALE : 0) | (g_scanReinit ? FLAG_REINIT : 0) |
            (g_lk.mode ? g_lk.outFlags : 0);
    p[15] = NUM_OUT;
    for (int o = 1; o <= NUM_OUT; o++) {
        uint32_t c = (uint32_t)outRaw[o] & 0xFFFFFF;
        uint8_t *q = p + 16 + 3 * (o - 1);
        q[0] = c; q[1] = c >> 8; q[2] = c >> 16;
    }
    send_frame(FT_SCAN, p, sizeof(p));
}

// ── Whisker / connector helpers ──────────────────────────────────────────────
static inline uint8_t whiskerOfOut(uint8_t o) { return (o + 1) / 2; }      // 1..16
static inline char    gaugeOfOut  (uint8_t o) { return (o & 1) ? 'A' : 'B'; }
static inline uint8_t jOfOut      (uint8_t o) { return 2 + (o - 1) / 8; }  // J2..J5
static void connOfOut(uint8_t o, uint16_t *c1, uint16_t *c2,
                                 uint8_t *p1, uint8_t *p2) {
    uint16_t a = 2 * o - 1, b = 2 * o;
    uint16_t base = 16 * (jOfOut(o) - 2);          // 0, 16, 32, 48
    *c1 = a; *c2 = b;
    *p1 = CONN_TO_JPIN[a - base];
    *p2 = CONN_TO_JPIN[b - base];
}

// ─────────────────────────────────────────────────────────────────────────────
//  SPI / ADC primitives
// ─────────────────────────────────────────────────────────────────────────────
static void spi_word(uint16_t cmd) {
    cs_low();
    vspi.transfer((cmd >> 8) & 0xFF);
    vspi.transfer( cmd       & 0xFF);
    vspi.transfer(0x00);
    cs_high();
}

// RESET is latched only at the end of a complete frame; "Terminating the frame
// early causes the RESET command to be ignored. Six words are required"
// (SBAS890D 8.4.1.3). spi_word() sends one word, so it can't be used here.
static void ads_reset_cmd() {
    uint8_t f[18] = {(uint8_t)(CMD_RESET >> 8), (uint8_t)(CMD_RESET & 0xFF)};
    cs_low();
    vspi.writeBytes(f, sizeof(f));
    cs_high();
}

static void ads_write_reg(uint8_t reg, uint16_t val) {
    cs_low();
    vspi.transfer((CMD_WREG(reg) >> 8) & 0xFF);
    vspi.transfer( CMD_WREG(reg)       & 0xFF);
    vspi.transfer(0x00);
    vspi.transfer((val >> 8) & 0xFF);
    vspi.transfer( val       & 0xFF);
    vspi.transfer(0x00);
    cs_high();
    delayMicroseconds(50);
}

// The register value is returned in WORD 0 of the NEXT frame.
static uint16_t ads_read_reg(uint8_t reg) {
    cs_low();
    vspi.transfer((CMD_RREG(reg) >> 8) & 0xFF);
    vspi.transfer( CMD_RREG(reg)       & 0xFF);
    vspi.transfer(0x00);
    cs_high();
    delayMicroseconds(50);
    cs_low();
    uint16_t r  = (uint16_t)vspi.transfer(0x00) << 8;
    r          |= (uint16_t)vspi.transfer(0x00);
    vspi.transfer(0x00);
    cs_high();
    return r;
}

static inline bool edgePending() { return (int32_t)(g_drdyEdges - g_drdyTaken) > 0; }

// True when an unread conversion is (probably) waiting; cheap, claims nothing.
static inline bool drdyPending() { return edgePending() || digitalRead(PIN_DRDY) == LOW; }

// Forget edges nobody read (text-mode pauses, re-init). Data still sitting in
// the ADC's FIFO keeps DRDY low, so waitDRDY still finds it.
static inline void drdyResync() { g_drdyTaken = g_drdyEdges; }

// Wait for a conversion that no read has claimed yet:
//  - an unclaimed DRDY falling edge (the normal case), or
//  - DRDY held low with no unclaimed edge: data whose falling edge never came
//    because it completed during the previous read (SBAS890D 8.5.1.5: "The
//    DRDY pulse is blocked when new conversions complete while conversion
//    data are read"), or the second slot of the 2-deep output FIFO after a
//    pause (8.5.1.9.1). The 10 us re-check stops a fresh edge whose ISR has
//    not run yet from being taken for this case and then claimed twice.
// *idx = edge count of the claimed conversion; an edgeless one gets the count
// of the last claimed edge, i.e. it is never labelled newer than it is.
// *tConv = micros() of the edge, or "now" for an edgeless one.
static bool waitDRDY(uint32_t timeout_ms, uint32_t *idx = nullptr, uint32_t *tConv = nullptr) {
    uint32_t t0 = millis();
    for (;;) {
        if (edgePending()) {
            g_drdyTaken++;
            if (tConv) *tConv = g_drdyEdgeT;
            break;
        }
        if (digitalRead(PIN_DRDY) == LOW) {
            delayMicroseconds(10);
            if (edgePending()) continue;
            if (digitalRead(PIN_DRDY) == LOW) {
                if (tConv) *tConv = micros();
                break;
            }
        }
        if (millis() - t0 > timeout_ms) return false;
    }
    if (idx) *idx = g_drdyTaken;
    return true;
}

static uint32_t ads_read_all(long ch[4], bool *crcOk) {
    static const uint8_t zeros[18] = {0};   // NULL command + 5 don't-care words
    uint8_t buf[18];                        // STATUS, CH0..CH3, CRC (+pad)
    cs_low();
    vspi.transferBytes(zeros, buf, sizeof(buf));   // one call: no per-byte gaps
    cs_high();
    uint16_t rxCrc = ((uint16_t)buf[15] << 8) | buf[16];
    uint32_t status = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    for (int i = 0; i < 4; i++) {
        uint32_t raw = ((uint32_t)buf[3 + 3*i] << 16) |
                       ((uint32_t)buf[4 + 3*i] << 8)  |
                        (uint32_t)buf[5 + 3*i];
        if (raw & 0x800000UL) raw |= 0xFF000000UL;   // sign-extend
        ch[i] = (long)(int32_t)raw;
    }
    *crcOk = (crc16_ccitt_update(0xFFFF, buf, 15) == rxCrc);
    return status & 0xFFFFFF;
}

static bool statusValid(uint32_t st) { return st != 0x000000 && st != 0xFFFFFF; }

// ─────────────────────────────────────────────────────────────────────────────
//  Init + escalating halt recovery
// ─────────────────────────────────────────────────────────────────────────────
#if HAVE_HW_RESET
static void hardwareReset() {
    digitalWrite(PIN_RESET, LOW);  delay(2);
    digitalWrite(PIN_RESET, HIGH); delay(5);
}
#endif

static void spiFlush() {
    cs_low();
    for (int i = 0; i < 2 * 18; i++) vspi.transfer(0x00);   // 2 full frames
    cs_high();
    delayMicroseconds(100);
}

static void setClkin(uint32_t hz) {
    g_clkinNow = hz;
    ledcDetach(PIN_CLKIN);
    delay(5);
    ledcAttach(PIN_CLKIN, hz, 1);
    ledcWrite(PIN_CLKIN, 1);
    delay(20);
}

static void restartCLKIN() { setClkin(CLKIN_HZ); }

// clockReg: CLOCK value to write; the normal scan uses CLOCK_ALL_CH_OSR128,
// the 'r' burst other OSRs. Also clears the MODE RESET flag (see STATUS_RESET).
static bool initADC(uint16_t clockReg) {
    ads_reset_cmd();
    delay(50);
    drdyResync();
    waitDRDY(300);
    spi_word(CMD_UNLOCK);
    delayMicroseconds(50);
    ads_write_reg(REG_MODE, MODE_CLEAR_RESET);
    ads_write_reg(REG_CLOCK, clockReg);
    ads_write_reg(REG_GAIN1, GAIN_ALL_1);
    g_clockRb = ads_read_reg(REG_CLOCK);     // read back BEFORE locking
    g_modeRb  = ads_read_reg(REG_MODE);
    g_id      = ads_read_reg(REG_ID);
    spi_word(CMD_LOCK);
    delay(10);
    drdyResync();
    g_respPending = true;                    // next frame answers LOCK (0x0555)
    return (g_clockRb & CLOCK_MASK) == (clockReg & CLOCK_MASK);
}

// STATUS said RESET: the ADC is back at its power-on defaults. Re-run the boot
// init. At most one per REINIT_MIN_MS, so a MODE write that doesn't stick
// can't turn into a re-init storm (the counter and the log still show it).
#define REINIT_MIN_MS  1000
static void reinitADC() {
    g_reinitMs = millis();
    g_adcReinits++;
    g_scanReinit = true;
    bool ok = initADC();
    logf("[adc reset #%lu] ADC had reset (STATUS RESET=1), re-initialised: CLOCK=0x%04X %s"
         " MODE=0x%04X ID=0x%04X\n", (unsigned long)g_adcReinits, g_clockRb,
         ok ? "ok" : "NOT accepted", g_modeRb, g_id);
}

// One conversion that is alive (STATUS not 0/FFFFFF), CRC-clean and fresh
// (all STATUS DRDY bits set). A frame read right after a command carries the
// command's response in word 0 instead of STATUS, so that one is not checked.
static bool readValid(long ch[4], uint32_t budget_ms, uint32_t *idx, uint32_t *tConv) {
    uint32_t t0 = millis();
    while (millis() - t0 < budget_ms) {
        uint32_t k, t;
        if (!waitDRDY(50, &k, &t)) continue;
        bool crcOk = false;
        uint32_t st = ads_read_all(ch, &crcOk);
        if (!statusValid(st)) continue;
        if (!crcOk) {
            g_crcFails++;                     // looked alive, was noise
            g_scanCrcRetry = true;
            continue;
        }
        if (g_respPending) {
            g_respPending = false;
        } else if (((st >> 8) & STATUS_RESET) && g_reinitArmed &&
                   millis() - g_reinitMs >= REINIT_MIN_MS) {
            reinitADC();                      // this conversion was at the defaults
            continue;
        } else if (((st >> 8) & STATUS_DRDY_ALL) != STATUS_DRDY_ALL) {
            g_staleReads++;                   // no new conversion behind this read
            g_scanStale = true;
            continue;
        }
        if (idx)   *idx = k;
        if (tConv) *tConv = t;
        return true;
    }
    return false;
}

// Levels: 1 soft re-init · 2 SPI flush + re-init · 3 CLKIN restart (+HW reset)
static int recoverADC(long ch[4], uint32_t *idx = nullptr) {
    for (int level = 1; level <= 3; level++) {
        switch (level) {
            case 1: break;
            case 2: spiFlush(); break;
            case 3: restartCLKIN();
#if HAVE_HW_RESET
                    hardwareReset();
#endif
                    spiFlush();
                    break;
        }
        initADC();
        if (readValid(ch, 300, idx)) return level;
    }
    return 0;
}

// 0 = failed, 1 = read, 2 = read after a halt recovery (filters restarted),
// 3 = read after a reinitADC (the ADC had reset; the sweep must restart).
static int readOrRecover(long ch[4], uint32_t *idx = nullptr) {
    uint32_t reinits = g_adcReinits;
    if (readValid(ch, 250, idx)) return g_adcReinits != reinits ? 3 : 1;
    g_halts++;
    g_scanRecovered = true;
    int lvl = recoverADC(ch, idx);
    if (lvl > 0) {
        g_recByLevel[lvl]++;
        logf("[halt #%lu] recovered at L%d  (L1=%lu L2=%lu L3=%lu fail=%lu crcbad=%lu stale=%lu)\n",
             (unsigned long)g_halts, lvl,
             (unsigned long)g_recByLevel[1], (unsigned long)g_recByLevel[2],
             (unsigned long)g_recByLevel[3], (unsigned long)g_unrecovered,
             (unsigned long)g_crcFails, (unsigned long)g_staleReads);
        return g_adcReinits != reinits ? 3 : 2;
    }
    g_unrecovered++;
    logf("[halt #%lu] UNRECOVERED  ID=0x%04X CLOCKrb=0x%04X crcbad=%lu"
         "  -> power-cycle the PCB with the ESP32 already running\n",
         (unsigned long)g_halts, g_id, g_clockRb, (unsigned long)g_crcFails);
    delay(500);
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  MUX select
// ─────────────────────────────────────────────────────────────────────────────
// Both CD4097s are stepped together, so each select value delivers four
// signals (one per ADC channel) simultaneously. 8 selects x 4 = 32 channels.
static void mux_select(uint8_t sel) {
    digitalWrite(PIN_SELA_1, (sel >> 0) & 1);
    digitalWrite(PIN_SELB_1, (sel >> 1) & 1);
    digitalWrite(PIN_SELC_1, (sel >> 2) & 1);
    digitalWrite(PIN_SELA_2, (sel >> 0) & 1);
    digitalWrite(PIN_SELB_2, (sel >> 1) & 1);
    digitalWrite(PIN_SELC_2, (sel >> 2) & 1);
}

// With an internal pull-up on, a free line reads 1 and a ground tie reads 0.
static int probeSelectsGrounded() {
    const uint8_t p[] = { PIN_SELA_1, PIN_SELB_1, PIN_SELC_1,
                          PIN_SELA_2, PIN_SELB_2, PIN_SELC_2 };
    int lows = 0;
    for (uint8_t i = 0; i < 6; i++) pinMode(p[i], INPUT_PULLUP);
    delay(5);
    for (uint8_t i = 0; i < 6; i++) if (digitalRead(p[i]) == LOW) lows++;
    for (uint8_t i = 0; i < 6; i++) pinMode(p[i], INPUT);
    return lows;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Scan — sweep select 0..7, fill outRaw/outV[1..32]
// ─────────────────────────────────────────────────────────────────────────────
// Upper bound on reads per select before giving up on edge accounting (only
// reachable if DRDY edges keep getting swallowed; normally DISCARD + 1 or +2).
#define MAX_READS_PER_SEL  (4 * (DISCARD_SAMPLES + 1))

// One sweep: 1 = done, 0 = ADC halt not recovered, 3 = the ADC had reset and
// was re-initialised mid-sweep (earlier selects ran at the wrong config).
static int sweep() {
    long ch[4];
    drdyResync();
    g_scanT0 = micros();
    for (uint8_t sel = 0; sel < NUM_SEL; sel++) {
        if (!((g_selMask >> sel) & 1)) continue;
        mux_select(sel);
        uint32_t tSel = micros();
        // Settle, but keep reading (and dropping) conversions meanwhile, so
        // none is left in the FIFO to come back later as a "new" sample.
        while (micros() - tSel < SETTLE_US) {
            if (!drdyPending()) continue;
            int r = readOrRecover(ch);
            if (r == 0 || r == 3) return r;
        }
        // Keep the (DISCARD_SAMPLES+1)-th conversion that completes after the
        // settle: its sinc3 window (3 conversions) lies entirely after it.
        uint32_t mark = g_drdyEdges, k;
        uint8_t got = 0;
        for (int reads = 0; got <= DISCARD_SAMPLES; reads++) {
            int r = readOrRecover(ch, &k);
            if (r == 0 || r == 3) return r;
            if (r == 2) { mark = k; got = 0; continue; }   // filters restarted
            if ((int32_t)(k - mark) > 0) got++;
            else if (reads >= MAX_READS_PER_SEL) { g_staleReads++; g_scanStale = true; break; }
        }
        for (uint8_t a = 0; a < NUM_ADC_CH; a++) {
            uint8_t o = OUT_OF[a][sel];
            outRaw[o] = ch[a];
            outV  [o] = (ch[a] / ADC_FS) * VREF;
        }
    }
    return 1;
}

static bool lkScanOnce();
static bool scan_all() {
    if (g_lk.mode) return lkScanOnce();
    g_scanRecovered = false;
    g_scanCrcRetry  = false;
    g_scanStale     = false;
    g_scanReinit    = false;
    int r = sweep();
    if (r == 3) r = sweep();     // restart cleanly at the restored config
    if (r != 1) return false;    // (a second reset within REINIT_MIN_MS can't recur)
    g_scanDur = micros() - g_scanT0;
    g_scans++;
    return true;
}

static void captureBaseline(int nScans) {
    double sum[NUM_OUT + 1] = {}, sq[NUM_OUT + 1] = {};
    int got = 0;
    for (int i = 0; i < nScans; i++) {
        if (!scan_all()) continue;
        for (int o = 1; o <= NUM_OUT; o++) { sum[o] += outV[o]; sq[o] += (double)outV[o] * outV[o]; }
        got++;
    }
    if (got < 2) { logf("Baseline FAILED — no valid scans.\n"); return; }
    for (int o = 1; o <= NUM_OUT; o++) {
        double m = sum[o] / got, v = sq[o] / got - m * m;
        outBase [o] = (float)m;
        outNoise[o] = (float)(v > 0 ? sqrt(v) : 0);
    }
    logf("Baseline captured from %d scans.\n", got);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Text-mode printing (bench debugging; unchanged from whisker_reader.ino)
// ─────────────────────────────────────────────────────────────────────────────
static void print_map() {
    Serial.println("\n=========== connector / channel map ===========");
    Serial.println(" W   gauge  OUT   CONN pair      J pins    ADC   sel");
    for (uint8_t w = 1; w <= NUM_WHISKER; w++) {
        for (uint8_t g = 0; g < 2; g++) {
            uint8_t o = 2 * w - 1 + g;
            uint16_t c1, c2; uint8_t p1, p2;
            connOfOut(o, &c1, &c2, &p1, &p2);
            int achan = -1, asel = -1;
            for (int a = 0; a < NUM_ADC_CH && achan < 0; a++)
                for (int s = 0; s < NUM_SEL; s++)
                    if (OUT_OF[a][s] == o) { achan = a; asel = s; break; }
            Serial.printf(" %2u    %c   OUT%-2u  CONN%-2u/CONN%-2u  J%u.%-2u+%-2u  CH%d   %d\n",
                          w, gaugeOfOut(o), o, c1, c2, jOfOut(o), p1, p2, achan, asel);
        }
    }
    Serial.println("===============================================");
    Serial.println("Whisker W uses 4 header pins: gauge A + gauge B.");
}

static void printCell(char g, uint8_t o) {
    if (isRailed(o))
        Serial.printf("  %c(OUT%-2u) %s open  ", g, o, outRaw[o] > 0 ? "+rail" : "-rail");
    else if (g_absolute)
        Serial.printf("  %c(OUT%-2u) %+9.6f V", g, o, outV[o]);
    else
        Serial.printf("  %c(OUT%-2u) %+8.3f mV", g, o, (outV[o] - outBase[o]) * 1000.0f);
}

static void print_scan() {
    int railed = 0;
    for (int o = 1; o <= NUM_OUT; o++) if (isRailed(o)) railed++;
    Serial.printf("\n--- scan #%lu   %s   (halts=%lu crcbad=%lu stale=%lu, %d/%d open) ---\n",
                  (unsigned long)g_scans,
                  g_absolute ? "absolute V at ADC" : "deviation from baseline, mV",
                  (unsigned long)g_halts, (unsigned long)g_crcFails,
                  (unsigned long)g_staleReads, railed, NUM_OUT);
    for (uint8_t w = 1; w <= NUM_WHISKER; w++) {
        Serial.printf("  W%-2u", w);
        printCell('A', 2 * w - 1);
        printCell('B', 2 * w);
        Serial.println();
    }
}

static inline float devMv(uint8_t o) { return (outV[o] - outBase[o]) * 1000.0f; }

// Grows instantly, decays slowly, so a one-off tap doesn't leave it zoomed out.
static void updateScale() {
    float peak = 0;
    for (int o = 1; o <= NUM_OUT; o++)
        if (!isRailed(o)) {
            float d = fabsf(devMv(o));
            if (d > peak) peak = d;
        }
    if (!g_autoScale) return;
    if (peak > g_scale)        g_scale = peak * 1.15f;
    else if (g_scale > SCALE_MIN) {
        g_scale *= 0.98f;
        if (g_scale < SCALE_MIN) g_scale = SCALE_MIN;
    }
}

static void print_profile() {
    updateScale();
    int8_t n[NUM_OUT + 1];
    int open = 0;
    for (int o = 1; o <= NUM_OUT; o++) {
        if (isRailed(o)) { n[o] = INT8_MIN; open++; continue; }
        int v = (int)lroundf(devMv(o) / g_scale * BAR_H);
        if (v >  BAR_H) v =  BAR_H;
        if (v < -BAR_H) v = -BAR_H;
        n[o] = (int8_t)v;
    }
    Serial.printf("\n== scan %lu | %d/%d open | scale +/-%.2f mV | halts %lu crc %lu stale %lu ==\n",
                  (unsigned long)g_scans, open, NUM_OUT, g_scale,
                  (unsigned long)g_halts, (unsigned long)g_crcFails, (unsigned long)g_staleReads);
    char row[NUM_OUT * COL_W + 1];
    for (int r = BAR_H; r >= -BAR_H; r--) {
        for (int i = 0; i < NUM_OUT; i++) {
            int  o    = i + 1;
            bool op   = (n[o] == INT8_MIN);
            char fill = ' ';
            if (r == 0)                              fill = op ? ':' : '-';
            else if (!op && n[o] > 0 && r >= 1  && r <= n[o]) fill = '#';
            else if (!op && n[o] < 0 && r <= -1 && r >= n[o]) fill = '#';
            row[i * COL_W + 0] = fill;
            row[i * COL_W + 1] = fill;
            row[i * COL_W + 2] = (r == 0) ? fill : ' ';
        }
        row[NUM_OUT * COL_W] = '\0';
        if      (r ==  BAR_H) Serial.printf("%+6.1f |%s\n",  g_scale, row);
        else if (r == -BAR_H) Serial.printf("%+6.1f |%s\n", -g_scale, row);
        else if (r ==  0)     Serial.printf("%6.1f +%s\n",   0.0f,    row);
        else                  Serial.printf("       |%s\n",           row);
    }
    Serial.printf("%*s", AXIS_W, "");
    for (uint8_t w = 1; w <= NUM_WHISKER; w++) Serial.printf("W%02u   ", w);
    Serial.println();
    Serial.printf("%*s", AXIS_W, "");
    for (uint8_t w = 1; w <= NUM_WHISKER; w++)
        Serial.printf(" %c  %c ", isRailed(2*w - 1) ? 'a' : 'A',
                                  isRailed(2*w)     ? 'b' : 'B');
    Serial.println();
    int shown = 0;
    for (int o = 1; o <= NUM_OUT; o++) {
        if (isRailed(o)) continue;
        if (shown % 6 == 0) Serial.print(shown ? "\n      " : "live: ");
        Serial.printf(" W%02u%c%+7.2f", whiskerOfOut(o), gaugeOfOut(o), devMv(o));
        shown++;
    }
    Serial.println(shown ? "" : "live: (none — every channel is open)");
}

#define CELL_H  5
static void print_cell(uint8_t o) {
    Serial.printf("W%02u%c ", whiskerOfOut(o), gaugeOfOut(o));
    if (isRailed(o)) { Serial.print("[   open    ]         "); return; }
    float mv    = devMv(o);
    float units = mv / g_scale * CELL_H;
    bool  over  = units >  CELL_H + 0.5f;
    bool  under = units < -CELL_H - 0.5f;
    int   n     = (int)lroundf(units);
    if (n >  CELL_H) n =  CELL_H;
    if (n < -CELL_H) n = -CELL_H;
    char b[2 * CELL_H + 2];
    for (int k = -CELL_H; k <= CELL_H; k++) {
        char c = ' ';
        if      (k == 0)                        c = '|';
        else if (k < 0 && n < 0 && k >= n)      c = '=';
        else if (k > 0 && n > 0 && k <= n)      c = '=';
        b[k + CELL_H] = c;
    }
    if (over)  b[2 * CELL_H] = '>';
    if (under) b[0]          = '<';
    b[2 * CELL_H + 1] = '\0';
    Serial.printf("[%s] %+7.2f ", b, mv);
}

static void print_grid() {
    updateScale();
    int open = 0, live = 0, peakOut = 0;
    float peak = 0;
    for (int o = 1; o <= NUM_OUT; o++) {
        if (isRailed(o)) { open++; continue; }
        live++;
        if (fabsf(devMv(o)) > fabsf(peak)) { peak = devMv(o); peakOut = o; }
    }
    Serial.printf("\n===== scan %lu | %d live, %d open | bar = +/-%.2f mV"
                  " | halts %lu crc %lu stale %lu =====\n",
                  (unsigned long)g_scans, live, open, g_scale,
                  (unsigned long)g_halts, (unsigned long)g_crcFails,
                  (unsigned long)g_staleReads);
    if (peakOut)
        Serial.printf("  largest: W%02u gauge %c  %+.2f mV\n",
                      whiskerOfOut(peakOut), gaugeOfOut(peakOut), peak);
    else
        Serial.println("  largest: (nothing connected)");
    Serial.println("      whiskers 1-8                                "
                   "  whiskers 9-16");
    for (uint8_t r = 1; r <= 8; r++) {
        print_cell(2 * r - 1);
        print_cell(2 * r);
        Serial.print(" ");
        print_cell(2 * (r + 8) - 1);
        print_cell(2 * (r + 8));
        Serial.println();
    }
}

static void noise_report() {
    Serial.println("\n=== noise floor (hands off) — baseline mV / stddev uV ===");
    for (uint8_t w = 1; w <= NUM_WHISKER; w++) {
        uint8_t oa = 2 * w - 1, ob = 2 * w;
        Serial.printf("  W%-2u  A ", w);
        if (isRailed(oa)) Serial.printf("     open        ");
        else              Serial.printf("%+7.2f mV /%6.0f uV", outBase[oa] * 1000.0f, outNoise[oa] * 1e6f);
        Serial.printf("    B ");
        if (isRailed(ob)) Serial.printf("     open\n");
        else              Serial.printf("%+7.2f mV /%6.0f uV\n", outBase[ob] * 1000.0f, outNoise[ob] * 1e6f);
    }
    Serial.printf("  A whisker swing is ~1-50 mV at the ADC (INA351 gain ~%.0f).\n", INA_GAIN);
}

// ── Calibration: deflect ONE whisker, it names the channel that moved ────────
#define CAL_MIN_MV  0.5f
#define CAL_SIGMA   6.0f
#define CAL_MARGIN  2.5f

static void calibrate_step() {
    if (!scan_all()) return;
    float best = 0, second = 0; int bo = -1;
    for (int o = 1; o <= NUM_OUT; o++) {
        float d = fabsf(outV[o] - outBase[o]);
        if (d > best) { second = best; best = d; bo = o; }
        else if (d > second) second = d;
    }
    if (bo < 0 || isRailed(bo)) return;
    float floorMv = max(CAL_MIN_MV, outNoise[bo] * CAL_SIGMA * 1000.0f);
    if (best * 1000.0f < floorMv) return;
    if (second > 0 && best < second * CAL_MARGIN) {
        Serial.printf("  ambiguous: OUT%d %+.3f mV vs next %+.3f mV — deflect ONE gauge only\n",
                      bo, (outV[bo] - outBase[bo]) * 1000.0f, second * 1000.0f);
        return;
    }
    uint16_t c1, c2; uint8_t p1, p2;
    connOfOut(bo, &c1, &c2, &p1, &p2);
    Serial.printf(">>> whisker %u gauge %c  (OUT%u, CONN%u/CONN%u, J%u pins %u+%u)  %+8.3f mV  %.1f sigma\n",
                  whiskerOfOut(bo), gaugeOfOut(bo), bo, c1, c2, jOfOut(bo), p1, p2,
                  (outV[bo] - outBase[bo]) * 1000.0f,
                  outNoise[bo] > 0 ? best / outNoise[bo] : 0.0f);
}

// ── Settling diagnostic ('d'): what a select change really costs ────────────
// For each ADC channel: dwell on a "from" select where that channel is railed
// (or, failing that, furthest from the target), switch to the target select
// right after a read, then read EVERY conversion for DIAG_WINDOW_US with no
// discards. Repeat DIAG_REPS times; per conversion index k print the mean time
// since the switch and the mean deviation from that rep's final value (mean
// of the last ms). A conversion whose t >= 3 x 320 us ("post" in the table)
// has its whole sinc3 window after the switch, so any residual left there is
// analog (MUX/INA351/RC) settling, not the digital filter.
#define DIAG_REPS       32
#define DIAG_DWELL_US   3000
#define DIAG_WINDOW_US  5000
#define DIAG_FINAL_US   1000      // last part of the window = "settled" value
#define DIAG_MAXK       24
#define CONV_US         (1000000UL / FDATA_SPS)

static inline float codeMv(long c) { return (c / ADC_FS) * VREF * 1000.0f; }

// Read every conversion for `us` microseconds; returns false on an ADC halt.
static bool diag_dwell(uint32_t us, long ch[4]) {
    uint32_t t0 = micros();
    do { if (!readOrRecover(ch)) return false; } while (micros() - t0 < us);
    return true;
}

static uint8_t diag_from_select(uint8_t a, uint8_t tgt) {
    uint8_t ot = OUT_OF[a][tgt];
    int best = -1; long bestD = -1;
    for (uint8_t s = 0; s < NUM_SEL; s++) {
        if (s == tgt) continue;
        long d = labs(outRaw[OUT_OF[a][s]] - outRaw[ot]);
        if (isRailed(OUT_OF[a][s])) d += 1L << 26;     // railed beats anything
        if (d > bestD) { bestD = d; best = s; }
    }
    return (uint8_t)best;
}

static void settle_diag(uint8_t tgt) {
    long ch[4];
    if (!scan_all()) { Serial.println("settle diag: ADC not responding"); return; }
    Serial.printf("\n=== mux settling diagnostic -> select %u  (%d reps, %lu us window,"
                  " conversion %lu us, SETTLE_US %d, discards %d) ===\n",
                  tgt, DIAG_REPS, (unsigned long)DIAG_WINDOW_US, (unsigned long)CONV_US,
                  SETTLE_US, DISCARD_SAMPLES);
    for (uint8_t a = 0; a < NUM_ADC_CH; a++) {
        uint8_t from = diag_from_select(a, tgt);
        uint8_t of = OUT_OF[a][from], ot = OUT_OF[a][tgt];
        static double sT[DIAG_MAXK], sD[DIAG_MAXK], sD2[DIAG_MAXK], sV[DIAG_MAXK];
        static int32_t tMin[DIAG_MAXK];
        static uint16_t n[DIAG_MAXK];
        for (int k = 0; k < DIAG_MAXK; k++) {
            sT[k] = sD[k] = sD2[k] = sV[k] = 0; n[k] = 0; tMin[k] = INT32_MAX;
        }
        double sPre = 0, sFin = 0, sFin2 = 0;
        int reps = 0;
        uint32_t staleBefore = g_staleReads;
        for (int r = 0; r < DIAG_REPS; r++) {
            float v[DIAG_MAXK]; int32_t t[DIAG_MAXK]; int nk = 0;
            mux_select(from);
            if (!diag_dwell(DIAG_DWELL_US, ch)) return;     // ends right after a read
            float pre = codeMv(ch[a]);
            mux_select(tgt);
            uint32_t tSw = micros(), tc;
            while (micros() - tSw < DIAG_WINDOW_US) {
                if (!readValid(ch, 250, nullptr, &tc)) { Serial.println("  ADC halt — aborted"); return; }
                if (nk < DIAG_MAXK) { v[nk] = codeMv(ch[a]); t[nk] = (int32_t)(tc - tSw); nk++; }
            }
            double fin = 0; int nf = 0;
            for (int k = 0; k < nk; k++)
                if (t[k] >= (int32_t)(DIAG_WINDOW_US - DIAG_FINAL_US)) { fin += v[k]; nf++; }
            if (!nf) continue;
            fin /= nf;
            for (int k = 0; k < nk; k++) {
                double d = v[k] - fin;
                sT[k] += t[k]; sV[k] += v[k]; sD[k] += d; sD2[k] += d * d; n[k]++;
                if (t[k] < tMin[k]) tMin[k] = t[k];
            }
            sPre += pre; sFin += fin; sFin2 += fin * fin; reps++;
        }
        if (reps < 2) { Serial.printf("  CH%u: no usable reps\n", a); continue; }
        double pre = sPre / reps, fin = sFin / reps;
        double finSd = sqrt(fmax(0.0, sFin2 / reps - fin * fin));
        double step = fin - pre;
        Serial.printf("\n-- CH%u: sel %u (OUT%u W%02u%c%s) -> sel %u (OUT%u W%02u%c%s), %d reps --\n",
                      a, from, of, whiskerOfOut(of), gaugeOfOut(of), isRailed(of) ? " railed" : "",
                      tgt, ot, whiskerOfOut(ot), gaugeOfOut(ot), isRailed(ot) ? " railed" : "", reps);
        Serial.printf("   before %+9.3f mV   after %+9.3f mV   step %+9.3f mV   final sd %.3f mV"
                      "   stale reads %lu\n", pre, fin, step, finSd,
                      (unsigned long)(g_staleReads - staleBefore));
        Serial.println("    k   t_us  t_min  window   mean_mV    dev_mV  dev_sd_mV  dev/step");
        int settledK = -1;
        for (int k = 0; k < DIAG_MAXK; k++) {
            if (n[k] < reps / 2) break;
            double mt = sT[k] / n[k], md = sD[k] / n[k];
            double sd = sqrt(fmax(0.0, sD2[k] / n[k] - md * md));
            const char *win = tMin[k] >= (int32_t)(3 * CONV_US) ? "post"
                            : (sT[k] / n[k] >= 3.0 * CONV_US ? "most" : "part");
            Serial.printf("   %2d  %5.0f  %5ld  %-6s %+9.3f %+9.3f  %9.3f  %+8.4f\n",
                          k, mt, (long)tMin[k], win, sV[k] / n[k], md, sd,
                          fabs(step) > 1e-9 ? md / step : 0.0);
            if (settledK < 0 && fabs(md) <= fmax(3.0 * finSd, 0.05)) settledK = k;
            else if (settledK >= 0 && fabs(md) > fmax(3.0 * finSd, 0.05)) settledK = -1;
        }
        if (settledK >= 0)
            Serial.printf("   settled (|dev| <= max(3 x final sd, 0.05 mV) from here on) at k=%d,"
                          " t ~ %.0f us after the switch\n", settledK, sT[settledK] / n[settledK]);
        else
            Serial.println("   did not settle within the window");
    }
    Serial.printf("\n  scan_all keeps a conversion that ends >= SETTLE_US + 4 x %lu us after the"
                  " switch.\n  'post' rows: whole sinc3 window after the switch (residual = analog)."
                  "\n", (unsigned long)CONV_US);
}

static void settle_diag_cmd() {
    Serial.println("\nsettle diag: target select 0-7? (10 s)");
    uint32_t t0 = millis();
    while (millis() - t0 < 10000) {
        if (!Serial.available()) { delay(5); continue; }
        char c = Serial.read();
        if (c == '\r' || c == '\n') continue;
        if (c >= '0' && c <= '7') { settle_diag(c - '0'); return; }
        break;
    }
    Serial.println("settle diag: cancelled");
}

// ── Raw burst capture ('r'): find the true frequency of interference ─────────
// The scan samples each gauge at ~76 Hz, so anything faster folds down
// (the 6-7 Hz common tone, the 16.25 / 27.5 Hz ROV tones). A burst holds ONE
// select and keeps EVERY conversion of all 4 ADC channels (no discards, no
// mux switching) with the DRDY edge micros(), at an OSR (and optionally a
// CLKIN) of your choice, into RAM, then dumps it as text:
//   r<sel> [n] [osr] [clkin_hz] [spi_hz]   terminated by '\n' or ';'
//   defaults n 8192, osr 128, clkin 800000, spi 600000. Example "r2 8192 1024;"
// Dump (all lines '\n'-terminated, nothing else interleaved):
//   #BURST v1 sel=.. n=.. osr=.. clkin_hz=.. fdata_sps=.. spi_hz=.. clock_rb=0x....
//          out=o0,o1,o2,o3 mv_per_code=.. crc_bad=.. stale=.. overrun=.. edgeless=..
//          adc_reset=..
//   t_us flags c0 c1 c2 c3          one line per conversion; t_us relative to
//                                   the first, codes = raw signed 24 bit
//   #END lines=<n> crc=<hex>        CRC-16-CCITT (init FFFF) over the data lines
//   or "#BURSTERR <reason>" alone.
// flags: bit0 CRC bad, bit1 STATUS DRDY not all set (stale), bit2 >1 DRDY
// edge pending at the claim (conversions were missed: the read can't keep up),
// bit3 edgeless (claimed off the DRDY level, t_us = read time not edge time),
// bit4 STATUS RESET set.
// Each conversion is one 18-byte SPI read, so its period must exceed the read:
// at 600 kHz SPI that is ~260 us, i.e. fDATA <= ~3.5 kSPS. With CLKIN 8 MHz
// use OSR >= 2048 or raise spi_hz (2-8 MHz; check crc_bad). CLKIN must divide
// 80 MHz (LEDC, 1-bit duty); 8.192 MHz is not reachable, 8 MHz is, and the ADC
// allows 0.3-8.4 MHz in high-resolution mode (SBAS890D 6.3).
// Afterwards CLKIN, SPI and the OSR-128 config are restored (same init as
// boot) and the previous mode resumes (binary mode drops to the grid).
#define BURST_MAX       16384
#define BURST_DEFAULT   8192
#define BURST_SKIP      8        // conversions dropped after init (filter + mux settle)
#define BF_CRC      0x01
#define BF_STALE    0x02
#define BF_OVERRUN  0x04
#define BF_EDGELESS 0x08
#define BF_RESET    0x10

// ── Temporary ADC configs for the diagnostics (r, D, p) ─────────────────────
// SBAS890D 6.6: tc(SC) >= 40 ns, i.e. SCLK <= 25 MHz. Past ~8 MHz the jumper
// wiring, not the chip, sets the limit; 'p' measures it.
#define SPI_MAX_HZ  20000000L

static int osrCode(long osr) {
    for (int c = 0; c < 8; c++) if (osr == (128L << c)) return c;
    return -1;
}

static void setSpi(uint32_t hz) {
    g_spiNow = hz;
    vspi.endTransaction();
    vspi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE1));
}

static const char *fastCfgCheck(long clkin, long osr, long spi) {
    if (osrCode(osr) < 0)                  return "osr must be 128..16384, power of 2";
    if (clkin < 300000 || clkin > 8400000) return "clkin out of 0.3-8.4 MHz";
    if (spi < 100000 || spi > SPI_MAX_HZ)  return "spi_hz out of 0.1-20 MHz";
    return nullptr;
}

// CLKIN, SPI clock and OSR, then the full ADC init. false: CLOCK not accepted.
static bool fastCfgApply(long clkin, long osr, long spi, FastCfg *c) {
    c->clkin = clkin; c->osr = osr; c->spi = spi;
    if ((uint32_t)clkin != g_clkinNow) setClkin((uint32_t)clkin);
    if ((uint32_t)spi != g_spiNow) setSpi((uint32_t)spi);
    uint16_t clockReg = (CLOCK_ALL_CH_OSR128 & ~CLOCK_OSR_MASK) | (osrCode(osr) << 2);
    bool ok = initADC(clockReg);
    c->clockRb   = g_clockRb;
    c->clkActual = ledcReadFreq(PIN_CLKIN);
    if (c->clkActual == 0) c->clkActual = (uint32_t)clkin;
    c->spiActual = spiClockDivToFrequency(vspi.bus(), spiGetClockDiv(vspi.bus()));
    c->fdata     = c->clkActual / 2.0f / osr;
    c->periodUs  = 1e6f / c->fdata;
    c->timeoutMs = (uint32_t)(2 * c->periodUs / 1000) + 50;
    return ok;
}

// Back to the normal scan config (0.8 MHz, SPI_HZ, OSR 128), same init as boot.
static void restoreNormalCfg() {
    if (g_clkinNow != CLKIN_HZ) setClkin(CLKIN_HZ);
    if (g_spiNow != SPI_HZ) setSpi(SPI_HZ);
    initADC();
    drdyResync();
}

// One conversion (struct Conv), read as soon as it is announced, with the
// BF_* flags. readUs = the SPI read (CS low to CS high), latUs = DRDY edge to
// end of read. When more than one edge is pending, reads fell behind: the
// latest edge is claimed and the skipped count goes to `missed`.
static bool readConv(uint32_t timeoutMs, Conv *c) {
    c->f = 0; c->missed = 0;
    int32_t pend = (int32_t)(g_drdyEdges - g_drdyTaken);
    if (pend > 1) {
        c->f |= BF_OVERRUN;
        c->missed = pend - 1 > 255 ? 255 : pend - 1;
        g_drdyTaken = g_drdyEdges - 1;
    }
    uint32_t taken = g_drdyTaken;
    if (!waitDRDY(timeoutMs, nullptr, &c->t)) return false;
    if (g_drdyTaken == taken) c->f |= BF_EDGELESS;     // waitDRDY's level path
    bool crcOk;
    uint32_t r0 = micros();
    uint32_t st = ads_read_all(c->ch, &crcOk);
    uint32_t r1 = micros();
    c->readUs = (uint16_t)min<uint32_t>(r1 - r0, 65535);
    c->latUs  = (uint16_t)min<uint32_t>(r1 - c->t, 65535);
    if (g_respPending) { g_respPending = false; return true; }   // word 0 = LOCK reply
    if (!crcOk) { c->f |= BF_CRC; return true; }                 // STATUS untrustworthy
    if (((st >> 8) & STATUS_DRDY_ALL) != STATUS_DRDY_ALL) c->f |= BF_STALE;
    if ((st >> 8) & STATUS_RESET) c->f |= BF_RESET;
    return true;
}

static void put_line_crc(const char *line, uint16_t *crc) {
    size_t n = strlen(line);
    *crc = crc16_ccitt_update(*crc, (const uint8_t *)line, (int)n);
    Serial.write((const uint8_t *)line, n);
}

// Command arguments: everything up to ';' or a newline (5 s timeout). An empty
// line is only ended by ';' (or the timeout).
static void readCmdLine(char *buf, int size) {
    int len = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 5000 && len < size - 1) {
        if (!Serial.available()) { delay(1); continue; }
        char c = Serial.read();
        if (c == ';') break;
        if (c == '\n' || c == '\r') { if (len) break; else continue; }
        buf[len++] = c;
    }
    buf[len] = '\0';
}

// Up to maxn integers from a command line; returns how many were given.
static int parseLongs(const char *buf, long *v, int maxn) {
    int n = 0;
    const char *p = buf;
    while (n < maxn) {
        char *end;
        long x = strtol(p, &end, 10);
        if (end == p) break;
        v[n++] = x;
        p = end;
        while (*p == ' ' || *p == ',') p++;
    }
    return n;
}

static void burst_capture(int sel, long n, long osr, long clkin, long spiHz) {
    const char *err = fastCfgCheck(clkin, osr, spiHz);
    if (sel < 0 || sel >= NUM_SEL) err = "bad select";
    if (err) { Serial.printf("#BURSTERR %s\n", err); return; }
    if (n < 16) n = 16;
    if (n > BURST_MAX) n = BURST_MAX;
    // Per conversion: 4 x i24 codes (12 B), t (4 B), flags (1 B), in three
    // blocks. Shrink n until they fit, keeping 16 kB of heap for everything else.
    uint8_t *codes = nullptr; uint32_t *tt = nullptr; uint8_t *fl = nullptr;
    for (; n >= 16; n -= n / 8) {
        size_t big = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        if ((size_t)n * 12 + 16384 > big) continue;
        codes = (uint8_t *)malloc((size_t)n * 12);
        tt    = (uint32_t *)malloc((size_t)n * 4);
        fl    = (uint8_t *)malloc((size_t)n);
        if (codes && tt && fl) break;
        free(codes); free(tt); free(fl); codes = nullptr; tt = nullptr; fl = nullptr;
    }
    if (!codes) { Serial.print("#BURSTERR out of RAM\n"); return; }

    mux_select(sel);
    FastCfg cfg;
    bool cfgOk = fastCfgApply(clkin, osr, spiHz, &cfg);

    uint32_t nCrc = 0, nStale = 0, nOver = 0, nEdgeless = 0, nReset = 0;
    bool fail = !cfgOk;
    long got = 0;
    for (long i = -BURST_SKIP; i < n && !fail; i++) {
        Conv c;
        if (!readConv(cfg.timeoutMs, &c)) { fail = true; break; }
        if (i < 0) continue;
        uint8_t *q = codes + 12 * i;
        for (int a = 0; a < 4; a++) {
            uint32_t v = (uint32_t)c.ch[a] & 0xFFFFFF;
            q[3*a] = v; q[3*a + 1] = v >> 8; q[3*a + 2] = v >> 16;
        }
        tt[i] = c.t; fl[i] = c.f;
        nCrc += !!(c.f & BF_CRC); nStale += !!(c.f & BF_STALE); nOver += !!(c.f & BF_OVERRUN);
        nEdgeless += !!(c.f & BF_EDGELESS); nReset += !!(c.f & BF_RESET);
        got = i + 1;
    }

    restoreNormalCfg();          // before the (slow) dump

    if (fail && got == 0) {
        Serial.printf("#BURSTERR %s\n", cfgOk ? "ADC not converting" : "CLOCK config not accepted");
    } else {
        char line[160];
        snprintf(line, sizeof(line),
                 "#BURST v1 sel=%d n=%ld osr=%ld clkin_hz=%lu fdata_sps=%.3f spi_hz=%ld"
                 " clock_rb=0x%04X out=%u,%u,%u,%u", sel, got, osr,
                 (unsigned long)cfg.clkActual, cfg.clkActual / 2.0 / osr, spiHz, cfg.clockRb,
                 OUT_OF[0][sel], OUT_OF[1][sel], OUT_OF[2][sel], OUT_OF[3][sel]);
        Serial.print(line);
        Serial.printf(" mv_per_code=%.12g crc_bad=%lu stale=%lu overrun=%lu edgeless=%lu"
                      " adc_reset=%lu%s\n", (double)VREF * 1000.0 / ADC_FS,
                      (unsigned long)nCrc, (unsigned long)nStale, (unsigned long)nOver,
                      (unsigned long)nEdgeless, (unsigned long)nReset,
                      fail ? " truncated=1" : "");
        uint16_t crc = 0xFFFF;
        uint32_t t0 = got ? tt[0] : 0;
        for (long i = 0; i < got; i++) {
            const uint8_t *q = codes + 12 * i;
            long c[4];
            for (int a = 0; a < 4; a++) {
                uint32_t v = q[3*a] | ((uint32_t)q[3*a + 1] << 8) | ((uint32_t)q[3*a + 2] << 16);
                if (v & 0x800000UL) v |= 0xFF000000UL;
                c[a] = (long)(int32_t)v;
            }
            snprintf(line, sizeof(line), "%lu %u %ld %ld %ld %ld\n",
                     (unsigned long)(tt[i] - t0), fl[i], c[0], c[1], c[2], c[3]);
            put_line_crc(line, &crc);
        }
        Serial.printf("#END lines=%ld crc=%04X\n", got, crc);
    }
    free(codes); free(tt); free(fl);
    Serial.flush();
    drdyResync();
}

static void burst_cmd() {
    char buf[64];
    readCmdLine(buf, sizeof(buf));
    long v[5] = {-1, BURST_DEFAULT, OSR_NORMAL, CLKIN_HZ, SPI_HZ};
    if (parseLongs(buf, v, 5) < 1) {
        Serial.print("#BURSTERR usage: r<sel> [n] [osr] [clkin_hz] [spi_hz] then ';' or newline\n");
        return;
    }
    burst_capture((int)v[0], v[1], v[2], v[3], v[4]);
}

// ── Fine mux settle diagnostic ('D') ─────────────────────────────────────────
// What a select change costs at fast-scan rates. Expected limit: charge
// sharing, not the mux (~0.1 us): at the switch the COM node (~30 pF) dumps
// the previous channel's voltage into the new channel's 1 k / 0.1 uF filter,
// a kick of ~3e-4 x the voltage difference that recovers with tau ~100 us.
//   D<from> <to> [clkin] [osr] [spi] [reps] [window_us] [dither] [dwell_us] [jitter_us]
//   defaults 8000000 256 4000000 64 2000 4 3000 1000.  Example "D6 2 8000000 256 4000000;"
// Per rep: dwell on <from> for dwell_us + a random 0..jitter_us (esp_random)
// reading every conversion. A fixed rep spacing is a multiple of the period of
// any board line at k/dwell (1 kHz with 3000 us), which then adds up coherently
// across reps and fakes a tail; the random spacing averages it out. Then wait
// (rep % dither) / dither of a conversion period (so the switch lands at
// different phases and the host can resolve below one period), switch to
// <to>, read every conversion for window_us. Pick a <from> where the channel
// is far from <to> (a railed slot), so the tail is large enough to see.
// Output (lines '\n'-terminated):
//   #SETTLE v1 from= to= out_from=a,b,c,d out_to=a,b,c,d clkin_hz= osr= fdata_sps=
//              period_us= spi_hz= spi_actual= reps= window_us= dither= dwell_us=
//              jitter_us= mux_us= clock_rb= mv_per_code=
//   rep t_us flags c0 c1 c2 c3   t_us = DRDY edge (end of conversion) minus the
//                                start of mux_select; the last 4 conversions
//                                before the switch come first (t_us < 0)
//   #END lines= crc= crc_bad= stale= overrun= missed= edgeless= reset= read_us_max=
//        lat_us_max= [truncated=1]      counts cover the recorded conversions
// Host side: msd/esp32/adc_diag.py settle (averages, ideal sinc3 step, fit).
#define SD_MAXK  512

static void settle_fine(int from, int to, long clkin, long osr, long spi, long reps,
                        long windowUs, long dither, long dwellUs, long jitterUs) {
    const char *err = fastCfgCheck(clkin, osr, spi);
    if (from < 0 || from >= NUM_SEL || to < 0 || to >= NUM_SEL) err = "bad select";
    if (reps < 1 || reps > 4096)                 err = "reps out of 1-4096";
    if (windowUs < 50 || windowUs > 50000)       err = "window_us out of 50-50000";
    if (dither < 1 || dither > 64)               err = "dither out of 1-64";
    if (dwellUs < 100 || dwellUs > 200000)       err = "dwell_us out of 100-200000";
    if (jitterUs < 0 || jitterUs > 100000)       err = "jitter_us out of 0-100000";
    if (err) { Serial.printf("#SETTLEERR %s\n", err); return; }
    Conv *buf = (Conv *)malloc(sizeof(Conv) * (SD_MAXK + 4));
    if (!buf) { Serial.print("#SETTLEERR out of RAM\n"); return; }

    FastCfg cfg;
    mux_select(from);
    bool ok = fastCfgApply(clkin, osr, spi, &cfg);
    if (!ok) {
        restoreNormalCfg(); free(buf);
        Serial.printf("#SETTLEERR CLOCK config not accepted (readback 0x%04X)\n", cfg.clockRb);
        return;
    }
    uint32_t m0 = micros(); mux_select(from); uint32_t muxUs = micros() - m0;
    Serial.printf("#SETTLE v1 from=%d to=%d out_from=%u,%u,%u,%u out_to=%u,%u,%u,%u"
                  " clkin_hz=%lu osr=%ld fdata_sps=%.3f period_us=%.3f spi_hz=%ld spi_actual=%lu"
                  " reps=%ld window_us=%ld dither=%ld dwell_us=%ld jitter_us=%ld mux_us=%lu"
                  " clock_rb=0x%04X"
                  " mv_per_code=%.12g\n",
                  from, to, OUT_OF[0][from], OUT_OF[1][from], OUT_OF[2][from], OUT_OF[3][from],
                  OUT_OF[0][to], OUT_OF[1][to], OUT_OF[2][to], OUT_OF[3][to],
                  (unsigned long)cfg.clkActual, osr, cfg.fdata, cfg.periodUs, spi,
                  (unsigned long)cfg.spiActual, reps, windowUs, dither, dwellUs, jitterUs,
                  (unsigned long)muxUs, cfg.clockRb, (double)VREF * 1000.0 / ADC_FS);

    uint32_t nCrc = 0, nStale = 0, nOver = 0, nMissed = 0, nEdge = 0, nReset = 0;
    uint32_t readMax = 0, latMax = 0, lines = 0;
    uint16_t crc = 0xFFFF;
    bool fail = false;
    char line[96];
    for (long r = 0; r < reps && !fail; r++) {
        mux_select(from);
        drdyResync();
        Conv pre[4];
        int np = 0;
        uint32_t dwell = (uint32_t)dwellUs + (jitterUs ? esp_random() % (uint32_t)(jitterUs + 1) : 0);
        uint32_t t0 = micros();
        while (micros() - t0 < dwell || np < 4) {
            if (!readConv(cfg.timeoutMs, &pre[np % 4])) { fail = true; break; }
            np++;
        }
        if (fail) break;
        if (dither > 1) delayMicroseconds((uint32_t)(cfg.periodUs * (r % dither) / dither));
        uint32_t tSw = micros();
        mux_select(to);
        int nk = 0;
        while (nk < SD_MAXK) {
            if (!readConv(cfg.timeoutMs, &buf[nk])) { fail = true; break; }
            if ((int32_t)(buf[nk++].t - tSw) > windowUs) break;
        }
        // the last 4 dwell conversions (oldest first), then the window
        for (int i = 0; i < 4 + nk; i++) {
            const Conv &c = i < 4 ? pre[(np + i) % 4] : buf[i - 4];
            if (i >= 4) {
                nCrc += !!(c.f & BF_CRC); nStale += !!(c.f & BF_STALE);
                nOver += !!(c.f & BF_OVERRUN); nMissed += c.missed;
                nEdge += !!(c.f & BF_EDGELESS); nReset += !!(c.f & BF_RESET);
                if (c.readUs > readMax) readMax = c.readUs;
                if (!(c.f & BF_EDGELESS) && c.latUs > latMax) latMax = c.latUs;
            }
            snprintf(line, sizeof(line), "%ld %ld %u %ld %ld %ld %ld\n", r,
                     (long)(int32_t)(c.t - tSw), c.f, c.ch[0], c.ch[1], c.ch[2], c.ch[3]);
            put_line_crc(line, &crc);
            lines++;
        }
    }
    Serial.printf("#END lines=%lu crc=%04X crc_bad=%lu stale=%lu overrun=%lu missed=%lu"
                  " edgeless=%lu reset=%lu read_us_max=%lu lat_us_max=%lu%s\n",
                  (unsigned long)lines, crc, (unsigned long)nCrc, (unsigned long)nStale,
                  (unsigned long)nOver, (unsigned long)nMissed, (unsigned long)nEdge,
                  (unsigned long)nReset, (unsigned long)readMax, (unsigned long)latMax,
                  fail ? " truncated=1" : "");
    free(buf);
    restoreNormalCfg();
    Serial.flush();
    drdyResync();
}

static void settle_fine_cmd() {
    char buf[96];
    readCmdLine(buf, sizeof(buf));
    long v[10] = {-1, -1, 8000000, 256, 4000000, 64, 2000, 4, 3000, 1000};
    if (parseLongs(buf, v, 10) < 2) {
        Serial.print("#SETTLEERR usage: D<from> <to> [clkin] [osr] [spi] [reps] [window_us]"
                     " [dither] [dwell_us] [jitter_us] then ';' or newline\n");
        return;
    }
    settle_fine((int)v[0], (int)v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9]);
}

// ── SPI speed sweep ('p') ────────────────────────────────────────────────────
// How fast the jumper-wired SPI link can go, and whether the ESP32 keeps up
// with a conversion every 32 / 64 us.
//   p[clkin] [osr] [secs] [spi_hz ...]       then ';' or newline
//   defaults 8000000 256 2, SPI 600k 1M 2M 3M 4M 5M 6M 8M. "p;" runs them all.
// Per SPI clock: full ADC init at that clock, then read every conversion for
// <secs> on the current select. Output:
//   #SPISWEEP v1 clkin_hz= osr= fdata_sps= period_us= secs= steps=
//   #SPI spi_hz= spi_actual= clock_ok= frames= expected= crc_bad= stale= overrun=
//        missed= edgeless= reset= timeout= read_us_avg= read_us_max= lat_us_avg=
//        lat_us_max=
//   ... one #SPI line per step ...  then #SPIEND
// clock_ok=0: the CLOCK register readback was wrong (a garbled register read).
// read_us = CS low to CS high of one 18-byte read; lat_us = DRDY edge to the
// end of that read, which must stay below period_us or DRDY pulses get lost
// (counted as edgeless / overrun / missed).
#define SWEEP_MAX  16

static void spi_sweep(long clkin, long osr, long secs, const long *spis, int nspi) {
    const char *err = nullptr;
    for (int i = 0; i < nspi && !err; i++) err = fastCfgCheck(clkin, osr, spis[i]);
    if (secs < 1 || secs > 60) err = "secs out of 1-60";
    if (err) { Serial.printf("#SPIERR %s\n", err); return; }
    bool header = false;
    for (int i = 0; i < nspi; i++) {
        FastCfg cfg;
        bool clockOk = fastCfgApply(clkin, osr, spis[i], &cfg);
        if (!header) {
            Serial.printf("#SPISWEEP v1 clkin_hz=%lu osr=%ld fdata_sps=%.3f period_us=%.3f"
                          " secs=%ld steps=%d\n", (unsigned long)cfg.clkActual, osr, cfg.fdata,
                          cfg.periodUs, secs, nspi);
            Serial.flush();
            header = true;
        }
        uint32_t frames = 0, nCrc = 0, nStale = 0, nOver = 0, nMissed = 0, nEdge = 0, nReset = 0;
        uint32_t readMax = 0, latMax = 0, nLat = 0;
        uint64_t readSum = 0, latSum = 0;
        bool timeout = false;
        Conv c;
        for (int k = 0; k < BURST_SKIP && !timeout; k++) timeout = !readConv(cfg.timeoutMs, &c);
        drdyResync();
        uint32_t t0 = micros(), el = 0;
        while (!timeout && (el = micros() - t0) < (uint32_t)secs * 1000000UL) {
            if (!readConv(cfg.timeoutMs, &c)) { timeout = true; break; }
            frames++;
            nCrc += !!(c.f & BF_CRC); nStale += !!(c.f & BF_STALE);
            nOver += !!(c.f & BF_OVERRUN); nMissed += c.missed;
            nEdge += !!(c.f & BF_EDGELESS); nReset += !!(c.f & BF_RESET);
            readSum += c.readUs; if (c.readUs > readMax) readMax = c.readUs;
            if (!(c.f & BF_EDGELESS)) {
                latSum += c.latUs; nLat++;
                if (c.latUs > latMax) latMax = c.latUs;
            }
        }
        Serial.printf("#SPI spi_hz=%ld spi_actual=%lu clock_ok=%d frames=%lu expected=%lu"
                      " crc_bad=%lu stale=%lu overrun=%lu missed=%lu edgeless=%lu reset=%lu"
                      " timeout=%d read_us_avg=%.1f read_us_max=%lu lat_us_avg=%.1f"
                      " lat_us_max=%lu\n", spis[i], (unsigned long)cfg.spiActual, clockOk ? 1 : 0,
                      (unsigned long)frames, (unsigned long)(el / cfg.periodUs),
                      (unsigned long)nCrc, (unsigned long)nStale, (unsigned long)nOver,
                      (unsigned long)nMissed, (unsigned long)nEdge, (unsigned long)nReset,
                      timeout ? 1 : 0, frames ? (double)readSum / frames : 0.0,
                      (unsigned long)readMax, nLat ? (double)latSum / nLat : 0.0,
                      (unsigned long)latMax);
        Serial.flush();
    }
    restoreNormalCfg();
    Serial.print("#SPIEND\n");
    Serial.flush();
}

static void spi_sweep_cmd() {
    static const long DEF_SPI[] = {600000, 1000000, 2000000, 3000000,
                                   4000000, 5000000, 6000000, 8000000};
    char buf[192];
    readCmdLine(buf, sizeof(buf));
    long v[3 + SWEEP_MAX] = {8000000, 256, 2};
    int n = parseLongs(buf, v, 3 + SWEEP_MAX);
    if (n <= 3) {
        spi_sweep(v[0], v[1], v[2], DEF_SPI, sizeof(DEF_SPI) / sizeof(DEF_SPI[0]));
    } else {
        spi_sweep(v[0], v[1], v[2], v + 3, n - 3);
    }
}

// ── Active selects ('S') ─────────────────────────────────────────────────────
//   S<digits 0-7>;   e.g. S0123457;  scan only these selects
//   S;               all 8 (the boot default; the mask is not kept over a reboot)
static void setSelMask(uint8_t m) {
    g_selMask = m;
    for (uint8_t sel = 0; sel < NUM_SEL; sel++) {
        if ((m >> sel) & 1) continue;
        for (uint8_t a = 0; a < NUM_ADC_CH; a++) {
            uint8_t o = OUT_OF[a][sel];
            outRaw[o] = SKIP_CODE;
            outV  [o] = (SKIP_CODE / ADC_FS) * VREF;
        }
    }
    char list[NUM_SEL + 1];
    int n = 0;
    for (uint8_t sel = 0; sel < NUM_SEL; sel++) if ((m >> sel) & 1) list[n++] = '0' + sel;
    list[n] = '\0';
    logf("selects: %s (%d of %d; skipped outputs read as railed/open)\n", list, n, NUM_SEL);
    if (g_lk.mode) {                          // new slot table, same grid
        LockState saved = g_lk;
        const char *err = lkConfigure(g_lk.mode, g_lk.settleUs, g_lk.osr, g_lk.scanUs, g_lk.spi);
        if (err) { g_lk = saved; logf("selects: %s; locked mode unchanged\n", err); }
        lkRearm(false);
    }
    if (g_mode == 'B') send_info();
}

static void selects_cmd() {
    char buf[32];
    readCmdLine(buf, sizeof(buf));
    uint8_t m = 0;
    for (const char *p = buf; *p; p++) {
        if (*p >= '0' && *p < '0' + NUM_SEL) m |= 1 << (*p - '0');
        else if (*p != ' ' && *p != ',') {
            logf("S: usage S<digits 0-7>; e.g. S0123457;   S; = all %d\n", NUM_SEL);
            return;
        }
    }
    setSelMask(m ? m : SEL_MASK_ALL);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Locked scan ('M1' quiet 62.5 Hz, 'M2' wide 250 Hz; 'M0' = legacy scan)
// ─────────────────────────────────────────────────────────────────────────────
// Most of the remaining noise is a comb at exact multiples of 125 Hz (bench
// 2026-10-23; ESP32/USB activity), locked to the ESP32 crystal. CLKIN (8 MHz
// from LEDC) and micros() come from the same crystal, so a scan that is
// exactly N conversions of the free-running ADC, with N x T a multiple of
// 8 ms, folds the whole comb onto 0 Hz (a constant offset per channel).
//   quiet  OSR 512 (T 128 us), SPI 8 MHz,  N 125 -> 16 ms, 62.5 Hz (default;
//          at the input-noise floor on the bench)
//          OSR 256 (T 64 us): N 250; period 8000 us (125 Hz): N 125
//   wide   OSR 128 (T 32 us),  SPI 12 MHz, N 125 ->  4 ms, 250 Hz
// The N conversions are split over the active selects (slots get floor/ceil of
// N / nsel). The mux switches right after the DRDY that ends the previous slot;
// scan k starts on conversion k x N. Per slot, the reading is the plain mean
// of the conversions whose sinc3 window (3 periods) starts >= settle_us after
// the switch, i.e. the first DISCARD = ceil(3 + (settle + LK_LAT_ALLOW_US)/T) - 1
// conversions of every slot are dropped (fixed per slot, so every scan uses the
// same conversion positions). The mux settle tail (tau 70-90 us, ~0.0136 x the
// step at the switch) is then ~2e-5 (quiet) to ~1e-4 (wide) of the step.
//
// Every conversion is read; the conversion index comes from the DRDY edge
// time (ISR) on the grid tRef + k x T, so a lost DRDY pulse or a late read
// can't shift the grid. A missed conversion, a late/missed mux switch or a
// resync taints the slot (no data from it) and sets FLAG_SYNC; a slot left
// with no usable conversion keeps its previous value and sets FLAG_EMPTY.
// Frames go through the TX queue and are written in chunks only when the next
// DRDY is at least LK_TX_GUARD_US away, and never just before a slot end.
// A HEALTH frame (FT_HEALTH) each ~1 s reports the reader: busy fraction,
// DRDY-to-read-end latency, SPI read time, overruns, missed, edgeless, CRC,
// stale, late switches, TX drops, empty slots, resyncs.
// The reader is lkStep() (one conversion) and keeps all its state in g_lk, so
// it can later move to its own pinned FreeRTOS task with the TX queue as the
// hand-over to a sender task.
#define LK_CLKIN           8000000UL
#define LK_LAT_ALLOW_US    10      // switch latency allowed for in the discard count
#define LK_SETTLE_DEFAULT  200
#define LK_TX_GUARD_US     6       // idle work only if the next DRDY is this far off (a
                                   // slightly late read is harmless except at a slot end,
                                   // which idle work never precedes)
#define LK_TIMEOUT_MS      50      // no conversion for this long: re-init the ADC
#ifndef LK_TRACE                   // host_sim hook: one line per reader step
#define LK_TRACE(...)
#endif
static const uint16_t LK_OSR [3] = {0, 512, 128};   // quiet: OSR 512 is at the input-noise floor
static const uint32_t LK_SPI [3] = {0, 8000000, 12000000};
static const uint32_t LK_SCAN[3] = {0, 16000, 4000};  // default scan period, us
static const uint32_t LK_SCAN_STEP[3] = {0, 8000, 4000}; // a period override must be a multiple
                                                          // (the comb is at multiples of 125 Hz)
static const char    *LK_NAME[3] = {"legacy", "quiet", "wide"};

static_assert(PIN_SELA_2 >= 32 && PIN_SELB_2 >= 32 && PIN_SELA_1 < 32 && PIN_SELB_1 < 32 &&
              PIN_SELC_1 < 32 && PIN_SELC_2 < 32 && PIN_CS < 32 && PIN_DRDY < 32,
              "muxFast / csFast assume these GPIO banks");

// Direct GPIO register access: digitalWrite x 6 took ~10 us per mux switch.
static uint32_t g_muxSet0[NUM_SEL], g_muxClr0[NUM_SEL], g_muxSet1[NUM_SEL], g_muxClr1[NUM_SEL];
static void muxFastInit() {
    const uint32_t all0 = (1UL << PIN_SELA_1) | (1UL << PIN_SELB_1) | (1UL << PIN_SELC_1) |
                          (1UL << PIN_SELC_2);
    const uint32_t all1 = (1UL << (PIN_SELA_2 - 32)) | (1UL << (PIN_SELB_2 - 32));
    for (uint8_t sel = 0; sel < NUM_SEL; sel++) {
        uint32_t s0 = ((sel & 1) ? 1UL << PIN_SELA_1 : 0) | ((sel & 2) ? 1UL << PIN_SELB_1 : 0) |
                      ((sel & 4) ? (1UL << PIN_SELC_1) | (1UL << PIN_SELC_2) : 0);
        uint32_t s1 = ((sel & 1) ? 1UL << (PIN_SELA_2 - 32) : 0) |
                      ((sel & 2) ? 1UL << (PIN_SELB_2 - 32) : 0);
        g_muxSet0[sel] = s0; g_muxClr0[sel] = all0 & ~s0;
        g_muxSet1[sel] = s1; g_muxClr1[sel] = all1 & ~s1;
    }
}
static inline void muxFast(uint8_t sel) {
    REG_WRITE(GPIO_OUT_W1TS_REG, g_muxSet0[sel]);
    REG_WRITE(GPIO_OUT_W1TC_REG, g_muxClr0[sel]);
    REG_WRITE(GPIO_OUT1_W1TS_REG, g_muxSet1[sel]);
    REG_WRITE(GPIO_OUT1_W1TC_REG, g_muxClr1[sel]);
}
static inline bool drdyLowFast() { return !((REG_READ(GPIO_IN_REG) >> PIN_DRDY) & 1); }

// One 18-byte frame: CS by register, the SPI driver's no-lock transfer (the
// transaction is held all the time), table CRC. Same result as ads_read_all.
static uint32_t adsReadFast(long ch[4], bool *crcOk) {
    static const uint32_t zeros[5] = {0, 0, 0, 0, 0};
    uint32_t w[5];                                   // word-aligned for the driver
    REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << PIN_CS);
    spiTransferBytesNL(vspi.bus(), zeros, (uint8_t *)w, 18);
    REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << PIN_CS);
    const uint8_t *buf = (const uint8_t *)w;
    for (int i = 0; i < 4; i++) {
        uint32_t raw = ((uint32_t)buf[3 + 3*i] << 16) | ((uint32_t)buf[4 + 3*i] << 8) |
                        (uint32_t)buf[5 + 3*i];
        if (raw & 0x800000UL) raw |= 0xFF000000UL;
        ch[i] = (long)(int32_t)raw;
    }
    *crcOk = crc16_fast(0xFFFF, buf, 15) == (((uint16_t)buf[15] << 8) | buf[16]);
    return (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]) & 0xFFFFFF;
}

// Slot table for mode/settle/OSR/SPI and the current selects. nullptr = OK.
static const char *lkConfigure(uint8_t mode, long settle, long osr, long scanUs, long spi) {
    if (mode < 1 || mode > 2) return "mode must be 1 (quiet) or 2 (wide)";
    if (mode == 2 && scanUs != (long)LK_SCAN[2]) return "wide mode has a fixed 4000 us period";
    if (scanUs <= 0 || scanUs > 64000 || scanUs % LK_SCAN_STEP[mode])
        return "period must be a multiple of 8000 us, up to 64000";
    if (osrCode(osr) < 0) return "osr must be a power of 2, 128..16384";
    if (spi < 100000 || spi > SPI_MAX_HZ) return "spi out of 0.1-20 MHz";
    if (settle < 0 || settle > 5000) return "settle out of 0-5000 us";
    if ((2ULL * osr * 1000000ULL) % LK_CLKIN) return "conversion period not a whole us";
    uint32_t T = (uint32_t)(2ULL * osr * 1000000ULL / LK_CLKIN);
    if (scanUs % T) return "scan period not a whole number of conversions at this OSR";
    uint32_t N = scanUs / T;
    if (N > LK_MAX_N) return "too many conversions per scan";
    uint8_t sels[NUM_SEL], nsel = 0;
    for (uint8_t s = 0; s < NUM_SEL; s++) if ((g_selMask >> s) & 1) sels[nsel++] = s;
    if (!nsel) return "no select active";
    uint32_t m0 = (uint32_t)ceil(3.0 + (double)(settle + LK_LAT_ALLOW_US) / T);
    uint8_t discard = (uint8_t)(m0 - 1);
    uint32_t minLen = N / nsel;
    if (minLen <= discard) return "settle too long: a slot would have no usable conversion";
    g_lk.mode = mode; g_lk.osr = osr; g_lk.spi = spi; g_lk.settleUs = settle;
    g_lk.T = T; g_lk.N = N; g_lk.scanUs = (uint32_t)scanUs;
    g_lk.nslot = nsel; g_lk.discard = discard;
    memset(g_lk.avgPerSel, 0, sizeof(g_lk.avgPerSel));
    uint16_t start = 0;
    for (uint8_t i = 0; i < nsel; i++) {
        uint16_t len = N / nsel + (i < N % nsel ? 1 : 0);
        g_lk.slotSel[i] = sels[i]; g_lk.slotStart[i] = start; g_lk.slotLen[i] = len;
        g_lk.avgPerSel[sels[i]] = len - discard;
        for (uint16_t p = start; p < start + len; p++) g_lk.posSlot[p] = i;
        start += len;
    }
    return nullptr;
}

static void lkClearScan() {
    memset(g_lk.acc, 0, sizeof(g_lk.acc));
    memset(g_lk.cnt, 0, sizeof(g_lk.cnt));
    memset(g_lk.taint, 0, sizeof(g_lk.taint));
}

static void lkHealthReset() {
    memset(&g_lk.h, 0, sizeof(g_lk.h));
    g_lk.h.t0Us = micros();
}

// Restart the reader: wait for the next scan boundary on the conversion grid
// (newGrid: take a new grid from the next DRDY, e.g. after an ADC re-init).
static void lkRearm(bool newGrid) {
    if (newGrid) g_lk.haveRef = false;
    g_lk.waitBoundary = true;
    g_lk.lastK = -1;
    g_lk.curSel = 0xFF;
    g_lk.pendingSend = false;
    lkClearScan();
    drdyResync();
    g_lk.lastActT = g_lk.lastEdgeT = micros();
}

// CLKIN 8 MHz, OSR and SPI of the configured mode, full ADC init, rearm.
static bool lkStart() {
    FastCfg cfg;
    bool ok = fastCfgApply(LK_CLKIN, g_lk.osr, g_lk.spi, &cfg);
    g_lk.spiActual = cfg.spiActual;
    lkRearm(true);
    return ok;
}

static void lkStop() {
    g_lk.mode = 0;
    g_lk.streaming = false;
    restoreNormalCfg();
}

static void lkSendHealth() {
    LockHealth &h = g_lk.h;
    uint32_t win = micros() - h.t0Us;
    if (!win) win = 1;
    uint8_t p[42];
    p[0] = 1;
    p[1] = g_lk.mode;
    put_u16(p + 2,  (uint16_t)min<uint64_t>(1000, h.busyUs * 1000 / win));
    put_u16(p + 4,  h.latMax);
    put_u16(p + 6,  (uint16_t)min<uint64_t>(65535, h.latN ? h.latSum * 10 / h.latN : 0));
    put_u16(p + 8,  (uint16_t)min<uint64_t>(65535, h.readN ? h.readSum * 10 / h.readN : 0));
    put_u16(p + 10, h.readMax);
    put_u16(p + 12, h.txMax);
    put_u32(p + 14, h.conv);
    const uint32_t v[12] = {h.overrun, h.missed, h.edgeless, h.crc, h.stale, h.lateSw,
                            h.txDrop, h.empty, h.resync, h.reinit, h.recov, win / 1000};
    for (int i = 0; i < 12; i++) put_u16(p + 18 + 2 * i, (uint16_t)min<uint32_t>(65535, v[i]));
    send_frame(FT_HEALTH, p, sizeof(p));
}

// The scan that just ended (its last conversion was read): averages -> outRaw,
// queue the frame (sent from idle time), next scan.
static void lkCloseScan(uint32_t tEnd) {
    for (uint8_t i = 0; i < g_lk.nslot; i++) {
        uint8_t sel = g_lk.slotSel[i];
        if (!g_lk.cnt[i]) { g_lk.scanFlags |= FLAG_EMPTY; g_lk.h.empty++; continue; }
        for (uint8_t a = 0; a < NUM_ADC_CH; a++) {
            uint8_t o = OUT_OF[a][sel];
            int64_t sum = g_lk.acc[i][a];
            int64_t n = g_lk.cnt[i];
            outRaw[o] = (long)(sum >= 0 ? (sum + n / 2) / n : -((-sum + n / 2) / n));
            outV[o] = (outRaw[o] / ADC_FS) * VREF;
        }
    }
    // t_us = grid time of the DRDY that ended the previous scan (where the mux
    // switched to slot 0), so successive scans are exactly N x T apart
    g_scanT0  = g_lk.tRef - g_lk.T;
    g_scanDur = (uint32_t)g_lk.N * g_lk.T;
    (void)tEnd;
    g_scanRecovered = g_scanCrcRetry = g_scanStale = g_scanReinit = false;
    g_lk.outFlags = g_lk.scanFlags;
    g_lk.scanFlags = 0;
    g_scans++;
    g_lk.scanDone = true;
    if (g_lk.streaming) {
        if (g_lk.pendingSend) g_lk.h.txDrop++;      // the previous one never got idle time
        g_lk.pendingSend = true;
    }
    lkClearScan();
    g_lk.tRef += (uint32_t)g_lk.N * g_lk.T;
    g_lk.lastK = -1;
    if (micros() - g_lk.h.t0Us >= 1000000UL) {
        if (g_lk.streaming) lkSendHealth();
        lkHealthReset();
    }
}

static inline int32_t lkIndex(uint32_t t) {
    int32_t d = (int32_t)(t - g_lk.tRef), T = g_lk.T;
    return d >= 0 ? (d + T / 2) / T : -((-d + T / 2) / T);
}

// Reads one conversion if one is waiting. 0 = none waiting, 1 = handled.
static int lkStep() {
    uint32_t e, t;
    bool edge;
    if ((int32_t)(g_drdyEdges - g_drdyTaken) > 0) {
        do { e = g_drdyEdges; t = g_drdyEdgeT; } while (e != g_drdyEdges);
        edge = true;
    } else if (drdyLowFast()) {
        // DRDY low with no edge: give a just-fired ISR ~3 us to count it,
        // else it is a conversion whose pulse was blocked by the previous read
        // (SBAS890D 8.5.1.5) or the FIFO's second slot after an overrun.
        uint32_t w0 = micros();
        while (micros() - w0 < 3) {}
        if ((int32_t)(g_drdyEdges - g_drdyTaken) > 0) return lkStep();
        e = g_drdyTaken; t = g_lk.lastEdgeT + g_lk.T;
        edge = false;
    } else {
        return 0;
    }
    uint32_t tIn = micros();
    LockHealth &h = g_lk.h;
    int32_t pend = (int32_t)(e - g_drdyTaken);
    g_drdyTaken = e;
    long ch[4];
    bool ok;

    if (!g_lk.haveRef) {                     // new grid: this conversion = pos N-1
        if (!edge) { adsReadFast(ch, &ok); g_respPending = false; return 1; }
        g_lk.tRef = t + g_lk.T;
        g_lk.haveRef = true;
        g_lk.lastK = -2;                     // this one is k = -1
    }
    int32_t k = edge ? lkIndex(t) : g_lk.lastK + 1;
    if (!g_lk.waitBoundary && k >= (int32_t)g_lk.N) {
        // the scan end was missed: close what we have, flagged
        g_lk.scanFlags |= FLAG_SYNC;
        lkCloseScan(t);
        k -= g_lk.N;
    }
    if (k >= (int32_t)g_lk.N) {              // whole scans skipped (pause, resume)
        int32_t q = k / g_lk.N;
        g_lk.tRef += (uint32_t)q * g_lk.N * g_lk.T;
        k -= q * g_lk.N;
        g_lk.lastK -= q * g_lk.N;
        g_scans += q;                        // seq counts grid scans: the host sees the gap
        if (!g_lk.waitBoundary) { g_lk.scanFlags |= FLAG_SYNC; h.missed += q * g_lk.N; }
    }
    // An edge whose grid index was already read: an earlier read got a label
    // one ahead (after an overrun it read the older of two FIFO entries, and the
    // narrow DRDY pulse at the end of that read is an extra edge, late enough to
    // round to the next index; or a late ISR time). Trust the edge-time grid:
    // take k as it is and don't use this read (index k was counted already).
    // Forcing k = lastK + 1 instead left every later index one ahead, with a
    // resync on every conversion (bench 2026-10-23, quiet OSR 256).
    bool back = edge && k <= g_lk.lastK;
    if (back) h.resync++;
    // (while waiting for a boundary nothing is collected, so gaps don't count)
    bool over = !g_lk.waitBoundary && (pend > 1 || k > g_lk.lastK + 1);
    bool sync = over;
    if (over) {
        h.overrun++;
        if (k > g_lk.lastK + 1) h.missed += k - g_lk.lastK - 1;
    }
    if (!edge) h.edgeless++;
    LK_TRACE(t, edge, pend, k, g_lk.lastK, back, over, g_lk.waitBoundary);

    // Mux: switch right after the DRDY that ends a slot (before the read).
    uint8_t s = 0;
    bool boundary;
    if (g_lk.waitBoundary) {
        boundary = (k == (int32_t)g_lk.N - 1) || k == -1;
    } else if (k < 0) {
        // realigned back into the scan that was already closed (one conversion
        // early, so slot 0 just got one more conversion of settling): nothing
        // to collect
        if (g_lk.curSel != g_lk.slotSel[0]) {
            muxFast(g_lk.slotSel[0]);
            g_lk.curSel = g_lk.slotSel[0];
            g_lk.taint[0] = true;
            g_lk.scanFlags |= FLAG_SYNC;
        }
        boundary = false;
    } else {
        s = g_lk.posSlot[k];
        // the read labelled k earlier was conversion k - 1: harmless inside the
        // settled part of the slot, unsettled if k is the first one averaged
        if (back && k - g_lk.slotStart[s] == g_lk.discard) sync = true;
        if (sync) g_lk.taint[s] = true;
        if (g_lk.curSel != g_lk.slotSel[s]) {          // missed a switch
            muxFast(g_lk.slotSel[s]);
            g_lk.curSel = g_lk.slotSel[s];
            g_lk.taint[s] = true;
            g_lk.scanFlags |= FLAG_SYNC;
        }
        if (sync) g_lk.scanFlags |= FLAG_SYNC;
        boundary = k == g_lk.slotStart[s] + g_lk.slotLen[s] - 1;
    }
    if (boundary) {
        uint8_t ns = g_lk.waitBoundary ? 0 : (s + 1 < g_lk.nslot ? s + 1 : 0);
        muxFast(g_lk.slotSel[ns]);
        g_lk.curSel = g_lk.slotSel[ns];
        g_lk.tSw = micros();
        if (edge && g_lk.tSw - t > LK_LAT_ALLOW_US) {
            h.lateSw++;
            if (!g_lk.waitBoundary) g_lk.scanFlags |= FLAG_SYNC;
        }
    }

    uint32_t r0 = micros();
    uint32_t st = adsReadFast(ch, &ok);
    uint32_t r1 = micros();
    h.conv++;
    uint16_t rd = (uint16_t)min<uint32_t>(r1 - r0, 65535);
    h.readSum += rd; h.readN++;
    if (rd > h.readMax) h.readMax = rd;
    if (edge) {
        uint16_t lat = (uint16_t)min<uint32_t>(r1 - t, 65535);
        h.latSum += lat; h.latN++;
        if (lat > h.latMax) h.latMax = lat;
    }
    if (over) {                              // drain the FIFO's older entry
        long junk[4]; bool jok;
        for (int i = 0; i < 2 && drdyLowFast() && (int32_t)(g_drdyEdges - g_drdyTaken) <= 0; i++)
            adsReadFast(junk, &jok);
    }
    g_lk.lastActT = r1;
    g_lk.lastEdgeT = t;

    bool use = true;
    if (g_respPending) { g_respPending = false; use = false; }      // LOCK reply frame
    else if (!ok) { h.crc++; g_crcFails++; g_lk.scanFlags |= FLAG_CRC_RETRY; use = false; }
    else if ((st >> 8) & STATUS_RESET) {
        // the ADC reset behind our back: same re-init as boot, new grid
        h.reinit++; g_adcReinits++;
        g_lk.scanFlags |= FLAG_REINIT;
        logf("[adc reset #%lu] in %s mode: re-initialising\n", (unsigned long)g_adcReinits,
             LK_NAME[g_lk.mode]);
        lkStart();
        h.busyUs += micros() - tIn;
        return 1;
    } else if (((st >> 8) & STATUS_DRDY_ALL) != STATUS_DRDY_ALL) {
        h.stale++; g_staleReads++; g_lk.scanFlags |= FLAG_STALE; use = false;
    }

    if (g_lk.waitBoundary) {
        if (boundary) {                      // the next conversion starts scan pos 0
            g_lk.waitBoundary = false;
            if (k == (int32_t)g_lk.N - 1) {  // the partial scan before it is skipped
                g_lk.tRef += (uint32_t)g_lk.N * g_lk.T;
                g_scans++;
            }
            g_lk.lastK = -1;
            lkClearScan();
        } else {
            g_lk.lastK = k;
        }
    } else {
        if (use && !back && k >= 0 && !g_lk.taint[s] && k - g_lk.slotStart[s] >= g_lk.discard) {
            for (uint8_t a = 0; a < NUM_ADC_CH; a++) g_lk.acc[s][a] += ch[a];
            g_lk.cnt[s]++;
        }
        g_lk.lastK = k;
        if (k == (int32_t)g_lk.N - 1) lkCloseScan(r1);
    }
    h.busyUs += micros() - tIn;
    return 1;
}

// After a pause (text mode, a command): drop what the FIFO holds and wait for
// the next scan boundary on the same grid (unless the pause was absurdly long).
static void lkResume() {
    long junk[4]; bool ok;
    for (int i = 0; i < 3 && drdyLowFast(); i++) adsReadFast(junk, &ok);
    if (g_lk.haveRef && (uint32_t)(micros() - g_lk.tRef) > 1000000000UL) g_lk.haveRef = false;
    lkRearm(false);
    g_lk.h.resync++;
}

static void lkRecover() {
    g_lk.h.recov++;
    g_halts++;
    g_lk.scanFlags |= FLAG_RECOVERED;
    logf("[halt] no conversion for %d ms in %s mode: CLKIN restart + re-init\n", LK_TIMEOUT_MS,
         LK_NAME[g_lk.mode]);
    spiFlush();
    setClkin(LK_CLKIN);
    lkStart();
}

// Run the reader for about budgetUs (it returns right after a conversion, so
// the caller's own work can't make the next read late). Idle time between
// conversions goes to the deferred frame and the TX chunks.
static void lkService(uint32_t budgetUs) {
    uint32_t t0 = micros();
    // A gap shorter than a scan (a busy loop(), an interrupt hog) is a sync loss
    // inside the scan: lkStep sees the jump, taints the slot and flags the
    // scan. Longer (text modes, commands): drop the FIFO, wait for a boundary.
    if (t0 - g_lk.lastActT > g_lk.scanUs) lkResume();
    for (;;) {
        if (lkStep()) {
            if (micros() - t0 >= budgetUs) return;
            continue;
        }
        uint32_t now = micros();
        if (now - g_lk.lastActT > LK_TIMEOUT_MS * 1000UL) {
            lkRecover();
            if (micros() - t0 >= budgetUs) return;
            continue;
        }
        // idle: only if the next DRDY is far enough off and doesn't end a slot
        if (!g_lk.streaming || g_lk.waitBoundary) continue;
        int32_t next = g_lk.lastK + 1;
        if (next >= 0 && next < (int32_t)g_lk.N) {
            uint8_t s = g_lk.posSlot[next];
            if (next == g_lk.slotStart[s] + g_lk.slotLen[s] - 1) continue;
        }
        if ((int32_t)(g_lk.lastEdgeT + g_lk.T - now) < LK_TX_GUARD_US) continue;
        uint32_t w0 = micros();
        if (g_lk.pendingSend) {
            g_lk.pendingSend = false;
            send_scan();                     // packs into the TX queue
        } else if (txqUsed()) {
            uint32_t n = txqWrite(g_lk.T >= 64 ? 48 : 16);
            uint32_t dt = micros() - w0;
            if (n && dt > g_lk.h.txMax) g_lk.h.txMax = (uint16_t)min<uint32_t>(dt, 65535);
        } else {
            continue;
        }
        g_lk.h.busyUs += micros() - w0;
    }
}

// One scan for the text modes (blocking; the reader resyncs after their pauses).
static bool lkScanOnce() {
    g_lk.scanDone = false;
    uint32_t t0 = millis();
    while (!g_lk.scanDone) {
        lkService(2000);
        if (millis() - t0 > 1000) return false;
    }
    return true;
}

static void lkPrintMode() {
    if (!g_lk.mode) {
        logf("mode 0 (legacy): CLKIN %lu Hz, OSR %d, SPI %lu Hz, settle %d us + %d discards,"
             " selects mask 0x%02X\n", (unsigned long)CLKIN_HZ, OSR_NORMAL, (unsigned long)SPI_HZ,
             SETTLE_US, DISCARD_SAMPLES, g_selMask);
        return;
    }
    char avg[3 * NUM_SEL + 1];
    int n = 0;
    for (uint8_t i = 0; i < g_lk.nslot; i++)
        n += snprintf(avg + n, sizeof(avg) - n, "%s%u", i ? "," : "", g_lk.slotLen[i] - g_lk.discard);
    logf("mode %u (%s): CLKIN %lu Hz, OSR %u (T %u us), SPI %lu Hz (%lu), N %u = %lu us"
         " (%.2f scans/s), %u selects, settle %u us -> drop %u, average %s per slot\n",
         g_lk.mode, LK_NAME[g_lk.mode], (unsigned long)g_clkinNow, g_lk.osr, g_lk.T,
         (unsigned long)g_lk.spi, (unsigned long)g_lk.spiActual, g_lk.N, (unsigned long)g_lk.scanUs,
         1e6 / g_lk.scanUs, g_lk.nslot, g_lk.settleUs, g_lk.discard, avg);
}

// M<mode> [settle_us] [osr] [period_us] [spi_hz];   M0 legacy, M1 quiet, M2 wide.
// 0 (or left out) = the mode's default: settle 200, quiet OSR 512 / 16000 us /
// SPI 8 MHz, wide OSR 128 / 4000 us / 12 MHz. Quiet periods: multiples of 8000.
static void mode_cmd() {
    char buf[64];
    readCmdLine(buf, sizeof(buf));
    long v[5] = {-1, LK_SETTLE_DEFAULT, 0, 0, 0};
    int n = parseLongs(buf, v, 5);
    if (n < 1 || v[0] < 0 || v[0] > 2) {
        logf("M: usage M<0|1|2> [settle_us] [osr] [period_us] [spi_hz];  0 legacy, 1 quiet,"
             " 2 wide\n");
        return;
    }
    if (v[0] == 0) {
        if (g_lk.mode) lkStop();
        lkPrintMode();
        if (g_mode == 'B') send_info();
        return;
    }
    uint8_t keep = g_lk.mode;
    LockState saved = g_lk;
    const char *err = lkConfigure((uint8_t)v[0], v[1], v[2] ? v[2] : LK_OSR[v[0]],
                                  v[3] ? v[3] : LK_SCAN[v[0]], v[4] ? v[4] : LK_SPI[v[0]]);
    if (err) {
        g_lk = saved;
        logf("M: %s\n", err);
        return;
    }
    if (!lkStart()) {
        logf("M: CLOCK config not accepted (readback 0x%04X); back to %s\n", g_clockRb,
             LK_NAME[keep]);
        if (keep) { g_lk = saved; lkStart(); } else lkStop();
        return;
    }
    lkHealthReset();
    lkPrintMode();
    if (g_mode == 'B') send_info();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    crcTabInit();
    Serial.setTxBufferSize(SERIAL_TXBUF);     // must precede begin()
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println("\n=================================================");
    Serial.println("  whisker_daq — 16 whiskers x 2 gauges = 32 ch");
    Serial.println("=================================================");

#if PROBE_SELECTS
    int lows = probeSelectsGrounded();
    Serial.printf("Select-line probe: %d of 6 read LOW with pull-up.\n", lows);
    if (lows > 0) {
        Serial.println("*** WARNING: a select line still looks tied to GND.");
        Serial.println("*** Driving it high would short that GPIO. Remove any");
        Serial.println("*** leftover jumper to GND before continuing.");
        Serial.println("*** Halting. (Set PROBE_SELECTS 0 to override.)");
        while (true) delay(1000);
    }
#endif
    {
        const uint8_t p[] = { PIN_SELA_1, PIN_SELB_1, PIN_SELC_1,
                              PIN_SELA_2, PIN_SELB_2, PIN_SELC_2 };
        for (uint8_t i = 0; i < 6; i++) { pinMode(p[i], OUTPUT); digitalWrite(p[i], LOW); }
        muxFastInit();
    }

    pinMode(PIN_CS, OUTPUT);  cs_high();
    pinMode(PIN_DRDY, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY), drdyISR, FALLING);
    pinMode(PIN_RESET, OUTPUT); digitalWrite(PIN_RESET, HIGH);   // idle = not reset

    // ledcAttach returns a BOOL in core 3.x, not a frequency
    bool ok = ledcAttach(PIN_CLKIN, CLKIN_HZ, 1);
    ledcWrite(PIN_CLKIN, 1);
    Serial.printf("CLKIN: %s, %lu Hz    SCLK: %lu Hz    fDATA ~ %lu SPS\n",
                  ok ? "on" : "*** FAILED — ADC will be dead ***",
                  (unsigned long)ledcReadFreq(PIN_CLKIN),
                  (unsigned long)SPI_HZ, (unsigned long)FDATA_SPS);
    delay(30);

    vspi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);
    pinMode(PIN_CS, OUTPUT); cs_high();
    vspi.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));

    // The PCB normally powers up before the ESP32 starts CLKIN, so the chip
    // resets with no clock and comes up stuck. Init, then escalate if needed.
    bool cfg = initADC();
    long tmp[4];
    if (!readValid(tmp, 300)) {
        Serial.println("Chip stuck at boot (powered before CLKIN) — recovering...");
        int lvl = recoverADC(tmp);
        Serial.printf("Boot recovery: %s\n", lvl ? "OK" : "FAILED");
        cfg = (g_clockRb & CLOCK_MASK) == (CLOCK_ALL_CH_OSR128 & CLOCK_MASK);
    }
    // ID register (SBAS890D): bits 15:12 = 0010b, CHANCNT 11:8 = 4 -> 0x24xx.
    Serial.printf("Device ID      = 0x%04X  %s\n", g_id,
                  (g_id >> 8) == 0x24 ? "(ADS131M04)" :
                  (g_id == 0x0000 || g_id == 0xFFFF) ? "(NO RESPONSE)" : "(unexpected ID)");
    Serial.printf("CLOCK readback = 0x%04X  %s\n", g_clockRb,
                  cfg ? "(config ACCEPTED)" : "(NOT accepted)");
    Serial.printf("MODE readback  = 0x%04X  %s\n", g_modeRb,
                  g_modeRb == MODE_CLEAR_RESET ? "(RESET flag cleared)" : "(unexpected)");
    g_reinitArmed = true;     // from now on STATUS RESET=1 means a real ADC reset

    int edges = 0, last = digitalRead(PIN_DRDY);
    uint32_t t0 = millis();
    while (millis() - t0 < 200) { int c = digitalRead(PIN_DRDY); if (c != last) { edges++; last = c; } }
    Serial.printf("DRDY edges/200ms = %d  (%s)\n", edges, edges > 0 ? "converting" : "NOT toggling");

    // Boot default: the quiet locked scan (M1;, OSR 512, 62.5 Hz) since
    // 2026-10-23; M0; gives the legacy scan.
    if (const char *err = lkConfigure(1, LK_SETTLE_DEFAULT, LK_OSR[1], LK_SCAN[1], LK_SPI[1])) {
        Serial.printf("quiet mode not available (%s): legacy scan\n", err);
    } else if (!lkStart()) {
        Serial.printf("quiet mode CLOCK not accepted (0x%04X): legacy scan\n", g_clockRb);
        lkStop();
    } else {
        lkHealthReset();
        lkPrintMode();
    }

    Serial.println("\nCapturing baseline — keep all whiskers still...");
    captureBaseline(16);
    Serial.println("\nCommands: B=binary stream  g=grid  x=profile  s=table  b=rebaseline");
    Serial.println("          v=abs/delta  m=map  c=calibrate  n=noise  [ ]=scale  0=auto");
    Serial.println("          d<0-7>=mux settling diagnostic for that select");
    Serial.println("          r<sel> [n] [osr] [clkin_hz] [spi_hz];=raw burst (see header)");
    Serial.println("          D<from> <to> [clkin] [osr] [spi] [reps] [win] [dither] [dwell] [jitter];=fine settle");
    Serial.println("          p[clkin] [osr] [secs] [spi ...];=SPI speed sweep");
    Serial.println("          S<digits>;=scan only these selects (S; = all 8)");
    Serial.println("          M<0|1|2> [settle_us] [osr] [period_us] [spi];=scan mode: 0 legacy"
                   " 76 Hz, 1 quiet 62.5 Hz (OSR 512), 2 wide 250 Hz");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────────────────────
static void enter_text_mode(char m, const char *banner) {
    g_mode = m;
    Serial.println(banner);
}

void loop() {
    while (Serial.available()) {
        char k = Serial.read();
        if (g_lk.streaming) {             // finish the queued frames before anything else
            txqFlushBlocking();
            g_lk.streaming = false;
        }
        switch (k) {
            case 'B': g_mode = 'B'; send_info(); break;
            case 'b': logf("Re-capturing baseline...\n"); captureBaseline(16); break;
            case 'v': g_absolute = !g_absolute; break;
            case 'm': if (g_mode == 'B') g_mode = 'g'; print_map(); break;
            case 'c': enter_text_mode('c', "\n-- CALIBRATE: deflect ONE gauge at a time --"); break;
            case 'g': enter_text_mode('g', "\n-- GRID --"); break;
            case 'x': enter_text_mode('x', "\n-- ARRAY PROFILE --"); break;
            case 's': enter_text_mode('s', "\n-- TABLE --"); break;
            case 'n': enter_text_mode('n', "\n-- NOISE: hands off --"); break;
            case '[': g_autoScale = false; g_scale *= 0.5f;
                      if (g_scale < 0.05f) g_scale = 0.05f;
                      logf("scale +/-%.2f mV (manual)\n", g_scale); break;
            case ']': g_autoScale = false; g_scale *= 2.0f;
                      logf("scale +/-%.2f mV (manual)\n", g_scale); break;
            case '0': g_autoScale = true; logf("scale: auto\n"); break;
            case 'd': if (g_mode == 'B') g_mode = 'g'; settle_diag_cmd(); break;
            // r / D / p change the ADC config and restore the legacy one; a locked
            // mode is put back afterwards
            case 'r': if (g_mode == 'B') g_mode = 'g'; burst_cmd();
                      if (g_lk.mode) lkStart(); break;
            case 'D': if (g_mode == 'B') g_mode = 'g'; settle_fine_cmd();
                      if (g_lk.mode) lkStart(); break;
            case 'p': if (g_mode == 'B') g_mode = 'g'; spi_sweep_cmd();
                      if (g_lk.mode) lkStart(); break;
            case 'S': selects_cmd(); break;
            case 'M': mode_cmd(); break;
            default: break;    // ignore CR/LF and anything unknown
        }
    }

    bool streaming = g_mode == 'B' && g_lk.mode;
    if (streaming && !g_lk.streaming) lkHealthReset();   // HEALTH windows cover streaming only
    g_lk.streaming = streaming;
    switch (g_mode) {
        case 'B': if (g_lk.mode) lkService(20000);           // locked: sends from idle time
                  else if (scan_all()) send_scan();          // no delay: stream fast
                  break;
        case 'c': calibrate_step(); delay(60); break;
        case 'n': captureBaseline(48); noise_report(); delay(1000); break;
        case 's': if (scan_all()) print_scan();    delay(400); break;
        case 'x': if (scan_all()) print_profile(); delay(200); break;
        default:  if (scan_all()) print_grid();    delay(250); break;
    }
}
