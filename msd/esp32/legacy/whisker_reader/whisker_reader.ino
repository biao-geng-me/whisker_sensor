/*
 * ============================================================================
 *  whisker_reader.ino — full 32-channel scan (16 whiskers x 2 gauges)
 *  MSD Seal Whisker Project · RIT · Flow Physics and Modeling Lab
 *  Rev 2026-08-04-c — mapping confirmed against the Altium netlist
 * ----------------------------------------------------------------------------
 *  Board : ELEGOO ESP32 (ESP-WROOM-32, 38-pin)   Arduino ESP32 core 3.x
 *  ADC   : ADS131M04IPWR (4-ch, 24-bit delta-sigma, NO internal oscillator)
 *  MUX   : 2x CD4097BM (dual 8:1)     Amp : INA351 (gain ~141)
 *  Serial: 115200 baud
 * ============================================================================
 *
 *  SIGNAL CHAIN
 *    whisker gauge -> 2 connector pins -> INA351 (diff, gain ~141) -> OUTn
 *    -> CD4097 channel -> COM_Xn/COM_Yn -> ADS131M04 AINxP -> SPI -> ESP32
 *
 *  Each physical whisker carries TWO strain gauges, so it occupies 4 connector
 *  pins and produces 2 OUT signals (called gauge A and B below).
 *    whisker W (1..16):  gauge A = OUT(2W-1),  gauge B = OUT(2W)
 *    OUT n:              CONN(2n-1) and CONN(2n)
 *
 *  MAPPING (from MAIN/Multiplexer/Whisker/Processing.SchDoc — see MUX_MAPPING.md)
 *    ADC:  AIN0P=COM_X2 (CH0)   AIN1P=COM_Y2 (CH1)
 *          AIN2P=COM_X1 (CH2)   AIN3P=COM_Y1 (CH3)
 *          all AINxN are grounded -> near-ground common mode -> PGA gain 1
 *    MUX:  XINn -> CD4097 X channel n-1,  YINn -> Y channel n-1  (in order)
 *    So (ADC channel, select) -> OUT is the OUT_OF[][] table below. The
 *    scrambled look comes from the OUT-net assignment on MAIN, done for PCB
 *    routing; it is not an error.
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
 *  SERIAL COMMANDS (type in the Serial Monitor)
 *    g  GRAPH mode (default) — live ASCII bar graph of all 32 gauges
 *    s  numeric table mode
 *    b  re-capture baseline (hold everything still)
 *    v  toggle: deviation-from-baseline (mV)  <->  absolute voltage (V)
 *    m  print the full connector/channel map
 *    c  calibration mode: deflect one whisker, it names the cell that moved
 *    n  noise-floor report (hands off the rig)
 *    [ ]  halve / double the graph scale     0  back to auto-scale
 *    p  PLOT STREAM: one compact machine-readable line per scan, for
 *       whisker_plot.py. Format:  $,<scan>,<railmask-hex>,<mv1>,...,<mv32>
 *
 *  GRAPH LAYOUT — the whole 32-gauge array across the screen, 104 columns
 *  wide by 17 lines, so it fills a wide Serial Monitor instead of a narrow
 *  strip. Each gauge is one vertical bar, centred on a zero axis: bars grow
 *  UP for positive deflection and DOWN for negative. Whiskers run left to
 *  right, gauge A then gauge B within each whisker.
 *
 *   +15.2 |                  ##
 *         |                  ##
 *     0.0 +-- -- -- -- -- -- -- -- :: :: -- -- ...
 *         |                  ##
 *   -15.2 |
 *          W01   W02   W03   W04   ...
 *           A B   A B   A B   a b
 *
 *    '::' on the axis and a lowercase gauge letter mean that channel is at
 *    full scale — nothing plugged in. Exact values for the live channels are
 *    listed underneath, so the numbers stay readable without eating width.
 * ============================================================================
 */

#include <SPI.h>

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
#define SPI_HZ     200000UL   // SCLK must stay BELOW CLKIN
// fMOD = CLKIN/2 = 400 kHz; OSR 128 -> fDATA = 3.125 kSPS -> 320 us/sample.

// ── ADS131M04 commands / registers ───────────────────────────────────────────
#define CMD_NULL    0x0000
#define CMD_RESET   0x0011
#define CMD_UNLOCK  0x0655
#define CMD_LOCK    0x0555
#define CMD_RREG(a) (0xA000 | ((a) << 7))
#define CMD_WREG(a) (0x6000 | ((a) << 7))
#define REG_ID      0x00
#define REG_CLOCK   0x03
#define REG_GAIN1   0x04
#define REG_GAIN2   0x05
// CLOCK: reserved[15:12]=1111 (preserve), CH_EN[11:8]=1111, OSR[4:2]=000 (128),
// PWR[1:0]=10 (high-resolution) -> 0xFF02
#define CLOCK_ALL_CH_OSR128  0xFF02
#define CLOCK_MASK  0x0FFF
#define GAIN_ALL_1  0x0000

#define VREF        1.2f
#define ADC_FS      8388608.0f   // 2^23
#define INA_GAIN    141.0f       // INA351 nominal, for referred-to-input display

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
static const char* CH_COMMON[NUM_ADC_CH] = {"COM_X2", "COM_Y2", "COM_X1", "COM_Y1"};

// CONN index within a J header (1..16) -> physical header pin.
// Every J header has its odd column transposed in pairs; this is that fix.
static const uint8_t CONN_TO_JPIN[17] =
  {0, 3, 1, 2, 4, 7, 5, 6, 8, 11, 9, 10, 12, 15, 13, 14, 16};

// ── State ────────────────────────────────────────────────────────────────────
SPIClass vspi(VSPI);

static float outV    [NUM_OUT + 1];   // latest voltage, indexed by OUT number
static long  outRaw  [NUM_OUT + 1];   // latest raw 24-bit code
static float outBase [NUM_OUT + 1];
static float outNoise[NUM_OUT + 1];

// A channel with nothing plugged in leaves the INA351 inputs floating, its
// output slams to a rail, and the ADC clips at a constant +/- full scale. That
// looks like a perfectly steady 0.000 mV deviation, which is misleading — so
// detect it and say "open" instead of printing a number.
#define SAT_CODE   8380000L           // 2^23 = 8388608; anything near it is clipped

static uint16_t g_id = 0, g_clockRb = 0;
static uint32_t g_halts = 0, g_crcFails = 0, g_unrecovered = 0, g_scans = 0;
static uint32_t g_recByLevel[4] = {0, 0, 0, 0};
static bool     g_absolute = false;   // 'v' toggles
static char     g_mode     = 'g';     // 'g' graph, 's' table, 'c' cal, 'n' noise, 'p' stream

// Graph geometry. BAR_H rows above and below the zero axis; each gauge gets
// COL_W characters of width, so the array spans NUM_OUT*COL_W columns.
#define BAR_H        6        // vertical half-height, rows
#define COL_W        3        // characters per gauge column
#define AXIS_W       8        // left margin holding the scale labels
#define SCALE_MIN  2.0f       // never zoom in tighter than this
static float g_scale    = SCALE_MIN;
static bool  g_autoScale = true;

inline void cs_low()  { digitalWrite(PIN_CS, LOW);  }
inline void cs_high() { digitalWrite(PIN_CS, HIGH); }

static bool initADC();
static bool readValid(long ch[4], uint32_t budget_ms);

// ── Whisker / connector helpers ──────────────────────────────────────────────
static inline uint8_t whiskerOfOut(uint8_t o) { return (o + 1) / 2; }      // 1..16
static inline char    gaugeOfOut  (uint8_t o) { return (o & 1) ? 'A' : 'B'; }
static inline uint8_t jOfOut      (uint8_t o) { return 2 + (o - 1) / 8; }  // J2..J5
// The two connector nets for an OUT, and their physical header pins.
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

// The register value is returned in WORD 0 of the NEXT frame — no leading
// status word to skip.
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

static bool waitDRDY(uint32_t timeout_ms) {
    uint32_t dl = millis() + timeout_ms;
    while (digitalRead(PIN_DRDY) == HIGH) { if (millis() > dl) return false; }
    return true;
}

// CRC-16-CCITT (poly 0x1021, init 0xFFFF) — the ADC's output-frame CRC.
// Floating-bus noise can fake a plausible STATUS word; it cannot fake this.
static uint16_t crc16_ccitt(const uint8_t *d, int n) {
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint32_t ads_read_all(long ch[4], bool *crcOk) {
    uint8_t buf[15];                        // STATUS + CH0..CH3, 3 bytes each
    cs_low();
    for (int i = 0; i < 15; i++) buf[i] = vspi.transfer(0x00);
    uint16_t rxCrc  = (uint16_t)vspi.transfer(0x00) << 8;
    rxCrc          |= (uint16_t)vspi.transfer(0x00);
    vspi.transfer(0x00);                    // CRC word pad byte
    cs_high();
    uint32_t status = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    for (int i = 0; i < 4; i++) {
        uint32_t raw = ((uint32_t)buf[3 + 3*i] << 16) |
                       ((uint32_t)buf[4 + 3*i] << 8)  |
                        (uint32_t)buf[5 + 3*i];
        if (raw & 0x800000UL) raw |= 0xFF000000UL;   // sign-extend
        ch[i] = (long)(int32_t)raw;
    }
    *crcOk = (crc16_ccitt(buf, 15) == rxCrc);
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

static void restartCLKIN() {
    ledcDetach(PIN_CLKIN);
    delay(5);
    ledcAttach(PIN_CLKIN, CLKIN_HZ, 1);
    ledcWrite(PIN_CLKIN, 1);
    delay(20);
}

static bool initADC() {
    spi_word(CMD_RESET);
    delay(50);
    waitDRDY(300);
    spi_word(CMD_UNLOCK);
    delayMicroseconds(50);
    ads_write_reg(REG_CLOCK, CLOCK_ALL_CH_OSR128);
    ads_write_reg(REG_GAIN1, GAIN_ALL_1);
    ads_write_reg(REG_GAIN2, GAIN_ALL_1);
    g_clockRb = ads_read_reg(REG_CLOCK);     // read back BEFORE locking
    g_id      = ads_read_reg(REG_ID);
    spi_word(CMD_LOCK);
    delay(10);
    waitDRDY(300);
    return (g_clockRb & CLOCK_MASK) == (CLOCK_ALL_CH_OSR128 & CLOCK_MASK);
}

static bool readValid(long ch[4], uint32_t budget_ms) {
    uint32_t dl = millis() + budget_ms;
    while (millis() < dl) {
        if (!waitDRDY(50)) continue;
        bool crcOk = false;
        uint32_t st = ads_read_all(ch, &crcOk);
        if (statusValid(st)) {
            if (crcOk) return true;
            g_crcFails++;                     // looked alive, was noise
        }
    }
    return false;
}

// Levels: 1 soft re-init · 2 SPI flush + re-init · 3 CLKIN restart (+HW reset)
static int recoverADC(long ch[4]) {
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
        if (readValid(ch, 300)) return level;
    }
    return 0;
}

static bool readOrRecover(long ch[4]) {
    if (readValid(ch, 250)) return true;
    g_halts++;
    int lvl = recoverADC(ch);
    if (lvl > 0) {
        g_recByLevel[lvl]++;
        Serial.printf("[halt #%lu] recovered at L%d  (L1=%lu L2=%lu L3=%lu fail=%lu crcbad=%lu)\n",
                      (unsigned long)g_halts, lvl,
                      (unsigned long)g_recByLevel[1], (unsigned long)g_recByLevel[2],
                      (unsigned long)g_recByLevel[3], (unsigned long)g_unrecovered,
                      (unsigned long)g_crcFails);
        return true;
    }
    g_unrecovered++;
    Serial.printf("[halt #%lu] UNRECOVERED  ID=0x%04X CLOCKrb=0x%04X crcbad=%lu"
                  "  -> power-cycle the PCB with the ESP32 already running\n",
                  (unsigned long)g_halts, g_id, g_clockRb, (unsigned long)g_crcFails);
    delay(500);
    return false;
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
// Guards against driving a select line that is still jumpered to GND.
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
//  Scan — sweep select 0..7, fill outV[1..32]
// ─────────────────────────────────────────────────────────────────────────────
static bool scan_all() {
    long ch[4];
    for (uint8_t sel = 0; sel < NUM_SEL; sel++) {
        mux_select(sel);
        delayMicroseconds(SETTLE_US);
        for (uint8_t d = 0; d < DISCARD_SAMPLES; d++)   // flush the sinc filter
            if (!readOrRecover(ch)) return false;
        if (!readOrRecover(ch)) return false;
        for (uint8_t a = 0; a < NUM_ADC_CH; a++) {
            uint8_t o = OUT_OF[a][sel];
            outRaw[o] = ch[a];
            outV  [o] = (ch[a] / ADC_FS) * VREF;
        }
    }
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
    if (got < 2) { Serial.println("Baseline FAILED — no valid scans."); return; }
    for (int o = 1; o <= NUM_OUT; o++) {
        double m = sum[o] / got, v = sq[o] / got - m * m;
        outBase [o] = (float)m;
        outNoise[o] = (float)(v > 0 ? sqrt(v) : 0);
    }
    Serial.printf("Baseline captured from %d scans.\n", got);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Printing
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

static inline bool isRailed(uint8_t o) { return labs(outRaw[o]) >= SAT_CODE; }

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
    Serial.printf("\n--- scan #%lu   %s   (halts=%lu crcbad=%lu, %d/%d open) ---\n",
                  (unsigned long)g_scans,
                  g_absolute ? "absolute V at ADC" : "deviation from baseline, mV",
                  (unsigned long)g_halts, (unsigned long)g_crcFails, railed, NUM_OUT);
    for (uint8_t w = 1; w <= NUM_WHISKER; w++) {
        Serial.printf("  W%-2u", w);
        printCell('A', 2 * w - 1);
        printCell('B', 2 * w);
        Serial.println();
    }
}

// ── ASCII bar graph (horizontal array view) ──────────────────────────────────
// Track the largest live deflection and size the bars to it. Grows instantly,
// decays slowly, so a one-off tap doesn't leave the display zoomed out.
static void updateScale() {
    float peak = 0;
    for (int o = 1; o <= NUM_OUT; o++)
        if (!isRailed(o)) {
            float d = fabsf(outV[o] - outBase[o]) * 1000.0f;
            if (d > peak) peak = d;
        }
    if (!g_autoScale) return;
    if (peak > g_scale)        g_scale = peak * 1.15f;
    else if (g_scale > SCALE_MIN) {
        g_scale *= 0.98f;                          // slow decay
        if (g_scale < SCALE_MIN) g_scale = SCALE_MIN;
    }
}

static inline float devMv(uint8_t o) { return (outV[o] - outBase[o]) * 1000.0f; }

// ── Array profile view ('x') — every gauge as a vertical bar, left to right.
// Best once most whiskers are connected; with a mostly-empty array it is a lot
// of ':' and not much signal, which is why the grid below is the default.
static void print_profile() {
    updateScale();

    // Bar height in rows for each gauge; INT8_MIN marks an open channel.
    int8_t n[NUM_OUT + 1];
    int open = 0;
    for (int o = 1; o <= NUM_OUT; o++) {
        if (isRailed(o)) { n[o] = INT8_MIN; open++; continue; }
        int v = (int)lroundf(devMv(o) / g_scale * BAR_H);
        if (v >  BAR_H) v =  BAR_H;
        if (v < -BAR_H) v = -BAR_H;
        n[o] = (int8_t)v;
    }

    Serial.printf("\n== scan %lu | %d/%d open | scale +/-%.2f mV | halts %lu crc %lu ==\n",
                  (unsigned long)g_scans, open, NUM_OUT, g_scale,
                  (unsigned long)g_halts, (unsigned long)g_crcFails);

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
            row[i * COL_W + 2] = (r == 0) ? fill : ' ';   // solid axis line
        }
        row[NUM_OUT * COL_W] = '\0';
        if      (r ==  BAR_H) Serial.printf("%+6.1f |%s\n",  g_scale, row);
        else if (r == -BAR_H) Serial.printf("%+6.1f |%s\n", -g_scale, row);
        else if (r ==  0)     Serial.printf("%6.1f +%s\n",   0.0f,    row);
        else                  Serial.printf("       |%s\n",           row);
    }

    // Whisker numbers, then gauge letters (lowercase = open).
    Serial.printf("%*s", AXIS_W, "");
    for (uint8_t w = 1; w <= NUM_WHISKER; w++) Serial.printf("W%02u   ", w);
    Serial.println();
    Serial.printf("%*s", AXIS_W, "");
    for (uint8_t w = 1; w <= NUM_WHISKER; w++)
        Serial.printf(" %c  %c ", isRailed(2*w - 1) ? 'a' : 'A',
                                  isRailed(2*w)     ? 'b' : 'B');
    Serial.println();

    // Exact values for connected channels only — keeps the numbers readable
    // without spending 200+ columns on 32 mostly-empty fields.
    int shown = 0;
    for (int o = 1; o <= NUM_OUT; o++) {
        if (isRailed(o)) continue;
        if (shown % 6 == 0) Serial.print(shown ? "\n      " : "live: ");
        Serial.printf(" W%02u%c%+7.2f", whiskerOfOut(o), gaugeOfOut(o), devMv(o));
        shown++;
    }
    Serial.println(shown ? "" : "live: (none — every channel is open)");
}

// ── GRID VIEW (default) ──────────────────────────────────────────────────────
// All 32 gauges as 4 columns x 8 rows. Each cell carries its own label, a
// centre-zero bar and the value, so a channel can be read without looking
// anywhere else. Left half is whiskers 1-8, right half is whiskers 9-16.
//   W03A [--===|     ] + 28.85
#define CELL_H  5                     // bar half-width; full bar = 2*CELL_H+1

static void print_cell(uint8_t o) {
    Serial.printf("W%02u%c ", whiskerOfOut(o), gaugeOfOut(o));
    // Same printed width as a live cell: 13 (bar) + 1 + 7 (value) + 1 = 22.
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
        else if (k < 0 && n < 0 && k >= n)      c = '=';   // negative, grows left
        else if (k > 0 && n > 0 && k <= n)      c = '=';   // positive, grows right
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
                  " | halts %lu crc %lu =====\n",
                  (unsigned long)g_scans, live, open, g_scale,
                  (unsigned long)g_halts, (unsigned long)g_crcFails);
    if (peakOut)
        Serial.printf("  largest: W%02u gauge %c  %+.2f mV\n",
                      whiskerOfOut(peakOut), gaugeOfOut(peakOut), peak);
    else
        Serial.println("  largest: (nothing connected)");
    Serial.println("      whiskers 1-8                                "
                   "  whiskers 9-16");

    for (uint8_t r = 1; r <= 8; r++) {
        print_cell(2 * r - 1);              // W r  gauge A
        print_cell(2 * r);                  // W r  gauge B
        Serial.print(" ");
        print_cell(2 * (r + 8) - 1);        // W r+8 gauge A
        print_cell(2 * (r + 8));            // W r+8 gauge B
        Serial.println();
    }
}

// Compact stream for whisker_plot.py. One line per scan, OUT order 1..32,
// mV deviation from baseline. The rail mask marks channels reading full scale
// (nothing plugged in) so the plotter can grey them out instead of drawing a
// flat line that looks like a healthy idle channel.
static void print_stream() {
    uint32_t railMask = 0;
    for (int o = 1; o <= NUM_OUT; o++) if (isRailed(o)) railMask |= (1UL << (o - 1));
    Serial.printf("$,%lu,%08lX", (unsigned long)g_scans, (unsigned long)railMask);
    for (int o = 1; o <= NUM_OUT; o++)
        Serial.printf(",%.2f", (outV[o] - outBase[o]) * 1000.0f);
    Serial.println();
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
#define CAL_MIN_MV  0.5f    // absolute floor, mV
#define CAL_SIGMA   6.0f    // ...and must exceed this many sigma
#define CAL_MARGIN  2.5f    // winner must beat runner-up by this factor

static void calibrate_step() {
    if (!scan_all()) return;
    float best = 0, second = 0; int bo = -1;
    for (int o = 1; o <= NUM_OUT; o++) {
        float d = fabsf(outV[o] - outBase[o]);
        if (d > best) { second = best; best = d; bo = o; }
        else if (d > second) second = d;
    }
    if (bo < 0 || isRailed(bo)) return;      // ignore clipped/open channels
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

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=================================================");
    Serial.println("  Whisker array — 16 whiskers x 2 gauges = 32 ch");
    Serial.println("=================================================");

    // ── Select lines ─────────────────────────────────────────────────────────
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
    }
    Serial.println("Selects driven by ESP32 — full 8-position sweep enabled.");

    pinMode(PIN_CS, OUTPUT);  cs_high();
    pinMode(PIN_DRDY, INPUT);
    pinMode(PIN_RESET, OUTPUT); digitalWrite(PIN_RESET, HIGH);   // idle = not reset

    // ── CLKIN (ledcAttach returns a BOOL in core 3.x, not a frequency) ───────
    bool ok = ledcAttach(PIN_CLKIN, CLKIN_HZ, 1);
    ledcWrite(PIN_CLKIN, 1);
    Serial.printf("CLKIN: %s, %lu Hz    SCLK: %lu Hz    fDATA ~ %lu SPS\n",
                  ok ? "on" : "*** FAILED — ADC will be dead ***",
                  (unsigned long)ledcReadFreq(PIN_CLKIN),
                  (unsigned long)SPI_HZ, (unsigned long)(CLKIN_HZ / 2 / 128));
    delay(30);

    // ── SPI + ADC ────────────────────────────────────────────────────────────
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
    Serial.printf("Device ID      = 0x%04X  %s\n", g_id,
                  ((g_id >> 8) == 0x22 || (g_id >> 8) == 0x20) ? "(responding)" : "(NO RESPONSE)");
    Serial.printf("CLOCK readback = 0x%04X  %s\n", g_clockRb,
                  cfg ? "(config ACCEPTED)" : "(NOT accepted)");

    int edges = 0, last = digitalRead(PIN_DRDY);
    uint32_t t0 = millis();
    while (millis() - t0 < 200) { int c = digitalRead(PIN_DRDY); if (c != last) { edges++; last = c; } }
    Serial.printf("DRDY edges/200ms = %d  (%s)\n", edges, edges > 0 ? "converting" : "NOT toggling");

    print_map();

    Serial.println("\nCapturing baseline — keep all whiskers still...");
    captureBaseline(16);
    Serial.println("\nCommands: g=grid  x=array profile  s=table  b=rebaseline  v=abs/delta  m=map");
    Serial.println("          c=calibrate  n=noise  p=stream   [ ]=scale  0=auto-scale");
    Serial.println("Tip: set the Serial Monitor line ending to \"No line ending\" so a");
    Serial.println("stray newline doesn't get read as a command.\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    while (Serial.available()) {
        char k = Serial.read();
        if (k == 'b') { Serial.println("Re-capturing baseline..."); captureBaseline(16); }
        else if (k == 'v') { g_absolute = !g_absolute; }
        else if (k == 'm') { print_map(); }
        else if (k == 'c') { g_mode = 'c'; Serial.println("\n-- CALIBRATE: deflect ONE gauge at a time --"); }
        else if (k == 'g') { g_mode = 'g'; Serial.println("\n-- GRID --"); }
        else if (k == 'x') { g_mode = 'x'; Serial.println("\n-- ARRAY PROFILE --"); }
        else if (k == 's') { g_mode = 's'; Serial.println("\n-- TABLE --"); }
        else if (k == 'n') { g_mode = 'n'; Serial.println("\n-- NOISE: hands off --"); }
        else if (k == 'p') { g_mode = 'p'; Serial.println("\n-- PLOT STREAM --"); }
        else if (k == '[') { g_autoScale = false; g_scale *= 0.5f;
                             if (g_scale < 0.05f) g_scale = 0.05f;
                             Serial.printf("scale +/-%.2f mV (manual)\n", g_scale); }
        else if (k == ']') { g_autoScale = false; g_scale *= 2.0f;
                             Serial.printf("scale +/-%.2f mV (manual)\n", g_scale); }
        else if (k == '0') { g_autoScale = true; Serial.println("scale: auto"); }
    }

    if      (g_mode == 'c') { calibrate_step(); delay(60); }
    else if (g_mode == 'n') { captureBaseline(48); noise_report(); delay(1000); }
    else if (g_mode == 'p') { if (scan_all()) print_stream(); }   // no delay: stream fast
    else if (g_mode == 's') { if (scan_all()) print_scan();    delay(400); }
    else if (g_mode == 'x') { if (scan_all()) print_profile(); delay(200); }
    else                    { if (scan_all()) print_grid();    delay(250); }
}
