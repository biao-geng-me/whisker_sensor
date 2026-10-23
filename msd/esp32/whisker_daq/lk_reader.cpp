// lk_reader.cpp — the locked-scan reader, split out of whisker_daq.ino.
#include <Arduino.h>
#include <SPI.h>
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "lk_reader.h"

LockState g_lk = {};
XQueue<XRec, LK_XQ_N> g_xq;
XCtl g_xc;
static TaskHandle_t g_lkTask = nullptr;
volatile uint32_t g_lkBeat = 0;
// Records to the host core instead of frames of our own (C1 streaming).
static inline bool lkToHost() { return g_lk.remote && g_lk.txHost; }
extern TaskHandle_t g_hostTask;     // whisker_daq.ino

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
static_assert(PIN_SELA_2 >= 32 && PIN_SELB_2 >= 32 && PIN_SELA_1 < 32 && PIN_SELB_1 < 32 &&
              PIN_SELC_1 < 32 && PIN_SELC_2 < 32 && PIN_CS < 32 && PIN_DRDY < 32,
              "muxFast / csFast assume these GPIO banks");

// Direct GPIO register access: digitalWrite x 6 took ~10 us per mux switch.
static uint32_t g_muxSet0[NUM_SEL], g_muxClr0[NUM_SEL], g_muxSet1[NUM_SEL], g_muxClr1[NUM_SEL];
void muxFastInit() {
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
static uint32_t IRAM_ATTR adsReadFast(long ch[4], bool *crcOk) {
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
const char *lkConfigure(uint8_t mode, long settle, long osr, long scanUs, long spi) {
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

void lkClearScan() {
    memset(g_lk.acc, 0, sizeof(g_lk.acc));
    memset(g_lk.cnt, 0, sizeof(g_lk.cnt));
    memset(g_lk.taint, 0, sizeof(g_lk.taint));
}

void IRAM_ATTR lkHealthReset() {
    memset(&g_lk.h, 0, sizeof(g_lk.h));
    g_lk.h.t0Us = micros();
}

// Restart the reader: wait for the next scan boundary on the conversion grid
// (newGrid: take a new grid from the next DRDY, e.g. after an ADC re-init).
void lkRearm(bool newGrid) {
    if (newGrid) g_lk.haveRef = false;
    g_lk.waitBoundary = true;
    g_lk.lastK = -1;
    g_lk.curSel = 0xFF;
    if (g_lk.pendingSend) {                  // a closed scan not packed yet (C0): send it,
        if (g_lk.streaming) send_scan();     // don't drop it silently (bench r5 audit)
        else g_lk.h.txDrop++;
        g_lk.pendingSend = false;
    }
    lkClearScan();
    g_lk.scanFlags = 0;                      // nothing of an interrupted scan carries over
    drdyResync();
    g_lk.lastActT = g_lk.lastEdgeT = micros();
}

// CLKIN 8 MHz, OSR and SPI of the configured mode, full ADC init, rearm.
bool lkStart() {
    FastCfg cfg;
    bool ok = fastCfgApply(LK_CLKIN, g_lk.osr, g_lk.spi, &cfg);
    g_lk.spiActual = cfg.spiActual;
    lkRearm(true);
    return ok;
}

void lkStop() {
    g_lk.mode = 0;
    g_lk.streaming = false;
    restoreNormalCfg();
}

void lkHealthPayload(uint8_t *p, const LockHealth &h, uint32_t win, uint8_t mode,
                     const HostHealth &hh) {
    if (!win) win = 1;
    p[0] = 2;
    p[1] = mode;
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
    p[42] = hh.core;
    p[43] = hh.qHwm;
    put_u16(p + 44, hh.txWinUs);
    put_u16(p + 46, hh.parks);
    put_u16(p + 48, hh.parkTimeouts);
    put_u16(p + 50, hh.hostTxMax);
    put_u16(p + 52, h.detMax);
    put_u16(p + 54, hh.txLate);
}

void lkMsgText(char *buf, int size, uint32_t code, uint32_t arg, uint8_t mode) {
    if (code == LKM_ADC_RESET)
        snprintf(buf, size, "[adc reset #%lu] in %s mode: re-initialising", (unsigned long)arg,
                 LK_NAME[mode]);
    else if (code == LKM_HALT)
        snprintf(buf, size, "[halt] no conversion for %d ms in %s mode: CLKIN restart + re-init",
                 LK_TIMEOUT_MS, LK_NAME[mode]);
    else
        snprintf(buf, size, "[reader message %lu %lu]", (unsigned long)code, (unsigned long)arg);
}

// A message from the reader: logged by whoever sends (the reader itself in C0,
// into its TX queue), queued for the host core in C1.
static void lkMsg(uint32_t code, uint32_t arg) {
    if (lkToHost()) {
        XRec r;
        r.kind = XR_MSG; r.mode = g_lk.mode; r.msg[0] = code; r.msg[1] = arg;
        g_xq.push(r);                        // (no notify: the host polls each tick)
        return;
    }
    char buf[120];
    lkMsgText(buf, sizeof(buf), code, arg, g_lk.mode);
    logf("%s\n", buf);
}

static void lkSendHealth() {               // C0: straight into the TX queue
    uint8_t p[LK_HEALTH_LEN];
    g_hh.core = 0;
    g_hh.qHwm = 0;
    lkHealthPayload(p, g_lk.h, micros() - g_lk.h.t0Us, g_lk.mode, g_hh);
    send_frame(FT_HEALTH, p, sizeof(p));
    g_hh.hostTxMax = 0;                   // per window (C1 fields; 0 in C0)
    g_hh.txLate = 0;
}

// The scan that just ended (its last conversion was read): averages -> outRaw,
// queue the frame (sent from idle time), next scan.
static void IRAM_ATTR lkCloseScan(uint32_t tEnd) {
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
    if (lkToHost()) {                        // C1: a record for the host core
        XRec r;
        r.kind = XR_SCAN; r.flags = g_lk.outFlags; r.mode = g_lk.mode;
        r.seq = g_scans; r.tUs = g_scanT0; r.durUs = g_scanDur;
        r.txAt = g_scanT0 + g_scanDur + g_lk.txWinUs;
        for (int o = 1; o <= NUM_OUT; o++) r.raw[o] = (int32_t)outRaw[o];
        if (!g_xq.push(r)) g_lk.h.txDrop++;  // (no notify: see lkTaskMain)
    } else if (g_lk.streaming) {
        if (g_lk.pendingSend) g_lk.h.txDrop++;      // the previous one never got idle time
        g_lk.pendingSend = true;
    }
    lkClearScan();
    g_lk.tRef += (uint32_t)g_lk.N * g_lk.T;
    g_lk.lastK = -1;
    if (micros() - g_lk.h.t0Us >= 1000000UL) {
        if (lkToHost()) {
            // HEALTH is never dropped: if the queue is full, the window just
            // runs on and goes out at a later scan end
            if (g_xq.full()) return;
            XRec r;
            r.kind = XR_HEALTH; r.mode = g_lk.mode; r.tUs = micros() - g_lk.h.t0Us;
            r.seq = g_xq.hwm;                // queue high-water mark of this window
            g_xq.hwm = 0;
            r.h = g_lk.h;
            g_xq.push(r);
        } else if (g_lk.streaming) {
            lkSendHealth();
        }
        lkHealthReset();
    }
}

static inline int32_t lkIndex(uint32_t t) {
    int32_t d = (int32_t)(t - g_lk.tRef), T = g_lk.T;
    return d >= 0 ? (d + T / 2) / T : -((-d + T / 2) / T);
}

// Reads one conversion if one is waiting. 0 = none waiting, 1 = handled.
static int IRAM_ATTR lkStep() {
    uint32_t e, t;
    bool edge;
    if ((int32_t)(g_drdyEdges - g_drdyTaken) > 0) {
        do { e = g_drdyEdges; t = g_drdyEdgeT; } while (e != g_drdyEdges);
        edge = true;
    } else if (drdyLowFast()) {
        // DRDY low with no edge: give a just-fired ISR time to count it, else
        // it is a conversion whose pulse was blocked by the previous read
        // (SBAS890D 8.5.1.5) or the FIFO's second slot after an overrun. The
        // ISR is on core 1: on the reader task (same core) ~3 us is plenty; on
        // the host core (text modes, baseline) core 1's tick can hold it back
        // ~8 us, so wait longer there (bench 2026-10-23 r3: C0 on core 0 lost
        // one conversion per LCM(1 ms tick, T) to this).
        uint32_t w0 = micros(), wait = g_lk.remote ? 3 : 15;
        while (micros() - w0 < wait) {}
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
        uint16_t det = (uint16_t)min<uint32_t>(r0 - t, 65535);
        if (det > h.detMax) h.detMax = det;
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
        lkMsg(LKM_ADC_RESET, g_adcReinits);
        lkStart();
        g_lk.scanFlags |= FLAG_REINIT;       // (after lkStart: its rearm clears the flags)
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
    lkMsg(LKM_HALT, 0);
    spiFlush();
    setClkin(LK_CLKIN);
    lkStart();
    g_lk.scanFlags |= FLAG_RECOVERED;        // (after lkStart: its rearm clears the flags)
}

// Run the reader for about budgetUs (it returns right after a conversion, so
// the caller's own work can't make the next read late). Idle time between
// conversions goes to the deferred frame and the TX chunks.
void lkService(uint32_t budgetUs) {
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
            uint32_t n = txqWrite(g_lk.T >= 64 ? 48 : 16, true);
            uint32_t dt = micros() - w0;
            if (n && dt > g_lk.h.txMax) g_lk.h.txMax = (uint16_t)min<uint32_t>(dt, 65535);
        } else if (g_lk.remote) {
            logqDrain(1);                    // captured ESP-IDF log lines, rate-limited
            continue;
        } else {
            continue;
        }
        g_lk.h.busyUs += micros() - w0;
    }
}

// ── The reader task (streaming, C0 and C1) ──────────────────────────────────
// The reader task is pinned to core 1 with the DRDY ISR (setup() runs there
// and attaches it); everything else runs on the host task on core 0, which
// also owns Serial (its UART ISR is on core 0). Ownership of the reader (g_lk,
// SPI, CS, mux, CLKIN) moves with the park/run handshake in lk_xcore.h: the
// host owns it while the task is parked (text modes, commands, legacy scan),
// the task while it streams a locked mode. In C0 the task also sends (the
// paced TX queue, as the single-core firmware did on core 1); in C1 it hands
// records to the host core. While it runs it makes no FreeRTOS calls at all (no
// notifies: the host polls the queue each tick; bench r5: the extra notify at
// the 1 s HEALTH push came with one overrun per window in wide mode), and its
// hot path (lkStep, lkCloseScan, adsReadFast) is in IRAM, so a rarely taken
// branch can't stall on flash cache misses. The task busy-polls; it is NOT on
// the task watchdog (bench r3: esp_task_wdt_reset from it logged "task not
// found" into the stream); the host watches g_lkBeat instead and restarts the
// ESP32 if the reader stops for LK_BEAT_TIMEOUT_MS. Priority 23: above
// everything of ours, below ipc1 (24), so cross-core IPC calls get through.
// SPI: the host holds the Arduino transaction (taken in hostSetup), so all
// SPIClass transfers take the no-lock path; the task never begins or ends a
// transaction (that mutex belongs to the host task) and never changes the SPI
// clock: lkStart's fastCfgApply only calls setSpi when the clock differs, and
// the host set it before handing over.
#define LK_TASK_PRIO   23
#define LK_TASK_STACK  4096

static void lkTaskMain(void *) {
    for (;;) {
        XcView v = xcPoll(g_xc);
        if (v.parking || v.ack) {
            xcParkDone(g_xc, v);           // the host owns everything from here
            continue;
        }
        if (!v.run) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
            continue;
        }
        if (v.starting) {
            lkResume();                    // drop the FIFO, next boundary, same grid
            xcStarted(g_xc, v);
        }
        g_lkBeat = g_lkBeat + 1;
        if (!g_lk.txHost) {                // C0: read and send, as the single-core loop
            // budget 0: back here after every conversion, so this loop runs at the
            // conversion rate, locked to the scan. With 2000 it came back every
            // 16 conversions (2048 us), a 256 ms = 16-scan cycle against the 16 ms
            // scan that moved the TX timing: bench r5, C0 lines at k x 3.906 Hz.
            lkService(0);
            continue;
        }
        if (lkStep()) continue;
        if (micros() - g_lk.lastActT > LK_TIMEOUT_MS * 1000UL) lkRecover();
    }
}

bool lkTaskStart() {
    if (xTaskCreatePinnedToCore(lkTaskMain, "lkReader", LK_TASK_STACK, nullptr, LK_TASK_PRIO,
                                &g_lkTask, 1) != pdPASS)
        g_lkTask = nullptr;
    return g_lkTask != nullptr;
}

bool lkTaskOk() { return g_lkTask != nullptr; }

void lkFlushHealth() {
    if (!g_lk.mode || g_lk.h.conv == 0 || micros() - g_lk.h.t0Us < 100000UL) return;
    uint8_t p[LK_HEALTH_LEN];
    g_hh.core = g_lk.txHost ? 1 : 0;
    g_hh.qHwm = (uint8_t)min<uint32_t>(g_xq.hwm, 255);
    lkHealthPayload(p, g_lk.h, micros() - g_lk.h.t0Us, g_lk.mode, g_hh);
    send_frame(FT_HEALTH, p, sizeof(p));
    g_hh.hostTxMax = 0;
    g_hh.txLate = 0;
    lkHealthReset();
}

void lkStateText(char *buf, int size) {
    uint32_t req = __atomic_load_n(&g_xc.req, __ATOMIC_ACQUIRE);
    uint32_t st = __atomic_load_n(&g_xc.state, __ATOMIC_ACQUIRE);
    snprintf(buf, size, "req %lu/%s state %lu/%s remote %d txHost %d streaming %d mode %u"
             " haveRef %d waitBoundary %d lastK %ld conv %lu last step %lu us ago drdy edges %lu"
             " taken %lu beat %lu task %s",
             (unsigned long)(req >> 1), (req & 1) ? "RUN" : "PARK", (unsigned long)(st >> 1),
             (st & 1) ? "RUNNING" : "PARKED", g_lk.remote, g_lk.txHost, g_lk.streaming, g_lk.mode,
             g_lk.haveRef, g_lk.waitBoundary, (long)g_lk.lastK, (unsigned long)g_lk.h.conv,
             (unsigned long)(micros() - g_lk.lastActT), (unsigned long)g_drdyEdges,
             (unsigned long)g_drdyTaken, (unsigned long)g_lkBeat, g_lkTask ? "ok" : "MISSING");
}

bool lkRemoteActive() { return xcWantRun(g_xc) || !xcParked(g_xc); }

// Host: take the reader back. The task lets go between two conversions, so
// this normally takes < one conversion period.
bool lkPark(uint32_t timeoutMs) {
    if (xcParked(g_xc)) return true;
    xcRequest(g_xc, XC_PARK);
    xTaskNotifyGive(g_lkTask);
    uint32_t t0 = millis();
    while (!xcParked(g_xc)) {
        if (millis() - t0 > timeoutMs) { g_hh.parkTimeouts++; return false; }
        delayMicroseconds(10);
    }
    g_lk.remote = false;
    g_lk.streaming = false;                // the host flushes what C0 left in the TX queue
    g_hh.parks++;
    return true;
}

// Host: hand the reader to core 1 for streaming. The queue must be empty.
void lkRun(bool txHost, uint32_t txWinUs) {
    if (!g_lkTask) return;
    g_lk.remote = true;
    g_lk.txHost = txHost;
    g_lk.streaming = true;
    g_lk.pendingSend = false;
    g_lk.txWinUs = txHost ? txWinUs : 0;
    lkHealthReset();
    xcRequest(g_xc, XC_RUN);
    xTaskNotifyGive(g_lkTask);
}

// One scan for the text modes (blocking; the reader resyncs after their pauses).
bool lkScanOnce() {
    g_lk.scanDone = false;
    uint32_t t0 = millis();
    while (!g_lk.scanDone) {
        lkService(2000);
        if (millis() - t0 > 1000) return false;
    }
    return true;
}

void lkPrintMode() {
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
