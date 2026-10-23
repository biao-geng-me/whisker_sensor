// lk_reader.h — the locked-scan reader (M1 quiet, M2 wide): state and API.
// See lk_reader.cpp for how it works.
#pragma once
#include "daq_common.h"

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

// ── Locked scan state (modes M1 quiet / M2 wide; see lk_reader.cpp)
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
extern LockState g_lk;

const char *lkConfigure(uint8_t mode, long settle, long osr, long scanUs, long spi);
void lkClearScan();
void lkHealthReset();
void lkRearm(bool newGrid);
bool lkStart();
void lkStop();
void lkService(uint32_t budgetUs);
bool lkScanOnce();
void lkPrintMode();
void muxFastInit();
