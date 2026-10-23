// lk_reader.h — the locked-scan reader (M1 quiet, M2 wide): state and API.
// See lk_reader.cpp for how it works.
#pragma once
#include "daq_common.h"
#include "lk_xcore.h"

#define LK_CLKIN           8000000UL
#define LK_LAT_ALLOW_US    20      // switch latency allowed for in the discard count (20:
                                   // core 1's ~8-12 us tick can land on a slot end every
                                   // scan in wide mode; same discard counts as 10 at settle
                                   // 200 for OSR 128/256/512)
#define LK_SETTLE_DEFAULT  200
#define LK_TX_GUARD_US     6       // idle work only if the next DRDY is this far off (a
                                   // slightly late read is harmless except at a slot end,
                                   // which idle work never precedes)
#define LK_TIMEOUT_MS      50      // no usable conversion for this long: re-init the ADC
#define LK_UNREC_N         5       // this many failed re-inits in a row: UNRECOVERED, then
#define LK_UNREC_RETRY_MS  5000    // ... one more re-init (and UNRECOVERED line) this often
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
    uint16_t latMax, readMax, txMax, detMax;   // detMax: DRDY edge -> start of the read
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
    uint32_t lastGoodT;             // the last usable conversion (CRC ok, fresh, no RESET)
    uint32_t haltT;                 // end of the last halt recovery
    uint32_t lastSvcT;              // the last return from lkService
    uint8_t  haltRun;               // halt recoveries since the last usable conversion
    int64_t  acc[NUM_SEL][NUM_ADC_CH];
    uint16_t cnt[NUM_SEL];
    bool     taint[NUM_SEL];
    uint8_t  scanFlags;             // flags of the scan being taken
    uint8_t  outFlags;              // flags of the last finished scan (send_scan)
    bool     remote;                // the reader task on core 1 owns the reader (streaming)
    bool     txHost;                // remote: records go to the host core, which sends
                                    // them (C1); else the reader sends them itself (C0)
    uint32_t txWinUs;               // C1: scan frames are sent this long after the
                                    // scan start (0 = as soon as they are ready)
    LockHealth h;
};
extern LockState g_lk;

// ── Dual-core (C1): the reader task on core 1 and the host task on core 0 ───
// Records the reader hands to the host (whisker_daq.ino formats and sends them).
enum : uint8_t { XR_SCAN = 1, XR_HEALTH = 2, XR_MSG = 3 };
enum : uint8_t { LKM_ADC_RESET = 1, LKM_HALT = 2, LKM_UNREC = 3, LKM_ADC_OK = 4 };
struct XRec {
    uint8_t  kind, flags, mode, pad;
    uint32_t seq, tUs, durUs;       // SCAN; HEALTH: tUs = window length, us
    uint32_t txAt;                  // SCAN: micros() to send it at (windowed TX)
    union {
        int32_t    raw[NUM_OUT + 1];   // SCAN: codes by OUT number (as outRaw)
        LockHealth h;                  // HEALTH
        uint32_t   msg[4];             // MSG: code, arguments (lkMsgText)
    };
};
#define LK_XQ_N  32                 // 128 ms of scans at 250 Hz
extern XQueue<XRec, LK_XQ_N> g_xq;
extern XCtl g_xc;

// HEALTH version 2: the host's fields (whisker_daq.ino keeps them).
struct HostHealth {
    uint8_t  core;                  // 0 = C0 (host core reads), 1 = C1
    uint8_t  qHwm;
    uint16_t txWinUs, parks, parkTimeouts, hostTxMax, txLate;
};
extern HostHealth g_hh;
#define LK_HEALTH_LEN  56
// The HEALTH payload (version 2) from a reader window and the host's fields.
void lkHealthPayload(uint8_t *p, const LockHealth &h, uint32_t winUs, uint8_t mode,
                     const HostHealth &hh);
// Text of a reader message (XR_MSG), for the host to log.
void lkMsgText(char *buf, int size, const uint32_t msg[4], uint8_t mode);

bool lkTaskStart();                 // setup(), on core 1: the reader task, parked (false: failed)
bool lkTaskOk();                    // the reader task exists
// Host, after taking the reader back: send the HEALTH window streamed so far
// (frequent commands would otherwise never let a 1 s window complete).
void lkFlushHealth();
// One line with the reader's state (ownership, flags, counters) for reports.
void lkStateText(char *buf, int size);
bool lkRemoteActive();              // host: the reader task owns (or was asked to own)
bool lkPark(uint32_t timeoutMs);    // host: take the reader back; false = it didn't let go
void lkRun(bool txHost, uint32_t txWinUs);   // host: hand the reader to core 1 (streaming)
extern volatile uint32_t g_lkBeat;  // the reader task's heartbeat (software watchdog)
// Host side (whisker_daq.ino): forward up to `max` captured ESP-IDF log lines
// as TEXT records (the reader calls it from idle time in C0 streaming).
void logqDrain(int max);

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
