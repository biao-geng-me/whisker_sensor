// Host simulation of whisker_daq.ino: the real sketch (arduino-cli --preprocess
// output, see run_sim.sh) compiled against mock/ and driven by a virtual clock
// with a model of the ADS131M04, the mux and the UART. It checks the firmware's
// logic (locked scan timing, slot averaging, sync handling, framing), not the
// ESP32's real speed: the CPU costs below are rough guesses, scalable with
// --cost-scale.
//
// ADC model: conversions every T = 2*OSR/CLKIN on a grid restarted by RESET,
// a CLOCK write or a CLKIN change; a 2-deep output FIFO; DRDY low while data
// are unread, high from the start of a read; a conversion that completes
// during a read gives no falling edge of its own (SBAS890D 8.5.1.5), one that
// completes while DRDY is already low gives a new edge, a read that leaves data
// in the FIFO ends with a narrow high pulse (a falling edge at the read end); the STATUS RESET flag is set by
// a reset and cleared by writing MODE bit 10 = 0; commands are parsed from the
// first word, the response comes in word 0 of the next frame, RESET needs the
// full 18-byte frame. A conversion is the box mean of the analog input over
// its 3T sinc3 window; the input after a mux switch is the new channel plus
// KICK x (old - new) x exp(-dt/TAU) (the measured charge-sharing tail).
//
// Usage: sim --end 3 --cmd 1.0:S0123457; --cmd 1.0:M2; --cmd 1.05:B --out out.bin
//        [--stall T:US] [--adc-reset T] [--seed N] [--cost-scale X] [--no-pulse]
//        [--trace T0:T1]  (one stderr line per reader step between T0 and T1 s)
#include "Arduino.h"
#include "SPI.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#include <array>
#include <deque>
#include <random>
#include <string>
#include <utility>
#include <vector>

// ---------------------------------------------------------------- costs (us)
struct Costs {
    double micros = 0.15, reg = 0.03, dwrite = 1.0, dread = 0.5;
    double isr_lat = 1.0, isr = 2.0, spi_ovh = 3.0, ser_call = 2.0, ser_byte = 0.02;
    double avail = 0.5, loop = 3.0, byte_xfer_ovh = 1.0;
} C;

static double sim_t = 0;                  // virtual time, us
static bool sim_inIsr = false;
static void (*sim_isr)(void) = nullptr;
static bool sim_isrPending = false;
static double sim_isrDue = 0;
static std::mt19937 sim_rng(1);
// --stall T:US: at T the sketch is preempted for US (a higher-priority task or
// an interrupt hog); the ADC and the DRDY ISR keep running. It can land
// anywhere, also inside an SPI read (CS low).
static std::vector<std::pair<double, double>> sim_stalls;
static bool sim_pulse = true;            // narrow DRDY pulse after a read that leaves data

// ---------------------------------------------------------------- pins
enum { P_CS = 21, P_DRDY = 4, P_CLKIN = 16, P_A1 = 25, P_B1 = 26, P_C1 = 27, P_A2 = 32, P_B2 = 33,
       P_C2 = 14 };
static uint8_t sim_level[40], sim_mode[40];

static const uint8_t SIM_OUT_OF[4][8] = {
    {32, 30, 28, 26, 31, 29, 27, 25},
    {18, 20, 22, 24, 23, 21, 19, 17},
    {5, 7, 3, 1, 2, 4, 6, 8},
    {15, 13, 11, 9, 10, 12, 16, 14},
};
static const double KICK = 0.0136, TAU = 80.0;
static uint32_t sim_railedOuts = 0;       // bit o-1: OUT o reads +full scale

static int32_t outValue(int o) {        // the "true" input of OUT o, in codes
    if (sim_railedOuts >> (o - 1) & 1) return 8388607;
    return -1600000 + 100003 * o;
}

struct MuxHist {                         // last switch per mux (ADC CH0/1: mux 2, CH2/3: mux 1)
    int cur = 0, prev = 0;
    double tsw = -1e9;
};
static MuxHist sim_mux[2];

static void muxUpdate(int m, int sel) {
    MuxHist &h = sim_mux[m];
    if (sel == h.cur) return;
    if (sim_t - h.tsw < 20) h.cur = sel;   // a multi-pin change in progress: same switch
    else { h.prev = h.cur; h.cur = sel; }
    h.tsw = sim_t;
}
static void pinsChanged() {
    int s1 = (sim_level[P_A1] ? 1 : 0) | (sim_level[P_B1] ? 2 : 0) | (sim_level[P_C1] ? 4 : 0);
    int s2 = (sim_level[P_A2] ? 1 : 0) | (sim_level[P_B2] ? 2 : 0) | (sim_level[P_C2] ? 4 : 0);
    muxUpdate(0, s1);
    muxUpdate(1, s2);
}

static double analog(int ch, double t) {
    const MuxHist &h = sim_mux[ch < 2 ? 1 : 0];
    if (t < h.tsw) return outValue(SIM_OUT_OF[ch][h.prev]);
    double vn = outValue(SIM_OUT_OF[ch][h.cur]), vp = outValue(SIM_OUT_OF[ch][h.prev]);
    return vn + KICK * (vp - vn) * exp(-(t - h.tsw) / TAU);
}

// ---------------------------------------------------------------- ADC
struct Adc {
    uint16_t reg[64]{};
    bool resetFlag = true;
    uint32_t clkin = 0;
    double T = 1e18, nextConv = 1e18;
    std::deque<std::array<int32_t, 4>> fifo;
    std::array<int32_t, 4> last{};
    bool drdyLow = false;
    bool inFrame = false, fresh = false;
    uint8_t out[64]{}, in[64]{};
    int pos = 0;
    uint16_t resp = 0;
    bool haveResp = false;
    uint32_t spiHz = 600000;
    uint64_t nConv = 0, nBlocked = 0, nOverwrite = 0, nPulse = 0;

    int osr() const { int c = (reg[3] >> 2) & 7; return 128 << c; }
    void restartConv(double delay = 0) {
        T = clkin ? 2.0 * osr() / clkin * 1e6 : 1e18;
        nextConv = clkin ? sim_t + delay + T : 1e18;
    }
    void reset() {
        memset(reg, 0, sizeof(reg));
        reg[0] = 0x2405; reg[2] = 0x0510; reg[3] = 0x0F0E;
        resetFlag = true;
        fifo.clear();
        drdyLow = false;
        haveResp = false;
        restartConv(10);
    }
    void edge() {
        if (!sim_isr) return;
        if (!sim_isrPending) { sim_isrPending = true; sim_isrDue = sim_t + C.isr_lat; }
        // a second edge before the ISR ran merges with the first (one status bit)
    }
    void complete() {
        nConv++;
        std::array<int32_t, 4> v{};
        std::normal_distribution<double> nz(0, 10);
        for (int c = 0; c < 4; c++) {
            double s = 0;
            for (int i = 0; i < 12; i++) s += analog(c, sim_t - 3 * T * (i + 0.5) / 12);
            double x = s / 12 + nz(sim_rng);
            v[c] = (int32_t)std::max(-8388608.0, std::min(8388607.0, std::round(x)));
        }
        if (fifo.size() >= 2) { fifo.pop_front(); nOverwrite++; }
        fifo.push_back(v);
        if (inFrame) { nBlocked++; }               // pulse blocked by the read
        else { drdyLow = true; edge(); }
    }
    uint16_t status() const {
        return 0x0100 | (resetFlag ? 0x0400 : 0) | (fresh ? 0x000F : 0);
    }
    void frameStart() {
        inFrame = true;
        pos = 0;
        fresh = !fifo.empty();
        const std::array<int32_t, 4> &d = fresh ? fifo.front() : last;
        uint16_t w0 = haveResp ? resp : status();
        haveResp = false;
        memset(out, 0, sizeof(out));
        out[0] = w0 >> 8; out[1] = w0 & 0xFF;
        for (int c = 0; c < 4; c++) {
            uint32_t u = (uint32_t)d[c] & 0xFFFFFF;
            out[3 + 3 * c] = u >> 16; out[4 + 3 * c] = u >> 8; out[5 + 3 * c] = u;
        }
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < 15; i++) {
            crc ^= (uint16_t)out[i] << 8;
            for (int k = 0; k < 8; k++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
        out[15] = crc >> 8; out[16] = crc & 0xFF;
        if (fresh) drdyLow = false;
    }
    uint8_t xfer(uint8_t b) {
        uint8_t r = pos < 64 ? out[pos] : 0;
        if (pos < 64) in[pos] = b;
        pos++;
        return r;
    }
    void frameEnd() {
        inFrame = false;
        if (pos >= 6 && fresh) { last = fifo.front(); fifo.pop_front(); }
        uint16_t cmd = (uint16_t)(in[0] << 8 | in[1]);
        if (cmd == 0x0011) {
            if (pos >= 18) { reset(); resp = 0xFF24; haveResp = true; }
        } else if (cmd == 0x0655 || cmd == 0x0555) {
            resp = cmd; haveResp = true;
        } else if ((cmd & 0xE000) == 0xA000) {
            resp = reg[(cmd >> 7) & 0x3F]; haveResp = true;
        } else if ((cmd & 0xE000) == 0x6000 && pos >= 6) {
            int a = (cmd >> 7) & 0x3F;
            uint16_t v = (uint16_t)(in[3] << 8 | in[4]);
            reg[a] = v;
            if (a == 2) resetFlag = (v >> 10) & 1;
            if (a == 3) { fifo.clear(); drdyLow = false; restartConv(); }
            resp = 0x4000 | (a << 7); haveResp = true;
        }
        bool left = !fifo.empty();
        // Data left after a read (a 2-deep FIFO, or a conversion that completed
        // during the read): DRDY goes high for a few CLKIN cycles at the end of
        // the read and falls again (SBAS890D Fig 8-21 "narrow DRDY pulse"), a
        // falling edge the ESP32 catches. --no-pulse: stays low, no edge.
        if (left && sim_pulse && fresh && pos >= 6) { nPulse++; edge(); }
        drdyLow = left;
    }
} adc;

// ---------------------------------------------------------------- time
static void advance(double dt) {
    double end = sim_t + dt;
    for (auto &s : sim_stalls)
        if (s.second > 0 && !sim_inIsr && s.first <= end) {
            end += s.second;
            s.second = 0;
        }
    for (;;) {
        double tc = adc.nextConv;
        double ti = (!sim_inIsr && sim_isrPending) ? sim_isrDue : 1e18;
        double tn = std::min(tc, ti);
        if (tn > end) break;
        if (tn > sim_t) sim_t = tn;
        if (tc <= ti) {
            adc.complete();
            adc.nextConv += adc.T;
        } else {
            sim_isrPending = false;
            double t0 = sim_t;
            sim_inIsr = true;
            sim_isr();
            sim_t += C.isr;
            sim_inIsr = false;
            end += sim_t - t0;                     // the ISR preempted the main code
        }
    }
    sim_t = end;
}

uint32_t micros() { advance(C.micros); return (uint32_t)(uint64_t)sim_t; }
uint32_t millis() { advance(C.micros); return (uint32_t)(uint64_t)(sim_t / 1000); }
void delay(uint32_t ms) { advance(ms * 1000.0); }
void delayMicroseconds(uint32_t us) { advance(us); }

static void setLevel(uint8_t pin, uint8_t v) {
    uint8_t old = sim_level[pin];
    sim_level[pin] = v ? 1 : 0;
    if (pin == P_CS) {
        if (old && !v) adc.frameStart();
        else if (!old && v) adc.frameEnd();
    }
    if (pin == P_A1 || pin == P_B1 || pin == P_C1 || pin == P_A2 || pin == P_B2 || pin == P_C2)
        pinsChanged();
}
void pinMode(uint8_t pin, uint8_t mode) { sim_mode[pin] = mode; }
void digitalWrite(uint8_t pin, uint8_t val) { advance(C.dwrite); setLevel(pin, val); }
int digitalRead(uint8_t pin) {
    advance(C.dread);
    if (pin == P_DRDY) return adc.drdyLow ? 0 : 1;
    if (sim_mode[pin] == INPUT_PULLUP) return 1;
    return sim_level[pin];
}
void attachInterrupt(uint8_t, void (*fn)(void), int) { sim_isr = fn; }
static uint32_t sim_ledcFreq = 0;
bool ledcAttach(uint8_t pin, uint32_t f, uint8_t) {
    if (pin == P_CLKIN) { sim_ledcFreq = f; adc.clkin = f; adc.restartConv(); }
    return true;
}
bool ledcWrite(uint8_t, uint32_t) { return true; }
bool ledcDetach(uint8_t pin) {
    if (pin == P_CLKIN) { adc.clkin = 0; adc.restartConv(); }
    return true;
}
uint32_t ledcReadFreq(uint8_t) { return sim_ledcFreq; }
uint32_t esp_random() { return sim_rng(); }

uint32_t mock_reg_read(uint32_t a) {
    advance(C.reg);
    if (a == GPIO_IN_REG) {
        uint32_t v = 0;
        for (int p = 0; p < 32; p++) if (sim_level[p]) v |= 1u << p;
        v = adc.drdyLow ? (v & ~(1u << P_DRDY)) : (v | (1u << P_DRDY));
        return v;
    }
    return 0;
}
void mock_reg_write(uint32_t a, uint32_t v) {
    advance(C.reg);
    int base = (a == GPIO_OUT1_W1TS_REG || a == GPIO_OUT1_W1TC_REG) ? 32 : 0;
    bool set = (a == GPIO_OUT_W1TS_REG || a == GPIO_OUT1_W1TS_REG);
    for (int b = 0; b < 32; b++)
        if (v >> b & 1 && base + b < 40) setLevel((uint8_t)(base + b), set ? 1 : 0);
}

// ---------------------------------------------------------------- SPI
static uint32_t spiActualHz(uint32_t f) {
    uint32_t n = (80000000u + f - 1) / f;
    return 80000000u / (n ? n : 1);
}
void SPIClass::beginTransaction(const SPISettings &s) { adc.spiHz = spiActualHz(s.hz); }
uint8_t SPIClass::transfer(uint8_t b) {
    uint8_t r = adc.xfer(b);
    advance(8e6 / adc.spiHz + C.byte_xfer_ovh);
    return r;
}
void spiTransferBytesNL(spi_t *, const void *out, uint8_t *in, uint32_t n) {
    const uint8_t *o = (const uint8_t *)out;
    for (uint32_t i = 0; i < n; i++) {
        uint8_t r = adc.xfer(o ? o[i] : 0xFF);
        if (in) in[i] = r;
    }
    advance(C.spi_ovh + n * 8e6 / adc.spiHz);
}
void SPIClass::transferBytes(const uint8_t *out, uint8_t *in, uint32_t n) {
    spiTransferBytesNL(&bus_, out, in, n);
}
uint32_t spiGetClockDiv(spi_t *) { return adc.spiHz; }
uint32_t spiClockDivToFrequency(spi_t *, uint32_t f) { return f; }

// ---------------------------------------------------------------- UART
HardwareSerial Serial;
static std::vector<std::pair<double, std::string>> sim_rx;   // (time, bytes)
static size_t sim_rxItem = 0, sim_rxPos = 0;
static std::vector<uint8_t> sim_tx;
static double sim_txPending = 0, sim_txLast = 0;
static size_t sim_txCap = 128;
static const double BAUD_BPS = 92160.0;                     // 921600 baud, 10 bits/byte

size_t HardwareSerial::setTxBufferSize(size_t n) { sim_txCap = n + 128; return n; }
static void txDrain() {
    sim_txPending = std::max(0.0, sim_txPending - (sim_t - sim_txLast) * BAUD_BPS / 1e6);
    sim_txLast = sim_t;
}
int HardwareSerial::available() {
    advance(0.1);
    int n = 0;
    for (size_t i = sim_rxItem; i < sim_rx.size() && sim_rx[i].first <= sim_t; i++)
        n += (int)sim_rx[i].second.size() - (i == sim_rxItem ? (int)sim_rxPos : 0);
    return n;
}
int HardwareSerial::read() {
    advance(0.1);
    while (sim_rxItem < sim_rx.size() && sim_rx[sim_rxItem].first <= sim_t) {
        if (sim_rxPos < sim_rx[sim_rxItem].second.size()) return (uint8_t)sim_rx[sim_rxItem].second[sim_rxPos++];
        sim_rxItem++; sim_rxPos = 0;
    }
    return -1;
}
int HardwareSerial::availableForWrite() {
    advance(C.avail);
    txDrain();
    return (int)(sim_txCap - sim_txPending);
}
size_t HardwareSerial::write(const uint8_t *d, size_t n) {
    advance(C.ser_call + n * C.ser_byte);
    txDrain();
    while (sim_txCap - sim_txPending < n) { advance(20); txDrain(); }   // blocks like the driver
    sim_tx.insert(sim_tx.end(), d, d + n);
    sim_txPending += n;
    return n;
}
size_t HardwareSerial::printf(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return 0;
    return write((const uint8_t *)buf, std::min(n, (int)sizeof(buf) - 1));
}
void HardwareSerial::flush() { while (sim_txPending > 0) { advance(20); txDrain(); } }

// ---------------------------------------------------------------- the sketch
static double sim_traceFrom = -1, sim_traceTo = -1;          // --trace T0:T1 (s)
#define LK_TRACE(t, edge, pend, k, lastK, back, over, wait)                                  \
    do {                                                                                     \
        if (sim_t >= sim_traceFrom && sim_t <= sim_traceTo)                                  \
            fprintf(stderr, "trace %.1f t=%u edge=%d pend=%d k=%d lastK=%d back=%d over=%d"  \
                    " wait=%d fifo=%zu drdy=%d\n", sim_t, (unsigned)(t), (int)(edge),       \
                    (int)(pend), (int)(k), (int)(lastK), (int)(back), (int)(over),           \
                    (int)(wait), adc.fifo.size(), (int)adc.drdyLow);                         \
    } while (0)
#include SKETCH

// ---------------------------------------------------------------- main
int main(int argc, char **argv) {
    double endS = 3;
    std::string outPath = "sim_out.bin";
    std::vector<double> resets;
    double scale = 1;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
        if (a == "--end") endS = atof(next().c_str());
        else if (a == "--out") outPath = next();
        else if (a == "--seed") sim_rng.seed(atoi(next().c_str()));
        else if (a == "--cost-scale") scale = atof(next().c_str());
        else if (a == "--rail") sim_railedOuts = (uint32_t)strtoul(next().c_str(), nullptr, 0);
        else if (a == "--cmd") {
            std::string v = next();
            size_t c = v.find(':');
            sim_rx.push_back({atof(v.substr(0, c).c_str()) * 1e6, v.substr(c + 1)});
        } else if (a == "--stall") {
            std::string v = next();
            size_t c = v.find(':');
            sim_stalls.push_back({atof(v.substr(0, c).c_str()) * 1e6, atof(v.substr(c + 1).c_str())});
        } else if (a == "--adc-reset") resets.push_back(atof(next().c_str()) * 1e6);
        else if (a == "--no-pulse") sim_pulse = false;
        else if (a == "--trace") {
            std::string v = next();
            size_t c = v.find(':');
            sim_traceFrom = atof(v.substr(0, c).c_str()) * 1e6;
            sim_traceTo = atof(v.substr(c + 1).c_str()) * 1e6;
        }
        else { fprintf(stderr, "unknown arg %s\n", a.c_str()); return 2; }
    }
    if (scale != 1) {
        double *p = &C.micros;
        for (size_t k = 0; k < sizeof(C) / sizeof(double); k++) p[k] *= scale;
    }
    std::stable_sort(sim_rx.begin(), sim_rx.end(),
                     [](const auto &x, const auto &y) { return x.first < y.first; });
    sim_level[P_CS] = 1;
    adc.reset();
    setup();
    while (sim_t < endS * 1e6) {
        loop();
        advance(C.loop);
        for (auto &r : resets)
            if (r > 0 && sim_t >= r) { adc.reset(); r = 0; }
    }
    FILE *f = fopen(outPath.c_str(), "wb");
    if (!f) { perror(outPath.c_str()); return 1; }
    fwrite(sim_tx.data(), 1, sim_tx.size(), f);
    fclose(f);
    fprintf(stderr, "sim: %.3f s virtual, %llu conversions, %llu blocked DRDY pulses, %llu FIFO"
            " overwrites, %llu narrow pulses, %zu bytes out\n", sim_t / 1e6, (unsigned long long)adc.nConv,
            (unsigned long long)adc.nBlocked, (unsigned long long)adc.nOverwrite,
            (unsigned long long)adc.nPulse, sim_tx.size());
    return 0;
}
