// lk_xcore.h — what the two cores share when the locked reader runs on its own
// core (C1): a single-producer/single-consumer record queue (reader -> host)
// and the park/run handshake that hands the ADC side (SPI, CS, mux, CLKIN,
// g_lk) from one core to the other. Plain C++ with GCC __atomic builtins, no
// Arduino or FreeRTOS, so msd/esp32/host_sim/xcore_tsan_test.cpp runs the
// same code under ThreadSanitizer.
//
// Memory ordering: a producer writes a slot, then publishes it with a release
// store of `head`; the consumer reads `head` with acquire before it reads the
// slot, and frees it with a release store of `tail`. The handshake works the
// same way: everything one side wrote before its release store of req/state is
// visible to the other side after its acquire load. On the ESP32 (Xtensa)
// GCC emits `memw` for these; internal SRAM has no data cache.
#pragma once
#include <stdint.h>

template <typename T, uint32_t N>
struct XQueue {
    static_assert(N && (N & (N - 1)) == 0, "N must be a power of 2");
    alignas(32) uint32_t head = 0;   // written by the producer only (free-running)
    alignas(32) uint32_t tail = 0;   // written by the consumer only (free-running)
    uint32_t hwm = 0;                // producer: most records waiting at a push
    T slot[N];

    // Producer. false = full (the record is not queued; the caller counts it).
    bool push(const T &r) {
        uint32_t h = __atomic_load_n(&head, __ATOMIC_RELAXED);
        uint32_t t = __atomic_load_n(&tail, __ATOMIC_ACQUIRE);
        if (h - t >= N) return false;
        slot[h & (N - 1)] = r;
        __atomic_store_n(&head, h + 1, __ATOMIC_RELEASE);
        if (h + 1 - t > hwm) hwm = h + 1 - t;
        return true;
    }
    // Producer: room for one more?
    bool full() const {
        return __atomic_load_n(&head, __ATOMIC_RELAXED) -
               __atomic_load_n(&tail, __ATOMIC_ACQUIRE) >= N;
    }
    // Consumer. false = empty.
    bool pop(T &r) {
        uint32_t t = __atomic_load_n(&tail, __ATOMIC_RELAXED);
        uint32_t h = __atomic_load_n(&head, __ATOMIC_ACQUIRE);
        if (h == t) return false;
        r = slot[t & (N - 1)];
        __atomic_store_n(&tail, t + 1, __ATOMIC_RELEASE);
        return true;
    }
    // Consumer: the oldest record without taking it (nullptr = empty). Valid
    // until the consumer's next pop.
    const T *peek() const {
        uint32_t t = __atomic_load_n(&tail, __ATOMIC_RELAXED);
        uint32_t h = __atomic_load_n(&head, __ATOMIC_ACQUIRE);
        return h == t ? nullptr : &slot[t & (N - 1)];
    }
    // Consumer: does a record wait?
    bool ready() const {
        return __atomic_load_n(&head, __ATOMIC_ACQUIRE) != __atomic_load_n(&tail, __ATOMIC_RELAXED);
    }
};

// Park/run handshake. The host owns the ADC side while the reader is parked;
// the reader owns it while it runs. Only the host writes req, only the reader
// writes state. Each request carries a generation (req = gen << 1 | RUN) and
// the reader acknowledges exactly that generation (state = gen << 1 | RUNNING),
// so the host can't mistake an old "parked" for the answer to a new request.
// Without it, RUN then PARK in quick succession let the host take over while
// the reader was still acting on the RUN (found by xcore_tsan_test.cpp).
// Boot: req = state = 0 (generation 0, parked: the host owns).
enum : uint32_t { XC_PARK = 0, XC_RUN = 1 };
struct XCtl {
    uint32_t req = 0;
    uint32_t state = 0;
};

// Host side. Returns the request's generation.
static inline uint32_t xcRequest(XCtl &c, uint32_t run) {
    uint32_t g = (__atomic_load_n(&c.req, __ATOMIC_RELAXED) >> 1) + 1;   // own variable
    __atomic_store_n(&c.req, g << 1 | (run & 1), __ATOMIC_RELEASE);
    return g;
}
static inline bool xcWantRun(const XCtl &c) { return __atomic_load_n(&c.req, __ATOMIC_RELAXED) & 1; }
// Host: has the reader acknowledged the current PARK request (the host owns)?
static inline bool xcParked(const XCtl &c) {
    uint32_t r = __atomic_load_n(&c.req, __ATOMIC_RELAXED);
    return !(r & 1) && __atomic_load_n(&c.state, __ATOMIC_ACQUIRE) == r;
}

// Reader side, called between conversions.
struct XcView {
    uint32_t req;
    bool run;        // the reader should be reading (it owns the ADC side)
    bool starting;   // take over now: set up, then xcStarted
    bool parking;    // let go now: tear down, then xcParkDone
    bool ack;        // already parked, but a new PARK request wants its xcParkDone
};
static inline XcView xcPoll(const XCtl &c) {
    XcView v;
    v.req = __atomic_load_n(&c.req, __ATOMIC_ACQUIRE);
    uint32_t s = __atomic_load_n(&c.state, __ATOMIC_RELAXED);    // own variable
    v.run = v.req & 1;
    v.starting = v.run && !(s & 1);
    v.parking = !v.run && (s & 1);
    v.ack = !v.run && !(s & 1) && s != v.req;
    return v;
}
static inline void xcStarted(XCtl &c, const XcView &v) {
    __atomic_store_n(&c.state, v.req | 1, __ATOMIC_RELEASE);
}
static inline void xcParkDone(XCtl &c, const XcView &v) {
    __atomic_store_n(&c.state, v.req & ~1u, __ATOMIC_RELEASE);
}
