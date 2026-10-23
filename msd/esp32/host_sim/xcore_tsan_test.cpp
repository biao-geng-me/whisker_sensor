// ThreadSanitizer test of lk_xcore.h (the firmware's cross-core queue and
// park/run handshake) with two real threads and random delays.
//   g++ -std=gnu++17 -O1 -g -fsanitize=thread -pthread -I../whisker_daq xcore_tsan_test.cpp
// Exit 0 = all checks passed; TSan prints "WARNING: ThreadSanitizer" on a race
// (test_host_sim.py fails on either).
#include "lk_xcore.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <random>
#include <thread>

static int g_fail = 0;
#define CHECK(c, ...) do { if (!(c)) { fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); g_fail++; } } while (0)

static void spin(std::mt19937 &rng, int maxIter) {
    volatile int x = 0;
    int n = (int)(rng() % (unsigned)(maxIter + 1));
    for (int i = 0; i < n; i++) x = x + 1;
}

// 1. Queue: order, contents, and drops exactly when full.
struct Rec {
    uint32_t seq;
    uint32_t body[30];
    uint32_t check;
};
static void testQueue() {
    static XQueue<Rec, 8> q;
    const uint32_t N = 400000;
    uint32_t drops = 0;
    std::atomic<bool> done{false};
    std::thread prod([&] {
        std::mt19937 rng(1);
        for (uint32_t s = 0; s < N; s++) {
            Rec r;
            r.seq = s;
            for (int i = 0; i < 30; i++) r.body[i] = s * 31 + i;
            r.check = s ^ 0xA5A5A5A5u;
            if (!q.push(r)) drops++;
            spin(rng, 200);
        }
        done.store(true, std::memory_order_release);
    });
    uint32_t got = 0, lost = 0;
    int64_t last = -1;
    std::mt19937 rng(2);
    for (;;) {
        Rec r;
        if (q.pop(r)) {
            bool ok = r.check == (r.seq ^ 0xA5A5A5A5u);
            for (int i = 0; i < 30 && ok; i++) ok = r.body[i] == r.seq * 31 + i;
            CHECK(ok, "torn record %u", r.seq);
            CHECK((int64_t)r.seq > last, "out of order %u after %lld", r.seq, (long long)last);
            lost += (uint32_t)(r.seq - last - 1);
            last = r.seq;
            got++;
            spin(rng, 250);
        } else if (done.load(std::memory_order_acquire) && !q.ready()) {
            break;
        }
    }
    prod.join();
    lost += N - 1 - (uint32_t)last;
    CHECK(got + drops == N, "got %u + drops %u != %u", got, drops, N);
    CHECK(lost == drops, "gaps %u != drops %u", lost, drops);
    CHECK(q.hwm <= 8, "hwm %u", q.hwm);
    printf("queue: %u records, %u delivered, %u dropped when full, hwm %u\n", N, got, drops, q.hwm);
}

// 2. Handshake: a plain struct (the ADC side) is touched only by its owner.
// Any access outside the ownership the handshake grants is a data race TSan
// reports; the counters check that both sides really took turns.
struct Hw {
    uint64_t a = 0, b = 0;          // invariant: a == b whenever the owner lets go
    uint32_t hostTurns = 0, readerSteps = 0;
};
static void testHandshake() {
    static XCtl ctl;
    static Hw hw;
    const int ROUNDS = 20000;
    std::atomic<bool> stop{false};
    std::thread reader([&] {
        std::mt19937 rng(3);
        while (!stop.load(std::memory_order_relaxed)) {
            XcView v = xcPoll(ctl);
            if (v.parking || v.ack) {
                if (v.parking) CHECK(hw.a == hw.b, "reader let go mid-update");
                xcParkDone(ctl, v);
                continue;
            }
            if (!v.run) { std::this_thread::yield(); continue; }
            if (v.starting) xcStarted(ctl, v);
            hw.a++;                  // owner-only access
            spin(rng, 50);
            hw.b++;
            hw.readerSteps++;
        }
    });
    std::mt19937 rng(4);
    for (int i = 0; i < ROUNDS; i++) {
        xcRequest(ctl, XC_PARK);
        while (!xcParked(ctl)) std::this_thread::yield();
        CHECK(hw.a == hw.b, "host took over mid-update");
        hw.a += 2; hw.b += 2;        // the host owns it now
        hw.hostTurns++;
        xcRequest(ctl, XC_RUN);
        spin(rng, 400);
    }
    xcRequest(ctl, XC_PARK);
    while (!xcParked(ctl)) std::this_thread::yield();
    stop.store(true);
    reader.join();
    CHECK(hw.hostTurns == (uint32_t)ROUNDS, "host turns %u", hw.hostTurns);
    CHECK(hw.readerSteps > 0, "the reader never ran");
    printf("handshake: %d host turns, %u reader steps\n", ROUNDS, hw.readerSteps);
}

int main() {
    testQueue();
    testHandshake();
    printf(g_fail ? "FAILED (%d)\n" : "OK\n", g_fail);
    return g_fail ? 1 : 0;
}
