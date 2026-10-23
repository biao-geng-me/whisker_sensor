"""FaultMonitor: synthetic firmware sequences and the bench captures.

Run: python -m pytest whisker_link/tests/test_esp32_fault.py
"""

from __future__ import annotations

import glob
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from whisker_link import esp32_frame as ef  # noqa: E402
from whisker_link.esp32_fault import FaultMonitor, classify, mark_event  # noqa: E402

UNRECOVERED = ("[halt #11] UNRECOVERED  ID=0x2405 CLOCKrb=0x00FF crcbad=0"
               "  -> power-cycle the PCB with the ESP32 already running")
LK_HALT = "[halt] no conversion for 50 ms in quiet mode: CLKIN restart + re-init"
BENCH = "/scratch/bigeme/whisker_dev/bench_1023"


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make():
    clk = Clock()
    return clk, FaultMonitor(clk, wall=lambda: 1.7e9 + clk.t)


def stream(clk, mon, secs, hz=62.5, flags=0):
    for _ in range(int(secs * hz)):
        clk.t += 1 / hz
        mon.on_scan(flags)
        mon.poll()


def idle(clk, mon, secs, step=0.1):
    for _ in range(int(round(secs / step))):
        clk.t += step
        mon.poll()


def test_classify():
    assert classify(UNRECOVERED)[:2] == ("fault", "adc_unrecovered")
    assert classify("CLOCK readback = 0x00FF  (NOT accepted)")[:2] == ("fault", "clock_rejected")
    assert classify("CLOCK readback = 0x0F02  (config ACCEPTED)") is None
    assert classify("quiet mode CLOCK not accepted (0x00FF): legacy scan")[1] == "clock_rejected"
    assert classify("M: CLOCK config not accepted (readback 0x00FF); back to legacy")[1] \
        == "clock_rejected"
    assert classify("#BURSTERR CLOCK config not accepted") is None
    assert classify("[adc reset #2] ADC had reset (STATUS RESET=1), re-initialised: CLOCK=0x00FF"
                    " NOT accepted MODE=0x0110 ID=0x2405")[1] == "clock_rejected"
    assert classify("[adc reset #2] in wide mode: re-initialising")[:2] == ("warn", "adc_reset")
    assert classify("Device ID      = 0xFFFF  (NO RESPONSE)")[1] == "adc_no_response"
    assert classify("DRDY edges/200ms = 0  (NOT toggling)")[1] == "adc_not_converting"
    assert classify("DRDY edges/200ms = 1246  (converting)") is None
    assert classify("[halt #3] recovered at L2  (L1=1 L2=1 L3=0 fail=0 crcbad=0 stale=0)") \
        == ("warn", "adc_halt", "ADC halt, recovered at L2")
    assert classify(LK_HALT) == ("warn", "adc_halt", "ADC halt in quiet mode, re-initialised")
    assert classify("[reader watchdog] no progress for 3000 ms: restarting")[1] == "reader_watchdog"
    assert classify("E (422977) task_wdt: esp_task_wdt_reset(707): task not found")[1] \
        == "idf_error"
    assert classify("[idf] E (292442) task_wdt: task not found")[1] == "idf_error"
    assert classify("[idf] I (12) boot: fine") is None
    assert classify("===== scan 17 | 5 live, 27 open | halts 0 crc 0 stale 0 =====") is None


def test_unrecovered_latches_until_clean_streaming():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 3)
    assert mon.status()["level"] == "ok" and not mon.take_events()
    mon.on_text(UNRECOVERED)
    (ev,) = mon.take_events()
    assert ev["event"] == "fault" and ev["code"] == "adc_unrecovered"
    assert "power-cycle the whisker PCB (vehicle power)" in ev["action"]
    st = mon.status()
    assert st["level"] == "fault" and st["count"] == 1 and st["detail"] == UNRECOVERED
    # the firmware keeps trying; INFOs (B kicks) and silence don't change or clear it
    for _ in range(5):
        idle(clk, mon, 1.5)
        mon.on_text(UNRECOVERED.replace("#11", "#12"))
    assert not mon.take_events()
    st = mon.status()
    assert st["code"] == "adc_unrecovered" and st["since"] == ev["unix"]
    # PCB power-cycled with the ESP32 running: recovered, scans again
    mon.on_text("[halt #19] recovered at L3  (L1=0 L2=0 L3=1 fail=8 crcbad=0 stale=0)")
    stream(clk, mon, 9)
    assert mon.status()["level"] == "fault"
    (w,) = mon.take_events()
    assert w["event"] == "warn" and w["code"] == "adc_halt"
    stream(clk, mon, 1.5)
    (c,) = mon.take_events()
    assert c["event"] == "clear" and c["code"] == "adc_unrecovered"
    st = mon.status()
    assert st["level"] == "warn" and st["count"] == 1      # the recovered halt, for a minute
    assert [h["event"] for h in st["history"]] == ["fault", "warn", "clear"]
    stream(clk, mon, 61)
    assert mon.status()["level"] == "ok" and mon.status()["history"]


def test_a_flagged_scan_restarts_the_clean_clock():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 1)
    mon.on_text(UNRECOVERED)
    stream(clk, mon, 8)
    mon.on_scan(ef.FLAG_RECOVERED)
    stream(clk, mon, 8)
    assert mon.status()["level"] == "fault"
    stream(clk, mon, 3)
    assert mon.status()["level"] != "fault"


def test_clock_rejected_at_boot():
    clk, mon = make()
    mon.link(True)
    for line in ("=================================================",
                 "  whisker_daq — 16 whiskers x 2 gauges = 32 ch",
                 "Device ID      = 0x2405  (ADS131M04)",
                 "CLOCK readback = 0x00FF  (NOT accepted)",
                 "DRDY edges/200ms = 0  (NOT toggling)",
                 "quiet mode CLOCK not accepted (0x00FF): legacy scan"):
        mon.on_text(line)
    evs = mon.take_events()
    assert [e["code"] for e in evs] == ["clock_rejected"]      # one onset, not three
    idle(clk, mon, 6)
    assert mon.status()["code"] == "clock_rejected" and not mon.take_events()


def test_locked_halt_storm_is_a_fault_quickly():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 2)
    mon.on_text(LK_HALT)
    stream(clk, mon, 1)                                     # one halt that recovers: a warning
    assert mon.status()["level"] == "warn"
    assert [e["event"] for e in mon.take_events()] == ["warn"]
    for _ in range(20):                                     # the ADC stops for good
        clk.t += 0.1
        mon.on_text(LK_HALT)
        mon.poll()
    evs = mon.take_events()
    assert [e["code"] for e in evs if e["event"] == "fault"] == ["adc_not_converting"]
    assert mon.status()["level"] == "fault"
    idle(clk, mon, 6)                                       # no_scans is vaguer: no new event
    assert not [e for e in mon.take_events() if e["event"] == "fault"]


def test_no_scans_and_short_gaps():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 2)
    idle(clk, mon, 3)                                       # a 3 s dropout: nothing
    stream(clk, mon, 2)
    assert mon.status()["level"] == "ok"
    idle(clk, mon, 5.5)
    (ev,) = mon.take_events()
    assert ev["code"] == "no_scans" and "USB" in ev["action"]
    stream(clk, mon, 11)
    assert mon.take_events()[0]["event"] == "clear"
    # right after connecting, 5 s of nothing is a fault too
    clk2, mon2 = make()
    mon2.link(True)
    idle(clk2, mon2, 5.5)
    assert mon2.status()["code"] == "no_scans"


def test_link_down():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 2)
    mon.link(False)
    idle(clk, mon, 4)
    assert mon.status()["level"] == "ok"
    idle(clk, mon, 2)
    assert mon.status()["code"] == "link_down"
    mon.link(True)
    stream(clk, mon, 11)
    assert mon.status()["level"] != "fault"


def test_reboots_and_watchdog():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 2)
    mon.on_text("[reader watchdog] no progress for 3000 ms: restarting")
    idle(clk, mon, 1)
    mon.on_text("  whisker_daq — 16 whiskers x 2 gauges = 32 ch")
    mon.on_text("reset reason 3")
    mon.on_text("previous reset: [reader watchdog] no progress for 3000 ms: ...")
    idle(clk, mon, 4)                                       # 5 s since the last scan, but booting
    stream(clk, mon, 2)
    evs = mon.take_events()
    assert [e["code"] for e in evs] == ["reader_watchdog"]
    assert mon.status()["warns"] == {"reader_watchdog": 1}
    # an unexpected reboot (brownout), reported once with its reason
    stream(clk, mon, 14)
    mon.on_text("  whisker_daq — 16 whiskers x 2 gauges = 32 ch")
    mon.on_text("reset reason 9")
    (ev,) = mon.take_events()
    assert ev["code"] == "esp32_reboot" and ev["msg"] == "ESP32 rebooted (brownout)"
    # older firmware, no reason line: reported after 3 s
    stream(clk, mon, 11)
    mon.on_text("  whisker_daq (SIMULATED) rebooted")
    idle(clk, mon, 3.5)
    (ev,) = mon.take_events()
    assert ev["msg"] == "ESP32 rebooted" and mon.status()["warns"]["esp32_reboot"] == 2
    # our own EN pulse: no warning
    stream(clk, mon, 12)
    mon.expect_reset()
    mon.on_text("  whisker_daq — 16 whiskers x 2 gauges = 32 ch")
    assert not mon.take_events()


def test_flag_rate_and_rate_limit():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 10)
    for _ in range(3):
        stream(clk, mon, 0.5, flags=ef.FLAG_SYNC)
        stream(clk, mon, 3)
    evs = mon.take_events()
    assert [e["code"] for e in evs] == ["scan_flags"]       # once per 10 s
    assert mon.status()["level"] == "warn"
    for _ in range(30):
        mon.on_text("[idf] E (292442) task_wdt: esp_task_wdt_reset(707): task not found")
    clk.t += 11
    mon.on_text("[idf] E (292443) task_wdt: esp_task_wdt_reset(707): task not found")
    evs = [e for e in mon.take_events() if e["code"] == "idf_error"]
    assert len(evs) == 2 and "+29 more" in evs[1]["msg"]


def test_mark_event():
    clk, mon = make()
    mon.link(True)
    mon.on_text(UNRECOVERED)
    (ev,) = mon.take_events()
    m = mark_event(ev, 12.5)
    assert m["k"] == "mark" and m["t"] == 12.5 and m["fault"]["code"] == "adc_unrecovered"
    assert m["text"].startswith("ESP32 FAULT adc_unrecovered: ADC not converting")
    assert m["text"].endswith("-> power-cycle the whisker PCB (vehicle power)")


# ----- bench captures (skipped where /scratch isn't mounted)

def replay(path, secs, chunk=2048):
    """Feed a raw serial capture through the monitor; time advances with the byte offset."""
    data = open(path, "rb").read()
    clk, mon = make()
    mon.link(True)
    dec = ef.Decoder()
    evs = []
    for i in range(0, len(data), chunk):
        clk.t = 1000.0 + secs * i / len(data)
        for ev in dec.feed(data[i:i + chunk]):
            if isinstance(ev, ef.Scan):
                mon.on_scan(ev.flags)
            elif isinstance(ev, ef.Text):
                mon.on_text(ev.text)
        mon.poll()
        evs += mon.take_events()
    return mon, evs


def _need(path):
    if not glob.glob(path):
        pytest.skip(f"{path} not available")


def _secs(path):
    return 600 if "10min" in path else 60 if "stress" in path else 20


def test_r4_watchdog_then_text_mode():
    """r4 (6d90c76): the reader watchdog fired on the first check after every
    start; the ESP32 rebooted into text mode and never streamed."""
    _need(f"{BENCH}/r4/*.bin")
    for path in sorted(glob.glob(f"{BENCH}/r4/quiet_c*.bin")) + [f"{BENCH}/r4/wide_c1.bin"]:
        mon, evs = replay(path, _secs(path))
        codes = [(e["event"], e["code"]) for e in evs]
        assert ("warn", "reader_watchdog") in codes, path
        assert ("fault", "no_scans") in codes, path
        assert mon.status()["level"] == "fault", path


def test_r4_stress_recovers():
    _need(f"{BENCH}/r4/stress_c1.bin")
    path = f"{BENCH}/r4/stress_c1.bin"
    mon, evs = replay(path, _secs(path))
    assert ("warn", "reader_watchdog") in [(e["event"], e["code"]) for e in evs]
    assert mon.status()["level"] != "fault"


def test_r3_raw_idf_lines():
    """r3 (old firmware): raw ESP-IDF task_wdt lines in the stream: a warning, rate-limited."""
    _need(f"{BENCH}/r3/*.bin")
    for path in sorted(glob.glob(f"{BENCH}/r3/*.bin")):
        mon, evs = replay(path, _secs(path))
        assert [e["code"] for e in evs if e["event"] != "warn"] == [], path
        idf = [e for e in evs if e["code"] == "idf_error"]
        assert 1 <= len(idf) <= 3 and mon.status()["warns"]["idf_error"] > 20, path


def test_r5_clean_runs():
    """r5 (6aeb4ef): every run streams; no fault (wide may warn about flagged scans)."""
    _need(f"{BENCH}/r5/*.bin")
    for path in sorted(glob.glob(f"{BENCH}/r5/*.bin")):
        mon, evs = replay(path, _secs(path))
        bad = [e for e in evs if e["event"] == "fault" or e["code"] not in ("scan_flags",)]
        assert not bad, (path, bad)


# ---------------------------------------------------------------- heartbeat (BEAT)
def _beat(t_ms, edges=7812, flags=ef.BEAT_BOOT_CLOCK_OK, halts=0, unrec=0, reset=1):
    return ef.Beat(1, t_ms, 1, 1, "B", flags, 1000, edges, halts, unrec, 0,
                   0x00FF if not edges else 0xFF0A, 0x2405, 0, reset, 0xFF02)


def beats(clk, mon, secs, start_ms, **kw):
    """`secs` of 1 Hz heartbeats (no scans), polling at 10 Hz."""
    for i in range(int(secs)):
        idle(clk, mon, 1.0)
        mon.on_beat(_beat(start_ms + 1000 * i, **kw))


def test_beat_dead_adc_vs_hung_esp32():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 3)
    mon.on_beat(_beat(3000))
    # the ADC dies, no text at all (lost / older line format): the beats say why
    beats(clk, mon, 3, 4000, edges=0)
    st = mon.status()
    assert st["level"] == "fault" and st["code"] == "adc_not_converting", st
    assert "ESP32 alive" in st["msg"] and "0 DRDY/s" in st["detail"] and st["beat_age_s"] < 1.1
    beats(clk, mon, 2, 7000, edges=0, flags=ef.BEAT_ADC_DOWN, unrec=1)
    assert mon.status()["code"] == "adc_unrecovered"         # the firmware gave up: refined
    assert [e["code"] for e in mon.take_events() if e["event"] == "fault"] == \
        ["adc_not_converting", "adc_unrecovered"]
    # PCB power cycle: clears after clean streaming
    stream(clk, mon, 11)
    assert mon.status()["level"] != "fault"
    # now the ESP32 itself stops: nothing at all
    mon.take_events()
    idle(clk, mon, 4.9)
    assert mon.status()["level"] != "fault"                 # no_scans waits for the verdict
    idle(clk, mon, 1.0)
    st = mon.status()
    assert st["code"] == "esp32_hung" and "reset the ESP32" in st["action"]
    assert [e["code"] for e in mon.take_events() if e["event"] == "fault"] == ["esp32_hung"]


def test_beat_keeps_alive_without_scans_and_old_firmware_is_no_scans():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 2)
    beats(clk, mon, 8, 2000)                  # alive, converting, but not streaming
    assert mon.status()["code"] == "no_scans"           # the old verdict, not hung
    clk2, old = make()                        # no beats at all: never "hung"
    old.link(True)
    stream(clk2, old, 2)
    idle(clk2, old, 8)
    assert old.status()["code"] == "no_scans"


def test_beat_uptime_going_back_is_a_reboot():
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 1)
    for i in range(3):
        mon.on_beat(_beat(50_000 + 1000 * i))
        stream(clk, mon, 1)
    mon.on_beat(_beat(1200, reset=4))         # the banner was lost; the beat tells
    ev = [e for e in mon.take_events() if e["code"] == "esp32_reboot"]
    assert len(ev) == 1 and "panic" in ev[0]["msg"]
    clk.t += 40
    mon.on_text("  whisker_daq — 16 whiskers x 2 gauges = 32 ch")   # banner seen this time
    mon.on_beat(_beat(900))
    assert not [e for e in mon.take_events() if e["code"] == "esp32_reboot" and "reason" in e["msg"]]


def test_firmware_unrecovered_text_in_locked_mode():
    """The locked modes now end a halt storm with the legacy UNRECOVERED line."""
    clk, mon = make()
    mon.link(True)
    stream(clk, mon, 3)
    for _ in range(5):
        mon.on_text(LK_HALT)
        idle(clk, mon, 0.45)
    mon.on_text("[halt #6] UNRECOVERED  ID=0x2405 CLOCKrb=0x00FF crcbad=0"
                "  -> power-cycle the PCB with the ESP32 already running")
    assert mon.status()["code"] == "adc_unrecovered"
    assert classify("[adc ok] quiet mode: converting again after 7 halt recoveries") is None
