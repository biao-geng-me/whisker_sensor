"""ADC fault alarm end to end: fake ESP32 (SIGUSR2 = ADC lock-up / PCB power
cycle) -> daq_server -> status ``esp32.fault``, ``mark`` events to the client
and into the recording, and the server log.

Run: python -m pytest rpi/tests/test_fault_alarm.py
"""

from __future__ import annotations

import json
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from whisker_link.wire import LineSplitter  # noqa: E402

FAKE = ROOT / "rpi" / "sim" / "fake_esp32.py"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Rig:
    def __init__(self, tmp_path, mode: str, fake_args=()):
        self.fake = subprocess.Popen([sys.executable, "-u", str(FAKE), *fake_args],
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        pty = self.fake.stdout.readline().decode().strip()
        assert pty.startswith("/dev/"), pty
        self.port = _free_port()
        self.rec_dir = tmp_path
        self.srv = subprocess.Popen([sys.executable, "-u", str(ROOT / "rpi" / "daq_server.py"),
                                     "--serial", pty, "--mavlink", "none", "--mode", mode,
                                     "--bind", f"127.0.0.1:{self.port}", "--rec-dir",
                                     str(tmp_path), "--record", "--log-every", "1"],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        for _ in range(50):
            try:
                self.c = socket.create_connection(("127.0.0.1", self.port), timeout=1)
                break
            except OSError:
                time.sleep(0.1)
        else:
            pytest.fail("daq_server did not start")
        self.c.settimeout(0.2)
        self.sp = LineSplitter()
        self.events: list[dict] = []

    def read(self, secs: float) -> list[dict]:
        new, t_end = [], time.monotonic() + secs
        while time.monotonic() < t_end:
            try:
                data = self.c.recv(65536)
            except socket.timeout:
                continue
            new += [ev for ev in self.sp.feed(data) if ev.get("k") in ("status", "mark")]
        self.events += new
        return new

    def wait_for(self, pred, secs: float) -> dict:
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            for ev in self.read(0.2):
                if pred(ev):
                    return ev
        pytest.fail(f"timed out; last events {self.events[-3:]}")

    def usr2(self) -> None:
        self.fake.send_signal(signal.SIGUSR2)

    def close(self) -> str:
        self.c.close()
        self.srv.send_signal(signal.SIGINT)
        try:
            out = self.srv.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            self.srv.kill()
            out = self.srv.communicate()[0]
        self.fake.kill()
        self.fake.wait()
        return out.decode(errors="replace")


def _fault(ev):
    return ev.get("esp32", {}).get("fault", {}) if ev["k"] == "status" else {}


def test_legacy_unrecovered_latches_and_clears(tmp_path):
    rig = Rig(tmp_path, "legacy")
    try:
        st = rig.wait_for(lambda e: e["k"] == "status" and e["esp32"]["streaming"], 8)
        assert _fault(st)["level"] == "ok"
        rig.usr2()                                       # the ADC locks up
        t0 = time.monotonic()
        m = rig.wait_for(lambda e: e["k"] == "mark" and e.get("fault", {}).get("event") == "fault", 5)
        onset_s = time.monotonic() - t0
        assert m["fault"]["code"] == "adc_unrecovered" and "UNRECOVERED" in m["fault"]["detail"]
        assert m["text"].endswith("-> power-cycle the whisker PCB (vehicle power)")
        st = rig.wait_for(lambda e: _fault(e).get("level") == "fault", 3)
        assert _fault(st)["code"] == "adc_unrecovered" and _fault(st)["count"] == 1
        rig.read(6)                                      # B kicks + INFOs: still latched
        assert _fault([e for e in rig.events if e["k"] == "status"][-1])["level"] == "fault"
        rig.usr2()                                       # PCB power-cycled
        t1 = time.monotonic()
        clr = rig.wait_for(lambda e: e["k"] == "mark" and e.get("fault", {}).get("event") == "clear",
                           16)
        clear_s = time.monotonic() - t1
        st = rig.wait_for(lambda e: e["k"] == "status", 2)
        assert _fault(st)["level"] == "warn"             # the recovered halt
        assert [h["event"] for h in _fault(st)["history"]][-3:] == ["fault", "warn", "clear"]
    finally:
        log = rig.close()
    assert 9.5 < clear_s < 13 and onset_s < 2.5, (onset_s, clear_s)
    assert "ESP32 FAULT adc_unrecovered" in log and "ESP32 fault adc_unrecovered cleared" in log
    (rec,) = list(tmp_path.glob("whisker_*.jsonl"))
    marks = [json.loads(x) for x in rec.read_text().splitlines() if '"k": "mark"' in x
             or '"k":"mark"' in x]
    assert [m["fault"]["event"] for m in marks] == ["fault", "warn", "clear"], marks
    print(f"onset {onset_s:.1f} s, clear {clear_s:.1f} s; marks: {[m['text'] for m in marks]}")


def test_quiet_mode_lockup_and_dead_boot(tmp_path):
    rig = Rig(tmp_path, "quiet")
    try:
        rig.wait_for(lambda e: e["k"] == "status" and e["esp32"]["streaming"], 8)
        rig.usr2()
        t0 = time.monotonic()
        m = rig.wait_for(lambda e: e["k"] == "mark" and e.get("fault", {}).get("event") == "fault", 6)
        onset_s = time.monotonic() - t0
        assert m["fault"]["code"] == "adc_not_converting", m
        # reboot with the ADC still dead: CLOCK not accepted is the better diagnosis
        rig.fake.send_signal(signal.SIGUSR1)
        m = rig.wait_for(lambda e: e["k"] == "mark" and e.get("fault", {}).get("event") == "fault", 6)
        assert m["fault"]["code"] == "clock_rejected"
        st = rig.wait_for(lambda e: e["k"] == "status", 2)
        assert _fault(st)["count"] == 1                  # one fault, refined
    finally:
        rig.close()
    assert onset_s < 3.0, onset_s


def test_record_start_during_a_fault(tmp_path):
    rig = Rig(tmp_path, "legacy", ["--adc-dead"])
    try:
        rig.wait_for(lambda e: _fault(e).get("level") == "fault", 10)
        rig.c.sendall(b'{"cmd": "rec_start", "tag": "late"}\n')
        rig.read(1.5)
    finally:
        rig.close()
    (rec,) = list(tmp_path.glob("whisker_*_late.jsonl"))
    first = [json.loads(x) for x in rec.read_text().splitlines()[:6]]
    marks = [e for e in first if e["k"] == "mark"]
    assert marks and marks[0]["fault"]["event"] == "state", first
    assert marks[0]["text"].startswith("at record start: ESP32 fault")
