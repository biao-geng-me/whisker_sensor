"""Scan modes (firmware M0/M1/M2) end to end with the fake ESP32: INFO version 4,
HEALTH frames, 250 Hz framing, and daq_server's --mode / --selects pass-through.

Run: python -m pytest rpi/tests/test_scan_modes.py
"""

from __future__ import annotations

import json
import os
import select
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from whisker_link import esp32_frame as ef  # noqa: E402
from whisker_link.channels import OUT_OF, channels_of_selects  # noqa: E402

FAKE = ROOT / "rpi" / "sim" / "fake_esp32.py"


@pytest.fixture
def fake_esp32(tmp_path):
    p = subprocess.Popen([sys.executable, "-u", str(FAKE), "--live", "W1_A,W2_B,W3_B"],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    path = p.stdout.readline().decode().strip()
    assert path.startswith("/dev/"), path
    yield path
    p.kill()
    p.wait()


def _read_events(fd: int, secs: float) -> list:
    dec, evs, t_end = ef.Decoder(), [], time.monotonic() + secs
    while time.monotonic() < t_end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            evs += list(dec.feed(os.read(fd, 65536)))
    return evs


def test_wide_mode_framing(fake_esp32):
    fd = os.open(fake_esp32, os.O_RDWR | os.O_NOCTTY)
    try:
        os.write(fd, b"S0123457;M2;B")
        t0 = time.monotonic()
        evs = _read_events(fd, 3.0)
        wall = time.monotonic() - t0
    finally:
        os.close(fd)
    infos = [e for e in evs if isinstance(e, ef.Info)]
    scans = [e for e in evs if isinstance(e, ef.Scan)]
    health = [e for e in evs if isinstance(e, ef.Health)]
    cfg = ef.info_cfg(dict(infos[-1].__dict__))
    assert cfg["mode_name"] == "wide" and cfg["n_conv"] == 125 and cfg["scan_hz"] == 250.0
    assert cfg["selects"] == "0123457" and cfg["avg_per_sel"][6] == 0
    assert sum(cfg["avg_per_sel"]) + 7 * cfg["discard_samples"] == 125
    # 250 Hz framing: t_us exactly 4000 apart, seq contiguous, ~250 frames/s on the wire
    assert len(scans) > 0.7 * 250 * wall
    assert {b.t_us - a.t_us for a, b in zip(scans, scans[1:])} == {4000}
    assert all(b.seq == a.seq + 1 for a, b in zip(scans, scans[1:]))
    assert all(s.dur_us == 4000 for s in scans)
    # select 6 is skipped: its four outputs read railed
    skipped = {OUT_OF[a][6] - 1 for a in range(4)}
    assert skipped.isdisjoint(channels_of_selects(0b10111111))
    for s in scans:
        assert all(s.railmask >> i & 1 and s.raw[i] == (1 << 23) - 1 for i in skipped)
    assert health and health[-1].mode == 2 and health[-1].window_ms == 1000
    assert health[-1].summary()["busy_pct"] == 38.0


def test_boot_default_is_quiet(fake_esp32):
    fd = os.open(fake_esp32, os.O_RDWR | os.O_NOCTTY)
    try:
        os.write(fd, b"B")
        evs = _read_events(fd, 1.0)
    finally:
        os.close(fd)
    cfg = ef.info_cfg(dict([e for e in evs if isinstance(e, ef.Info)][0].__dict__))
    assert cfg["mode_name"] == "quiet" and cfg["osr"] == 512 and cfg["scan_hz"] == 62.5
    assert {e.dur_us for e in evs if isinstance(e, ef.Scan)} == {16000}


def test_back_to_legacy(fake_esp32):
    fd = os.open(fake_esp32, os.O_RDWR | os.O_NOCTTY)
    try:
        os.write(fd, b"M1;B")
        evs = _read_events(fd, 1.2)
        os.write(fd, b"M0;")
        evs += _read_events(fd, 1.0)
    finally:
        os.close(fd)
    modes = [e.mode for e in evs if isinstance(e, ef.Info)]
    assert modes[0] == 1 and modes[-1] == 0
    quiet = [e for e in evs if isinstance(e, ef.Scan) and e.dur_us == 16000]
    assert len(quiet) >= 40 and {b.t_us - a.t_us for a, b in zip(quiet, quiet[1:])} == {16000}


def test_quiet_defaults_and_125hz(fake_esp32):
    fd = os.open(fake_esp32, os.O_RDWR | os.O_NOCTTY)
    try:
        os.write(fd, b"M1;B")
        evs = _read_events(fd, 0.6)
        os.write(fd, b"M1 200 512 8000;")       # N = 8000 / 128 us is not whole: refused
        evs += _read_events(fd, 0.4)
        os.write(fd, b"M1 200 256 8000;")
        evs += _read_events(fd, 1.2)
    finally:
        os.close(fd)
    infos = [ef.info_cfg(dict(e.__dict__)) for e in evs if isinstance(e, ef.Info)]
    assert infos[0]["osr"] == 512 and infos[0]["n_conv"] == 125 and infos[0]["scan_hz"] == 62.5
    assert any("whole number of conversions" in e.text for e in evs if isinstance(e, ef.Text))
    assert infos[-1]["osr"] == 256 and infos[-1]["n_conv"] == 125 and infos[-1]["scan_hz"] == 125.0
    fast = [e for e in evs if isinstance(e, ef.Scan) and e.dur_us == 8000]
    assert len(fast) >= 60 and {b.t_us - a.t_us for a, b in zip(fast, fast[1:])} == {8000}


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_daq_server_mode_passthrough(fake_esp32, tmp_path):
    from whisker_link.wire import LineSplitter

    port = _free_port()
    srv = subprocess.Popen([sys.executable, "-u", str(ROOT / "rpi" / "daq_server.py"),
                            "--serial", fake_esp32, "--mavlink", "none",
                            "--bind", f"127.0.0.1:{port}", "--rec-dir", str(tmp_path),
                            "--mode", "wide", "--selects", "7012345"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        for _ in range(50):
            try:
                c = socket.create_connection(("127.0.0.1", port), timeout=1)
                break
            except OSError:
                time.sleep(0.1)
        else:
            pytest.fail("daq_server did not start")
        sp, statuses, t_end = LineSplitter(), [], time.monotonic() + 6
        c.settimeout(0.5)
        while time.monotonic() < t_end:
            try:
                data = c.recv(65536)
            except socket.timeout:
                continue
            statuses += [ev for ev in sp.feed(data) if ev.get("k") == "status"]
        c.close()
    finally:
        srv.kill()
        out = srv.communicate()[0].decode(errors="replace")
    e = statuses[-1]["esp32"]
    assert e["cfg"]["mode_name"] == "wide" and e["cfg"]["selects"] == "0123457", out[-2000:]
    assert e["health"]["mode"] == 2 and "busy_pct" in e["health"]
    assert statuses[-1]["hz"]["wsk"] > 200
    assert e["adc_sync"] == 0 and e["seq_gaps"] == 0
    assert json.dumps(statuses[-1])                   # serialisable for the jsonl log
