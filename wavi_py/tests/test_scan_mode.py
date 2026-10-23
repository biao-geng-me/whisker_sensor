"""Scan-mode request / confirmation logic and the Sim transport's firmware
emulation (headless).

Run: python -m pytest wavi_py/tests/test_scan_mode.py
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from wavi_py.scan_mode import (PRESETS, ModeKeeper, ScanRequest, info_for,  # noqa: E402
                               parse_mode, parse_selects)
from wavi_py.transports.sim_transport import SimSignals, SimTransport  # noqa: E402
from whisker_link import esp32_frame as ef  # noqa: E402
from whisker_link.channels import channels_of_selects  # noqa: E402


def cfg_of(mode, sel_mask=0b10111111, *args):
    from wavi_py.scan_mode import lock_params
    lk = lock_params(mode, *args) if mode else None
    return ef.info_cfg(info_for(mode, sel_mask, lk).__dict__)


def test_parse_mode_spelling():
    assert parse_mode("legacy") == (0,)
    assert parse_mode("quiet") == (1,)
    assert parse_mode("quiet,200,256,8000") == (1, 200, 256, 8000)
    assert parse_mode("2") == (2,)
    for bad in ("", "fast", "3", "quiet,x"):
        with pytest.raises(ValueError):
            parse_mode(bad)
    assert parse_selects("7543210") == "0123457"
    assert parse_selects("") == ""
    with pytest.raises(ValueError):
        parse_selects("8")


def test_commands_of_the_presets():
    want = {"legacy (~76 Hz)": "S0123457;M0;B",
            "quiet 62.5 Hz, OSR 512": "S0123457;M1;B",
            "quiet 62.5 Hz, OSR 256": "S0123457;M1 200 256;B",
            "quiet 125 Hz, OSR 256": "S0123457;M1 200 256 8000;B",
            "wide 250 Hz (experimental: overruns)": "S0123457;M2;B"}
    assert {label: ScanRequest(args).command() for label, args in PRESETS} == want
    assert ScanRequest((1,), "").command() == "S;M1;B"


def test_mismatch():
    q125 = ScanRequest((1, 200, 256, 8000))
    assert q125.mismatch(cfg_of(1, 0b10111111, 200, 256, 8000)) == []
    assert q125.mismatch(cfg_of(1, 0b10111111, 200, 256)) == ["scan_hz 62.5 (wanted 125)"]
    assert "mode legacy (wanted quiet)" in q125.mismatch(cfg_of(0))
    assert ScanRequest((1,)).mismatch(cfg_of(1)) == []
    assert ScanRequest((1,)).mismatch(cfg_of(1, 0xFF)) == ["sel_mask 01234567 (wanted 0123457)"]
    assert ScanRequest((1,), "").mismatch(cfg_of(1, 0xFF)) == []
    assert ScanRequest((0,)).mismatch(cfg_of(0)) == []
    assert ScanRequest((2,)).mismatch(cfg_of(2)) == []


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_keeper_confirm_reboot_and_timeouts():
    clk = Clock()
    k = ModeKeeper(clk)
    q = ScanRequest((1, 200, 256, 8000))
    assert k.set_wanted(q, connected=True) == "S0123457;M1 200 256 8000;B"
    assert k.state == "pending"
    # an INFO with the old mode right after the send is not a failure yet
    assert k.on_info(cfg_of(0)) is None and k.state == "pending"
    assert k.on_info(cfg_of(1, 0b10111111, 200, 256, 8000)) is None
    assert k.state == "ok" and "quiet 125 Hz OSR 256 confirmed" in k.note
    # reboot: back in legacy -> re-sent once, and said so
    assert k.on_info(cfg_of(0)) == q.command()
    assert "re-sending" in k.note
    k.on_info(cfg_of(1, 0b10111111, 200, 256, 8000))
    assert k.state == "ok" and "re-applied" in k.note
    # a refused mode: the INFO keeps showing legacy; after 2 s it is reported
    bad = ScanRequest((1, 200, 512, 8000))
    assert k.set_wanted(bad, connected=True) == "S0123457;M1 200 512 8000;B"
    k.on_text("M: scan period not a whole number of conversions at this OSR")
    k.on_info(cfg_of(0))
    clk.t += 2.5
    assert k.tick() is None
    assert k.state == "failed" and "NOT SET" in k.note and "M: scan period" in k.note
    # no reply at all
    k.set_wanted(q, connected=True)
    clk.t += 2.5
    k.tick()
    assert k.state == "failed" and "no INFO reply" in k.note
    # after a failure one re-send is still allowed, then it gives up
    assert k.on_info(cfg_of(0)) == q.command()
    clk.t += 2.5
    k.tick()
    assert k.on_info(cfg_of(0)) is None and k.state == "failed"
    assert "not re-sent" in k.note


def test_keeper_on_connect():
    clk = Clock()
    k = ModeKeeper(clk)
    q = ScanRequest((1,))
    assert k.set_wanted(q, connected=False) is None
    k.connected()
    assert k.on_info(cfg_of(1)) is None and k.state == "ok"     # already set: nothing sent
    k.connected()
    assert k.on_info(cfg_of(0)) == q.command()                  # legacy: set it
    k.connected()
    clk.t += 2
    assert k.tick() == q.command()                              # no INFO at all: set anyway
    assert ModeKeeper(clk).set_wanted(None, connected=True) is None


def _events(tr: SimTransport, secs: float) -> list[dict]:
    out, buf, t_end = [], b"", time.monotonic() + secs
    while time.monotonic() < t_end:
        buf += tr.read_some(65536)
        *lines, buf = buf.split(b"\n")
        out += [json.loads(x) for x in lines if x]
    return out


def _esp32(tr: SimTransport, data: str) -> None:
    tr.write((json.dumps({"cmd": "esp32", "data": data}) + "\n").encode())


def test_sim_transport_modes():
    sig = SimSignals(rate_hz=80.0)
    tr = SimTransport(sig)
    evs = _events(tr, 0.3)
    infos = [e for e in evs if e["k"] == "info"]
    assert len(infos) == 1 and infos[0]["mode"] == 0              # INFO on connect, legacy
    _esp32(tr, "S0123457;M1 200 256 8000;B")
    evs = _events(tr, 1.2)
    infos = [ef.info_cfg(e) for e in evs if e["k"] == "info"]
    assert infos and all(c["scan_hz"] == 125 and c["selects"] == "0123457" for c in infos)
    scans = [e for e in evs if e["k"] == "wsk"]
    i0 = next(i for i, e in enumerate(evs) if e["k"] == "info")
    after = [e for e in evs[i0:] if e["k"] == "wsk"]
    assert len(after) > 50
    dt = {b["tus"] - a["tus"] for a, b in zip(after, after[1:])}
    assert dt == {8000}
    skipped = set(range(32)) - set(channels_of_selects(0b10111111))
    assert all(after[-1]["rail"] >> i & 1 for i in skipped)
    st = [e for e in evs if e["k"] == "status"][-1]
    assert st["esp32"]["cfg"]["scan_hz"] == 125 and "health" in st["esp32"]
    assert scans
    # refused: no INFO from M, an M: text line, the old mode stays
    _esp32(tr, "M1 200 512 8000;")
    evs = _events(tr, 0.3)
    assert not [e for e in evs if e["k"] == "info"]
    assert any(e["k"] == "txt" and e["msg"].startswith("M:") for e in evs)
    # reboot: ~2 s without scans, then a legacy INFO (all selects) and 80 Hz
    sig.fault("reboot")
    evs = _events(tr, 2.6)
    infos = [e for e in evs if e["k"] == "info"]
    assert infos and infos[-1]["mode"] == 0 and infos[-1]["sel_mask"] == 0xFF
    tus = [e["tus"] for e in evs if e["k"] == "wsk"][-5:]
    assert {b - a for a, b in zip(tus, tus[1:])} == {12500}
    tr.close()
