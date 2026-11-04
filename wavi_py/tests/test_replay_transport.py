"""ReplayTransport pacing / seek / loop / EOF / commands on a fake clock, and a
recording replayed through WaviClient + ServerReader (the live pipeline),
cross-checked against whisker_link.convert.

Run: python -m pytest wavi_py/tests/test_replay_transport.py
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wavi_py.client import WaviClient  # noqa: E402
from wavi_py.replay.index import RecordingIndex  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.tests.test_replay_index import DIVE  # noqa: E402
from wavi_py.transports.replay_transport import ReplayTransport  # noqa: E402
from whisker_link import convert  # noqa: E402


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def idx(tmp_path):
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    return RecordingIndex.build(p)


def drain(tr: ReplayTransport, max_calls: int = 100000) -> list[dict]:
    """Everything read_some has now (until it returns nothing)."""
    out = []
    buf = b""
    for _ in range(max_calls):
        b = tr.read_some(65536)
        if not b:
            break
        buf += b
    for line in buf.splitlines():
        out.append(json.loads(line))
    return out


def data_events(evs):
    return [e for e in evs if e["k"] != "replay"]


def test_pacing(idx):
    for speed in (1.0, 4.0, 0.25):
        clk = FakeClock()
        tr = ReplayTransport(idx, speed=speed, clock=clk, playing=True)
        first = drain(tr)
        assert [e["k"] for e in first[:2]] == ["replay", "hello"]
        assert first[-1] == {**first[-1], "k": "replay", "op": "live"}
        served = len(data_events(first))            # the hello, the line at 0
        for w in (0.1, 0.5, 1.0, 1.7):
            clk.now = 100.0 + w
            evs = data_events(drain(tr))
            served += len(evs)
            limit = idx.t0 + speed * w
            assert served == int(np.searchsorted(idx.t_play, limit, side="right"))
            assert all(e["t"] <= limit for e in evs)
            assert abs(tr.position - min(speed * w, idx.duration)) < 1e-9
        tr.close()


def test_pause_speed_backstep_max(idx):
    clk = FakeClock()
    tr = ReplayTransport(idx, speed=1.0, clock=clk, playing=True)
    drain(tr)
    clk.now += 2.0
    drain(tr)
    tr.pause()
    clk.now += 5.0
    assert drain(tr) == [] and abs(tr.position - 2.0) < 1e-9 and tr.state == "paused"
    tr.play()
    clk.now += 0.5
    drain(tr)
    tr.speed = 2.0                                  # keeps the position
    assert abs(tr.position - 2.5) < 1e-9
    # across the late MAVLink block (t steps back 0.2 s at 3 s): no stall, no burst
    n_prev = None
    for k in range(1, 9):
        clk.now += 0.05
        n = len(drain(tr))
        if n_prev is not None:
            assert 0 < n < 3 * max(n_prev, 30)
        n_prev = n
    tr.speed = None                                 # max: everything at once
    rest = drain(tr)
    assert rest[-1]["op"] == "eof" and tr.state == "eof"
    assert drain(tr) == [] and drain(tr) == []      # never raises at the end
    tr.close()
    with pytest.raises(ConnectionError):
        tr.read_some(10)


def test_seek_order_and_gen(idx):
    clk = FakeClock()
    tr = ReplayTransport(idx, speed=1.0, clock=clk, preroll_s=2.0)
    first = drain(tr)
    assert first[0]["op"] == "seek" and first[0]["gen"] == 1 and "v0" in first[0]
    assert abs(first[0]["t0"] - idx.t0) < 1e-12 and abs(first[0]["unix0"] - idx.unix0) < 1e-6
    tr.seek(6.0)
    evs = drain(tr)
    assert [e["k"] for e in evs[:4]] == ["replay", "hello", "info", "status"]
    assert evs[0]["op"] == "seek" and evs[0]["gen"] == 2 and evs[0]["pos"] == 6.0
    assert "v0" not in evs[0]
    assert evs[2]["t"] - idx.t0 < 4.0 and evs[3]["t"] - idx.t0 < 4.0     # before T - preroll
    pre = evs[4:-1]
    assert evs[-1]["op"] == "live" and evs[-1]["pos"] == 6.0
    assert all(4.0 - 0.21 <= e["t"] - idx.t0 <= 6.0 for e in pre)
    assert len(pre) == int(idx.line_at(6.0) - idx.line_at(4.0))
    assert tr.position == 6.0 and tr.state == "paused"
    tr.seek(1e6)                                    # past the end: at the end, then eof
    tr.play()
    evs = drain(tr)
    assert evs[0]["gen"] == 3 and evs[-1]["op"] == "eof"
    tr.play()                                       # after the end: from 0
    evs = drain(tr)
    assert evs[0]["op"] == "seek" and evs[0]["pos"] == 0.0 and tr.state == "playing"


def test_loop(idx):
    clk = FakeClock()
    tr = ReplayTransport(idx, speed=1.0, clock=clk, preroll_s=0.5, playing=True)
    drain(tr)
    tr.set_loop(2.0, 3.0)                           # outside: jumps in
    evs = drain(tr)
    assert evs[0]["op"] == "seek" and evs[0]["pos"] == 2.0
    gens = set()
    for _ in range(30):
        clk.now += 0.1
        for e in drain(tr):
            if e["k"] == "replay":
                gens.add((e["gen"], e["op"]))
            else:
                assert e["t"] - idx.t0 <= 3.0
    seeks = [g for g, op in gens if op == "seek"]
    assert len(seeks) >= 2                          # wrapped (3 s of play over a 1 s loop)
    tr.set_loop(None)
    tr.speed = None
    assert drain(tr)[-1]["op"] == "eof"
    # a loop with no line in it (inside a stall gap): refused, not a hot re-seek loop
    t_a = float(idx.t_play[idx.line_at(4.0)] - idx.t0)
    tr.set_loop(t_a + 1e-7, t_a + 2e-7)
    assert tr._loop is None and tr._end_line == idx.n_lines


def test_commands_refused(idx, tmp_path):
    tr = ReplayTransport(idx, speed=1.0, clock=FakeClock())
    drain(tr)
    before = sorted(tmp_path.iterdir())
    tr.write(b'{"cmd": "esp32", "data": "M2;"}\n{"cmd": "rec_start"}\n{"cmd": "status"}\n')
    acks = drain(tr)
    assert [(a["cmd"], a["ok"]) for a in acks] == [("esp32", False), ("rec_start", False),
                                                   ("status", True)]
    assert acks[0]["error"] == "replay: read-only"
    assert sorted(tmp_path.iterdir()) == before


def _run_client(path, **kw) -> WaviClient:
    c = WaviClient(transport="replay", path=str(path), replay_speed=None, **kw)
    c.start()
    assert c.wait_ready(5)
    rp = c.replay
    t_end = time.monotonic() + 20
    while rp.state != "eof" and time.monotonic() < t_end:
        time.sleep(0.02)
    time.sleep(0.2)                                 # the reader's last flush
    return c


def test_pipeline(tmp_path):
    p = tmp_path / "rec.jsonl"
    truth = fx.write_recording(p)
    c = _run_client(p)
    idx = c.index
    try:
        assert c.is_running and c.has_vehicle         # still running at EOF
        assert c.endpoint_str == "replay://rec.jsonl"
        assert abs(c.fs - fx.FS) < 1e-6
        assert c.bus.total_published == truth["n_wsk"]       # V0 came with the seek
        assert c.imu_bus.total_published == truth["n_raw_imu"]
        meta = c.bus.get_metadata()
        assert np.allclose(meta["V0"], idx.v0) and meta["v0_version"] == 1
        assert abs(meta["t0_unix"] - idx.unix0) < 1e-6
        marks = meta["replay_marks"]
        # the scan at time 0 is served with the (empty) preroll, before "live"
        assert [(m[1], m[2]) for m in marks] == [("seek", 0), ("live", 1),
                                                 ("eof", truth["n_wsk"])]
        x, t, _ = c.bus.read_since_t(0)
        # the bus content is convert's wsk_mv, the times convert's fitted wsk_t
        arr = convert.to_arrays(convert.load(p))
        assert np.allclose(x, arr["wsk_mv"], atol=1e-4)
        shift = idx.t0 - truth["hello"]["t"]         # convert's zero is the hello
        # (convert's 5 s envelope windows can't fit a slope to 10 s: 0.8 ms of drift)
        assert np.abs(t - (arr["wsk_t"] - shift)).max() < 1.2e-3
        assert np.all(np.diff(t) > 0)
        _, ti, _ = c.imu_bus.read_since_t(0)
        assert np.all(np.diff(ti) > 0) and abs(ti[0]) < 0.01
        # a seek: marks at the bus index, V0 kept, the preroll republished
        n0 = c.bus.total_published
        c.replay.seek(5.0)
        c.replay.play()
        t_end = time.monotonic() + 10
        while c.replay.state != "eof" and time.monotonic() < t_end:
            time.sleep(0.02)
        time.sleep(0.2)
        meta = c.bus.get_metadata()
        seek = [m for m in meta["replay_marks"] if m[0] == 2]
        assert seek[0][1] == "seek" and seek[0][2] == n0 and seek[0][4] == 5.0
        assert meta["v0_version"] == 1
        live = [m for m in seek if m[1] == "live"][0]
        _, t2, _ = c.bus.read_since_t(n0)
        assert abs(t2[0] - (5.0 - c.preroll_s)) < 0.02 or t2[0] < 0.02   # preroll from 0
        assert abs(t2[live[2] - n0] - 5.0) < 0.02
        # commands are refused, and Rec does nothing at the source
        assert c.send_command({"cmd": "esp32", "data": "B"})
        assert not c.send_record_start("x", local_copy=str(tmp_path / "copy.jsonl"))
        assert not (tmp_path / "copy.jsonl").exists()
    finally:
        c.stop()


def test_pipeline_start_offset(tmp_path):
    """Starting in the middle: the V0 is still the file's first 0.25 s."""
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    c = WaviClient(transport="replay", path=str(p), replay_speed=None, replay_start_s=6.0)
    c.start()
    try:
        assert c.wait_ready(5)
        time.sleep(0.3)
        assert np.allclose(c.bus.get_metadata()["V0"], c.index.v0)
    finally:
        c.stop()


@pytest.mark.skipif(not DIVE.exists(), reason="pilot2 recordings not here")
def test_real_dive_seek_play():
    """Seek to 400 s and play 5 s at 8x: no drops, times continuous."""
    c = WaviClient(transport="replay", path=str(DIVE), replay_speed=8.0, replay_start_s=400.0)
    c.start()
    try:
        assert c.wait_ready(10)
        seq, t_last, n = 0, None, 0
        t_end = time.monotonic() + 10.0
        while c.replay.position < 405.0 and time.monotonic() < t_end:
            x, t, new = c.bus.read_since_t(seq, block=True, timeout=0.05)
            if x is None:
                continue
            assert new - seq == len(x)              # nothing dropped
            seq = new
            n += len(x)
            t_last = t[-1]
        print(f"\n{n} samples, last at {t_last:.2f} s, playhead {c.replay.position:.2f} s")
        assert c.replay.position >= 405.0
        assert abs(t_last - c.replay.position) < 0.3
        assert n > 0.9 * 250 * (c.preroll_s + 4.0)
    finally:
        c.stop()
