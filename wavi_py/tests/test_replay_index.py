"""The recording index (replay/index.py), its events (replay/events.py) and the
fitted clocks (daq/clock.py), on the synthetic recording (replay_fixture.py);
plus the real pilot2 / vac_1102 recordings when they are there (slow).

Run: python -m pytest wavi_py/tests/test_replay_index.py
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wavi_py.daq.clock import LiveClock  # noqa: E402
from wavi_py.replay import index as index_mod  # noqa: E402
from wavi_py.replay.index import K, KINDS, NotARecording, RecordingIndex  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402

PILOT2 = Path("/scratch/bigeme/whisker_dev/pilot2")
DIVE = PILOT2 / "st_2026-11-02_1644_57.91_in_water_pilot2.jsonl"
SHORT = PILOT2 / "st_2026-11-02_1639_23.86_in_water_pilot2.jsonl"


@pytest.fixture
def rec(tmp_path):
    p = tmp_path / "rec.jsonl"
    return p, fx.write_recording(p)


def test_offsets_round_trip(rec):
    p, truth = rec
    idx = RecordingIndex.build(p)
    data = p.read_bytes()
    assert idx.n_lines == truth["n_lines"]              # the truncated line is dropped
    assert idx.off[-1] == data.rfind(b"\n") + 1
    for i in range(idx.n_lines):
        ev = json.loads(data[idx.off[i]:idx.off[i + 1]])
        assert ev["t"] == idx.t[i]
        assert KINDS[idx.kind[i]] == ev["k"]
    assert idx.hello_line == 0 and idx.hello["mono"] == fx.MONO
    assert idx.t0 == truth["t0"]
    assert idx.n_wsk == truth["n_wsk"] and idx.n_raw_imu == truth["n_raw_imu"]
    assert abs(idx.fs - fx.FS) < 1e-6
    assert np.all(np.diff(idx.t_play) >= 0)
    assert np.diff(idx.t).min() < -0.15                 # the late block steps back
    assert len(idx.info_lines) == 1 and len(idx.status_lines) == 10
    assert abs(idx.unix0 - (fx.UNIX + idx.t0 - fx.MONO)) < 1e-6
    assert idx.has_vehicle
    # V0: the first 0.25 s of scans, in mV
    v0 = np.mean([fx.raw_of(5000 + i) for i in range(int(fx.FS / 4))], axis=0) * fx.MV
    assert np.allclose(idx.v0, v0, rtol=1e-6)


def test_compact_hello_and_no_hello(tmp_path):
    p = tmp_path / "c.jsonl"
    fx.write_recording(p, compact_hello=True, truncated=False, vehicle=False)
    idx = RecordingIndex.build(p)
    assert idx.kind[0] == K["hello"] and idx.hello["server"] == "wavi_py-esp32"
    assert not idx.has_vehicle and idx.n_raw_imu == 0 and not len(idx.press_t)
    bad = tmp_path / "bad.jsonl"
    bad.write_bytes(b'{"k":"wsk","t":1.0,"seq":1,"tus":1,"raw":[0]}\n')
    with pytest.raises(NotARecording):
        RecordingIndex.build(bad)


def test_events_and_depth(rec):
    p, truth = rec
    idx = RecordingIndex.build(p)
    t0 = truth["t0"]
    off = truth["t_first"] - t0                          # fixture times are from t_first
    # armed from the [1,1] heartbeats only (the [1,194] one with base_mode 128 is ignored)
    assert len(idx.armed_spans) == 1
    a, b = idx.armed_spans[0]
    assert abs(a - (fx.ARM_AT + off)) < 0.4 and abs(b - (fx.DISARM_AT + off)) < 0.4
    kinds = [e.kind for e in idx.events]
    assert kinds.count("arm") == 1 and kinds.count("disarm") == 1
    modes = [e.text for e in idx.events if e.kind == "mode"]
    assert modes == ["MANUAL", "MANUAL -> ALT_HOLD"]
    st = [e for e in idx.events if e.kind == "statustext"]
    assert [(e.text, e.level, e.count) for e in st] == [
        ("Lost manual control", "error", 3), ("#Gain is 50%", "info", 1)]
    assert abs(st[0].t_last - (7.0 + off)) < 0.01
    gaps = [e for e in idx.events if e.kind == "seq_gap"]
    assert len(gaps) == 1 and "5 scans" in gaps[0].text
    assert abs(gaps[0].t - (fx.SEQ_GAP_AT + 0.02 + off)) < 0.03
    faults = [e for e in idx.events if e.kind == "fault"]
    assert len(faults) == 1 and faults[0].level == "fault"
    ctr = [e for e in idx.events if e.kind == "counter"]
    assert len(ctr) == 1 and "adc_crc +3" in ctr[0].text
    assert all(x.t <= y.t for x, y in zip(idx.events, idx.events[1:]))
    # depth: zero at the surface (the disarmed start), the dive's maximum, crossings
    assert abs(idx.surface_hpa - fx.HPA_SURF) < 1e-6
    tm, dm = idx.max_depth()
    assert abs(dm - fx.MAX_DEPTH_CM) < 0.5 and abs(tm - (5.0 + off)) < 0.1
    down = idx.depth_crossing(0.0, 20.0)
    up = idx.depth_crossing(down, 20.0)
    assert abs(down - (2.5 + 5 / 6 + off)) < 0.1 and abs(up - (7.5 - 5 / 6 + off)) < 0.1
    assert idx.depth_crossing(up, 20.0) is None
    assert idx.depth_crossing(9.0, 20.0, forward=False) == up
    assert abs(idx.depth_at(5.0 + off) - fx.MAX_DEPTH_CM) < 0.5


def test_fitted_clock(rec):
    """The scan times from the fit sit on the true device times plus the smallest
    latency (1 ms), much closer than the bursty receive times."""
    p, truth = rec
    idx = RecordingIndex.build(p)
    clock = idx.clock()
    data = [json.loads(line) for line in p.read_bytes().splitlines()[:-1]]
    wsk = [e for e in data if e["k"] == "wsk"]
    fit = np.asarray([clock.wsk(e["t"], e["tus"]) for e in wsk])
    rx = np.asarray([e["t"] for e in wsk])
    true = truth["wsk_t_dev"] + 0.001
    assert np.abs(fit - true).max() < 0.3e-3
    assert np.abs(rx - true).max() > 5e-3
    # the live clock (window minimum) is close too, after its first window
    live = LiveClock()
    lt = np.asarray([live.wsk(e["t"], e["tus"]) for e in wsk])
    assert np.abs(lt - true)[len(lt) // 2:].max() < 1e-3
    imu = [e for e in data if e["k"] == "mav" and e["type"] == "RAW_IMU"]
    ti = np.asarray([clock.imu(e["t"], e["d"]["time_usec"]) for e in imu])
    assert np.all(np.diff(ti) > 0.009)                  # smooth 100 Hz despite the jitter


def test_cache(rec, tmp_path, monkeypatch):
    p, _ = rec
    a = RecordingIndex.open(p)
    assert not a.from_cache and (tmp_path / "rec.jsonl.idx.npz").exists()
    b = RecordingIndex.open(p)
    assert b.from_cache
    for name in ("off", "t", "kind", "v0", "press_t"):
        assert np.array_equal(getattr(a, name), getattr(b, name))
    assert [e.to_dict() for e in a.events] == [e.to_dict() for e in b.events]
    assert (a.t0, a.fs, a.wsk_segs, a.hello) == (b.t0, b.fs, b.wsk_segs, b.hello)
    with open(p, "ab") as f:                             # changed: rebuilt
        f.write(b"\n")
    assert not RecordingIndex.open(p).from_cache


def test_cache_fallback(tmp_path, monkeypatch):
    if os.name != "posix" or os.geteuid() == 0:
        pytest.skip("needs a read-only directory")
    d = tmp_path / "ro"
    d.mkdir()
    p = d / "rec.jsonl"
    fx.write_recording(p)
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    d.chmod(0o555)
    try:
        assert not RecordingIndex.open(p).from_cache
        assert RecordingIndex.open(p).from_cache
        assert list((tmp_path / "cache" / "wavi_py").glob("rec.*.idx.npz"))
    finally:
        d.chmod(0o755)
    win = index_mod.cache_paths(Path("C:/data/x.jsonl"))
    assert win[0].name == "x.jsonl.idx.npz"


@pytest.mark.skipif(not DIVE.exists(), reason="pilot2 recordings not here")
def test_real_dive():
    t = time.perf_counter()
    idx = RecordingIndex.build(DIVE)
    secs = time.perf_counter() - t
    print(f"\n{idx.summary()}  built in {secs:.2f} s")
    assert secs < 5.0
    assert abs(idx.fs - 250.0) < 0.05
    assert len(idx.armed_spans) == 1
    a, b = idx.armed_spans[0]
    # quick.txt gives the last armed heartbeat (629.7); this is the first disarmed one
    assert abs(a - 21.7) < 0.5 and 629.6 < b < 631.2
    texts = [(round(e.t, 1), e.text, e.count) for e in idx.events if e.kind == "statustext"]
    print(texts)
    assert ("#Gain is 50%" in [x[1] for x in texts]
            and any("yaw alignment complete" in x[1] for x in texts))
    tm, dm = idx.max_depth()
    print(f"max depth {dm:.1f} cm at {tm:.1f} s, surface {idx.surface_hpa:.2f} hPa")
    assert 40 < dm < 55
    print([f"{e.t:.1f} {e.kind} {e.text} x{e.count}" for e in idx.events])


@pytest.mark.skipif(not SHORT.exists(), reason="pilot2 recordings not here")
def test_real_short():
    idx = RecordingIndex.build(SHORT)
    lost = [e for e in idx.events if e.text == "Lost manual control"]
    # 9 in quick.txt: 69.7, 72.7 | 83.8 ... 107.1 (the 11 s pause starts a new entry)
    assert sum(e.count for e in lost) == 9 and len(lost) == 2
    a, b = idx.armed_spans[0]
    assert abs(a - 0.8) < 0.5 and 68.7 < b < 70.3
