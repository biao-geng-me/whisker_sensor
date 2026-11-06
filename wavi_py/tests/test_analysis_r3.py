"""Analysis round 3: the vehicle telemetry as variables, the grid picker and
legends, colour by sensor, expand, the zoom box, cursor time labels and several
sub-ranges.

The ROV part: extraction from the index pass (values and scaling against the
fixture's truth and on the real 13 min dive when it is there), the live path
(the tracker fed parsed events gives the same as the index's byte parse), the
cache (round trip, an older version rebuilt), the Dataset (curves, spectra, the
mode band timeline-only) and the Analysis window's ROV group (shown only when
the recording has the telemetry).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_analysis_r3.py
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.daq.analysis import (ROV, Dataset, Selection, SelSource, rov_variables,  # noqa: E402
                                  snapshot_of)
from wavi_py.daq.series import ArraySeries  # noqa: E402
from wavi_py.replay import index as rindex  # noqa: E402
from wavi_py.replay.events import EventTracker  # noqa: E402
from wavi_py.replay.index import RecordingIndex  # noqa: E402
from wavi_py.replay.rov import MODE_KEY, rov_data  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.tests.test_analysis import SHOTS, _open, _pump, _shot  # noqa: E402

DIVE = Path("/scratch/bigeme/whisker_dev/pilot2/st_2026-11-02_1644_57.91_in_water_pilot2.jsonl")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


@pytest.fixture
def rov_rec(tmp_path):
    p = tmp_path / "rov.jsonl"
    truth = fx.write_recording(p, rov=True)
    return p, truth


# ----------------------------------------------------------------------
# part 1: the ROV telemetry
# ----------------------------------------------------------------------

def test_rov_extraction_values_and_scaling(rov_rec):
    p, truth = rov_rec
    idx = RecordingIndex.build(p)
    r = idx.rov()
    keys = [k for k, *_ in r.variables()]
    assert keys[0] == MODE_KEY
    assert {"rov:fwd", "rov:lat", "rov:vert", "rov:yaw", "rov:thr1", "rov:thr7", "rov:current",
            "rov:climb", "rov:heading", "rov:vib_x"} <= set(keys)
    assert "rov:thr8" not in keys                       # servo8 = 0 throughout: unused
    assert "rov:current_sys" not in keys
    t, fwd = r.series("rov:fwd")
    assert len(t) == int(fx.DUR_S * 20)                 # the [1,2] RC_CHANNELS are ignored
    assert np.all(np.diff(t) >= 0)                      # sorted (the late MAVLink block)
    s = np.arange(len(t)) / 20.0                        # the fixture's s of each row
    assert np.allclose(t, s + truth["t_first"] + 0.004 - truth["t0"], atol=1e-6)
    want = np.array([fx.rov_truth(x)[0] for x in s])
    assert np.allclose(fwd, np.round(400 * want) / 400, atol=1e-9)    # (raw - 1500) / 400
    _t, lat = r.series("rov:lat")
    assert np.allclose(lat, 0.0)
    _t, thr1 = r.series("rov:thr1")
    assert np.allclose(thr1, [round(400 * fx.rov_truth(x)[1]) / 400 for x in s])
    _t, thr8 = r.series("rov:thr8")
    assert np.isnan(thr8).all()
    _t, hdg = r.series("rov:heading")                   # unwrapped through north
    assert np.allclose(hdg, [int(fx.rov_truth(x)[2]) for x in s])
    assert hdg.max() > 360
    _t, amps = r.series("rov:current")
    assert np.allclose(amps, 1.5)                       # 150 cA
    tg, gain = r.series("rov:gain")
    assert set(np.unique(gain)) == {0.25, 0.5}           # CamTilt is not the gain
    # the modes (the tracker's heartbeat parsing): MANUAL, then ALT_HOLD at MODE_AT
    names = [m[2] for m in r.modes]
    assert names == ["MANUAL", "ALT_HOLD"]
    assert abs(r.modes[1][0] - fx.MODE_AT) < 0.4
    assert r.modes[-1][1] == pytest.approx(idx.duration)
    assert r.armed == idx.armed_spans


def test_rov_live_path_matches_index(rov_rec):
    """The tracker fed parsed events (live) gives what the index's byte parse does."""
    p, truth = rov_rec
    idx = RecordingIndex.build(p)
    tr = EventTracker()
    with open(p, "rb") as f:
        for line in f:
            if line.endswith(b"\n") and b'"k":"mav"' in line:
                tr.feed(json.loads(line))
    live = rov_data(tr.rov, idx.t0, tr.mode_changes, idx.armed_spans, idx.duration)
    for k, *_ in idx.rov().variables():
        if k == MODE_KEY:
            continue
        a, b = idx.rov().series(k), live.series(k)
        assert np.array_equal(a[0], b[0]) and np.allclose(a[1], b[1], equal_nan=True), k
    assert [m[2] for m in live.modes] == [m[2] for m in idx.rov().modes]


def test_rov_cache_round_trip_and_old_version(rov_rec, monkeypatch, tmp_path):
    p, _truth = rov_rec
    a = RecordingIndex.open(p)
    b = RecordingIndex.open(p)
    assert b.from_cache and not a.from_cache
    for k, *_ in a.rov().variables():
        if k != MODE_KEY:
            assert np.array_equal(a.rov().series(k)[1], b.rov().series(k)[1], equal_nan=True)
    assert b.rov().modes == a.rov().modes
    # an index cached by the previous version (no telemetry) is rebuilt
    monkeypatch.setattr(rindex, "INDEX_VERSION", 2)
    RecordingIndex.open(p, use_cache=False)._save_cache(rindex.cache_paths(p)[0])
    monkeypatch.undo()
    c = RecordingIndex.open(p)
    assert not c.from_cache and c.rov()


def test_no_mavlink_no_rov_group(app, tmp_path):
    p = tmp_path / "esp.jsonl"
    fx.write_recording(p, vehicle=False)
    idx = RecordingIndex.build(p)
    assert not idx.rov() and rov_variables(idx.rov()) == []
    w = _open(app, p, t_buffer_s=2.0)
    try:
        win = w.new_analysis_window()
        assert not any(v.source == ROV for v in win._vars)
        win.close()
    finally:
        w.close()


def test_rov_dataset_curves_spectra_and_snapshot(rov_rec):
    p, _truth = rov_rec
    idx = RecordingIndex.build(p)
    r = idx.rov()
    vs = rov_variables(r)
    mode = next(v for v in vs if v.key == MODE_KEY)
    assert not mode.plots and mode.unit == "mode"
    fwd = next(v for v in vs if v.key == "rov:fwd")
    src = SelSource(ArraySeries(), rov=r)
    sel = Selection(1, 1.0, 9.0, src)
    ds = Dataset([sel], [fwd, mode])
    trs = ds.traces(400)
    assert [t.var.key for t in trs] == ["rov:fwd"]       # the band is not a curve
    assert abs(trs[0].y.max() - 0.5) < 0.01
    f, a = ds.spectrum(sel, fwd)                        # a 0.5 Hz sine of 0.5, at ~20 Hz
    assert f[-1] == pytest.approx(10.0, rel=0.05)
    assert f[np.argmax(a)] == pytest.approx(0.5, abs=0.15)
    assert a.max() == pytest.approx(0.5, rel=0.15)
    assert ds.spectrum(sel, mode) is None
    ov = Dataset([], [fwd, mode]).overview(src, 0.0, 10.0, 300)
    assert [t.var.key for t in ov] == ["rov:fwd"]
    snap = snapshot_of(ArraySeries(), None, r)          # the paused live copy keeps its own
    assert snap.rov.series("rov:fwd")[1] is not r.series("rov:fwd")[1]
    assert np.array_equal(snap.rov.series("rov:fwd")[1], r.series("rov:fwd")[1])


def test_rov_group_in_window_and_timeline(app, rov_rec):
    p, _truth = rov_rec
    w = _open(app, p, t_buffer_s=2.0)
    try:
        win = w.new_analysis_window()
        win.resize(1300, 860)
        keys = [v.key for v in win._vars if v.source == ROV]
        assert keys[0] == MODE_KEY and "rov:fwd" in keys
        win.set_checked([MODE_KEY], column=1)             # no Plots box: ignored
        assert MODE_KEY not in win.checked_keys()
        win.set_timeline(["depth", MODE_KEY, "rov:fwd", "rov:thr1"])
        assert _pump(app, lambda: "mode" in win.timeline.rows and win.timeline.renders > 0, 10)
        _pump(app, lambda: False, 0.5)
        tl = win.timeline
        assert list(tl.rows) == ["cm", "mode", "pilot", "thrust"]
        assert len(tl._mode_items) >= 2 * 2 + 1           # two mode spans (band + label), armed
        assert MODE_KEY in tl.curves and len(tl.curves[MODE_KEY].getData()[0]) > 0
        _shot(win, "r3_rov_timeline_fixture")
        win.close()
    finally:
        w.close()


@pytest.mark.skipif(not DIVE.exists(), reason="the pilot2 dive is not here")
def test_rov_real_dive():
    idx = RecordingIndex.open(DIVE, use_cache=False)
    r = idx.rov()
    assert [m[2] for m in r.modes] == ["MANUAL", "ALT_HOLD", "MANUAL"]
    for k in ("rov:fwd", "rov:lat", "rov:vert", "rov:yaw"):
        t, y = r.series(k)
        assert len(t) == 15517
        assert (len(t) - 1) / (t[-1] - t[0]) == pytest.approx(20.0, abs=1.5)
        assert np.nanmin(y) == pytest.approx(-0.5) and np.nanmax(y) == pytest.approx(0.5)
    for i in range(1, 9):
        _t, y = r.series(f"rov:thr{i}")
        assert -1.0 <= np.nanmin(y) and np.nanmax(y) <= 1.0 and np.nanmax(np.abs(y)) > 0.6
    assert set(np.unique(r.series("rov:gain")[1])) == {0.25, 0.5}
    _t, amps = r.series("rov:current")
    assert 0.2 <= np.nanmin(amps) and np.nanmax(amps) < 60     # cA -> A
    assert r.frame is None                                  # no FRAME_CONFIG: numbers only
