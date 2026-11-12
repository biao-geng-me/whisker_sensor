"""A live stream that has run for a while, and the Analysis window on it.

- ``daq/grow.Rows`` and the tracker's reads (``RovTrack.arrays``, the depth): a
  read converts only the rows since the last one, and what an earlier read
  returned does not change;
- live an Analysis window works on a copy and stands still: its timeline is not
  read again while the stream runs on (it was, on every redraw, up to 20 times a
  second, each time converting the whole stream's telemetry: an hour in, the
  GUI thread had no time left). Asked for again it takes a new copy; paused it
  is the Pause's; a region sent to it after a later Pause brings its newer one;
  a Select drag in its timeline makes a selection on its copy without Pause;
- a hidden timeline bar is not rebuilt once a second.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_live_analysis.py
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from PyQt6.QtCore import QTimer

from wavi_py.daq.grow import FOLD, Rows
from wavi_py.daq.series import WSK
from wavi_py.replay.events import EventTracker, LiveTimeline, hpa_per_cm
from wavi_py.replay.rov import rov_data
from wavi_py.tests.test_analysis import _drag, _pump
from wavi_py.tests.test_analysis_r2 import app, no_modal_errors  # noqa: F401
from wavi_py.views.selections import COMPARE

P0 = 1013.0


def _mav(t: float, mtype: str, **d) -> dict:
    return {"k": "mav", "t": t, "type": mtype, "src": [1, 1], "d": d}


def _feed(tr: EventTracker, t: float, deep: bool = True) -> None:
    """One instant of the vehicle's stream: pilot, thrusters, pressure (at the
    surface, or 5 hPa and a wave under it)."""
    tr.feed(_mav(t, "RC_CHANNELS", chan1_raw=1500, chan2_raw=1500, chan3_raw=1500,
                 chan4_raw=1500, chan5_raw=1500 + int(200 * math.sin(t)), chan6_raw=1500))
    tr.feed(_mav(t, "SERVO_OUTPUT_RAW", **{f"servo{i}_raw": 1500 + 10 * i for i in range(1, 9)}))
    tr.feed(_mav(t, "SCALED_PRESSURE2", press_abs=P0 + (5.0 + math.sin(t) if deep else 0.0)))


def _tracker(t0: float, t1: float, dt: float = 0.05) -> EventTracker:
    tr = EventTracker()
    tr.feed(_mav(t0, "HEARTBEAT", base_mode=81, custom_mode=19))      # disarmed: the surface
    for t in np.arange(t0, t1, dt):
        _feed(tr, float(t), deep=t >= t0 + 12.0)             # (its first 10 s: the surface)
    return tr


def test_rows():
    r = Rows(3)
    for i in range(10):
        r.append((i, 2 * i, 3 * i))
    a = r.array()
    keep = a.copy()
    assert a.shape == (10, 3) and len(r) == 10
    n = 2 * FOLD + 500                                       # past a fold and a new buffer
    for i in range(10, n):
        r.append((i, 2 * i, 3 * i))
    assert len(r) == n and len(r._new) < FOLD                # folded while it was fed
    b = r.array()
    assert b.shape == (n, 3) and np.array_equal(b[:, 0], np.arange(n))
    assert np.array_equal(a, keep) and np.array_equal(b[:10], keep)      # the old view stands
    r.append((n, 0, 0))
    c = r.array()
    assert len(c) == n + 1 and np.shares_memory(b, c)        # the same buffer: one row added
    assert Rows(2).array().shape == (0, 2)


def test_tracker_reads_only_the_new_rows():
    tr = _tracker(0.0, 300.0)                                # 6000 instants
    lt = LiveTimeline(tr, lambda: 0.0, lambda: 300.0, lambda: None)
    d0 = lt.rov()
    t0, y0 = d0.series("rov:fwd")
    keep = (t0.copy(), y0.copy())
    td0, dd0 = lt.depth()
    assert len(t0) == len(td0) == 6000
    rows = tr.rov.rows["RC_CHANNELS"]
    assert not rows._new and not tr._press._new              # all in the buffers now
    for k in range(5):
        _feed(tr, 300.0 + 0.05 * k)
    assert len(rows._new) == 5 and len(tr._press._new) == 5  # what the next read converts
    t1, y1 = lt.rov().series("rov:fwd")
    td1, dd1 = lt.depth()
    assert len(t1) == len(td1) == 6005
    assert np.array_equal(t1[:6000], keep[0]) and np.array_equal(y1[:6000], keep[1])
    assert np.array_equal(t0, keep[0]) and np.array_equal(y0, keep[1])   # the first read's
    assert np.array_equal(td1[:6000], td0) and np.array_equal(dd1[:6000], dd0)
    # the values: the pilot's (raw - 1500) / 400, the depth from the surface pressure
    assert y1[0] == pytest.approx(0.0) and np.nanmax(np.abs(y1)) == pytest.approx(0.5, abs=0.01)
    assert dd1[0] == pytest.approx(0.0, abs=1e-6)
    assert dd1[-1] == pytest.approx((5.0 + math.sin(float(td1[-1]))) / hpa_per_cm(), abs=1e-6)
    # as the index reads it (once, at the end)
    whole = rov_data(_tracker(0.0, 300.25).rov, 0.0, [], [], 300.25)
    assert np.array_equal(whole.series("rov:fwd")[1], y1)


@pytest.fixture
def live(app, tmp_path):
    """A sim stream with a live timeline as the Server transport's: 2 min of the
    vehicle's telemetry before it, and more as it runs."""
    from wavi_py.main_window import MainWindow
    w = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    w.show()
    w._connect()
    assert _pump(app, lambda: w._samples_since_clear > 500, 10)
    tr = _tracker(-120.0, 0.0)
    lt = LiveTimeline(tr, lambda: 0.0, lambda: float(w._sig_t[-1]), lambda: None)
    calls = {"rov": 0, "depth": 0}
    rov, depth = lt.rov, lt.depth

    def count(name, fn):
        def f(*a, **k):
            calls[name] += 1
            return fn(*a, **k)
        return f
    lt.rov, lt.depth = count("rov", rov), count("depth", depth)
    w._replay.set_live(lt)
    feeder = QTimer()
    feeder.setInterval(20)
    feeder.timeout.connect(lambda: _feed(tr, float(w._sig_t[-1])))
    feeder.start()
    yield w, lt, calls
    feeder.stop()
    w.close()


def test_analysis_live_is_a_copy(app, live):
    w, _lt, calls = live
    m = w._selections
    aw = w.show_analysis()                                   # not paused
    aw.resize(1300, 860)
    tl = aw.timeline
    assert _pump(app, lambda: tl.renders >= 1 and "pilot" in tl.rows and "cm" in tl.rows, 3)
    _pump(app, lambda: False, 0.3)
    copy = aw._copy
    ext = copy.extent()
    assert copy.snapshot and ext[1] <= w.series().extent(WSK)[1]
    assert len(copy.depth[0]) and copy.rov                   # the stream so far, with it
    assert tl._extent[0] == pytest.approx(-120.0, abs=0.1)   # (the telemetry's start)
    assert not any(h.isVisible() for h in tl._heads.values())     # no "now" on a copy

    # the stream runs on: the window stands still and does not read it
    n, extent, n_rov = tl.renders, tl._extent, calls["rov"]
    assert _pump(app, lambda: w.series().extent(WSK)[1] > ext[1] + 1.5, 5)
    assert aw._copy is copy and tl.renders == n and tl._extent == extent
    assert calls["rov"] - n_rov <= 4                         # the variables' 1 Hz poll only

    # asked for again: a copy up to now
    assert w.show_analysis() is aw
    assert aw._copy is not copy and aw._copy.extent()[1] > ext[1] + 1.5
    assert _pump(app, lambda: tl.renders > n and tl._extent[1] > extent[1] + 1.5, 3)

    # a Select drag in its timeline: a selection on its copy, without Pause
    assert isinstance(m.create(1.0, 2.0), str)               # (the Line window's needs Pause)
    e = aw._copy.extent()
    aw._act_select.setChecked(True)
    _drag(tl.x_plot().getViewBox(), e[1] - 1.0, e[1] - 0.4)
    aw._act_select.setChecked(False)
    assert len(m) == 1
    sel = m.selections[0]
    assert sel.source is aw._copy and sel in aw.included and sel.dome_map is None
    assert sel.t0 == pytest.approx(e[1] - 1.0, abs=0.02) and sel.bounds == e
    aw.set_checked(["wsk:0"])
    assert _pump(app, lambda: (sel.number, "wsk:0") in aw.compare.curves, 3)

    # paused: a window opened then has the Pause's copy
    aw.close()
    w._pause_btn.setChecked(True)
    first = m.frozen_source
    aw = w.show_analysis()                                   # (the closed one, reused)
    assert aw._copy is first and sel in aw.included
    # a later Pause's region sent here brings its copy
    w._pause_btn.setChecked(False)
    assert _pump(app, lambda: w.series().extent(WSK)[1] > first.extent()[1] + 0.5, 5)
    w._pause_btn.setChecked(True)
    second = m.frozen_source
    assert second is not first
    e2 = second.extent()
    sel2 = m.create(e2[1] - 1.0, e2[1] - 0.5)
    w._on_selection_request(COMPARE, sel2, ["wsk:0"])
    assert aw._copy is second and sel2 in aw.included
    aw.include(sel2, False)                                  # it stays (only forward)
    assert aw._copy is second
    w._pause_btn.setChecked(False)
    aw.close()


def test_hidden_timeline_bars_wait(app, live):
    w, _lt, calls = live
    bars = [d.widget() for d in w._timeline_docks]
    shown = [b for b in bars if b.isVisible()]
    assert len(bars) >= 6 and shown == [bars[0]]             # the panel's only
    _pump(app, lambda: False, 0.1)
    n = calls["depth"]
    w._replay.sourceUpdated.emit()
    assert calls["depth"] - n == 1 and all(b._stale for b in bars[1:])
    w._toggle_line.setChecked(True)
    w._line_view.timeline_dock.show()                        # it catches up when shown
    assert _pump(app, lambda: not bars[1]._stale, 2)
    assert calls["depth"] - n >= 2
    t = bars[1].strip.depth.xData                            # (all of it, not the part in view)
    assert t is not None and len(t) > 2000
