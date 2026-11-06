"""Selections and the Analysis window (phase C1): the model, Select-mode drag, edge
drags propagating to every plot and the Analysis windows, Compare's data against
``series().channels``, two independent windows, the paused-live snapshot.
Screenshots go to $WAVI_TEST_SHOTS if set; $WAVI_ANALYSIS_REC replays a real
recording instead of the synthetic one for the screenshot test.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_analysis.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtCore import QPointF, Qt  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.daq.analysis import (Dataset, SelSource, imu_variables, snapshot_of,  # noqa: E402
                                  whisker_variables)
from wavi_py.daq.series import IMU, WSK, ArraySeries  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402

SHOTS = os.environ.get("WAVI_TEST_SHOTS")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


def _pump(app, pred, secs: float) -> bool:
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        app.processEvents()
        if pred():
            return True
        time.sleep(0.01)
    app.processEvents()
    return pred()


def _shot(widget, name: str) -> None:
    if SHOTS:
        Path(SHOTS).mkdir(parents=True, exist_ok=True)
        widget.grab().save(str(Path(SHOTS) / f"{name}.png"))


def _open(app, path, **kw):
    from wavi_py.main_window import MainWindow
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, replay_path=str(path), **kw)
    w.show()
    w.open_replay(str(path))
    assert _pump(app, lambda: w._client is not None and w._replay.active, 30)
    assert _pump(app, lambda: w._store is not None, 60)
    return w


@pytest.fixture
def win(app, tmp_path):
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = _open(app, p, t_buffer_s=2.0)
    yield w
    w.close()


class _Drag:
    """What a ViewBox's mouseDragEvent reads of a pyqtgraph MouseDragEvent."""

    def __init__(self, down, pos, start=False, finish=False):
        self._down, self._pos, self._start, self._finish = down, pos, start, finish
        self.accepted = False

    def button(self):
        return Qt.MouseButton.LeftButton

    def buttonDownScenePos(self):  # noqa: N802
        return self._down

    def scenePos(self):  # noqa: N802
        return self._pos

    def isStart(self):  # noqa: N802
        return self._start

    def isFinish(self):  # noqa: N802
        return self._finish

    def accept(self):
        self.accepted = True

    def ignore(self):
        pass


def _drag(vb, t0: float, t1: float, steps: int = 4) -> None:
    """A Select drag in ``vb`` from time t0 to t1 (at the middle of its height)."""
    y = vb.viewRange()[1]
    ym = 0.5 * (y[0] + y[1])
    down = vb.mapViewToScene(QPointF(t0, ym))
    for i in range(1, steps + 1):
        pos = vb.mapViewToScene(QPointF(t0 + (t1 - t0) * i / steps, ym))
        vb.mouseDragEvent(_Drag(down, pos, start=i == 1, finish=i == steps))


# ----------------------------------------------------------------------
# the model and the dataset (no windows)
# ----------------------------------------------------------------------

def _series():
    t = np.arange(0, 20, 0.004)
    y = np.stack([np.sin(t + c) for c in range(4)], axis=1).astype(np.float32)
    ti = np.arange(0, 20, 0.01)
    yi = np.tile(np.arange(9, dtype=np.float32), (len(ti), 1)) * 100
    return ArraySeries({WSK: (t, y), IMU: (ti, yi)},
                       {WSK: ["W1_A", "W1_B", "W2_A", "W2_B"], IMU: [f"i{c}" for c in range(9)]})


def test_selection_model(app):
    from wavi_py.views.selections import SelectionModel, sel_color
    m = SelectionModel()
    got = []
    for sig in ("added", "changed", "removed"):
        getattr(m, sig).connect(lambda s, sig=sig: got.append((sig, s.number)))
    assert m.create(1, 2) == "no data"                      # no source: the reason
    src = SelSource(_series())
    m.source_fn = lambda: src
    a, b = m.create(5.0, 2.0), m.create(10, 30)            # swapped; past the end
    assert (a.number, a.t0, a.t1) == (1, 2.0, 5.0)
    assert (b.number, b.t0) == (2, 10.0) and abs(b.t1 - 19.996) < 1e-9   # clamped to the data
    assert sel_color(1) != sel_color(2)
    m.set_range(a, 3.0, 4.0)
    m.set_range(a, 3.0, 4.0)                               # unchanged: no signal
    m.remove(a)
    assert got == [("added", 1), ("added", 2), ("changed", 1), ("removed", 1)]
    assert m.selections == [b] and m.by_number(2) is b
    # paused live: the frozen copy, not source_fn; the selection keeps it
    snap = snapshot_of(_series())
    m.freeze(snap)
    c = m.create(1, 2)
    m.freeze(None)
    assert c.source is snap and c.number == 3 and m.create(1, 2).source is src
    m.clear()
    assert len(m) == 0 and m.create(1, 2).number == 1


def test_dataset_matches_series():
    ser = _series()
    sel = SelSource(ser)
    from wavi_py.daq.analysis import Selection
    s1, s2 = Selection(1, 2.0, 6.0, sel, (0, 20)), Selection(2, 10.0, 12.0, sel, (0, 20))
    vs = whisker_variables(4, ["W1", "W2"]) + imu_variables(depth=False)
    v0 = np.array([0.5, 0, 0, 0], np.float32)
    ds = Dataset([s1, s2], [vs[0], vs[4], vs[7]], v0=v0, scale=2.0)
    # every sample: channels() of the span, in the display units
    t, y = ds.samples(s1, vs[0])
    tt, yy = ser.channels(["W1_A"], 2.0, 6.0)["W1_A"]
    assert np.array_equal(t, tt) and np.allclose(y, (yy - 0.5) * 2.0)
    t, y = ds.samples(s2, vs[4])                           # accel x: mg, as the IMU window
    assert np.allclose(y, 0.0) and len(t) == len(ser.channels(["i0"], 10, 12)["i0"][0])
    t, y = ds.samples(s2, vs[7])                           # gyro x (col 3): mrad/s as it is
    assert np.allclose(y, 300.0)
    assert ds.samples(s2, vs[10])[1][0] == pytest.approx(np.degrees(600.0))   # roll: rad -> deg
    # Compare's traces: relative time inside each selection, an envelope of the samples
    trs = {(tr.sel.number, tr.var.key): tr for tr in ds.traces(100)}
    assert set(trs) == {(n, v.key) for n in (1, 2) for v in (vs[0], vs[4], vs[7])}
    tr = trs[(1, "wsk:0")]
    _, full = ds.samples(s1, vs[0])
    assert tr.t[0] >= 0 and tr.t[-1] <= 4.0 and len(tr.t) <= 2 * 100 + 4
    assert abs(tr.y.min() - full.min()) < 1e-6 and abs(tr.y.max() - full.max()) < 1e-6
    # half as long: half the columns (plus up to a block of samples at each edge)
    assert len(trs[(2, "wsk:0")].t) <= 2 * 50 + 2 * 10 + 8 < 500


# ----------------------------------------------------------------------
# replay GUI
# ----------------------------------------------------------------------

def test_select_drag_and_propagation(app, win):
    w = win
    w._toggle_line.setChecked(True)
    w._toggle_imu.setChecked(True)
    _pump(app, lambda: False, 0.3)
    lv, m = w._line_view, w._selections
    tile = lv._tiles[2]
    # Select off: the drag goes to the ViewBox's own (pan)
    vb = tile.getViewBox()
    panned = []
    orig = type(vb).mouseDragEvent
    type(vb).mouseDragEvent = lambda self, ev, axis=None: panned.append(ev)
    try:
        _drag(vb, 2.0, 4.0)
    finally:
        type(vb).mouseDragEvent = orig
    assert len(m) == 0 and len(panned) == 4
    w._line_select.setChecked(True)
    _drag(tile.getViewBox(), 2.0, 4.0)
    assert len(m) == 1
    sel = m.selections[0]
    assert abs(sel.t0 - 2.0) < 0.05 and abs(sel.t1 - 4.0) < 0.05 and sel.source.series is w._store
    # a too-short drag makes nothing
    _drag(tile.getViewBox(), 5.0, 5.0001)
    assert len(m) == 1
    # shown in every Line plot, every IMU row and every timeline strip
    ov_line, ov_imu, ov_strip = w._sel_overlays
    assert len(ov_line.regions(sel)) == len(lv._all_plots())
    assert len(ov_imu.regions(sel)) == 3 and len(ov_strip.regions(sel)) == len(w._timeline_docks)
    # an Analysis window from the region's menu (the tile's gauges pre-checked)
    m.request("compare", sel, w._line_vars_at(tile, QPointF()))
    aw = w._analysis[-1]
    assert aw.isVisible() and aw.included == [sel] and aw.checked_keys() == ["wsk:4", "wsk:5"]
    _pump(app, lambda: bool(aw.compare.curves), 2)
    assert set(aw.compare.curves) == {(1, "wsk:4"), (1, "wsk:5")}
    # drag an edge in the IMU window: Line, the strips, the Analysis window follow
    r = ov_imu.regions(sel)[1]
    r.lines[1].setValue(6.0)
    assert sel.t1 == 6.0
    assert all(abs(x.getRegion()[1] - 6.0) < 1e-9 for x in ov_line.regions(sel) + ov_strip.regions(sel))
    _pump(app, lambda: False, 0.2)
    box = aw.sel_list.itemWidget(aw.sel_list.topLevelItem(0), 2)
    assert abs(box.value() - 6.0) < 1e-9
    t = aw.compare.curves[(1, "wsk:4")].getData()[0]
    assert t[-1] > 3.9 and t[-1] <= 4.0 + 1e-9                # relative: 6 - 2
    # Compare = the samples of the span through series().channels, as Line draws them
    name = w._store.names(WSK)[4]
    tt, yy = w.series().channels([name], sel.t0, sel.t1)[name]
    _, y = aw.compare.curves[(1, "wsk:4")].getData()
    ref = (yy - w._v0[4]) * w._scale
    assert abs(y.min() - ref.min()) < 1e-4 and abs(y.max() - ref.max()) < 1e-4
    ds = aw.dataset()
    ts, ys = ds.samples(sel, ds.variables[0])
    assert np.array_equal(ts, tt) and np.allclose(ys, ref)
    # t0 typed in the side panel moves it everywhere
    aw.sel_list.itemWidget(aw.sel_list.topLevelItem(0), 1).setValue(3.0)
    assert sel.t0 == 3.0 and abs(ov_line.regions(sel)[0].getRegion()[0] - 3.0) < 1e-9
    # Del deletes the hovered one, everywhere
    m.set_hovered(sel)
    del_act = next(a for a in lv.actions() if a.text() == "Delete selection")
    del_act.trigger()
    assert len(m) == 0 and not ov_line.regions(sel) and aw.included == []
    _pump(app, lambda: not aw.compare.curves or aw.compare._pages.currentIndex() == 0, 1)
    assert aw.compare._pages.currentIndex() == 0              # the hint
    aw.close()


def test_two_analysis_windows(app, win):
    w = win
    m = w._selections
    a, b = m.create(1.0, 3.0), m.create(5.0, 6.0)
    w1 = w.new_analysis_window(a, ["imu:0", "imu:1"])
    w2 = w.new_analysis_window(b, ["wsk:0"])
    assert w._analysis == [w1, w2] and w1 is not w2
    assert w1.included == [a] and w2.included == [b]
    assert w1.checked_keys() == ["imu:0", "imu:1"] and w2.checked_keys() == ["wsk:0"]
    _pump(app, lambda: bool(w1.compare.curves) and bool(w2.compare.curves), 2)
    assert set(w1.compare.rows) == {"mg"} and set(w2.compare.rows) == {"mV"}
    # the newest focused takes a region's Compare
    w._on_analysis_activated(w1)
    m.request("compare", b, [])
    assert w1.included == [a, b] and w2.included == [b]
    # colour by sensor is the default; by selection: the selection's (its region's)
    # colour, shade = the variable within the row
    from wavi_py.views.analysis_cells import SELECTION, SENSOR
    from wavi_py.views.selections import sel_color
    assert w1.color_by == SENSOR == w2.color_by
    w1.set_color_by(SELECTION)
    w1._render()
    col = {k: c.opts["pen"].color() for k, c in w1.compare.curves.items()}
    assert col[(1, "imu:0")].hslHue() == sel_color(1).hslHue() != col[(2, "imu:0")].hslHue()
    assert col[(2, "imu:0")].hslHue() == col[(2, "imu:1")].hslHue() == sel_color(2).hslHue()
    assert col[(1, "imu:0")].lightnessF() < col[(1, "imu:1")].lightnessF() - 0.2
    # the panel's Analysis button raises the newest focused one
    assert w.show_analysis() is w1
    # a window from Go to includes every selection; the menu lists the open ones
    acts = {}
    from PyQt6.QtWidgets import QMenu
    menu = QMenu()
    w._fill_goto(menu)
    acts = {x.text(): x for x in menu.actions() if x.text()}
    assert "Analysis 1" in acts and "Analysis 2" in acts and "New analysis window" in acts
    acts["New analysis window"].trigger()
    w3 = w._analysis[-1]
    assert w3.included == [a, b] and w3.checked_keys() == []
    # hidden: no redraws (the scheduler skips it), moving a selection only marks it
    w2.close()
    assert w2 not in w._analysis and w2 in w._analysis_spare
    calls = []
    orig = w2.compare.render
    w2.compare.render = lambda *x: (calls.append(1), orig(*x))
    m.set_range(b, 5.0, 5.5)
    _pump(app, lambda: False, 0.3)
    assert calls == []
    # reused by the next new window, fresh
    w4 = w.new_analysis_window()
    assert w4 is w2 and w4.number == 4 and w4.included == [a, b] and w4.checked_keys() == []
    # every Analysis window has the panel's chrome
    assert w1.goto_button is not None and w1.rec_indicator is not None
    for x in (w1, w3, w4):
        x.close()
    # ... or opens a new one when none is open (a closed one reused)
    w5 = w.show_analysis()
    assert w5.isVisible() and w._analysis == [w5] and w5.included == [a, b]
    w5.close()


def test_shade_levels():
    from PyQt6.QtGui import QColor
    from wavi_py.views.analysis_cells import shade, shade_levels
    assert shade_levels(1) == [None] and shade_levels(0) == []
    lv = shade_levels(4)
    assert lv[0] < lv[1] < lv[2] < lv[3] and lv[-1] - lv[0] > 0.4
    c = QColor(230, 140, 0)
    assert shade(c, None) == c and abs(shade(c, 0.3).lightnessF() - 0.3) < 0.01
    assert shade(c, 0.3).hslHue() == c.hslHue()


def test_new_source_clears(app, win):
    w = win
    w._selections.create(1.0, 2.0)
    w._disconnect()
    w.open_replay(w._initial_replay_path)
    assert _pump(app, lambda: w._client is not None and w._replay.active, 10)
    assert len(w._selections) == 0


def test_live_paused_snapshot(app, tmp_path):
    from wavi_py.main_window import MainWindow
    w = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    w.show()
    w._connect()
    try:
        assert _pump(app, lambda: w._samples_since_clear > 500, 10)
        m = w._selections
        assert isinstance(m.create(0.5, 1.0), str)             # not paused: refused
        w._pause_btn.setChecked(True)
        assert m.frozen
        ext = w.series().extent(WSK)
        sel = m.create(ext[1] - 1.0, ext[1] - 0.5)
        assert sel.source.snapshot and sel.bounds[1] <= ext[1] + 1e-9
        name = sel.source.series.names(WSK)[0]
        t0, y0 = sel.source.series.channels([name], sel.t0, sel.t1)[name]
        tl, yl = w.series().channels([name], sel.t0, sel.t1)[name]
        assert np.array_equal(t0, tl) and np.array_equal(y0, yl)
        # the buffers move on; the selection keeps its copy, edits stay inside it
        assert _pump(app, lambda: w.series().extent(WSK)[1] > ext[1] + 0.5, 5)
        m.set_range(sel, sel.t0, ext[1] + 5)
        assert sel.t1 == sel.bounds[1]
        t1, y1 = sel.source.series.channels([name], sel.t0, sel.t1)[name]
        assert t1[-1] <= ext[1] + 1e-9
        aw = w.new_analysis_window(sel, ["wsk:0"])
        _pump(app, lambda: bool(aw.compare.curves), 2)
        assert (sel.number, "wsk:0") in aw.compare.curves
        w._pause_btn.setChecked(False)
        assert not m.frozen and isinstance(m.create(0.5, 1.0), str)
        assert m.selections == [sel]                           # kept after Resume
        aw.close()
    finally:
        w.close()


# ----------------------------------------------------------------------
# screenshots (the 13 min dive with WAVI_ANALYSIS_REC)
# ----------------------------------------------------------------------

def test_screenshots(app, tmp_path):
    if not SHOTS:
        pytest.skip("WAVI_TEST_SHOTS not set")
    rec = os.environ.get("WAVI_ANALYSIS_REC")
    if rec:
        p, spans = Path(rec), [(400.0, 430.0), (520.0, 550.0), (600.0, 615.0)]
    else:
        p = tmp_path / "rec.jsonl"
        fx.write_recording(p)
        spans = [(1.0, 3.0), (4.0, 6.0), (6.5, 7.5)]
    w = _open(app, p)
    try:
        assert _pump(app, lambda: bool(np.any(w._v0)), 5)       # zeroed without playing
        w._toggle_line.setChecked(True)
        lv = w._line_view
        lv.resize(1400, 850)
        if rec:
            lv.set_time_window(360.0, 660.0)
        _pump(app, lambda: False, 0.5)
        lv._line_select.setChecked(True) if hasattr(lv, "_line_select") else w._line_select.setChecked(True)
        for t0, t1 in spans:
            _drag(lv._tiles[0].getViewBox(), t0, t1)
        _pump(app, lambda: False, 0.5)
        _shot(lv, "analysis_line_selections")
        w.resize(w.minimumSizeHint().width(), w.height())     # the narrowest panel
        _pump(app, lambda: False, 0.2)
        _shot(w, "analysis_panel")
        m = w._selections
        s1, s2 = m.selections[:2]
        keys = ["wsk:0", "wsk:1", "wsk:6", "wsk:7", "imu:0", "imu:1", "imu:2"]
        a1 = w.new_analysis_window(s1, keys)
        a1.include(s2)
        a1.resize(1400, 850)
        _pump(app, lambda: False, 0.8)
        _shot(a1, "analysis_compare")
        # the legibility case: 4 variables x 3 selections (W1 A / B, W4 A / B)
        a1.include(m.selections[2])
        a1.set_checked(["imu:0", "imu:1", "imu:2"], False)
        _pump(app, lambda: False, 0.8)
        _shot(a1, "analysis_compare_4x3")
        a1.include(m.selections[2], False)
        a1.set_checked(["imu:0", "imu:1", "imu:2"], True)
        _shot(a1.centralWidget().widget(0), "analysis_side_panel")
        a2 = w.new_analysis_window(m.selections[2], ["imu:3", "imu:4", "imu:5", "imu:8", "depth"])
        a2.resize(1100, 700)
        _pump(app, lambda: False, 0.8)
        _shot(a2, "analysis_second_window")
        # both windows together, side by side
        from PyQt6.QtGui import QPainter, QPixmap
        g1, g2 = a1.grab(), a2.grab()
        pm = QPixmap(g1.width() + g2.width(), max(g1.height(), g2.height()))
        pm.fill(Qt.GlobalColor.white)
        pt = QPainter(pm)
        pt.drawPixmap(0, 0, g1)
        pt.drawPixmap(g1.width(), 0, g2)
        pt.end()
        pm.save(str(Path(SHOTS) / "analysis_two_windows.png"))
    finally:
        w.close()
