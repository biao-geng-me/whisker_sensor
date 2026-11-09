"""Analysis round 2: the 2x2 layout, the timeline, the cursors, keys and Probe.

Layout and splitter state per window (the column border shared by both rows,
collapse, double-click to equalise), the spectrogram picker and click-to-pick,
cursors added / moved / deleted with the bottom-right curves against a direct
spectrogram column, timeline variables independent of the plot variables (and
a Select drag / double-click there), the zoom keys and Probe, no work while
hidden, and the cost of an edge drag and a cursor drag (12 traces, 3 cursors).
Screenshots go to $WAVI_TEST_SHOTS if set; $WAVI_ANALYSIS_REC replays a real
recording for them.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_analysis_r2.py
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
from PyQt6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PyQt6.QtGui import QKeyEvent  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.daq.analysis import spectrogram_column  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.tests.test_analysis import SHOTS, _drag, _open, _pump, _shot  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


@pytest.fixture
def win(app, tmp_path):
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = _open(app, p, t_buffer_s=2.0)
    yield w
    w.close()


class _Click:
    """What a ViewBox's / an item's mouseClickEvent reads of a MouseClickEvent."""

    def __init__(self, pos, button=Qt.MouseButton.LeftButton, double=False):
        self._pos, self._button, self._double = pos, button, double
        self.accepted = False

    def button(self):
        return self._button

    def double(self):
        return self._double

    def scenePos(self):  # noqa: N802
        return self._pos

    def screenPos(self):  # noqa: N802
        return self._pos

    def accept(self):
        self.accepted = True

    def ignore(self):
        pass


def _click(vb, t, y=None, **kw):
    if y is None:
        r = vb.viewRange()[1]
        y = 0.5 * (r[0] + r[1])
    ev = _Click(vb.mapViewToScene(QPointF(t, y)), **kw)
    vb.mouseClickEvent(ev)
    return ev


def _window(app, w, n_sel=3, keys=("wsk:0", "wsk:1", "wsk:2", "wsk:3")):
    m = w._selections
    spans = [(1.0, 4.0), (4.5, 7.5), (7.0, 9.5)][:n_sel]
    sels = [m.create(*s) for s in spans]
    aw = w.new_analysis_window(sels[0], list(keys))
    for s in sels[1:]:
        aw.include(s)
    aw.resize(1400, 900)
    assert _pump(app, lambda: bool(aw.compare.curves) and aw.spectrogram.data is not None, 3)
    return aw, sels


# ----------------------------------------------------------------------
# layout
# ----------------------------------------------------------------------

def test_layout_and_splitters_per_window(app, win):
    aw, sels = _window(app, win)
    g = aw.grid
    # the 2x2: waveform / spectrum on top, spectrogram / cursors below; the timeline over it
    assert g.top.widget(0) is aw.compare and g.top.widget(1) is aw.spectrum
    assert g.bottom.widget(0) is aw.spectrogram and g.bottom.widget(1) is aw.at_cursors
    assert aw.right.widget(0) is aw.timeline and aw.right.widget(1) is g
    assert g.top.sizes() == g.bottom.sizes()                         # one column border
    # moving the top border moves the bottom one too
    g.top.moveSplitter(500, 1)
    _pump(app, lambda: False, 0.1)
    assert g.top.sizes()[0] == pytest.approx(500, abs=8) and g.bottom.sizes() == g.top.sizes()
    g.bottom.moveSplitter(700, 1)
    assert g.top.sizes() == g.bottom.sizes() and g.top.sizes()[0] == pytest.approx(700, abs=8)
    # the waveform and spectrogram line up: same time at the same screen x
    _pump(app, lambda: False, 0.3)
    wv = next(iter(aw.compare.rows.values())).getViewBox()
    sv = aw.spectrogram.plot.getViewBox()
    for t in (0.5, 2.0):
        xw = wv.mapViewToDevice(QPointF(t, 0)).x() + aw.compare.glw.mapTo(aw, aw.compare.glw.rect().topLeft()).x()
        xs = sv.mapViewToDevice(QPointF(t, 0)).x() + aw.spectrogram.glw.mapTo(aw, aw.spectrogram.glw.rect().topLeft()).x()
        assert abs(xw - xs) < 3
    # the shared border dragged to the edge collapses the right column (both cells):
    # not drawn (an option change there does no work), listed on the restore bar
    g.top.moveSplitter(700, 1)
    cols = g.top.sizes()
    total = sum(g.top.sizes())
    g.top.moveSplitter(total + 10, 1)
    _pump(app, lambda: False, 0.1)
    assert aw.spectrum.isHidden() and aw.at_cursors.isHidden() and g.bar.isVisibleTo(aw)
    assert g.restore_buttons[aw.spectrum].isVisibleTo(aw)
    r = aw.spectrum.renders
    aw.spectrum.window_box.setCurrentIndex(1)
    _pump(app, lambda: False, 0.3)
    assert aw.spectrum.renders == r
    # back from the bar: at the sizes before the drag, both rows in step again
    g.restore_buttons[aw.spectrum].click()
    g.restore_buttons[aw.at_cursors].click()
    assert not g.collapsed() and not g.bar.isVisibleTo(aw)
    assert g.top.sizes() == g.bottom.sizes() and abs(g.top.sizes()[0] - cols[0]) <= 8
    assert _pump(app, lambda: aw.spectrum.renders > r, 2)            # drawn again
    # double-click the border: equal again, both rows
    g.top.handle(1).mouseDoubleClickEvent(None)
    a, b = g.top.sizes()
    assert abs(a - b) <= 2 and g.bottom.sizes() == g.top.sizes()
    # a second window keeps its own sizes
    aw2 = win.new_analysis_window(sels[0], ["wsk:0"])
    aw2.resize(1400, 900)
    _pump(app, lambda: False, 0.2)
    aw2.grid.set_state([300, 900], [200, 500])
    aw2.grid.collapse(aw2.spectrogram)
    _pump(app, lambda: False, 0.1)
    assert aw.grid.top.sizes() == [a, b] and aw2.grid.top.sizes() != aw.grid.top.sizes()
    assert aw2.grid.state()["collapsed"] == ["Spectrogram"] and aw.grid.state()["collapsed"] == []
    aw.close()
    aw2.close()


def test_single_cell_collapse(app, win):
    aw, sels = _window(app, win)
    g = aw.grid
    g.set_state([800, 500], [400, 400])
    _pump(app, lambda: False, 0.3)
    bottom, rows = g.bottom.sizes(), g.rows.sizes()
    # a right cell's button: its row's left cell takes the full width, the other row stays
    aw.spectrum.header.collapse_btn.click()
    _pump(app, lambda: False, 0.3)
    assert aw.spectrum.isHidden() and not aw.at_cursors.isHidden()
    assert aw.compare.width() >= sum(g.top.sizes()) - 2 and g.bottom.sizes() == bottom
    assert [b.text() for c, b in g.restore_buttons.items() if b.isVisibleTo(aw)] == ["▸ Spectrum"]
    # still the same time span in the waveform and the spectrogram (widths differ)
    assert aw.compare.width() > aw.spectrogram.width() + 100
    wr = aw.compare.x_plot().getViewBox().viewRange()[0]
    assert aw.spectrogram.plot.getViewBox().viewRange()[0] == pytest.approx(wr, abs=1e-6)
    aw._hover_cell = aw.spectrogram
    aw._zoom_x(1 / 1.25, aw.spectrogram)
    _pump(app, lambda: False, 0.1)
    assert aw.compare.x_plot().getViewBox().viewRange()[0] == pytest.approx(
        aw.spectrogram.plot.getViewBox().viewRange()[0], abs=1e-6)
    # dragging the bottom border no longer moves the top one (it has one cell)
    g.bottom.moveSplitter(600, 1)
    assert g.top.sizes()[1] == 0 and g.bottom.sizes()[0] == pytest.approx(600, abs=8)
    moved = g.bottom.sizes()
    # both cells of the bottom row: the row goes, the top row takes the height
    aw.spectrogram.header.collapse_btn.click()
    aw.at_cursors.header.collapse_btn.click()
    _pump(app, lambda: False, 0.3)
    assert g.bottom.isHidden() and g.top.height() >= g.rows.height() - 2
    assert len([b for b in g.restore_buttons.values() if b.isVisibleTo(aw)]) == 3
    # an option change in a collapsed cell does no work
    r = aw.at_cursors.renders, aw.spectrogram.renders
    aw.spectrogram.seg_box.setCurrentIndex(0)
    _pump(app, lambda: False, 0.3)
    assert (aw.at_cursors.renders, aw.spectrogram.renders) == r
    # back: the row at its height again, then all four in step
    g.restore(aw.spectrogram)
    got = g.rows.sizes()                     # (the restore bar takes a little height)
    assert not g.bottom.isHidden() and got[0] / sum(got) == pytest.approx(rows[0] / sum(rows), abs=0.02)
    assert aw.spectrogram.width() >= sum(g.bottom.sizes()) - 2       # alone in its row
    assert _pump(app, lambda: aw.spectrogram.renders > r[1], 2)
    g.restore(aw.at_cursors)
    assert g.bottom.sizes() == pytest.approx(moved, abs=4)
    g.restore(aw.spectrum)
    assert _pump(app, lambda: g.top.sizes() == g.bottom.sizes(), 1) and not g.bar.isVisibleTo(aw)
    # a region menu's Spectrum brings a collapsed spectrum back
    g.collapse(aw.spectrum)
    win._selections.request("spectrum", sels[0], [])
    assert not aw.spectrum.isHidden()
    aw.close()


# ----------------------------------------------------------------------
# picker, click-to-pick, cursors
# ----------------------------------------------------------------------

def test_picker_and_click_to_pick(app, win):
    aw, sels = _window(app, win)
    sg = aw.spectrogram
    assert sg.pick == (1, "wsk:0")
    # the picker's boxes
    sg.sel_box.setCurrentIndex(2)
    sg.sel_box.activated.emit(2)
    sg.var_box.setCurrentIndex(3)
    sg.var_box.activated.emit(3)
    assert sg.pick == (3, "wsk:3")
    assert _pump(app, lambda: sg.sel is sels[2] and sg.var.key == "wsk:3", 2)
    # the picked trace is drawn thick and on top
    assert _pump(app, lambda: aw.compare.picked == (3, "wsk:3"), 2)
    thick = aw.compare.curves[(3, "wsk:3")]
    assert thick.opts["pen"].widthF() > 2 and thick.zValue() > 0
    # a click on another waveform trace only picks it (no cursor); the mark moves
    c = aw.compare.curves[(2, "wsk:1")]
    x, y = c.getData()
    i = len(x) // 3
    vb = aw.compare.rows["mV"].getViewBox()
    ev = _click(vb, x[i], y[i])
    assert ev.accepted and sg.pick == (2, "wsk:1") and aw.cursors == []
    assert c.opts["pen"].widthF() > 2 and thick.opts["pen"].widthF() == 1 and thick.zValue() == 0
    # cursors are the spectrogram's: a click on the picked trace, or away from
    # every trace, adds none (the pick stays)
    _click(vb, x[i], y[i])
    assert aw.cursors == [] and sg.pick == (2, "wsk:1")
    lo, hi = vb.viewRange()[1]
    vb.setYRange(hi + 50 * (hi - lo), hi + 60 * (hi - lo), padding=0)
    _click(vb, 1.0)
    assert sg.pick == (2, "wsk:1") and aw.cursors == []
    # the pick falls back when its variable is unchecked
    aw.set_checked(["wsk:1"], False)
    assert _pump(app, lambda: sg.pick[1] == "wsk:0", 2)
    aw.close()


def test_cursors_add_move_delete(app, win):
    aw, sels = _window(app, win)
    sg, cc = aw.spectrogram, aw.at_cursors
    svb = sg.plot.getViewBox()
    for t in (0.6, 1.5, 2.4):                                   # clicks in the spectrogram
        assert _click(svb, t).accepted
    assert [n for n, _t in aw.cursors] == [1, 2, 3]
    assert _pump(app, lambda: set(cc.curves) == {1, 2, 3}, 2)
    # drawn in every waveform row and in the spectrogram
    assert len(aw._wave_lines.lines) == 3 * len(aw.compare.rows) and len(aw._spec_lines.lines) == 3

    def check():
        ds = aw.dataset()
        direct = ds.spectrogram(sels[0], ds.variables[0], 1.0, 0.75)
        for n, t in aw.cursors:
            f, a, _c = spectrogram_column(direct, t)
            x, y = cc.curves[n].getData()
            assert np.allclose(x, f) and np.allclose(y, a)
    check()
    # drag cursor 2 (in the spectrogram): the waveform's lines and the curve follow
    line = next(ln for (_p, ln) in aw._spec_lines.lines.values() if ln.number == 2)
    line.setValue(1.9)
    line.sigDragged.emit(line)
    assert aw.cursors[1][1] == pytest.approx(1.9)
    assert all(ln.value() == pytest.approx(1.9) for (_p, ln) in aw._wave_lines.lines.values()
               if ln.number == 2)
    assert _pump(app, lambda: cc.columns.get(2, (0, 0, -1))[2] == pytest.approx(
        spectrogram_column(sg.data, 1.9)[2]), 2)
    check()
    # a cursor drag recomputes no spectrum nor spectrogram
    comp = (aw.spectrum.computed, sg.computed, aw.compare.renders)
    line.setValue(2.0)
    line.sigDragged.emit(line)
    _pump(app, lambda: False, 0.2)
    assert (aw.spectrum.computed, sg.computed, aw.compare.renders) == comp
    # right-click deletes; Del deletes the hovered one
    wl = next(ln for (_p, ln) in aw._wave_lines.lines.values() if ln.number == 1)
    wl.mouseClickEvent(_Click(QPointF(), Qt.MouseButton.RightButton))
    assert [n for n, _t in aw.cursors] == [2, 3]
    aw.hover_cursor(3)
    next(a for a in aw.actions() if a.text() == "Delete cursor / selection").trigger()
    assert [n for n, _t in aw.cursors] == [2]
    assert _pump(app, lambda: set(cc.curves) == {2}, 2)
    assert len(aw._wave_lines.lines) == len(aw.compare.rows) and len(aw._spec_lines.lines) == 1
    assert len(win._selections) == 3                                 # no selection deleted
    # a picked tile change redraws the curves from the new tile
    sg.set_pick(2, "wsk:1")
    assert _pump(app, lambda: cc.columns and np.allclose(
        cc.curves[2].getData()[1], spectrogram_column(sg.data, aw.cursors[0][1])[1]), 2)
    aw.close()


# ----------------------------------------------------------------------
# timeline
# ----------------------------------------------------------------------

def test_timeline_variables_independent(app, win):
    aw, sels = _window(app, win)
    tl = aw.timeline
    src = win._replay.source
    has_depth = len(src.depth()[0]) > 0
    # the default: depth when the recording has it, else the first Plots variable
    assert aw.timeline_keys() == (["depth"] if has_depth else ["wsk:0"])
    assert _pump(app, lambda: list(tl.rows) == (["cm"] if has_depth else ["mV"]), 2)
    # the two columns are independent
    aw.set_timeline(["imu:0", "imu:1"])
    assert _pump(app, lambda: list(tl.rows) == ["mg"] and set(tl.curves) == {"imu:0", "imu:1"}, 2)
    assert aw.checked_keys() == ["wsk:0", "wsk:1", "wsk:2", "wsk:3"]
    aw.set_checked(["imu:2"])
    _pump(app, lambda: False, 0.3)
    assert aw.timeline_keys() == ["imu:0", "imu:1"] and "mg" in aw.compare.rows
    # absolute time, the whole recording: the store's envelope
    x0, x1 = tl.x_plot().getViewBox().viewRange()[0]
    ext = win._store.extent("wsk")
    assert x0 <= ext[0] + 0.2 and x1 >= ext[1] - 0.2
    t, y = tl.curves["imu:0"].getData()
    assert t[0] >= x0 - 1 and t[-1] <= x1 + 1 and len(t) < 4 * max(50, int(tl.x_plot().getViewBox().width()))
    # the selections are regions there (the shared item), moved by the model
    ov = tl._overlays["mg"]
    assert len(ov.regions(sels[0])) == 1
    win._selections.set_range(sels[0], 1.5, 4.0)
    assert ov.regions(sels[0])[0].getRegion()[0] == pytest.approx(1.5)
    # Select + drag in the timeline makes a selection, included here
    aw._act_select.setChecked(True)
    _drag(tl.x_plot().getViewBox(), 8.0, 9.0)
    aw._act_select.setChecked(False)
    assert len(win._selections) == 4 and win._selections.selections[-1] in aw.included
    # a single click does not seek; a double-click does
    pos = win._replay.position
    _click(tl.x_plot().getViewBox(), 5.0)
    _pump(app, lambda: False, 0.2)
    assert win._replay.position == pytest.approx(pos, abs=0.3)
    _click(tl.x_plot().getViewBox(), 5.0, double=True)
    assert _pump(app, lambda: abs(win._replay.position - 5.0) < 0.3, 3)
    shown = [h for u, h in tl._heads.items() if u in tl.rows]
    assert _pump(app, lambda: all(h.isVisible() and abs(h.value() - win._replay.position) < 0.05
                                  for h in shown), 2)
    aw.close()


def test_timeline_row_height(app, win):
    from wavi_py.views.analysis_timeline import ROW_MIN
    aw, sels = _window(app, win)
    aw.set_timeline(["depth", "imu:2", "imu:3", "wsk:0"])            # 4 unit rows
    assert _pump(app, lambda: len(aw.timeline.rows) == 4, 2)
    _pump(app, lambda: False, 0.3)
    assert aw.timeline.minimumHeight() >= 4 * ROW_MIN
    assert aw.timeline.height() >= aw.timeline.minimumHeight()
    for p in aw.timeline.rows.values():
        assert p.getViewBox().height() >= ROW_MIN - 12               # room for the y labels
    # a new window's default: room for two rows at least
    aw.reset_state(9)
    assert aw.right.sizes()[0] >= 2 * ROW_MIN
    aw.close()


def test_spectrum_linear_default_and_log(app, win):
    aw, sels = _window(app, win)
    sp = aw.spectrum
    assert not sp.log_a.isChecked()                                  # linear by default
    assert _pump(app, lambda: (1, "wsk:0") in sp.curves, 2)
    f, a = aw.dataset().spectrum(sels[0], aw.dataset().variables[0], *sp.options)
    _, y = sp.curves[(1, "wsk:0")].getData()
    assert np.allclose(y, a)
    sp.log_a.setChecked(True)
    assert _pump(app, lambda: sp.rows["mV"].getAxis("left").logMode, 2)
    _, y = sp.curves[(1, "wsk:0")].getData()
    with np.errstate(divide="ignore"):
        assert np.allclose(y, np.where(a > 0, np.log10(a), np.nan), equal_nan=True)
    assert all(p.getAxis("left").logMode for p in sp.rows.values())
    aw.close()


# ----------------------------------------------------------------------
# keys and Probe
# ----------------------------------------------------------------------

def _key(aw, key, text=""):
    target = aw.compare.glw.viewport()
    ev = QKeyEvent(QEvent.Type.KeyPress, key, Qt.KeyboardModifier.NoModifier, text)
    return aw.eventFilter(target, ev)


def test_keys_and_probe(app, win, monkeypatch):
    aw, sels = _window(app, win, keys=("wsk:0", "imu:0"))
    monkeypatch.setattr(aw, "_cell_at", lambda pos: None)         # no real mouse: the last hovered
    # the waveform: = zooms X in (all its rows and the spectrogram, linked), . Y of all rows
    aw._hover_cell = aw.compare
    x0 = aw.compare.x_plot().getViewBox().viewRange()[0]
    assert _key(aw, Qt.Key.Key_Equal, "=")
    x1 = aw.compare.x_plot().getViewBox().viewRange()[0]
    assert (x1[1] - x1[0]) == pytest.approx((x0[1] - x0[0]) / 1.25, rel=0.02)
    _pump(app, lambda: False, 0.2)
    xs = aw.spectrogram.plot.getViewBox().viewRange()[0]
    assert (xs[1] - xs[0]) == pytest.approx(x1[1] - x1[0], rel=0.05)
    assert not aw.compare._auto                                    # a hand zoom: kept
    ys = {u: p.getViewBox().viewRange()[1] for u, p in aw.compare.rows.items()}
    assert _key(aw, Qt.Key.Key_Period, ".")
    for u, p in aw.compare.rows.items():
        y = p.getViewBox().viewRange()[1]
        assert (y[1] - y[0]) == pytest.approx((ys[u][1] - ys[u][0]) / 1.25, rel=0.02)
    assert _key(aw, Qt.Key.Key_Left)
    x2 = aw.compare.x_plot().getViewBox().viewRange()[0]
    assert x2[0] == pytest.approx(x1[0] + 0.1 * (x1[1] - x1[0]), rel=0.02)
    # the timeline: - zooms its X out; the waveform is left alone
    aw._hover_cell = aw.timeline
    t0 = aw.timeline.x_plot().getViewBox().viewRange()[0]
    assert _key(aw, Qt.Key.Key_Minus, "-")
    t1 = aw.timeline.x_plot().getViewBox().viewRange()[0]
    assert (t1[1] - t1[0]) == pytest.approx((t0[1] - t0[0]) * 1.25, rel=0.02)
    assert aw.compare.x_plot().getViewBox().viewRange()[0] == pytest.approx(x2)
    # the spectrum: X keys act on f
    aw._hover_cell = aw.spectrum
    f0 = aw.spectrum.x_plot().getViewBox().viewRange()[0]
    _key(aw, Qt.Key.Key_Equal, "=")
    f1 = aw.spectrum.x_plot().getViewBox().viewRange()[0]
    assert (f1[1] - f1[0]) < (f0[1] - f0[0])
    # typing into a box is left alone
    ev = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_Minus, Qt.KeyboardModifier.NoModifier, "-")
    assert not aw.eventFilter(aw.spectrum.f_lo, ev)
    # R: reset (all cells, the mouse being outside them)
    aw._reset_view()
    assert aw.compare._auto and not aw.timeline.zoomed
    # Probe: every trace's value at the mouse, marked and labelled next to it
    aw._act_probe.setChecked(True)
    _pump(app, lambda: False, 0.3)
    vb = aw.compare.rows["mV"].getViewBox()
    aw._on_mouse(aw.compare, vb.mapViewToScene(QPointF(1.0, 0.0)))
    h = aw._hovers[aw.compare]
    n = sum(1 for c in aw.compare.curves.values() if c.unit == "mV")
    assert n >= 3 and h.shown[0].startswith("t = 1") and len(h.shown) == 1 + n   # one per trace
    assert all("#" in t and "mV" in t for t in h.shown[1:])
    r = h._rows[id(aw.compare.rows["mV"])]
    assert r.dots.isVisible() and len(r.dots.data) == n and all(t.isVisible() for t in r.labels[:n])
    ys = sorted(t.pos().y() for t in r.labels[:n])                # pushed apart (scene px)
    assert min(np.diff(ys)) >= 0.9 * 13
    assert all(t.parentItem() is None and t.zValue() >= 1e6 for t in r.labels[:n])   # on top
    assert aw.compare.probes["mV"].isVisible() and aw.compare.probes["mV"].value() == pytest.approx(1.0)
    assert not aw.statusBar().currentMessage()
    tvb = aw.timeline.x_plot().getViewBox()
    aw._on_mouse(aw.timeline, tvb.mapViewToScene(QPointF(3.0, tvb.viewRange()[1][0])))
    assert aw._hovers[aw.timeline].shown[0].startswith("t = 3")
    assert not aw.compare.probes["mV"].isVisible() and not r.dots.isVisible()
    svb = aw.spectrum.rows["mV"].getViewBox()
    aw._on_mouse(aw.spectrum, svb.mapViewToScene(QPointF(12.0, 0.0)))
    hs = aw._hovers[aw.spectrum]
    assert hs.shown[0].startswith("f = 12") and any(t.startswith("W1 A #1") for t in hs.shown)
    sg = aw.spectrogram
    sp = sg.plot.getViewBox()
    got = None
    aw._on_mouse(aw.spectrogram, sp.mapViewToScene(QPointF(1.5, 20.0)))
    assert sg.ptext.isVisible() and sg.ptext.textItem.toPlainText().startswith("t = 1.5")
    assert sg.peak_text.isVisible() and sg.peak_text.textItem.toPlainText().startswith("peak ")
    got = sg.probe_at(1.5, 20.0)
    col = spectrogram_column(sg.data, 1.5)
    pos = col[0] > 0
    assert got[1][0] == pytest.approx(col[0][pos][np.argmax(col[1][pos])])
    # hover (Probe off): the nearest trace thick, named at the mouse with its value
    aw._act_probe.setChecked(False)
    c = aw.compare.curves[(1, "wsk:0")]
    x, y = c.getData()
    i = len(x) // 2
    aw._on_mouse(aw.compare, vb.mapViewToScene(QPointF(x[i], y[i])))
    assert h.hovered is not None and h.shown and "mV" in h.shown[0]
    rr = h._rows[id(aw.compare.rows["mV"])]
    assert rr.halo.isVisible() and rr.halo.opts["pen"].widthF() > 3
    # the spectrum: the hovered trace's peaks marked
    c = aw.spectrum.curves[(1, "wsk:0")]
    x, y = c.getData()
    j = int(np.argmax(y[1:])) + 1
    aw._on_mouse(aw.spectrum, svb.mapViewToScene(QPointF(x[j], y[j])))
    assert hs.hovered is c and len(hs.shown) >= 2 and f"{x[j]:.4g} Hz" in hs.shown[1]
    aw._act_probe.setChecked(True)
    aw._act_probe.setChecked(False)
    assert not any(ln.isVisible() for ln in aw._probe_lines())
    aw.close()


# ----------------------------------------------------------------------
# no work while hidden; the cost of a drag
# ----------------------------------------------------------------------

def _counts(aw):
    return (aw.compare.renders, aw.spectrum.renders, aw.spectrum.computed, aw.spectrogram.renders,
            aw.spectrogram.computed, aw.at_cursors.renders, aw.timeline.renders)


def test_no_work_while_hidden(app, win):
    aw, sels = _window(app, win)
    aw.add_cursor(1.0)
    _pump(app, lambda: False, 0.3)
    c = _counts(aw)
    aw._render()                                                   # nothing changed: no work
    assert _counts(aw) == c
    aw.hide()
    win._selections.set_range(sels[0], 1.0, 3.0)
    aw.add_cursor(2.0)
    aw.set_timeline(["imu:0"])
    aw.spectrum.log_f.setChecked(True)
    _pump(app, lambda: False, 0.4)
    assert _counts(aw) == c
    aw.show()
    assert _pump(app, lambda: all(a > b for a, b in zip(_counts(aw)[:2], c[:2]))
                 and aw.timeline.renders > c[-1] and aw.at_cursors.renders > c[5], 2)
    aw.close()


def _drag_cost(app, aw, sel, n=40):
    """Median / max ms of an edge-drag step, render and paint included."""
    ts = []
    t1 = sel.t1
    for i in range(n):
        t = time.perf_counter()
        aw.model.set_range(sel, sel.t0, t1 + 0.02 * ((i % 10) - 5))
        aw._render()
        app.processEvents()
        aw.grab()                                                  # the paint
        ts.append((time.perf_counter() - t) * 1e3)
    return float(np.median(ts)), float(np.max(ts))


def _cursor_cost(app, aw, n=40):
    line = next(ln for (_p, ln) in aw._spec_lines.lines.values() if ln.number == 2)
    base = line.value()
    ts = []
    for i in range(n):
        t = time.perf_counter()
        line.setValue(base + 0.01 * ((i % 10) - 5))
        line.sigDragged.emit(line)
        aw._render()
        app.processEvents()
        aw.grab()
        ts.append((time.perf_counter() - t) * 1e3)
    return float(np.median(ts)), float(np.max(ts))


def test_drag_cost(app, win):
    aw, sels = _window(app, win)                                   # 3 selections x 4 gauges
    for t in (0.6, 1.5, 2.4):
        aw.add_cursor(t)
    _pump(app, lambda: len(aw.at_cursors.curves) == 3, 2)
    sg = aw.spectrogram
    sg.set_pick(2, "wsk:0")                                        # the dragged one shown
    _pump(app, lambda: sg.sel is sels[1], 2)
    med, mx = _drag_cost(app, aw, sels[1])
    cmed, cmx = _cursor_cost(app, aw)
    print(f"\nedge drag: {med:.1f} ms median, {mx:.1f} max; cursor drag: {cmed:.1f} / {cmx:.1f} ms")
    assert med < 80 and cmed < 50
    aw.close()


# ----------------------------------------------------------------------
# screenshots (the 13 min dive with WAVI_ANALYSIS_REC)
# ----------------------------------------------------------------------

def test_screenshots_r2(app, tmp_path):
    if not SHOTS:
        pytest.skip("WAVI_TEST_SHOTS not set")
    rec = os.environ.get("WAVI_ANALYSIS_REC")
    if rec:
        p, spans = Path(rec), [(400.0, 430.0), (520.0, 550.0), (600.0, 615.0)]
        sub, cur = (5.0, 25.0), (9.0, 17.5)
    else:
        p = tmp_path / "rec.jsonl"
        fx.write_recording(p)
        spans, sub, cur = [(1.0, 3.0), (4.0, 6.0), (6.5, 7.5)], (0.3, 1.5), (0.6, 1.2)
    w = _open(app, p)
    try:
        assert _pump(app, lambda: bool(np.any(w._v0)), 5)
        _pump(app, lambda: False, 0.5)
        m = w._selections
        sels = [m.create(*s) for s in spans]
        keys = ["wsk:0", "wsk:1", "wsk:6", "imu:2"]
        aw = w.new_analysis_window(sels[0], keys)
        for s in sels[1:]:
            aw.include(s)
        aw.resize(1600, 1000)
        aw.default_sizes()
        aw.set_timeline(["depth", "imu:2"])
        _pump(app, lambda: False, 1.0)
        aw.set_sub(sub)
        aw.spectrogram.db_box.setChecked(True)
        aw.spectrogram.set_pick(2, "wsk:6")
        for t in cur:
            aw.add_cursor(t)
        _pump(app, lambda: False, 1.2)
        _shot(aw, "r2_window")
        # one right cell collapsed (its button): the spectrogram takes the row's width
        g = aw.grid
        aw.at_cursors.header.collapse_btn.click()
        _pump(app, lambda: False, 0.8)
        _shot(aw, "r2_right_cell_collapsed")
        g.restore(aw.at_cursors)
        # a whole row collapsed: the waveform and spectrum take the height
        aw.spectrogram.header.collapse_btn.click()
        aw.at_cursors.header.collapse_btn.click()
        _pump(app, lambda: False, 0.8)
        _shot(aw, "r2_row_collapsed")
        g.restore(aw.spectrogram)
        g.restore(aw.at_cursors)
        _pump(app, lambda: False, 0.3)
        _shot(aw.centralWidget().widget(0), "r2_variables_tree")
        # the measured drag on real data: 3 selections x 4 gauges, 3 cursors
        aw.set_checked(keys, False)
        aw.set_checked(["wsk:0", "wsk:1", "wsk:6", "wsk:7"])
        aw.add_cursor(cur[1] + 3.0)
        _pump(app, lambda: False, 1.0)
        aw.spectrogram.set_pick(2, "wsk:0")
        _pump(app, lambda: False, 0.5)
        med, mx = _drag_cost(app, aw, sels[1])
        cmed, cmx = _cursor_cost(app, aw)
        print(f"\n[dive] edge drag: {med:.1f} ms median, {mx:.1f} max; cursor drag: "
              f"{cmed:.1f} / {cmx:.1f} ms")
    finally:
        w.close()
