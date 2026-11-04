"""Round 2 of the selections: a drag on a region's body moves the whole selection
(length kept, clamped, everywhere, the Analysis windows follow), a drag elsewhere
still pans, the cursors, the newest region on top takes the drag; and the Line
window's view span on every timeline strip. Real mouse events go to the plot's
viewport, so pyqtgraph's own hover / drag routing is what is tested.
Screenshots go to $WAVI_TEST_SHOTS if set; $WAVI_ANALYSIS_REC replays a real
recording (the 13 min dive) instead of the synthetic one for the screenshot test.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_regions.py
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
from PyQt6.QtCore import QEvent, QPoint, QPointF, Qt  # noqa: E402
from PyQt6.QtGui import QMouseEvent  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.tests import replay_fixture as fx  # noqa: E402

SHOTS = os.environ.get("WAVI_TEST_SHOTS")
L = Qt.MouseButton.LeftButton


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


# ----------------------------------------------------------------------
# real mouse events on a plot
# ----------------------------------------------------------------------

class _Mouse:
    """Mouse events at (time, middle of the height) in ``plot``'s ViewBox, sent to
    its GraphicsView's viewport (pyqtgraph routes hover / drags from these)."""

    def __init__(self, app, plot):
        self.app, self.vb = app, plot.getViewBox()
        self.view = plot.scene().views()[0]
        self.vp = self.view.viewport()

    def at(self, t: float) -> QPoint:
        y = self.vb.viewRange()[1]
        sp = self.vb.mapViewToScene(QPointF(t, 0.5 * (y[0] + y[1])))
        return self.view.mapFromScene(sp)

    def _send(self, kind, t, button, buttons) -> None:
        if kind == QEvent.Type.MouseMove:
            time.sleep(0.02)                  # pyqtgraph drops moves closer than its rate limit
        p = QPointF(self.at(t))
        g = QPointF(self.vp.mapToGlobal(p.toPoint()))
        QApplication.sendEvent(self.vp, QMouseEvent(kind, p, g, button, buttons,
                                                    Qt.KeyboardModifier.NoModifier))
        self.app.processEvents()

    def hover(self, t: float) -> None:
        self._send(QEvent.Type.MouseMove, t, Qt.MouseButton.NoButton, Qt.MouseButton.NoButton)

    def press(self, t: float) -> None:
        self.hover(t)
        self._send(QEvent.Type.MouseButtonPress, t, L, L)

    def move(self, t: float) -> None:
        self._send(QEvent.Type.MouseMove, t, Qt.MouseButton.NoButton, L)

    def release(self, t: float) -> None:
        self._send(QEvent.Type.MouseButtonRelease, t, L, Qt.MouseButton.NoButton)

    def drag(self, t0: float, t1: float, steps: int = 6, during=None) -> None:
        self.press(t0)
        for i in range(1, steps + 1):
            self.move(t0 + (t1 - t0) * i / steps)
            if during is not None and i == steps // 2:
                during()
        self.release(t1)


def _line_tile(app, w):
    w._toggle_line.setChecked(True)
    lv = w._line_view
    lv.resize(1200, 800)
    _pump(app, lambda: False, 0.3)
    lv.set_time_window(0.0, 8.0)                      # still: not following the playhead
    _pump(app, lambda: False, 0.2)
    return lv, lv._tiles[2]


def test_body_drag_moves_and_clamps(app, win):
    w, m = win, win._selections
    w._toggle_imu.setChecked(True)
    lv, tile = _line_tile(app, w)
    sel = m.create(2.0, 3.0)
    ov_line, ov_imu, ov_strip = w._sel_overlays
    aw = w.new_analysis_window(sel, ["wsk:4"])
    _pump(app, lambda: bool(aw.compare.curves), 2)
    x_before = tile.getViewBox().viewRange()[0]
    mouse = _Mouse(app, tile)
    mouse.drag(2.5, 4.0)                              # the body: +1.5 s
    assert abs(sel.t0 - 3.5) < 0.02 and abs(sel.t1 - 4.5) < 0.02
    assert abs(sel.duration - 1.0) < 1e-9             # length kept exactly
    assert np.allclose(tile.getViewBox().viewRange()[0], x_before)   # no pan
    # everywhere: every Line plot, IMU row, timeline strip, and the Analysis window
    for r in ov_line.regions(sel) + ov_imu.regions(sel) + ov_strip.regions(sel):
        assert np.allclose(r.getRegion(), (sel.t0, sel.t1))
    _pump(app, lambda: False, 0.2)
    box = aw.sel_list.itemWidget(aw.sel_list.topLevelItem(0), 1)
    assert abs(box.value() - sel.t0) < 1e-3              # (the box shows 3 decimals)
    # clamped to the source's extent, length kept
    lo, hi = sel.bounds
    m.move(sel, hi + 100)
    assert sel.t1 == hi and abs(sel.duration - 1.0) < 1e-9
    m.move(sel, lo - 100)
    assert sel.t0 == lo and abs(sel.duration - 1.0) < 1e-9
    # a body drag past the start in the IMU window stops at the start, length kept
    m.set_range(sel, 2.0, 3.0)
    imu_mouse = _Mouse(app, w._imu_view._plots[0])
    imu_mouse.drag(2.5, 0.2)
    assert abs(sel.t0 - lo) < 1e-9 and abs(sel.duration - 1.0) < 1e-9
    aw.close()


def test_drag_elsewhere_pans_and_edges(app, win):
    w, m = win, win._selections
    lv, tile = _line_tile(app, w)
    sel = m.create(2.0, 3.0)
    mouse = _Mouse(app, tile)
    x_before = tile.getViewBox().viewRange()[0]
    mouse.drag(6.0, 5.0)                              # empty plot: pans (traces follow)
    x_after = tile.getViewBox().viewRange()[0]
    assert abs(x_after[0] - x_before[0]) > 0.5 and (sel.t0, sel.t1) == (2.0, 3.0)
    lv.set_time_window(0.0, 8.0)
    _pump(app, lambda: False, 0.1)
    # an edge drag moves only that edge
    mouse.drag(3.0, 3.6)
    assert sel.t0 == 2.0 and abs(sel.t1 - 3.6) < 0.02
    # overlapping: the newest (on top) takes a drag over both bodies
    new = m.create(2.4, 3.2)
    mouse.drag(2.8, 3.3)
    assert abs(new.t0 - 2.9) < 0.02 and abs(new.duration - 0.8) < 1e-9
    assert sel.t0 == 2.0 and abs(sel.t1 - 3.6) < 0.02
    # Select on: a drag on a body makes a new (overlapping) selection, no move
    w._line_select.setChecked(True)
    mouse.drag(2.2, 2.7)
    assert len(m) == 3 and sel.t0 == 2.0
    third = m.selections[-1]
    assert abs(third.t0 - 2.2) < 0.02 and abs(third.t1 - 2.7) < 0.02
    w._line_select.setChecked(False)


def test_cursors(app, win):
    w, m = win, win._selections
    lv, tile = _line_tile(app, w)
    sel = m.create(2.0, 3.0)
    r = next(x for x in w._sel_overlays[0].regions(sel) if x.plot is tile)
    assert all(ln.cursor().shape() == Qt.CursorShape.SizeHorCursor for ln in r.lines)
    mouse = _Mouse(app, tile)
    mouse.hover(2.5)
    assert m.hovered is sel and r.cursor().shape() == Qt.CursorShape.OpenHandCursor
    seen = []
    mouse.drag(2.5, 3.5, during=lambda: seen.append(r.cursor().shape()))
    assert seen == [Qt.CursorShape.ClosedHandCursor]
    assert r.cursor().shape() == Qt.CursorShape.OpenHandCursor
    # the cursor the viewport shows is the topmost item's: the edge's over an edge
    mouse.hover(sel.t1)
    assert mouse.vp.cursor().shape() == Qt.CursorShape.SizeHorCursor
    mouse.hover(sel.t0 + 0.5 * sel.duration)
    assert mouse.vp.cursor().shape() == Qt.CursorShape.OpenHandCursor
    # Select on: the body has no hand (a drag there makes a selection)
    w._line_select.setChecked(True)
    mouse.hover(sel.t0 + 0.4 * sel.duration)
    assert not r.hasCursor()
    w._line_select.setChecked(False)


def test_view_span_on_strips(app, win):
    w = win
    bars = [d.widget() for d in w._timeline_docks]
    assert all(not b.strip.view_span.isVisible() for b in bars)   # Line window closed
    assert w._replay.view_span is None
    w._toggle_line.setChecked(True)
    lv = w._line_view
    _pump(app, lambda: False, 0.3)
    span = w._replay.view_span
    assert span is not None and np.allclose(span, lv.time_window())

    def shown():
        return [b.strip.view_span.getRegion() + (b.strip.view_edge.value(),) for b in bars]
    for a, b_, e in shown():
        assert np.allclose((a, b_), span) and e == pytest.approx(span[1])
    # panned away: follows the Line window's X
    lv.set_time_window(1.0, 4.0)
    _pump(app, lambda: False, 0.1)
    assert all(np.allclose(s, (1.0, 4.0, 4.0)) for s in shown())
    # following the playhead: the right edge is the playhead
    w._replay.seek(6.0)
    assert _pump(app, lambda: abs(w._replay.view_span[1] - w._replay.position) < 0.05
                 and w._replay.position >= 5.9, 3)
    # the band takes no mouse (the playhead and the strip's click / pan do)
    strip = bars[0].strip
    assert strip.view_span.acceptedMouseButtons() == Qt.MouseButton.NoButton
    assert strip.view_edge.acceptedMouseButtons() == Qt.MouseButton.NoButton
    assert strip.view_edge.zValue() < strip.head.zValue()
    # closed: hidden on every strip; back when it opens again
    lv.close()
    _pump(app, lambda: False, 0.1)
    assert w._replay.view_span is None
    assert all(not b.strip.view_span.isVisible() and not b.strip.view_edge.isVisible() for b in bars)
    w._toggle_line.setChecked(True)
    _pump(app, lambda: False, 0.2)
    assert w._replay.view_span is not None
    w._toggle_line.setChecked(False)
    _pump(app, lambda: False, 0.1)
    assert w._replay.view_span is None


def test_view_span_live(app, tmp_path):
    from wavi_py.main_window import MainWindow
    w = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    w.show()
    w._connect()
    try:
        w._toggle_line.setChecked(True)
        assert _pump(app, lambda: w._samples_since_clear > 300, 10)
        # Sim has no events / depth: a live timeline as the Server transport's would be
        from wavi_py.daq.series import WSK
        from wavi_py.replay.events import EventTracker, LiveTimeline
        w._replay.set_live(LiveTimeline(EventTracker(), lambda: 0.0,
                                        lambda: w.series().extent(WSK)[1], lambda: None))
        r0 = w._replay.view_span
        assert r0 is not None
        assert _pump(app, lambda: w._replay.view_span[1] > r0[1] + 0.3, 5)   # moves live
        bar = w._timeline_docks[0].widget()
        assert bar.strip.view_span.isVisible()
        assert np.allclose(bar.strip.view_span.getRegion(), w._replay.view_span)
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
        p, spans, head, away = Path(rec), [(400.0, 430.0), (520.0, 550.0)], 480.0, (250.0, 330.0)
    else:
        p = tmp_path / "rec.jsonl"
        fx.write_recording(p)
        spans, head, away = [(1.0, 3.0), (4.0, 6.0)], 6.0, (1.0, 3.0)
    w = _open(app, p)
    try:
        assert _pump(app, lambda: bool(np.any(w._v0)), 5)
        w._toggle_line.setChecked(True)
        lv = w._line_view
        lv.resize(1400, 850)
        _pump(app, lambda: False, 0.3)
        m = w._selections
        for t0, t1 in spans:
            m.create(t0, t1)
        lv.timeline_dock.show()                           # the Line window's (wide) strip
        # following: the band's right edge is the playhead
        w._replay.seek(head)
        _pump(app, lambda: False, 1.0)
        _shot(lv.timeline_dock.widget(), "regions_strip_following")
        w.resize(w.minimumSizeHint().width(), w.height())
        _pump(app, lambda: False, 0.3)
        _shot(w, "regions_panel_following")
        lv.set_time_window(*away)                         # panned away
        _pump(app, lambda: False, 0.5)
        _shot(lv.timeline_dock.widget(), "regions_strip_panned")
        # mid body-drag in the Line window
        a, b = spans[0]
        lv.set_time_window(a - 0.5 * (b - a), b + 1.5 * (b - a))
        _pump(app, lambda: False, 0.5)
        mouse = _Mouse(app, lv._tiles[0])
        mid = 0.5 * (a + b)
        mouse.press(mid)
        for i in range(1, 5):
            mouse.move(mid + 0.25 * (b - a) * i / 4)
        _pump(app, lambda: False, 0.4)
        _shot(lv, "regions_line_body_drag")
        mouse.release(mid + 0.25 * (b - a))
        assert abs(m.selections[0].t0 - (a + 0.25 * (b - a))) < 0.02 * (b - a)
    finally:
        w.close()
