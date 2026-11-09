"""Drag tools (``views/plot_tools.py``): Select and Zoom box are exclusive in the
Analysis, Line and IMU windows, the key again goes back to Pan, every plot shows
the armed tool's cursor where the tool acts (the arrow elsewhere), and a drag
right after a switch does the new tool's thing (a Z after S zooms, not selects).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_plot_tools.py
"""

from __future__ import annotations

import pyqtgraph as pg
from PyQt6.QtCore import QPoint, Qt
from PyQt6.QtTest import QTest

from wavi_py.tests.test_analysis_r2 import _pump, _window, app, no_modal_errors, win  # noqa: F401
from wavi_py.views.plot_tools import PAN, SELECT, ZOOM, zoom_cursor

CROSS = Qt.CursorShape.CrossCursor


def _cursor(view):
    """'zoom', 'cross' or 'arrow' (unset) of a GraphicsView's viewport."""
    vp = view.viewport()
    if not vp.testAttribute(Qt.WidgetAttribute.WA_SetCursor):
        return "arrow"
    c = vp.cursor()
    if c.shape() == Qt.CursorShape.BitmapCursor:
        return "zoom" if c.pixmap().cacheKey() == zoom_cursor().pixmap().cacheKey() else "bitmap"
    return "cross" if c.shape() == CROSS else str(c.shape())


def _modes(vbs):
    return {vb.state["mouseMode"] for vb in vbs}


def _key(widget, k):
    QTest.keyClick(widget, k)


def test_analysis_exclusive_and_cursors(app, win):
    aw, _sels = _window(app, win)
    aw.show()
    _pump(app, lambda: False, 0.3)
    vp = aw.compare.glw.viewport()
    wave_vbs = [p.getViewBox() for p in aw.compare.rows.values()]
    spec_vbs = [p.getViewBox() for p in aw.spectrum.rows.values()]

    _key(vp, Qt.Key.Key_Z)
    assert aw.tools.tool == ZOOM and aw._act_zoom.isChecked() and not aw._act_select.isChecked()
    assert _modes(wave_vbs) == _modes(spec_vbs) == {pg.ViewBox.RectMode}
    assert _cursor(aw.compare.glw) == _cursor(aw.spectrum.glw) == _cursor(aw.timeline.glw) == "zoom"

    _key(vp, Qt.Key.Key_S)                                       # S while Z is armed: Z goes
    assert aw.tools.tool == SELECT and aw._act_select.isChecked() and not aw._act_zoom.isChecked()
    assert _modes(wave_vbs) == _modes(spec_vbs) == {pg.ViewBox.PanMode}
    assert _cursor(aw.compare.glw) == _cursor(aw.timeline.glw) == "cross"
    assert _cursor(aw.spectrum.glw) == _cursor(aw.spectrogram.glw) == "arrow"   # no Select there

    _key(vp, Qt.Key.Key_Z)                                       # and back: S goes
    assert aw.tools.tool == ZOOM and not aw._act_select.isChecked()
    _key(vp, Qt.Key.Key_Z)                                       # Z again: pan
    assert aw.tools.tool == PAN and not aw._act_zoom.isChecked()
    assert all(_cursor(c.glw) == "arrow" for c in aw.cells())
    assert _modes(wave_vbs) == {pg.ViewBox.PanMode}
    aw.close()


def _drag(vp, a, b):
    QTest.mousePress(vp, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier, a)
    for i in range(1, 11):
        QTest.mouseMove(vp, a + (b - a) * (i / 10))
    QTest.mouseRelease(vp, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier, b)


def test_analysis_drag_follows_the_last_key(app, win):
    """S then Z: the drag zooms (it used to make a sub-range: Select won)."""
    aw, _sels = _window(app, win)
    aw.show()
    _pump(app, lambda: False, 0.3)
    c = aw.compare
    vp = c.glw.viewport()
    vb = next(iter(c.rows.values())).getViewBox()
    r = vb.sceneBoundingRect()
    a = c.glw.mapFromScene(r.left() + 0.3 * r.width(), r.top() + 0.4 * r.height())
    b = c.glw.mapFromScene(r.left() + 0.6 * r.width(), r.top() + 0.7 * r.height())
    _key(vp, Qt.Key.Key_S)
    _key(vp, Qt.Key.Key_Z)
    x0 = vb.viewRange()[0]
    _drag(vp, a, b)
    _pump(app, lambda: False, 0.2)
    assert not c.subs and vb.viewRange()[0][1] - vb.viewRange()[0][0] < 0.6 * (x0[1] - x0[0])
    _key(vp, Qt.Key.Key_S)                                       # now Select: a sub-range
    _drag(vp, a + QPoint(0, 5), b)
    _pump(app, lambda: False, 0.2)
    assert len(c.subs) == 1
    aw.close()


def test_line_and_imu_exclusive(app, win):
    for view, sel in ((win._line_view, win._line_select), (win._imu_view, win._imu_select)):
        view.show()
        _pump(app, lambda: False, 0.2)
        zoom = view.tools.actions[ZOOM]
        assert sel is view.tools.actions[SELECT]
        vbs = [vb for target in view.tools._targets
               for vb in (target[0]() if callable(target[0]) else target[0])]
        views = [v for target in view.tools._targets
                 for v in (target[1]() if callable(target[1]) else target[1])]
        zoom.trigger()
        assert zoom.isChecked() and not sel.isChecked()
        assert _modes(vbs) == {pg.ViewBox.RectMode} and {_cursor(v) for v in views} == {"zoom"}
        sel.trigger()
        assert sel.isChecked() and not zoom.isChecked()
        assert _modes(vbs) == {pg.ViewBox.PanMode} and {_cursor(v) for v in views} == {"cross"}
        sel.trigger()
        assert view.tools.tool == PAN and {_cursor(v) for v in views} == {"arrow"}
        view.hide()


def test_timeline_zoom_box_time_only(app, win):
    aw, _sels = _window(app, win)
    aw.show()
    _pump(app, lambda: False, 0.3)
    tl = aw.timeline
    p = tl.x_plot()
    vb = p.getViewBox()
    vp = tl.glw.viewport()
    _key(vp, Qt.Key.Key_Z)
    assert vb.state["mouseMode"] == pg.ViewBox.RectMode
    x0, y0 = vb.viewRange()
    r = vb.sceneBoundingRect()
    a = tl.glw.mapFromScene(r.left() + 0.2 * r.width(), r.top() + 0.3 * r.height())
    b = tl.glw.mapFromScene(r.left() + 0.5 * r.width(), r.top() + 0.6 * r.height())
    _drag(vp, a, b)
    _pump(app, lambda: False, 0.2)
    x1, y1 = vb.viewRange()
    assert x1[1] - x1[0] < 0.4 * (x0[1] - x0[0]) and tl.zoomed
    assert y1[1] - y1[0] > 0.6 * (y0[1] - y0[0])                 # Y not boxed (it auto-fits)
    assert not aw.compare.subs and len(aw.model) == 3             # no selection made
    aw.close()
