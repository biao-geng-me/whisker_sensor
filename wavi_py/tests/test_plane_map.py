"""The Analysis window's plane map (daq/plane_overlay.py, the Plots grid's map
mode in views/sensor_grid.py): on by default in either view; every position at
the plane head (a line on the timeline, absolute time, from the recording) with
its trail; each included selection's path in degrees (the sensor's calibration,
polarity, origin at the selection's start without V0), numbered arrows at the
cursors; the one scale; the side's forward; the chips toggling the variables;
Events' relative head in the waveform. Also the Zoom box cursor, F and the
colour default.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_plane_map.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtCore import Qt  # noqa: E402
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.daq.analysis import Dataset, Selection, position_variables  # noqa: E402
from wavi_py.daq.plane_overlay import (HEAD, ORIGIN_S, TRAIL_S, deg_per_mv,  # noqa: E402
                                       heads, plane_overlay, set_arrows)
from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.tests.test_analysis import _open, _pump, _shot  # noqa: E402


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
    fx.write_recording(p, rov=True)
    w = _open(app, p, t_buffer_s=2.0)
    from whisker_link.dome_map import DEFAULT_MAP, DomeMap
    w.set_dome_map(DomeMap.load(DEFAULT_MAP))
    yield w
    w.close()


def _positions(g):
    return [n for s, n in enumerate(g.grid.names) if g.position_of(2 * s)]


def test_plane_map_head_and_paths(app, win):
    from PyQt6.QtTest import QTest

    from wavi_py.tests.test_analysis_r2 import _window
    from wavi_py.views.analysis_window import GAUGES_VIEW, PLOTS, POSITIONS_VIEW
    aw, sels = _window(app, win, n_sel=2, keys=("wsk:2",))
    g = aw.grids[PLOTS]
    # on by default (D / L view by default), in the gauge view too (the same cells, A / B chips)
    assert aw.grid_view.currentText() == POSITIONS_VIEW
    aw.grid_view.setCurrentText(GAUGES_VIEW)
    assert aw.plane_box.isChecked() and g.map
    assert _pump(app, lambda: g.overlay is not None and bool(g.overlay.head), 5)
    ov = g.overlay
    positions = _positions(g)
    assert len(positions) == 12
    # the head: a line in every waveform row (relative), each selection at it, overlaid
    t = aw.plane_t
    assert t is not None and aw._head_lines
    assert all(line.value() == pytest.approx(t) for _r, line in aw._head_lines.values())
    assert len(ov.head) >= 6 and set(ov.head) <= set(positions)
    p = next(iter(ov.head))
    assert {a.sel.number for a in ov.head[p]} == {s.number for s in sels}
    m = aw.model
    vd, vl = position_variables([p])
    for a in ov.head[p]:
        sel = a.sel
        ds = Dataset([sel], [], despike=m.despike, v0=m.v0, scale=m.scale, dome_map=aw._position_map())
        at = sel.zero + t
        _t, d = ds.samples(sel, vd, (at - TRAIL_S, at + 1e-6))
        _t, lf = ds.samples(sel, vl, (at - TRAIL_S, at + 1e-6))
        o = np.zeros(2)
        if m.v0 is None:                       # the selection's own origin, as its path
            h = (sel.t0, sel.t0 + ORIGIN_S)
            o = np.array([np.median(ds.samples(sel, vd, h)[1]), np.median(ds.samples(sel, vl, h)[1])])
        want = (np.array([d[-1], lf[-1]]) - o) @ (deg_per_mv(ds.map_of(sel), p) / m.scale).T
        assert np.allclose(a.xy, want)
    # independent of the variables: unchecking every Plots variable keeps it
    aw.set_checked(["wsk:2"], False)
    _pump(app, lambda: False, 0.3)
    assert g.overlay.head and g.map
    aw.set_checked(["wsk:2"])
    _pump(app, lambda: False, 0.3)
    # dragging the head in the waveform re-reads only the head; past a selection's end too
    _r, line = next(iter(aw._head_lines.values()))
    paths = id(g.overlay.paths)
    before = [a.xy for a in g.overlay.head[p]]
    end = max(s.duration for s in sels) + 0.5
    line.setValue(end)
    line.sigDragged.emit(line)
    assert aw.plane_t == pytest.approx(end) and id(g.overlay.paths) == paths
    assert all(ln.value() == pytest.approx(end) for _r, ln in aw._head_lines.values())
    assert len(g.overlay.head[p]) == len(sels) and [a.xy for a in g.overlay.head[p]] != before
    assert all(len(tr) > 5 for _s, tr in g.overlay.trail[p])   # each one's last second
    # the head's arrow ends on the path inside the span
    aw.set_plane_t(1.0)
    path = g.overlay.paths[p][0]
    a = next(a for a in g.overlay.head[p] if a.sel is path.sel)
    i = int(np.searchsorted(path.t, path.sel.zero + 1.0 + 1e-6)) - 1   # the last sample at or before
    assert np.allclose(a.xy, path.deg[i], atol=1e-6)
    # the selections' paths, no peak arrows; a cursor adds a numbered arrow per path
    assert {pp.sel.number for pp in g.overlay.paths[p]} == {s.number for s in sels}
    assert not any(g.overlay.arrows.get(q) for q in positions)
    aw.add_cursor(1.0)
    assert _pump(app, lambda: any(a.label == "1" for a in g.overlay.arrows.get(p, [])), 3)
    path = g.overlay.paths[p][0]
    a = next(a for a in g.overlay.arrows[p] if a.sel is path.sel)
    assert np.allclose(a.xy, path.deg[int(np.searchsorted(path.t, path.sel.zero + 1.0))])
    # the path numbers: (mV - the median of the first ORIGIN_S, or V0) through the matrix
    pds = Dataset([path.sel], [], despike=m.despike, v0=m.v0, scale=m.scale,
                  dome_map=aw._position_map())
    t2, d2 = pds.samples(path.sel, vd)
    _t, l2 = pds.samples(path.sel, vl)
    head = t2 <= path.sel.t0 + ORIGIN_S
    o2 = np.array([np.median(d2[head]), np.median(l2[head])]) if m.v0 is None else np.zeros(2)
    w2 = (np.c_[d2, l2] - o2) @ deg_per_mv(aw._position_map(), p).T / m.scale
    assert np.allclose(path.deg, w2[np.isfinite(w2).all(axis=1)])
    # port forward is screen right, starboard left (as the dome's plane map)
    s_l = g.grid.names.index("LFM")
    s_r = g.grid.names.index("RFM")
    assert g.grid.fwd[s_l] == 1.0 and g.grid.fwd[s_r] == -1.0
    # gauge view: the chips are A / B and toggle gauges; the plot toggles both
    g.grab()
    _cell, _name, achip, _bchip, plot = g.map_rects()[s_l]
    QTest.mouseClick(g, Qt.MouseButton.LeftButton, pos=achip.center().toPoint())
    app.processEvents()
    assert f"wsk:{2 * s_l}" in aw.checked_keys() and f"wsk:{2 * s_l + 1}" not in aw.checked_keys()
    _pump(app, lambda: False, 0.3)
    _shot(aw, "plane_map_gauges")
    # position view: the same map, D / L chips
    aw.grid_view.setCurrentText(POSITIONS_VIEW)
    assert g.map and aw.plane_box.isChecked()
    g.grab()
    _cell, _name, dchip, _lchip, plot = g.map_rects()[s_l]
    QTest.mouseClick(g, Qt.MouseButton.LeftButton, pos=dchip.center().toPoint())
    app.processEvents()
    assert "pos:LFM:drag" in aw.checked_keys() and "pos:LFM:lift" not in aw.checked_keys()
    QTest.mouseClick(g, Qt.MouseButton.LeftButton, pos=plot.center().toPoint())
    app.processEvents()
    assert {"pos:LFM:drag", "pos:LFM:lift"} <= set(aw.checked_keys())
    _pump(app, lambda: False, 0.3)
    _shot(aw, "plane_map")
    # off: the compact picker, no head line
    h_map = g.height()
    aw.plane_box.setChecked(False)
    assert not g.map and g.height() < h_map and not aw._head_lines
    aw.close()


def test_plane_overlay_sub_ranges_and_cache(app, win):
    from wavi_py.tests.test_analysis_r2 import _window
    from wavi_py.views.analysis_window import PLOTS
    aw, sels = _window(app, win, n_sel=1, keys=("wsk:2",))
    aw.add_sub(0.5, 1.5)
    aw.add_sub(2.0, 2.8)
    g = aw.grids[PLOTS]
    assert _pump(app, lambda: g.overlay is not None and len(next(iter(g.overlay.paths.values()), [])) == 2, 5)
    p = next(iter(g.overlay.paths))
    paths = g.overlay.paths[p]
    assert [pp.sub for pp in paths] == [1, 2]
    z = sels[0].zero
    assert paths[0].t[0] >= z + 0.5 - 0.05 and paths[0].t[-1] <= z + 1.5 + 0.05
    assert aw._plane_pen([], paths[1]).style() == Qt.PenStyle.DashLine
    aw.close()
    # the cache: a second selection read once; the first one reused
    ds = Dataset(sels, [], dome_map=aw._position_map())
    cache: dict = {}
    pos = ["LFM", "RFM"]
    a = plane_overlay(ds, pos, cache=cache)
    b = plane_overlay(ds, pos, cache=cache)
    assert b.paths["LFM"][0] is a.paths["LFM"][0]
    set_arrows(b, [], None, peaks=False)
    assert not any(b.arrows.values())
    set_arrows(b, [], None)
    assert all(len(b.arrows[q]) == len(b.paths[q]) for q in b.paths)
    hc: dict = {}
    st = heads(ds, pos, 1.0, hc)
    assert set(st.head) <= set(pos) and st.big >= 0 and len(hc) == len(sels) + 1
    assert heads(ds, pos, 1.0, hc) is st                          # nothing changed: the same
    sels[0].t1 += 0.1                                             # an end drag: not re-read
    assert heads(ds, pos, 1.0, hc) is st
    sels[0].t0 += 0.1                                             # a start drag moves its zero
    assert heads(ds, pos, 1.0, hc) is not st
    assert all(len(st.head[q]) == len(sels) for q in st.head)


def test_zoom_cursor_fit_key_and_colour_default(app, win):
    from PyQt6.QtCore import QEvent
    from PyQt6.QtGui import QKeyEvent

    from wavi_py.tests.test_analysis_r2 import _window
    from wavi_py.views.analysis_cells import SENSOR, zoom_cursor
    aw, _sels = _window(app, win, n_sel=1, keys=("wsk:2",))
    assert aw.color_by == SENSOR and aw.compare.color_by == SENSOR
    aw._act_zoom.setChecked(True)
    for cell in (aw.compare, aw.spectrum, aw.spectrogram, aw.at_cursors):
        assert cell.glw.viewport().cursor().pixmap().cacheKey() == zoom_cursor().pixmap().cacheKey()
    aw._act_zoom.setChecked(False)
    assert aw.compare.glw.viewport().cursor().shape() == Qt.CursorShape.ArrowCursor
    row = next(iter(aw.compare.rows.values()))
    vb = row.getViewBox()
    vb.setRange(xRange=(0.5, 0.6), yRange=(100, 101), padding=0)
    aw._target = lambda: (aw.compare, row)
    ev = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_F, Qt.KeyboardModifier.NoModifier, "f")
    assert aw.eventFilter(aw.compare.glw.viewport(), ev)
    (x0, x1), (y0, y1) = vb.viewRange()
    assert x0 < 0.1 and x1 > 2.5 and y1 - y0 < 50
    aw.close()


def test_right_click_solo(app, win):
    from PyQt6.QtTest import QTest

    from wavi_py.tests.test_analysis_r2 import _window
    from wavi_py.views.analysis_window import PLOTS, POSITIONS_VIEW
    aw, _sels = _window(app, win, n_sel=1, keys=("wsk:2",))
    aw.grid_view.setCurrentText(POSITIONS_VIEW)
    g = aw.grids[PLOTS]
    start = ["pos:LFM:drag", "pos:LFM:lift", "pos:RFM:drag", "pos:LBU:lift"]
    aw.set_checked(start)
    g.grab()
    rects = g.map_rects()
    s_l, s_r = g.grid.names.index("LFM"), g.grid.names.index("RFM")
    right = Qt.MouseButton.RightButton

    def keys():
        return {k for k in aw.checked_keys() if k.startswith("pos:")}
    # right-click LFM's drag chip: that alone
    QTest.mouseClick(g, right, pos=rects[s_l][2].center().toPoint())
    app.processEvents()
    assert keys() == {"pos:LFM:drag"}
    assert "wsk:2" in aw.checked_keys()                # the other view's checks stay
    # another target meanwhile (RFM's plot, both halves): solos it, keeps the hidden
    QTest.mouseClick(g, right, pos=rects[s_r][4].center().toPoint())
    app.processEvents()
    assert keys() == {"pos:RFM:drag", "pos:RFM:lift"}
    # the same again: everything hidden comes back (RFM lift stays on)
    QTest.mouseClick(g, right, pos=rects[s_r][4].center().toPoint())
    app.processEvents()
    assert keys() == set(start) | {"pos:RFM:lift"}
    aw.close()
