"""The Analysis window's plane map (daq/plane_overlay.py, the Plots grid's map
mode in views/sensor_grid.py): the paths per position and selection in degrees
(the sensor's calibration, polarity, origin at the selection's start without V0),
the arrows at the cursors or the peaks, the one scale, the side's forward, the
chips toggling the variables, and the gauge view turning it off.

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

from wavi_py.daq.analysis import Dataset, position_variables  # noqa: E402
from wavi_py.daq.plane_overlay import HEAD, ORIGIN_S, deg_per_mv, plane_overlay, set_arrows  # noqa: E402
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


def test_plane_map(app, win):
    from PyQt6.QtTest import QTest

    from wavi_py.tests.test_analysis_r2 import _window
    from wavi_py.views.analysis_window import GAUGES_VIEW, PLOTS, POSITIONS_VIEW
    aw, sels = _window(app, win, n_sel=2, keys=("wsk:2",))
    g = aw.grids[PLOTS]
    h0 = g.height()
    aw.plane_box.setChecked(True)
    assert aw.grid_view.currentText() == POSITIONS_VIEW and g.map and g.height() > h0 + 60
    assert _pump(app, lambda: g.overlay is not None and bool(g.overlay.paths), 5)
    ov = g.overlay
    positions = [n for s, n in enumerate(g.grid.names) if g.position_of(2 * s)]
    assert len(positions) == 12 and set(ov.paths) <= set(positions) and len(ov.paths) >= 6
    assert ov.relative == (aw.model.v0 is None) and ov.scale > 0
    # every path is one selection's; the head (in every waveform row): one arrow per path
    p = next(iter(ov.paths))
    assert {pp.sel.number for pp in ov.paths[p]} == {s.number for s in sels}
    assert aw.plane_t is not None and len(aw._head_lines) == len(aw.compare.rows)
    assert all(len(ov.arrows[q]) == len(ov.paths[q]) and all(a.label == HEAD for a in ov.arrows[q])
               for q in ov.paths)
    # dragging the head scrubs the arrows (no re-read)
    _row, line = next(iter(aw._head_lines.values()))
    line.setValue(2.0)
    line.sigDragged.emit(line)
    assert aw.plane_t == 2.0 and g.overlay is ov
    path0 = ov.paths[p][0]
    i = int(np.searchsorted(path0.t, path0.sel.zero + 2.0))
    assert np.allclose(next(a for a in ov.arrows[p] if a.sel is path0.sel).xy, path0.deg[i])
    assert all(abs(ln.value() - 2.0) < 1e-12 for _r, ln in aw._head_lines.values())
    # the numbers: (mV - the median of the first ORIGIN_S) through the sensor's matrix
    path = ov.paths[p][0]
    sel = path.sel
    dm = aw._position_map()
    m = aw.model
    ds = Dataset([sel], [], despike=m.despike, v0=m.v0, scale=m.scale, dome_map=dm)
    vd, vl = position_variables([p])
    t, d = ds.samples(sel, vd)
    _t, lf = ds.samples(sel, vl)
    head = t <= sel.t0 + ORIGIN_S
    o = np.array([np.median(d[head]), np.median(lf[head])]) if ov.relative else np.zeros(2)
    want = (np.c_[d, lf] - o) @ deg_per_mv(dm, p).T / m.scale
    assert np.allclose(path.deg, want[np.isfinite(want).all(axis=1)])
    # port forward is screen right, starboard left (as the dome's plane map)
    s_l = next(s for s, n in enumerate(g.grid.names) if n == "LFM")
    s_r = next(s for s, n in enumerate(g.grid.names) if n == "RFM")
    assert g.grid.fwd[s_l] == 1.0 and g.grid.fwd[s_r] == -1.0
    # a cursor: the arrows move to it (labelled), the paths are not re-read
    aw.add_cursor(1.0)
    assert _pump(app, lambda: any(a.label == "1" for a in g.overlay.arrows.get(p, [])), 3)
    assert g.overlay is ov
    a = next(a for a in g.overlay.arrows[p] if a.sel is sel and a.label == "1")
    i = int(np.searchsorted(path.t, sel.zero + 1.0))
    assert np.allclose(a.xy, path.deg[i])
    set_arrows(ov, [])                                    # neither head nor cursor: the peaks
    assert all(a.label == "" for a in ov.arrows[p])
    # the chips toggle one axis, the plot both
    g.grab()
    cell, _name, dchip, lchip, plot = g.map_rects()[s_l]
    QTest.mouseClick(g, Qt.MouseButton.LeftButton, pos=dchip.center().toPoint())
    app.processEvents()
    assert "pos:LFM:drag" in aw.checked_keys() and "pos:LFM:lift" not in aw.checked_keys()
    QTest.mouseClick(g, Qt.MouseButton.LeftButton, pos=plot.center().toPoint())
    app.processEvents()
    assert {"pos:LFM:drag", "pos:LFM:lift"} <= set(aw.checked_keys())
    _pump(app, lambda: False, 0.3)
    _shot(aw, "plane_map")
    # without V0: each path from its selection's start
    v0 = m.v0
    m.v0 = None
    aw._request()
    assert _pump(app, lambda: g.overlay is not ov and g.overlay.relative, 3)
    q = g.overlay.paths[p][0]
    assert np.allclose(np.median(q.deg[q.t <= q.sel.t0 + 0.25], axis=0), 0, atol=1e-9)
    m.v0 = v0
    # the gauge view turns it off
    aw.grid_view.setCurrentText(GAUGES_VIEW)
    assert not aw.plane_box.isChecked() and not g.map and g.height() == h0
    assert not aw._head_lines
    aw.close()


def test_plane_overlay_sub_ranges(app, win):
    from wavi_py.tests.test_analysis_r2 import _window
    aw, sels = _window(app, win, n_sel=1, keys=("wsk:2",))
    aw.add_sub(0.5, 1.5)
    aw.add_sub(2.0, 2.8)
    aw.plane_box.setChecked(True)
    g = aw.grids[0 if 0 in aw.grids else min(aw.grids)]
    assert _pump(app, lambda: g.overlay is not None and bool(g.overlay.paths), 5)
    p = next(iter(g.overlay.paths))
    paths = g.overlay.paths[p]
    assert [pp.sub for pp in paths] == [1, 2]
    z = sels[0].zero
    assert paths[0].t[0] >= z + 0.5 - 0.05 and paths[0].t[-1] <= z + 1.5 + 0.05
    assert aw._plane_pen([], paths[1]).style() == Qt.PenStyle.DashLine
    aw.close()


def test_zoom_cursor_fit_key_and_colour_default(app, win):
    from PyQt6.QtCore import QEvent
    from PyQt6.QtGui import QKeyEvent

    from wavi_py.tests.test_analysis_r2 import _window
    from wavi_py.views.analysis_cells import SENSOR, zoom_cursor
    aw, _sels = _window(app, win, n_sel=1, keys=("wsk:2",))
    assert aw.color_by == SENSOR and aw.compare.color_by == SENSOR
    # Zoom box: the magnifier over the zoomable cells, back to the arrow when off
    aw._act_zoom.setChecked(True)
    for cell in (aw.compare, aw.spectrum, aw.spectrogram, aw.at_cursors):
        assert cell.glw.viewport().cursor().pixmap().cacheKey() == zoom_cursor().pixmap().cacheKey()
    aw._act_zoom.setChecked(False)
    assert aw.compare.glw.viewport().cursor().shape() == Qt.CursorShape.ArrowCursor
    # F: the plot under the mouse fits its data
    row = next(iter(aw.compare.rows.values()))
    vb = row.getViewBox()
    vb.setRange(xRange=(0.5, 0.6), yRange=(100, 101), padding=0)
    aw._target = lambda: (aw.compare, row)
    ev = QKeyEvent(QEvent.Type.KeyPress, Qt.Key.Key_F, Qt.KeyboardModifier.NoModifier, "f")
    assert aw.eventFilter(aw.compare.glw.viewport(), ev)
    (x0, x1), (y0, y1) = vb.viewRange()
    assert x0 < 0.1 and x1 > 2.5 and y1 - y0 < 50
    aw.close()
