"""Dome view smoke test (offscreen Qt: no GL context, so nothing is painted).

Builds the window and runs the per-frame path (bus -> pose -> tube / line
geometry handed to pyqtgraph) without showing it. The GL paint itself is
exercised by the benchmark under Xvfb (see the dome-view report).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_dome_view.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from PyQt6.QtWidgets import QApplication  # noqa: E402

pytest.importorskip("OpenGL")

from wavi_py.daq.bus import SampleBus  # noqa: E402
from whisker_link.channels import CHANNEL_NAMES, NCH  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def test_dome_view_frame_path(app):
    from wavi_py.views import dome_view as dvm

    v = dvm.DomeView(fs=250.0)
    try:
        _frame_path(v, dvm)
    finally:
        v.set_bus(None)
        v.close()


def _frame_path(v, dvm):
    bus = SampleBus(1000, NCH)
    v.set_bus(bus, 250.0)
    base = np.full((40, NCH), 250.0, np.float32)
    bend = base.copy()
    bend[:, CHANNEL_NAMES.index("W1_A")] += 10.0
    for blk in (base, bend):                            # tare, then a bend on W1
        bus.publish(blk)
        samples, v._cursor = bus.read_since(v._cursor)
        v.pose.feed(samples, 250.0)
    v._render_pose()

    h = len(v.layout_)
    n_pts = dvm.N_ROOT + dvm.N_SHAFT + 1
    md = v._tube_item.opts["meshdata"]
    assert md.vertexes().shape == (h * n_pts * dvm.TUBE_SIDES, 3)
    assert md.faces().shape == (h * (n_pts - 1) * dvm.TUBE_SIDES * 2, 3)
    assert md.faces().max() < len(md.vertexes())
    col = md.vertexColors().reshape(h, -1, 4)
    hole = v.pose.hole_of[1]
    assert not np.allclose(col[hole, 0], col[(hole + 1) % h, 0])   # W1 bent: own colour
    assert np.isfinite(md.vertexes()).all()

    v.set_tubes(False)
    pos = v._line_item.pos
    assert pos.shape == (h * (n_pts - 1) * 2, 3)
    v.tare()
    assert not v.pose.tare.ready


def test_tube_geometry_ring_is_normal_to_the_axis():
    from wavi_py.views.dome_view import tube_vertices

    pts = np.zeros((1, 5, 3))
    pts[0, :, 2] = np.arange(5.0)
    V, N = tube_vertices(pts, 2.0, 6, np.array([[0.0, 1.0, 0.0]]))
    assert np.allclose(N[:, 2], 0) and np.allclose(np.linalg.norm(N, axis=1), 1)
    assert np.allclose(np.hypot(V[:, 0], V[:, 1]), 2.0)
