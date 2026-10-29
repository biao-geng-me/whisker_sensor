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


def test_dome_view_shell_pivot_trail(app):
    from wavi_py.views import dome_view as dvm

    v = dvm.DomeView(fs=250.0)
    try:
        base = np.full((40, NCH), 250.0)
        v.pose.feed(base, 250.0)                         # tare
        assert v._centre_pivot and v._show_grid and v._show_arrows
        for k in range(10):                              # W1 swings, 10 frames at 60 Hz
            blk = base[:5].copy()
            blk[:, CHANNEL_NAMES.index("W1_A")] += 5.0 * k
            v.pose.feed(blk, 250.0)
            v._render_pose(now=k / 60)
        hole = v.pose.hole_of[1]
        md = v._tube_item.opts["meshdata"]
        n_pts = dvm.N_ROOT + dvm.N_SHAFT + 1
        ring = md.vertexes().reshape(len(v.layout_), n_pts, dvm.TUBE_SIDES, 3)
        tip = ring[:, -1].mean(axis=1)                   # tube end centres
        assert np.abs(v.shell.distance(tip)).max() < 1e-3
        pos, col = v._trail_item.pos, v._trail_item.color
        assert len(pos) == len(col) > 0 and np.isfinite(pos).all()
        assert col[:, 3].max() > 0.5 and col[:, 3].min() >= 0
        trail = pos.reshape(len(v.layout_), -1, 2, 3)[hole]
        assert np.linalg.norm(trail[-1, 1] - trail[0, 0]) > 10   # W1's tip moved
        assert np.abs(v.shell.distance(trail.reshape(-1, 3))).max() < 1e-3
        assert v._grid_item.pos.shape[0] > 100
        assert "deg bend" in v._grid_label.text()
        v._gain_box.setValue(6.0)
        assert v.trail.count == 1                        # cleared, then one new sample
        v._pivot_box.setCurrentText("root")
        assert not v._centre_pivot
        v._grid_box.setValue(2.0)
        assert v._grid_label.text().strip().startswith("2 deg bend = 12 deg")
        v._radius_box.setValue(300)
        assert v.shell.radius == 300
        v._trail_box.setValue(0)
        assert not v._trail_item.visible()
    finally:
        v.close()


def test_tube_geometry_ring_is_normal_to_the_axis():
    from wavi_py.views.dome_view import tube_vertices

    pts = np.zeros((1, 5, 3))
    pts[0, :, 2] = np.arange(5.0)
    V, N = tube_vertices(pts, 2.0, 6, np.array([[0.0, 1.0, 0.0]]))
    assert np.allclose(N[:, 2], 0) and np.allclose(np.linalg.norm(N, axis=1), 1)
    assert np.allclose(np.hypot(V[:, 0], V[:, 1]), 2.0)


def test_dome_view_camera_cube_marks_arrows(app):
    from wavi_py.views import dome_view as dvm
    from wavi_py.views.dome_camera import SNAP_VIEWS

    v = dvm.DomeView(fs=250.0)
    try:
        glw, cam = v._glw, v._glw.cam
        glw.resize(800, 600)
        # the GL view matrix is the camera's
        m = np.array(glw.viewMatrix().data()).reshape(4, 4).T     # Qt: column-major data
        assert np.allclose(m, cam.view_matrix(), atol=1e-4)
        # tumble past the pole: no clamp
        for _ in range(60):
            glw.orbit(0, 5.0)                                     # 300 deg of elevation
        assert abs(cam.to_view([0, 0, 1])[1]) < 1.0
        # snaps: animated from the frame timer, exact at the end
        v.snap_view("top")
        assert cam.animating()
        cam.step(1e12)
        assert np.allclose(cam.position() - cam.center, cam.distance * SNAP_VIEWS["top"][0])
        names = {n for n, _, _ in glw.cube.faces()}
        assert names == {"top"}
        c = glw.cube.SIZE / 2
        assert glw.cube.face_at(c, c) == "top"
        glw.cube.clicked.emit("front")
        cam.step(1e12)
        assert np.allclose(cam.rot[2], SNAP_VIEWS["front"][0])
        acts = {a.shortcut().toString(): a for a in v.actions()}
        assert set(acts) >= set("0123456")
        acts["5"].trigger()
        cam.step(1e12)
        assert np.allclose(cam.rot[2], SNAP_VIEWS["starboard"][0])
        acts["0"].trigger()
        assert v._up_item.pos[1, 1] > v._up_item.pos[0, 1]       # UP arrow points +y
        # the UP label sits where the camera projects the arrow's end
        up_lab, up_at = v._marks["UP"]
        xy = glw.project(up_at)
        g = glw.cam.view_matrix() @ np.r_[up_at, 1]
        assert xy is not None and g[2] < 0
        assert abs(up_lab.x() + up_lab.width() / 2 - xy[0]) <= 1
        assert glw.project(glw.cam.center) == pytest.approx((400, 300))
        # orthographic by default; Perspective switches, keeping the camera
        assert not v._persp_box.isChecked() and not cam.perspective
        glw.cam.tumble(30, 10)
        glw.moved()
        behind = glw.cam.position() + glw.cam.rot[2]
        assert glw.project(behind) is not None
        pose = cam.rot.copy(), cam.center.copy(), cam.distance
        for persp in (True, False, True):
            v._persp_box.setChecked(persp)
            assert cam.perspective == persp
            assert np.allclose(cam.rot, pose[0]) and np.allclose(cam.center, pose[1])
            assert cam.distance == pose[2]
            vp = (0, 0, 800, 600)
            pm = np.array(glw.projectionMatrix(vp, vp).data()).reshape(4, 4).T
            assert np.allclose(pm, cam.projection_matrix(800, 600), rtol=1e-5, atol=1e-6)
            xy = glw.project(up_at)                   # the label follows the projection
            assert abs(up_lab.x() + up_lab.width() / 2 - xy[0]) <= 1
        assert glw.project(behind) is None
        v._persp_box.setChecked(False)
        # the orthographic clip planes hold the dome, the shell and the whiskers
        near, far = cam.clip_planes()
        pts = np.r_[v._dome_item.opts["meshdata"].vertexes(), v._grid_item.pos,
                    v.layout_.root + v.layout_.length[:, None] * v.layout_.axis]
        depth = -(cam.view_matrix() @ np.c_[pts, np.ones(len(pts))].T)[2]
        assert near < depth.min() and depth.max() < far

        # neutral dots on the shell, arrows from them to the tips
        base = np.full((40, NCH), 250.0)
        v.pose.feed(base, 250.0)
        blk = base[:5].copy()
        blk[:, CHANNEL_NAMES.index("W1_B")] += 8.0
        v.pose.feed(blk, 250.0)
        v._render_pose(now=0.0)
        assert np.abs(v.shell.distance(v._neutral_item.pos)).max() < 1e-3
        h = len(v.layout_)
        seg = v._arrow_item.pos.reshape(h, -1, 2, 3)
        hole = v.pose.hole_of[1]
        assert np.allclose(seg[hole, 0, 0], v._neutral[hole], atol=1e-3)
        tip = v._tube_item.opts["meshdata"].vertexes().reshape(h, -1, dvm.TUBE_SIDES, 3)[:, -1]
        assert np.allclose(seg[hole, -1, 0], tip[hole].mean(0), atol=1e-3)   # head at the tip
        assert np.abs(v.shell.distance(seg[hole, :-2].reshape(-1, 3))).max() < 1e-3
        v._grid_check.setChecked(False)
        assert not v._grid_item.visible()
        v._surface_box.setChecked(True)
        assert v._surface_item.visible()                      # independent of the grid
        v._arrow_box.setChecked(False)
        v._neutral_box.setChecked(False)
        assert not v._arrow_item.visible() and not v._neutral_item.visible()
        v._pivot_box.setCurrentText("root")
        assert np.allclose(v._neutral, v.layout_.root + v.layout_.length[:, None]
                           * v.layout_.axis)
    finally:
        v.close()


def test_dome_view_cylinder_grows_under_a_tip(app):
    from wavi_py.views import dome_view as dvm

    v = dvm.DomeView(fs=250.0)
    try:
        base = np.full((40, NCH), 250.0)
        v.pose.feed(base, 250.0)
        v._gain_box.setValue(8.0)
        L0 = v.shell.cyl_len
        n0 = len(v._grid_item.pos)
        blk = base[:5].copy()
        cyl = v.layout_.root[:, 2] < v.layout_.center[2] - 0.5
        w = next(w for w, hl in v.pose.hole_of.items() if cyl[hl])
        blk[:, CHANNEL_NAMES.index(f"W{w}_A")] -= 10.0        # 80 deg aft on a cylinder whisker
        v.pose.feed(blk, 250.0)
        v._render_pose(now=0.0)
        assert v.shell.cyl_len > L0 and len(v._grid_item.pos) > n0
        tip_z = v.trail.recent(0.0, 1.0)[0][-1, v.pose.hole_of[w], 2]
        assert tip_z >= v.layout_.center[2] - v.shell.cyl_len
        v.pose.feed(base, 250.0)                               # released
        v._gain_box.setValue(3.0)                              # restart: back to the base length
        assert v.shell.cyl_len == L0
    finally:
        v.close()


def test_dome_view_with_a_dome_map(app):
    """A dome map: its positions are the holes (labelled), the frame path
    works, a W7 bend lands on LFU; None goes back to the placeholder holes."""
    from wavi_py.views import dome_view as dvm
    from whisker_link.dome_map import DEFAULT_MAP, DomeMap

    m = DomeMap.load(DEFAULT_MAP)
    v = dvm.DomeView(fs=250.0, dome_map=m)
    try:
        assert v._labels_box.isChecked()
        assert v.hole_texts() == [p.display for p in m.positions]
        lfu = next(i for i, p in enumerate(m.positions) if p.key == "LFU")
        assert v._hole_labels[lfu].text() == "LFU (W7)"
        assert v.pose.ch[list(v.pose.holes).index(lfu)].tolist() == [
            CHANNEL_NAMES.index("W7_A"), CHANNEL_NAMES.index("W7_B")]
        assert np.allclose(v.layout_.root[lfu], m.positions[lfu].root)
        v.pose.hole_of = {1: 0}                 # _frame_path bends W1_A: LBL, hole 0
        _frame_path(v, dvm)
        v.pose.request_tare()
        base = np.full((40, NCH), 250.0)
        v.pose.feed(base, 250.0)
        bend = base[:5].copy()
        bend[:, CHANNEL_NAMES.index("W7_A")] += 5.0
        v.pose.feed(bend, 250.0)
        th = v.pose.angles()
        assert abs(th[lfu, 0]) > 0 and np.nanmax(np.abs(np.delete(th[:, 0], lfu))) == 0
        v.set_labels_visible(False)
        assert not any(lab.isVisible() for lab in v._hole_labels)
        v.set_dome_map(None)
        assert v.dome_map is None and not v._labels_box.isChecked()
        assert len(v.layout_) == 12 and v.hole_texts()[0] == "W1?"
        v._render_pose()
    finally:
        v.set_bus(None)
        v.close()
