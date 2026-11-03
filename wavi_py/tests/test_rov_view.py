"""IMU pose / translational acceleration (daq/imu_pose.py), the ROV view
(offscreen Qt: no GL context, nothing painted; built and fed, the transforms
and arrows checked) and the ROV mesh asset.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_rov_view.py
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from wavi_py.daq.imu_pose import (FRD_TO_SCENE, ImuPose, body_to_ned,  # noqa: E402
                                  gravity_body_mg, linear_accel_mg, scene_rotation)

ROV_NPZ = Path(__file__).resolve().parents[1] / "assets" / "rov" / "rov_mesh.npz"


def imu_rows(n, accel_lin_body=(0.0, 0.0, 0.0), roll=0.0, pitch=0.0, yaw=0.0, bias=(0, 0, 0)):
    """``n`` IMU_CHANNELS rows of a vehicle at this attitude accelerating so (mg,
    body), as RAW_IMU reads it: specific force = a - g."""
    f = np.asarray(accel_lin_body, float) - gravity_body_mg(roll, pitch) + bias
    rows = np.zeros((n, 9), np.float32)
    rows[:, 0:3] = f
    rows[:, 6:9] = (roll, pitch, yaw)
    return rows


def test_gravity_and_linear_accel():
    for roll, pitch in ((0.0, 0.0), (0.3, -0.2), (-0.5, 0.6)):
        rest = imu_rows(3, roll=roll, pitch=pitch)
        assert np.allclose(linear_accel_mg(rest), 0.0, atol=1e-3)
        # gravity in the body frame is the NED down axis turned into it
        g = body_to_ned(roll, pitch, 1.0).T @ [0.0, 0.0, 1000.0]
        assert np.allclose(gravity_body_mg(roll, pitch), g)
    level = imu_rows(1, (50.0, -20.0, 0.0))
    assert np.allclose(level[0, :3], [50.0, -20.0, -1000.0])       # RAW_IMU's sign at rest
    assert np.allclose(linear_accel_mg(level), [[50.0, -20.0, 0.0]], atol=1e-3)


def test_scene_frame():
    assert np.isclose(np.linalg.det(FRD_TO_SCENE), 1.0)
    fwd, up, port = np.eye(3)[2], np.eye(3)[1], np.eye(3)[0]
    assert np.allclose(FRD_TO_SCENE @ [1, 0, 0], fwd)              # forward +z
    assert np.allclose(FRD_TO_SCENE @ [0, 0, -1], up)              # up +y
    assert np.allclose(FRD_TO_SCENE @ [0, -1, 0], port)            # port +x
    nose_up = scene_rotation(0.0, np.radians(20)) @ fwd
    assert nose_up[1] > 0.3 and np.isclose(nose_up[0], 0.0)
    stbd_down = scene_rotation(np.radians(20), 0.0) @ -port        # roll + : starboard down
    assert stbd_down[1] < -0.3
    turn_right = scene_rotation(0.0, 0.0, np.radians(90)) @ fwd    # yaw + : to starboard
    assert np.allclose(turn_right, -port, atol=1e-9)


def test_imu_pose_at_and_tare():
    t = np.arange(300) / 100.0
    rows = np.concatenate([imu_rows(100, bias=(25.0, -10.0, 0.0)),
                           imu_rows(200, (80.0, 0.0, -30.0), pitch=0.1, bias=(25.0, -10.0, 0.0))])
    p = ImuPose(smooth_s=0.2)
    s = p.at(t, rows)
    assert s.n == 20 and np.isclose(s.t, 2.99) and np.isclose(s.pitch, 0.1)
    assert np.allclose(s.accel_body, [105.0, -10.0, -30.0], atol=1e-3)   # the bias is in
    assert p.tare(t, rows, 0.5) and np.isclose(p.bias_t, 0.5)
    assert np.allclose(p.bias, [25.0, -10.0, 0.0], atol=1e-3)
    assert np.allclose(p.at(t, rows).accel_body, [80.0, 0.0, -30.0], atol=1e-3)
    at1 = p.at(t, rows, 1.05)                    # the window straddles the step: a mean
    assert at1.n == 20 and 0.0 < at1.accel_body[0] < 80.0
    assert p.at(t, rows, -1.0) is None and p.at(t, rows, 10.0) is None    # not in the buffer
    assert p.at(np.empty(0), np.empty((0, 9))) is None
    assert not p.tare(t, rows, 50.0) and np.isclose(p.bias_t, 0.5)       # kept
    w = p.at(t, rows).accel_world                # pitched up: surge has an up (-down) part
    assert w[2] < -5.0


def test_rov_mesh_asset():
    z = np.load(ROV_NPZ)
    groups = json.loads(str(z["groups"]))
    assert set(groups) == {"thrusters", "enclosures", "buoyancy", "frame"}
    src = json.loads(str(z["source"]))
    assert "CC BY 4.0" in src["license"] and "BLUEROV2-R4" in src["url"]
    n_tri = sum(len(z[f"f_{g}"]) for g in groups)
    assert n_tri < 40_000
    for g in groups:
        V, F = z[f"v_{g}"], z[f"f_{g}"]
        assert V.dtype == np.float32 and F.max() < len(V)
    V = np.concatenate([z[f"v_{g}"] for g in groups])
    lo, hi = V.min(0), V.max(0)
    # the collar frame: the vehicle behind the seal face (z < ~70), symmetric port / stbd
    assert hi[2] < 80 and lo[2] < -350 and abs(lo[0] + hi[0]) < 5 and 300 < hi[0] - lo[0] < 400


@pytest.fixture(scope="module")
def app():
    pytest.importorskip("OpenGL")
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_rov_view(app):
    from wavi_py.views import rov_view as rvm

    v = rvm.RovView()
    try:
        assert {it.group for it in v._body} == {"thrusters", "enclosures", "buoyancy", "frame",
                                                "dome"}
        v._render()
        assert v.sample is None and "no IMU data" in v._readout.text()
        t = np.arange(400) / 100.0
        bias = (25.0, -10.0, 0.0)
        rows = np.concatenate([imu_rows(150, bias=bias, yaw=1.0),
                               imu_rows(250, (100.0, 0.0, 0.0), roll=0.2, pitch=-0.1, yaw=1.5,
                                        bias=bias)])
        v.update_view(t[:120], rows[:120])           # the first second: auto tare
        assert v.pose.bias_t is not None and np.allclose(v.pose.bias, bias, atol=1e-3)
        v.update_view(t, rows)
        v._render()
        s = v.sample
        assert np.allclose(s.accel_body, [100.0, 0.0, 0.0], atol=1e-3)
        # the body turned about the centre by roll / pitch only (Heading off)
        R = scene_rotation(0.2, -0.1)
        m = np.array(v._body[0].transform().data()).reshape(4, 4).T     # column-major
        assert np.allclose(m[:3, :3], R, atol=1e-6)
        assert np.allclose(m[:3, :3] @ v.centre + m[:3, 3], v.centre, atol=1e-3)
        # the resultant arrow: centre -> centre + scale * (body surge, turned)
        pos = v._arrow_item.pos
        tip = v.centre + v._scale / 100.0 * (FRD_TO_SCENE @ body_to_ned(0.2, -0.1, 0.0)
                                             @ [100.0, 0.0, 0.0])
        assert np.allclose(pos[0], v.centre, atol=1e-3) and np.allclose(pos[1], tip, atol=1e-2)
        assert len(pos) == 10 + 10                   # resultant + the one non-zero part
        v.set_components(False)
        v._render()
        assert len(v._arrow_item.pos) == 10
        # Heading: the yaw relative to the tare's
        v.set_heading(True)
        v._render()
        m = np.array(v._body[0].transform().data()).reshape(4, 4).T
        assert np.allclose(m[:3, :3], scene_rotation(0.2, -0.1, 0.5), atol=1e-6)
        # a picked time (paused live, timeline click): the rest part
        v.show_at(0.8)
        v._render()
        assert np.isclose(v.sample.t, 0.8) and np.allclose(v.sample.accel_body, 0.0, atol=1e-3)
        assert "(picked)" in v._readout.text()
        v.update_view(t, rows)                       # new rows: the newest again
        v._render()
        assert np.isclose(v.sample.t, 3.99)
        # Tare at the shown time; clear (a new connection) forgets it
        assert v.tare(3.0) and np.allclose(v.pose.bias, np.asarray(bias) + [100, 0, 0], atol=0.1)
        v.clear()
        assert v.pose.bias_t is None and v.sample is not None    # until the next render
        v._render()
        assert v.sample is None
    finally:
        v.close()


def test_dome_view_rov_toggle(app):
    from wavi_py.views import dome_view as dvm

    v = dvm.DomeView(fs=250.0)
    try:
        r0 = v._glw.cam.bounds[1]
        d0 = v._glw.cam.distance
        v._rov_box.setChecked(True)
        assert v._rov_items and all(it.visible() for it in v._rov_items)
        assert v._glw.cam.bounds[1] > 1.5 * r0 and v._glw.cam.distance > d0     # the whole ROV
        v._rov_box.setChecked(False)
        assert not any(it.visible() for it in v._rov_items)
        assert np.isclose(v._glw.cam.bounds[1], r0) and np.isclose(v._glw.cam.distance, d0)
    finally:
        v.close()
