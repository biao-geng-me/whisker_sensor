"""whisker_link.dome_pose: mapping, tare, smoothing, calibration, shape."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from whisker_link import dome_pose as dp
from whisker_link.channels import CHANNEL_NAMES, NCH

ASSET = Path(__file__).resolve().parents[2] / "wavi_py" / "assets" / "dome" / "whiskers.json"


def toy_layout(h=3, length=130.0):
    """h holes on the x axis, all pointing +z, e1 = +x, e2 = +y."""
    root = np.c_[np.arange(h) * 30.0, np.zeros(h), np.zeros(h)]
    rep = lambda v: np.tile(v, (h, 1)).astype(float)    # noqa: E731
    return dp.DomeLayout(root, rep([0, 0, 1]), rep([1, 0, 0]), rep([0, 1, 0]),
                         np.full(h, length))


def test_placeholder_mapping_is_a_bijection_onto_12_holes():
    assert len(dp.LIVE_WHISKERS) == 12
    assert sorted(dp.PLACEHOLDER_HOLE_OF.values()) == list(range(12))
    assert not {3, 8, 10, 14} & set(dp.PLACEHOLDER_HOLE_OF)
    ch = dp.channels_of([1, 16])
    assert [CHANNEL_NAMES[i] for i in ch.ravel()] == ["W1_A", "W1_B", "W16_A", "W16_B"]


def test_asset_layout_frames_orthonormal():
    lay = dp.DomeLayout.load(ASSET)
    assert len(lay) == 12
    for v in (lay.axis, lay.e1, lay.e2):
        assert np.allclose(np.linalg.norm(v, axis=1), 1, atol=1e-5)
    assert np.allclose((lay.axis * lay.e1).sum(1), 0, atol=1e-5)
    assert np.allclose(np.cross(lay.axis, lay.e1), lay.e2, atol=1e-5)


def test_tare_auto_then_button_and_rail():
    t = dp.Tare(NCH, n_avg=10)
    base = np.linspace(200, 300, NCH)
    rail = np.full(NCH, False)
    rail[5] = True
    blk = np.tile(base, (10, 1))
    blk[:, 5] = 1e5                                    # railed
    assert np.isnan(t.update(blk, 250)).all()         # collecting
    assert t.ready
    d = t.update(blk + 3.0, 250)
    assert np.allclose(d[:, ~rail], 3.0)
    assert np.isnan(d[:, 5]).all()
    t.request()
    t.update(blk + 3.0, 250)
    assert np.allclose(t.update(blk + 3.0, 250)[:, ~rail], 0.0)


def test_tare_drift_removal_tracks_slowly():
    t = dp.Tare(1, n_avg=1, drift_tau_s=1.0)
    t.update(np.zeros((1, 1)), 100)
    for _ in range(100):                               # 1 s of a 10 mV step
        d = t.update(np.full((1, 1), 10.0), 100)
    assert 10 * math.exp(-1) * 0.9 < d[0, 0] < 10 * math.exp(-1) * 1.1


def test_smoother_step_response_and_nan_hold():
    s = dp.Smoother(2, tau_s=0.1)
    fs = 100.0
    s.update(np.zeros((1, 2)), fs)
    y = s.update(np.ones((10, 2)), fs)                 # 0.1 s = one tau
    assert y == pytest.approx(1 - math.exp(-1), rel=1e-6)
    blk = np.full((5, 2), np.nan)
    assert s.update(blk, fs) == pytest.approx(y)       # NaN holds
    # block-size invariance: one block of 20 == 20 blocks of 1
    a, b = dp.Smoother(1, 0.05), dp.Smoother(1, 0.05)
    x = np.random.default_rng(0).normal(size=(20, 1))
    a.update(x, fs)
    for r in x:
        b.update(r[None], fs)
    assert a.value == pytest.approx(b.value)


def test_calibration_nominal_diagonal():
    c = dp.Calibration.nominal(2, deg_per_mv=2.0)
    th = c.apply(np.array([[1.0, 0.0], [0.0, -3.0]]))
    assert np.degrees(th) == pytest.approx(np.array([[2.0, 0.0], [0.0, -6.0]]))


def test_shape_straight_and_lengths():
    lay = toy_layout()
    pts = dp.whisker_shape(lay, np.zeros((3, 2)), n_root=5, n_shaft=7)
    assert pts.shape == (3, 13, 3)
    assert np.allclose(pts[:, :, :2], lay.root[:, None, :2])
    assert np.allclose(pts[:, -1, 2], 130.0)
    # arc length is preserved for any bend
    th = np.array([[0.3, 0.0], [0.0, -0.5], [1.0, 1.0]])
    pts = dp.whisker_shape(lay, th, n_root=40, n_shaft=4)
    seg = np.linalg.norm(np.diff(pts, axis=1), axis=2).sum(1)
    assert seg == pytest.approx(130.0, rel=2e-4)


def test_shape_bend_direction_and_tip_angle():
    lay = toy_layout(1)
    phi = math.radians(20)
    pts = dp.whisker_shape(lay, np.array([[0.0, phi]]))        # toward +e2 = +y
    tip = pts[0, -1]
    assert tip[0] == pytest.approx(0, abs=1e-9) and tip[1] > 0
    shaft = pts[0, -1] - pts[0, -2]
    assert math.atan2(shaft[1], shaft[2]) == pytest.approx(phi)
    # root end: constant curvature arc of length LR
    r = dp.LR_MM / phi
    end = pts[0, 6]
    assert end[1] == pytest.approx(r * (1 - math.cos(phi)))
    assert end[2] == pytest.approx(r * math.sin(phi))


def test_dome_pose_pipeline():
    lay = toy_layout(3)
    pose = dp.DomePose(lay, hole_of={1: 0, 4: 2}, tare_n=4, smooth_tau_s=0.0)
    base = np.full(NCH, 250.0)
    pose.feed(np.tile(base, (4, 1)), 250)
    blk = base.copy()
    blk[CHANNEL_NAMES.index("W4_A")] += 10                # 10 mV on W4 gauge A
    pose.feed(blk[None], 250)
    th = pose.angles()
    assert np.isnan(th[1]).all()                             # unmapped hole
    assert np.degrees(th[2]) == pytest.approx([10.0, 0.0])
    assert np.degrees(th[0]) == pytest.approx([0.0, 0.0])
    pts, mag = pose.shape(gain=2.0)
    assert np.degrees(mag[2]) == pytest.approx(20.0)
    assert pts[2, -1, 0] > lay.root[2, 0] + 30               # tip leans toward +e1
    assert np.allclose(pts[1, -1], lay.root[1] + [0, 0, 130])
