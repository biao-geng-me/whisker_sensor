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


# ----- scribe shell, centre pivot, trails

def test_layout_center_from_asset_and_fallback():
    lay = dp.DomeLayout.load(ASSET)
    assert lay.center[:2] == pytest.approx([0, 0]) and lay.dome_radius > 50
    fit = dp.DomeLayout(lay.root, lay.axis, lay.e1, lay.e2, lay.length)
    assert fit.center[2] == pytest.approx(lay.center[2], abs=3.0)


def test_centre_pivots_sphere_and_cylinder():
    lay = dp.DomeLayout.load(ASSET)
    piv = dp.centre_pivots(lay)
    cyl = lay.root[:, 2] < lay.center[2] - 0.5
    assert cyl.any() and (~cyl).any()          # this dome has both kinds
    assert np.allclose(piv[~cyl], lay.center)
    assert np.allclose(piv[cyl, :2], 0) and np.allclose(piv[cyl, 2], lay.root[cyl, 2])


@pytest.mark.parametrize("scale", [0.0, 0.2, 0.8, 1.4])
def test_centre_pivot_tips_on_shell(scale):
    lay = dp.DomeLayout.load(ASSET)
    shell = dp.Shell.around(lay)
    th = np.random.default_rng(3).normal(size=(len(lay), 2)) * scale
    pts, tips = dp.centre_pivot_shape(lay, shell, th, n_pts=9)
    assert np.abs(shell.distance(tips)).max() < 1e-6
    assert np.allclose(pts[:, 0], lay.root) and np.allclose(pts[:, -1], tips)


def test_centre_pivot_sphere_arc_equals_bend():
    lay = dp.DomeLayout.load(ASSET)
    shell = dp.Shell.around(lay, radius=200.0)
    sph = lay.root[:, 2] >= lay.center[2] - 0.5
    th = np.zeros((len(lay), 2))
    th[:, 1] = math.radians(10)               # sideways: stays on the sphere
    _, rest = dp.centre_pivot_shape(lay, shell, np.zeros_like(th))
    _, tips = dp.centre_pivot_shape(lay, shell, th)
    a, b = rest[sph] - lay.center, tips[sph] - lay.center
    ang = np.arccos(np.einsum("ij,ij->i", a, b) / 200.0 ** 2)
    assert np.degrees(ang) == pytest.approx(10.0, abs=1e-6)


def test_shell_hit_crosses_equator_both_ways():
    sh = dp.Shell(np.array([0.0, 0, 100]), 50.0)
    o = np.array([[0.0, 0, 100], [0.0, 0, 90], [0.0, 0, 100]])
    v = np.array([[1.0, 0, -1], [0.0, 0.2, 1], [0.0, 0, 1]])
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    p = sh.hit(o, v)
    assert np.abs(sh.distance(p)).max() < 1e-9
    assert p[0, 2] == pytest.approx(50.0)                      # cylinder, 45 deg down
    assert p[2] == pytest.approx([0, 0, 150])                  # pole


def test_shell_grid_on_surface_and_spacing():
    sh = dp.Shell(np.zeros(3), 100.0)
    g = sh.grid(math.radians(15))
    assert np.abs(sh.distance(g.reshape(-1, 3))).max() < 1e-6
    rings = {round(z, 6) for z in g[:, :, 2].ravel() if z > 0}
    assert any(abs(z - 100 * math.cos(math.radians(15))) < 1e-6 for z in rings)
    assert dp.nice_grid_step(3.0) == 5.0 and dp.nice_grid_step(1.0) == 10.0


def test_trail_buffer_window_rate_and_wrap():
    tb = dp.TrailBuffer(2, max_s=1.0, rate_hz=10.0)
    for k in range(40):                         # 4 s at 10 Hz: wraps the ring
        tb.push(k * 0.1, np.full((2, 3), float(k)), np.full(2, 0.1))
    assert not tb.push(3.95, np.zeros((2, 3)), np.zeros(2))   # faster than 10 Hz
    pos, mag, age = tb.recent(3.9, 0.5)
    assert pos[:, 0, 0].tolist() == [34.0, 35.0, 36.0, 37.0, 38.0, 39.0]
    assert age[-1] == pytest.approx(0) and age[0] == pytest.approx(0.5)
    p, m, a = tb.segments(3.9, 0.5)
    assert p.shape == (2 * 5 * 2, 3) and a.max() <= 1 and a.min() >= 0
    assert a[1] > a[0]                          # fades with age


def test_trail_gap_is_transparent():
    tb = dp.TrailBuffer(1, max_s=1.0, rate_hz=10.0)
    for k, x in enumerate([0.0, 1.0, np.nan, 3.0]):
        tb.push(k * 0.1, np.full((1, 3), x), np.zeros(1))
    p, m, a = tb.segments(0.3, 1.0)
    assert np.isfinite(p).all()
    assert (a.reshape(-1, 2)[1:] == 0).all() and (a.reshape(-1, 2)[0] > 0).all()


def test_trail_time_going_back_restarts():
    tb = dp.TrailBuffer(1, max_s=1.0, rate_hz=10.0)
    tb.push(100.0, np.zeros((1, 3)), np.zeros(1))
    assert tb.push(0.0, np.ones((1, 3)), np.zeros(1)) and tb.count == 1


# ----- neutral marks and deflection arrows

def test_root_tips_match_whisker_shape():
    lay = dp.DomeLayout.load(ASSET)
    th = np.random.default_rng(5).normal(size=(len(lay), 2)) * 0.5
    th[0] = 0.0
    th[1] = [1e-8, 0.0]                         # the small-angle branches
    assert np.allclose(dp.root_tips(lay, th), dp.whisker_shape(lay, th)[:, -1], atol=1e-6)
    stacked = dp.root_tips(lay, np.stack([th, 2 * th]))
    assert stacked.shape == (2, len(lay), 3)
    assert np.allclose(stacked[1], dp.root_tips(lay, 2 * th))


def test_neutral_tips_both_modes():
    lay = dp.DomeLayout.load(ASSET)
    shell = dp.Shell.around(lay)
    n = dp.neutral_tips(lay, shell)
    assert np.abs(shell.distance(n)).max() < 1e-6
    # on the line from the pivot through the root
    u = lay.root - dp.centre_pivots(lay)
    w = n - dp.centre_pivots(lay)
    assert np.allclose(np.cross(u, w), 0, atol=1e-6) and ((u * w).sum(1) > 0).all()
    assert np.allclose(dp.neutral_tips(lay), lay.root + lay.length[:, None] * lay.axis)


def test_tip_path_from_neutral_to_tip_on_the_shell():
    lay = dp.DomeLayout.load(ASSET)
    shell = dp.Shell.around(lay, radius=200.0)
    th = np.zeros((len(lay), 2))
    th[:, 1] = math.radians(30)
    path = dp.tip_path(lay, th, shell, n=7)
    assert path.shape == (len(lay), 7, 3)
    assert np.allclose(path[:, 0], dp.neutral_tips(lay, shell))
    assert np.allclose(path[:, -1], dp.centre_pivot_shape(lay, shell, th)[1])
    assert np.abs(shell.distance(path.reshape(-1, 3))).max() < 1e-6
    # on the sphere a sideways bend is a great-circle arc, evenly spaced
    sph = lay.root[:, 2] >= lay.center[2] - 0.5
    step = np.linalg.norm(np.diff(path[sph], axis=1), axis=2)
    assert np.allclose(step, 2 * 200 * math.sin(math.radians(5) / 2))
    root = dp.tip_path(lay, th, None, n=4)
    assert np.allclose(root[:, -1], dp.whisker_shape(lay, th)[:, -1])


def test_arrow_segments_shaft_and_head():
    path = np.zeros((2, 5, 3))
    path[0, :, 0] = np.linspace(0, 40, 5)       # 40 mm along +x
    normal = np.array([[0, 0, 1.0], [0, 0, 1.0]])
    seg = dp.arrow_segments(path, normal, head_mm=8.0, head_deg=30.0)
    assert seg.shape == (2, 6, 2, 3)
    assert np.allclose(seg[0, :4, 0], path[0, :-1]) and np.allclose(seg[0, :4, 1], path[0, 1:])
    head = seg[0, 4:]
    assert np.allclose(head[:, 0], [40, 0, 0])  # barbs start at the tip
    b = head[:, 1] - [40, 0, 0]
    assert np.allclose(np.linalg.norm(b, axis=1), 8.0)
    assert np.allclose(b[:, 2], 0)              # in the plane normal to `normal`
    assert np.allclose(b[:, 0], -8 * math.cos(math.radians(30)))
    assert b[0, 1] == pytest.approx(-b[1, 1]) and abs(b[0, 1]) > 1
    # zero arrow: everything collapses onto the tip
    assert np.allclose(seg[1], 0) and np.isfinite(seg).all()
    # short arrow: the head is at most 40 % of the chord
    short = dp.arrow_segments(path * 0.25, normal, head_mm=8.0)
    assert np.allclose(np.linalg.norm(short[0, 4, 1] - short[0, 4, 0]), 4.0)


def test_shell_cover_grows_the_cylinder_in_steps():
    sh = dp.Shell(np.array([0.0, 0, 100]), 40.0)
    assert sh.cyl_len == 40.0
    assert not sh.cover(np.array([[0, 40.0, 70]]))                  # within
    assert sh.cover(np.array([[0, 40.0, 41], [np.nan] * 3]))       # 59 below -> 60
    assert sh.cyl_len == pytest.approx(60.0)
    assert sh.cover(np.array([[0, 40.0, -1e6]])) and sh.cyl_len == pytest.approx(400.0)
    assert sh.uncover() and sh.cyl_len == 40.0 and not sh.uncover()


def test_plane_map_layout_and_tips():
    from whisker_link.dome_pose import PlaneMap

    lay = dp.DomeLayout.load(ASSET)
    pm = PlaneMap.of(lay)
    port = lay.root[:, 0] > 0
    assert (pm.xy[port, 0] < 0).all() and (pm.xy[~port, 0] > 0).all()
    assert np.allclose(pm.fwd, np.where(port, 1.0, -1.0))
    assert len(pm.segments) == 14 and 20 < pm.spacing < 40
    # the front column (larger z) is nearer the centre line
    front = lay.root[:, 2] > lay.root[:, 2].mean()
    assert np.abs(pm.xy[front, 0]).max() < np.abs(pm.xy[~front, 0]).min()
    th = np.zeros((len(lay), 2))
    th[:, 0] = 0.1
    tips = pm.tips(lay, th, 50.0)
    assert np.allclose(tips[:, 0] - pm.xy[:, 0], 5.0 * pm.fwd)
    th[:] = [0.0, 0.1]
    up = pm.tips(lay, th, 50.0)[:, 1] - pm.xy[:, 1]
    assert np.allclose(up, 5.0 * PlaneMap.lift_up(lay))
    th[0] = np.nan
    assert np.allclose(pm.tips(lay, th, 50.0)[0], pm.xy[0])
