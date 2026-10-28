"""wavi_py.views.dome_camera: the trackball camera's math (no Qt, no GL)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from wavi_py.views import dome_camera as dc  # noqa: E402


def is_rotation(r):
    return np.allclose(r @ r.T, np.eye(3), atol=1e-9) and np.linalg.det(r) == pytest.approx(1.0)


def test_rov_frame_is_right_handed():
    # facing forward, up overhead: forward x up points to the right (starboard)
    assert np.allclose(np.cross(dc.FORWARD, dc.UP), dc.STARBOARD)
    # FRD body frame (forward, starboard, down) is right-handed
    assert np.allclose(np.cross(dc.FORWARD, dc.STARBOARD), -dc.UP)


@pytest.mark.parametrize("name", list(dc.SNAP_VIEWS))
def test_snap_views_look_from_their_side_with_their_up(name):
    to_cam, up = dc.SNAP_VIEWS[name]
    cam = dc.TrackballCamera(center=(1.0, 2.0, 3.0), distance=50.0)
    cam.snap_to(name)
    assert is_rotation(cam.rot)
    assert np.allclose(cam.position(), cam.center + 50.0 * to_cam)
    assert np.allclose(cam.screen_up(), up)
    # the view matrix takes the centre to (0, 0, -d) and the camera to the origin
    m = cam.view_matrix()
    assert np.allclose(m @ np.r_[cam.center, 1], [0, 0, -50, 1])
    assert np.allclose(m @ np.r_[cam.position(), 1], [0, 0, 0, 1])


def test_snap_keys_and_screen_right():
    assert dc.SNAP_KEYS == {"1": "front", "2": "back", "3": "top", "4": "bottom",
                            "5": "starboard", "6": "port"}
    cam = dc.TrackballCamera()
    cam.snap_to("starboard")                    # from starboard: the nose is to the right
    assert np.allclose(cam.to_view(dc.FORWARD), [1, 0, 0])
    cam.snap_to("top")                          # from above, nose up: starboard on the right
    assert np.allclose(cam.to_view(dc.STARBOARD), [1, 0, 0])
    cam.snap_to("front")                        # facing the nose: port on the right
    assert np.allclose(cam.to_view(-dc.STARBOARD), [1, 0, 0])


def test_tumble_follows_the_pointer_and_has_no_pole():
    cam = dc.TrackballCamera(deg_per_px=1.0)
    cam.snap_to("front")
    nose = dc.FORWARD                            # at the screen centre, toward us
    cam.tumble(10, 0)                            # drag right: the nose moves right
    assert cam.to_view(nose)[0] > 0.1
    cam.snap_to("front")
    cam.tumble(0, 10)                            # drag down: it moves down
    assert cam.to_view(nose)[1] < -0.1
    # straight over the top and on: 360 deg of vertical drag comes back round
    cam.snap_to("front")
    r0 = cam.rot.copy()
    seen_upside_down = False
    for _ in range(72):
        cam.tumble(0, 5)
        assert is_rotation(cam.rot)
        seen_upside_down |= cam.screen_up() @ dc.UP < -0.99
    assert seen_upside_down                      # elevation was not clamped at +-90
    assert np.allclose(cam.rot, r0, atol=1e-6)


def test_tumble_composes_on_the_view_axes():
    # a horizontal drag turns about the SCREEN's vertical, whatever the world up
    cam = dc.TrackballCamera(deg_per_px=1.0)
    cam.snap_to("top")
    y_view = cam.rot[1].copy()                   # screen up, in world coordinates
    cam.tumble(37, 0)
    assert np.allclose(cam.rot[1], y_view)       # the screen-vertical axis stays put


def test_roll_pan_zoom():
    cam = dc.TrackballCamera(distance=100.0, fov_deg=60.0)
    cam.snap_to("front")
    cam.roll(90)
    assert np.allclose(cam.to_view(dc.UP), [-1, 0, 0], atol=1e-9)   # counter-clockwise
    cam.snap_to("front")
    c0 = cam.center.copy()
    cam.pan(10, 0, width_px=1000)                # drag right: the scene follows
    mm = 100 * 2 * np.tan(np.radians(30)) / 1000
    assert np.allclose(cam.center, c0 - 10 * mm * cam.rot[0])
    cam.zoom(0.5)
    assert cam.distance == pytest.approx(50.0)


def test_snap_animation_slerps_and_ends_exactly():
    cam = dc.TrackballCamera()
    cam.snap_to("front")
    cam.snap_to("starboard", animate_s=0.2, now=10.0)
    assert cam.animating() and cam.step(10.1)
    half = cam.rot[2]
    assert is_rotation(cam.rot)
    assert np.allclose(half, (dc.FORWARD + dc.STARBOARD) / np.sqrt(2), atol=1e-9)
    cam.step(10.5)
    assert not cam.animating() and not cam.step(11.0)
    assert np.allclose(cam.rot[2], dc.STARBOARD)
    cam.tumble(1, 0)                              # a drag cancels an animation
    cam.snap_to("top", animate_s=0.2, now=0.0)
    cam.tumble(1, 0)
    assert not cam.animating()


def test_look_rotation_up_along_view():
    r = dc.look_rotation([0, 0, 1], [0, 0, 1])   # degenerate up: still a rotation
    assert is_rotation(r) and np.allclose(r[2], [0, 0, 1])


# ----- projection
W, H = 800, 600


def posed_camera(perspective: bool) -> dc.TrackballCamera:
    cam = dc.TrackballCamera(center=(5.0, -3.0, 40.0), distance=300.0)
    cam.tumble(37, -21)
    cam.roll(12)
    cam.perspective = perspective
    return cam


def ndc(cam, p, region=None):
    c = cam.projection_matrix(W, H, region) @ cam.view_matrix() @ np.r_[p, 1.0]
    return c[:3] / c[3]


def test_perspective_projection_is_the_pinhole_with_horizontal_fov():
    cam = posed_camera(True)
    rng = np.random.default_rng(1)
    f = 0.5 * W / np.tan(np.radians(30))
    for p in cam.center + rng.normal(0, 80, (20, 3)):
        e = cam.view_matrix() @ np.r_[p, 1]
        x, y = cam.project(p, W, H)
        assert x == pytest.approx(W / 2 + f * e[0] / -e[2])
        assert y == pytest.approx(H / 2 - f * e[1] / -e[2])
    assert cam.project(cam.position() + cam.rot[2], W, H) is None          # behind
    near, far = cam.clip_planes()
    assert (near, far) == pytest.approx((0.3, 3e5))                         # pyqtgraph's


def test_ortho_projection_is_parallel_with_the_perspective_scale_at_the_look_at():
    o, p = posed_camera(False), posed_camera(True)
    mm = o.mm_per_px(W)
    assert mm == pytest.approx(p.mm_per_px(W))
    assert o.project(o.center, W, H) == pytest.approx((W / 2, H / 2))
    for a, b in ((10, 0), (0, 25), (-120, 77)):
        q = o.center + mm * (a * o.rot[0] + b * o.rot[1])
        # in the look-at plane both projections put it at the same pixel ...
        assert o.project(q, W, H) == pytest.approx((W / 2 + a, H / 2 - b))
        assert p.project(q, W, H) == pytest.approx((W / 2 + a, H / 2 - b))
        # ... and the orthographic one at any depth, even behind the eye point
        for dz in (-500.0, 290.0, 1000.0):
            assert o.project(q + dz * o.rot[2], W, H) == pytest.approx((W / 2 + a, H / 2 - b))
    # zoom scales the extent
    o.zoom(0.5)
    q = o.center + 100 * o.rot[0]
    assert o.project(q, W, H)[0] - W / 2 == pytest.approx(2 * 100 / mm)


@pytest.mark.parametrize("perspective", [False, True])
def test_pan_follows_the_pointer(perspective):
    cam = posed_camera(perspective)
    q = cam.center + 30 * cam.rot[0] - 10 * cam.rot[1]       # a point in the look-at plane
    x0, y0 = cam.project(q, W, H)
    cam.pan(40, -15, W)
    assert cam.project(q, W, H) == pytest.approx((x0 + 40, y0 - 15))


@pytest.mark.parametrize("perspective", [False, True])
def test_region_projection_maps_the_region_to_the_full_clip_square(perspective):
    """pyqtgraph's picking renders a sub-rectangle (x, y, w, h; y up from the
    viewport's bottom, as GL's) with projectionMatrix(region, viewport)."""
    cam = posed_camera(perspective)
    region = (300.0, 200.0, 50.0, 40.0)
    for p in cam.center + np.random.default_rng(2).normal(0, 60, (10, 3)):
        full = ndc(cam, p)
        px, py = (full[0] + 1) / 2 * W, (full[1] + 1) / 2 * H     # GL window coords
        sub = ndc(cam, p, region)
        assert sub[0] == pytest.approx((px - region[0]) / region[2] * 2 - 1)
        assert sub[1] == pytest.approx((py - region[1]) / region[3] * 2 - 1)
        assert sub[2] == pytest.approx(full[2])


def test_ortho_clip_planes_hold_the_bounds_at_any_zoom_pan_and_turn():
    rng = np.random.default_rng(3)
    cam = dc.TrackballCamera(distance=400.0)
    cam.perspective = False
    c, r = np.array([0.0, 0.0, 100.0]), 250.0
    cam.set_bounds(c, r)
    d = rng.normal(size=(500, 3))
    sphere = c + r * d / np.linalg.norm(d, axis=1, keepdims=True)
    for _ in range(40):
        cam.tumble(*rng.uniform(-400, 400, 2))
        cam.zoom(float(np.exp(rng.uniform(-3, 3))))          # inside the dome and far off
        cam.pan(*rng.uniform(-300, 300, 2), W)
        z = np.array([ndc(cam, p)[2] for p in sphere])
        assert np.all(np.abs(z) < 1.0)
        assert z.max() - z.min() > 1.5                        # and use most of the depth range
