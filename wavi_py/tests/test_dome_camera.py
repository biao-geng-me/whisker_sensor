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
