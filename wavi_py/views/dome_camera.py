"""Trackball camera for the Dome view (numpy only, no Qt or GL, so it is testable).

pyqtgraph's GLViewWidget orbits by azimuth / elevation and clamps the
elevation to +-90 deg, so the dome cannot be turned over. Here the camera is a
rotation ``rot`` (world -> view; view x right, y up, the camera looking down
-z) plus a look-at ``center`` and a ``distance``. A drag turns the scene about
the VIEW's x and y axes (rotations composed on the left), so it tumbles freely
in every direction with no gimbal lock and no pole; the up vector is whatever
the drags made it.

Scene frame (the dome assets', i.e. rov-cad's collar frame) in ROV terms:
+z = forward (the tube axis; the dome is the ROV's nose), +y = up (the ROV's
underside is -y), so +x = port and -x = starboard (right-handed).
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

FORWARD = np.array([0.0, 0.0, 1.0])
UP = np.array([0.0, 1.0, 0.0])
STARBOARD = np.array([-1.0, 0.0, 0.0])

# Snap views: name -> (direction from the centre TO the camera, screen-up).
# Keys 1..6 in this order; 0 is the default 3/4 view.
SNAP_VIEWS = {
    "front": (FORWARD, UP),             # looking aft at the dome's nose
    "back": (-FORWARD, UP),
    "top": (UP, FORWARD),               # from above, nose at the top of the screen
    "bottom": (-UP, -FORWARD),          # from below, nose at the bottom
    "starboard": (STARBOARD, UP),       # from the starboard side, nose to the right
    "port": (-STARBOARD, UP),
}
SNAP_KEYS = {str(k + 1): name for k, name in enumerate(SNAP_VIEWS)}
# default: from front-starboard, a little above, ROV-up up the screen
HOME_DIR = 0.62 * FORWARD + 0.62 * STARBOARD + 0.48 * UP


def look_rotation(to_camera, up) -> np.ndarray:
    """World -> view rotation (3x3, rows = view x, y, z in world coordinates)
    for a camera on the ``to_camera`` side of the centre with ``up`` (made
    normal to the view direction) up the screen."""
    z = np.asarray(to_camera, float)
    z = z / np.linalg.norm(z)
    up = np.asarray(up, float)
    y = up - (up @ z) * z
    if np.linalg.norm(y) < 1e-9:                      # up along the view: any normal
        y = np.cross(z, [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0])
    y = y / np.linalg.norm(y)
    return np.stack([np.cross(y, z), y, z])


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


class TrackballCamera:
    def __init__(self, center=(0.0, 0.0, 0.0), distance: float = 100.0, fov_deg: float = 60.0,
                 deg_per_px: float = 0.4):
        self.center = np.asarray(center, float).copy()
        self.distance = float(distance)
        self.fov_deg = float(fov_deg)
        self.deg_per_px = float(deg_per_px)
        self.rot = look_rotation(HOME_DIR, UP)
        self._anim: Slerp | None = None

    # ----- state
    def position(self) -> np.ndarray:
        """Camera position, world mm."""
        return self.center + self.distance * self.rot[2]

    def view_matrix(self) -> np.ndarray:
        """4x4 world -> eye: rotate about the centre, then back off by the distance."""
        m = np.eye(4)
        m[:3, :3] = self.rot
        m[:3, 3] = -self.rot @ self.center - [0.0, 0.0, self.distance]
        return m

    def to_view(self, v) -> np.ndarray:
        """Directions (..., 3) in view coordinates (x right, y up, z toward the viewer)."""
        return np.asarray(v, float) @ self.rot.T

    def screen_up(self) -> np.ndarray:
        return self.rot[1].copy()

    # ----- mouse
    def tumble(self, dx_px: float, dy_px: float) -> None:
        """Drag by (dx, dy) screen pixels (y down): the scene turns with the
        pointer about the view's y and x axes. Unlimited in every direction."""
        k = math.radians(self.deg_per_px)
        self.rot = _rot_x(dy_px * k) @ _rot_y(dx_px * k) @ self.rot
        self._orthonormalise()
        self._anim = None

    def roll(self, deg: float) -> None:
        """Turn the scene about the view direction (counter-clockwise on screen)."""
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        self.rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]) @ self.rot
        self._anim = None

    def pan(self, dx_px: float, dy_px: float, width_px: float) -> None:
        """Drag the scene by (dx, dy) pixels in the view plane (it follows the pointer)."""
        mm = self.mm_per_px(width_px)
        self.center -= mm * (dx_px * self.rot[0] - dy_px * self.rot[1])

    def zoom(self, factor: float) -> None:
        self.distance = float(np.clip(self.distance * factor, 1.0, 1e6))

    def mm_per_px(self, width_px: float) -> float:
        """Size of a pixel at the look-at point."""
        return self.distance * 2 * math.tan(math.radians(self.fov_deg) / 2) / max(width_px, 1)

    # ----- snaps
    def snap(self, to_camera, up, animate_s: float = 0.0, now: float = 0.0) -> None:
        """Turn to look from ``to_camera`` with ``up`` up; with ``animate_s`` > 0
        the turn is spread over that time (``step(now)`` advances it)."""
        target = look_rotation(to_camera, up)
        if animate_s <= 0:
            self.rot, self._anim = target, None
            return
        self._anim = Slerp([now, now + animate_s], Rotation.from_matrix(np.stack([self.rot, target])))

    def snap_to(self, name: str, **kw) -> None:
        to_camera, up = (HOME_DIR, UP) if name == "home" else SNAP_VIEWS[name]
        self.snap(to_camera, up, **kw)

    def animating(self) -> bool:
        return self._anim is not None

    def step(self, now: float) -> bool:
        """Advance a snap animation to ``now``; False when there is none (left)."""
        if self._anim is None:
            return False
        t0, t1 = self._anim.times
        self.rot = self._anim(min(max(now, t0), t1)).as_matrix()
        if now >= t1:
            self._anim = None
        return True

    def _orthonormalise(self) -> None:
        u, _, vt = np.linalg.svd(self.rot)
        self.rot = u @ vt
