"""Trackball camera for the Dome view (numpy only, no Qt or GL, so it is testable).

pyqtgraph's GLViewWidget orbits by azimuth / elevation and clamps the
elevation to +-90 deg, so the dome cannot be turned over. Here the camera is a
rotation ``rot`` (world -> view; view x right, y up, the camera looking down
-z) plus a look-at ``center`` and a ``distance``. A drag turns the scene about
the VIEW's x and y axes (rotations composed on the left), so it tumbles freely
in every direction with no gimbal lock and no pole; the up vector is whatever
the drags made it.

Projection: perspective (``fov_deg`` horizontal, as pyqtgraph's) or
orthographic. Both share the one zoom state, ``distance``: the orthographic
view's half-width is the perspective view's half-width at the look-at point,
so the scene keeps its size on screen across a switch and ``mm_per_px`` (pan)
holds in both. The orthographic near / far planes are put round the scene's
bounding sphere (``set_bounds``), so nothing in it clips at any zoom or turn.

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
        self.perspective = True
        self.bounds: tuple[np.ndarray, float] | None = None     # scene sphere (centre, radius)
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
        """Size of a pixel at the look-at point (anywhere, in the orthographic view)."""
        return 2 * self.half_width() / max(width_px, 1)

    def half_width(self) -> float:
        """Half the view's width at the look-at point, mm (the orthographic extent)."""
        return self.distance * math.tan(math.radians(self.fov_deg) / 2)

    # ----- projection
    def set_bounds(self, center, radius: float) -> None:
        """The sphere holding everything drawn: the orthographic clip planes enclose it."""
        self.bounds = (np.asarray(center, float).copy(), float(radius))

    def clip_planes(self) -> tuple[float, float]:
        """(near, far) eye distances. Perspective: pyqtgraph's (distance x 1e-3,
        x 1e3). Orthographic: the bounding sphere's depth +- its radius (near
        may be negative: there is no eye point to stay in front of)."""
        if self.perspective:
            return self.distance * 1e-3, self.distance * 1e3
        if self.bounds is None:
            c, r = self.center, 1e3 * self.distance
        else:
            c, r = self.bounds
        d = -(self.view_matrix() @ np.r_[c, 1.0])[2]
        r = 1.05 * r + 1.0
        return d - r, d + r

    def projection_matrix(self, width_px: float, height_px: float, region=None) -> np.ndarray:
        """4x4 eye -> clip for a ``width_px`` x ``height_px`` viewport. ``region``
        (x, y, w, h) px restricts it to that part of the viewport (pyqtgraph's
        picking), as ``GLViewWidget.projectionMatrix(region, viewport)`` does."""
        w, h = max(float(width_px), 1.0), max(float(height_px), 1.0)
        x0, y0, rw, rh = region if region is not None else (0.0, 0.0, w, h)
        near, far = self.clip_planes()
        r = (near if self.perspective else self.distance) * math.tan(math.radians(self.fov_deg) / 2)
        t = r * h / w
        lft, rgt = r * (x0 * 2 / w - 1), r * ((x0 + rw) * 2 / w - 1)
        bot, top = t * (y0 * 2 / h - 1), t * ((y0 + rh) * 2 / h - 1)
        m = np.zeros((4, 4))
        if self.perspective:                                    # glFrustum
            m[0, 0], m[0, 2] = 2 * near / (rgt - lft), (rgt + lft) / (rgt - lft)
            m[1, 1], m[1, 2] = 2 * near / (top - bot), (top + bot) / (top - bot)
            m[2, 2], m[2, 3] = -(far + near) / (far - near), -2 * far * near / (far - near)
            m[3, 2] = -1.0
        else:                                                   # glOrtho
            m[0, 0], m[0, 3] = 2 / (rgt - lft), -(rgt + lft) / (rgt - lft)
            m[1, 1], m[1, 3] = 2 / (top - bot), -(top + bot) / (top - bot)
            m[2, 2], m[2, 3] = -2 / (far - near), -(far + near) / (far - near)
            m[3, 3] = 1.0
        return m

    def project(self, p, width_px: float, height_px: float) -> tuple[float, float] | None:
        """Pixel (x right, y down) of the world point ``p`` in a ``width_px`` x
        ``height_px`` view; None behind the camera (perspective only)."""
        e = self.view_matrix() @ np.r_[np.asarray(p, float), 1.0]
        if self.perspective and e[2] > -1e-6:
            return None
        c = self.projection_matrix(width_px, height_px) @ e
        x, y = c[0] / c[3], c[1] / c[3]
        return 0.5 * width_px * (1 + x), 0.5 * height_px * (1 - y)

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
