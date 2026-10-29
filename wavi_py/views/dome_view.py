"""Dome view: the whiskers bending live on the whisker dome (pyqtgraph.opengl).

The dome is one static shaded ``GLMeshItem`` uploaded once (the asset from
``wavi_py/tools/export_dome_assets.py``). All whiskers together are ONE item,
rewritten in place each frame: a tube mesh (``GLMeshItem``, ring vertices and
normals computed in numpy, fixed faces) or, with Tubes off, one
``GLLinePlotItem`` of segments. Colour = bend magnitude (after the display
gain), grey = no data (unmapped hole, railed gauge, tare collecting).

The view reads the client's ``SampleBus`` with its own cursor on its own
QTimer (default 60 Hz, the fps box), so it renders at its own rate whatever the
scan rate and the control panel's 20 Hz tick; each frame takes the scans that
arrived since the last one into the pose model (``whisker_link.dome_pose``:
tare, smoothing, calibration, shape) and draws the latest smoothed pose. The
timer stops while the window is hidden or minimised, and frames are skipped
while paused.

Mapping, calibration and bend model are PLACEHOLDERS (see dome_pose), unless
a dome map is given (``whisker_link.dome_map``, ``set_dome_map``): then the
holes are the map's positions (its coordinates, labelled e.g. "LFU (W7)" on the
view), each hole reads the map's drag and lift gauges with their polarity, and
each sensor's calibration comes from the map's calibration file (the
placeholder gain where there is none).

Scribe shell (second toolbar row): an imaginary sphere + cylinder concentric
with the dome (``dome_pose.Shell``) drawn as one grid line item (latitude
rings and meridians on the sphere, rings and axial lines on the cylinder) and,
optionally, a faint translucent surface. Pivot "centre" turns every whisker
about the dome centre (the axis point at its height on the cylinder part), so
the tips move on the shell and a bend of phi is phi of arc there; the whisker
is drawn straight from its hole to that tip. Pivot "root" is the physical
model. Each tip leaves a trail (one line item, per-vertex alpha fading with
age) of the last Trail seconds. The grid spacing is set in degrees of bend
(auto: a 1-2-5 value near 12 deg on the shell at the current gain); the
readout next to it gives the same spacing on the shell, in degrees and mm.
Grid and Surface toggle separately. The cylinder grows downward (in R/4
steps) when a tip goes below its end, so the trails stay on the drawn shell.
Neutral puts a dot at each whisker's zero-bend tip (on the shell in centre
mode, the rest tip in root mode) and Arrows draws an arrow from it to the
current tip, along the tip's way (an arc on the shell).

Camera (``dome_camera.TrackballCamera``, replacing GLViewWidget's azimuth /
elevation orbit, which stops at the poles): left drag tumbles freely in every
direction, Shift + left drag rolls, Ctrl + left or middle drag pans, wheel or
right drag zooms. A view cube (top right, click a face) and keys 1-6 snap to
front / back / top / bottom / starboard / port, 0 to the default view; the
triad (bottom left) and the green UP arrow show the ROV frame (the scene is
rov-cad's collar frame: +z forward along the tube axis, +y up, -x starboard).
The projection is orthographic unless Perspective is ticked (fov 60 deg); the
two share the zoom, so a switch keeps the dome's size on screen.

The status bar shows the measured frame rate (paints per second) and the mean
per-frame times: pose (bus read + pose model + tube geometry), upload
(handing the arrays to pyqtgraph; the GL buffer upload itself happens in the
paint) and draw (paintGL; with WAVI_DOME_FINISH=1 it ends with glFinish, so it
includes the GPU), plus the CPU load of the GUI thread (everything wavi_py draws, all windows)
and of the whole process (all threads: DAQ reader, software GL if any).
"""

from __future__ import annotations

import ctypes
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import (QAction, QColor, QFont, QGuiApplication, QKeySequence, QMatrix4x4,
                         QOffscreenSurface, QOpenGLContext, QPainter, QPen, QPolygonF, QTransform)
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QLabel, QMenu, QSpinBox,
                             QToolBar, QToolButton, QWidget)


def _match_pyopengl_platform() -> None:
    """Point PyOpenGL at the API Qt makes its GL contexts with (Linux only).

    PyOpenGL looks the current context up through its own platform layer, GLX
    by default; where Qt made the context through EGL (Wayland, or xcb with
    the EGL integration, as under Xvfb here) that lookup finds nothing and every
    pyqtgraph draw fails with "no valid context". So, before PyOpenGL is first
    imported, make a throwaway Qt context and ask EGL whether it is current.
    """
    if not sys.platform.startswith("linux") or "PYOPENGL_PLATFORM" in os.environ \
            or "OpenGL" in sys.modules or QGuiApplication.instance() is None:
        return
    try:
        egl = ctypes.CDLL("libEGL.so.1")
    except OSError:
        return
    egl.eglGetCurrentContext.restype = ctypes.c_void_p
    surf = QOffscreenSurface()
    surf.create()
    ctx = QOpenGLContext()
    if ctx.create() and ctx.makeCurrent(surf):
        if egl.eglGetCurrentContext():
            os.environ["PYOPENGL_PLATFORM"] = "egl"
        ctx.doneCurrent()


_match_pyopengl_platform()

import pyqtgraph as pg                                   # noqa: E402
import pyqtgraph.opengl as gl                            # noqa: E402
from pyqtgraph.opengl import shaders                    # noqa: E402
from pyqtgraph.opengl.MeshData import MeshData          # noqa: E402

from wavi_py.views.base import ViewWindow               # noqa: E402
from wavi_py.views.dome_camera import (FORWARD, SNAP_KEYS, SNAP_VIEWS, STARBOARD, UP,  # noqa: E402
                                       TrackballCamera, look_rotation)
from wavi_py.views.whisker_plots import _ElidedLabel    # noqa: E402
from whisker_link.dome_map import DomeMap                 # noqa: E402
from whisker_link.dome_pose import (PLACEHOLDER_HOLE_OF, DomeLayout, DomePose, Shell,  # noqa: E402
                                    TrailBuffer, arrow_segments, centre_pivots, neutral_tips,
                                    nice_grid_step, tip_path)

ASSET_DIR = Path(__file__).resolve().parents[1] / "assets" / "dome"
TUBE_SIDES = 8
TUBE_RADIUS_MM = 1.5
N_ROOT, N_SHAFT = 6, 8                 # polyline: 15 points per whisker
COLOR_FULL_DEG = 45.0                  # colour map top (after the display gain)
_GREY = (0.6, 0.6, 0.6, 1.0)
_DOME_COLOR = (0.80, 0.82, 0.86, 1.0)
_GRID_COLOR = (0.25, 0.35, 0.55, 0.35)
_SURFACE_COLOR = (0.55, 0.65, 0.85, 0.07)
TRAIL_MAX_S = 10.0
TRAIL_RATE_HZ = 60.0                   # trail samples per second at most
ARROW_PTS = 8                          # points along each arrow's shaft
_ARROW_COLOR = (0.85, 0.25, 0.08, 1.0)
_NEUTRAL_COLOR = (0.1, 0.1, 0.12, 0.9)
_UP_COLOR = (0.1, 0.6, 0.2, 1.0)
SNAP_ANIM_S = 0.25
# corner widgets: label, colour per ROV axis
_AXES = (("Up", UP, QColor(30, 150, 50)), ("Fwd", FORWARD, QColor(40, 90, 220)),
         ("Stbd", STARBOARD, QColor(210, 50, 40)))
_FACE_LABEL = {"front": "FRONT", "back": "BACK", "top": "TOP", "bottom": "BOTTOM",
               "starboard": "STBD", "port": "PORT"}
_CMAP = pg.colormap.get("viridis")    # bend magnitude: purple straight .. yellow full

# A headlight: lit from the camera, two-sided (the CAD mesh's winding is not
# consistent everywhere). pyqtgraph's own "shaded" lights from behind-below,
# which leaves everything facing the camera dark.
_HEADLIGHT = shaders.ShaderProgram("wavi_headlight", [
    shaders.VertexShader("""
        uniform mat4 u_mvp;
        uniform mat3 u_normal;
        attribute vec4 a_position;
        attribute vec3 a_normal;
        attribute vec4 a_color;
        varying vec4 v_color;
        varying vec3 v_normal;
        void main() {
            v_normal = normalize(u_normal * a_normal);
            v_color = a_color;
            gl_Position = u_mvp * a_position;
        }
    """),
    shaders.FragmentShader("""
        #ifdef GL_ES
        precision mediump float;
        #endif
        varying vec4 v_color;
        varying vec3 v_normal;
        void main() {
            float p = abs(dot(normalize(v_normal), normalize(vec3(0.3, 0.4, 1.0))));
            gl_FragColor = vec4(v_color.rgb * (0.3 + 0.75 * p), v_color.a);
        }
    """),
])


class _MeshData(MeshData):
    """MeshData with normals given, not recomputed: pyqtgraph's own vertex
    normals loop over the vertices in Python, too slow to run every frame."""

    def set_normals(self, normals: np.ndarray) -> None:
        self._vertexNormals = np.ascontiguousarray(normals, dtype=np.float32)


def tube_faces(n_whisker: int, n_pts: int, sides: int) -> np.ndarray:
    """Triangles of ``n_whisker`` tubes, vertices laid out (H, S, K)."""
    h, s, k = np.meshgrid(np.arange(n_whisker), np.arange(n_pts - 1), np.arange(sides),
                          indexing="ij")
    v = lambda hh, ss, kk: (hh * n_pts + ss) * sides + kk % sides   # noqa: E731
    a, b = v(h, s, k), v(h, s, k + 1)
    c, d = v(h, s + 1, k), v(h, s + 1, k + 1)
    return np.concatenate([np.stack([a, b, d], -1).reshape(-1, 3),
                           np.stack([a, d, c], -1).reshape(-1, 3)]).astype(np.uint32)


def tube_vertices(pts: np.ndarray, radius: float, sides: int,
                  plane_normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(vertices, normals), both (H*S*K, 3), of tubes round planar polylines
    ``pts`` (H, S, 3). Each whisker bends in one plane (normal ``plane_normal``
    (H, 3)), so that normal is a twist-free binormal all along it."""
    t = np.gradient(pts, axis=1)
    t /= np.maximum(np.linalg.norm(t, axis=2, keepdims=True), 1e-12)
    b = np.broadcast_to(plane_normal[:, None, :], t.shape)
    nrm = np.cross(b, t)
    ang = np.linspace(0, 2 * np.pi, sides, endpoint=False)
    ring = (np.cos(ang)[None, None, :, None] * nrm[:, :, None, :]
            + np.sin(ang)[None, None, :, None] * b[:, :, None, :])          # (H, S, K, 3)
    verts = pts[:, :, None, :] + radius * ring
    return verts.reshape(-1, 3), ring.reshape(-1, 3)


def shell_surface(shell: Shell, n_az: int = 72, n_pol: int = 24, n_z: int = 6) -> MeshData:
    """A triangle mesh of the shell (hemisphere + cylinder), for the faint surface."""
    R, c = shell.radius, shell.center
    pol = np.r_[np.linspace(0, np.pi / 2, n_pol + 1)]
    az = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
    sph = np.stack([np.sin(pol)[:, None] * np.cos(az), np.sin(pol)[:, None] * np.sin(az),
                    np.cos(pol)[:, None] + 0 * az], -1) * R
    z = -np.linspace(0, shell.cyl_len, n_z + 1)[1:]
    cyl = np.stack([R * np.cos(az) + 0 * z[:, None], R * np.sin(az) + 0 * z[:, None],
                    z[:, None] + 0 * az], -1)
    rows = np.concatenate([sph, cyl]) + c                      # (Nr, n_az, 3)
    nr = len(rows)
    i, j = np.meshgrid(np.arange(nr - 1), np.arange(n_az), indexing="ij")
    a, b = i * n_az + j, i * n_az + (j + 1) % n_az
    f = np.concatenate([np.stack([a, b, b + n_az], -1).reshape(-1, 3),
                        np.stack([a, b + n_az, a + n_az], -1).reshape(-1, 3)])
    return MeshData(vertexes=rows.reshape(-1, 3).astype(np.float32), faces=f.astype(np.uint32))


class _TranslucentMesh(gl.GLMeshItem):
    """Blended, depth-tested but not depth-writing, drawn after the opaque
    items (depth value); restores the depth mask so the next clear works."""

    def paint(self):
        from OpenGL import GL
        GL.glDepthMask(GL.GL_FALSE)
        try:
            super().paint()
        finally:
            GL.glDepthMask(GL.GL_TRUE)


class ViewCube(QWidget):
    """Corner view cube: turns with the camera; a click on a face emits
    ``clicked(view name)``. Painted with QPainter over the GL view, only when
    the camera moves."""

    clicked = pyqtSignal(str)
    SIZE = 100

    def __init__(self, glw: "_GLView"):
        super().__init__(glw)
        self._glw = glw
        self._hover: Optional[str] = None
        self.setFixedSize(self.SIZE, self.SIZE)
        self.setMouseTracking(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Click a face to look at it (keys: "
                        + ", ".join(f"{k} {n}" for k, n in SNAP_KEYS.items()) + ", 0 default)")
        # face corners in the face's own (right, up) as seen when snapped to it
        self._quads = {}
        for name, (n, up) in SNAP_VIEWS.items():
            r = look_rotation(n, up)
            self._quads[name] = np.array([n + a * r[0] + b * r[1]
                                          for a, b in ((-1, 1), (1, 1), (1, -1), (-1, -1))])

    def faces(self) -> list[tuple[str, np.ndarray, float]]:
        """Faces toward the camera: (name, screen quad (4, 2) px, facing 0..1)."""
        cam = self._glw.cam
        s, c = 0.27 * self.SIZE, 0.5 * self.SIZE
        out = []
        for name, (n, _) in SNAP_VIEWS.items():
            facing = float(cam.to_view(n)[2])
            if facing > 1e-6:
                v = cam.to_view(self._quads[name])
                out.append((name, np.c_[c + s * v[:, 0], c - s * v[:, 1]], facing))
        return out

    def face_at(self, x: float, y: float) -> Optional[str]:
        for name, q, _ in self.faces():
            if QPolygonF([QPointF(*p) for p in q]).containsPoint(
                    QPointF(x, y), Qt.FillRule.OddEvenFill):
                return name
        return None

    def paintEvent(self, _ev) -> None:  # noqa: N802 — Qt naming
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        font = QFont(self.font())
        font.setPixelSize(26)
        font.setBold(True)
        for name, q, facing in self.faces():
            poly = QPolygonF([QPointF(*pt) for pt in q])
            base = QColor(255, 210, 120) if name == self._hover else QColor(205, 215, 235)
            p.setTransform(QTransform())
            p.setPen(QPen(QColor(60, 70, 90), 1.2))
            p.setBrush(base.darker(int(100 + 25 * (1 - facing))))
            p.drawPolygon(poly)
            if facing < 0.25:
                continue
            t = QTransform()
            square = QPolygonF([QPointF(-50, -50), QPointF(50, -50), QPointF(50, 50),
                                QPointF(-50, 50)])
            if QTransform.quadToQuad(square, poly, t):
                p.setTransform(t)
                p.setFont(font)
                p.setPen(QColor(30, 35, 50))
                p.drawText(QRectF(-50, -50, 100, 100), Qt.AlignmentFlag.AlignCenter,
                           _FACE_LABEL[name])
        p.end()

    def mousePressEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        name = self.face_at(ev.position().x(), ev.position().y())
        if name is None or ev.button() != Qt.MouseButton.LeftButton:
            ev.ignore()                        # a drag that starts here turns the scene
            return
        self.clicked.emit(name)

    def mouseMoveEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        name = self.face_at(ev.position().x(), ev.position().y())
        if name != self._hover:
            self._hover = name
            self.update()
        ev.ignore()

    def leaveEvent(self, ev) -> None:  # noqa: N802 — Qt naming
        self._hover = None
        self.update()


class AxisTriad(QWidget):
    """Corner triad of the ROV's axes (Up, Fwd, Stbd) as the camera sees them."""

    SIZE = 110

    def __init__(self, glw: "_GLView"):
        super().__init__(glw)
        self._glw = glw
        self.setFixedSize(self.SIZE, self.SIZE)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)

    def paintEvent(self, _ev) -> None:  # noqa: N802 — Qt naming
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        font = QFont(self.font())
        font.setPixelSize(12)
        font.setBold(True)
        p.setFont(font)
        c, s = self.SIZE / 2, 0.24 * self.SIZE
        axes = [(name, self._glw.cam.to_view(v), col) for name, v, col in _AXES]
        for name, v, col in sorted(axes, key=lambda a: a[1][2]):      # back to front
            p.setPen(QPen(col, 3 if v[2] >= 0 else 2, Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.RoundCap))
            p.setBrush(Qt.BrushStyle.NoBrush)
            flat = math.hypot(v[0], v[1])
            if flat < 0.3:                     # along the view: a dot (toward) or a cross (away)
                p.drawEllipse(QPointF(c, c), 5, 5)
                if v[2] > 0:
                    p.drawPoint(QPointF(c, c))
                else:
                    p.drawLine(QPointF(c - 3, c - 3), QPointF(c + 3, c + 3))
                    p.drawLine(QPointF(c - 3, c + 3), QPointF(c + 3, c - 3))
                d = (0.7, -0.7) if flat < 1e-3 else (v[0] / flat, -v[1] / flat)
                at = QPointF(c + 18 * d[0], c + 18 * d[1])
            else:
                p.drawLine(QPointF(c, c), QPointF(c + s * v[0], c - s * v[1]))
                at = QPointF(c + (s + 13) * v[0] / max(flat, 1.0),
                             c - (s + 13) * v[1] / max(flat, 1.0))
            p.drawText(QRectF(at.x() - 20, at.y() - 8, 40, 16), Qt.AlignmentFlag.AlignCenter, name)
        p.end()


class _GLView(gl.GLViewWidget):
    """GLViewWidget with a trackball camera (free tumble), the view cube and the
    axis triad, and timed paints."""

    camera_moved = pyqtSignal()

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.paint_s = self.paint_cpu_s = 0.0
        self.paints = 0
        self.finish = os.environ.get("WAVI_DOME_FINISH", "") not in ("", "0")
        self.cam = TrackballCamera(fov_deg=self.opts["fov"])
        self._mouse = None
        self.cube = ViewCube(self)
        self.triad = AxisTriad(self)
        self.camera_moved.connect(self.cube.update)
        self.camera_moved.connect(self.triad.update)

    # ----- camera (GLViewWidget's own state is not used)
    def viewMatrix(self):  # noqa: N802 — pyqtgraph naming
        self.opts["distance"] = self.cam.distance
        return QMatrix4x4(*self.cam.view_matrix().ravel().tolist())

    def projectionMatrix(self, region, viewport):  # noqa: N802 — pyqtgraph naming
        """Perspective or orthographic (``cam.perspective``); ``region`` is a
        sub-rectangle of ``viewport`` (x, y, w, h, device px) when picking."""
        x0, y0, w, h = viewport
        sub = (region[0] - x0, region[1] - y0, region[2], region[3])
        return QMatrix4x4(*self.cam.projection_matrix(w, h, sub).ravel().tolist())

    def cameraPosition(self):  # noqa: N802 — pyqtgraph naming
        return pg.Vector(*self.cam.position())

    def orbit(self, azim, elev):
        """pyqtgraph's arrow keys: azimuth / elevation steps, here a tumble."""
        k = self.cam.deg_per_px
        self.cam.tumble(-azim / k, elev / k)
        self.moved()

    def pan(self, dx, dy, dz=0.0, relative="view"):
        self.cam.pan(dx, dy, self.width())
        self.moved()

    def moved(self) -> None:
        self.update()
        self.camera_moved.emit()

    def project(self, p) -> Optional[tuple[float, float]]:
        """Widget pixel (x, y) of the world point ``p``; None behind the camera."""
        return self.cam.project(p, self.width(), self.height())

    def mousePressEvent(self, ev):  # noqa: N802 — Qt naming
        self._mouse = ev.position()

    def mouseMoveEvent(self, ev):  # noqa: N802 — Qt naming
        pos = ev.position()
        if self._mouse is None:
            self._mouse = pos
        dx, dy = pos.x() - self._mouse.x(), pos.y() - self._mouse.y()
        self._mouse = pos
        btn, mod = ev.buttons(), ev.modifiers()
        if btn == Qt.MouseButton.LeftButton:
            if mod & Qt.KeyboardModifier.ControlModifier:
                self.cam.pan(dx, dy, self.width())
            elif mod & Qt.KeyboardModifier.ShiftModifier:
                self.cam.roll(-dx * self.cam.deg_per_px)
            else:
                self.cam.tumble(dx, dy)
        elif btn == Qt.MouseButton.MiddleButton:
            self.cam.pan(dx, dy, self.width())
        elif btn == Qt.MouseButton.RightButton:
            self.cam.zoom(1.005 ** dy)
        else:
            return
        self.moved()

    def mouseReleaseEvent(self, ev):  # noqa: N802 — Qt naming
        self._mouse = None

    def wheelEvent(self, ev):  # noqa: N802 — Qt naming
        delta = ev.angleDelta().y() or ev.angleDelta().x()
        self.cam.zoom(0.999 ** delta)
        self.moved()

    def resizeEvent(self, ev):  # noqa: N802 — Qt naming
        super().resizeEvent(ev)
        self.cube.move(self.width() - self.cube.width() - 8, 8)
        self.triad.move(8, self.height() - self.triad.height() - 8)
        self.camera_moved.emit()               # labels placed in pixels

    def paintGL(self, *a, **kw):  # noqa: N802 — Qt naming
        t0, c0 = time.perf_counter(), time.thread_time()
        super().paintGL(*a, **kw)
        if self.finish:
            from OpenGL import GL
            GL.glFinish()
        self.paint_s += time.perf_counter() - t0
        self.paint_cpu_s += time.thread_time() - c0
        self.paints += 1


class FrameStats:
    """Rolling per-second averages for the status bar (and benchmarks)."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.t0 = time.perf_counter()
        self.cpu0 = time.process_time()
        self.main0 = time.thread_time()       # roll() runs on the GUI thread
        self.frames = 0
        self.pose_s = self.upload_s = 0.0
        self.last: dict = {}

    def roll(self, glw: _GLView) -> Optional[dict]:
        dt = time.perf_counter() - self.t0
        if dt < 1.0:
            return None
        n = max(self.frames, 1)
        p = max(glw.paints, 1)
        self.last = {"fps": glw.paints / dt, "updates_per_s": self.frames / dt,
                     "pose_ms": 1e3 * self.pose_s / n, "upload_ms": 1e3 * self.upload_s / n,
                     "draw_ms": 1e3 * glw.paint_s / p,
                     "draw_cpu_ms": 1e3 * glw.paint_cpu_s / p,
                     "cpu_pct": 100 * (time.process_time() - self.cpu0) / dt,
                     "main_pct": 100 * (time.thread_time() - self.main0) / dt}
        glw.paint_s, glw.paint_cpu_s, glw.paints = 0.0, 0.0, 0
        self.reset_keep_last()
        return self.last

    def reset_keep_last(self) -> None:
        last = self.last
        self.reset()
        self.last = last


class DomeView(ViewWindow):
    def __init__(self, fs: float = 62.5, target_fps: float = 60.0, asset_dir: Path = ASSET_DIR,
                 dome_map: Optional[DomeMap] = None):
        super().__init__()
        self.setWindowTitle("Wavi - Dome")
        self.resize(900, 700)
        self._fs = float(fs)
        self._bus = None
        self._cursor = 0
        self._paused = False
        self._gain = 3.0
        self._tubes = True
        self._perspective = False
        self._show_grid = True
        self._show_neutral = True
        self._show_arrows = True
        self._centre_pivot = True
        self._trail_s = 2.0
        self._grid_bend_deg = 0.0              # 0: auto
        self._shell_radius = 0.0               # 0: auto (rest tips' reach)

        self._asset_layout = DomeLayout.load(asset_dir / "whiskers.json")
        self.dome_map = dome_map
        self._show_labels = dome_map is not None
        self._hole_labels: list[QLabel] = []
        self._make_pose()
        self._lut = _CMAP.getLookupTable(0.0, 1.0, 256, alpha=True) / 255.0

        self._glw = _GLView()
        self._glw.cam.perspective = self._perspective
        self._glw.setBackgroundColor((245, 245, 248))
        self.setCentralWidget(self._glw)
        self._add_dome(asset_dir / "dome_mesh.npz")
        self._tube_item = gl.GLMeshItem(meshdata=_MeshData(), smooth=True, shader=_HEADLIGHT,
                                        glOptions="opaque")
        self._line_item = gl.GLLinePlotItem(mode="lines", width=2.0, antialias=True,
                                            glOptions="opaque")
        self._line_item.setVisible(False)
        self._glw.addItem(self._tube_item)
        self._glw.addItem(self._line_item)
        # shell grid, trails and the optional surface: blended, drawn after the opaque items
        self._grid_item = gl.GLLinePlotItem(mode="lines", width=1.0, antialias=False,
                                            glOptions="translucent")
        self._grid_item.setDepthValue(10)
        self._trail_item = gl.GLLinePlotItem(mode="lines", width=2.0, antialias=True,
                                             glOptions="translucent")
        self._trail_item.setDepthValue(11)
        self._surface_item = _TranslucentMesh(meshdata=shell_surface(self.shell), smooth=False,
                                              shader=None, color=_SURFACE_COLOR,
                                              glOptions="translucent")
        self._surface_item.setDepthValue(12)
        self._surface_item.setVisible(False)
        # neutral dots, deflection arrows, the UP marker
        self._neutral_item = gl.GLScatterPlotItem(size=8.0, color=_NEUTRAL_COLOR, pxMode=True,
                                                  glOptions="translucent")
        self._neutral_item.setDepthValue(13)
        self._arrow_item = gl.GLLinePlotItem(mode="lines", width=2.5, antialias=True,
                                             glOptions="translucent")
        self._arrow_item.setDepthValue(14)
        self._up_item = gl.GLLinePlotItem(mode="lines", width=3.0, antialias=True,
                                          color=_UP_COLOR, glOptions="opaque")
        for it in (self._grid_item, self._trail_item, self._surface_item, self._neutral_item,
                   self._arrow_item, self._up_item):
            self._glw.addItem(it)
        # their labels: plain Qt labels over the GL view, moved only when the camera moves
        # (pyqtgraph's GLTextItem paints nothing here under Mesa / EGL)
        self._marks: dict[str, tuple[QLabel, np.ndarray]] = {}
        for text, col in (("UP", "#1e9632"), ("FWD", "#285adc")):
            lab = QLabel(text, self._glw)
            lab.setStyleSheet(f"color: {col}; font-weight: bold; font-size: 13px;"
                              " background: transparent;")
            lab.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            lab.adjustSize()
            self._marks[text] = (lab, np.zeros(3))
        self._glw.camera_moved.connect(self._place_marks)
        self._make_hole_labels()
        self._update_marks()
        self._glw.cube.clicked.connect(self.snap_view)
        self.reset_camera()

        self._build_toolbar(target_fps)
        self._build_shell_toolbar()
        self._stats_label = QLabel()
        self.statusBar().addWidget(self._stats_label)
        self._stream_status = _ElidedLabel()
        self.statusBar().addPermanentWidget(self._stream_status, 1)
        self.stats = FrameStats()

        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._on_frame)
        self.set_target_fps(target_fps)
        self._update_grid()
        self._update_neutral()
        self._render_pose()

    # ----- holes: the asset's (placeholder mapping) or a dome map's positions
    def _make_pose(self) -> None:
        """Layout, pose model and everything sized by the hole count, from ``dome_map``."""
        m = self.dome_map
        if m is None:
            self.layout_ = self._asset_layout
            self.pose = DomePose(self.layout_)
        else:
            a = self._asset_layout
            self.layout_ = m.layout(a.center, a.dome_radius)
            self.pose = DomePose(self.layout_, calibration=m.calibration(),
                                 hole_channels=m.hole_channels())
        self._tube_faces = tube_faces(len(self.layout_), N_ROOT + N_SHAFT + 1, TUBE_SIDES)
        self.shell = Shell.around(self.layout_, self._shell_radius)
        self.trail = TrailBuffer(len(self.layout_), TRAIL_MAX_S, TRAIL_RATE_HZ)

    def hole_texts(self) -> list[str]:
        """The label of each hole: the map's "LFU (W7)", or the placeholder's whisker."""
        if self.dome_map is not None:
            return list(self.layout_.labels)
        w_of = {h: w for w, h in PLACEHOLDER_HOLE_OF.items()}
        return [f"W{w_of[h]}?" if h in w_of else "" for h in range(len(self.layout_))]

    def _make_hole_labels(self) -> None:
        for lab in self._hole_labels:
            lab.deleteLater()
        self._hole_labels = []
        for text in self.hole_texts():
            lab = QLabel(text, self._glw)
            lab.setStyleSheet("color: #303040; font-size: 11px;"
                              " background: rgba(255, 255, 255, 150);")
            lab.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            lab.adjustSize()
            self._hole_labels.append(lab)

    def set_dome_map(self, dome_map: Optional[DomeMap]) -> None:
        """Use this map's positions and calibration (None: the placeholder
        mapping on the asset's holes); tares again."""
        drift = self.pose.tare.drift_tau_s
        self.dome_map = dome_map
        self._make_pose()
        self.pose.tare.drift_tau_s = drift
        self._labels_box.setChecked(dome_map is not None)
        self._make_hole_labels()
        self._shell_changed()
        self._update_neutral()
        self._render_pose()

    def set_labels_visible(self, on: bool) -> None:
        self._show_labels = bool(on)
        self._place_marks()

    # ----- scene
    def _add_dome(self, path: Path) -> None:
        z = np.load(path)
        V = z["vertices"].astype(np.float32)
        F = z["faces"].astype(np.uint32)
        md = _MeshData(vertexes=V, faces=F)
        md.set_normals(z["normals"])                    # sharp at the CAD edges
        self._dome_item = gl.GLMeshItem(meshdata=md, smooth=True, shader=_HEADLIGHT,
                                        color=_DOME_COLOR, glOptions="opaque")
        self._glw.addItem(self._dome_item)
        self._dome_center = V.mean(axis=0)
        self._dome_reach = float(np.linalg.norm(V - self.layout_.center, axis=1).max())

    def reset_camera(self) -> None:
        """The default 3/4 view (key 0): centred, the whole shell in view."""
        cam = self._glw.cam
        cam.center = self.layout_.center - [0.0, 0.0, 0.2 * self.shell.radius]
        cam.distance = 4.4 * self.shell.radius
        cam.snap_to("home")
        self._glw.moved()

    def snap_view(self, name: str) -> None:
        """Turn (animated) to one of the ``dome_camera.SNAP_VIEWS``, or "home"."""
        self._glw.cam.snap_to(name, animate_s=SNAP_ANIM_S, now=time.monotonic())
        self._glw.moved()

    def _update_marks(self) -> None:
        """The UP arrow above the shell's equator and the FWD label at its pole."""
        R, c = self.shell.radius, self.layout_.center
        a, b = c + 1.04 * R * UP, c + 1.32 * R * UP
        h = 0.06 * R
        seg = [a, b, b, b - h * UP + 0.6 * h * FORWARD, b, b - h * UP - 0.6 * h * FORWARD]
        self._up_item.setData(pos=np.array(seg, np.float32))
        self._update_bounds()
        self._marks["UP"] = (self._marks["UP"][0], b + 0.04 * R * UP)
        self._marks["FWD"] = (self._marks["FWD"][0], c + 1.06 * R * FORWARD)
        self._place_marks()

    def _update_bounds(self) -> None:
        """The sphere round everything drawn (the orthographic clip planes):
        dome, full-length whiskers bent any way, shell with its cylinder, UP mark."""
        lay = self.layout_
        whiskers = (np.linalg.norm(lay.root - lay.center, axis=1) + lay.length).max()
        off = float(np.linalg.norm(self.shell.center - lay.center))
        shell = math.hypot(self.shell.radius, off + self.shell.cyl_len)
        self._glw.cam.set_bounds(lay.center, max(self._dome_reach, whiskers, shell,
                                                 off + 1.5 * self.shell.radius))

    def _place_marks(self) -> None:
        marks = list(self._marks.values())
        neutral = getattr(self, "_neutral", None)
        if neutral is not None and len(neutral) == len(self._hole_labels):
            # just outside each neutral tip, away from the dome
            out = neutral - self.layout_.center
            out /= np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-9)
            at = neutral + 0.08 * self.shell.radius * out
            for lab, p in zip(self._hole_labels, at):
                if self._show_labels:
                    marks.append((lab, p))
                else:
                    lab.setVisible(False)
        for lab, at in marks:
            xy = self._glw.project(at)
            lab.setVisible(xy is not None)
            if xy is not None:
                lab.move(int(xy[0] - lab.width() / 2), int(xy[1] - lab.height() / 2))

    def _build_toolbar(self, target_fps: float) -> None:
        tb = QToolBar("Dome")
        self.addToolBar(tb)
        tare = QAction("Tare", self)
        tare.setToolTip("Take the current gauge readings as zero (also done at connect)")
        tare.triggered.connect(self.tare)
        tb.addAction(tare)
        self._drift = QCheckBox("Drift removal")
        self._drift.setToolTip("Let the zero follow slow drift (60 s time constant);"
                               " a held bend fades out too")
        self._drift.toggled.connect(lambda on: setattr(self.pose.tare, "drift_tau_s",
                                                       60.0 if on else 0.0))
        tb.addWidget(self._drift)
        tb.addSeparator()
        tb.addWidget(QLabel(" Gain "))
        self._gain_box = QDoubleSpinBox()
        self._gain_box.setRange(0.1, 1000.0)       # high: still-whisker noise is ~0.03 deg
        self._gain_box.setSingleStep(0.5)
        self._gain_box.setValue(self._gain)
        self._gain_box.setSuffix(" x")
        self._gain_box.setToolTip("Display exaggeration of the bend angles")
        self._gain_box.valueChanged.connect(self._set_gain)
        tb.addWidget(self._gain_box)
        self._tubes_box = QCheckBox("Tubes")
        self._tubes_box.setChecked(True)
        self._tubes_box.setToolTip("Shaded tubes, or lines (cheaper)")
        self._tubes_box.toggled.connect(self.set_tubes)
        tb.addWidget(self._tubes_box)
        self._persp_box = QCheckBox("Perspective")
        self._persp_box.setChecked(self._perspective)
        self._persp_box.setToolTip("Perspective projection (fov 60 deg); off: orthographic")
        self._persp_box.toggled.connect(self.set_perspective)
        tb.addWidget(self._persp_box)
        tb.addSeparator()
        tb.addWidget(QLabel(" fps "))
        self._fps_box = QSpinBox()
        self._fps_box.setRange(5, 240)
        self._fps_box.setValue(int(target_fps))
        self._fps_box.setToolTip("Target render rate (frames per second)")
        self._fps_box.valueChanged.connect(self.set_target_fps)
        tb.addWidget(self._fps_box)
        views = QToolButton()
        views.setText("View")
        views.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        menu = QMenu(views)
        home = QAction("Default (reset)", self)
        home.setShortcut(QKeySequence("0"))
        home.triggered.connect(self.reset_camera)
        menu.addAction(home)
        self.addAction(home)                   # shortcuts work with the menu closed
        for key, name in SNAP_KEYS.items():
            act = QAction(name.capitalize(), self)
            act.setShortcut(QKeySequence(key))
            act.triggered.connect(lambda _=False, n=name: self.snap_view(n))
            menu.addAction(act)
            self.addAction(act)
        views.setMenu(menu)
        views.setToolTip(
            "Snap the view (also the view cube, top right). Keys: 1 front, 2 back, 3 top,"
            " 4 bottom, 5 starboard, 6 port, 0 default.\nMouse: left drag tumbles (no limit),"
            " Shift+left rolls, Ctrl+left or middle pans, wheel or right drag zooms.\n"
            "ROV frame (rov-cad collar frame): forward +z (tube axis), up +y, starboard -x.")
        tb.addWidget(views)

    def _build_shell_toolbar(self) -> None:
        self.addToolBarBreak()
        tb = QToolBar("Shell")
        self.addToolBar(tb)
        self._grid_check = QCheckBox("Grid")
        self._grid_check.setChecked(self._show_grid)
        self._grid_check.setToolTip("Scribe shell grid: an imaginary sphere + cylinder"
                                    " concentric with the dome")
        self._grid_check.toggled.connect(self.set_grid_visible)
        tb.addWidget(self._grid_check)
        self._surface_box = QCheckBox("Surface")
        self._surface_box.setToolTip("A faint translucent shell surface")
        self._surface_box.toggled.connect(self.set_surface_visible)
        tb.addWidget(self._surface_box)
        self._neutral_box = QCheckBox("Neutral")
        self._neutral_box.setChecked(self._show_neutral)
        self._neutral_box.setToolTip("A dot at each whisker's zero-bend tip (on the shell in"
                                     " centre mode)")
        self._neutral_box.toggled.connect(self.set_neutral_visible)
        tb.addWidget(self._neutral_box)
        self._arrow_box = QCheckBox("Arrows")
        self._arrow_box.setChecked(self._show_arrows)
        self._arrow_box.setToolTip("An arrow from each neutral dot to the current tip")
        self._arrow_box.toggled.connect(self.set_arrows_visible)
        tb.addWidget(self._arrow_box)
        self._labels_box = QCheckBox("Labels")
        self._labels_box.setChecked(self._show_labels)
        self._labels_box.setToolTip("Each whisker's position and channel (from the dome map;"
                                    " without one, the placeholder hole's whisker, marked ?)")
        self._labels_box.toggled.connect(self.set_labels_visible)
        tb.addWidget(self._labels_box)
        tb.addWidget(QLabel(" Pivot "))
        self._pivot_box = QComboBox()
        self._pivot_box.addItems(["centre", "root"])
        self._pivot_box.setCurrentIndex(0 if self._centre_pivot else 1)
        self._pivot_box.setToolTip("centre: whiskers turn about the dome centre, tips move on"
                                   " the shell (display aid); root: the physical bend model")
        self._pivot_box.currentTextChanged.connect(lambda t: self.set_centre_pivot(t == "centre"))
        tb.addWidget(self._pivot_box)
        tb.addWidget(QLabel(" Trail "))
        self._trail_box = QDoubleSpinBox()
        self._trail_box.setRange(0.0, TRAIL_MAX_S)
        self._trail_box.setSingleStep(0.5)
        self._trail_box.setDecimals(1)
        self._trail_box.setSuffix(" s")
        self._trail_box.setSpecialValueText("off")
        self._trail_box.setValue(self._trail_s)
        self._trail_box.setToolTip("Tip trail length, fading with age (0: off)")
        self._trail_box.valueChanged.connect(self.set_trail_s)
        tb.addWidget(self._trail_box)
        tb.addWidget(QLabel(" Grid every "))
        self._grid_box = QDoubleSpinBox()
        self._grid_box.setRange(0.0, 45.0)
        self._grid_box.setDecimals(2)
        self._grid_box.setSingleStep(0.5)
        self._grid_box.setSuffix(" deg bend")
        self._grid_box.setSpecialValueText("auto")
        self._grid_box.setToolTip("Grid spacing in degrees of bend (0: auto, a 1-2-5 value)")
        self._grid_box.valueChanged.connect(self.set_grid_step)
        tb.addWidget(self._grid_box)
        self._grid_label = QLabel()
        tb.addWidget(self._grid_label)
        tb.addWidget(QLabel(" R "))
        self._radius_box = QDoubleSpinBox()
        self._radius_box.setRange(0.0, 2000.0)
        self._radius_box.setDecimals(0)
        self._radius_box.setSingleStep(10.0)
        self._radius_box.setSuffix(" mm")
        self._radius_box.setSpecialValueText("auto")
        self._radius_box.setToolTip("Shell radius (0: auto, where the tips are at rest)")
        self._radius_box.valueChanged.connect(self.set_shell_radius)
        tb.addWidget(self._radius_box)

    # ----- shell
    def grid_step_bend_deg(self) -> float:
        return self._grid_bend_deg or nice_grid_step(self._gain)

    def _update_grid(self) -> None:
        bend = self.grid_step_bend_deg()
        shell_deg = max(bend * self._gain, 1.0)        # Shell.grid's floor
        seg = self.shell.grid(np.radians(shell_deg)).reshape(-1, 3)
        self._grid_item.setData(pos=seg.astype(np.float32), color=_GRID_COLOR)
        mm = self.shell.radius * np.radians(shell_deg)
        auto = "" if self._grid_bend_deg else "auto "
        self._grid_label.setText(f" {auto}{bend:g} deg bend = {shell_deg:g} deg / {mm:.0f} mm"
                                 f" on the shell (R {self.shell.radius:.0f}) ")
        self._grid_item.setVisible(self._show_grid)

    def _shell_changed(self) -> None:
        """The shell's radius or cylinder length changed: grid, surface, marks."""
        self._surface_item.setMeshData(meshdata=shell_surface(self.shell))
        self._update_grid()
        self._update_marks()

    def _restart_trail(self) -> None:
        """Gain / pivot / radius changed: the old trail and the cylinder's growth go."""
        self.trail.clear()
        if self.shell.uncover():
            self._shell_changed()

    def set_grid_visible(self, on: bool) -> None:
        self._show_grid = bool(on)
        self._grid_item.setVisible(self._show_grid)

    def set_surface_visible(self, on: bool) -> None:
        self._surface_item.setVisible(bool(on))

    def set_neutral_visible(self, on: bool) -> None:
        self._show_neutral = bool(on)
        self._neutral_item.setVisible(self._show_neutral)

    def set_arrows_visible(self, on: bool) -> None:
        self._show_arrows = bool(on)
        self._arrow_item.setVisible(self._show_arrows)
        self._render_pose()

    def set_centre_pivot(self, on: bool) -> None:
        self._centre_pivot = bool(on)
        self._restart_trail()
        self._update_neutral()
        self._render_pose()

    def _update_neutral(self) -> None:
        self._neutral = neutral_tips(self.layout_, self.shell if self._centre_pivot else None)
        self._neutral_item.setData(pos=self._neutral.astype(np.float32))
        self._place_marks()

    def set_trail_s(self, secs: float) -> None:
        self._trail_s = float(secs)
        self._trail_item.setVisible(self._trail_s > 0)

    def set_grid_step(self, bend_deg: float) -> None:
        self._grid_bend_deg = float(bend_deg)
        self._update_grid()

    def set_shell_radius(self, r: float) -> None:
        self._shell_radius = float(r)
        self.shell = Shell.around(self.layout_, self._shell_radius)
        self.trail.clear()
        self._shell_changed()
        self._update_neutral()
        self._render_pose()

    # ----- control (main window)
    def set_bus(self, bus, fs: Optional[float] = None) -> None:
        """Start reading ``bus`` (None: stop); tares on the first scans."""
        self._bus = bus
        self._cursor = bus.total_published if bus is not None else 0
        if fs:
            self._fs = float(fs)
        self.tare()

    def set_rate(self, fs: float) -> None:
        self._fs = float(fs)

    def set_paused(self, on: bool) -> None:
        self._paused = bool(on)

    def set_stream_status(self, text: str) -> None:
        self._stream_status.setText(text)
        self._stream_status.setToolTip(text)

    def tare(self) -> None:
        self.pose.request_tare()

    def set_target_fps(self, fps: float) -> None:
        self._timer.setInterval(max(1, int(round(1000.0 / float(fps)))))

    def _set_gain(self, g: float) -> None:
        self._gain = float(g)
        self._restart_trail()                  # the old trail was drawn at the old gain
        self._update_grid()
        self._render_pose()

    def set_tubes(self, on: bool) -> None:
        self._tubes = bool(on)
        self._tube_item.setVisible(self._tubes)
        self._line_item.setVisible(not self._tubes)
        self._render_pose()

    def set_perspective(self, on: bool) -> None:
        """Perspective or orthographic; the camera's turn, centre and zoom stay."""
        self._perspective = bool(on)
        self._glw.cam.perspective = self._perspective
        self._glw.moved()

    # ----- frame
    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        self.stats.reset()
        self._timer.start()

    def hideEvent(self, event) -> None:  # noqa: N802 — Qt naming
        self._timer.stop()
        super().hideEvent(event)

    def _on_frame(self) -> None:
        if not self.isVisible() or self.isMinimized():
            return
        if self._glw.cam.step(time.monotonic()):         # a snap turning
            self._glw.moved()
        roll = self.stats.roll(self._glw)
        if roll:
            self._stats_label.setText(
                f"{roll['fps']:.0f} fps | pose {roll['pose_ms']:.2f} ms, upload "
                f"{roll['upload_ms']:.2f} ms, draw {roll['draw_ms']:.2f} ms | CPU GUI thread "
                f"{roll['main_pct']:.0f} %, process {roll['cpu_pct']:.0f} %")
        t0 = time.perf_counter()
        if self._bus is not None:
            samples, self._cursor = self._bus.read_since(self._cursor, block=False)
            if samples is not None and not self._paused:
                self.pose.feed(samples, self._fs)
        if self._paused:
            return
        self._render_pose(t0)

    def _colors(self, mag: np.ndarray) -> np.ndarray:
        """(H, 4) RGBA from bend magnitude (rad), grey for NaN."""
        x = np.degrees(np.nan_to_num(mag)) / COLOR_FULL_DEG
        c = self._lut[np.clip((x * 255).astype(int), 0, 255)]
        c[~np.isfinite(mag)] = _GREY
        return c

    def _binormals(self, pts: np.ndarray) -> np.ndarray:
        """(H, 3) unit normal of each whisker's (planar) polyline's plane."""
        lay = self.layout_
        if self._centre_pivot:                 # straight hole -> tip: any normal to it
            t = pts[:, -1] - pts[:, 0]
            b = np.cross(t, lay.e2)
            weak = np.linalg.norm(b, axis=1) < 1e-6 * np.linalg.norm(t, axis=1)
            b[weak] = np.cross(t[weak], lay.e1[weak])
        else:
            th = np.nan_to_num(self.pose.angles())
            b = th[:, :1] * lay.e2 - th[:, 1:] * lay.e1          # n x d, unnormalised
            b[np.linalg.norm(b, axis=1) < 1e-9] = lay.e2[np.linalg.norm(b, axis=1) < 1e-9]
        return b / np.linalg.norm(b, axis=1, keepdims=True)

    def _arrows(self, mag: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(segment points, colours) of the neutral -> tip arrows, float32;
        transparent for a whisker without data."""
        shell = self.shell if self._centre_pivot else None
        th = self.pose.angles() * self._gain
        path = tip_path(self.layout_, th, shell, ARROW_PTS)
        pivot = centre_pivots(self.layout_) if self._centre_pivot else self.layout_.root
        seg = arrow_segments(path, path[:, -1] - pivot, head_mm=max(6.0, 0.05 * self.shell.radius))
        n_per = seg.shape[1] * 2
        col = np.tile(np.array(_ARROW_COLOR, np.float32), (len(seg), n_per, 1))
        col[~np.isfinite(mag), :, 3] = 0.0
        return seg.reshape(-1, 3).astype(np.float32), col.reshape(-1, 4)

    def _render_pose(self, t0: Optional[float] = None, now: Optional[float] = None) -> None:
        """Draw the latest pose; ``now`` (s, monotonic by default) stamps the trail."""
        t0 = time.perf_counter() if t0 is None else t0
        now = time.monotonic() if now is None else now
        pts, mag = self.pose.shape(self._gain, self.shell if self._centre_pivot else None,
                                   n_root=N_ROOT, n_shaft=N_SHAFT)
        col = self._colors(mag)                                  # (H, 4)
        n_pts = pts.shape[1]
        if self._centre_pivot and self.shell.cover(pts[:, -1]):
            self._shell_changed()                    # the cylinder grew under a tip
        arrows = self._arrows(mag) if self._show_arrows else None
        trail = None
        if self._trail_s > 0:
            tips = np.where(np.isfinite(mag)[:, None], pts[:, -1], np.nan)
            self.trail.push(now, tips, mag)
            tp, tm, ta = self.trail.segments(now, self._trail_s)
            tc = self._colors(tm)
            tc[:, 3] = ta
            trail = (tp, tc)
        if self._tubes:
            V, N = tube_vertices(pts, TUBE_RADIUS_MM, TUBE_SIDES, self._binormals(pts))
            C = np.repeat(col, n_pts * TUBE_SIDES, axis=0)
            t1 = time.perf_counter()
            md = _MeshData(vertexes=V.astype(np.float32), faces=self._tube_faces,
                           vertexColors=C.astype(np.float32))
            md.set_normals(N)
            self._tube_item.setMeshData(meshdata=md)
        else:
            seg = np.stack([pts[:, :-1], pts[:, 1:]], axis=2).reshape(-1, 3)
            C = np.repeat(col, 2 * (n_pts - 1), axis=0)
            t1 = time.perf_counter()
            self._line_item.setData(pos=seg.astype(np.float32), color=C.astype(np.float32))
        if trail is not None:
            self._trail_item.setData(pos=trail[0].astype(np.float32),
                                     color=trail[1].astype(np.float32))
        if arrows is not None:
            self._arrow_item.setData(pos=arrows[0], color=arrows[1])
        t2 = time.perf_counter()
        self.stats.frames += 1
        self.stats.pose_s += t1 - t0
        self.stats.upload_s += t2 - t1
        self._glw.update()
