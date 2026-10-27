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

Mapping, calibration and bend model are PLACEHOLDERS (see dome_pose).

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

The status bar shows the measured frame rate (paints per second) and the mean
per-frame times: pose (bus read + pose model + tube geometry), upload
(handing the arrays to pyqtgraph; the GL buffer upload itself happens in the
paint) and draw (paintGL; with WAVI_DOME_FINISH=1 it ends with glFinish, so it
includes the GPU), plus the CPU load of the GUI thread (everything wavi_py draws, all windows)
and of the whole process (all threads: DAQ reader, software GL if any).
"""

from __future__ import annotations

import ctypes
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QAction, QGuiApplication, QOffscreenSurface, QOpenGLContext
from PyQt6.QtWidgets import QCheckBox, QComboBox, QDoubleSpinBox, QLabel, QSpinBox, QToolBar


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
from wavi_py.views.whisker_plots import _ElidedLabel    # noqa: E402
from whisker_link.dome_pose import (DomeLayout, DomePose, Shell, TrailBuffer,  # noqa: E402
                                    nice_grid_step)

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


class _GLView(gl.GLViewWidget):
    """GLViewWidget that times its paints."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.paint_s = self.paint_cpu_s = 0.0
        self.paints = 0
        self.finish = os.environ.get("WAVI_DOME_FINISH", "") not in ("", "0")

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
    def __init__(self, fs: float = 62.5, target_fps: float = 60.0, asset_dir: Path = ASSET_DIR):
        super().__init__()
        self.setWindowTitle("Wavi - Dome")
        self.resize(900, 700)
        self._fs = float(fs)
        self._bus = None
        self._cursor = 0
        self._paused = False
        self._gain = 3.0
        self._tubes = True
        self._show_shell = True
        self._centre_pivot = True
        self._trail_s = 2.0
        self._grid_bend_deg = 0.0              # 0: auto
        self._shell_radius = 0.0               # 0: auto (rest tips' reach)

        self.layout_ = DomeLayout.load(asset_dir / "whiskers.json")
        self.pose = DomePose(self.layout_)
        n_pts = N_ROOT + N_SHAFT + 1
        self._tube_faces = tube_faces(len(self.layout_), n_pts, TUBE_SIDES)
        self._lut = _CMAP.getLookupTable(0.0, 1.0, 256, alpha=True) / 255.0
        self.shell = Shell.around(self.layout_, self._shell_radius)
        self.trail = TrailBuffer(len(self.layout_), TRAIL_MAX_S, TRAIL_RATE_HZ)

        self._glw = _GLView()
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
        for it in (self._grid_item, self._trail_item, self._surface_item):
            self._glw.addItem(it)
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
        self._render_pose()

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

    def reset_camera(self) -> None:
        c = self.layout_.center - [0.0, 0.0, 0.2 * self.shell.radius]
        self._glw.setCameraPosition(pos=pg.Vector(*c), distance=4.4 * self.shell.radius,
                                    elevation=25, azimuth=-60)

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
        tb.addSeparator()
        tb.addWidget(QLabel(" fps "))
        self._fps_box = QSpinBox()
        self._fps_box.setRange(5, 240)
        self._fps_box.setValue(int(target_fps))
        self._fps_box.setToolTip("Target render rate (frames per second)")
        self._fps_box.valueChanged.connect(self.set_target_fps)
        tb.addWidget(self._fps_box)
        view = QAction("Reset view", self)
        view.triggered.connect(self.reset_camera)
        tb.addAction(view)

    def _build_shell_toolbar(self) -> None:
        self.addToolBarBreak()
        tb = QToolBar("Shell")
        self.addToolBar(tb)
        self._shell_box = QCheckBox("Shell")
        self._shell_box.setChecked(self._show_shell)
        self._shell_box.setToolTip("Scribe shell: grid on an imaginary sphere + cylinder"
                                   " concentric with the dome")
        self._shell_box.toggled.connect(self.set_shell_visible)
        tb.addWidget(self._shell_box)
        self._surface_box = QCheckBox("Surface")
        self._surface_box.setToolTip("Also a faint translucent shell surface")
        self._surface_box.toggled.connect(self.set_surface_visible)
        tb.addWidget(self._surface_box)
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
        tb.addWidget(QLabel(" Grid "))
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
        self._grid_item.setVisible(self._show_shell)

    def set_shell_visible(self, on: bool) -> None:
        self._show_shell = bool(on)
        self._grid_item.setVisible(self._show_shell)
        self._surface_item.setVisible(self._show_shell and self._surface_box.isChecked())

    def set_surface_visible(self, on: bool) -> None:
        self._surface_item.setVisible(bool(on) and self._show_shell)

    def set_centre_pivot(self, on: bool) -> None:
        self._centre_pivot = bool(on)
        self.trail.clear()
        self._render_pose()

    def set_trail_s(self, secs: float) -> None:
        self._trail_s = float(secs)
        self._trail_item.setVisible(self._trail_s > 0)

    def set_grid_step(self, bend_deg: float) -> None:
        self._grid_bend_deg = float(bend_deg)
        self._update_grid()

    def set_shell_radius(self, r: float) -> None:
        self._shell_radius = float(r)
        self.shell = Shell.around(self.layout_, self._shell_radius)
        self._surface_item.setMeshData(meshdata=shell_surface(self.shell))
        self.trail.clear()
        self._update_grid()
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
        self.trail.clear()                     # the old trail was drawn at the old gain
        self._update_grid()
        self._render_pose()

    def set_tubes(self, on: bool) -> None:
        self._tubes = bool(on)
        self._tube_item.setVisible(self._tubes)
        self._line_item.setVisible(not self._tubes)
        self._render_pose()

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

    def _render_pose(self, t0: Optional[float] = None, now: Optional[float] = None) -> None:
        """Draw the latest pose; ``now`` (s, monotonic by default) stamps the trail."""
        t0 = time.perf_counter() if t0 is None else t0
        now = time.monotonic() if now is None else now
        pts, mag = self.pose.shape(self._gain, self.shell if self._centre_pivot else None,
                                   n_root=N_ROOT, n_shaft=N_SHAFT)
        col = self._colors(mag)                                  # (H, 4)
        n_pts = pts.shape[1]
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
        t2 = time.perf_counter()
        self.stats.frames += 1
        self.stats.pose_s += t1 - t0
        self.stats.upload_s += t2 - t1
        self._glw.update()
