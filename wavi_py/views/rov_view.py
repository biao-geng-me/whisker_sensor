"""ROV view: the vehicle's attitude and translational acceleration from the IMU.

A BlueROV2 silhouette (``assets/rov/rov_mesh.npz``, see export_rov_mesh.py:
frame, thrusters, enclosures, buoyancy) with the whisker dome on its nose,
turned by the ATTITUDE roll / pitch (and, with Heading ticked, the yaw
relative to the heading at the last Tare) over a level grid that stays put. Arrows from the
vehicle's centre show the translational acceleration (``daq/imu_pose.py``:
RAW_IMU specific force + gravity from the attitude, minus the at-rest bias,
averaged over Smooth seconds): the resultant (orange) and, with Components,
its body-axis parts (surge blue, sway red, heave green, as the triad), at
Scale mm per 100 mg, drawn over the hull. The readout (top left) gives the numbers.

It draws whatever IMU buffer the control panel hands it (``update_view``: the
newest row; ``show_at(t)``: the row at a picked time, e.g. a timeline click
while paused live), so it works live, paused and in replay alike. Tare takes
the acceleration over the second round the shown time as zero (the ROV at
rest there); the first second of rows after a connect is taken automatically.

Same scene frame and camera as the Dome view (rov-cad's collar frame: +z
forward, +y up, +x port; trackball, view cube, keys 0-6).
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QAction, QKeySequence, QMatrix4x4
from PyQt6.QtWidgets import QCheckBox, QDoubleSpinBox, QLabel, QToolBar

from wavi_py.daq.imu_pose import FRD_TO_SCENE, ImuPose, PoseSample, body_to_ned
from wavi_py.views.base import ViewWindow
from wavi_py.views.dome_view import (ASSET_DIR, ROV_ASSET, SNAP_KEYS, _UP_COLOR, _GLView,
                                     _flat_gl, gl, rov_mesh_items)
from wavi_py.views.render_sched import scheduler
from wavi_py.views.whisker_plots import _ElidedLabel

AUTO_TARE_S = 1.0
_RESULT_COLOR = (0.95, 0.45, 0.05, 1.0)
_COMPONENT_COLORS = ((0.16, 0.35, 0.86, 1.0),      # surge: Fwd blue
                     (0.82, 0.20, 0.16, 1.0),      # sway: Stbd red
                     (0.12, 0.59, 0.20, 1.0))      # heave: green (down +)
_GRID_COLOR = (90, 95, 110, 120)


def arrow_lines(p0: np.ndarray, v: np.ndarray, head_frac: float = 0.22,
                head_min: float = 10.0, min_len: float = 1.0) -> np.ndarray:
    """Line-segment points (10, 3) of an arrow from ``p0`` along ``v``: the
    shaft and a four-line head; empty if shorter than ``min_len``."""
    n = float(np.linalg.norm(v))
    if n < max(min_len, 1e-9):
        return np.empty((0, 3))
    d = v / n
    a = np.cross(d, [0.0, 1.0, 0.0] if abs(d[1]) < 0.9 else [1.0, 0.0, 0.0])
    a /= np.linalg.norm(a)
    b = np.cross(d, a)
    tip = p0 + v
    h = min(max(head_frac * n, head_min), 0.6 * n)
    base = tip - h * d
    out = [p0, tip]
    for e in (a, -a, b, -b):
        out += [tip, base + 0.45 * h * e]
    return np.asarray(out)


class RovView(ViewWindow):
    MAX_HZ = 30.0

    def __init__(self, mesh_path: Path = ROV_ASSET, dome_dir: Path = ASSET_DIR):
        super().__init__()
        self.setWindowTitle("Wavi - ROV")
        self.resize(900, 700)
        self.pose = ImuPose()
        self._t = np.empty(0)
        self._rows = np.empty((0, 9), dtype=np.float32)
        self._t_at: Optional[float] = None          # None: the newest row
        self._auto_tare = True
        self._yaw_ref: Optional[float] = None
        self._heading = False
        self._components = True
        self._scale = 300.0                         # mm per 100 mg
        self.sample: Optional[PoseSample] = None

        self._glw = _GLView()
        self._glw.setBackgroundColor((245, 245, 248))
        self.setCentralWidget(self._glw)
        self._body: list[gl.GLMeshItem] = rov_mesh_items(mesh_path, dome_dir)
        lo, hi = self._bbox
        self.centre = (lo + hi) / 2
        self.radius = float(np.linalg.norm(hi - lo) / 2)
        for it in self._body:
            self._glw.addItem(it)
        grid = gl.GLGridItem(color=_GRID_COLOR)
        grid.setSize(1200, 1200)
        grid.setSpacing(100, 100)
        grid.rotate(90, 1, 0, 0)                    # the xz plane: level
        grid.translate(self.centre[0], lo[1] - 60.0, self.centre[2])
        self._glw.addItem(grid)
        # the arrows start inside the hull: drawn over it (no depth test), last
        self._arrow_item = gl.GLLinePlotItem(mode="lines", width=3.0, antialias=True,
                                             glOptions=_flat_gl())
        self._arrow_item.setDepthValue(20)
        self._glw.addItem(self._arrow_item)
        self._up_item = gl.GLLinePlotItem(mode="lines", width=2.0, antialias=True,
                                          color=_UP_COLOR, glOptions="opaque")
        self._up_item.setData(pos=arrow_lines(self.centre + [0, hi[1] - self.centre[1], 0],
                                              np.array([0.0, 80.0, 0.0])).astype(np.float32))
        self._glw.addItem(self._up_item)
        self._readout = QLabel(self._glw)
        self._readout.setStyleSheet("color: #222; font-family: monospace; font-size: 12px;"
                                    " background: rgba(255,255,255,170); padding: 4px;")
        self._readout.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self._readout.move(8, 8)
        self._glw.cube.clicked.connect(self.snap_view)
        self._build_toolbar()
        self._stream_status = _ElidedLabel()
        self.statusBar().addPermanentWidget(self._stream_status, 1)
        self.reset_camera()
        scheduler().register(self, self._render, self.MAX_HZ)
        self._render()

    @property
    def _bbox(self) -> tuple[np.ndarray, np.ndarray]:
        pts = [it.opts["meshdata"].vertexes() for it in self._body]
        V = np.concatenate([p for p in pts if p is not None and len(p)])
        return V.min(axis=0).astype(float), V.max(axis=0).astype(float)

    # ----- toolbar
    def _build_toolbar(self) -> None:
        tb = QToolBar("ROV")
        self.addToolBar(tb)
        tare = QAction("Tare", self)
        tare.setToolTip("Take the acceleration over the second round the shown time as zero"
                        " (the ROV at rest there; also done on the first second of data), and"
                        " the heading there as straight ahead")
        tare.triggered.connect(lambda: self.tare())
        tb.addAction(tare)
        self._heading_box = QCheckBox("Heading")
        self._heading_box.setToolTip("Turn the ROV by the yaw too (relative to the heading at"
                                     " Tare); off: it always faces forward")
        self._heading_box.toggled.connect(self.set_heading)
        tb.addWidget(self._heading_box)
        self._comp_box = QCheckBox("Components")
        self._comp_box.setChecked(self._components)
        self._comp_box.setToolTip("Also draw the surge (blue), sway (red) and heave (green)"
                                  " parts of the acceleration along the body axes")
        self._comp_box.toggled.connect(self.set_components)
        tb.addWidget(self._comp_box)
        tb.addSeparator()
        tb.addWidget(QLabel(" Smooth "))
        self._smooth_box = QDoubleSpinBox()
        self._smooth_box.setRange(0.0, 5.0)
        self._smooth_box.setSingleStep(0.1)
        self._smooth_box.setDecimals(2)
        self._smooth_box.setSuffix(" s")
        self._smooth_box.setValue(self.pose.smooth_s)
        self._smooth_box.setToolTip("Average the acceleration over this long before the shown"
                                    " time (thruster vibration is ~25 mg rms); 0: one row")
        self._smooth_box.valueChanged.connect(self.set_smooth)
        tb.addWidget(self._smooth_box)
        tb.addWidget(QLabel(" Scale "))
        self._scale_box = QDoubleSpinBox()
        self._scale_box.setRange(5.0, 5000.0)
        self._scale_box.setSingleStep(25.0)
        self._scale_box.setDecimals(0)
        self._scale_box.setSuffix(" mm / 100 mg")
        self._scale_box.setValue(self._scale)
        self._scale_box.setToolTip("Arrow length per 100 mg (0.98 m/s^2); the ROV is ~460 mm"
                                   " long")
        self._scale_box.valueChanged.connect(self.set_scale)
        tb.addWidget(self._scale_box)
        tb.addSeparator()
        home = QAction("Default view", self)
        home.setShortcut(QKeySequence("0"))
        home.triggered.connect(self.reset_camera)
        tb.addAction(home)
        for key, name in SNAP_KEYS.items():
            act = QAction(name.capitalize(), self)
            act.setShortcut(QKeySequence(key))
            act.triggered.connect(lambda _=False, n=name: self.snap_view(n))
            self.addAction(act)

    # ----- camera
    def reset_camera(self) -> None:
        cam = self._glw.cam
        cam.center = self.centre.copy()
        cam.distance = 3.2 * self.radius
        cam.snap_to("home")
        cam.set_bounds(self.centre, 1.6 * self.radius + 2 * self._scale)
        self._glw.moved()

    def snap_view(self, name: str) -> None:
        self._glw.cam.snap_to(name)
        self._glw.moved()

    # ----- settings
    def set_heading(self, on: bool) -> None:
        self._heading = bool(on)
        scheduler().request(self)

    def set_components(self, on: bool) -> None:
        self._components = bool(on)
        scheduler().request(self)

    def set_smooth(self, secs: float) -> None:
        self.pose.smooth_s = float(secs)
        scheduler().request(self)

    def set_scale(self, mm_per_100mg: float) -> None:
        self._scale = float(mm_per_100mg)
        self._glw.cam.set_bounds(self.centre, 1.6 * self.radius + 2 * self._scale)
        scheduler().request(self)

    def set_stream_status(self, text: str) -> None:
        self._stream_status.setText(text)
        self._stream_status.setToolTip(text)

    # ----- data
    def update_view(self, t: np.ndarray, rows: np.ndarray) -> None:
        """The IMU buffer (times, IMU_CHANNELS rows); shows the newest row."""
        if rows.ndim != 2 or len(t) != len(rows):
            return
        self._t, self._rows, self._t_at = t, rows, None
        if self._auto_tare and len(t) and t[-1] - t[0] >= AUTO_TARE_S:
            self._auto_tare = not self.tare(float(t[0]) + AUTO_TARE_S / 2)
        scheduler().request(self)

    def show_at(self, t_at: Optional[float]) -> None:
        """Show the buffered row at session time ``t_at`` (None: the newest)."""
        self._t_at = None if t_at is None else float(t_at)
        scheduler().request(self)

    def clear(self, retare: bool = True) -> None:
        """A new connection: no rows; with ``retare`` the first second is the zero again."""
        self._t, self._rows, self._t_at = np.empty(0), np.empty((0, 9), np.float32), None
        if retare:
            self.pose.clear_tare()
            self._auto_tare, self._yaw_ref = True, None
        scheduler().request(self)

    def tare(self, t_at: Optional[float] = None) -> bool:
        """Zero the acceleration (and the heading) at ``t_at`` (None: the shown time)."""
        if t_at is None:
            t_at = self._t_at if self._t_at is not None else (
                float(self._t[-1]) - 0.5 if len(self._t) else None)
        ok = self.pose.tare(self._t, self._rows, t_at)
        if ok:
            s = self.pose.at(self._t, self._rows, t_at)
            self._yaw_ref = s.yaw if s is not None else None
            scheduler().request(self)
        return ok

    # ----- drawing
    def _rotation(self, s: PoseSample) -> np.ndarray:
        """Body (FRD) -> scene rotation for the drawn attitude."""
        yaw = s.yaw - (self._yaw_ref or 0.0) if self._heading else 0.0
        return FRD_TO_SCENE @ body_to_ned(s.roll, s.pitch, yaw)

    def _render(self) -> None:
        s = self.sample = self.pose.at(self._t, self._rows, self._t_at)
        if s is None:
            m = QMatrix4x4()
            for it in self._body:
                it.setTransform(m)
            self._arrow_item.setData(pos=np.empty((0, 3), np.float32))
            self._readout.setText("no IMU data" + ("" if self._t_at is None
                                                   else f" at {self._t_at:.2f} s"))
            self._readout.adjustSize()
            return
        Rb = self._rotation(s)                       # body FRD -> scene
        R = Rb @ FRD_TO_SCENE.T                      # mesh (scene, level) -> scene
        c = self.centre
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = c - R @ c                         # turn about the centre
        m = QMatrix4x4(*T.ravel().tolist())
        for it in self._body:
            it.setTransform(m)
        k = self._scale / 100.0                      # mm per mg
        segs, cols = [arrow_lines(c, k * (Rb @ s.accel_body))], [_RESULT_COLOR]
        if self._components:
            for i in range(3):
                e = np.zeros(3)
                e[i] = s.accel_body[i]
                segs.append(arrow_lines(c, k * (Rb @ e), head_min=6.0))
                cols.append(_COMPONENT_COLORS[i])
        pos = [p for p in segs if len(p)]
        if pos:
            col = np.concatenate([np.tile(cl, (len(p), 1)) for p, cl in zip(segs, cols)
                                  if len(p)])
            self._arrow_item.setData(pos=np.concatenate(pos).astype(np.float32),
                                     color=col.astype(np.float32))
        else:
            self._arrow_item.setData(pos=np.empty((0, 3), np.float32))
        a = s.accel_body
        bias = ("bias: none" if self.pose.bias_t is None
                else f"bias at {self.pose.bias_t:.1f} s")
        yaw_txt = f"{np.degrees(s.yaw):7.1f}"
        self._readout.setText(
            f"t {s.t:9.2f} s{'' if self._t_at is None else '  (picked)'}\n"
            f"roll {np.degrees(s.roll):6.1f}  pitch {np.degrees(s.pitch):6.1f}  yaw {yaw_txt} deg\n"
            f"accel  surge {a[0]:6.0f}  sway {a[1]:6.0f}  heave {a[2]:6.0f}  |a| "
            f"{np.linalg.norm(a):5.0f} mg\n"
            f"({self.pose.smooth_s:g} s mean of {s.n} rows; {bias})")
        self._readout.adjustSize()

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        scheduler().request(self)
