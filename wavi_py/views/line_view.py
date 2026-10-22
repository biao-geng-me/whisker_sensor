"""Multi-channel time-series view — port of ``wavi/line_view.m``.

Each sensor uses two channels, gauge A then gauge B (red / blue, with a legend).
Two layouts:

- Stacked: the two channels of sensor ``i`` are plotted at Y-offset ``i``, so
  all nsensor are visible at once on one axis.
- Grid: one tile per sensor (4x4 for 16 whiskers), X linked, Y auto-ranged
  per tile.

Double-clicking a tile (grid) or near a whisker's trace (stacked) shows that
whisker alone; double-click again, Esc or "All whiskers" goes back.

Toolbar: box zoom, reset view, probe. Zooming or panning stops the plot from
following the newest data until Reset view. Pause and Re-zero mirror the
control panel's Pause and Reset: the view only emits ``pauseToggled`` /
``resetRequested``, and the control panel pushes its state back with
``set_paused`` / ``set_stream_controls_enabled``.

Keys (window-wide): Space pause/resume, Z zoom box on/off, + / = / - zoom
in / out (x1.25 about the view centre), R reset view, 0 re-zero, Esc back to
all whiskers.

The status bar's right side shows the stream status line pushed by the
control panel (``set_stream_status``); its left side keeps transient messages
(probe readout, "not following").

Every plot also shows the current absolute value of each gauge (raw, before V0
and the display scale; mean of the last 0.1 s) in its corner, or on the right
of each row in the stacked layout. Labels refresh at ~7 Hz, not every frame.

Redraw cost: curves are min/max decimated to about two points per pixel column
(one vectorized pass over all channels) and drawn with 1 px pens; only major
grid lines are drawn. These dominate the paint time with 30 s of 32 channels.
"""

from __future__ import annotations

import math
import time

import numpy as np
from PyQt6.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QFont, QFontMetricsF, QKeySequence
from PyQt6.QtWidgets import QLabel, QSizePolicy, QStackedWidget, QToolBar

import pyqtgraph as pg

from wavi_py.views.base import ViewWindow


_RED = (220, 40, 40)
_BLUE = (40, 80, 220)
_PROBE_PEN = pg.mkPen((90, 90, 90), width=1, style=pg.QtCore.Qt.PenStyle.DashLine)
_GAUGE_NAMES = ("A", "B")
_HTML_COLOR = ("#dc2828", "#2850dc")
_PEN_W = 1                      # stacked + grid: wider pens cost 3-6x the paint time
_SINGLE_PEN_W = 2
_LABEL_PERIOD_S = 0.15          # absolute-value labels refresh at ~7 Hz
_LABEL_AVG_S = 0.1              # ...showing the mean of the last 0.1 s
_LABEL_FILL = pg.mkBrush(255, 255, 255, 225)
_ZOOM_STEP = 1.25               # +/- keys


def _add_gauge_legend(plot: pg.PlotItem) -> None:
    """A / B legend in the top-right corner; curves named afterwards join it."""
    plot.addLegend(offset=(-10, 10), labelTextColor="k",
                   brush=pg.mkBrush(255, 255, 255, 200), pen=pg.mkPen(180, 180, 180))


def _show_grid(plot: pg.PlotItem) -> None:
    """Major grid lines only: the minor ones cost as much to paint as the data."""
    plot.showGrid(x=True, y=True, alpha=0.2)
    for ax in ("left", "bottom"):
        plot.getAxis(ax).setStyle(maxTickLevel=0)


def _peak_decimate(t: np.ndarray, y: np.ndarray, k: int) -> tuple[np.ndarray, np.ndarray]:
    """Min and max of each ``k``-sample block of ``y`` (N, m), interleaved.

    Two points per block keep the full envelope, so a 1 px pen draws what the
    full data would. Blocks are aligned to the absolute sample index (t / dt)
    so they don't shimmer as the window scrolls; the newest partial block is
    passed through as is.
    """
    n = len(t)
    if k <= 1 or n < 4 * k:
        return t, y
    head = -int(round(t[0] * (n - 1) / (t[-1] - t[0]))) % k
    nb = (n - head) // k
    end = head + nb * k
    blocks = y[head:end].reshape(nb, k, y.shape[1])
    yd = np.empty((2 * nb + n - end, y.shape[1]), dtype=y.dtype)
    np.min(blocks, axis=1, out=yd[0:2 * nb:2])
    np.max(blocks, axis=1, out=yd[1:2 * nb:2])
    yd[2 * nb:] = y[end:]
    tb = t[head:end].reshape(nb, k)
    td = np.empty(len(yd))
    td[0:2 * nb:2] = tb[:, 0]
    td[1:2 * nb:2] = tb[:, -1]
    td[2 * nb:] = t[end:]
    return td, yd


def _value_label() -> pg.TextItem:
    label = pg.TextItem(anchor=(0, 0), fill=_LABEL_FILL)
    label.setZValue(100)
    # text is only re-laid out at the label rate; blit it from a cache otherwise
    label.textItem.setCacheMode(label.textItem.CacheMode.DeviceCoordinateCache)
    return label


_STACKED, _GRID, _SINGLE = 0, 1, 2              # pages of the QStackedWidget
_TITLE = "Wavi - Line"


class LineView(ViewWindow):
    pauseToggled = pyqtSignal(bool)
    resetRequested = pyqtSignal()

    def __init__(self, nsensor: int, nch: int, t_buffer_s: float = 30.0):
        super().__init__()
        self.setWindowTitle(_TITLE)
        self.resize(900, 600)

        self._nsensor = int(nsensor)
        self._nch = int(nch)
        self._follow = True                       # keep X pinned to the newest data
        self._x_range: tuple[float, float] | None = None
        self._single: int | None = None            # the maximized sensor, if any
        self._t = np.empty(0)                      # last data shown (copied), for the probe
        self._sig = np.empty((0, self._nch), dtype=np.float32)   # raw, before V0 / scale
        self._v0 = np.zeros(self._nch, dtype=np.float32)
        self._scale = 1.0
        self._offsets = (np.arange(self._nch) // 2).astype(np.float32)   # stacked Y offsets
        self._units = ""                           # absolute-value labels; see set_value_format
        self._open_level: float | None = None
        self._labels_t = 0.0

        # --- stacked: one axis, sensor i at Y-offset i
        self._stacked = pg.PlotWidget()
        self._stacked.setBackground("w")
        self._stacked.setLabel("bottom", "Time", units="s")
        self._stacked.setLabel("left", "Whisker")
        self._stacked.getAxis("left").setTicks([[(s, f"W{s + 1}") for s in range(self._nsensor)]])
        _show_grid(self._stacked.getPlotItem())
        _add_gauge_legend(self._stacked.getPlotItem())
        self._stacked_curves: list[pg.PlotDataItem] = []
        for c in range(self._nch):
            color = _RED if (c % 2) == 0 else _BLUE
            name = _GAUGE_NAMES[c] if c < 2 else None
            self._stacked_curves.append(self._stacked.plot(pen=pg.mkPen(color=color, width=_PEN_W), name=name))
        # absolute values: a column right of the axis, one label per whisker row
        self._stacked.showAxis("right")
        self._value_axis = self._stacked.getAxis("right")
        self._value_axis.setStyle(showValues=False, tickLength=0)
        self._stacked_labels = [_value_label() for _ in range(self._nsensor)]
        for label in self._stacked_labels:
            label.setAnchor((0, 0.5))
            label.setParentItem(self._value_axis)
        vb = self._stacked.getPlotItem().getViewBox()
        vb.sigYRangeChanged.connect(self._place_stacked_labels)
        vb.sigResized.connect(self._place_stacked_labels)

        # --- grid: one tile per sensor
        self._grid = pg.GraphicsLayoutWidget()
        self._grid.setBackground("w")
        ncol = math.ceil(math.sqrt(self._nsensor))
        self._tiles: list[pg.PlotItem] = []
        self._grid_curves: list[pg.PlotDataItem] = []
        for s in range(self._nsensor):
            p = self._grid.addPlot(row=s // ncol, col=s % ncol, title=f"W{s + 1}")
            _show_grid(p)
            if self._tiles:
                p.setXLink(self._tiles[0])
            if not self._tiles:
                _add_gauge_legend(p)
            self._tiles.append(p)
            for g in range(2):
                color = _RED if g == 0 else _BLUE
                name = _GAUGE_NAMES[g] if s == 0 else None
                self._grid_curves.append(p.plot(pen=pg.mkPen(color=color, width=_PEN_W), name=name))
        self._tile_labels = [self._corner_label(p) for p in self._tiles]

        # --- single: one sensor filling the window (double-click to enter)
        self._single_plot = pg.PlotWidget()
        self._single_plot.setBackground("w")
        self._single_plot.setLabel("bottom", "Time", units="s")
        _show_grid(self._single_plot.getPlotItem())
        _add_gauge_legend(self._single_plot.getPlotItem())
        self._single_curves = [
            self._single_plot.plot(pen=pg.mkPen(color=color, width=_SINGLE_PEN_W), name=name)
            for color, name in ((_RED, "A"), (_BLUE, "B"))]
        self._single_label = self._corner_label(self._single_plot.getPlotItem())

        self._pages = QStackedWidget()
        self._pages.addWidget(self._stacked)
        self._pages.addWidget(self._grid)
        self._pages.addWidget(self._single_plot)
        self.setCentralWidget(self._pages)

        # --- probe: a vertical line per plot plus a status-bar readout
        self._probe_lines: list[pg.InfiniteLine] = []
        for p in self._all_plots():
            line = pg.InfiniteLine(angle=90, movable=False, pen=_PROBE_PEN)
            line.setVisible(False)
            p.addItem(line, ignoreBounds=True)
            self._probe_lines.append(line)
        for scene in (self._stacked.scene(), self._grid.scene(), self._single_plot.scene()):
            scene.sigMouseMoved.connect(self._on_mouse_moved)
            scene.sigMouseClicked.connect(self._on_mouse_clicked)

        for p in self._all_plots():
            p.getViewBox().sigRangeChangedManually.connect(self._on_manual_range)

        self._build_toolbar()
        self._stream_status = QLabel()
        self._stream_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        # don't let a long status line set the window's minimum width; it clips instead
        self._stream_status.setMinimumWidth(1)
        self._stream_status.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        self.statusBar().addPermanentWidget(self._stream_status)
        self.set_value_format("", None)
        self._reset_view()

    # ------------------------------------------------------------------
    # toolbar
    # ------------------------------------------------------------------

    def _build_toolbar(self) -> None:
        tb = QToolBar("View")
        tb.setMovable(False)
        self.addToolBar(tb)

        self._act_grid = QAction("Grid", self, checkable=True)
        self._act_grid.setToolTip("One tile per sensor instead of one stacked axis")
        self._act_grid.toggled.connect(self._on_grid_toggled)

        self._act_zoom = QAction("Zoom box", self, checkable=True)
        self._act_zoom.setToolTip("Drag a box to zoom (off: drag pans). Wheel zooms either way. [Z]")
        self._act_zoom.setShortcut(QKeySequence("Z"))
        self._act_zoom.toggled.connect(self._on_zoom_toggled)

        act_reset = QAction("Reset view", self)
        act_reset.setToolTip("Full range, and follow the newest data again [R]")
        act_reset.setShortcut(QKeySequence("R"))
        act_reset.triggered.connect(self._reset_view)

        # "=" too, so zoom-in needs no Shift; the keypad keys carry KeypadModifier
        act_zoom_in = QAction("+", self)
        act_zoom_in.setToolTip("Zoom in x1.25 about the view centre [+ or =]")
        act_zoom_in.setShortcuts([QKeySequence("+"), QKeySequence("="),
                                  QKeySequence(Qt.KeyboardModifier.KeypadModifier | Qt.Key.Key_Plus)])
        act_zoom_in.triggered.connect(lambda: self._zoom_by(1 / _ZOOM_STEP))
        act_zoom_out = QAction("\u2212", self)
        act_zoom_out.setToolTip("Zoom out x1.25 about the view centre [-]")
        act_zoom_out.setShortcuts([QKeySequence("-"),
                                   QKeySequence(Qt.KeyboardModifier.KeypadModifier | Qt.Key.Key_Minus)])
        act_zoom_out.triggered.connect(lambda: self._zoom_by(_ZOOM_STEP))

        self._act_probe = QAction("Probe", self, checkable=True)
        self._act_probe.setToolTip("Show channel values under the cursor")
        self._act_probe.toggled.connect(self._on_probe_toggled)

        self._act_all = QAction("All whiskers", self)
        self._act_all.setToolTip("Back to all whiskers (or double-click the plot) [Esc]")
        self._act_all.setShortcut(QKeySequence("Esc"))
        self._act_all.setEnabled(False)
        self._act_all.triggered.connect(self._restore)

        # Pause / Re-zero: the control panel owns the logic (see module docstring).
        self._act_pause = QAction("Pause", self, checkable=True)
        self._act_pause.setToolTip("Freeze the plots; data keeps flowing and recording continues [Space]")
        self._act_pause.setShortcut(QKeySequence(Qt.Key.Key_Space))
        self._act_pause.toggled.connect(self._on_pause_action)

        self._act_rezero = QAction("Re-zero", self)
        self._act_rezero.setToolTip("Re-capture the V0 baseline (the control panel's Reset) [0]")
        self._act_rezero.setShortcut(QKeySequence("0"))
        self._act_rezero.triggered.connect(self.resetRequested)

        for a in (self._act_grid, self._act_zoom, act_zoom_in, act_zoom_out, act_reset,
                  self._act_probe, self._act_all):
            tb.addAction(a)
        tb.addSeparator()
        tb.addAction(self._act_pause)
        tb.addAction(self._act_rezero)
        # Shortcuts are window-wide (QAction's default WindowShortcut context), so they
        # work whichever plot has focus. A shortcut beats a focused button's own Space
        # handling; the toolbar buttons also take no focus, so Space never clicks one.
        for a in tb.actions():
            w = tb.widgetForAction(a)
            if w is not None:
                w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.set_stream_controls_enabled(False)

    def _all_plots(self) -> list[pg.PlotItem]:
        return [self._stacked.getPlotItem(), *self._tiles, self._single_plot.getPlotItem()]

    # ------------------------------------------------------------------
    # pause / re-zero (state pushed from the control panel)
    # ------------------------------------------------------------------

    def _on_pause_action(self, on: bool) -> None:
        self._act_pause.setText("Resume" if on else "Pause")
        self.pauseToggled.emit(on)

    def set_paused(self, on: bool) -> None:
        """Show the pause state without emitting ``pauseToggled``."""
        self._act_pause.blockSignals(True)
        self._act_pause.setChecked(on)
        self._act_pause.blockSignals(False)
        self._act_pause.setText("Resume" if on else "Pause")

    def set_stream_status(self, text: str) -> None:
        """The control panel's stream status line, shown on the right of the status bar."""
        self._stream_status.setText(text)
        self._stream_status.setToolTip(text)

    def set_stream_controls_enabled(self, on: bool) -> None:
        """Pause / Re-zero only make sense while connected."""
        self._act_pause.setEnabled(on)
        self._act_rezero.setEnabled(on)

    # ------------------------------------------------------------------
    # layouts
    # ------------------------------------------------------------------

    def _on_grid_toggled(self, on: bool) -> None:
        self._leave_single()
        self._pages.setCurrentIndex(_GRID if on else _STACKED)
        self._reset_view()

    def _on_mouse_clicked(self, ev) -> None:
        if not ev.double() or ev.button() != pg.QtCore.Qt.MouseButton.LeftButton:
            return
        page = self._pages.currentIndex()
        pos = ev.scenePos()
        if page == _SINGLE:
            self._restore()
        elif page == _GRID:
            sensor = next((i for i, p in enumerate(self._tiles)
                           if p.sceneBoundingRect().contains(pos)), None)
            if sensor is not None:
                self._maximize(sensor)
        else:
            vb = self._stacked.getPlotItem().getViewBox()
            if vb.sceneBoundingRect().contains(pos):
                sensor = int(round(vb.mapSceneToView(pos).y()))
                self._maximize(int(np.clip(sensor, 0, self._nsensor - 1)))

    def _maximize(self, sensor: int) -> None:
        x = self._visible_x_range()
        self._single = sensor
        self._single_plot.setTitle(f"W{sensor + 1}")
        self.setWindowTitle(f"{_TITLE} - W{sensor + 1}")
        self._act_all.setEnabled(True)
        self._pages.setCurrentIndex(_SINGLE)
        self._single_plot.enableAutoRange(axis="y")
        self._show_x(x)
        self._redraw()
        self._update_labels()

    def _restore(self) -> None:
        if self._single is None:
            return
        x = self._visible_x_range()
        self._leave_single()
        self._pages.setCurrentIndex(_GRID if self._act_grid.isChecked() else _STACKED)
        self._show_x(x)
        self._redraw()
        self._update_labels()

    def _leave_single(self) -> None:
        self._single = None
        self._act_all.setEnabled(False)
        self.setWindowTitle(_TITLE)

    def _visible_x_range(self) -> tuple[float, float]:
        x0, x1 = self._current_x_plot().getViewBox().viewRange()[0]
        return x0, x1

    def _show_x(self, x: tuple[float, float]) -> None:
        """Carry the X view across a page switch: live window if following, else the zoom."""
        if self._follow:
            self._apply_x_range()
        else:
            self._current_x_plot().setXRange(*x, padding=0)

    def _zoom_by(self, f: float) -> None:
        """Scale the shown plots' X and Y ranges by ``f`` about their view centre."""
        page = self._pages.currentIndex()
        if page == _GRID:
            self._tiles[0].getViewBox().scaleBy(x=f)      # the other tiles are X-linked
            for p in self._tiles:
                p.getViewBox().scaleBy(y=f)
        else:
            self._current_x_plot().getViewBox().scaleBy(x=f, y=f)
        self._on_manual_range()

    def _on_zoom_toggled(self, on: bool) -> None:
        mode = pg.ViewBox.RectMode if on else pg.ViewBox.PanMode
        for p in self._all_plots():
            p.getViewBox().setMouseMode(mode)
        msg = "Zoom box: drag a rectangle to zoom" if on else "Pan: drag to move the view"
        if not self._follow:
            msg += "; not following live data — Reset view to resume"
        self.statusBar().showMessage(msg)

    def _on_manual_range(self, *_args) -> None:
        if self._follow:
            self._follow = False
            self.statusBar().showMessage("Not following live data — Reset view to resume")
        # re-decimate for the new zoom (matters while paused); after the range lands
        QTimer.singleShot(0, self._redraw)

    def _reset_view(self) -> None:
        self._follow = True
        self.statusBar().clearMessage()
        self._stacked.setYRange(-0.5, self._nsensor)
        for p in (*self._tiles, self._single_plot.getPlotItem()):
            p.enableAutoRange(axis="y")
        self._apply_x_range()
        self._redraw()
        self._update_labels()

    def _current_x_plot(self) -> pg.PlotItem:
        page = self._pages.currentIndex()
        if page == _STACKED:
            return self._stacked.getPlotItem()
        if page == _GRID:
            return self._tiles[0]                         # the rest are X-linked
        return self._single_plot.getPlotItem()

    def _apply_x_range(self) -> None:
        if self._x_range is None:
            return
        x0, x1 = self._x_range
        self._current_x_plot().setXRange(x0, x1, padding=0)

    # ------------------------------------------------------------------
    # probe
    # ------------------------------------------------------------------

    def _on_probe_toggled(self, on: bool) -> None:
        for line in self._probe_lines:
            line.setVisible(on)
        if not on:
            self.statusBar().showMessage(
                "" if self._follow else "Not following live data — Reset view to resume")

    def _on_mouse_moved(self, pos) -> None:
        if not self._act_probe.isChecked() or self._t.size == 0:
            return
        page = self._pages.currentIndex()
        if page in (_STACKED, _SINGLE):
            vb = self._current_x_plot().getViewBox()
            if not vb.sceneBoundingRect().contains(pos):
                return
            pt = vb.mapSceneToView(pos)
            sensor = int(round(pt.y())) if page == _STACKED else self._single
        else:
            sensor = next((i for i, p in enumerate(self._tiles)
                           if p.getViewBox().sceneBoundingRect().contains(pos)), None)
            if sensor is None:
                return
            pt = self._tiles[sensor].getViewBox().mapSceneToView(pos)
        for line in self._probe_lines:
            line.setPos(pt.x())
        if not self._t[0] <= pt.x() <= self._t[-1]:
            self.statusBar().showMessage(f"t = {pt.x():.3f} s   (no data here)")
            return
        i = int(np.clip(np.searchsorted(self._t, pt.x()), 0, self._t.size - 1))
        if not 0 <= sensor < self._nsensor:
            self.statusBar().showMessage(f"t = {self._t[i]:.3f} s")
            return
        a, b = (self._sig[i, 2 * sensor:2 * sensor + 2] - self._v0[2 * sensor:2 * sensor + 2]) * self._scale
        self.statusBar().showMessage(
            f"t = {self._t[i]:.3f} s   W{sensor + 1}   A = {a:+.4g}   B = {b:+.4g}   (minus V0)")

    # ------------------------------------------------------------------
    # data
    # ------------------------------------------------------------------

    def update_view(
        self,
        t_seconds: np.ndarray,
        sig: np.ndarray,
        v0: np.ndarray,
        scale: float,
        x_range: tuple[float, float] | None = None,
    ) -> None:
        """Refresh all curves with new data.

        Parameters
        ----------
        t_seconds : (N,) float — x-axis values in seconds (any monotonic axis OK).
        sig       : (N, nch) float — signal block.
        v0        : (nch,) float — per-channel baseline (subtracted).
        scale     : float — amplitude scale.
        x_range   : optional explicit (x_min, x_max). When supplied, the plot's
                    X range is pinned to it (no auto-range) while following.
                    Lets the caller keep a fixed-width scrolling window even
                    when ``t_seconds`` is shorter than the window during
                    buffer fill-up.
        """
        if sig.ndim != 2 or sig.shape[1] != self._nch:
            return
        if len(t_seconds) != sig.shape[0]:
            return

        # copy: the caller's buffer is shifted in place on the next tick, also while paused
        if self._sig.shape == sig.shape:
            np.copyto(self._t, t_seconds)
            np.copyto(self._sig, sig)
        else:
            self._t = np.array(t_seconds, dtype=float)
            self._sig = np.array(sig, dtype=np.float32)
        self._v0 = np.asarray(v0, dtype=np.float32)
        self._scale = float(scale)

        if x_range is not None:
            self._x_range = x_range
            if self._follow:
                self._apply_x_range()
        self._redraw()
        now = time.monotonic()
        if now - self._labels_t >= _LABEL_PERIOD_S:
            self._update_labels()

    def _decimated(self, plot: pg.PlotItem, cols: slice) -> tuple[np.ndarray, np.ndarray]:
        """``(t, (sig - v0) * scale)`` for ``cols``, ~2 points per pixel of ``plot``."""
        vb = plot.getViewBox()
        n_vis = self._t.size
        if not self._follow:
            x0, x1 = vb.viewRange()[0]
            n_vis = int(np.searchsorted(self._t, x1) - np.searchsorted(self._t, x0))
        k = max(1, n_vis // max(1, int(vb.width())))
        t, y = _peak_decimate(self._t, self._sig[:, cols], k)
        return t, (y - self._v0[cols]) * self._scale

    def _redraw(self) -> None:
        # Only the visible layout is drawn; switching layouts redraws the other.
        if self._t.size == 0:
            return
        page = self._pages.currentIndex()
        if page == _STACKED:
            t, y = self._decimated(self._stacked.getPlotItem(), slice(None))
            y = (y + self._offsets).T.copy()      # contiguous rows, one per curve
            finite = bool(np.isfinite(y).all())
            for c, curve in enumerate(self._stacked_curves):
                curve.setData(t, y[c], skipFiniteCheck=finite)
        elif page == _GRID:
            t, y = self._decimated(self._tiles[0], slice(None))
            y = y.T.copy()
            finite = bool(np.isfinite(y).all())
            for c, curve in enumerate(self._grid_curves):
                curve.setData(t, y[c], skipFiniteCheck=finite)
        elif self._single is not None:
            c0 = 2 * self._single
            t, y = self._decimated(self._single_plot.getPlotItem(), slice(c0, min(c0 + 2, self._nch)))
            for g in range(y.shape[1]):
                self._single_curves[g].setData(t, y[:, g])

    # ------------------------------------------------------------------
    # absolute-value labels
    # ------------------------------------------------------------------

    def set_value_format(self, units: str, open_level: float | None) -> None:
        """Units of the raw samples, and the absolute level at or above which a
        gauge reads "open" (railed: nothing plugged in); None = never."""
        self._units = units
        self._open_level = open_level
        self._value_axis.setLabel(f"absolute ({units})" if units else "absolute")
        self._update_labels()

    def _corner_label(self, plot: pg.PlotItem) -> pg.TextItem:
        """Top-left of ``plot``'s view box, font sized to the plot."""
        label = _value_label()
        vb = plot.getViewBox()
        label.setParentItem(vb)
        label.setPos(2, 2)

        def resize() -> None:
            label.setFont(_px_font(round(0.07 * vb.height()), 8, 16))
        vb.sigResized.connect(resize)
        resize()
        return label

    def _place_stacked_labels(self) -> None:
        vb = self._stacked.getPlotItem().getViewBox()
        y0, y1 = vb.viewRange()[1]
        row_px = vb.height() / max(y1 - y0, 1e-9)
        font = _px_font(round(0.45 * row_px), 7, 14)
        width = math.ceil(QFontMetricsF(font).horizontalAdvance("A -0000.0   B -0000.0")) + 12
        if self._value_axis.width() != width:
            self._value_axis.setWidth(width)
        for s, label in enumerate(self._stacked_labels):
            y = self._value_axis.mapFromScene(vb.mapViewToScene(QPointF(0, s))).y()
            label.setVisible(0 <= y <= self._value_axis.height())
            label.setFont(font)
            label.setPos(4, y)

    def _update_labels(self) -> None:
        """Refresh the visible page's absolute-value labels."""
        self._labels_t = time.monotonic()
        if self._t.size == 0:
            vals = np.full(self._nch, np.nan)
        else:
            i0 = int(np.searchsorted(self._t, self._t[-1] - _LABEL_AVG_S))
            vals = self._sig[i0:].mean(axis=0)
        page = self._pages.currentIndex()
        if page == _STACKED:
            for s, label in enumerate(self._stacked_labels):
                label.setHtml(self._value_html(vals, s, unit=False))
        elif page == _GRID:
            for s, label in enumerate(self._tile_labels):
                label.setHtml(self._value_html(vals, s))
        elif self._single is not None:
            self._single_label.setHtml(self._value_html(vals, self._single))

    def _value_html(self, vals: np.ndarray, sensor: int, unit: bool = True) -> str:
        parts = []
        for g in range(2):
            c = 2 * sensor + g
            v = vals[c] if c < self._nch else np.nan
            if not np.isfinite(v):
                text = "–"
            elif self._open_level is not None and abs(v) >= self._open_level:
                text = "open"
            else:
                text = f"{v:.1f}"
            parts.append(f"<span style='color:{_HTML_COLOR[g]}'>{_GAUGE_NAMES[g]}&nbsp;{text}</span>")
        html = "&nbsp;&nbsp;".join(parts)
        if unit and self._units:
            html += f"&nbsp;<span style='color:#666'>{self._units}</span>"
        return html


def _px_font(px: int, lo: int, hi: int) -> QFont:
    font = QFont()
    font.setPixelSize(int(min(max(px, lo), hi)))
    return font
