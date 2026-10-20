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
"""

from __future__ import annotations

import math

import numpy as np
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import QStackedWidget, QToolBar

import pyqtgraph as pg

from wavi_py.views.base import ViewWindow


_RED = (220, 40, 40)
_BLUE = (40, 80, 220)
_PROBE_PEN = pg.mkPen((90, 90, 90), width=1, style=pg.QtCore.Qt.PenStyle.DashLine)
_GAUGE_NAMES = ("A", "B")


def _add_gauge_legend(plot: pg.PlotItem) -> None:
    """A / B legend in the top-right corner; curves named afterwards join it."""
    plot.addLegend(offset=(-10, 10), labelTextColor="k",
                   brush=pg.mkBrush(255, 255, 255, 200), pen=pg.mkPen(180, 180, 180))


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
        self._t = np.empty(0)                      # last data shown, for the probe
        self._y = np.empty((0, self._nch))         # (sig - v0) * scale, no offsets

        # --- stacked: one axis, sensor i at Y-offset i
        self._stacked = pg.PlotWidget()
        self._stacked.setBackground("w")
        self._stacked.setLabel("bottom", "Time", units="s")
        self._stacked.setLabel("left", "Whisker")
        self._stacked.getAxis("left").setTicks([[(s, f"W{s + 1}") for s in range(self._nsensor)]])
        self._stacked.showGrid(x=True, y=True, alpha=0.2)
        _add_gauge_legend(self._stacked.getPlotItem())
        self._stacked_curves: list[pg.PlotDataItem] = []
        for c in range(self._nch):
            color = _RED if (c % 2) == 0 else _BLUE
            name = _GAUGE_NAMES[c] if c < 2 else None
            self._stacked_curves.append(self._stacked.plot(pen=pg.mkPen(color=color, width=4), name=name))

        # --- grid: one tile per sensor
        self._grid = pg.GraphicsLayoutWidget()
        self._grid.setBackground("w")
        ncol = math.ceil(math.sqrt(self._nsensor))
        self._tiles: list[pg.PlotItem] = []
        self._grid_curves: list[pg.PlotDataItem] = []
        for s in range(self._nsensor):
            p = self._grid.addPlot(row=s // ncol, col=s % ncol, title=f"W{s + 1}")
            p.showGrid(x=True, y=True, alpha=0.2)
            if self._tiles:
                p.setXLink(self._tiles[0])
            if not self._tiles:
                _add_gauge_legend(p)
            self._tiles.append(p)
            for g in range(2):
                color = _RED if g == 0 else _BLUE
                name = _GAUGE_NAMES[g] if s == 0 else None
                self._grid_curves.append(p.plot(pen=pg.mkPen(color=color, width=2), name=name))

        # --- single: one sensor filling the window (double-click to enter)
        self._single_plot = pg.PlotWidget()
        self._single_plot.setBackground("w")
        self._single_plot.setLabel("bottom", "Time", units="s")
        self._single_plot.showGrid(x=True, y=True, alpha=0.2)
        _add_gauge_legend(self._single_plot.getPlotItem())
        self._single_curves = [
            self._single_plot.plot(pen=pg.mkPen(color=color, width=2), name=name)
            for color, name in ((_RED, "A"), (_BLUE, "B"))]

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
        self._act_zoom.setToolTip("Drag a box to zoom (off: drag pans). Wheel zooms either way.")
        self._act_zoom.toggled.connect(self._on_zoom_toggled)

        act_reset = QAction("Reset view", self)
        act_reset.setToolTip("Full range, and follow the newest data again")
        act_reset.triggered.connect(self._reset_view)

        self._act_probe = QAction("Probe", self, checkable=True)
        self._act_probe.setToolTip("Show channel values under the cursor")
        self._act_probe.toggled.connect(self._on_probe_toggled)

        self._act_all = QAction("All whiskers", self)
        self._act_all.setToolTip("Back to all whiskers (Esc, or double-click the plot)")
        self._act_all.setShortcut(QKeySequence("Esc"))
        self._act_all.setEnabled(False)
        self._act_all.triggered.connect(self._restore)

        # Pause / Re-zero: the control panel owns the logic (see module docstring).
        self._act_pause = QAction("Pause", self, checkable=True)
        self._act_pause.setToolTip("Freeze the plots; data keeps flowing and recording continues")
        self._act_pause.toggled.connect(self._on_pause_action)

        self._act_rezero = QAction("Re-zero", self)
        self._act_rezero.setToolTip("Re-capture the V0 baseline (the control panel's Reset)")
        self._act_rezero.triggered.connect(self.resetRequested)

        for a in (self._act_grid, self._act_zoom, act_reset, self._act_probe, self._act_all):
            tb.addAction(a)
        tb.addSeparator()
        tb.addAction(self._act_pause)
        tb.addAction(self._act_rezero)
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
        self._redraw()
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

    def _restore(self) -> None:
        if self._single is None:
            return
        x = self._visible_x_range()
        self._leave_single()
        self._pages.setCurrentIndex(_GRID if self._act_grid.isChecked() else _STACKED)
        self._show_x(x)
        self._redraw()

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

    def _on_zoom_toggled(self, on: bool) -> None:
        mode = pg.ViewBox.RectMode if on else pg.ViewBox.PanMode
        for p in self._all_plots():
            p.getViewBox().setMouseMode(mode)

    def _on_manual_range(self, *_args) -> None:
        if self._follow:
            self._follow = False
            self.statusBar().showMessage("Not following live data — Reset view to resume")

    def _reset_view(self) -> None:
        self._follow = True
        self.statusBar().clearMessage()
        self._stacked.setYRange(-0.5, self._nsensor)
        for p in (*self._tiles, self._single_plot.getPlotItem()):
            p.enableAutoRange(axis="y")
        self._apply_x_range()

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
        a, b = self._y[i, 2 * sensor], self._y[i, 2 * sensor + 1]
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

        self._t = np.asarray(t_seconds)
        self._y = (sig - v0) * float(scale)
        self._redraw()

        if x_range is not None:
            self._x_range = x_range
            if self._follow:
                self._apply_x_range()

    def _redraw(self) -> None:
        # Only the visible layout is drawn; switching layouts redraws the other.
        if self._t.size == 0:
            return
        page = self._pages.currentIndex()
        if page == _STACKED:
            for c in range(self._nch):
                self._stacked_curves[c].setData(self._t, self._y[:, c] + float(c // 2))
        elif page == _GRID:
            for c in range(self._nch):
                self._grid_curves[c].setData(self._t, self._y[:, c])
        elif self._single is not None:
            for g, curve in enumerate(self._single_curves):
                c = 2 * self._single + g
                if c < self._nch:
                    curve.setData(self._t, self._y[:, c])
