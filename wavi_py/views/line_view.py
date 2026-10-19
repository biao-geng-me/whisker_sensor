"""Multi-channel time-series view — port of ``wavi/line_view.m``.

Each sensor uses two channels, gauge A then gauge B (red / blue, with a legend).
Two layouts:

- Stacked: the two channels of sensor ``i`` are plotted at Y-offset ``i``, so
  all nsensor are visible at once on one axis.
- Grid: one tile per sensor (4x4 for 16 whiskers), X linked, Y auto-ranged
  per tile.

Toolbar: box zoom, reset view, probe. Zooming or panning stops the plot from
following the newest data until Reset view.
"""

from __future__ import annotations

import math

import numpy as np
from PyQt6.QtGui import QAction
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


class LineView(ViewWindow):
    def __init__(self, nsensor: int, nch: int, t_buffer_s: float = 30.0):
        super().__init__()
        self.setWindowTitle("Wavi - Line")
        self.resize(900, 600)

        self._nsensor = int(nsensor)
        self._nch = int(nch)
        self._follow = True                       # keep X pinned to the newest data
        self._x_range: tuple[float, float] | None = None
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

        self._pages = QStackedWidget()
        self._pages.addWidget(self._stacked)
        self._pages.addWidget(self._grid)
        self.setCentralWidget(self._pages)

        # --- probe: a vertical line per plot plus a status-bar readout
        self._probe_lines: list[pg.InfiniteLine] = []
        for p in self._all_plots():
            line = pg.InfiniteLine(angle=90, movable=False, pen=_PROBE_PEN)
            line.setVisible(False)
            p.addItem(line, ignoreBounds=True)
            self._probe_lines.append(line)
        for scene in (self._stacked.scene(), self._grid.scene()):
            scene.sigMouseMoved.connect(self._on_mouse_moved)

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

        for a in (self._act_grid, self._act_zoom, act_reset, self._act_probe):
            tb.addAction(a)

    def _all_plots(self) -> list[pg.PlotItem]:
        return [self._stacked.getPlotItem(), *self._tiles]

    def _on_grid_toggled(self, on: bool) -> None:
        self._pages.setCurrentIndex(1 if on else 0)
        self._redraw()
        self._reset_view()

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
        for p in self._tiles:
            p.enableAutoRange(axis="y")
        self._apply_x_range()

    def _apply_x_range(self) -> None:
        if self._x_range is None:
            return
        x0, x1 = self._x_range
        if self._pages.currentIndex() == 0:
            self._stacked.setXRange(x0, x1, padding=0)
        elif self._tiles:
            self._tiles[0].setXRange(x0, x1, padding=0)   # the rest are X-linked

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
        if self._pages.currentIndex() == 0:
            vb = self._stacked.getPlotItem().getViewBox()
            if not vb.sceneBoundingRect().contains(pos):
                return
            pt = vb.mapSceneToView(pos)
            sensor = int(round(pt.y()))
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
        if self._pages.currentIndex() == 0:
            for c in range(self._nch):
                self._stacked_curves[c].setData(self._t, self._y[:, c] + float(c // 2))
        else:
            for c in range(self._nch):
                self._grid_curves[c].setData(self._t, self._y[:, c])
