"""Multi-channel time-series view — port of ``wavi/line_view.m``.

Each sensor uses two channels (X/Y); the two channels of sensor ``i`` are
plotted stacked at Y-offset ``i``, so all nsensor are visible at once.
Color alternates red/blue between the two channels of each sensor.
"""

from __future__ import annotations

import numpy as np
from PyQt6.QtWidgets import QMainWindow

import pyqtgraph as pg


_RED = (220, 40, 40)
_BLUE = (40, 80, 220)


class LineView(QMainWindow):
    def __init__(self, nsensor: int, nch: int, t_buffer_s: float = 30.0):
        super().__init__()
        self.setWindowTitle("Wavi - Line")
        self.resize(900, 600)

        self._nsensor = int(nsensor)
        self._nch = int(nch)

        self._plot = pg.PlotWidget()
        self.setCentralWidget(self._plot)
        self._plot.setBackground("w")
        self._plot.setLabel("bottom", "Time", units="s")
        self._plot.setLabel("left", "Sensor index")
        self._plot.setYRange(-0.5, self._nsensor)
        self._plot.showGrid(x=True, y=True, alpha=0.2)

        self._curves: list[pg.PlotDataItem] = []
        for c in range(self._nch):
            color = _RED if (c % 2) == 0 else _BLUE
            pen = pg.mkPen(color=color, width=4)
            curve = self._plot.plot(pen=pen)
            self._curves.append(curve)

    def update_view(
        self,
        t_seconds: np.ndarray,
        sig: np.ndarray,
        v0: np.ndarray,
        scale: float,
    ) -> None:
        """Refresh all curves with new data.

        Parameters
        ----------
        t_seconds : (N,) float — x-axis values in seconds (any monotonic axis OK).
        sig       : (N, nch) float — signal block.
        v0        : (nch,) float — per-channel baseline (subtracted).
        scale     : float — amplitude scale.
        """
        if sig.ndim != 2 or sig.shape[1] != self._nch:
            return
        if len(t_seconds) != sig.shape[0]:
            return

        for c in range(self._nch):
            sensor_idx = c // 2
            y = (sig[:, c] - v0[c]) * float(scale) + float(sensor_idx)
            self._curves[c].setData(t_seconds, y)
