"""IMU view — accel / gyro / attitude from the server transport's ``imu_bus``.

Three stacked plots sharing one time axis. Rows are plotted against the
sample index scaled by the measured IMU rate, so this is a live monitor,
not a time-accurate record (use the recording + ``whisker_link.convert``).
"""

from __future__ import annotations

import numpy as np
from PyQt6.QtWidgets import QMainWindow

import pyqtgraph as pg

_COLORS = [(220, 40, 40), (40, 160, 40), (40, 80, 220)]
_GROUPS = [
    ("Accel", "mg", (0, 1, 2), ("x", "y", "z")),
    ("Gyro", "mrad/s", (3, 4, 5), ("x", "y", "z")),
    ("Attitude", "deg", (6, 7, 8), ("roll", "pitch", "yaw")),
]


class ImuView(QMainWindow):
    def __init__(self, t_buffer_s: float = 30.0):
        super().__init__()
        self.setWindowTitle("Wavi - IMU")
        self.resize(900, 700)
        self._t_buffer_s = float(t_buffer_s)
        glw = pg.GraphicsLayoutWidget()
        glw.setBackground("w")
        self.setCentralWidget(glw)
        self._curves: list[tuple[int, pg.PlotDataItem]] = []
        first = None
        for row, (title, unit, cols, names) in enumerate(_GROUPS):
            p = glw.addPlot(row=row, col=0)
            p.setLabel("left", title, units=unit)
            p.showGrid(x=True, y=True, alpha=0.2)
            p.addLegend(offset=(5, 5))
            if first is None:
                first = p
            else:
                p.setXLink(first)
            for c, name, color in zip(cols, names, _COLORS):
                self._curves.append((c, p.plot(pen=pg.mkPen(color=color, width=2), name=name)))
        p.setLabel("bottom", "Time", units="s")

    def update_view(self, t: np.ndarray, rows: np.ndarray) -> None:
        if rows.ndim != 2 or len(t) != len(rows) or len(t) == 0:
            return
        for c, curve in self._curves:
            y = np.degrees(rows[:, c]) if c >= 6 else rows[:, c]
            curve.setData(t, y)
