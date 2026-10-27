"""IMU view — accel / gyro / attitude from the server transport's ``imu_bus``.

Three stacked plots sharing one time axis. Rows are plotted against the
sample index scaled by the measured IMU rate, so this is a live monitor,
not a time-accurate record (use the recording + ``whisker_link.convert``).
Its time axis and buffer are the IMU's own, independent of the whisker scan
rate. The status bar shows the control panel's stream status line (whisker
rate, scan mode, ADC heartbeat, error counters), and the ESP32 / ADC alarm
banner sits on top as in the whisker views.

Redraw cost: new rows only mark the view dirty; the render scheduler redraws
it at most ``MAX_HZ`` times a second while shown, with the curves min/max
decimated to about two points per pixel column (as in the Line window), and
drawn 1 px wide while decimated (2 px when zoomed in to fewer points than
pixels).
"""

from __future__ import annotations

import numpy as np

import pyqtgraph as pg

from wavi_py.views.base import ViewWindow
from wavi_py.views.line_view import _peak_decimate
from wavi_py.views.render_sched import GraphicsLayoutWidget, scheduler
from wavi_py.views.whisker_plots import _ElidedLabel, _show_grid, set_pen_width

_COLORS = [(220, 40, 40), (40, 160, 40), (40, 80, 220)]
_GROUPS = [
    ("Accel", "mg", (0, 1, 2), ("x", "y", "z")),
    ("Gyro", "mrad/s", (3, 4, 5), ("x", "y", "z")),
    ("Attitude", "deg", (6, 7, 8), ("roll", "pitch", "yaw")),
]


class ImuView(ViewWindow):
    MAX_HZ = 20.0

    def __init__(self, t_buffer_s: float = 30.0):
        super().__init__()
        self.setWindowTitle("Wavi - IMU")
        self.resize(900, 700)
        self._t_buffer_s = float(t_buffer_s)
        self._t = np.empty(0)
        self._rows = np.empty((0, 9), dtype=np.float32)
        scheduler().register(self, self._render, self.MAX_HZ)
        glw = GraphicsLayoutWidget()
        glw.setBackground("w")
        self.setCentralWidget(glw)
        self._curves: list[tuple[int, pg.PlotDataItem]] = []
        first = None
        self._plots: list[pg.PlotItem] = []
        for row, (title, unit, cols, names) in enumerate(_GROUPS):
            p = glw.addPlot(row=row, col=0)
            self._plots.append(p)
            p.setLabel("left", title, units=unit)
            _show_grid(p)                                   # major grid lines, cached axes
            p.addLegend(offset=(5, 5), labelTextColor="k")
            p.getAxis("left").enableAutoSIPrefix(False)    # mg, not "kmg"
            if first is None:
                first = p
            else:
                p.setXLink(first)
            for c, name, color in zip(cols, names, _COLORS):
                self._curves.append((c, p.plot(pen=pg.mkPen(color=color, width=2), name=name)))
        p.setLabel("bottom", "Time", units="s")
        self._stream_status = _ElidedLabel()
        self.statusBar().addPermanentWidget(self._stream_status, 1)

    def set_stream_status(self, text: str) -> None:
        """The control panel's stream status line."""
        self._stream_status.setText(text)
        self._stream_status.setToolTip(text)

    def update_view(self, t: np.ndarray, rows: np.ndarray) -> None:
        if rows.ndim != 2 or len(t) != len(rows) or len(t) == 0:
            return
        self._t, self._rows = t, rows
        scheduler().request(self)

    def _render(self) -> None:
        if not self.isVisible() or len(self._t) == 0:
            return
        vb = self._plots[0].getViewBox()
        x0, x1 = vb.viewRange()[0]
        n_vis = int(np.searchsorted(self._t, x1) - np.searchsorted(self._t, x0))
        k = max(1, min(n_vis, len(self._t)) // max(1, int(vb.width())))
        t, rows = _peak_decimate(self._t, self._rows, k)
        # 2 px while sparse; decimated (a min/max band) 1 px, a third of the paint time
        set_pen_width([c for _, c in self._curves], 2 if len(t) <= vb.width() else 1)
        for c, curve in self._curves:
            y = np.degrees(rows[:, c]) if c >= 6 else rows[:, c]
            curve.setData(t, y)
        self.centralWidget().scene().prepareForPaint()   # auto-range now, not in the paint

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        scheduler().request(self)
