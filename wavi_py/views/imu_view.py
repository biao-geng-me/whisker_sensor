"""IMU view — accel / gyro / attitude from the server transport's ``imu_bus``.

Three stacked plots sharing one time axis: each row's own time (the
autopilot's RAW_IMU clock mapped onto the Pi's receive clock, ``daq/clock.py``)
in session seconds, the same axis as the Line window's. With "Link to Line"
(the default) the time range follows the Line window's while it is open, and
zooming or panning time here moves the Line window too; otherwise it shows
the newest ``view_span_s`` (30 s) up to the newest row (the playhead in replay),
and a mouse zoom there changes that span (up to the whole buffer, or the whole
recording in replay). In replay the rows come from the recording's store
(``replay/store.py``: any span, the full recording included) instead of the
pipeline's buffer, with the playhead drawn as a red line. The status bar shows the control panel's stream status line (whisker
rate, scan mode, ADC heartbeat, error counters), and the ESP32 / ADC alarm
banner sits on top as in the whisker views.

Redraw cost: new rows only mark the view dirty; the render scheduler redraws
it at most ``MAX_HZ`` times a second while shown, with the curves min/max
decimated to about two points per pixel column (as in the Line window), and
drawn 1 px wide while decimated (2 px when zoomed in to fewer points than
pixels).
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QAction

import pyqtgraph as pg

from wavi_py.config import DEFAULT_VIEW_SPAN_S
from wavi_py.daq.series import IMU, ArraySeries, Series
from wavi_py.views.base import ViewWindow
from wavi_py.views.line_view import _HEAD_PEN, _MIN_SPAN_S, follow_window
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
    # a zoom / pan of time here (while linked): the Line window shows the same
    timeWindowChanged = pyqtSignal(float, float)

    def __init__(self, t_buffer_s: float = 30.0, view_span_s: float = DEFAULT_VIEW_SPAN_S):
        super().__init__()
        self.setWindowTitle("Wavi - IMU")
        self.resize(900, 700)
        self._t_buffer_s = float(t_buffer_s)
        self._span = min(float(view_span_s), self._t_buffer_s)   # unlinked: the span shown
        self._t = np.empty(0)
        self._rows = np.empty((0, 9), dtype=np.float32)
        self._live = ArraySeries()               # the pushed rows, through the series API
        self._store: Optional[Series] = None     # replay: the whole recording
        self._head = 0.0                         # replay: the playhead
        self._ext_x: Optional[tuple[float, float]] = None     # the Line window's range
        self._setting_x = False
        scheduler().register(self, self._render, self.MAX_HZ)
        tb = self.addToolBar("View")
        tb.setMovable(False)
        self._act_link = QAction("Link to Line", self, checkable=True)
        self._act_link.setChecked(True)
        self._act_link.setToolTip("Show the Line window's time range (while it is open); zoom /"
                                  " pan of time here moves it too")
        self._act_link.toggled.connect(lambda _on: scheduler().request(self))
        tb.addAction(self._act_link)
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
            p.getViewBox().sigRangeChangedManually.connect(self._on_manual_range)
        self._head_lines = []
        for p in self._plots:
            line = pg.InfiniteLine(angle=90, movable=False, pen=_HEAD_PEN)
            line.setVisible(False)
            line.setZValue(50)
            p.addItem(line, ignoreBounds=True)
            self._head_lines.append(line)
        p.setLabel("bottom", "Time", units="s")
        self._stream_status = _ElidedLabel()
        self.statusBar().addPermanentWidget(self._stream_status, 1)

    def set_stream_status(self, text: str) -> None:
        """The control panel's stream status line."""
        self._stream_status.setText(text)
        self._stream_status.setToolTip(text)

    @property
    def linked(self) -> bool:
        return self._act_link.isChecked() and self._ext_x is not None

    def set_time_window(self, x: Optional[tuple[float, float]]) -> None:
        """The Line window's visible time range (None: it closed)."""
        self._ext_x = None if x is None else (float(x[0]), float(x[1]))
        if self._act_link.isChecked():
            scheduler().request(self)

    def _on_manual_range(self, *_args) -> None:
        if self._setting_x:
            return
        x0, x1 = self._plots[0].getViewBox().viewRange()[0]
        if not self.linked:                     # unlinked: a zoom sets the span shown
            full = self._full_span()
            self._span = max(_MIN_SPAN_S, min(x1 - x0, full) if full else x1 - x0)
            scheduler().request(self)
            return
        self._ext_x = (x0, x1)
        self.timeWindowChanged.emit(float(x0), float(x1))

    def update_view(self, t: np.ndarray, rows: np.ndarray) -> None:
        if rows.ndim != 2 or len(t) != len(rows) or len(t) == 0:
            return
        self._t, self._rows = t, rows
        self._live.set(IMU, t, rows)
        if self._store is None:                 # replay: drawn from the store
            scheduler().request(self)

    # ----- replay

    def set_store(self, store: Optional[Series]) -> None:
        """Replay: draw from ``store`` (the whole recording); None: the pushed rows."""
        self._store = store
        for line in self._head_lines:
            line.setVisible(store is not None)
        scheduler().request(self)

    def set_playhead(self, t: float) -> None:
        if not np.isfinite(t) or t == self._head:
            return
        self._head = float(t)
        for line in self._head_lines:
            line.setPos(self._head)
        if self._store is not None:
            scheduler().request(self)

    def series(self) -> Series:
        """What this window draws (``daq/series.py``): the store in replay, else the rows."""
        return self._store if self._store is not None else self._live

    def _full_span(self) -> Optional[float]:
        ext = self.series().extent(IMU)
        if self._store is None:
            return self._t_buffer_s
        return None if ext is None else ext[1] - ext[0]

    def _render(self) -> None:
        src = self.series()
        ext = src.extent(IMU)
        if not self.isVisible() or ext is None:
            return
        vb = self._plots[0].getViewBox()
        if self.linked:
            x = self._ext_x
        elif self._store is not None:
            x = follow_window(self._head, self._span, ext)
        else:
            x = follow_window(ext[1], self._span, None)
        self._setting_x = True
        vb.setXRange(*x, padding=0)
        self._setting_x = False
        t, rows = src.envelope(IMU, slice(None), x[0], x[1], max(1, int(vb.width())))
        if not len(t):
            for _, curve in self._curves:
                curve.setData([], [])
            return
        # 2 px while sparse; decimated (a min/max band) 1 px, a third of the paint time
        set_pen_width([c for _, c in self._curves], 2 if len(t) <= vb.width() else 1)
        for c, curve in self._curves:
            y = np.degrees(rows[:, c]) if c >= 6 else rows[:, c]
            curve.setData(t, y)
        self.centralWidget().scene().prepareForPaint()   # auto-range now, not in the paint

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        scheduler().request(self)
