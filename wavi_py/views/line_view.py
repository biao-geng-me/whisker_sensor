"""Multi-channel time-series view — port of ``wavi/line_view.m``.

Layouts, keys, Link Y, probe, pause and the redraw scheme are shared with the
Spectrum window: see ``whisker_plots.WhiskerPlotsView``. This module adds the
time axis.

Time (X) is shared by all plots. While live, X shows the newest ``span``
seconds (default: the whole buffer). Zooming time (= / -) keeps it live (the
right edge stays at "now"); panning time or mouse zoom / pan leaves live.
Live (L) returns to live keeping the zoom (time span, Y ranges, gain); Resume
does the same. Reset view (R) also restores the full span, auto Y and gain 1.
Re-zero mirrors the control panel's Reset (re-capture V0). Zero at... (Q) then
a click in a plot re-zeroes at that time instead: V0 = the mean of the 0.25 s
around it (``MainWindow.zero_at``; live while paused, or in replay).

Y fixed (toolbar): every plot shows +-value around zero, i.e. around each
channel's own V0 baseline (the traces are drawn minus V0, so the gauges' DC
offsets are already gone), the same span for all channels; in the stacked
layout +-value spans one row. If a channel drifts out of the window, Re-zero
re-centres it.

Every plot also shows the current absolute value of each gauge (raw, before V0
and the display scale; mean of the last 0.1 s) in its corner, or on the right
of each row in the stacked layout.

Redraw cost: curves are min/max decimated to about two points per pixel column
(one vectorized pass over all channels) and drawn with 1 px pens; only major
grid lines are drawn. These dominate the paint time with 30 s of 32 channels.
"""

from __future__ import annotations

import numpy as np
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence

import pyqtgraph as pg

from wavi_py.config import DEFAULT_VIEW_SPAN_S
from wavi_py.daq.series import WSK, ArraySeries, Series, peak_decimate
from wavi_py.views.whisker_plots import GRID, STACKED, WhiskerPlotsView, gauge_html


_LABEL_AVG_S = 0.1              # absolute-value labels: mean of the last 0.1 s
_MIN_SPAN_S = 0.05
_PROBE_S = 0.05                 # the probe reads the sample nearest the mouse within this
_HEAD_PEN = pg.mkPen((230, 0, 0), width=1)   # the replay playhead (red, as on the timeline)
_NOT_LIVE = "Not live: L goes live at this zoom, R resets the view"
_KEYS_HELP = """\
<table cellspacing=4>
<tr><td><b>Space</b></td><td>pause / resume (resume goes live at the current zoom)</td></tr>
<tr><td><b>L</b></td><td>live: follow the newest data, keep the zoom (also resumes)</td></tr>
<tr><td><b>R</b></td><td>reset view: whole buffer, auto Y (or the fixed range), gain 1, live</td></tr>
<tr><td><b>= &nbsp;-</b></td><td>zoom time in / out (all plots; stays live). Keypad + - too</td></tr>
<tr><td><b>+ &nbsp;_</b></td><td>zoom Y in / out: framed tile (stacked: trace gain; Y fixed: the +- value)</td></tr>
<tr><td><b>. &nbsp;,</b></td><td>zoom Y in / out: all tiles (stacked: trace gain; Y fixed: the +- value)</td></tr>
<tr><td><b>arrows</b></td><td>move the data (Left: traces slide left); Left / Right all plots (leaves live), Up / Down the framed tile</td></tr>
<tr><td><b>Y</b></td><td>link Y: all grid tiles share one Y range</td></tr>
<tr><td><b>Z</b></td><td>zoom box mode (drag a rectangle) / pan mode</td></tr>
<tr><td><b>0</b></td><td>re-zero (re-capture V0)</td></tr>
<tr><td><b>Q</b></td><td>zero at: the next click in a plot re-zeroes at that time</td></tr>
<tr><td><b>Esc</b></td><td>back to all whiskers</td></tr>
</table>
The framed tile is the one last under the mouse. Mouse: wheel zooms time and that
tile's Y, drag pans, double-click shows one whisker / goes back."""


_peak_decimate = peak_decimate          # (the IMU view and older callers import it from here)


def follow_window(end: float, span: float, extent: tuple[float, float] | None) -> tuple[float, float]:
    """The X range that follows ``end`` (the newest sample live, the playhead in replay)
    at ``span``: ``end`` on the right edge. With ``extent`` (replay: the recording), it
    stays inside it: near the start the playhead moves right across the window, and a
    span over the whole recording shows all of it."""
    x0, x1 = end - span, end
    if extent is not None:
        lo, hi = extent
        if span >= hi - lo:
            return lo, max(hi, lo + 1e-3)
        if x0 < lo:
            x0, x1 = lo, lo + span
    return x0, x1


class LineView(WhiskerPlotsView):
    # the visible time range changed (any layout); the IMU window follows it
    timeWindowChanged = pyqtSignal(float, float)
    zeroAtRequested = pyqtSignal(float)            # re-zero at this time (Zero at...)

    TITLE = "Wavi - Line"
    X_LABEL = ("Time", "s")
    KEYS_HELP = _KEYS_HELP
    VALUE_AXIS_LABEL = "absolute"
    STACKED_CENTRE = 0.5                          # fixed Y: +-value centred on the row
    DEFAULT_FIXED = 5.0                           # +- this, in the sample units (mV)

    def __init__(self, nsensor: int, nch: int, t_buffer_s: float = 30.0,
                 view_span_s: float = DEFAULT_VIEW_SPAN_S):
        super().__init__(nsensor, nch)
        self._follow = True                        # keep X pinned to the newest data / playhead
        self._view_span = float(view_span_s)       # the span it opens with, and Reset view's
        self._span: float | None = None            # following span; None = all there is
        self._x_range: tuple[float, float] | None = None   # live: the buffer's time window
        self._t = np.empty(0)                      # last data shown (copied), for the probe
        self._sig = np.empty((0, nch), dtype=np.float32)   # raw, before V0 / scale
        self._live = ArraySeries()                 # the copy above, through the series API
        self._store: Series | None = None          # replay: the whole recording (replay/store.py)
        self._despike = False                      # replay: draw the store's despiked scans
        self._head = 0.0                           # replay: the playhead (session seconds)
        self._v0 = np.zeros(self._nch, dtype=np.float32)
        self._scale = 1.0
        self._units = ""                           # absolute-value labels; see set_value_format
        self._open_level: float | None = None
        self._vals = np.full(self._nch, np.nan)
        self._set_box(self._y_top, self.DEFAULT_FIXED, 1e-3, 1e6, 3, prefix="±")
        self._y_top.setToolTip("Y fixed: every plot shows +- this around the channel's V0 baseline")
        self.set_value_format("", None)
        for p in (self._stacked.getPlotItem(), self._tiles[0], self._single_plot.getPlotItem()):
            p.getViewBox().sigXRangeChanged.connect(
                lambda vb, r, p=p: self._emit_time_window(p, r))
        self._head_lines: list[pg.InfiniteLine] = []
        for p in self._all_plots():                # replay: the playhead in every plot
            line = pg.InfiniteLine(angle=90, movable=False, pen=_HEAD_PEN)
            line.setVisible(False)
            line.setZValue(50)
            p.addItem(line, ignoreBounds=True)
            self._head_lines.append(line)
        self._act_grid.setChecked(True)            # grid by default (this also resets the view)

    # ------------------------------------------------------------------
    # toolbar hooks
    # ------------------------------------------------------------------

    def _view_actions(self) -> list[QAction]:
        if not hasattr(self, "_act_live"):
            self._act_live = QAction("Live", self)
            self._act_live.setToolTip("Follow the newest data at the current zoom; also resumes [L]")
            self._act_live.setShortcut(QKeySequence("L"))
            self._act_live.triggered.connect(self._on_live)
            self._act_reset.setToolTip("The opening span (newest 30 s), auto Y (or the fixed "
                                       "range), gain 1, and live again [R]")
        return [self._act_live]

    def _stream_actions(self) -> list[QAction]:
        if not hasattr(self, "_act_rezero"):
            self._act_rezero = QAction("Re-zero", self)
            self._act_rezero.setToolTip("Re-capture the V0 baseline (the control panel's Reset) [0]")
            self._act_rezero.setShortcut(QKeySequence("0"))
            self._act_rezero.triggered.connect(self.resetRequested)
            self._act_zero_at = QAction("Zero at...", self, checkable=True)
            self._act_zero_at.setToolTip("Then click a time in a plot: re-zero (V0) from the 0.25 s"
                                         " around it (pause first, or in replay) [Q]")
            self._act_zero_at.setShortcut(QKeySequence("Q"))
            self._act_zero_at.toggled.connect(self._on_zero_at_toggled)
        return [self._act_rezero, self._act_zero_at]

    def _on_zero_at_toggled(self, on: bool) -> None:
        self.statusBar().showMessage("Zero at: click the time to re-zero at (Q again: cancel)"
                                     if on else self._idle_hint())

    def _on_mouse_clicked(self, ev) -> None:
        if (self._act_zero_at.isChecked() and not ev.double()
                and ev.button() == pg.QtCore.Qt.MouseButton.LeftButton):
            pos = ev.scenePos()
            page = self._pages.currentIndex()
            plots = ([self._stacked.getPlotItem()] if page == STACKED else
                     self._tiles if page == GRID else [self._single_plot.getPlotItem()])
            for p in plots:
                vb = p.getViewBox()
                if vb.sceneBoundingRect().contains(pos):
                    t = float(vb.mapSceneToView(pos).x())
                    self._act_zero_at.setChecked(False)
                    self.zeroAtRequested.emit(t)
                    self.statusBar().showMessage(f"Re-zeroed at t = {t:.3f} s", 4000)
                    return
            return
        super()._on_mouse_clicked(ev)

    # ------------------------------------------------------------------
    # live following
    # ------------------------------------------------------------------

    def _idle_hint(self) -> str:
        return "" if self._follow else _NOT_LIVE

    def _on_resumed(self) -> None:
        self._go_live()

    def set_paused(self, on: bool) -> None:
        if on and not self._paused:              # freeze what is shown: the buffer moves on
            self._t, self._sig = self._t.copy(), self._sig.copy()
            self._live.set(WSK, self._t, self._sig)
        super().set_paused(on)

    def go_live(self) -> None:
        """Follow the newest data (keeping the zoom), e.g. after a replay seek."""
        if not self._follow:
            self._go_live()

    def set_time_window(self, x0: float, x1: float) -> None:
        """Show times x0..x1 (another window's zoom, e.g. the IMU's): leaves live."""
        if (x0, x1) == tuple(self._visible_x_range()):
            return
        if self._follow:
            self._follow = False
            self.statusBar().showMessage(_NOT_LIVE)
        WhiskerPlotsView._show_x(self, (x0, x1))
        self._request_render()

    def _emit_time_window(self, plot, rng) -> None:
        if plot is self._current_x_plot():
            self.timeWindowChanged.emit(float(rng[0]), float(rng[1]))

    def _full_span(self) -> float | None:
        """How far time zooms out while following: the buffer live, the recording in replay."""
        if self._store is not None:
            ext = self._store.extent(WSK)
            return None if ext is None else max(ext[1] - ext[0], _MIN_SPAN_S)
        return None if self._x_range is None else self._x_range[1] - self._x_range[0]

    def _zoom_x(self, f: float) -> None:
        """Time zoom, all plots. Live: the right edge stays at the newest sample."""
        full = self._full_span()
        if self._follow and full:
            span = (self._span or full) * f
            self._span = None if span >= full else max(span, _MIN_SPAN_S)
            self._render()
        else:
            super()._zoom_x(f)

    def _on_live(self) -> None:
        if self._paused:
            self._act_pause.setChecked(False)     # -> control panel -> set_paused -> _go_live
        else:
            self._go_live()

    def _go_live(self) -> None:
        """Follow the newest data again, keeping the visible time span and Y ranges."""
        full = self._full_span()
        x0, x1 = self._visible_x_range()
        if not self._follow and full:
            self._span = None if x1 - x0 >= full else max(x1 - x0, _MIN_SPAN_S)
        self._follow = True
        if self.statusBar().currentMessage() == _NOT_LIVE:
            self.statusBar().clearMessage()
        self._render(labels=True)

    def _on_manual_range(self, *_args) -> None:
        """A mouse zoom / pan, or a key that moves time: leave live."""
        if getattr(self, "_follow", False):
            self._follow = False
            self.statusBar().showMessage(_NOT_LIVE)
        super()._on_manual_range()

    def _reset_x(self) -> None:
        self._follow = True
        full = self._full_span()
        self._span = None if full is not None and self._view_span >= full else self._view_span

    def _show_x(self, x: tuple[float, float]) -> None:
        """Carry the X view across a page switch: live window if following, else the zoom."""
        if self._follow:
            self._apply_x_range()
        else:
            super()._show_x(x)

    def _before_redraw(self) -> None:
        if self._follow:
            self._apply_x_range()

    def _apply_x_range(self) -> None:
        """Following: the newest ``span`` seconds (all there is if None) up to the newest
        sample live, up to the playhead in replay (``follow_window``)."""
        if self._store is not None:
            ext = self._store.extent(WSK)
            if ext is None:
                return
            full = ext[1] - ext[0]
            x0, x1 = follow_window(self._head, self._span or full, ext)
        elif self._x_range is None:
            return
        else:
            x0, x1 = self._x_range
            if self._span is not None and self._span < x1 - x0:
                x0 = x1 - self._span
        self._current_x_plot().setXRange(x0, x1, padding=0)

    # ------------------------------------------------------------------
    # replay: the whole recording, the playhead
    # ------------------------------------------------------------------

    def set_store(self, store: Series | None, despike: bool = False) -> None:
        """Replay: draw from ``store`` (the whole recording) for any time range, the
        playhead on the right edge while following; None: back to the pushed data."""
        self._store = store
        self._despike = bool(despike)
        for line in self._head_lines:
            line.setVisible(store is not None)
        full = self._full_span()
        if self._span is None or (full is not None and self._span >= full):
            self._span = None if full is not None and self._view_span >= full else self._view_span
        self._request_render()

    def set_despike(self, on: bool) -> None:
        if bool(on) != self._despike:
            self._despike = bool(on)
            self._request_render()

    def set_playhead(self, t: float) -> None:
        """Replay: the playhead moved (playing, a seek, or a scrub at mouse rate)."""
        if not np.isfinite(t) or t == self._head:
            return
        self._head = float(t)
        for line in self._head_lines:
            line.setPos(self._head)
        if self._store is not None:
            self._request_render()

    @property
    def following(self) -> bool:
        return self._follow

    def series(self) -> Series:
        """What this window draws, through the series API (``daq/series.py``): the
        store in replay, else its copy of the pushed buffer (frozen while paused)."""
        return self._store if self._store is not None else self._live

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

        self._v0 = np.asarray(v0, dtype=np.float32)
        self._scale = float(scale)
        if self._store is not None:              # replay: the store has it all; V0 / scale only
            self._request_render()
            return
        # live, the caller's buffer itself (a view of the ring's window, daq/ring.py: no
        # 90 s copy per tick; it stays valid until the ring next moves, and every tick
        # that moves it pushes the new view here); paused, a copy, as the ring keeps
        # moving (and compacting) under a paused window
        if self._paused:
            self._t = np.array(t_seconds, dtype=float)
            self._sig = np.array(sig, dtype=np.float32)
        else:
            self._t = np.asarray(t_seconds, dtype=float)
            self._sig = np.asarray(sig, dtype=np.float32)
        self._live.set(WSK, self._t, self._sig)

        if x_range is not None:
            self._x_range = x_range
        self._request_render()

    def _has_data(self) -> bool:
        return self.series().has(WSK)

    def _plot_data(self, plot: pg.PlotItem, cols: slice) -> tuple[np.ndarray, np.ndarray]:
        """``(t, (sig - v0) * scale)`` for ``cols`` over the visible time range, min / max
        per pixel column of ``plot`` (``Series.envelope``: the live copy, or the replay
        store's pyramid, so the whole recording draws as cheaply as 30 s)."""
        vb = plot.getViewBox()
        x0, x1 = vb.viewRange()[0]                # already the live window when following
        t, y = self.series().envelope(WSK, cols, x0, x1, max(1, int(vb.width())), self._despike)
        return t, (y - self._v0[cols]) * self._scale

    def _probe_text(self, sensor: int | None, x: float) -> str:
        src = self.series()
        ext = src.extent(WSK)
        if ext is None or not ext[0] <= x <= ext[1]:
            return f"t = {x:.3f} s   (no data here)"
        t, y = src.get(WSK, slice(None), x - _PROBE_S, x + _PROBE_S, self._despike)
        if not len(t):
            return f"t = {x:.3f} s   (no data here)"
        i = int(np.argmin(np.abs(t - x)))
        if sensor is None:
            return f"t = {t[i]:.3f} s"
        a, b = (y[i, 2 * sensor:2 * sensor + 2] - self._v0[2 * sensor:2 * sensor + 2]) * self._scale
        return f"t = {t[i]:.3f} s   {self._names[sensor]}   A = {a:+.4g}   B = {b:+.4g}   (minus V0)"

    # ------------------------------------------------------------------
    # absolute-value labels
    # ------------------------------------------------------------------

    def set_value_format(self, units: str, open_level: float | None) -> None:
        """Units of the raw samples, and the absolute level at or above which a
        gauge reads "open" (railed: nothing plugged in); None = never."""
        self._units = units
        self._open_level = open_level
        self._value_axis.setLabel(f"absolute ({units})" if units else "absolute")
        self._y_top.setSuffix(f" {units}" if units else "")
        self._update_labels()

    def _fixed_text(self) -> str:
        return f"±{self._y_top.value():g}{' ' + self._units if self._units else ''}"

    def _prepare_labels(self) -> None:
        """The mean of the last 0.1 s: before the newest sample, or the playhead."""
        src = self.series()
        ext = src.extent(WSK)
        self._vals = np.full(self._nch, np.nan)
        if ext is not None:
            end = min(self._head, ext[1]) if self._store is not None else ext[1]
            _, y = src.get(WSK, slice(None), end - _LABEL_AVG_S, end, self._despike)
            if len(y):
                self._vals = y.mean(axis=0)

    def _label_html(self, sensor: int, compact: bool) -> str:
        texts = []
        for g in range(2):
            c = 2 * sensor + g
            v = self._vals[c] if c < self._nch else np.nan
            if not np.isfinite(v):
                texts.append("–")
            elif self._open_level is not None and abs(v) >= self._open_level:
                texts.append("open")
            else:
                texts.append(f"{v:.1f}")
        return gauge_html(tuple(texts), "" if compact else self._units)
