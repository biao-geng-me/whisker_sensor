"""Multi-channel time-series view — port of ``wavi/line_view.m``.

Layouts, keys, Link Y, probe, pause and the redraw scheme are shared with the
Spectrum window: see ``whisker_plots.WhiskerPlotsView``. This module adds the
time axis.

Time (X) is shared by all plots. While live, X shows the newest ``span``
seconds (default: the whole buffer). Zooming time (= / -) keeps it live (the
right edge stays at "now"); panning time or mouse zoom / pan leaves live.
Live (L) returns to live keeping the zoom (time span, Y ranges, gain); Resume
does the same. Reset view (R) also restores the full span, auto Y and gain 1.
Re-zero mirrors the control panel's Reset (re-capture V0).

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
from PyQt6.QtGui import QAction, QKeySequence

import pyqtgraph as pg

from wavi_py.views.whisker_plots import WhiskerPlotsView, gauge_html


_LABEL_AVG_S = 0.1              # absolute-value labels: mean of the last 0.1 s
_MIN_SPAN_S = 0.05
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
<tr><td><b>Esc</b></td><td>back to all whiskers</td></tr>
</table>
The framed tile is the one last under the mouse. Mouse: wheel zooms time and that
tile's Y, drag pans, double-click shows one whisker / goes back."""


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


class LineView(WhiskerPlotsView):
    TITLE = "Wavi - Line"
    X_LABEL = ("Time", "s")
    KEYS_HELP = _KEYS_HELP
    VALUE_AXIS_LABEL = "absolute"
    STACKED_CENTRE = 0.5                          # fixed Y: +-value centred on the row
    DEFAULT_FIXED = 5.0                           # +- this, in the sample units (mV)

    def __init__(self, nsensor: int, nch: int, t_buffer_s: float = 30.0):
        super().__init__(nsensor, nch)
        self._follow = True                        # live: keep X pinned to the newest data
        self._span: float | None = None            # live time span; None = the whole buffer
        self._x_range: tuple[float, float] | None = None
        self._t = np.empty(0)                      # last data shown (copied), for the probe
        self._sig = np.empty((0, self._nch), dtype=np.float32)   # raw, before V0 / scale
        self._v0 = np.zeros(self._nch, dtype=np.float32)
        self._scale = 1.0
        self._units = ""                           # absolute-value labels; see set_value_format
        self._open_level: float | None = None
        self._vals = np.full(self._nch, np.nan)
        self._set_box(self._y_top, self.DEFAULT_FIXED, 1e-3, 1e6, 3, prefix="±")
        self._y_top.setToolTip("Y fixed: every plot shows +- this around the channel's V0 baseline")
        self.set_value_format("", None)
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
            self._act_reset.setToolTip("Whole buffer, auto Y (or the fixed range), gain 1, "
                                       "and live again [R]")
        return [self._act_live]

    def _stream_actions(self) -> list[QAction]:
        if not hasattr(self, "_act_rezero"):
            self._act_rezero = QAction("Re-zero", self)
            self._act_rezero.setToolTip("Re-capture the V0 baseline (the control panel's Reset) [0]")
            self._act_rezero.setShortcut(QKeySequence("0"))
            self._act_rezero.triggered.connect(self.resetRequested)
        return [self._act_rezero]

    # ------------------------------------------------------------------
    # live following
    # ------------------------------------------------------------------

    def _idle_hint(self) -> str:
        return "" if self._follow else _NOT_LIVE

    def _on_resumed(self) -> None:
        self._go_live()

    def _full_span(self) -> float | None:
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
        self._span = None

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
        """Live: the newest ``span`` seconds (the whole buffer window if None)."""
        if self._x_range is None:
            return
        x0, x1 = self._x_range
        if self._span is not None:
            x0 = x1 - self._span
        self._current_x_plot().setXRange(x0, x1, padding=0)

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
        self._request_render()

    def _has_data(self) -> bool:
        return self._t.size > 0

    def _plot_data(self, plot: pg.PlotItem, cols: slice) -> tuple[np.ndarray, np.ndarray]:
        """``(t, (sig - v0) * scale)`` for ``cols``, ~2 points per pixel of ``plot``."""
        vb = plot.getViewBox()
        x0, x1 = vb.viewRange()[0]                # already the live window when following
        n_vis = int(np.searchsorted(self._t, x1) - np.searchsorted(self._t, x0))
        k = max(1, n_vis // max(1, int(vb.width())))
        t, y = _peak_decimate(self._t, self._sig[:, cols], k)
        return t, (y - self._v0[cols]) * self._scale

    def _probe_text(self, sensor: int | None, x: float) -> str:
        if not self._t[0] <= x <= self._t[-1]:
            return f"t = {x:.3f} s   (no data here)"
        i = int(np.clip(np.searchsorted(self._t, x), 0, self._t.size - 1))
        if sensor is None:
            return f"t = {self._t[i]:.3f} s"
        a, b = (self._sig[i, 2 * sensor:2 * sensor + 2] - self._v0[2 * sensor:2 * sensor + 2]) * self._scale
        return f"t = {self._t[i]:.3f} s   {self._names[sensor]}   A = {a:+.4g}   B = {b:+.4g}   (minus V0)"

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
        if self._t.size == 0:
            self._vals = np.full(self._nch, np.nan)
        else:
            i0 = int(np.searchsorted(self._t, self._t[-1] - _LABEL_AVG_S))
            self._vals = self._sig[i0:].mean(axis=0)

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
