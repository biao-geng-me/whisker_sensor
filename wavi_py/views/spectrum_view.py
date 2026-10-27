"""Live amplitude spectrum per whisker, in the Line window's layouts.

Layouts, keys, Link Y, probe, pause and the redraw scheme are shared with the
Line window (``whisker_plots.WhiskerPlotsView``); X is frequency, linear, 0 to
fs/2, shared by all plots.

The control panel computes the spectra (``update_view``) from the newest
``window_s`` seconds of each channel: mean removed (a linear detrend would
take ~45 % of a sine with one cycle in the window), Hann window, rFFT,
scaled so a sine of amplitude a reads a (the same units as the Line window,
mV at the ADC times ``--scale``). It does so only while this window is shown
and not paused. The window length cycles 0.5 / 1 / 2 / 5 s (default 1 s;
shorter reacts faster, longer resolves finer: the bin spacing is 1 / window).

Y is linear amplitude, or dB re 1 unit (toolbar dB; zero, i.e. a constant
gauge, is left out there). In the stacked layout a
row is one unit of amplitude times the gain (linear), or 80 dB from -60 to
+20 dB times the gain (dB). Each plot's corner shows the dominant frequency
(largest bin above DC) of each gauge and its amplitude.

Y fixed (toolbar): every plot shows 0 to "top" (linear), or "top" dB down to
"span" dB below it (dB; 20 dB is a decade), the same for all channels; in the
stacked layout that range spans one row. Linear and dB keep their own values.
"""

from __future__ import annotations

import numpy as np
from PyQt6.QtGui import QAction, QKeySequence

import pyqtgraph as pg

from wavi_py.views.whisker_plots import WhiskerPlotsView, gauge_html


WINDOWS_S = (0.5, 1.0, 2.0, 5.0)
_DEFAULT_WINDOW_S = 1.0
_DB_FLOOR = -120.0
_DB_ROW = (-60.0, 20.0)         # stacked dB layout: the dB range one row spans
_KEYS_HELP = """\
<table cellspacing=4>
<tr><td><b>Space</b></td><td>pause / resume</td></tr>
<tr><td><b>R</b></td><td>reset view: 0 to fs/2, auto Y (or the fixed range), gain 1</td></tr>
<tr><td><b>= &nbsp;-</b></td><td>zoom frequency in / out (all plots). Keypad + - too</td></tr>
<tr><td><b>+ &nbsp;_</b></td><td>zoom Y in / out: framed tile (stacked: trace gain; Y fixed: top / dB span)</td></tr>
<tr><td><b>. &nbsp;,</b></td><td>zoom Y in / out: all tiles (stacked: trace gain; Y fixed: top / dB span)</td></tr>
<tr><td><b>arrows</b></td><td>move the data (Left: traces slide left); Left / Right all plots, Up / Down the framed tile</td></tr>
<tr><td><b>D</b></td><td>dB / linear amplitude</td></tr>
<tr><td><b>W</b></td><td>FFT window length: 0.5 / 1 / 2 / 5 s</td></tr>
<tr><td><b>Y</b></td><td>link Y: all grid tiles share one Y range</td></tr>
<tr><td><b>Z</b></td><td>zoom box mode (drag a rectangle) / pan mode</td></tr>
<tr><td><b>Esc</b></td><td>back to all whiskers</td></tr>
</table>
The framed tile is the one last under the mouse. Mouse: wheel zooms frequency and
that tile's Y, drag pans, double-click shows one whisker / goes back."""


class SpectrumView(WhiskerPlotsView):
    TITLE = "Wavi - Spectrum"
    X_LABEL = ("Frequency", "Hz")
    KEYS_HELP = _KEYS_HELP
    VALUE_AXIS_LABEL = "peak"
    STACKED_LABEL_SAMPLE = "A 000.0 Hz 000.00   B 000.0 Hz 000.00"
    MAX_HZ = 10.0                    # spectra of a 0.5-5 s window: 10 redraws a second is plenty
    MIN_HZ = 2.5

    def __init__(self, nsensor: int, nch: int):
        super().__init__(nsensor, nch)
        self.window_s = _DEFAULT_WINDOW_S
        self._f = np.empty(0)                      # bin frequencies, Hz
        self._amp = np.empty((0, self._nch))       # amplitude per bin and channel
        self._units = ""
        self._peaks: list[tuple[float, float]] = []
        self._fixed_saved = {False: (5.0, 0.0), True: (_DB_ROW[1], _DB_ROW[1] - _DB_ROW[0])}
        self._show_fixed_boxes()
        self._act_grid.setChecked(True)            # grid by default (this also resets the view)

    # ------------------------------------------------------------------
    # toolbar hooks
    # ------------------------------------------------------------------

    def _view_actions(self) -> list[QAction]:
        if not hasattr(self, "_act_db"):
            self._act_db = QAction("dB", self, checkable=True)
            self._act_db.setToolTip("Amplitude in dB re 1 unit instead of linear [D]")
            self._act_db.setShortcut(QKeySequence("D"))
            self._act_db.toggled.connect(self._on_db)
            self._act_window = QAction(self)
            self._act_window.setToolTip("FFT window length; click to cycle 0.5 / 1 / 2 / 5 s [W]")
            self._act_window.setShortcut(QKeySequence("W"))
            self._act_window.triggered.connect(self._cycle_window)
            self._act_window.setText(f"{_DEFAULT_WINDOW_S:g} s")
            self._act_reset.setToolTip("0 to fs/2, auto Y (or the fixed range), gain 1 [R]")
        return [self._act_db, self._act_window]

    def _cycle_window(self) -> None:
        i = (WINDOWS_S.index(self.window_s) + 1) % len(WINDOWS_S)
        self.window_s = WINDOWS_S[i]
        self._act_window.setText(f"{self.window_s:g} s")
        self.statusBar().showMessage(
            f"FFT window {self.window_s:g} s: {1 / self.window_s:g} Hz bins", 3000)

    def set_units(self, units: str) -> None:
        """Units of the amplitudes (the Line window's sample units)."""
        self._units = units
        self._show_fixed_boxes()
        self._update_labels()

    # ------------------------------------------------------------------
    # fixed Y: linear 0..top, dB top-span..top; each scale keeps its own
    # ------------------------------------------------------------------

    def _on_db(self, on: bool) -> None:
        self._fixed_saved[not on] = (self._y_top.value(), self._y_span.value())
        self._show_fixed_boxes()
        self._reset_view()

    def _show_fixed_boxes(self) -> None:
        db = self._act_db.isChecked()
        top, span = self._fixed_saved[db]
        if db:
            self._set_box(self._y_top, top, -300, 300, 1, prefix="top ", suffix=" dB")
            self._set_box(self._y_span, span, 1, 400, 1, prefix="span ", suffix=" dB")
        else:
            unit = f" {self._units}" if self._units else ""
            self._set_box(self._y_top, top, 1e-6, 1e6, 4, prefix="top ", suffix=unit)
        self._act_y_span.setVisible(db)
        self._y_top.setToolTip("Y fixed: the top of every plot" + (" (dB re 1 unit)" if db else ""))
        self._y_span.setToolTip("Y fixed, dB: how far below the top every plot reaches (20 dB = a decade)")

    def _fixed_range(self) -> tuple[float, float]:
        top = self._y_top.value()
        if self._act_db.isChecked():
            return top - self._y_span.value(), top
        return 0.0, top

    def _fixed_text(self) -> str:
        lo, hi = self._fixed_range()
        unit = "dB" if self._act_db.isChecked() else self._units
        return f"{lo:g} to {hi:g}{' ' + unit if unit else ''}"

    def _zoom_fixed(self, f: float) -> None:
        if self._act_db.isChecked():
            self._y_span.setValue(self._y_span.value() * f)
        else:
            super()._zoom_fixed(f)

    # ------------------------------------------------------------------
    # data
    # ------------------------------------------------------------------

    def update_view(self, freqs: np.ndarray, amp: np.ndarray) -> None:
        """``freqs`` (nf,) Hz, ``amp`` (nf, nch) amplitude spectra."""
        if amp.ndim != 2 or amp.shape[1] != self._nch or len(freqs) != amp.shape[0]:
            return
        new_axis = self._f.shape != freqs.shape or (self._f.size and self._f[-1] != freqs[-1])
        self._f = np.asarray(freqs, dtype=float)
        self._amp = np.asarray(amp)
        if new_axis:
            self._reset_view()                       # first data, or window / rate changed
        else:
            self._request_render()

    def _has_data(self) -> bool:
        return self._f.size > 0

    def _values(self, cols: slice) -> np.ndarray:
        a = self._amp[:, cols]
        if not self._act_db.isChecked():
            return a
        # zero (a constant, e.g. railed, gauge) is not drawn in dB: no line at the floor
        with np.errstate(divide="ignore"):
            return np.where(a > 0, np.maximum(20 * np.log10(a), _DB_FLOOR), np.nan)

    def _plot_data(self, plot: pg.PlotItem, cols: slice) -> tuple[np.ndarray, np.ndarray]:
        return self._f, self._values(cols)

    def _stacked_y(self, y: np.ndarray) -> np.ndarray:
        if self.fixed_y:
            return np.clip(super()._stacked_y(y), 0, None)     # dB below the range: the row base
        if not self._act_db.isChecked():
            return y * self._gain
        lo, hi = _DB_ROW
        return np.clip(y - lo, 0, None) / (hi - lo) * self._gain

    def _reset_x(self) -> None:
        if self._f.size:
            self._current_x_plot().setXRange(0, self._f[-1], padding=0)
        unit = self._units or "units"
        if self._act_db.isChecked():
            left = f"dB re 1 {unit}"
        else:
            left = f"amplitude ({self._units})" if self._units else "amplitude"
        # only the one-whisker plot: 16 rotated tile titles cost space and paint time
        self._single_plot.setLabel("left", left)

    def _probe_text(self, sensor: int | None, x: float) -> str:
        i = int(np.clip(np.searchsorted(self._f, x), 0, self._f.size - 1))
        if i > 0 and abs(self._f[i - 1] - x) < abs(self._f[i] - x):
            i -= 1
        if sensor is None:
            return f"f = {self._f[i]:.2f} Hz"
        v = self._values(slice(2 * sensor, 2 * sensor + 2))[i]
        unit = "dB" if self._act_db.isChecked() else self._units
        a, b = (f"{x:.4g}" if np.isfinite(x) else "–" for x in v)
        return f"f = {self._f[i]:.2f} Hz   W{sensor + 1}   A = {a}   B = {b}   {unit}"

    # ------------------------------------------------------------------
    # peak labels
    # ------------------------------------------------------------------

    def _prepare_labels(self) -> None:
        """Dominant frequency (largest bin above DC) and amplitude per channel."""
        self._peaks = [(np.nan, np.nan)] * self._nch
        if self._f.size < 3:
            return
        a = self._amp[1:]
        k = np.argmax(np.nan_to_num(a, nan=-1.0), axis=0)
        peak = a[k, np.arange(self._nch)]
        self._peaks = [(self._f[1 + k[c]], peak[c]) for c in range(self._nch)]

    def _label_html(self, sensor: int, compact: bool) -> str:
        texts = []
        for g in range(2):
            c = 2 * sensor + g
            f, a = self._peaks[c] if c < len(self._peaks) else (np.nan, np.nan)
            # a railed or disconnected gauge is constant: nothing left after the mean removal
            texts.append("–" if not np.isfinite(a) or a < 1e-9 else f"{f:.1f}&nbsp;Hz&nbsp;{a:.3g}")
        return gauge_html(tuple(texts), "" if compact else self._units)
