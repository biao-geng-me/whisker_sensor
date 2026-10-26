"""FFT window — port of ``wavi/fft_view.m``: the latest spectrum of every channel.

Top: a heatmap, channel (x, W1 A, W1 B, ... W16 B) by frequency (y), colour =
amplitude (or dB), with a colour bar. = / - zoom the frequency axis from 0 Hz.

Bottom: one of three per-channel summaries of the frequency band set right
above it (fmin..fmax), drawn in a closed frame:

- Peak bars: the largest amplitude in the band.
- Box over frequency: the spread of the channel's amplitudes across the
  band's bins. Box = 25..75 %, line = median (the noise floor, roughly),
  whiskers = 5..95 %, dot = the peak.
- Box over time: the spread of the band peak over the last ``history``
  seconds of frames. Box / whiskers as above, dot = the newest frame. The
  history restarts when the band changes and only collects while the window
  is shown.

Each channel's bar / box sits right under its heatmap column: both plots get
the same left axis width, and the lower plot a right margin as wide as the
heatmap's colour bar (re-matched whenever either plot's layout changes).
Open (railed) gauges have nothing after the mean removal and are left out.
The lower plot's Y auto-ranges while the colour range is Auto; with Auto off
it is fixed to the colour range's min / max, the same amplitude scale as the
heatmap (peak bars then rise from the min).
Toolbar, dB, colour range, FFT window and pause: see ``spectral_base``.
"""

from __future__ import annotations

import collections
import time

import numpy as np
from PyQt6.QtCore import QPointF, Qt
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

import pyqtgraph as pg

from wavi_py.views.spectral_base import SpectralWindow, to_db
from whisker_link.channels import CHANNEL_NAMES

_MODES = ("Peak bars", "Box over frequency", "Box over time")
_A_RGB, _B_RGB = (220, 40, 40), (40, 80, 220)
_KEYS_HELP = """\
<table cellspacing=4>
<tr><td><b>Space</b></td><td>pause / resume</td></tr>
<tr><td><b>D</b></td><td>dB / linear amplitude</td></tr>
<tr><td><b>A</b></td><td>auto colour range / the min and max boxes (also the lower plot's Y range)</td></tr>
<tr><td><b>W</b></td><td>FFT window length: 0.5 / 1 / 2 / 5 s (also the Spec window)</td></tr>
<tr><td><b>= &nbsp;-</b></td><td>zoom the heatmap's frequency axis in / out from 0 Hz</td></tr>
<tr><td><b>M</b></td><td>lower plot: peak bars / box over frequency / box over time</td></tr>
<tr><td><b>R</b></td><td>reset view: all frequencies, auto colour range</td></tr>
</table>
Mouse over the heatmap: channel, frequency and amplitude in the status bar."""


def _whisker_ticks(nch: int, offset: float) -> list[list[tuple[float, str]]]:
    return [[(2 * w + offset, f"W{w + 1}") for w in range(nch // 2)]]


def _auto_width(ax: pg.AxisItem) -> float:
    """The width a left axis would take for its current tick labels (pyqtgraph's own rule)."""
    w = ax.textWidth + ax.style["tickTextOffset"][0] + max(0, ax.style["tickLength"])
    if ax.label.isVisible():
        w += ax.label.boundingRect().height() * 0.8
    return float(int(w + 0.999))


def _frame(plot: pg.PlotItem) -> None:
    """Axes on all four sides, values only left and bottom: a closed box."""
    for ax in ("top", "right"):
        plot.showAxis(ax)
        plot.getAxis(ax).setStyle(showValues=False, tickLength=0)


class FftView(SpectralWindow):
    TITLE = "Wavi - FFT"
    KEYS_HELP = _KEYS_HELP

    def __init__(self, nch: int, nfreq: int, t_fft_s: float):
        self._nch = int(nch)
        self._nfreq = int(nfreq)
        self._fft: np.ndarray | None = None        # (nfreq, nch) amplitude
        self._fs = 0.0
        self._fzoom = 1.0                           # heatmap shows 0 .. fzoom * fs/2
        self._hist: collections.deque = collections.deque()   # (t, band peaks (nch,))
        super().__init__(t_fft_s)
        self.resize(1000, 760)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        central.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setCentralWidget(central)

        # top: heatmap channel x frequency
        self._heat = pg.PlotWidget()
        self._heat.setBackground("w")
        hp = self._heat.getPlotItem()
        hp.setLabel("left", "Frequency", units="Hz")
        hp.getAxis("bottom").setTicks(_whisker_ticks(self._nch, 1.0))
        hp.setMouseEnabled(x=False, y=False)
        hp.hideButtons()
        _frame(hp)
        self._image = pg.ImageItem()
        hp.addItem(self._image)
        self._cmap = pg.colormap.get("viridis")
        self._image.setColorMap(self._cmap)
        self._cbar = pg.ColorBarItem(colorMap=self._cmap, interactive=False, width=14)
        self._cbar.setImageItem(self._image, insert_in=hp)
        self._heat.scene().sigMouseMoved.connect(self._on_heat_mouse)
        layout.addWidget(self._heat, stretch=3)

        # bottom: controls right above its plot
        bar = QWidget()
        row = QHBoxLayout(bar)
        row.setContentsMargins(8, 2, 8, 0)
        self._mode = QComboBox()
        self._mode.addItems(_MODES)
        self._mode.setToolTip("Per-channel summary of the band [M]")
        self._mode.setFocusPolicy(Qt.FocusPolicy.NoFocus)        # keys stay with the window
        self._mode.currentIndexChanged.connect(lambda _i: (self._update_mode_widgets(), self._redraw()))
        self._fmin = self._hz_box(0.0, "band start")
        self._fmax = self._hz_box(1e6, "band end (clipped to fs/2)")
        self._band_auto = -1.0                 # the band end as last set to fs/2 (as shown)
        self._low_set: object = None           # lower plot Y as last set: "auto" or (lo, hi)
        self._history = QDoubleSpinBox()
        self._history.setRange(1, 600)
        self._history.setValue(10)
        self._history.setSuffix(" s")
        self._history.setKeyboardTracking(False)
        self.keep_keys(self._history)
        self._history.setToolTip("Box over time: how many seconds of frames")
        self._history.valueChanged.connect(lambda _v: self._redraw())
        for w in (QLabel("Lower plot"), self._mode, QLabel("  band"), self._fmin, QLabel("to"),
                  self._fmax, QLabel("  history"), self._history):
            row.addWidget(w)
        row.addStretch(1)
        layout.addWidget(bar)
        act_mode = QAction(self)
        act_mode.setShortcut(QKeySequence("M"))
        act_mode.triggered.connect(
            lambda: self._mode.setCurrentIndex((self._mode.currentIndex() + 1) % len(_MODES)))
        self.addAction(act_mode)

        self._low = pg.PlotWidget()
        self._low.setBackground("w")
        lp = self._low.getPlotItem()
        lp.getAxis("bottom").setTicks(_whisker_ticks(self._nch, 0.5))
        lp.showGrid(y=True, alpha=0.2)
        lp.setMouseEnabled(x=False, y=True)
        _frame(lp)
        self._brushes = [pg.mkBrush(*(_A_RGB if c % 2 == 0 else _B_RGB), 110) for c in range(self._nch)]
        self._pens = [pg.mkPen(_A_RGB if c % 2 == 0 else _B_RGB) for c in range(self._nch)]
        self._boxes = pg.BarGraphItem(x=np.arange(self._nch), height=np.zeros(self._nch), width=0.6,
                                      brushes=self._brushes, pens=self._pens)
        self._whiskers = pg.PlotCurveItem(pen=pg.mkPen((60, 60, 60)), connect="pairs")
        self._medians = pg.PlotCurveItem(pen=pg.mkPen((0, 0, 0), width=2), connect="pairs")
        self._dots = pg.ScatterPlotItem(size=7, pen=None, brush=pg.mkBrush(0, 0, 0))
        for item in (self._boxes, self._whiskers, self._medians, self._dots):
            lp.addItem(item)
        layout.addWidget(self._low, stretch=2)
        for p in (hp, lp):
            p.getViewBox().sigResized.connect(self._align_plots)
        self._update_mode_widgets()

    def _hz_box(self, value: float, tip: str) -> QDoubleSpinBox:
        b = QDoubleSpinBox()
        b.setRange(0, 1e6)
        b.setDecimals(1)
        b.setSuffix(" Hz")
        b.setValue(value)
        b.setKeyboardTracking(False)
        self.keep_keys(b)
        b.setToolTip(tip)
        b.valueChanged.connect(self._on_band_changed)
        return b

    # ----- data

    def set_nfreq(self, nfreq: int) -> None:
        """The FFT length changed (rate measured, FFT window changed)."""
        self._nfreq = int(nfreq)
        self._fft = None
        self._hist.clear()

    def update_view(self, fft_map: np.ndarray, fs: float) -> None:
        """``fft_map`` (nfreq, nch) amplitude of the newest frame, at sample rate ``fs``."""
        if fft_map.ndim != 2 or fft_map.shape != (self._nfreq, self._nch):
            return
        new_axis = fs != self._fs
        self._fft, self._fs = fft_map, float(fs)
        k = self._band_bins()
        self._hist.append((time.monotonic(), fft_map[k].max(axis=0) if k.size else
                           np.zeros(self._nch)))
        while self._hist and self._hist[-1][0] - self._hist[0][0] > self._history.maximum():
            self._hist.popleft()
        if new_axis:
            self._reset_axes()
        self._redraw()

    def _freqs(self) -> np.ndarray:
        return np.linspace(0, self._fs / 2, self._nfreq)

    def _band_bins(self) -> np.ndarray:
        f = self._freqs()
        return np.flatnonzero((f >= self._fmin.value()) & (f <= self._fmax.value()) & (f > 0))

    def _on_band_changed(self, *_args) -> None:
        self._hist.clear()
        self._update_mode_widgets()
        self._redraw()

    def _update_mode_widgets(self) -> None:
        self._history.setEnabled(self._mode.currentIndex() == 2)

    # ----- drawing

    def _redraw(self) -> None:
        if self._fft is None or not self.isVisible():
            return
        v = self._values(self._fft)
        lo, hi = self._levels(v)
        if self._act_auto.isChecked():
            self._show_levels((lo, hi))
        self._image.setImage(v.T, autoLevels=False, levels=(lo, hi),
                             rect=pg.QtCore.QRectF(0, 0, self._nch, self._fs / 2))
        self._cbar.setLevels((lo, hi))
        self._cbar.axis.setLabel(self._unit())
        self._draw_lower()
        self._align_plots()                     # tick labels may have changed width

    def _draw_lower(self) -> None:
        mode = self._mode.currentIndex()
        k = self._band_bins()
        n = self._nch
        x = np.arange(n, dtype=float)
        # an open (railed) gauge is constant: an all-zero spectrum, left out
        present = np.any(self._fft > 0, axis=0)
        if k.size == 0 or (mode == 2 and not self._hist):
            stats = np.full((6, n), np.nan)
        elif mode == 2:
            hist = [p for t, p in self._hist if self._hist[-1][0] - t <= self._history.value()]
            samples = self._values(np.array(hist))                 # (frames, nch)
            samples[:, ~present] = np.nan
            stats = self._box_stats(samples, newest=samples[-1])
        else:
            samples = self._values(self._fft[k])                   # (bins, nch)
            samples[:, ~present] = np.nan
            with np.errstate(all="ignore"):
                newest = np.nanmax(np.where(np.isfinite(samples), samples, -np.inf), axis=0)
            stats = self._box_stats(samples, newest=np.where(np.isfinite(newest), newest, np.nan))
        q05, q25, q50, q75, q95, dot = stats
        if mode == 0:                                              # peak bars from the floor
            if not self._act_auto.isChecked():
                base = self._lo.value()
            elif self.db:
                base = np.nanmin(dot) - 10 if np.isfinite(dot).any() else 0.0
            else:
                base = 0.0
            ok = np.isfinite(dot) & present
            self._set_boxes(ok, y0=np.full(n, base), height=dot - base)
            self._whiskers.setData([], [])
            self._medians.setData([], [])
            self._dots.setData([], [])
        else:
            ok = np.isfinite(q50)
            self._set_boxes(ok, y0=q25, height=q75 - q25)
            xs = np.repeat(x[ok], 4)
            ys = np.column_stack([q05[ok], q25[ok], q75[ok], q95[ok]]).ravel()
            self._whiskers.setData(xs, ys)
            mx = np.column_stack([x[ok] - 0.3, x[ok] + 0.3]).ravel()
            self._medians.setData(mx, np.repeat(q50[ok], 2))
            self._dots.setData(x[ok], dot[ok])
        what = ("band peak" if mode == 0 else "amplitude across the band" if mode == 1
                else f"band peak, last {self._history.value():g} s")
        self._low.getPlotItem().setLabel("left", f"{what} ({self._unit()})" if self._unit() else what)
        self._low_y()

    def _low_y(self, force: bool = False) -> None:
        """Lower plot Y: auto with the colour range, else the min / max boxes. Only
        on a change, so a mouse zoom of the lower plot stays until then."""
        lp = self._low.getPlotItem()
        want = "auto" if self._act_auto.isChecked() else (self._lo.value(), self._hi.value())
        if want == self._low_set and not force:
            return
        self._low_set = want
        if want == "auto":
            lp.enableAutoRange(axis="y")
        elif want[1] > want[0]:
            lp.disableAutoRange(axis="y")
            lp.setYRange(*want, padding=0)

    def _align_plots(self, *_args) -> None:
        """Channel c's column in the heatmap (x c..c+1) and its bar (x c) at the same
        screen x: the same left axis width, and a lower right margin as wide as the
        heatmap's colour bar."""
        hp, lp = self._heat.getPlotItem(), self._low.getPlotItem()
        left = max(_auto_width(hp.getAxis("left")), _auto_width(lp.getAxis("left")))
        for p in (hp, lp):
            if p.getAxis("left").fixedWidth != left:
                p.getAxis("left").setWidth(left)
        gap_h = self._heat.width() - self._heat.mapFromScene(
            hp.getViewBox().sceneBoundingRect()).boundingRect().right()
        gap_l = self._low.width() - self._low.mapFromScene(
            lp.getViewBox().sceneBoundingRect()).boundingRect().right()
        if gap_h != gap_l:
            right = lp.getAxis("right")
            right.setWidth(max(0, right.width() + gap_h - gap_l))

    def _set_boxes(self, ok: np.ndarray, y0: np.ndarray, height: np.ndarray) -> None:
        """Boxes / bars for the channels in ``ok`` only (absent ones draw nothing)."""
        idx = np.flatnonzero(ok)
        self._boxes.setOpts(x=idx.astype(float), y0=y0[idx], height=height[idx],
                            brushes=[self._brushes[i] for i in idx], pens=[self._pens[i] for i in idx])

    @staticmethod
    def _box_stats(samples: np.ndarray, newest: np.ndarray) -> np.ndarray:
        """5 / 25 / 50 / 75 / 95 % per column and ``newest``; all-NaN columns stay NaN."""
        out = np.full((6, samples.shape[1]), np.nan)
        ok = np.isfinite(samples).any(axis=0)
        if ok.any():
            out[:5, ok] = np.nanpercentile(samples[:, ok], (5, 25, 50, 75, 95), axis=0)
        out[5] = np.where(ok, newest, np.nan)
        return out

    # ----- view

    def _reset_axes(self) -> None:
        self._fzoom = 1.0
        nyq = self._fs / 2
        for box in (self._fmin, self._fmax):                       # band within 0..fs/2
            box.blockSignals(True)
            box.setMaximum(nyq)
            box.blockSignals(False)
        # a band end at the old fs/2 (not set by the user) follows the new fs/2 up too
        v = self._fmax.value()
        if v >= nyq or v == 0 or v == self._band_auto:
            self._fmax.blockSignals(True)
            self._fmax.setValue(nyq)
            self._fmax.blockSignals(False)
            self._band_auto = self._fmax.value()
        hp = self._heat.getPlotItem()
        hp.setXRange(0, self._nch, padding=0)
        hp.setYRange(0, self._fs / 2, padding=0)
        self._low.getPlotItem().setXRange(-0.5, self._nch - 0.5, padding=0)   # = heatmap 0..nch
        self._low_y(force=True)

    def _zoom_freq(self, f: float) -> None:
        if not self._fs:
            return
        self._fzoom = float(np.clip(self._fzoom * f, 0.02, 1.0))
        self._heat.getPlotItem().setYRange(0, self._fzoom * self._fs / 2, padding=0)

    def _reset_view(self) -> None:
        super()._reset_view()
        if self._fs:
            self._reset_axes()
        self._redraw()

    def _on_heat_mouse(self, pos: QPointF) -> None:
        if self._fft is None:
            return
        vb = self._heat.getPlotItem().getViewBox()
        if not vb.sceneBoundingRect().contains(pos):
            return
        p = vb.mapSceneToView(pos)
        c = int(p.x())
        f = self._freqs()
        i = int(np.argmin(np.abs(f - p.y())))
        if not 0 <= c < self._nch:
            return
        v = self._values(self._fft[i:i + 1, c])[0]
        val = "–" if not np.isfinite(v) else f"{v:.4g} {self._unit()}"
        self.statusBar().showMessage(f"{CHANNEL_NAMES[c] if self._nch == len(CHANNEL_NAMES) else c}"
                                     f"   f = {f[i]:.2f} Hz   {val}")
