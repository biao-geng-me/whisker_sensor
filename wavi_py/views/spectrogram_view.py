"""Spectrogram window — port of ``wavi/spectrogram_view.m``.

Time (x, seconds before now) by frequency (y), colour = amplitude (or dB),
with a colour bar. Show all channels stacked (each channel one row band,
labelled W1 A ... W16 B, its frequencies bottom to top) or one gauge, whose
y axis is then in Hz; [ and ] step through the channels. = / - zoom the
frequency axis from 0 Hz (in the stacked view, each band keeps only its
lowest frequencies). The span (10 / 20 / 60 s) is kept by the control panel,
which owns the history (a ``SpecHistory`` ring the window reads when it
redraws): the window emits ``spanChanged`` and gets ``set_span`` back. Toolbar, dB, colour range, FFT window and pause: see
``spectral_base``.
"""

from __future__ import annotations

import math

import numpy as np
from PyQt6.QtCore import QPointF, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import QComboBox, QLabel

import pyqtgraph as pg

from wavi_py.views.render_sched import PlotWidget
from wavi_py.views.spectral_base import SpectralWindow
from whisker_link.channels import CHANNEL_NAMES

SPANS_S = (10.0, 20.0, 60.0)
_KEYS_HELP = """\
<table cellspacing=4>
<tr><td><b>Space</b></td><td>pause / resume</td></tr>
<tr><td><b>D</b></td><td>dB / linear amplitude</td></tr>
<tr><td><b>A</b></td><td>auto colour range / the min and max boxes</td></tr>
<tr><td><b>W</b></td><td>FFT window length: 0.5 / 1 / 2 / 5 s (also the FFT window)</td></tr>
<tr><td><b>= &nbsp;-</b></td><td>zoom the frequency axis in / out from 0 Hz</td></tr>
<tr><td><b>[ &nbsp;]</b></td><td>previous / next channel (All stacked, W1 A, W1 B, ...)</td></tr>
<tr><td><b>R</b></td><td>reset view: all frequencies, auto colour range</td></tr>
</table>
Mouse over the plot: time, channel, frequency and amplitude in the status bar."""


class SpectrogramView(SpectralWindow):
    TITLE = "Wavi - Spectrogram"
    KEYS_HELP = _KEYS_HELP
    spanChanged = pyqtSignal(float)

    def __init__(self, nch: int, nfreq: int, fs: float, hop: int, t_spec_s: float,
                 t_fft_s: float = 1.0):
        self._nch = int(nch)
        self._names = [n.replace("_", " ") for n in CHANNEL_NAMES[:self._nch]]   # per gauge
        self._nfreq = int(nfreq)
        self._fs = float(fs)
        self._col_dt = hop / fs
        self._hist = None                          # SpecHistory: (cols, nch * nfreq) ring
        self._fzoom = 1.0
        self._shown: tuple[np.ndarray, float, float] | None = None   # what the mouse reads
        super().__init__(t_fft_s)
        self.resize(1000, 700)
        self.set_span(t_spec_s)

        self._plot = PlotWidget()
        self._plot.setBackground("w")
        self.setCentralWidget(self._plot)
        p = self._plot.getPlotItem()
        p.setLabel("bottom", "Time", units="s")
        p.setMouseEnabled(x=False, y=False)
        p.hideButtons()
        for ax in ("top", "right"):
            p.showAxis(ax)
            p.getAxis(ax).setStyle(showValues=False, tickLength=0)
        self._image = pg.ImageItem()
        p.addItem(self._image)
        cmap = pg.colormap.get("viridis")
        self._image.setColorMap(cmap)
        self._cbar = pg.ColorBarItem(colorMap=cmap, interactive=False, width=14)
        self._cbar.setImageItem(self._image, insert_in=p)
        self._plot.scene().sigMouseMoved.connect(self._on_mouse)
        self._label_axis()

    def set_whisker_names(self, names: list[str]) -> None:
        """Whisker names (default W1 .., or a dome map's "LFU (W7)"); each gauge is
        shown as "<name> A" / "<name> B"."""
        self._names = [f"{names[c // 2]} {'AB'[c % 2]}" if c // 2 < len(names)
                       else CHANNEL_NAMES[c].replace("_", " ") for c in range(self._nch)]
        for c, name in enumerate(self._names):
            self._chan.setItemText(c + 1, name)
        self._label_axis()

    def _extra_toolbar(self) -> None:
        self._chan = QComboBox()
        self._chan.addItems(["All stacked", *self._names])
        self._chan.setToolTip("All channels stacked, or one gauge with its frequency axis in Hz [ [ ] ]")
        self._chan.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._chan.currentIndexChanged.connect(lambda _i: (self._label_axis(), self._redraw()))
        self._span = QComboBox()
        self._span.addItems([f"{s:g} s" for s in SPANS_S])
        self._span.setToolTip("How much history to show")
        self._span.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._span.currentIndexChanged.connect(lambda i: self.spanChanged.emit(SPANS_S[i]))
        for w in (QLabel("Channel "), self._chan, QLabel("  span "), self._span):
            self._tb.addWidget(w)
        self._tb.addSeparator()
        for key, step in (("[", -1), ("]", 1)):
            act = QAction(self)
            act.setShortcut(QKeySequence(key))
            act.triggered.connect(lambda _=False, step=step: self._chan.setCurrentIndex(
                (self._chan.currentIndex() + step) % self._chan.count()))
            self.addAction(act)

    # ----- control panel interface

    def set_span(self, secs: float) -> None:
        """Show the span without emitting ``spanChanged``."""
        if secs in SPANS_S:
            self._span.blockSignals(True)
            self._span.setCurrentIndex(SPANS_S.index(secs))
            self._span.blockSignals(False)

    def set_nfreq(self, nfreq: int) -> None:
        self._nfreq = int(nfreq)
        self._hist = None

    def update_view(self, hist, fs: float | None = None, col_dt: float | None = None) -> None:
        """New columns in ``hist`` (a ``SpecHistory``: amplitude columns of nfreq * nch
        channel-major rows); ``fs`` the sample rate, ``col_dt`` seconds per column.
        Only marks the window dirty: it reads ``hist`` when it redraws."""
        if hist.rows != self._nfreq * self._nch:
            return
        if fs is not None:
            self._fs = float(fs)
        if col_dt is not None:
            self._col_dt = float(col_dt)
        self._hist = hist
        self._request_redraw()

    # ----- drawing

    def _channel(self) -> int | None:
        i = self._chan.currentIndex()
        return None if i <= 0 else i - 1

    def _kept_bins(self) -> int:
        return max(2, int(math.ceil(self._nfreq * self._fzoom)))

    def _redraw(self) -> None:
        if self._hist is None or not self.isVisible():
            return
        k = self._kept_bins()
        c = self._channel()
        # (cols, rows) oldest first, already in dB if dB is on (the ring keeps a dB copy)
        if c is None:                                # every channel: its lowest k bins
            d = self._hist.ordered(self.db)
            cols = d.shape[0]
            v = d.reshape(cols, self._nch, self._nfreq)[:, :, :k].reshape(cols, self._nch * k)
            height = self._nch
        else:
            v = self._hist.ordered(self.db, slice(c * self._nfreq, c * self._nfreq + k))
            cols = v.shape[0]
            height = (k - 1) * self._fs / 2 / (self._nfreq - 1) if self._nfreq > 1 else self._fs / 2
        t0 = -cols * self._col_dt
        rect = pg.QtCore.QRectF(t0, 0, cols * self._col_dt, height)
        lo, hi = self._levels(v)
        if self._act_auto.isChecked():
            self._show_levels((lo, hi))
        self._image.setImage(v, autoLevels=False, levels=(lo, hi), rect=rect)
        self._cbar.setLevels((lo, hi))
        self._cbar.axis.setLabel(self._unit())
        self._shown = (v, t0, rect.height())
        p = self._plot.getPlotItem()
        p.setXRange(t0, 0, padding=0)
        p.setYRange(0, rect.height(), padding=0)
        self._plot.scene().prepareForPaint()     # range work now, not in the paint

    def _label_axis(self) -> None:
        p = self._plot.getPlotItem()
        left = p.getAxis("left")
        if self._channel() is None:
            p.setLabel("left", "Channel (each band: 0 Hz at the bottom)")
            left.setTicks([[(c + 0.5, self._names[c]) for c in range(self._nch)]])
        else:
            left.setTicks(None)
            p.setLabel("left", f"{self._names[self._channel()]}  frequency",
                       units="Hz")

    # ----- view

    def _zoom_freq(self, f: float) -> None:
        self._fzoom = float(np.clip(self._fzoom * f, 0.02, 1.0))
        self._redraw()

    def _reset_view(self) -> None:
        super()._reset_view()
        self._fzoom = 1.0
        self._redraw()

    def _on_mouse(self, pos: QPointF) -> None:
        if self._shown is None:
            return
        vb = self._plot.getPlotItem().getViewBox()
        if not vb.sceneBoundingRect().contains(pos):
            return
        v, t0, height = self._shown                  # v: (cols, rows)
        p = vb.mapSceneToView(pos)
        cols = v.shape[0]
        j = int(np.clip((p.x() - t0) / self._col_dt, 0, cols - 1))
        k = self._kept_bins()
        c = self._channel()
        if c is None:
            ch = int(np.clip(p.y(), 0, self._nch - 1e-9))
            b = int(np.clip((p.y() - ch) * k, 0, k - 1))
            row = ch * k + b
        else:
            ch = c
            b = int(np.clip(round(p.y() / height * (k - 1)), 0, k - 1))
            row = b
        f = b * self._fs / 2 / max(self._nfreq - 1, 1)
        val = v[j, row]
        text = "–" if not np.isfinite(val) else f"{val:.4g} {self._unit()}"
        self.statusBar().showMessage(f"t = {t0 + (j + 1) * self._col_dt:.2f} s   "
                                     f"{self._names[ch]}   f = {f:.2f} Hz   {text}")
