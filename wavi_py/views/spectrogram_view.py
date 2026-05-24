"""Spectrogram view — port of ``wavi/spectrogram_view.m`` (+ init_spec_plot).

MATLAB stacks per-channel spectra vertically into a single ``(nfreq*nch, T)``
array and renders it with ``imagesc``. We do the same with a pyqtgraph
``ImageItem``: rows are stacked (frequency, channel) bins, columns are time.
"""

from __future__ import annotations

import numpy as np
from PyQt6.QtWidgets import QMainWindow

import pyqtgraph as pg


class SpectrogramView(QMainWindow):
    def __init__(self, nch: int, nfreq: int, fs: float, ns_spec: int, t_spec_s: float):
        super().__init__()
        self.setWindowTitle("Wavi - Spectrogram")
        self.resize(900, 600)

        self._nch = int(nch)
        self._nfreq = int(nfreq)
        self._fs = float(fs)
        self._ns_spec = int(ns_spec)
        self._t_spec_s = float(t_spec_s)

        self._plot = pg.PlotWidget()
        self.setCentralWidget(self._plot)
        self._plot.setBackground("w")
        self._plot.setLabel("bottom", "Time", units="s")
        self._plot.setLabel("left", "Freq bin (stacked by channel)")

        self._image = pg.ImageItem()
        self._plot.addItem(self._image)
        try:
            self._image.setColorMap(pg.colormap.get("viridis"))
        except Exception:
            pass

    def update_view(self, spec_data: np.ndarray) -> None:
        """``spec_data`` shape is ``(nfreq*nch, T)``."""
        if spec_data.ndim != 2:
            return
        # ImageItem treats data as (rows -> x, cols -> y) by default; we want
        # rows -> y (frequency bin), cols -> x (time), so transpose.
        h, w = spec_data.shape
        self._image.setImage(
            spec_data.T,
            autoLevels=True,
            rect=pg.QtCore.QRectF(0, 0, w, h),
        )
