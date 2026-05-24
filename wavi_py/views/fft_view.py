"""FFT contour view — port of ``wavi/fft_view.m`` (+ ``init_fft_surf`` / ``update_fft_surf``).

MATLAB renders a 3D surface ``(nfreq × nch)`` of the latest FFT frame, plus a
bar chart of each channel's peak amplitude. We do the same thing as a 2D
heatmap (pyqtgraph ``ImageItem``) — same information, much simpler than an
OpenGL 3D mesh — and stack a ``BarGraphItem`` underneath.
"""

from __future__ import annotations

import numpy as np
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout

import pyqtgraph as pg


class FftView(QMainWindow):
    def __init__(self, nch: int, nfreq: int, t_fft_s: float):
        super().__init__()
        self.setWindowTitle("Wavi - FFT contour")
        self.resize(900, 700)

        self._nch = int(nch)
        self._nfreq = int(nfreq)
        self._t_fft_s = float(t_fft_s)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setCentralWidget(central)

        # Top: heatmap of |FFT| per (channel, frequency)
        self._heatmap_widget = pg.PlotWidget()
        self._heatmap_widget.setBackground("w")
        self._heatmap_widget.setLabel("bottom", "Channel")
        self._heatmap_widget.setLabel("left", "Frequency", units="Hz")
        self._image = pg.ImageItem()
        self._heatmap_widget.addItem(self._image)
        # Setting image rect maps array indices to plot coordinates:
        #   x in [0, nch], y in [0, fs/2] (we set y-extent on first update).
        layout.addWidget(self._heatmap_widget, stretch=3)

        # Bottom: bar chart of peak amplitude per channel
        self._bar_widget = pg.PlotWidget()
        self._bar_widget.setBackground("w")
        self._bar_widget.setLabel("bottom", "Channel")
        self._bar_widget.setLabel("left", "Peak |FFT|")
        self._bar_item = pg.BarGraphItem(
            x=np.arange(self._nch), height=np.zeros(self._nch), width=0.8,
            brush=(80, 120, 220),
        )
        self._bar_widget.addItem(self._bar_item)
        layout.addWidget(self._bar_widget, stretch=1)

        # Use a colormap that emphasizes structure
        try:
            self._image.setColorMap(pg.colormap.get("viridis"))
        except Exception:
            pass

    def update_view(self, fft_map: np.ndarray, fs: float) -> None:
        """Refresh with a new FFT frame.

        ``fft_map`` is (nfreq, nch) amplitude. ``fs`` lets us label the y-axis.
        """
        if fft_map.ndim != 2 or fft_map.shape != (self._nfreq, self._nch):
            return

        # ImageItem expects (col, row) = (x, y) when transposed; pyqtgraph's
        # default orientation: data is (rows, cols) interpreted with row=x, col=y.
        # To get x=channel, y=frequency: pass (nch, nfreq), i.e. transpose.
        self._image.setImage(
            fft_map.T,
            autoLevels=True,
            rect=pg.QtCore.QRectF(0, 0, self._nch, fs / 2.0),
        )

        peaks = fft_map.max(axis=0)
        # BarGraphItem doesn't have setOpts for height in older pyqtgraph;
        # re-set via setOpts(height=...). Newer versions accept it.
        try:
            self._bar_item.setOpts(height=peaks)
        except Exception:
            self._bar_widget.removeItem(self._bar_item)
            self._bar_item = pg.BarGraphItem(
                x=np.arange(self._nch), height=peaks, width=0.8,
                brush=(80, 120, 220),
            )
            self._bar_widget.addItem(self._bar_item)
