"""Shared toolbar and state of the FFT and Spectrogram windows.

Both show amplitude spectra the control panel computes from the newest
``fft_window_s`` seconds of each channel (mean removed, Hann window, scaled
so a sine of amplitude a reads a; the same method as the Spectrum window).

Toolbar: Pause (in sync with the control panel), dB (D), colour range: Auto
(A; 1st to 99.5th percentile of what is shown) or the min / max boxes, FFT
window length (W cycles 0.5 / 1 / 2 / 5 s; shared by both windows, so the
control panel owns it: the window emits ``fftWindowChanged`` and gets
``set_fft_window`` back), Reset view (R), Keys. = / - zoom the frequency axis
from 0 Hz. The right of the status bar shows the stream status line; the
left, the value under the mouse.
"""

from __future__ import annotations

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence
from PyQt6.QtWidgets import QComboBox, QDoubleSpinBox, QLabel, QMessageBox, QToolBar

from wavi_py.views.base import ViewWindow
from wavi_py.views.whisker_plots import _ElidedLabel

FFT_WINDOWS_S = (0.5, 1.0, 2.0, 5.0)
DB_FLOOR = -120.0
_LIN_LEVELS = (0.0, 5.0)
_DB_LEVELS = (-60.0, 20.0)


def to_db(a: np.ndarray) -> np.ndarray:
    """20 log10, zero -> NaN (not drawn / left out of the auto range)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(a > 0, np.maximum(20 * np.log10(a), DB_FLOOR), np.nan)


class SpectralWindow(ViewWindow):
    pauseToggled = pyqtSignal(bool)
    fftWindowChanged = pyqtSignal(float)

    TITLE = "Wavi"
    KEYS_HELP = ""

    def __init__(self, fft_window_s: float):
        super().__init__()
        self.setWindowTitle(self.TITLE)
        self._units = ""
        self._levels_saved = {False: _LIN_LEVELS, True: _DB_LEVELS}

        self._tb = QToolBar("View")
        self._tb.setMovable(False)
        self.addToolBar(self._tb)

        self._act_pause = QAction("Pause", self, checkable=True)
        self._act_pause.setToolTip("Freeze the plots; data keeps flowing and recording continues [Space]")
        self._act_pause.setShortcut(QKeySequence(Qt.Key.Key_Space))
        self._act_pause.toggled.connect(self._on_pause_action)

        self._act_db = QAction("dB", self, checkable=True)
        self._act_db.setToolTip("Amplitude in dB re 1 unit instead of linear [D]")
        self._act_db.setShortcut(QKeySequence("D"))
        self._act_db.toggled.connect(self._on_db)

        self._act_auto = QAction("Auto", self, checkable=True)
        self._act_auto.setChecked(True)
        self._act_auto.setToolTip("Colour range from the data (1st to 99.5th percentile); "
                                  "off: the min / max boxes [A]")
        self._act_auto.setShortcut(QKeySequence("A"))
        self._act_auto.toggled.connect(self._on_auto)
        self._lo = QDoubleSpinBox()
        self._hi = QDoubleSpinBox()
        for box, tip in ((self._lo, "colour range minimum"), (self._hi, "colour range maximum")):
            box.setRange(-1e6, 1e6)
            box.setDecimals(2)
            box.setKeyboardTracking(False)
            box.setToolTip(tip)
            self.keep_keys(box)
            box.valueChanged.connect(self._on_levels_edited)

        self._win = QComboBox()
        self._win.addItems([f"{w:g} s" for w in FFT_WINDOWS_S])
        self._win.setToolTip("FFT window length (both FFT and Spec windows): "
                             "bins are 1 / window apart [W]")
        self._win.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._win.currentIndexChanged.connect(
            lambda i: self.fftWindowChanged.emit(FFT_WINDOWS_S[i]))
        act_win = QAction(self)
        act_win.setShortcut(QKeySequence("W"))
        act_win.triggered.connect(
            lambda: self._win.setCurrentIndex((self._win.currentIndex() + 1) % len(FFT_WINDOWS_S)))
        self.addAction(act_win)

        self._act_reset = QAction("Reset view", self)
        self._act_reset.setToolTip("Whole frequency range, auto colour range [R]")
        self._act_reset.setShortcut(QKeySequence("R"))
        self._act_reset.triggered.connect(self._reset_view)
        for key, f in (("=", 1 / 1.25), ("-", 1.25)):
            act = QAction(self)
            act.setShortcut(QKeySequence(key))
            act.triggered.connect(lambda _=False, f=f: self._zoom_freq(f))
            self.addAction(act)

        act_keys = QAction("Keys", self)
        act_keys.triggered.connect(self._show_keys)

        self._extra_toolbar()
        for w in (self._act_db, self._act_auto, QLabel(" min "), self._lo, QLabel(" max "), self._hi,
                  QLabel("  FFT "), self._win, self._act_reset):
            self._tb.addAction(w) if isinstance(w, QAction) else self._tb.addWidget(w)
        self._tb.addSeparator()
        self._tb.addAction(self._act_pause)
        self._tb.addSeparator()
        self._tb.addAction(act_keys)
        for a in self._tb.actions():
            w = self._tb.widgetForAction(a)
            if w is not None and not isinstance(w, (QDoubleSpinBox, QComboBox)):
                w.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self._stream_status = _ElidedLabel()
        self._stream_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.statusBar().addPermanentWidget(self._stream_status)
        self.set_fft_window(fft_window_s)
        self.set_stream_controls_enabled(False)
        self._show_levels()

    def _extra_toolbar(self) -> None:
        """Hook: actions / widgets placed first on the toolbar."""

    def keep_keys(self, w) -> None:
        """Let a value box take typing only while clicked into, then hand the keys
        back to the plots (otherwise D, A, = ... would go into the box)."""
        w.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        w.editingFinished.connect(self._focus_plot)

    def _focus_plot(self) -> None:
        if self.centralWidget() is not None:
            self.centralWidget().setFocus()

    # ----- control panel interface

    def _on_pause_action(self, on: bool) -> None:
        self._act_pause.setText("Resume" if on else "Pause")
        self.pauseToggled.emit(on)

    def set_paused(self, on: bool) -> None:
        self._act_pause.blockSignals(True)
        self._act_pause.setChecked(on)
        self._act_pause.blockSignals(False)
        self._act_pause.setText("Resume" if on else "Pause")

    def set_stream_status(self, text: str) -> None:
        self._stream_status.setText(text)
        self._stream_status.setToolTip(text)

    def set_stream_controls_enabled(self, on: bool) -> None:
        self._act_pause.setEnabled(on)

    def set_fft_window(self, secs: float) -> None:
        """Show the FFT window length without emitting ``fftWindowChanged``."""
        if secs in FFT_WINDOWS_S:
            self._win.blockSignals(True)
            self._win.setCurrentIndex(FFT_WINDOWS_S.index(secs))
            self._win.blockSignals(False)

    def set_units(self, units: str) -> None:
        self._units = units
        self._redraw()

    # ----- levels / dB

    @property
    def db(self) -> bool:
        return self._act_db.isChecked()

    def _values(self, a: np.ndarray) -> np.ndarray:
        return to_db(a) if self.db else a

    def _unit(self) -> str:
        return "dB" if self.db else (self._units or "")

    def _levels(self, v: np.ndarray) -> tuple[float, float]:
        """Colour range for values ``v`` (already in dB if dB is on)."""
        if not self._act_auto.isChecked():
            return self._lo.value(), self._hi.value()
        finite = v[np.isfinite(v)]
        if finite.size == 0:
            return self._levels_saved[self.db]
        lo, hi = np.percentile(finite, (1, 99.5))
        if hi <= lo:
            hi = lo + (1.0 if self.db else 1e-3)
        return float(lo), float(hi)

    def _show_levels(self, lo_hi: tuple[float, float] | None = None) -> None:
        """Put the range in use into the min / max boxes (without editing it)."""
        lo, hi = lo_hi if lo_hi is not None else self._levels_saved[self.db]
        for box, v in ((self._lo, lo), (self._hi, hi)):
            box.blockSignals(True)
            box.setValue(v)
            box.blockSignals(False)

    def _on_levels_edited(self, _v: float) -> None:
        self._act_auto.setChecked(False)          # typing a range means: use it
        self._levels_saved[self.db] = (self._lo.value(), self._hi.value())
        self._redraw()

    def _on_auto(self, on: bool) -> None:
        if not on:
            self._levels_saved[self.db] = (self._lo.value(), self._hi.value())
        self._redraw()

    def _on_db(self, _on: bool) -> None:
        if not self._act_auto.isChecked():
            self._show_levels()                   # each scale keeps its own fixed range
        self._redraw()

    # ----- view

    def _zoom_freq(self, f: float) -> None:
        """Hook: scale the frequency axis by ``f`` from 0 Hz."""

    def _reset_view(self) -> None:
        self._act_auto.setChecked(True)

    def _redraw(self) -> None:
        """Hook: redraw from the stored data with the current options."""

    def _show_keys(self) -> None:
        box = QMessageBox(QMessageBox.Icon.NoIcon, f"{self.TITLE} keys", self.KEYS_HELP, parent=self)
        box.setTextFormat(Qt.TextFormat.RichText)
        box.setModal(False)
        box.show()

    def showEvent(self, event) -> None:  # noqa: N802 — Qt naming
        super().showEvent(event)
        self._focus_plot()
        self._redraw()
