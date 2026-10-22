"""Signal panel for the simulated array (``--transport sim`` / the Sim transport).

Edits a ``SimSignals`` live, also before connecting: per-gauge live / open,
sine amplitude and frequency; scan rate, noise and drift; presets; taps (a
decaying ring on one whisker, by hand or every few seconds); and faults (the
scan flags, a sequence gap, a link CRC error, a dropout) to exercise the
status line.
"""

from __future__ import annotations

import random

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QPushButton, QScrollArea, QSpinBox,
                             QVBoxLayout, QWidget)

from wavi_py.transports.sim_transport import RATES_HZ, SimSignals
from wavi_py.views.base import ViewWindow
from whisker_link.channels import NUM_WHISKER

_FAULTS = (("ADC recovered", "recovered"), ("ADC CRC retry", "crc_retry"),
           ("ADC stale", "stale"), ("ADC re-init", "reinit"), ("Seq gap", "seq_gap"),
           ("Link CRC", "link_crc"), ("Dropout 3 s", "dropout"))


def _spin(lo: float, hi: float, step: float, decimals: int, suffix: str) -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(decimals)
    s.setSuffix(suffix)
    s.setKeyboardTracking(False)            # apply on Enter / arrows, not every digit typed
    return s


class SimPanel(ViewWindow):
    def __init__(self, signals: SimSignals):
        super().__init__()
        self.setWindowTitle("Wavi - Sim signals")
        self.resize(560, 760)
        self._sig = signals
        self._loading = False

        central = QWidget()
        outer = QVBoxLayout(central)
        outer.addWidget(self._build_global())
        outer.addWidget(self._build_gauges(), stretch=1)
        outer.addWidget(self._build_taps())
        outer.addWidget(self._build_faults())
        self.setCentralWidget(central)

        self._auto_tap = QTimer(self)
        self._auto_tap.timeout.connect(self._tap)
        self._load()

    # ----- layout

    def _build_global(self) -> QGroupBox:
        box = QGroupBox("Array")
        h = QHBoxLayout(box)
        self._rate = QComboBox()
        self._rate.addItems([f"{r:g} Hz" for r in RATES_HZ])
        self._rate.setToolTip("Scan rate. The GUI adopts a new measured rate unless recording.")
        self._rate.currentIndexChanged.connect(lambda i: setattr(self._sig, "rate_hz", RATES_HZ[i]))
        self._noise = _spin(0, 100, 0.1, 2, " mV")
        self._noise.setToolTip("White noise on every live gauge (RMS)")
        self._noise.valueChanged.connect(lambda v: setattr(self._sig, "noise_mv", v))
        self._drift = _spin(-100, 100, 0.1, 2, " mV/s")
        self._drift.setToolTip("Linear drift on every live gauge (Re-zero to take it out)")
        self._drift.valueChanged.connect(lambda v: setattr(self._sig, "drift_mv_s", v))
        self._preset = QComboBox()
        self._preset.addItems(["preset...", *SimSignals.PRESETS])
        self._preset.setToolTip("demo: a few gauges, distinct frequencies; all live: 1..16 Hz "
                                "by whisker; all live, same: 2 Hz 3 mV; all open")
        self._preset.activated.connect(self._on_preset)
        for w in (QLabel("Rate"), self._rate, QLabel("Noise"), self._noise,
                  QLabel("Drift"), self._drift, self._preset):
            h.addWidget(w)
        return box

    def _build_gauges(self) -> QGroupBox:
        box = QGroupBox("Gauges: sine per gauge (unticked = open, railed)")
        grid = QGridLayout()
        for col, text in enumerate(("", "A", "amp", "freq", "B", "amp", "freq")):
            grid.addWidget(QLabel(f"<b>{text}</b>"), 0, col)
        self._rows: list[tuple[QCheckBox, QDoubleSpinBox, QDoubleSpinBox]] = []
        for w in range(NUM_WHISKER):
            grid.addWidget(QLabel(f"W{w + 1}"), w + 1, 0)
            for g in range(2):
                ch = 2 * w + g
                live = QCheckBox()
                amp = _spin(0, 1000, 0.5, 2, " mV")
                freq = _spin(0, 500, 0.5, 2, " Hz")
                live.toggled.connect(lambda on, ch=ch: self._edit(ch, live=on))
                amp.valueChanged.connect(lambda v, ch=ch: self._edit(ch, amp_mv=v))
                freq.valueChanged.connect(lambda v, ch=ch: self._edit(ch, freq_hz=v))
                for k, wdg in enumerate((live, amp, freq)):
                    grid.addWidget(wdg, w + 1, 1 + 3 * g + k)
                self._rows.append((live, amp, freq))
        inner = QWidget()
        inner.setLayout(grid)
        scroll = QScrollArea()
        scroll.setWidget(inner)
        scroll.setWidgetResizable(True)
        v = QVBoxLayout(box)
        v.addWidget(scroll)
        return box

    def _build_taps(self) -> QGroupBox:
        box = QGroupBox("Tap: a decaying ring on one whisker")
        h = QHBoxLayout(box)
        self._tap_w = QSpinBox()
        self._tap_w.setRange(0, NUM_WHISKER)
        self._tap_w.setSpecialValueText("any")
        self._tap_w.setPrefix("W")
        self._tap_w.setToolTip("Whisker to tap ('any': a random one with a live gauge). "
                               "An open gauge stays railed, so it shows no tap.")
        self._tap_amp = _spin(0, 1000, 1, 1, " mV")
        self._tap_amp.setValue(20)
        self._tap_freq = _spin(0.1, 500, 1, 1, " Hz")
        self._tap_freq.setValue(8)
        self._tap_decay = _spin(0.01, 10, 0.1, 2, " s")
        self._tap_decay.setValue(0.5)
        tap = QPushButton("Tap")
        tap.clicked.connect(self._tap)
        self._tap_every = QCheckBox("every")
        self._tap_period = _spin(0.2, 60, 0.5, 1, " s")
        self._tap_period.setValue(3)
        self._tap_every.toggled.connect(self._on_auto_tap)
        self._tap_period.valueChanged.connect(lambda _v: self._on_auto_tap(self._tap_every.isChecked()))
        for w in (self._tap_w, self._tap_amp, self._tap_freq, self._tap_decay, tap,
                  self._tap_every, self._tap_period):
            h.addWidget(w)
        return box

    def _build_faults(self) -> QGroupBox:
        box = QGroupBox("Faults (watch the status line's error counters)")
        grid = QGridLayout(box)
        for i, (text, kind) in enumerate(_FAULTS):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, kind=kind: self._sig.fault(kind))
            grid.addWidget(b, i // 4, i % 4)
        return box

    # ----- actions

    def _load(self) -> None:
        """Show the current SimSignals (after a preset)."""
        self._loading = True
        self._rate.setCurrentIndex(RATES_HZ.index(self._sig.rate_hz) if self._sig.rate_hz in RATES_HZ else 1)
        self._noise.setValue(self._sig.noise_mv)
        self._drift.setValue(self._sig.drift_mv_s)
        for ch, (live, amp, freq) in enumerate(self._rows):
            live.setChecked(bool(self._sig.live[ch]))
            amp.setValue(float(self._sig.amp_mv[ch]))
            freq.setValue(float(self._sig.freq_hz[ch]))
        self._loading = False

    def _edit(self, ch: int, **kw) -> None:
        if not self._loading:
            self._sig.set_channel(ch, **kw)

    def _on_preset(self, i: int) -> None:
        if i > 0:
            self._sig.preset(SimSignals.PRESETS[i - 1])
            self._load()
        self._preset.setCurrentIndex(0)

    def _tap(self) -> None:
        w = self._tap_w.value() - 1
        if w < 0:                                  # any whisker with a live gauge
            live = [i for i in range(NUM_WHISKER) if self._sig.live[2 * i:2 * i + 2].any()]
            w = random.choice(live or range(NUM_WHISKER))
        self._sig.tap(w, self._tap_amp.value(), self._tap_freq.value(), self._tap_decay.value())

    def _on_auto_tap(self, on: bool) -> None:
        if on:
            self._auto_tap.start(int(self._tap_period.value() * 1000))
        else:
            self._auto_tap.stop()
