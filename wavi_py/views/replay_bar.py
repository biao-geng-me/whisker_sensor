"""Replay controls: one shared ``ReplayModel`` and the widgets that drive it.

``ReplayModel`` wraps the replay transport (``transports/replay_transport.py``,
thread-safe) for the GUI thread: ``poll()`` (the control panel's 20 Hz tick)
reads the playhead and emits ``positionChanged`` / ``stateChanged``, so every
widget bound to the model shows the same thing, whichever window it sits in.

``ReplayControls`` is the compact row: play / pause, speed, step back and
forward, the time readout (session seconds / length, the recording's wall
clock, the depth at the playhead) and a "go to" box that takes seconds
(``123.4``), ``m:ss`` (``2:03.4``) or a depth (``40cm``: the next time the
depth crosses 40 cm, either way; ``-40cm`` the previous one; ``max``: the
deepest point).
"""

from __future__ import annotations

import math
import re
from datetime import datetime
from typing import Optional

from PyQt6.QtCore import QObject, Qt, pyqtSignal
from PyQt6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QLineEdit, QPushButton, QWidget

from wavi_py.transports.replay_transport import SPEEDS

STEP_S = 5.0
FINE_STEP_S = 1.0


def fmt_t(t: float, decimals: int = 1) -> str:
    """Session seconds as m:ss.s (h:mm:ss.s past an hour)."""
    sign = "-" if t < 0 else ""
    t = abs(t)
    h, rem = divmod(t, 3600)
    m, s = divmod(rem, 60)
    w = 3 + decimals if decimals else 2
    if h:
        return f"{sign}{int(h)}:{int(m):02d}:{s:0{w}.{decimals}f}"
    return f"{sign}{int(m)}:{s:0{w}.{decimals}f}"


def parse_t(text: str) -> Optional[float]:
    """``123.4``, ``2:03.4`` or ``1:02:03`` -> seconds; None if not a time."""
    text = text.strip()
    if not re.fullmatch(r"\d+(\.\d*)?(:\d+(\.\d*)?){0,2}", text):
        return None
    secs = 0.0
    for part in text.split(":"):
        secs = secs * 60 + float(part)
    return secs


class ReplayModel(QObject):
    positionChanged = pyqtSignal(float)        # session seconds
    stateChanged = pyqtSignal(str)             # playing / paused / eof / closed
    speedChanged = pyqtSignal(object)          # float, or None = max
    attached = pyqtSignal()                    # a recording was opened (index available)
    seeked = pyqtSignal(float)                 # a seek was asked for (GUI side)

    def __init__(self) -> None:
        super().__init__()
        self.transport = None
        self.index = None
        self._pos = math.nan
        self._state = "closed"

    # ----- binding

    def attach(self, transport, index) -> None:
        self.transport, self.index = transport, index
        self._pos, self._state = math.nan, ""
        self.attached.emit()
        self.speedChanged.emit(transport.speed)
        self.poll()

    def detach(self) -> None:
        self.transport = None
        self._state = "closed"
        self.stateChanged.emit("closed")

    @property
    def active(self) -> bool:
        return self.transport is not None

    def poll(self) -> None:
        tr = self.transport
        if tr is None:
            return
        pos, state = tr.position, tr.state
        if state != self._state:
            self._state = state
            self.stateChanged.emit(state)
        if not abs(pos - self._pos) < 1e-4:
            self._pos = pos
            self.positionChanged.emit(pos)

    # ----- state

    @property
    def position(self) -> float:
        return self._pos if self.active else math.nan

    @property
    def state(self) -> str:
        return self._state

    @property
    def duration(self) -> float:
        return self.index.duration if self.index is not None else 0.0

    @property
    def speed(self) -> Optional[float]:
        return self.transport.speed if self.transport is not None else 1.0

    def wall_clock(self, t: float) -> Optional[datetime]:
        if self.index is None or not math.isfinite(t):
            return None
        return datetime.fromtimestamp(self.index.unix0 + t)

    # ----- control

    def play(self) -> None:
        if self.transport is not None:
            self.transport.play()
            self.poll()

    def pause(self) -> None:
        if self.transport is not None:
            self.transport.pause()
            self.poll()

    def toggle(self) -> None:
        if self.transport is not None:
            self.transport.toggle()
            self.poll()

    def seek(self, t: float, preroll_s: Optional[float] = None) -> None:
        if self.transport is None:
            return
        t = min(max(0.0, float(t)), self.duration)
        self.transport.seek(t, preroll_s)
        self.seeked.emit(t)
        self.poll()

    def step(self, dt: float) -> None:
        if self.active:
            self.seek(self.position + dt)

    def set_speed(self, speed: Optional[float]) -> None:
        if self.transport is not None:
            self.transport.speed = speed
            self.speedChanged.emit(speed)

    def speed_step(self, up: bool) -> None:
        s = self.speed
        if s is None:
            return
        i = min(range(len(SPEEDS)), key=lambda k: abs(SPEEDS[k] - s))
        self.set_speed(SPEEDS[max(0, min(len(SPEEDS) - 1, i + (1 if up else -1)))])

    def goto(self, text: str) -> Optional[str]:
        """Seek to what ``text`` says (see the module doc). Returns an error, or None."""
        text = text.strip().lower()
        idx = self.index
        if idx is None:
            return "no recording"
        if text in ("max", "max depth", "deepest"):
            m = idx.max_depth()
            if m is None:
                return "no depth in this recording"
            self.seek(m[0])
            return None
        m = re.fullmatch(r"(-?)\s*(\d+(?:\.\d*)?)\s*cm", text)
        if m:
            back = m.group(1) == "-"
            t = idx.depth_crossing(self.position, float(m.group(2)), forward=not back)
            if t is None:
                return (f"the depth doesn't cross {m.group(2)} cm "
                        f"{'before' if back else 'after'} {fmt_t(self.position)}")
            self.seek(t)
            return None
        t = parse_t(text)
        if t is None:
            return "a time (123.4 or 2:03.4), a depth (40cm, -40cm) or max"
        self.seek(t)
        return None


class ReplayControls(QWidget):
    """Play / pause, speed, steps, the time readout and "go to", bound to a model."""

    def __init__(self, model: ReplayModel, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._model = model
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        self._back = QPushButton("◀◀")
        self._back.setToolTip(f"Back {STEP_S:g} s")
        self._back.clicked.connect(lambda: model.step(-STEP_S))
        self._play = QPushButton("▶ Play")
        self._play.setToolTip("Play / pause the recording [Space]")
        self._play.clicked.connect(model.toggle)
        self._fwd = QPushButton("▶▶")
        self._fwd.setToolTip(f"Forward {STEP_S:g} s")
        self._fwd.clicked.connect(lambda: model.step(STEP_S))
        self._speed = QComboBox()
        for s in SPEEDS:
            self._speed.addItem(f"{s:g}x", s)
        self._speed.addItem("max", None)
        self._speed.setToolTip("Playback speed (max: as fast as it can be read)")
        self._speed.setCurrentIndex(SPEEDS.index(1.0))
        self._speed.activated.connect(lambda i: model.set_speed(self._speed.itemData(i)))
        self._time = QLabel("–")
        self._time.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._goto = QLineEdit()
        self._goto.setPlaceholderText("go to: 2:03 / 40cm / max")
        self._goto.setToolTip("Seek to a time (seconds or m:ss), to the next time the depth\n"
                              "crosses a value (40cm; -40cm: the previous one), or to the\n"
                              "deepest point (max). Enter to go.")
        self._goto.setFixedWidth(150)
        self._goto.returnPressed.connect(self._on_goto)
        for w in (self._back, self._play, self._fwd):
            w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            w.setMaximumWidth(80)
        h.addWidget(self._back)
        h.addWidget(self._play)
        h.addWidget(self._fwd)
        h.addWidget(self._speed)
        h.addWidget(self._time, stretch=1)
        h.addWidget(self._goto)
        model.positionChanged.connect(self._show_time)
        model.stateChanged.connect(self._show_state)
        model.speedChanged.connect(self._show_speed)
        model.attached.connect(lambda: self._show_time(model.position))

    def _on_goto(self) -> None:
        err = self._model.goto(self._goto.text())
        self._goto.setStyleSheet("QLineEdit { background: #fdd; }" if err else "")
        self._goto.setToolTip(err or self._goto.toolTip())
        if err:
            self._time.setText(err)

    def _show_state(self, state: str) -> None:
        self._play.setText("❚❚ Pause" if state == "playing" else "▶ Play")
        on = state != "closed"
        for w in (self._back, self._play, self._fwd, self._speed, self._goto):
            w.setEnabled(on)

    def _show_speed(self, s) -> None:
        i = self._speed.findData(s)
        if i >= 0 and i != self._speed.currentIndex():
            self._speed.setCurrentIndex(i)

    def _show_time(self, t: float) -> None:
        m = self._model
        if not m.active or not math.isfinite(t):
            self._time.setText("–")
            return
        parts = [f"{fmt_t(t)} / {fmt_t(m.duration)}"]
        wall = m.wall_clock(t)
        if wall is not None:
            parts.append(f"{wall:%H:%M:%S}")
        idx = m.index
        if idx is not None:
            d = idx.depth_at(t)
            if d is not None:
                parts.append(f"depth {d:.0f} cm")
            if idx.armed_spans:
                parts.append("ARMED" if idx.armed_at(t) else "disarmed")
        self._time.setText("  ·  ".join(parts))
