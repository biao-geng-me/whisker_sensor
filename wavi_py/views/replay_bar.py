"""Replay / timeline: one shared ``ReplayModel`` and the widgets bound to it.

``ReplayModel`` wraps the replay transport (``transports/replay_transport.py``,
thread-safe) for the GUI thread: ``poll()`` (the control panel's 20 Hz tick)
reads the playhead and emits ``positionChanged`` / ``stateChanged``, so every
widget bound to the model shows the same thing, whichever window it sits in.
Its ``source`` is the timeline (``replay/events.py``: events, armed spans,
depth): the recording's index in replay, or a live stream so far
(``set_live``; the panel moves its position to the newest sample, and a seek
asks the panel to show that time from the plot buffers, ``liveSeekRequested``).

``TimelineBar`` is the timeline, one per window (a dock at the bottom of every
view window and of the control panel): the controls row (replay only), a strip
with the depth (cm, down), the armed spans (green), the events as ticks
coloured by kind / severity (hover for the text), the playhead (drag to scrub:
a short-preroll seek at most every ``SCRUB_S``, a full one on release; click
anywhere to go there) and the loop range (Loop: ``LOOP_S`` either side of the
playhead; drag its edges), an event list (picking one goes to ``EVENT_LEAD_S``
before it) and previous / next event.

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
import time
from datetime import datetime
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QObject, QPoint, Qt, pyqtSignal
from PyQt6.QtWidgets import (QComboBox, QHBoxLayout, QLabel, QLineEdit, QPushButton, QToolTip,
                             QVBoxLayout, QWidget)

from wavi_py.transports.replay_transport import SPEEDS

STEP_S = 5.0
FINE_STEP_S = 1.0
SCRUB_S = 0.15                  # dragging the playhead: a seek at most this often ...
SCRUB_PREROLL_S = 2.0           # ... with this much preroll (a full one on release)
EVENT_LEAD_S = 2.0              # going to an event lands this far before it
LOOP_S = 5.0                    # Loop: this either side of the playhead

# event tick colours: by kind, STATUSTEXT by level
KIND_COLORS = {"arm": (0, 150, 0), "disarm": (90, 130, 90), "mode": (130, 60, 170),
               "seq_gap": (200, 120, 0), "counter": (200, 160, 40), "fault": (170, 0, 0),
               "mark": (0, 120, 160)}
LEVEL_COLORS = {"fault": (170, 0, 0), "error": (220, 30, 30), "warn": (230, 140, 0),
                "info": (60, 90, 200), "debug": (150, 150, 150)}


def event_color(e) -> tuple:
    if e.kind == "statustext" or e.kind not in KIND_COLORS:
        return LEVEL_COLORS.get(e.level, LEVEL_COLORS["info"])
    return KIND_COLORS[e.kind]


def event_text(e) -> str:
    n = f" (x{e.count}" + (f", to {fmt_t(e.t_last)})" if e.t_last is not None else ")") \
        if e.count > 1 else ""
    return f"{fmt_t(e.t)}  {e.text}{n}"


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
    stateChanged = pyqtSignal(str)             # playing / paused / eof / closed / live
    speedChanged = pyqtSignal(object)          # float, or None = max
    sourceChanged = pyqtSignal()               # a recording opened / a live stream started
    sourceUpdated = pyqtSignal()               # live: more events / depth since
    seeked = pyqtSignal(float)                 # a seek was asked for (GUI side)
    loopChanged = pyqtSignal(object)           # (t0, t1) or None
    liveSeekRequested = pyqtSignal(float)      # live: show this time (from the buffers)

    def __init__(self) -> None:
        super().__init__()
        self.transport = None
        self.source = None                     # TimelineQueries
        self._pos = math.nan
        self._state = "closed"
        self._scrub_t = 0.0
        self._loop: Optional[tuple[float, float]] = None

    # ----- binding

    def attach(self, transport, index) -> None:
        """A recording opened: ``transport`` plays ``index``."""
        self.transport, self.source = transport, index
        self._pos, self._state, self._loop = math.nan, "", None
        self.sourceChanged.emit()
        self.speedChanged.emit(transport.speed)
        self.loopChanged.emit(None)
        self.poll()

    def set_live(self, timeline) -> None:
        """A live stream: ``timeline`` its events so far (None: nothing to show)."""
        self.transport, self.source = None, timeline
        self._pos, self._state, self._loop = math.nan, "live" if timeline else "closed", None
        self.sourceChanged.emit()
        self.stateChanged.emit(self._state)

    def detach(self) -> None:
        self.transport = self.source = None
        self._state, self._loop = "closed", None
        self.sourceChanged.emit()
        self.stateChanged.emit("closed")

    @property
    def active(self) -> bool:
        """Replaying (a recording is open)."""
        return self.transport is not None

    @property
    def index(self):
        return self.source if self.active else None

    def set_live_position(self, t: float) -> None:
        if self.transport is None and self.source is not None and t != self._pos:
            self._pos = float(t)
            self.positionChanged.emit(self._pos)

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
        return self._pos if self.source is not None else math.nan

    @property
    def state(self) -> str:
        return self._state

    @property
    def duration(self) -> float:
        return self.source.duration if self.source is not None else 0.0

    @property
    def loop(self) -> Optional[tuple[float, float]]:
        return self._loop

    @property
    def speed(self) -> Optional[float]:
        return self.transport.speed if self.transport is not None else 1.0

    def wall_clock(self, t: float) -> Optional[datetime]:
        src = self.source
        if src is None or not math.isfinite(t) or not getattr(src, "unix0", 0):
            return None
        return datetime.fromtimestamp(src.unix0 + t)

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
            if self.source is not None:
                self.liveSeekRequested.emit(float(t))
            return
        t = min(max(0.0, float(t)), self.duration)
        self.transport.seek(t, preroll_s)
        self.seeked.emit(t)
        self.poll()

    def request_seek(self, t: float, final: bool) -> None:
        """A seek from the timeline: while dragging (``final`` False) a short-preroll
        one at most every SCRUB_S; on release the full one."""
        if not final:
            now = time.monotonic()
            if now - self._scrub_t < SCRUB_S:
                return
            self._scrub_t = now
            self.seek(t, SCRUB_PREROLL_S)
        else:
            self.seek(t)

    def step(self, dt: float) -> None:
        if self.active:
            self.seek(self.position + dt)

    def event_step(self, forward: bool) -> None:
        """Go to the next / previous event (EVENT_LEAD_S before it)."""
        src = self.source
        if src is None or not math.isfinite(self.position):
            return
        # from where the last jump landed, so repeated presses walk the list
        e = src.event_near(self.position + EVENT_LEAD_S, forward)
        if e is not None:
            self.seek(max(0.0, e.t - EVENT_LEAD_S))

    def set_loop(self, rng: Optional[tuple[float, float]]) -> None:
        """Loop over ``rng`` (session seconds); None ends it. Replay only."""
        if self.transport is None:
            rng = None
        if rng is not None:
            rng = (max(0.0, float(rng[0])), min(self.duration, float(rng[1])))
            if rng[1] - rng[0] < 0.2:
                rng = None
        self._loop = rng
        if self.transport is not None:
            self.transport.set_loop(*(rng if rng else (None, None)))
        self.loopChanged.emit(rng)
        self.poll()

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
        idx = self.source
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
        model.sourceChanged.connect(lambda: self._show_time(model.position))

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
        idx = m.source
        if idx is not None:
            d = idx.depth_at(t)
            if d is not None:
                parts.append(f"depth {d:.0f} cm")
            if idx.armed_spans:
                parts.append("ARMED" if idx.armed_at(t) else "disarmed")
        self._time.setText("  ·  ".join(parts))


class _Strip(pg.PlotWidget):
    """The timeline strip: depth, armed spans, event ticks, playhead, loop."""

    def __init__(self, bar: "TimelineBar"):
        super().__init__()
        self._bar = bar
        self.setBackground("w")
        self.setFixedHeight(96)
        pi = self.getPlotItem()
        pi.hideButtons()
        pi.setMenuEnabled(False)
        pi.setMouseEnabled(x=True, y=False)
        pi.showGrid(x=True, y=False, alpha=0.25)
        pi.getViewBox().invertY(True)                    # depth goes down
        pi.getViewBox().setLimits(xMin=-1.0)
        pi.setLabel("left", "depth", units="cm")
        pi.getAxis("left").enableAutoSIPrefix(False)
        pi.getAxis("left").setWidth(42)
        self.depth = pi.plot(pen=pg.mkPen((40, 90, 170), width=1.5))
        self.depth.setDownsampling(auto=True, method="peak")
        self.depth.setClipToView(True)
        self.zoomed = False                              # the user zoomed / panned time
        pi.getViewBox().sigRangeChangedManually.connect(lambda *_: setattr(self, "zoomed", True))
        self.ticks: list[pg.PlotCurveItem] = []
        self.spans: list[pg.LinearRegionItem] = []
        self.loop = pg.LinearRegionItem(movable=True, brush=pg.mkBrush(60, 120, 255, 45))
        self.loop.setZValue(5)
        self.loop.hide()
        pi.addItem(self.loop, ignoreBounds=True)
        self.head = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen((210, 0, 0), width=2),
                                    hoverPen=pg.mkPen((255, 60, 60), width=4))
        self.head.setZValue(10)
        pi.addItem(self.head, ignoreBounds=True)
        self.scene().sigMouseClicked.connect(self._clicked)
        self.scene().sigMouseMoved.connect(self._hover)
        self._ev_t = np.empty(0)
        self._events: list = []

    def set_events(self, events, y0: float, y1: float) -> None:
        pi = self.getPlotItem()
        for c in self.ticks:
            pi.removeItem(c)
        self.ticks = []
        self._events = list(events)
        self._ev_t = np.asarray([e.t for e in self._events])
        by_color: dict = {}
        for e in self._events:
            by_color.setdefault(event_color(e), []).append(e.t)
        for color, ts in by_color.items():
            x = np.repeat(np.asarray(ts), 2)
            y = np.tile([y0, y0 + 0.35 * (y1 - y0)], len(ts))
            c = pg.PlotCurveItem(x, y, connect="pairs", pen=pg.mkPen(color, width=2))
            c.setZValue(3)
            pi.addItem(c, ignoreBounds=True)
            self.ticks.append(c)

    def set_spans(self, spans) -> None:
        pi = self.getPlotItem()
        for r in self.spans:
            pi.removeItem(r)
        self.spans = []
        for a, b in spans:
            r = pg.LinearRegionItem((a, b), movable=False, brush=pg.mkBrush(0, 170, 0, 35),
                                    pen=pg.mkPen(None))
            r.setZValue(-5)
            pi.addItem(r, ignoreBounds=True)
            self.spans.append(r)

    def _x_at(self, scene_pos) -> Optional[float]:
        vb = self.getPlotItem().getViewBox()
        if not vb.sceneBoundingRect().contains(scene_pos):
            return None
        return float(vb.mapSceneToView(scene_pos).x())

    def _clicked(self, ev) -> None:
        if ev.button() != Qt.MouseButton.LeftButton:
            return
        if ev.double():                                  # back to the whole recording
            self.zoomed = False
            self._bar.show_all()
            return
        x = self._x_at(ev.scenePos())
        if x is not None:
            self._bar.model.request_seek(x, True)

    def _hover(self, pos) -> None:
        x = self._x_at(pos)
        if x is None or not len(self._ev_t):
            return
        vb = self.getPlotItem().getViewBox()
        px = vb.viewPixelSize()[0]
        i = int(np.argmin(np.abs(self._ev_t - x)))
        if abs(self._ev_t[i] - x) <= 5 * px:
            near = [e for e in self._events if abs(e.t - self._ev_t[i]) <= 5 * px]
            QToolTip.showText(self.mapToGlobal(QPoint(int(self.mapFromScene(pos).x()), 0)),
                              "\n".join(event_text(e) for e in near[:12]), self)


class TimelineBar(QWidget):
    """The timeline of the shared model (see the module doc); one per window."""

    def __init__(self, model: ReplayModel, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.model = model
        v = QVBoxLayout(self)
        v.setContentsMargins(4, 2, 4, 2)
        v.setSpacing(2)
        self.controls = ReplayControls(model)
        v.addWidget(self.controls)
        h = QHBoxLayout()
        v.addLayout(h)
        self.strip = _Strip(self)
        h.addWidget(self.strip, stretch=1)
        side = QVBoxLayout()
        h.addLayout(side)
        self.events = QComboBox()
        self.events.setMinimumWidth(200)
        self.events.setMaximumWidth(320)
        self.events.setToolTip(f"Events: pick one to go {EVENT_LEAD_S:g} s before it. Strip:"
                               " click to go there, drag the red playhead to scrub, wheel / drag"
                               " to zoom / pan time, double-click for all of it")
        self.events.activated.connect(self._on_event_picked)
        side.addWidget(self.events)
        row = QHBoxLayout()
        side.addLayout(row)
        self._prev = QPushButton("◀ event")
        self._prev.setToolTip("Previous event [,]")
        self._prev.clicked.connect(lambda: model.event_step(False))
        self._next = QPushButton("event ▶")
        self._next.setToolTip("Next event [.]")
        self._next.clicked.connect(lambda: model.event_step(True))
        self.loop_btn = QPushButton("Loop")
        self.loop_btn.setCheckable(True)
        self.loop_btn.setToolTip(f"Loop over {LOOP_S:g} s either side of the playhead; drag the"
                                 " blue range's edges to change it")
        self.loop_btn.toggled.connect(self._on_loop_toggled)
        for w in (self._prev, self._next, self.loop_btn):
            w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            row.addWidget(w)
        side.addStretch(1)
        self._dragging = False
        self.strip.head.sigDragged.connect(self._on_drag)
        self.strip.head.sigPositionChangeFinished.connect(self._on_drag_done)
        self.strip.loop.sigRegionChangeFinished.connect(self._on_loop_dragged)
        model.sourceChanged.connect(self._rebuild)
        model.sourceUpdated.connect(self._rebuild)
        model.positionChanged.connect(self._show_position)
        model.loopChanged.connect(self._show_loop)
        model.stateChanged.connect(self._show_state)
        self._rebuild()

    # ----- model -> widgets

    def _rebuild(self) -> None:
        m = self.model
        src = m.source
        self.controls.setVisible(m.active)
        self.loop_btn.setEnabled(m.active)
        self.strip.head.setMovable(True)
        if src is None:
            self.strip.depth.setData([], [])
            self.strip.set_events([], 0, 1)
            self.strip.set_spans([])
            self.events.clear()
            return
        t, d = src.depth()
        self.strip.depth.setData(t, d)
        if len(d):
            y0, y1 = min(0.0, float(np.nanmin(d))), max(10.0, float(np.nanmax(d)))
        else:
            y0, y1 = 0.0, 1.0
        pad = 0.08 * (y1 - y0)
        self.strip.getPlotItem().setYRange(y0 - pad, y1 + pad, padding=0)
        self.strip.getPlotItem().getAxis("left").setStyle(showValues=bool(len(d)))
        evs = src.events
        self.strip.set_events(evs, y0 - pad, y1 + pad)
        self.strip.set_spans(src.armed_spans)
        self.strip.getPlotItem().getViewBox().setLimits(xMax=max(1.0, src.duration) + 1.0)
        if not self.strip.zoomed:
            self.show_all()
        self.events.clear()
        self.events.addItem(f"Events ({len(evs)})" if evs else "No events", None)
        for e in evs:
            self.events.addItem(event_text(e), e.t)
        self._show_position(m.position)

    def show_all(self) -> None:
        """The whole recording (stream so far) on the strip."""
        src = self.model.source
        dur = max(1.0, src.duration) if src is not None else 1.0
        self.strip.getPlotItem().setXRange(0, dur, padding=0.01)

    def _show_position(self, t: float) -> None:
        if self._dragging or not math.isfinite(t):
            return
        self.strip.head.blockSignals(True)
        self.strip.head.setValue(t)
        self.strip.head.blockSignals(False)

    def _show_state(self, state: str) -> None:
        self.controls.setVisible(self.model.active)

    def _show_loop(self, rng) -> None:
        on = rng is not None
        self.strip.loop.blockSignals(True)
        if on:
            self.strip.loop.setRegion(rng)
        self.strip.loop.setVisible(on)
        self.strip.loop.blockSignals(False)
        self.loop_btn.blockSignals(True)
        self.loop_btn.setChecked(on)
        self.loop_btn.blockSignals(False)

    # ----- widgets -> model

    def _on_drag(self, line) -> None:
        self._dragging = True
        self.model.request_seek(float(line.value()), False)

    def _on_drag_done(self, line) -> None:
        self._dragging = False
        self.model.request_seek(float(line.value()), True)

    def _on_event_picked(self, i: int) -> None:
        t = self.events.itemData(i)
        if t is not None:
            self.model.seek(max(0.0, float(t) - EVENT_LEAD_S))
        self.events.setCurrentIndex(0)

    def _on_loop_toggled(self, on: bool) -> None:
        p = self.model.position
        self.model.set_loop((p - LOOP_S, p + LOOP_S) if on and math.isfinite(p) else None)

    def _on_loop_dragged(self, region) -> None:
        self.model.set_loop(tuple(region.getRegion()))
