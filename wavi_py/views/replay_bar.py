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
the model's position follows the mouse at once, ``scrubbed`` / ``positionChanged``,
so the Line and IMU windows, which draw from the recording's store, move with
it; the transport gets a short-preroll seek at most every ``SCRUB_S`` and a
full one on release, which the windows fed by the pipeline catch up on; click
anywhere to go there), the Line window's visible time span (the model's
``view_span``: a light red band with a thin red line at its right edge, display
only, under the playhead and taking no mouse; hidden while the Line window is
closed) and the loop range (Loop: ``LOOP_S`` either side of the
playhead; drag its edges), an event list (picking one goes to ``EVENT_LEAD_S``
before it), previous / next event, and Export (``exportRequested``: the panel
saves the loop range, see replay/export.py), and Notes (``notesRequested``:
the panel opens the notes list).

ROV lanes (``ROV ▾`` on the side, or a right-click on the strip): the vehicle
telemetry (``replay/rov.py``, the source's ``rov()``: pilot demand, thrusters,
thrust per axis, current, climb, heading, vibration, and the flight-mode band)
as optional rows under the strip, one row per unit (``_Lane``: x linked to the
strip, its own playhead, click to go there, double-click for all of it, hover
for the values at the mouse). Each timeline has its own choice; the last one
made is kept (``timeline/rov_lanes`` in the app settings) and new timelines
start on it. A recording without the telemetry shows none (the choice is kept).

Notes (``views/notes_panel.py``, the model's ``notes``, set by the panel): the
pinned notes of the recording are amber triangles along the strip's bottom
edge (hover for the text, click to go to the note's time); a recording marked
bad tints the strip pink with a "BAD" label (its reason as the tooltip).

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
from PyQt6.QtGui import QAction, QColor
from PyQt6.QtWidgets import (QComboBox, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMenu, QPushButton,
                             QSizePolicy, QToolButton, QToolTip, QVBoxLayout, QWidget)

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
    exportRequested = pyqtSignal()             # Export...: the panel saves a range
    seeked = pyqtSignal(float)                 # a seek was asked for (GUI side)
    loopChanged = pyqtSignal(object)           # (t0, t1) or None
    liveSeekRequested = pyqtSignal(float)      # live: show this time (from the buffers)
    scrubbed = pyqtSignal(float)               # the playhead is being dragged here (mouse rate)
    viewSpanChanged = pyqtSignal(object)       # the Line window's time span (t0, t1), or None
    notesChanged = pyqtSignal()                # the notes model's recording / lines / offset
    notesRequested = pyqtSignal()              # Notes on a timeline: open the notes list

    def __init__(self) -> None:
        super().__init__()
        self.transport = None
        self.source = None                     # TimelineQueries
        self._pos = math.nan
        self._state = "closed"
        self._scrub_t = 0.0
        self._scrubbing = False                # a playhead drag: the position is the mouse's
        self._loop: Optional[tuple[float, float]] = None
        self._view_span: Optional[tuple[float, float]] = None
        self.notes = None                      # views/notes_panel.NotesModel, see set_notes

    # ----- binding

    def set_notes(self, notes) -> None:
        """The notes model the timelines draw (its ``changed`` re-emitted here)."""
        self.notes = notes
        notes.changed.connect(self.notesChanged.emit)
        self.notesChanged.emit()

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
        if self._scrubbing:
            return                             # the mouse has the position until release
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
    def view_span(self) -> Optional[tuple[float, float]]:
        """The Line window's visible time span (None: it is closed)."""
        return self._view_span

    def set_view_span(self, rng: Optional[tuple[float, float]]) -> None:
        """The panel passes the Line window's X range on every change (live, it moves
        with each redraw while following) and None when the window closes."""
        rng = None if rng is None else (float(rng[0]), float(rng[1]))
        if rng != self._view_span:
            self._view_span = rng
            self.viewSpanChanged.emit(rng)

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
        """A seek from the timeline: while dragging (``final`` False) the position
        follows the mouse at once (``scrubbed``, ``positionChanged``) and the transport
        gets a short-preroll seek at most every SCRUB_S; on release the full one."""
        if not final:
            t = min(max(0.0, float(t)), self.duration)
            self._scrubbing = self.transport is not None
            if self._scrubbing and t != self._pos:
                self._pos = t
                self.scrubbed.emit(t)
                self.positionChanged.emit(t)
            now = time.monotonic()
            if now - self._scrub_t < SCRUB_S:
                return
            self._scrub_t = now
            self.seek(t, SCRUB_PREROLL_S)
        else:
            self._scrubbing = False
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
        # cut at the right in a narrow window instead of setting the window's width,
        # but the position always shows
        self._time.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self._time.setMinimumWidth(self._time.fontMetrics().horizontalAdvance("00:00.0 / 00:00.0"))
        self._goto = QLineEdit()
        self._goto.setPlaceholderText("go to: 2:03 / 40cm / max")
        self._goto.setToolTip("Seek to a time (seconds or m:ss), to the next time the depth\n"
                              "crosses a value (40cm; -40cm: the previous one), or to the\n"
                              "deepest point (max). Enter to go.")
        self._goto.setMinimumWidth(100)
        self._goto.setMaximumWidth(150)
        self._goto.returnPressed.connect(self._on_goto)
        for w, widest in ((self._back, "◀◀"), (self._play, "❚❚ Pause"), (self._fwd, "▶▶")):
            w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            w.setMaximumWidth(80)
            w.setMinimumWidth(w.fontMetrics().horizontalAdvance(widest) + 20)   # not the style's 80
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


# the Line window's span on the strip: a light red band and a thin, lighter red right
# edge, so the playhead (2 px, dark red, on top) stays the one that reads as "the line"
VIEW_SPAN_BRUSH = (255, 0, 50, 45)      # pinkish over the green armed spans, not the orange of selection 1
VIEW_EDGE_PEN = (235, 70, 70)


class _Strip(pg.PlotWidget):
    """The timeline strip: depth, armed spans, event ticks, playhead, loop, view span."""

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
        # the Line window's span: display only (no mouse: clicks, drags and the playhead
        # go through); the band under everything, its edge under the loop and playhead
        self.view_span = pg.LinearRegionItem(movable=False, brush=pg.mkBrush(*VIEW_SPAN_BRUSH),
                                             pen=pg.mkPen(None))
        self.view_span.setZValue(-12)
        self.view_edge = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen(VIEW_EDGE_PEN, width=1))
        self.view_edge.setZValue(4)
        for it in (self.view_span, *self.view_span.lines, self.view_edge):
            it.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            it.setAcceptHoverEvents(False)
        for it in (self.view_span, self.view_edge):
            it.hide()
            pi.addItem(it, ignoreBounds=True)
        self.head = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen((210, 0, 0), width=2),
                                    hoverPen=pg.mkPen((255, 60, 60), width=4))
        self.head.setZValue(10)
        pi.addItem(self.head, ignoreBounds=True)
        self.note_marks = pg.ScatterPlotItem(symbol="t1", size=12, pen=pg.mkPen((120, 70, 0), width=1),
                                             brush=pg.mkBrush(255, 180, 0))
        self.note_marks.setZValue(8)
        pi.addItem(self.note_marks, ignoreBounds=True)
        self.note_lines = pg.PlotCurveItem(connect="pairs",
                                           pen=pg.mkPen((230, 150, 0), width=1, style=Qt.PenStyle.DashLine))
        self.note_lines.setZValue(2)
        pi.addItem(self.note_lines, ignoreBounds=True)
        self.bad_label = QLabel("BAD", self)
        self.bad_label.setStyleSheet("QLabel { color: white; background: #c00; font-weight: bold;"
                                     " padding: 0px 4px; }")
        self.bad_label.hide()
        self.scene().sigMouseClicked.connect(self._clicked)
        self.scene().sigMouseMoved.connect(self._hover)
        self._ev_t = np.empty(0)
        self._events: list = []
        self._note_x = np.empty(0)                       # timeline x of the pinned notes
        self._notes: list = []

    def set_notes(self, notes: list, offset: Optional[float], bad: bool,
                  reason: str = "") -> None:
        """The pinned ``notes`` (note lines) at ``t + offset`` (None: not on this
        timeline); ``bad`` tints the strip."""
        self._notes = list(notes) if offset is not None else []
        self._note_x = np.asarray([float(n["t"]) + offset for n in self._notes])
        self._place_notes()
        self.setBackground((255, 228, 228) if bad else "w")
        self.bad_label.setVisible(bad)
        self.bad_label.setToolTip(f"Marked bad: {reason}" if reason else "Marked bad")
        self._place_bad_label()

    def _place_notes(self) -> None:
        """The markers 8 px above the bottom edge (whole, whatever the depth range),
        a dashed line up from each."""
        vb = self.getPlotItem().getViewBox()
        top, bottom = vb.viewRange()[1]                  # inverted: bottom is the max
        py = abs(vb.viewPixelSize()[1])
        y = bottom - 8 * py
        x = self._note_x
        self.note_marks.setData(x, np.full(len(x), y))
        self.note_lines.setData(np.repeat(x, 2), np.tile([top, y], len(x)))

    def _place_bad_label(self) -> None:
        if "bad_label" not in self.__dict__:             # a resize from PlotWidget's __init__
            return
        self.bad_label.adjustSize()
        self.bad_label.move(self.width() - self.bad_label.width() - 6, 3)

    def resizeEvent(self, ev) -> None:
        super().resizeEvent(ev)
        self._place_bad_label()
        if "note_marks" in self.__dict__:
            self._place_notes()

    def _note_near(self, scene_pos) -> Optional[int]:
        """The pinned note within 6 px of the mouse (x only), or None."""
        x = self._x_at(scene_pos)
        if x is None or not len(self._note_x):
            return None
        px = self.getPlotItem().getViewBox().viewPixelSize()[0]
        i = int(np.argmin(np.abs(self._note_x - x)))
        return i if abs(self._note_x[i] - x) <= 6 * px else None

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
        if ev.button() == Qt.MouseButton.RightButton:
            self._bar.lanes_menu().popup(ev.screenPos().toPoint())
            ev.accept()
            return
        if ev.button() != Qt.MouseButton.LeftButton:
            return
        if ev.double():                                  # back to the whole recording
            self.zoomed = False
            self._bar.show_all()
            return
        i = self._note_near(ev.scenePos())
        x = self._note_x[i] if i is not None else self._x_at(ev.scenePos())
        if x is not None:
            self._bar.model.request_seek(float(x), True)

    def _hover(self, pos) -> None:
        j = self._note_near(pos)
        if j is not None:
            from wavi_py.views.notes_panel import note_tooltip
            px = self.getPlotItem().getViewBox().viewPixelSize()[0]
            near = [n for n, x in zip(self._notes, self._note_x)
                    if abs(x - self._note_x[j]) <= 6 * px]
            QToolTip.showText(self.mapToGlobal(QPoint(int(self.mapFromScene(pos).x()), 0)),
                              "\n".join(note_tooltip(n) for n in near[:12]), self)
            return
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


# ----------------------------------------------------------------------
# ROV lanes: the vehicle telemetry under the strip
# ----------------------------------------------------------------------

LANE_H = 54                     # px of a lane (its y labels don't overlap)
LANE_AXIS_W = 42                # the strip's left axis width: the x axes line up
LANE_COLORS = ((40, 90, 170), (210, 90, 0), (20, 130, 60), (150, 30, 30), (110, 60, 160),
               (0, 120, 130), (130, 110, 0), (190, 70, 140))
LANES_SETTING = "timeline/rov_lanes"
_MODE_KEY = "rov:mode"


def _saved_lanes() -> list[str]:
    try:
        from wavi_py.remote.config import _settings
        v = _settings().value(LANES_SETTING)
    except Exception:                                    # noqa: BLE001 — a setting, never fatal
        return []
    if not v:
        return []
    return [k for k in (v if isinstance(v, list) else str(v).split(",")) if k]


def _save_lanes(keys: list[str]) -> None:
    try:
        from wavi_py.remote.config import _settings
        st = _settings()
        st.setValue(LANES_SETTING, ",".join(keys))
        st.sync()
    except Exception:                                    # noqa: BLE001
        pass


class _Lane(pg.PlotWidget):
    """One unit's row of ROV variables (or the mode band) under the strip."""

    def __init__(self, bar: "TimelineBar", unit: str):
        super().__init__()
        self._bar, self.unit = bar, unit
        self.setBackground("w")
        self.setFixedHeight(LANE_H)
        pi = self.getPlotItem()
        pi.hideButtons()
        pi.setMenuEnabled(False)
        pi.setMouseEnabled(x=True, y=False)
        pi.showGrid(x=True, y=unit != "mode", alpha=0.25)
        pi.hideAxis("bottom")
        ax = pi.getAxis("left")
        ax.enableAutoSIPrefix(False)
        ax.setWidth(LANE_AXIS_W)
        pi.setLabel("left", "mode" if unit == "mode" else unit)
        pi.setXLink(bar.strip.getPlotItem())
        self.head = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen((210, 0, 0), width=2))
        self.head.setZValue(10)
        pi.addItem(self.head, ignoreBounds=True)
        self.curves: list[tuple[str, str, pg.PlotDataItem, np.ndarray, np.ndarray]] = []
        self._band: list = []
        self.scene().sigMouseClicked.connect(self._clicked)
        self.scene().sigMouseMoved.connect(self._hover)

    def set_curves(self, items: list[tuple[str, str, np.ndarray, np.ndarray]]) -> None:
        """[(key, label, t, y)]: one curve each, coloured in order."""
        pi = self.getPlotItem()
        for _k, _l, c, _t, _y in self.curves:
            pi.removeItem(c)
        self.curves = []
        lo, hi = math.inf, -math.inf
        for i, (key, label, t, y) in enumerate(items):
            c = pi.plot(t, y, pen=pg.mkPen(LANE_COLORS[i % len(LANE_COLORS)], width=1.3))
            c.setDownsampling(auto=True, method="peak")
            c.setClipToView(True)
            self.curves.append((key, label, c, np.asarray(t), np.asarray(y)))
            fin = np.asarray(y)[np.isfinite(y)]
            if len(fin):
                lo, hi = min(lo, float(fin.min())), max(hi, float(fin.max()))
        if self.unit in ("pilot", "thrust"):             # -1 .. +1: the stick's range
            lo, hi = min(lo, -1.0) if math.isfinite(lo) else -1.0, max(hi, 1.0) if math.isfinite(hi) else 1.0
        if not math.isfinite(lo):
            lo, hi = 0.0, 1.0
        if hi - lo < 1e-9:
            lo, hi = lo - 0.5, hi + 0.5
        pad = 0.06 * (hi - lo)
        pi.setYRange(lo - pad, hi + pad, padding=0)
        self.setToolTip("\n".join(f"{lab}" for _k, lab, *_ in self.curves))

    def set_modes(self, modes, gain) -> None:
        """The flight-mode band: one labelled span per mode, the pilot gain as a step."""
        from wavi_py.views.analysis_timeline import MODE_COLORS, _MODE_OTHER
        pi = self.getPlotItem()
        for it in self._band:
            pi.removeItem(it)
        self._band = []
        self._modes = list(modes)
        for t0, t1, name in modes:
            r = pg.LinearRegionItem((t0, t1), movable=False, pen=pg.mkPen(None),
                                    brush=pg.mkBrush(*MODE_COLORS.get(name, _MODE_OTHER), 230))
            r.setZValue(-5)
            for ln in r.lines:
                ln.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            r.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            pi.addItem(r, ignoreBounds=True)
            lab = pg.TextItem(name, color=(60, 60, 60), anchor=(0, 0))
            lab.setPos(t0, 1.0)
            pi.addItem(lab, ignoreBounds=True)
            self._band += [r, lab]
        gt, gy = gain
        if len(gt):
            c = pi.plot(gt, gy, pen=pg.mkPen((40, 40, 40), width=1.5))
            self._band.append(c)
        pi.setYRange(0.0, 1.0, padding=0.05)
        self.getPlotItem().getAxis("left").setTicks([[(0, "0"), (0.5, "gain"), (1, "1")]])

    def _x_at(self, scene_pos) -> Optional[float]:
        vb = self.getPlotItem().getViewBox()
        if not vb.sceneBoundingRect().contains(scene_pos):
            return None
        return float(vb.mapSceneToView(scene_pos).x())

    def _clicked(self, ev) -> None:
        if ev.button() == Qt.MouseButton.RightButton:
            self._bar.lanes_menu().popup(ev.screenPos().toPoint())
            ev.accept()
            return
        if ev.button() != Qt.MouseButton.LeftButton:
            return
        if ev.double():
            self._bar.strip.zoomed = False
            self._bar.show_all()
            return
        x = self._x_at(ev.scenePos())
        if x is not None:
            self._bar.model.request_seek(x, True)

    def text_at(self, x: float) -> str:
        """The values at time ``x``: one line per curve (the mode band: the mode)."""
        if self.unit == "mode":
            name = next((n for a, b, n in getattr(self, "_modes", []) if a <= x <= b), None)
            return f"{fmt_t(x)}  mode {name}" if name else f"{fmt_t(x)}"
        lines = [fmt_t(x)]
        for _k, label, _c, t, y in self.curves:
            if not len(t):
                continue
            i = int(np.clip(np.searchsorted(t, x), 0, len(t) - 1))
            if i > 0 and abs(t[i - 1] - x) < abs(t[i] - x):
                i -= 1
            lines.append(f"{label}: {y[i]:.3g} {self.unit}")
        return "\n".join(lines)

    def _hover(self, pos) -> None:
        x = self._x_at(pos)
        if x is None:
            return
        QToolTip.showText(self.mapToGlobal(QPoint(int(self.mapFromScene(pos).x()), 0)),
                          self.text_at(x), self)


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
        col = QVBoxLayout()                # the strip over the ROV lanes
        col.setSpacing(0)
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self.strip)
        self._lane_box = QVBoxLayout()
        self._lane_box.setSpacing(0)
        col.addLayout(self._lane_box)
        h.addLayout(col, stretch=1)
        self.lanes: dict[str, _Lane] = {}               # unit -> its lane (shown ones)
        self.rov_keys: list[str] = _saved_lanes()        # the ROV variables chosen
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
        row = QGridLayout()               # 2 x 2: the side column stays narrow
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
        self.export_btn = QPushButton("Export...")
        self.export_btn.setToolTip("Save the loop range (without Loop: the Line window's time"
                                   " span) as a .jsonl clip or a .dat")
        self.export_btn.clicked.connect(model.exportRequested.emit)
        self.notes_btn = QPushButton("Notes")
        self.notes_btn.setToolTip("The recording's notes: list, add, edit, delete [N: add one]")
        self.notes_btn.clicked.connect(model.notesRequested.emit)
        self.rov_btn = QToolButton()
        self.rov_btn.setText("ROV ▾")
        self.rov_btn.setToolTip("ROV lanes: show the vehicle telemetry (pilot, thrusters,"
                                " current, mode, ...) under the strip [or right-click the strip]")
        self.rov_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.rov_btn.clicked.connect(lambda: self.lanes_menu().exec(
            self.rov_btn.mapToGlobal(QPoint(0, self.rov_btn.height()))))
        for i, w in enumerate((self._prev, self._next, self.loop_btn, self.export_btn,
                               self.notes_btn, self.rov_btn)):
            w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            row.addWidget(w, i // 2, i % 2)
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
        model.viewSpanChanged.connect(self._show_view_span)
        model.notesChanged.connect(self._show_notes)
        self._rebuild()

    # ----- model -> widgets

    def _rebuild(self) -> None:
        m = self.model
        src = m.source
        self.controls.setVisible(m.active)
        self.loop_btn.setEnabled(m.active)
        self.export_btn.setVisible(m.active)
        self.strip.head.setMovable(True)
        self._show_view_span(m.view_span)
        if src is None:
            self.strip.depth.setData([], [])
            self.strip.set_events([], 0, 1)
            self.strip.set_spans([])
            self.events.clear()
            self._show_notes()
            self._build_lanes()
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
        self._show_notes()
        self.strip.set_spans(src.armed_spans)
        self.strip.getPlotItem().getViewBox().setLimits(xMax=max(1.0, src.duration) + 1.0)
        if not self.strip.zoomed:
            self.show_all()
        self.events.clear()
        self.events.addItem(f"Events ({len(evs)})" if evs else "No events", None)
        for e in evs:
            self.events.addItem(event_text(e), e.t)
        self._build_lanes()
        self._show_position(m.position)

    # ----- ROV lanes

    def _rov(self):
        src = self.model.source
        fn = getattr(src, "rov", None) if src is not None else None
        try:
            return fn() if callable(fn) else None
        except Exception:                                # noqa: BLE001 — a lane, never fatal
            return None

    def lanes_menu(self) -> QMenu:
        """The ROV variables there are, checkable, by group (applied at once)."""
        menu = QMenu(self)
        menu.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        rov = self._rov()
        avail = rov.variables() if rov else []
        if not avail:
            a = menu.addAction("No ROV telemetry in this source")
            a.setEnabled(False)
        groups: dict[str, QMenu] = {}
        for key, label, unit, group in avail:
            sub = groups.get(group)
            if sub is None:
                sub = groups[group] = menu.addMenu(group)
            a = QAction(label if key != _MODE_KEY else "flight mode / gain", sub)
            a.setCheckable(True)
            a.setChecked(key in self.rov_keys)
            a.toggled.connect(lambda on, k=key: self.set_rov_keys(
                [x for x in self.rov_keys if x != k] + ([k] if on else [])))
            sub.addAction(a)
        menu.addSeparator()
        clear = menu.addAction("Hide all ROV lanes")
        clear.setEnabled(bool(self.rov_keys))
        clear.triggered.connect(lambda: self.set_rov_keys([]))
        return menu

    def set_rov_keys(self, keys) -> None:
        """Show these ROV variables (``rov:<name>``, ``rov:mode`` the band) as lanes."""
        keys = list(dict.fromkeys(keys))
        if keys == self.rov_keys:
            return
        self.rov_keys = keys
        _save_lanes(keys)
        self._build_lanes()
        self._show_position(self.model.position)

    def _build_lanes(self) -> None:
        rov = self._rov() if self.rov_keys else None
        avail = {k: (lab, u) for k, lab, u, _g in (rov.variables() if rov else [])}
        by_unit: dict[str, list] = {}
        for k in self.rov_keys:
            if k in avail:
                by_unit.setdefault(avail[k][1], []).append(k)
        for u in list(self.lanes):
            if u not in by_unit:
                lane = self.lanes.pop(u)
                self._lane_box.removeWidget(lane)
                lane.deleteLater()
        order = ["mode"] + [u for u in by_unit if u != "mode"]
        for u in order:
            if u not in by_unit:
                continue
            lane = self.lanes.get(u)
            if lane is None:
                lane = self.lanes[u] = _Lane(self, u)
            if u == "mode":
                lane.set_modes(rov.modes, rov.series("rov:gain"))
            else:
                lane.set_curves([(k, avail[k][0], *rov.series(k)) for k in by_unit[u]])
        self.lanes = {u: self.lanes[u] for u in order if u in self.lanes}   # top to bottom
        for i, lane in enumerate(self.lanes.values()):
            self._lane_box.insertWidget(i, lane)
        self.rov_btn.setText(f"ROV ({len(self.lanes)}) ▾" if self.lanes else "ROV ▾")

    def _show_notes(self) -> None:
        nm = self.model.notes
        if nm is None or nm.recording is None:
            self.strip.set_notes([], None, False)
            self.notes_btn.setText("Notes")
            self.notes_btn.setEnabled(nm is not None and nm.recording is not None)
            self.notes_btn.setStyleSheet("")
            return
        st = nm.state
        on = self.model.source is not None
        self.strip.set_notes(st.pinned, nm.offset if on else None, st.bad and on, st.reason)
        self.notes_btn.setEnabled(True)
        self.notes_btn.setText(f"Notes ({st.count})" + (" BAD" if st.bad else ""))
        self.notes_btn.setStyleSheet("QPushButton { color: #b00; font-weight: bold; }"
                                     if st.bad else "")

    def show_all(self) -> None:
        """The whole recording (stream so far) on the strip."""
        src = self.model.source
        dur = max(1.0, src.duration) if src is not None else 1.0
        self.strip.getPlotItem().setXRange(0, dur, padding=0.01)

    def _show_position(self, t: float) -> None:
        if not math.isfinite(t):
            return
        for lane in self.lanes.values():
            lane.head.setValue(t)
        if self._dragging:
            return
        self.strip.head.blockSignals(True)
        self.strip.head.setValue(t)
        self.strip.head.blockSignals(False)

    def _show_state(self, state: str) -> None:
        self.controls.setVisible(self.model.active)

    def _show_view_span(self, rng) -> None:
        on = rng is not None and self.model.source is not None
        if on:
            self.strip.view_span.setRegion(rng)
            self.strip.view_edge.setValue(rng[1])
        self.strip.view_span.setVisible(on)
        self.strip.view_edge.setVisible(on)

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
