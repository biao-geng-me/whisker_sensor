"""Events and depth from a daq_server event stream, for the replay timeline.

``EventTracker`` is fed parsed events one at a time, in stream order, so the
same code builds a recording's event list (``replay/index.py``, at open) and
can follow a live stream. It collects:

- arm / disarm, from HEARTBEAT ``src [1,1]`` only (the autopilot; ``[1,194]``
  is another component whose ``base_mode`` 128 would read as armed);
- flight mode changes (ArduSub ``custom_mode``);
- every STATUSTEXT, levelled by its severity; the same text again within
  ``REPEAT_S`` of its last occurrence is counted into one entry;
- ESP32 / ADC fault marks and user marks;
- whisker scan sequence gaps (and a restart of the counter: a reboot);
- increments of the ESP32 / ADC error counters in the 1 Hz status, summed
  over ``REPEAT_S`` into one entry;
- the external pressure (SCALED_PRESSURE2, the Bar30) for depth.

Depth = (pressure - surface) / (rho g), with the surface pressure the median
over the first ``SURFACE_S`` of disarmed time (a vehicle sits at the surface
while disarmed). ArduSub's own depth (VFR_HUD.alt, GLOBAL_POSITION_INT) is not
used: it re-zeroes BARO*_GND_PRESS all the time while disarmed, so its zero
wanders. Fresh water by default; set ``WATER_DENSITY`` (or pass ``rho``) for
salt water (~1025 kg/m^3).

Times are seconds on the Pi's receive clock (the event ``t``), as given;
the index converts them to session seconds. The tracker may be fed by the DAQ
thread while the GUI reads it (``lock``).

``TimelineQueries`` is what the timeline widgets ask of a source, in session
seconds: ``events``, ``armed_spans``, ``depth()``, ``duration`` and the
queries built on them. ``RecordingIndex`` is one source (a whole recording);
``LiveTimeline`` is another (a live stream so far, from the reader's tracker).
"""

from __future__ import annotations

import threading
from dataclasses import asdict, dataclass, replace
from typing import Callable, Optional

import numpy as np

REPEAT_S = 10.0                 # repeats / counter increments within this go into one entry
SURFACE_S = 10.0                # disarmed time the surface pressure is the median of
WATER_DENSITY = 1000.0          # kg/m^3: fresh water; ~1025 for sea water
G = 9.80665

AUTOPILOT = (1, 1)              # the only HEARTBEAT sender that arms
MAV_MODE_FLAG_SAFETY_ARMED = 128

ARDUSUB_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
                 7: "CIRCLE", 9: "SURFACE", 16: "POSHOLD", 19: "MANUAL",
                 20: "MOTOR_DETECT", 21: "SURFTRAK"}

# MAV_SEVERITY 0 EMERGENCY .. 7 DEBUG -> the event level
_SEVERITY_LEVEL = {0: "error", 1: "error", 2: "error", 3: "error", 4: "warn",
                   5: "info", 6: "info", 7: "debug"}
SEVERITY_NAMES = ("EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG")

# ESP32 / ADC counters in status.esp32 whose increments are events
COUNTERS = ("crc_fail", "adc_crc", "adc_stale", "adc_recov", "adc_reinit", "adc_sync",
            "adc_empty")


def hpa_per_cm(rho: float = WATER_DENSITY) -> float:
    """Pressure of 1 cm of water in hPa (= mbar): 0.981 for fresh water."""
    return rho * G * 0.01 / 100.0


@dataclass
class Event:
    t: float                    # first occurrence (receive clock)
    kind: str                   # arm, disarm, mode, statustext, fault, mark, seq_gap, counter
    text: str
    level: str = "info"         # debug / info / warn / error / fault
    count: int = 1
    t_last: Optional[float] = None    # last occurrence of a grouped entry

    def to_dict(self) -> dict:
        return asdict(self)


class EventTracker:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.events: list[Event] = []
        self.armed_spans: list[list[float]] = []      # [t_arm, t_disarm or None]
        self._armed: Optional[bool] = None
        self._mode: Optional[int] = None
        self._last_text: dict[str, Event] = {}        # STATUSTEXT text -> its entry
        self._last_seq: Optional[int] = None
        self._counters: Optional[dict] = None
        self._counter_ev: Optional[Event] = None
        self._counter_sum: dict[str, int] = {}
        # pressure (hPa) for depth, and which samples were taken disarmed
        self.press_t: list[float] = []
        self.press_hpa: list[float] = []
        self._surface: list[float] = []
        self._surface_t0: Optional[float] = None
        self._surface_dt = 0.0
        self._last_press_t: Optional[float] = None

    # ----- feeding

    def feed(self, ev: dict) -> None:
        """One parsed event (any kind; the ones not used are ignored)."""
        with self.lock:
            self._feed(ev)

    def _feed(self, ev: dict) -> None:
        k = ev.get("k")
        t = float(ev.get("t", 0.0))
        if k == "mav":
            mtype = ev.get("type")
            d = ev.get("d") or {}
            if mtype == "HEARTBEAT":
                if tuple(ev.get("src") or ()) == AUTOPILOT:
                    self._heartbeat(t, d)
            elif mtype == "STATUSTEXT":
                self._statustext(t, d)
            elif mtype == "SCALED_PRESSURE2":
                self._pressure(t, d)
        elif k == "wsk":
            if "seq" in ev:
                self._wsk_seq(t, int(ev["seq"]))
        elif k == "status":
            self._status(t, ev)
        elif k == "mark":
            f = ev.get("fault")
            if isinstance(f, dict):
                lvl = "fault" if f.get("event") == "fault" else (
                    "warn" if f.get("event") == "warn" else "info")
                self.events.append(Event(t, "fault", str(ev.get("text", "")), lvl))
            else:
                self.events.append(Event(t, "mark", str(ev.get("text", "")), "info"))

    def wsk_seq(self, t: float, seq: int) -> None:
        """A whisker scan's sequence number (the index pulls it without json)."""
        if self._last_seq is not None and seq == self._last_seq + 1:
            self._last_seq = seq                 # the common case, without the lock
            return
        with self.lock:
            self._wsk_seq(t, seq)

    def _wsk_seq(self, t: float, seq: int) -> None:
        prev, self._last_seq = self._last_seq, seq
        if prev is None or seq == prev + 1:
            return
        if seq <= prev:
            self.events.append(Event(t, "seq_gap", f"scan counter restarted ({prev} -> {seq}):"
                                     " ESP32 reboot?", "warn"))
        else:
            n = seq - prev - 1
            self.events.append(Event(t, "seq_gap", f"seq gap: {n} scan{'s' * (n != 1)} lost",
                                     "warn"))

    def _heartbeat(self, t: float, d: dict) -> None:
        armed = bool(int(d.get("base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
        if armed != self._armed:
            if armed:
                self.armed_spans.append([t, None])
            elif self.armed_spans and self.armed_spans[-1][1] is None:
                self.armed_spans[-1][1] = t
            if self._armed is not None or armed:
                self.events.append(Event(t, "arm" if armed else "disarm",
                                         "ARMED" if armed else "disarmed",
                                         "warn" if armed else "info"))
            self._armed = armed
        mode = d.get("custom_mode")
        if mode is not None and mode != self._mode:
            name = ARDUSUB_MODES.get(int(mode), f"mode {mode}")
            self.events.append(Event(t, "mode", name if self._mode is None
                                     else f"{ARDUSUB_MODES.get(int(self._mode), self._mode)}"
                                     f" -> {name}"))
            self._mode = mode

    def _statustext(self, t: float, d: dict) -> None:
        text = str(d.get("text", "")).rstrip("\x00").strip()
        sev = int(d.get("severity", 6))
        prev = self._last_text.get(text)
        if prev is not None and t - (prev.t_last or prev.t) <= REPEAT_S:
            prev.count += 1
            prev.t_last = t
            return
        e = Event(t, "statustext", text, _SEVERITY_LEVEL.get(sev, "info"))
        self.events.append(e)
        self._last_text[text] = e

    def _pressure(self, t: float, d: dict) -> None:
        p = d.get("press_abs")
        if p is None:
            return
        p = float(p)
        self.press_t.append(t)
        self.press_hpa.append(p)
        # the first SURFACE_S of disarmed time (before the first heartbeat counts as disarmed)
        if not self._armed and self._surface_dt < SURFACE_S:
            if self._last_press_t is not None and self._surface:
                self._surface_dt += min(t - self._last_press_t, 1.0)
            self._surface.append(p)
        self._last_press_t = t

    def _status(self, t: float, st: dict) -> None:
        e = st.get("esp32") or {}
        now = {c: int(e[c]) for c in COUNTERS if isinstance(e.get(c), (int, float))}
        prev, self._counters = self._counters, now
        if prev is None:
            return
        inc = {c: v - prev[c] for c, v in now.items() if c in prev and v > prev[c]}
        if not inc:
            return
        ev = self._counter_ev
        if ev is None or t - ev.t > REPEAT_S:
            self._counter_sum = {}
            ev = self._counter_ev = Event(t, "counter", "", "warn", count=0)
            self.events.append(ev)
        for c, n in inc.items():
            self._counter_sum[c] = self._counter_sum.get(c, 0) + n
        ev.count += 1
        ev.t_last = t
        ev.text = "ESP32 counters: " + ", ".join(f"{c} +{n}" for c, n in self._counter_sum.items())

    # ----- results

    @property
    def armed(self) -> Optional[bool]:
        return self._armed

    def surface_hpa(self) -> Optional[float]:
        """The surface pressure: the median over the first disarmed seconds (the
        first 2 s of data if the vehicle never was disarmed)."""
        if len(self._surface) >= 5:
            return float(np.median(self._surface))
        if self.press_hpa:
            t = np.asarray(self.press_t)
            return float(np.median(np.asarray(self.press_hpa)[t <= t[0] + 2.0]))
        return None

    def depth_cm(self, rho: float = WATER_DENSITY) -> tuple[np.ndarray, np.ndarray]:
        """(t, depth in cm) from the external pressure; empty without it."""
        p0 = self.surface_hpa()
        if p0 is None:
            return np.empty(0), np.empty(0)
        return (np.asarray(self.press_t, dtype=np.float64),
                (np.asarray(self.press_hpa, dtype=np.float64) - p0) / hpa_per_cm(rho))

    def sorted_events(self) -> list[Event]:
        return sorted(self.events, key=lambda e: e.t)


class TimelineQueries:
    """Queries over ``events`` / ``armed_spans`` / ``depth()`` (session seconds)."""

    events: list
    armed_spans: list

    @property
    def duration(self) -> float:
        raise NotImplementedError

    def depth(self, rho: float = WATER_DENSITY) -> tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError

    def depth_at(self, t_sess: float, rho: float = WATER_DENSITY) -> Optional[float]:
        t, d = self.depth(rho)
        if not len(t):
            return None
        i = max(0, int(np.searchsorted(t, t_sess, side="right")) - 1)
        return float(d[i])

    def depth_crossing(self, t_from: float, depth_cm: float, forward: bool = True,
                       rho: float = WATER_DENSITY) -> Optional[float]:
        """The next (``forward``) or previous time the depth crosses ``depth_cm``
        (going down or up), from session time ``t_from``."""
        t, d = self.depth(rho)
        if len(t) < 2:
            return None
        s = np.sign(d - depth_cm)
        s[s == 0] = 1
        cross = np.nonzero(s[1:] != s[:-1])[0] + 1          # first sample past the crossing
        tc = t[cross]
        if forward:
            k = tc[tc > t_from + 1e-6]
            return float(k[0]) if k.size else None
        k = tc[tc < t_from - 1e-6]
        return float(k[-1]) if k.size else None

    def max_depth(self, rho: float = WATER_DENSITY) -> Optional[tuple[float, float]]:
        """(session time, depth cm) of the deepest point."""
        t, d = self.depth(rho)
        if not len(t):
            return None
        i = int(np.argmax(d))
        return float(t[i]), float(d[i])

    def armed_at(self, t_sess: float) -> bool:
        return any(a <= t_sess < b for a, b in self.armed_spans)

    def event_near(self, t_sess: float, forward: bool, skip_s: float = 0.5) -> Optional[Event]:
        """The next (previous) event more than ``skip_s`` after (before) ``t_sess``."""
        evs = self.events
        if forward:
            return next((e for e in evs if e.t > t_sess + skip_s), None)
        return next((e for e in reversed(evs) if e.t < t_sess - skip_s), None)


class LiveTimeline(TimelineQueries):
    """A live stream's events so far, in session seconds: the reader's tracker,
    its session zero (``t_ref()``, receive clock) and its newest time
    (``t_last()``)."""

    def __init__(self, tracker: EventTracker, t_ref: Callable[[], Optional[float]],
                 t_last: Callable[[], float], unix0: Callable[[], Optional[float]]):
        self._tr = tracker
        self._t_ref = t_ref
        self._t_last = t_last
        self._unix0 = unix0

    def _t0(self) -> float:
        t0 = self._t_ref()
        return float(t0) if t0 is not None else 0.0

    @property
    def unix0(self) -> float:
        u = self._unix0()
        return float(u) if u is not None else 0.0

    @property
    def duration(self) -> float:
        return max(0.0, float(self._t_last()) - self._t0())

    @property
    def events(self) -> list[Event]:
        t0 = self._t0()
        with self._tr.lock:
            evs = [replace(e, t=e.t - t0, t_last=None if e.t_last is None else e.t_last - t0)
                   for e in self._tr.events]
        return sorted(evs, key=lambda e: e.t)

    @property
    def armed_spans(self) -> list[list[float]]:
        t0, end = self._t0(), self.duration
        with self._tr.lock:
            spans = [list(s) for s in self._tr.armed_spans]
        return [[a - t0, (b - t0) if b is not None else end] for a, b in spans]

    def depth(self, rho: float = WATER_DENSITY) -> tuple[np.ndarray, np.ndarray]:
        with self._tr.lock:
            t, d = self._tr.depth_cm(rho)
        return t - self._t0(), d
