"""The vehicle's telemetry as variables: what the pilot asked for, what the
vehicle did (``RovTrack``, ``RovData``).

The laptop's ``.jsonl`` (server transport) carries the vehicle's MAVLink as
``{"k":"mav","t":...,"type":...,"src":[1,1],"d":{...}}``. The Analysis window
lines the whisker signals up with these (only the autopilot's, ``src [1,1]``):

- pilot commands, RC_CHANNELS (~19 Hz): ch5 forward, ch6 lateral, ch3 vertical
  (throttle), ch4 yaw, ch1 pitch, ch2 roll, as (raw - 1500) / 400: -1 .. +1,
  full stick at 100 % gain (the pilot gain scales the stick, so at 50 % they run
  about +-0.5). The camera / lights channels (7-11) are left out;
- thruster outputs, SERVO_OUTPUT_RAW servo1..8 (~19 Hz), the same scaling
  (1100 .. 1900 us is -1 .. +1); labelled with their role (horizontal /
  vertical) when a PARAM_VALUE FRAME_CONFIG names a vectored frame;
- battery current, BATTERY_STATUS ``current_battery`` (cA, ~3 Hz; SYS_STATUS's
  where there is none), in A: the thrusters' load;
- climb rate, VFR_HUD ``climb`` (m/s, ~16 Hz), and heading, VFR_HUD
  ``heading`` (deg), **unwrapped** (a turn through north keeps going past 360
  or below 0, so the curve, its decimation and its spectrum have no jumps);
- vibration, VIBRATION x / y / z (m/s/s, ~3 Hz);
- the pilot gain, NAMED_VALUE_FLOAT "PilotGain" (0.25, 0.5, ...): a step;
- derived, the thrust per axis (``rov:th_surge``, ``th_sway``, ``th_heave``,
  ``th_yaw``): the eight thruster outputs projected through ArduSub's
  vectored-6DOF mix with this vehicle's motor directions (``daq/vehicle.py``), in
  the pilot's units, so demand and achieved thrust compare axis by axis.
  ``axis_variables`` (the pilot's four axes and these) are the same keys in every
  recording with a vehicle (the Events windows list them before any is open).

The flight mode and the armed state are the event tracker's
(``replay/events.py`` parses HEARTBEAT once for both, and keeps the mode
changes); with the gain they make the timeline's band row (``RovData.modes``,
``armed``).

``RovTrack`` is fed in stream order: ``feed_line`` pulls the fields from a
line's bytes (the index pass, ``replay/index.py``, which does not
``json.loads`` the ~75k such lines of a 13 min dive), ``feed`` from a parsed
event (live, ``daq/server_reader.py`` via the tracker). Both convert the same
way (``_value``). Times are the Pi's receive time (as the depth's); the
messages are unevenly spaced at about 19 Hz. Its rows are ``daq/grow.Rows``:
a live stream is read again and again while it grows (``arrays``), and each
read converts only the rows since the last. ``RovData`` is the result in
session seconds, one ``(t, y)`` per variable key (``rov:fwd``, ``rov:thr1``,
...), cached in the index.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from wavi_py.daq.grow import Rows

NEUTRAL_US, SPAN_US = 1500.0, 400.0
AUTOPILOT_SRC = b'"src":[1,1]'

# (key, label, unit, group, field): per message type; the order is the Variables'
PILOT = (("rov:fwd", "forward", "ch5_raw"), ("rov:lat", "lateral", "ch6_raw"),
         ("rov:vert", "vertical", "ch3_raw"), ("rov:yaw", "yaw", "ch4_raw"),
         ("rov:pitch", "pitch", "ch1_raw"), ("rov:roll", "roll", "ch2_raw"))
FIELDS: dict[str, tuple] = {
    "RC_CHANNELS": tuple((k, f"pilot {lab}", "pilot", "Pilot", "chan" + f[2:])
                         for k, lab, f in PILOT),
    "SERVO_OUTPUT_RAW": tuple((f"rov:thr{i}", f"thruster {i}", "thrust", "Thrusters",
                               f"servo{i}_raw") for i in range(1, 9)),
    "BATTERY_STATUS": (("rov:current", "current", "A", "Load / motion", "current_battery"),),
    "SYS_STATUS": (("rov:current_sys", "current", "A", "Load / motion", "current_battery"),),
    "VFR_HUD": (("rov:climb", "climb", "m/s", "Load / motion", "climb"),
                ("rov:heading", "heading", "hdg", "Load / motion", "heading")),
    "VIBRATION": tuple((f"rov:vib_{a}", f"vibration {a}", "m/s²", "Load / motion",
                        f"vibration_{a}") for a in "xyz"),
    "NAMED_VALUE_FLOAT": (("rov:gain", "pilot gain", "gain", "Mode", "value"),),
}
TYPES = tuple(FIELDS)
TYPES_B = tuple(t.encode() for t in TYPES)
_FIELD_B = {t: tuple(f'"{f[4]}":'.encode() for f in fs) for t, fs in FIELDS.items()}
# derived: the thrust per axis from SERVO_OUTPUT_RAW (the pilot's axes, in its order)
AXIS_KEYS = (("surge", "rov:fwd", "rov:th_surge"), ("sway", "rov:lat", "rov:th_sway"),
             ("heave", "rov:vert", "rov:th_heave"), ("yaw", "rov:yaw", "rov:th_yaw"))
DERIVED = tuple((th, f"thrust {ax}", "thrust", "Thrust per axis") for ax, _p, th in AXIS_KEYS)
MODE_KEY = "rov:mode"              # the timeline's band row (mode spans, armed, gain)
# ArduSub FRAME_CONFIG -> (frame name, role of thrusters 1..8)
FRAMES = {1: ("Vectored", ("horiz",) * 4 + ("vert",) * 2),
          2: ("Vectored 6DOF", ("horiz",) * 4 + ("vert",) * 4),
          3: ("Vectored 6DOF 90", ("horiz",) * 4 + ("vert",) * 4)}


def _num(line: bytes, key: bytes, start: int):
    i = line.find(key, start)
    if i < 0:
        return None
    i += len(key)
    j = i
    n = len(line)
    while j < n and line[j] not in b",}":
        j += 1
    try:
        return float(line[i:j])
    except ValueError:
        return None


def _value(mtype: str, j: int, v) -> float:
    """Field ``j`` of ``mtype`` in its variable's units (NaN: not there / unknown)."""
    if v is None:
        return np.nan
    v = float(v)
    if mtype in ("RC_CHANNELS", "SERVO_OUTPUT_RAW"):
        return np.nan if v <= 0 else (v - NEUTRAL_US) / SPAN_US     # 0: channel unused
    if mtype in ("BATTERY_STATUS", "SYS_STATUS"):
        return np.nan if v < 0 else v / 100.0                       # cA; -1: unknown
    return v


class RovTrack:
    """See the module doc. Fed by one thread; ``lock`` for readers (live)."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        # type -> its rows (t, v0, v1, ...): ``daq/grow.Rows``, so a read of a live
        # stream converts only the rows since the last one
        self.rows: dict[str, Rows] = {t: Rows(1 + len(FIELDS[t])) for t in TYPES}
        self.frame: Optional[int] = None                         # FRAME_CONFIG
        self._hdg_last: Optional[float] = None
        self._hdg_off = 0.0

    def _add(self, mtype: str, t: float, vals: list) -> None:
        if mtype == "VFR_HUD" and np.isfinite(vals[1]):         # heading, unwrapped
            h = vals[1]
            if self._hdg_last is not None:
                d = h - self._hdg_last
                if d > 180.0:
                    self._hdg_off -= 360.0
                elif d < -180.0:
                    self._hdg_off += 360.0
            self._hdg_last = h
            vals[1] = h + self._hdg_off
        self.rows[mtype].append((t, *vals))

    def feed_line(self, t: float, mtype: bytes, line: bytes, start: int = 0) -> None:
        """A mav line of type ``mtype`` (bytes) at receive time ``t`` (the index)."""
        if AUTOPILOT_SRC not in line:
            return
        if mtype == b"PARAM_VALUE":
            if b'"FRAME_CONFIG"' in line:
                v = _num(line, b'"param_value":', start)
                if v is not None:
                    self.frame = int(v)
            return
        name = mtype.decode()
        if name == "NAMED_VALUE_FLOAT" and b'"name":"PilotGain"' not in line:
            return
        d = line.find(b'"d":', start)
        self._add(name, t, [_value(name, j, _num(line, k, d))
                            for j, k in enumerate(_FIELD_B[name])])

    def feed(self, ev: dict) -> None:
        """A parsed mav event (live)."""
        if tuple(ev.get("src") or ()) != (1, 1):
            return
        name, d = ev.get("type"), ev.get("d") or {}
        if name == "PARAM_VALUE":
            if str(d.get("param_id", "")).rstrip("\x00") == "FRAME_CONFIG":
                self.frame = int(d.get("param_value", 0))
            return
        if name not in FIELDS or (name == "NAMED_VALUE_FLOAT" and d.get("name") != "PilotGain"):
            return
        vals = [_value(name, j, d.get(f[4]) if isinstance(d.get(f[4]), (int, float)) else None)
                for j, f in enumerate(FIELDS[name])]
        with self.lock:
            self._add(name, float(ev.get("t", 0.0)), vals)

    def arrays(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """{type: (t (N,), y (N, m))} on the receive clock, for the types seen: views
        of rows that do not change afterwards (only the new rows are converted)."""
        out = {}
        with self.lock:
            for name, rows in self.rows.items():
                if len(rows):
                    a = rows.array()
                    out[name] = (a[:, 0], a[:, 1:])
        return out


@dataclass
class RovData:
    """The telemetry of a source in session seconds (see the module doc):
    ``tracks`` {type: (t, y (N, m))}, ``modes`` [(t0, t1, name)], ``armed``
    [[t0, t1]], ``frame`` (FRAME_CONFIG or None)."""
    tracks: dict = field(default_factory=dict)
    modes: list = field(default_factory=list)
    armed: list = field(default_factory=list)
    frame: Optional[int] = None

    def __bool__(self) -> bool:
        return bool(self.tracks)

    def series(self, key: str) -> tuple[np.ndarray, np.ndarray]:
        """(t, y) of variable ``key`` (empty if absent). ``rov:current`` falls back
        on SYS_STATUS's current when there is no BATTERY_STATUS; ``rov:th_<axis>``
        is derived from the thrusters (see the module doc)."""
        if key.startswith("rov:th_"):
            return self._axis_thrust(key)
        if key == "rov:current" and "BATTERY_STATUS" not in self.tracks:
            key = "rov:current_sys"
        for name, fs in FIELDS.items():
            for j, f in enumerate(fs):
                if f[0] == key:
                    got = self.tracks.get(name)
                    if got is None:
                        return np.empty(0), np.empty(0)
                    return got[0], got[1][:, j]
        return np.empty(0), np.empty(0)

    def _axis_thrust(self, key: str) -> tuple[np.ndarray, np.ndarray]:
        got = self.tracks.get("SERVO_OUTPUT_RAW")
        ax = next((a for a, _p, th in AXIS_KEYS if th == key), None)
        if got is None or ax is None:
            return np.empty(0), np.empty(0)
        from wavi_py.daq.vehicle import DOF, FULL_US, NEUTRAL_US, thrust
        t, y = got
        th = thrust(NEUTRAL_US + FULL_US * np.asarray(y, float))     # NaN servo: NaN
        return t, th[:, DOF.index(ax)]

    def extent(self) -> Optional[tuple[float, float]]:
        ts = [t for t, _y in self.tracks.values() if len(t)]
        return (min(float(t[0]) for t in ts), max(float(t[-1]) for t in ts)) if ts else None

    def variables(self) -> list[tuple[str, str, str, str]]:
        """(key, label, unit, group) of what is there, in the Variables' order;
        the mode band first when there are mode changes or a gain."""
        out = []
        if self.modes or "NAMED_VALUE_FLOAT" in self.tracks:
            out.append((MODE_KEY, "mode / gain / armed", "mode", "Mode"))
        roles = FRAMES.get(self.frame or -1, (None, ()))[1]
        for name, fs in FIELDS.items():
            if name == "NAMED_VALUE_FLOAT" or name not in self.tracks:
                continue
            if name == "SYS_STATUS" and "BATTERY_STATUS" in self.tracks:
                continue
            for j, (key, label, unit, group, _f) in enumerate(fs):
                if name == "SERVO_OUTPUT_RAW" and j < len(roles):
                    label = f"{label} ({roles[j]})"
                if key == "rov:current_sys":
                    key = "rov:current"
                y = self.tracks[name][1][:, j]
                if np.isfinite(y).any():
                    out.append((key, label, unit, group))
            if name == "SERVO_OUTPUT_RAW":
                out += list(DERIVED)
        return out

    def copy(self) -> "RovData":
        """A copy (a paused live copy: the tracker's lists move on)."""
        return RovData({n: (t.copy(), y.copy()) for n, (t, y) in self.tracks.items()},
                       [tuple(m) for m in self.modes], [list(a) for a in self.armed], self.frame)

    # ----- the index's cache (flat arrays in its npz)

    def to_npz(self) -> dict:
        out = {}
        for name, (t, y) in self.tracks.items():
            out[f"rov_t_{name}"], out[f"rov_y_{name}"] = t, y
        return out

    @classmethod
    def from_npz(cls, z, modes, armed, frame) -> "RovData":
        tracks = {n: (z[f"rov_t_{n}"], z[f"rov_y_{n}"]) for n in TYPES if f"rov_t_{n}" in z}
        return cls(tracks, [tuple(m) for m in modes], [list(a) for a in armed], frame)


def axis_variables() -> list[tuple[str, str, str, str]]:
    """(key, label, unit, group) of the pilot's four axes and the derived thrust
    per axis: the per-axis ROV variables (see the module doc)."""
    pilot = {f[0]: f for f in FIELDS["RC_CHANNELS"]}
    return ([pilot[p][:4] for _a, p, _t in AXIS_KEYS] + list(DERIVED))


def rov_data(track: RovTrack, t0: float, mode_changes: list, armed: list, end: float) -> RovData:
    """``RovData`` from a tracker's ``RovTrack`` and its mode changes [(t, name)]
    (receive clock), in session seconds (``t0`` the session zero); ``armed`` is
    already in session seconds; the last mode runs to ``end``."""
    tracks = {}
    for n, (t, y) in track.arrays().items():
        if len(t) > 1 and (np.diff(t) < 0).any():        # daq_server's MAVLink thread ran late:
            order = np.argsort(t, kind="stable")          # t steps back in file order
            t, y = t[order], y[order]
        tracks[n] = (t - t0, y)
    modes = []
    for i, (t, name) in enumerate(mode_changes):
        t1 = mode_changes[i + 1][0] - t0 if i + 1 < len(mode_changes) else end
        modes.append((t - t0, max(t - t0, t1), name))
    return RovData(tracks, modes, [list(a) for a in armed], track.frame)
