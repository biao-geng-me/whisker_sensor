"""Manoeuvres: a recording cut into stick holds and transitions, with context
and measurements, as ``<stem>.manoeuvres.json``.

Every whisker signal of a dive is caused by the ROV's motion and by its
thrusters' jets and wake, and the pilot flies in holds: a stick (or two) held
at a level for a while, then moved. So a recording is cut by the pilot's
demand, and each piece is measured the same way, to compare same-class events
across runs (are the whiskers still seated as they were?). No Qt here; numpy
and scipy only. (``replay/events.py`` is the system events: arm, modes,
STATUSTEXT.)

Inputs, all through the repo's loaders: ``RecordingIndex`` (session zero,
armed spans, the surface-referenced depth of ``EventTracker``), the whole
``RecordingStore`` (whisker mV, IMU) and one pass over the autopilot's
(``src [1,1]``) MAVLink lines for what the store does not keep: RC_CHANNELS,
SERVO_OUTPUT_RAW, HEARTBEAT mode, RANGEFINDER, NAMED_VALUE_FLOAT PilotGain, and
the scans' ADC flags. All of it is put on one ``FS_HZ`` grid of session seconds
(the replay's time axis: the scans and IMU at their fitted times, the other
MAVLink at receive time; ``Flight``).

Demand: ``u = (raw - 1500) / 400`` per axis (RC ch5 surge, ch6 sway, ch3 heave,
ch4 yaw), pilot gain included. Neutral is fixed at 1500: a stick held for most
of a run must not become its rest. The pilot's stick deadband and expo put RC at
exactly 1500 at rest, so ``|u| <= REST_TOL`` is rest (the tolerance is a legacy
fallback: before the deadband, on 2026-11-03, a stick rested at ~+10 %); the
median of the samples with ``|u| <= REST_OFFSET_MAX`` is reported per axis as
the rest offset.

Minor inputs: an axis under ``MINOR`` while another axis is at or above it (a
small yaw correction during a surge) is dropped from the *effective demand*
(``effective_demand``) that segments are cut on, so it neither starts a segment
nor makes a hold a combo; each hold and transition records what was dropped as
``minor: {axis: {"max": max |u|, "share": fraction of the span}}``, so filters
can leave corrected holds out. A small input with no larger axis is kept: a
segment of its own.

Segments (``segment``): armed time only (disarmed spans are ``disarmed``
segments). On the effective demand, a new segment starts where the set of
active axes changes or any axis moves more than ``STEP`` from the value at the
segment's start; a segment shorter than ``DEBOUNCE_S`` (a stick on its way
between levels) joins the next one, and neighbours with the same active axes
whose median levels are within ``STEP`` are one. Class: ``rest``,
``single:<axis><sign>`` (with its level) or ``combo``.

Events (``extract``): a ``hold`` per armed segment, and a ``transition`` at
each boundary between two: ``onset`` (rest -> active), ``release`` (active ->
rest: the vehicle coasts without thrust), ``step`` (a level change, no sign
change) or ``reversal`` (an axis changes sign), windowed ``PRE_S`` before to
``POST_S`` after. A transition's *response hold* is the segment after it.

Per event:

- context: mode (in ALT_HOLD the heave stick is a climb-rate demand, in
  MANUAL thrust), depth (surface-referenced) and a near-surface flag (None
  when the depth goes below -``DEPTH_NEG_CM``: a run armed under water from
  its start has no surface pressure, ``recording.depth_ok``), the
  rangefinder distance if any, mean roll / pitch and a level flag, the quiet
  time before (the rest before an onset; under ``QUIET_S`` the baseline is not
  clean), the pilot gain, and the thrust per axis: SERVO1..8 projected through
  ArduSub's BlueROV2 Heavy mix (``MIX``, ``MOTOR_DIRECTION``) onto roll, pitch,
  yaw, heave, surge, sway (``mix_check`` checks it on single-axis holds);
- motion achieved (yaw / roll / pitch rate from the gyro, depth rate from the
  depth, body acceleration without gravity): the value before, the peak, the
  steady value over the last third of a response hold of at least
  ``STEADY_MIN_S``, the rise time to 63 % of it, and after a release the decay
  time constant;
- per dome position, drag and lift (polarity applied): baseline (median of the
  last ``BASELINE_S`` of the quiet rest before the event), then relative to it
  and drift-corrected by the reference sensor (the whisker-less gauges of the
  dome map's ``reference``: their mean, low-passed, minus its own baseline;
  not the whiskered gauges' common mode, which in flight is real flow): peak,
  steady, decay time constant, and the ``BAND_HZ`` band RMS; ``alive`` from the
  band RMS of drag + lift (a lost whisker sits at the ~10-15 uV floor, one in
  flight at 100-600 uV);
- quality: ADC-flagged scans and railed gauges in the window, baseline_clean.

The positions and the reference come from the recording's own dome map
(``replay/dome_maps.dome_map_for``: embedded, else the history entry at its
start); the sidecar's ``recording.dome_map`` / ``whisker_set`` name it.

Sidecar: ``write_manoeuvres_for`` (``tools/make_manoeuvres.py`` for old
recordings and for a folder that is not ours: ``--out``), ``is_current``,
``write_manoeuvres_in_background`` (as the card, ``replay/card.py``);
``events_csv`` flattens sidecars into one row per event x position.
"""

from __future__ import annotations

import csv
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Union

import numpy as np
from scipy.optimize import curve_fit
from scipy.signal import butter, savgol_filter, sosfiltfilt

from wavi_py.daq.vehicle import (AXES, DOF, FULL_US, MIX, MOTOR_DIRECTION, NEUTRAL_US,
                                 RC_CHANNEL)
from wavi_py.replay import recname
from wavi_py.replay.events import ARDUSUB_MODES, AUTOPILOT, MAV_MODE_FLAG_SAFETY_ARMED
from wavi_py.replay.index import K, RecordingIndex, _int_after, cache_paths
from wavi_py.replay.store import RecordingStore
from wavi_py.replay.dome_maps import dome_map_for
from whisker_link.dome_map import DEFAULT_MAP, DomeMap
from whisker_link.esp32_frame import MV_PER_CODE

VERSION = 2                     # 2: the recording's own dome map (replay/dome_maps.py)
SUFFIX = ".manoeuvres.json"
MADE_BY = f"wavi_py manoeuvres v{VERSION}"

FS_HZ = 50.0                    # the analysis grid
REST_TOL = 0.02                 # |u| at or below: rest (RC rests at exactly 1500 behind the
                                # pilot's stick deadband; the tolerance is a legacy fallback)
STEP = 0.08                     # a held level moving more than this: a new segment
MINOR = 0.15                    # |u| under this next to a larger axis: a minor input (module doc)
REST_OFFSET_MAX = MINOR         # the rest offset is the median of |u| <= this
DEBOUNCE_S = 0.15               # shorter segments join the next one
PRE_S = 2.0                     # transition window: PRE_S before .. POST_S after
POST_S = 5.0
QUIET_S = 2.0                   # rest before an event at least this long: baseline_clean
BASELINE_S = 1.0                # baseline: median of the last BASELINE_S of that rest
PRE_MOTION_S = 0.5              # motion "before": median of this long before the event
PEAK_S = 2.0                    # transient peak: within PEAK_S after a transition
STEADY_MIN_S = 3.0              # response holds this long get a steady value (last third)
DECAY_MIN_S = 0.5               # a release's decay is fitted on at least this much rest
BAND_HZ = (1.5, 10.0)
ALIVE_UV = 40.0                 # band RMS of drag + lift: above alive; below, in flight, lost
REF_LP_HZ = 0.5                 # the reference's low-pass (it tracks the slow drift)
NEAR_SURFACE_CM = 20.0
DEPTH_NEG_CM = 10.0             # depth below -this anywhere: the surface reference is wrong
LEVEL_DEG = 10.0                # |roll| and |pitch| under this: level
RAIL_MV = 8388607 * MV_PER_CODE * 0.999

# the stick axes, ArduSub's BlueROV2 Heavy mix and this vehicle's motor directions:
# daq/vehicle.py (AXES, RC_CHANNEL, NEUTRAL_US, FULL_US, DOF, MIX, MOTOR_DIRECTION)
MOTION = ("yaw_rate_dps", "roll_rate_dps", "pitch_rate_dps", "depth_rate_cms",
          "acc_x", "acc_y", "acc_z")
# a response smaller than this (or than 3x the scatter before) gets no rise time
MOTION_FLOOR = {"yaw_rate_dps": 2.0, "roll_rate_dps": 2.0, "pitch_rate_dps": 2.0,
                "depth_rate_cms": 1.0, "acc_x": 0.05, "acc_y": 0.05, "acc_z": 0.05}
G = 9.80665


def constants() -> dict:
    """The module's tunables, as written into every sidecar."""
    return {"fs_hz": FS_HZ, "axes": list(AXES), "rc_channel": RC_CHANNEL,
            "neutral_us": NEUTRAL_US, "full_us": FULL_US, "rest_tol": REST_TOL,
            "step": STEP, "minor": MINOR,
            "debounce_s": DEBOUNCE_S, "pre_s": PRE_S, "post_s": POST_S, "quiet_s": QUIET_S,
            "baseline_s": BASELINE_S, "pre_motion_s": PRE_MOTION_S, "peak_s": PEAK_S,
            "steady_min_s": STEADY_MIN_S, "decay_min_s": DECAY_MIN_S, "band_hz": list(BAND_HZ),
            "alive_uv": ALIVE_UV, "ref_lp_hz": REF_LP_HZ, "near_surface_cm": NEAR_SURFACE_CM, "depth_neg_cm": DEPTH_NEG_CM,
            "level_deg": LEVEL_DEG, "motion_floor": MOTION_FLOOR, "dof": list(DOF), "mix": MIX.tolist(),
            "motor_direction": list(MOTOR_DIRECTION)}


# ---------------------------------------------------------------------------
# the flight on one grid
# ---------------------------------------------------------------------------
@dataclass
class Flight:
    """A recording on the ``FS_HZ`` grid of session seconds. NaN where a source is
    missing (no servo, no depth, ...). ``wsk`` is mV at the ADC, the store's column
    order and ``names``."""
    t: np.ndarray                               # (n,) bin centres
    armed: np.ndarray                           # (n,) bool
    u: np.ndarray                               # (n, 4) demand, AXES order
    wsk: np.ndarray                             # (n, nch) mV
    names: list
    mode: np.ndarray = None                     # (n,) ArduSub custom_mode, -1 unknown
    servo: np.ndarray = None                    # (n, 8) us
    gyro: np.ndarray = None                     # (n, 3) rad/s body
    acc: np.ndarray = None                      # (n, 3) m/s^2 body, gravity removed
    roll: np.ndarray = None                     # (n,) rad
    pitch: np.ndarray = None
    depth_cm: np.ndarray = None                 # (n,) surface-referenced
    range_m: np.ndarray = None                  # (n,) rangefinder, NaN without
    gain: np.ndarray = None                     # (n,) PilotGain
    railed: np.ndarray = None                   # (n, nch) bool
    flag_t: np.ndarray = None                   # session seconds of ADC-flagged scans
    stem: str = ""
    meta: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        n = len(self.t)
        nan = lambda *s: np.full((n,) + s, np.nan)   # noqa: E731
        self.mode = np.full(n, -1) if self.mode is None else self.mode
        for name, shape in (("servo", (8,)), ("gyro", (3,)), ("acc", (3,)), ("roll", ()),
                            ("pitch", ()), ("depth_cm", ()), ("range_m", ()), ("gain", ())):
            if getattr(self, name) is None:
                setattr(self, name, nan(*shape))
        if self.railed is None:
            self.railed = np.abs(self.wsk) >= RAIL_MV
        if self.flag_t is None:
            self.flag_t = np.empty(0)

    @property
    def dt(self) -> float:
        return 1.0 / FS_HZ

    def i_at(self, t: float) -> int:
        return int(np.clip(np.searchsorted(self.t, t), 0, len(self.t)))

    def thrust(self) -> np.ndarray:
        """(n, 6) thrust per DOF: the servos (-1..1) projected through the mix."""
        th = (self.servo - NEUTRAL_US) / FULL_US
        a = MIX * np.asarray(MOTOR_DIRECTION, np.float64)[:, None]
        return th @ np.linalg.pinv(a).T

    def depth_rate(self) -> np.ndarray:
        """cm/s, + = deeper: a 0.5 s Savitzky-Golay derivative of the depth."""
        d = self.depth_cm
        if not np.isfinite(d).any():
            return np.full(len(d), np.nan)
        w = int(0.5 * FS_HZ) | 1
        ok = np.isfinite(d)
        dd = np.interp(self.t, self.t[ok], d[ok])
        if len(dd) <= w:
            return np.gradient(dd) * FS_HZ
        return savgol_filter(dd, w, 2, deriv=1) * FS_HZ

    def motion(self) -> np.ndarray:
        """(n, len(MOTION)) achieved motion."""
        r2d = 180.0 / np.pi
        return np.column_stack([self.gyro[:, 2] * r2d, self.gyro[:, 0] * r2d,
                                self.gyro[:, 1] * r2d, self.depth_rate(), self.acc])


def _bins(t: np.ndarray, n: int) -> np.ndarray:
    return np.clip((t * FS_HZ).astype(np.int64), 0, n - 1)


def _bin_mean(t: np.ndarray, y: np.ndarray, n: int) -> np.ndarray:
    """Mean of the samples in each grid bin; empty bins linearly filled."""
    y = np.asarray(y, np.float64).reshape(len(t), -1)
    out = np.full((n, y.shape[1]), np.nan)
    if not len(t):
        return out
    ok = (t >= 0) & (t < n / FS_HZ)
    b = _bins(t[ok], n)
    cnt = np.bincount(b, minlength=n)
    has = cnt > 0
    for j in range(y.shape[1]):
        out[has, j] = np.bincount(b, y[ok, j], minlength=n)[has] / cnt[has]
    if has.any() and not has.all():
        tc = np.arange(n)
        for j in range(y.shape[1]):
            out[:, j] = np.interp(tc, tc[has], out[has, j])
    return out


def _hold(t_src: np.ndarray, y: np.ndarray, grid: np.ndarray, fill=np.nan) -> np.ndarray:
    """Zero-order hold of samples (t_src sorted) onto the grid; ``fill`` before the first."""
    y = np.asarray(y, np.float64)
    out = np.full((len(grid),) + y.shape[1:], fill, dtype=np.float64)
    if not len(t_src):
        return out
    i = np.searchsorted(t_src, grid, side="right") - 1
    ok = i >= 0
    out[ok] = y[i[ok]]
    return out


def _read_vehicle(index: RecordingIndex) -> dict:
    """The autopilot's MAVLink the store does not keep, and the scans' ADC flags:
    {name: (session seconds, values)}, receive time."""
    want = {b"RC_CHANNELS", b"SERVO_OUTPUT_RAW", b"HEARTBEAT", b"RANGEFINDER",
            b"NAMED_VALUE_FLOAT"}
    rc, sv, hb, rf, gain, flg, wrx = [], [], [], [], [], [], []
    lines = np.nonzero((index.kind == K["mav"]) | (index.kind == K["wsk"]))[0]
    lines = lines[lines > index.hello_line]
    k_wsk = K["wsk"]
    t0 = index.t0
    with open(index.path, "rb") as f:
        for c0 in range(0, len(lines), 50_000):
            chunk = lines[c0:c0 + 50_000]
            base = int(index.off[chunk[0]])
            f.seek(base)
            data = f.read(int(index.off[chunk[-1] + 1]) - base)
            for ln in chunk.tolist():
                a = int(index.off[ln]) - base
                e = data.find(b"\n", a)
                line = data[a:e if e >= 0 else len(data)]
                if index.kind[ln] == k_wsk:
                    t = index.t[ln] - t0
                    wrx.append(t)
                    fl = _int_after(line, b'"flg":')
                    if fl:
                        flg.append(t)
                    continue
                ty = line.find(b'"type":"')
                if ty < 0:
                    continue
                mtype = line[ty + 8:line.find(b'"', ty + 8)]
                if mtype not in want:
                    continue
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue
                if tuple(ev.get("src") or ()) != AUTOPILOT:
                    continue
                t, d = float(ev["t"]) - t0, ev.get("d") or {}
                if mtype == b"RC_CHANNELS":
                    rc.append([t] + [d.get(f"chan{RC_CHANNEL[a]}_raw", 0) for a in AXES])
                elif mtype == b"SERVO_OUTPUT_RAW":
                    sv.append([t] + [d.get(f"servo{i}_raw", 0) for i in range(1, 9)])
                elif mtype == b"HEARTBEAT":
                    hb.append((t, int(d.get("base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED,
                               int(d.get("custom_mode", -1))))
                elif mtype == b"RANGEFINDER":
                    rf.append((t, float(d.get("distance", 0.0))))
                elif d.get("name") == "PilotGain":
                    gain.append((t, float(d.get("value", np.nan))))

    def arr(rows, w):
        a = np.asarray(rows, np.float64).reshape(-1, w)
        return a[np.argsort(a[:, 0], kind="stable")] if len(a) else a

    return {"rc": arr(rc, 5), "servo": arr(sv, 9), "hb": arr(hb, 3), "rf": arr(rf, 2),
            "gain": arr(gain, 2), "flag_t": np.sort(np.asarray(flg, np.float64)),
            "wsk_rx": np.asarray(wrx, np.float64)}


def load_store(index: RecordingIndex, cache: bool = True) -> RecordingStore:
    """The whole-recording store: from a replay's cache if one is there, else built
    (and cached only with ``cache``)."""
    if cache:
        for c in cache_paths(index.path):
            st = RecordingStore._load_cache(
                c.with_name(c.name.replace(".idx.npz", ".store.npz")), index)
            if st is not None:
                return st
    return RecordingStore.open(index, use_cache=True) if cache else RecordingStore.build(index)


def load_flight(recording: Union[str, Path], cache: bool = True) -> Flight:
    """A recording as a ``Flight``. ``cache``: read and write the index / store caches
    next to it (as replay does); off for a folder that is not ours."""
    rec = Path(recording)
    index = RecordingIndex.open(rec, use_cache=cache)
    store = load_store(index, cache)
    veh = _read_vehicle(index)
    dur = max(index.duration, store.duration)
    n = max(1, int(np.ceil(dur * FS_HZ)))
    t = (np.arange(n) + 0.5) / FS_HZ
    armed = np.zeros(n, bool)
    for a, b in index.armed_spans:
        armed[(t >= a) & (t < b)] = True
    rc = veh["rc"]
    u = (_hold(rc[:, 0], rc[:, 1:], t, fill=NEUTRAL_US) - NEUTRAL_US) / FULL_US if len(rc) \
        else np.zeros((n, 4))
    sv = veh["servo"]
    servo = _hold(sv[:, 0], sv[:, 1:], t) if len(sv) else None
    hb = veh["hb"]
    mode = _hold(hb[:, 0], hb[:, 2], t, fill=-1).astype(int) if len(hb) else None
    wsk = _bin_mean(store.wsk_t, store.despiked() if len(store.spk_row) else store.wsk, n)
    railed = _bin_mean(store.wsk_t, np.abs(store.wsk) >= RAIL_MV, n) > 0 if len(store.wsk_t) \
        else None
    gyro = acc = roll = pitch = None
    if len(store.imu_t):
        im = _bin_mean(store.imu_t, store.imu, n)
        roll, pitch = im[:, 6], im[:, 7]
        gyro = im[:, 3:6] * 1e-3
        f = im[:, 0:3] * G * 1e-3                    # specific force, body FRD
        gb = G * np.column_stack([-np.sin(pitch), np.sin(roll) * np.cos(pitch),
                                  np.cos(roll) * np.cos(pitch)])
        acc = f + gb
    dt_, dcm = index.depth()
    depth = np.interp(t, dt_, dcm) if len(dt_) else None
    rf = veh["rf"]
    rng = None
    if len(rf) and (rf[:, 1] > 0).any():
        rng = _hold(rf[:, 0], np.where(rf[:, 1] > 0, rf[:, 1], np.nan), t)
    gn = veh["gain"]
    gain = _hold(gn[:, 0], gn[:, 1], t) if len(gn) else None
    meta = {"duration_s": round(float(dur), 3), "fs_scan_hz": round(float(index.fs), 4),
            "armed_spans": [[round(a, 3), round(b, 3)] for a, b in index.armed_spans],
            "n_rc": int(len(rc)), "n_servo": int(len(sv))}
    wrx = veh["wsk_rx"]
    if len(wrx) == len(store.wsk_t) and len(wrx):
        d = (store.wsk_t - wrx) * 1e3                # fitted scan time - receive time
        meta["clock"] = {"wsk_fit_minus_rx_ms": [round(float(np.percentile(d, q)), 2)
                                                 for q in (5, 50, 95)]}
    return Flight(t=t, armed=armed, u=u, wsk=wsk, names=list(store.wsk_names), mode=mode,
                  servo=servo, gyro=gyro, acc=acc, roll=roll, pitch=pitch, depth_cm=depth,
                  range_m=rng, gain=gain, railed=railed, flag_t=veh["flag_t"],
                  stem=recname.stem_of(rec.name), meta=meta)


# ---------------------------------------------------------------------------
# segments
# ---------------------------------------------------------------------------
@dataclass
class Segment:
    i0: int                     # grid rows [i0, i1)
    i1: int
    t0: float
    t1: float
    level: np.ndarray           # (4,) median effective demand, AXES order
    armed: bool = True
    minor: dict = field(default_factory=dict)   # minor inputs dropped (``_minor``)

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    @property
    def signs(self) -> np.ndarray:
        return np.where(np.abs(self.level) > REST_TOL, np.sign(self.level), 0).astype(int)

    @property
    def cls(self) -> str:
        if not self.armed:
            return "disarmed"
        act = np.nonzero(self.signs)[0]
        if not len(act):
            return "rest"
        if len(act) == 1:
            k = int(act[0])
            return f"single:{AXES[k]}{'+' if self.level[k] > 0 else '-'}"
        return "combo"

    @property
    def axes(self) -> str:
        """``surge+,yaw-``: the active axes with their signs."""
        s = self.signs
        return ",".join(f"{AXES[k]}{'+' if s[k] > 0 else '-'}" for k in range(4) if s[k])


def _runs(mask: np.ndarray) -> list[tuple[int, int, bool]]:
    """[(i0, i1, value)] of the runs of a bool array."""
    if not len(mask):
        return []
    edges = np.nonzero(np.diff(mask.astype(np.int8)))[0] + 1
    starts = np.r_[0, edges]
    ends = np.r_[edges, len(mask)]
    return [(int(a), int(b), bool(mask[a])) for a, b in zip(starts, ends)]


def effective_demand(u: np.ndarray) -> np.ndarray:
    """The demand the segments are cut on: per sample, an axis under ``MINOR``
    while another axis is at or above it is zeroed (a correction held alongside
    the main input); small inputs on their own are kept."""
    a = np.abs(u)
    major = (a >= MINOR).any(axis=1, keepdims=True)
    return np.where(major & (a < MINOR), 0.0, u)


def _active(v: np.ndarray) -> np.ndarray:
    return np.abs(v) > REST_TOL


def _split(u: np.ndarray, i0: int, i1: int) -> list[int]:
    """Segment starts in [i0, i1): a new one where the set of active axes changes
    or any axis moves more than STEP from the value at the segment's start."""
    starts = [i0]
    ref = u[i0]
    for i in range(i0 + 1, i1):
        if (np.any(_active(u[i]) != _active(ref)) or np.any(np.abs(u[i] - ref) > STEP)):
            starts.append(i)
            ref = u[i]
    return starts


def _minor(u: np.ndarray, ue: np.ndarray) -> dict:
    """{axis: {"max": max |u|, "share": fraction of samples active}} of the inputs
    the effective demand dropped in a span."""
    out = {}
    for k, ax in enumerate(AXES):
        dropped = (ue[:, k] == 0) & _active(u[:, k])
        if dropped.any():
            out[ax] = {"max": _f(np.max(np.abs(u[dropped, k])), 3),
                       "share": _f(dropped.mean(), 3)}
    return out


def segment(fl: Flight) -> list[Segment]:
    """The flight cut into segments (module doc), in time order, covering it."""
    out: list[Segment] = []
    dt = fl.dt
    min_n = max(1, int(round(DEBOUNCE_S * FS_HZ)))
    ue = effective_demand(fl.u)

    def seg(s0: int, s1: int) -> Segment:
        return Segment(s0, s1, s0 * dt, s1 * dt, np.median(ue[s0:s1], axis=0),
                       minor=_minor(fl.u[s0:s1], ue[s0:s1]))

    for a, b, armed in _runs(fl.armed):
        if not armed:
            out.append(Segment(a, b, a * dt, b * dt, np.zeros(4), armed=False))
            continue
        starts = _split(ue, a, b)
        bounds = list(zip(starts, starts[1:] + [b]))
        # de-bounce: a short segment joins the next (the last one joins the one before)
        changed = True
        while changed and len(bounds) > 1:
            changed = False
            for j, (s, e) in enumerate(bounds):
                if e - s < min_n:
                    if j + 1 < len(bounds):
                        bounds[j + 1] = (s, bounds[j + 1][1])
                    else:
                        bounds[j - 1] = (bounds[j - 1][0], e)
                    del bounds[j]
                    changed = True
                    break
        # neighbours with the same active axes and levels within STEP: one
        merged: list[Segment] = []
        for sg in (seg(s, e) for s, e in bounds):
            if merged:
                p = merged[-1]
                if (np.array_equal(p.signs, sg.signs)
                        and np.all(np.abs(p.level - sg.level) <= STEP)):
                    merged[-1] = seg(p.i0, sg.i1)
                    continue
            merged.append(sg)
        out += merged
    return out


def transition_type(a: Segment, b: Segment) -> str:
    sa, sb = a.signs, b.signs
    if not sa.any():
        return "onset"
    if not sb.any():
        return "release"
    if np.any(sa * sb < 0):
        return "reversal"
    return "step"


# ---------------------------------------------------------------------------
# measurements
# ---------------------------------------------------------------------------
def _f(x, nd: int = 4):
    """A JSON number (None for NaN / None), rounded to ``nd`` significant digits."""
    if x is None:
        return None
    x = float(x)
    if not np.isfinite(x):
        return None
    return float(f"{x:.{nd}g}")


def _nanmedian(x) -> float:
    x = np.asarray(x, np.float64)
    x = x[np.isfinite(x)]
    return float(np.median(x)) if len(x) else float("nan")


def _signed_peak(x: np.ndarray) -> float:
    x = np.asarray(x, np.float64)
    ok = np.isfinite(x)
    if not ok.any():
        return float("nan")
    i = int(np.argmax(np.where(ok, np.abs(x), -1)))
    return float(x[i])


def rise_time(x: np.ndarray, pre: float, steady: float, dt: float) -> Optional[float]:
    """Seconds from x[0] until x - pre first reaches 63 % of steady - pre (None if
    never, or no steady)."""
    d = steady - pre
    if not np.isfinite(d) or d == 0:
        return None
    y = (np.asarray(x, np.float64) - pre) * np.sign(d)
    k = np.nonzero(y >= 0.63 * abs(d))[0]
    return float(k[0] * dt) if len(k) else None


def fit_decay(x: np.ndarray, dt: float) -> Optional[float]:
    """Time constant (s) of x(t) = a + b exp(-t / tau), fitted from x[0]; None
    without a clean decay (too short, no amplitude, a poor fit)."""
    x = np.asarray(x, np.float64)
    if len(x) < max(8, int(DECAY_MIN_S / dt)) or not np.isfinite(x).all():
        return None
    t = np.arange(len(x)) * dt
    a0, b0 = x[-1], x[0] - x[-1]
    if abs(b0) < 1e-9:
        return None
    k = np.nonzero(np.abs(x - a0) <= abs(b0) / np.e)[0]
    tau0 = max(float(t[k[0]]) if len(k) else t[-1] / 2, 2 * dt)
    try:
        p, _ = curve_fit(lambda tt, a, b, tau: a + b * np.exp(-tt / tau), t, x,
                         p0=(a0, b0, tau0), bounds=([-np.inf, -np.inf, dt], [np.inf, np.inf, 60.0]),
                         maxfev=2000)
    except (RuntimeError, ValueError):
        return None
    fit = p[0] + p[1] * np.exp(-t / p[2])
    ss = np.sum((x - x.mean()) ** 2)
    if ss <= 0 or 1 - np.sum((x - fit) ** 2) / ss < 0.5 or p[2] > 4 * t[-1]:
        return None
    return float(p[2])


class _Prepared:
    """What every event's measurement reads, computed once per flight."""

    def __init__(self, fl: Flight, dome: Optional[DomeMap]) -> None:
        self.fl = fl
        self.motion = fl.motion()
        self.thrust = fl.thrust()
        self.positions = []
        nch = fl.wsk.shape[1]
        name_ch = {n: i for i, n in enumerate(fl.names)}
        if dome is not None:
            from whisker_link.channels import CHANNEL_NAMES
            for p in dome.positions:
                g = {}
                for ax in ("drag", "lift"):
                    ga = p.daq[ax]
                    c = name_ch.get(CHANNEL_NAMES[ga.ch]) if ga.ch is not None else None
                    g[ax] = (c, ga.polarity)
                self.positions.append((p.key, g))
        used = sorted({c for _, g in self.positions for c, _ in g.values() if c is not None})
        x = np.nan_to_num(fl.wsk[:, used]) if used else np.zeros((len(fl.t), 0))
        sos = butter(4, BAND_HZ, "bandpass", fs=FS_HZ, output="sos")
        long = len(fl.t) > 3 * 4 * 2 * 4
        band = sosfiltfilt(sos, x - x.mean(axis=0), axis=0) if long and x.size else x * 0
        self.band = np.zeros((len(fl.t), nch))
        self.band[:, used] = band
        # EventTracker's surface is the median of the first disarmed seconds: a run
        # armed (and submerged) from its start has a wrong one; depth reads negative
        d = fl.depth_cm
        self.depth_min = float(np.nanmin(d)) if np.isfinite(d).any() else None
        self.depth_ok = self.depth_min is not None and self.depth_min > -DEPTH_NEG_CM
        self.ref = None
        self.ref_names = []
        if dome is not None and dome.reference is not None:
            from whisker_link.channels import CHANNEL_NAMES
            rc = [name_ch.get(CHANNEL_NAMES[c]) for c in dome.reference.gauges]
            rc = [c for c in rc if c is not None and not fl.railed[:, c].mean() > 0.5]
            if rc:
                r = fl.wsk[:, rc].mean(axis=1)
                lp = butter(2, REF_LP_HZ, "low", fs=FS_HZ, output="sos")
                self.ref = sosfiltfilt(lp, r) if long else r
                self.ref_names = [fl.names[c] for c in rc]


def _span(fl: Flight, t0: float, t1: float) -> slice:
    return slice(fl.i_at(t0), max(fl.i_at(t0) + 1, fl.i_at(t1)))


def _baseline_rest(segs: list[Segment], j: int) -> Optional[Segment]:
    """The quiet rest before the event at the start of segment ``j`` (its hold, or
    the transition into it): the newest armed rest that ends at or before it,
    without a disarmed span between."""
    k = j - 1
    while k >= 0:
        s = segs[k]
        if not s.armed:
            return None
        if s.cls == "rest":
            return s
        k -= 1
    return None


def measure(pp: _Prepared, segs: list[Segment], j: int, kind: str) -> dict:
    """One event: ``kind`` "hold" (segment ``j``) or "transition" (between segments
    j - 1 and j)."""
    fl = pp.fl
    dt = fl.dt
    seg = segs[j]
    run0, run1 = seg.t0, seg.t1                     # the armed run around the event
    k = j
    while k - 1 >= 0 and segs[k - 1].armed:
        k -= 1
    run0 = segs[k].t0
    k = j
    while k + 1 < len(segs) and segs[k + 1].armed:
        k += 1
    run1 = segs[k].t1
    ev: dict = {"kind": kind}
    if kind == "hold":
        t = seg.t0
        t_start, t_end = seg.t0, seg.t1
        resp = seg
        ev.update({"type": "hold", "class": seg.cls, "axes": seg.axes, "minor": seg.minor,
                   "level": [_f(v, 3) for v in seg.level], "duration_s": _f(seg.duration)})
        peak_win = (seg.t0, seg.t1)
        resp_win = (seg.t0, seg.t1)
    else:
        a, resp = segs[j - 1], seg
        t = seg.t0
        typ = transition_type(a, seg)
        dlev = seg.level - a.level
        ax = int(np.argmax(np.abs(dlev)))
        side = a if typ == "release" else seg
        t_start, t_end = max(run0, t - PRE_S), min(run1, t + POST_S)
        ev.update({"type": typ, "class": side.cls, "from": a.cls, "to": seg.cls,
                   "axes": side.axes, "minor": side.minor, "axis": AXES[ax],
                   "sign": int(np.sign(side.level[ax])) if side.signs[ax] else
                   int(np.sign(dlev[ax])),
                   "level_from": [_f(v, 3) for v in a.level],
                   "level_to": [_f(v, 3) for v in seg.level],
                   "next_s": _f(seg.duration)})
        peak_win = (t, min(t + PEAK_S, run1))
        resp_win = (t, t_end)
    prev = segs[j - 1] if j > 0 and segs[j - 1].armed else None
    quiet = prev.duration if prev is not None and prev.cls == "rest" else 0.0
    ev.update({"t": _f(t, 7), "t_start": _f(t_start, 7), "t_end": _f(t_end, 7)})
    win = _span(fl, t_start, t_end)
    rwin = _span(fl, *resp_win)
    pwin = _span(fl, *peak_win)
    has_steady = resp.duration >= STEADY_MIN_S
    swin = _span(fl, resp.t1 - resp.duration / 3.0, resp.t1) if has_steady else None
    is_release = ev["type"] == "release"
    dwin = _span(fl, t, min(t + POST_S, resp.t1)) if is_release else None

    # ----- context
    m = fl.mode[win]
    m0 = int(fl.mode[min(fl.i_at(t), len(fl.t) - 1)])
    depth = _nanmedian(fl.depth_cm[win])
    dmin = np.nanmin(fl.depth_cm[win]) if np.isfinite(fl.depth_cm[win]).any() else np.nan
    roll = np.degrees(np.nanmean(fl.roll[win])) if np.isfinite(fl.roll[win]).any() else np.nan
    pitch = np.degrees(np.nanmean(fl.pitch[win])) if np.isfinite(fl.pitch[win]).any() else np.nan
    base_seg = _baseline_rest(segs, j)
    clean = base_seg is not None and base_seg.duration >= QUIET_S
    if base_seg is not None:
        b0 = max(base_seg.t0, base_seg.t1 - BASELINE_S)
        bwin = _span(fl, b0, base_seg.t1)
        base_t = (b0, base_seg.t1)
    else:                                            # none: the second before (not clean)
        bwin = _span(fl, max(run0, t - BASELINE_S), t)
        base_t = (max(run0, t - BASELINE_S), t)
    th_r = np.nanmean(pp.thrust[rwin], axis=0) if np.isfinite(pp.thrust[rwin]).any() \
        else np.full(6, np.nan)
    prew = _span(fl, max(run0, t - PRE_MOTION_S), t)
    th_p = np.nanmean(pp.thrust[prew], axis=0) if np.isfinite(pp.thrust[prew]).any() \
        else np.full(6, np.nan)
    ev["context"] = {
        "mode": ARDUSUB_MODES.get(m0, None if m0 < 0 else f"mode {m0}"), "mode_id": m0,
        "mode_changes": bool(len(np.unique(m)) > 1),
        "depth_cm": _f(depth), "near_surface": bool(np.isfinite(dmin) and dmin < NEAR_SURFACE_CM)
        if pp.depth_ok else None,
        "range_m": _f(_nanmedian(fl.range_m[win])),
        "roll_deg": _f(roll), "pitch_deg": _f(pitch),
        "level": bool(np.isfinite(roll) and abs(roll) < LEVEL_DEG and abs(pitch) < LEVEL_DEG),
        "quiet_before_s": _f(quiet), "baseline_clean": bool(clean),
        "baseline_t": [_f(base_t[0], 7), _f(base_t[1], 7)],
        "baseline_age_s": _f(t - base_t[1]),
        "pilot_gain": _f(_nanmedian(fl.gain[win]), 3),
        "thrust": {d: _f(v, 3) for d, v in zip(DOF, th_r)},
        "thrust_pre": {d: _f(v, 3) for d, v in zip(DOF, th_p)},
    }

    # ----- motion achieved
    mo = {}
    for c, name in enumerate(MOTION):
        x = pp.motion[:, c]
        pre = _nanmedian(x[prew])
        peak = _signed_peak(x[pwin] - pre) if kind == "transition" else _signed_peak(x[pwin])
        steady = _nanmedian(x[swin]) if swin is not None else np.nan
        rise = None
        if kind == "transition" and np.isfinite(steady) and np.isfinite(pre):
            scatter = np.nanstd(x[prew]) if np.isfinite(x[prew]).sum() > 2 else 0.0
            if abs(steady - pre) > max(MOTION_FLOOR[name], 3 * scatter):
                rise = rise_time(x[_span(fl, t, resp.t1)], pre, steady, dt)
        tau = fit_decay(x[dwin], dt) if dwin is not None else None
        mo[name] = {"pre": _f(pre), "peak": _f(peak), "steady": _f(steady),
                    "rise63_s": _f(rise), "tau_s": _f(tau)}
    ev["motion"] = mo

    # ----- whiskers
    ref_b = _nanmedian(pp.ref[bwin]) if pp.ref is not None else 0.0
    ref_dev = (pp.ref - ref_b) if pp.ref is not None else np.zeros(len(fl.t))
    ev["reference"] = {"gauges": pp.ref_names,
                       "shift_mv": _f(_nanmedian(ref_dev[rwin])) if pp.ref is not None else None}
    pos_out = {}
    for key, g in pp.positions:
        po = {}
        sq = 0.0
        railed = False
        for ax, (c, pol) in g.items():
            if c is None:
                po[ax] = None
                continue
            raw = fl.wsk[:, c]
            base = _nanmedian(raw[bwin])
            def rel(sl):
                return pol * (raw[sl] - base - ref_dev[sl])
            peak = _signed_peak(rel(pwin))
            steady = _nanmedian(rel(swin)) if swin is not None else np.nan
            tau = fit_decay(rel(dwin), dt) if dwin is not None else None
            band = float(np.sqrt(np.mean(pp.band[rwin, c] ** 2))) * 1e3
            sq += band ** 2
            railed = railed or bool(fl.railed[win, c].any())
            po[ax] = {"base": _f(base, 6), "peak": _f(peak), "steady": _f(steady),
                      "tau_s": _f(tau), "band_uv": _f(band, 3)}
        po["band_uv"] = _f(np.sqrt(sq), 3)
        po["railed"] = railed
        pos_out[key] = po
    active = not (kind == "hold" and seg.cls == "rest")
    bands = [p["band_uv"] for p in pos_out.values() if p["band_uv"] is not None]
    flying = active and len(bands) and np.median(bands) >= 2 * ALIVE_UV
    for p in pos_out.values():
        b = p["band_uv"]
        p["alive"] = True if b is not None and b >= ALIVE_UV else (False if flying else None)
    ev["positions"] = pos_out

    # ----- quality
    nflag = int(np.count_nonzero((fl.flag_t >= t_start) & (fl.flag_t < t_end)))
    ev["quality"] = {"adc_flagged_scans": nflag,
                     "railed": [k for k, p in pos_out.items() if p["railed"]],
                     "lost": [k for k, p in pos_out.items() if p["alive"] is False],
                     "baseline_clean": bool(clean), "steady": bool(has_steady)}
    return ev


def mix_check(fl: Flight, segs: list[Segment]) -> dict:
    """The thrust mix against the demand: the sign each servo takes against the
    mixed RC demand (``MOTOR_DIRECTION`` should match it; 0 where the correlation
    is under 0.2, e.g. a motor no stick asked for), and on single-axis
    holds of at least 1 s the mean projected thrust of that axis' DOF against the
    mean demand (slope, r), with the largest mean |thrust| leaking into another DOF
    (roll / pitch excluded: the attitude controller drives them)."""
    th_servo = (fl.servo - NEUTRAL_US) / FULL_US
    ok = fl.armed & np.isfinite(th_servo).all(axis=1)
    out: dict = {"n": int(ok.sum())}
    if ok.sum() < FS_HZ:
        return out
    dof_of = {"surge": 4, "sway": 5, "heave": 3, "yaw": 2}
    cmd = np.zeros((len(fl.t), 6))
    for a, d in dof_of.items():
        cmd[:, d] = fl.u[:, AXES.index(a)]
    pred = cmd[ok] @ MIX.T
    sign = []                                       # 0: that motor was never asked
    for i in range(8):
        p, s = pred[:, i], th_servo[ok, i]
        den = np.sqrt(np.sum(p * p) * np.sum(s * s))
        r = np.sum(p * s) / den if den > 0 else 0.0
        sign.append(int(np.sign(r)) if abs(r) > 0.2 else 0)
    out["servo_sign"] = sign
    out["direction_ok"] = any(sign) and all(s in (0, d) for s, d in zip(sign, MOTOR_DIRECTION))
    th = fl.thrust()
    axes = {}
    for a, d in dof_of.items():
        k = AXES.index(a)
        xs, ys, leak = [], [], []
        modes = []
        for s in segs:
            if not s.armed or s.duration < 1.0 or not s.cls.startswith(f"single:{a}"):
                continue
            sl = slice(s.i0, s.i1)
            xs.append(float(np.median(fl.u[sl, k])))
            m = np.nanmean(th[sl], axis=0)
            ys.append(float(m[d]))
            leak.append(max(abs(m[o]) for o in (2, 3, 4, 5) if o != d))
            modes.append(int(np.median(fl.mode[sl])))
        if len(xs) >= 2 and np.std(xs) > 0:
            x, y = np.asarray(xs), np.asarray(ys)
            slope = float(np.sum(x * y) / np.sum(x * x))
            r = float(np.corrcoef(x, y)[0, 1]) if np.std(y) > 0 else float("nan")
        else:
            slope = r = float("nan")
        axes[a] = {"holds": len(xs), "slope": _f(slope, 3), "r": _f(r, 3),
                   "leak_max": _f(max(leak), 3) if leak else None,
                   "modes": sorted(set(ARDUSUB_MODES.get(m, str(m)) for m in modes))}
    out["axes"] = axes
    return out


def extract(fl: Flight, dome: Optional[DomeMap] = None) -> dict:
    """The sidecar document of a flight (module doc)."""
    if dome is None:
        dome = DomeMap.load(DEFAULT_MAP)
    segs = segment(fl)
    pp = _Prepared(fl, dome)
    events = []
    for j, s in enumerate(segs):
        if not s.armed:
            continue
        if j > 0 and segs[j - 1].armed:
            events.append(measure(pp, segs, j, "transition"))
        events.append(measure(pp, segs, j, "hold"))
    for i, e in enumerate(events):
        e["id"] = i
    am = fl.armed
    rest = {}
    for k, a in enumerate(AXES):
        x = fl.u[am, k]
        small = x[np.abs(x) <= REST_OFFSET_MAX]
        rest[a] = {"offset": _f(np.median(small), 3) if len(small) else None,
                   "active_frac": _f(np.mean(np.abs(x) > REST_TOL), 3) if len(x) else None,
                   # active but small (rest offset or a minor input)
                   "small_frac": _f(np.mean((np.abs(x) > REST_TOL) & (np.abs(x) < MINOR)), 3)
                   if len(x) else None}
    rn = recname.parse(fl.stem)
    return {"v": VERSION, "stem": fl.stem,
            "start": rn.start.isoformat(timespec="seconds") if rn else None,
            "tag": rn.tag if rn else "", "made_by": MADE_BY, "constants": constants(),
            "recording": {**fl.meta, "armed_s": _f(am.sum() / FS_HZ),
                          "rest": rest, "reference": pp.ref_names,
                          "dome_map": dome.id if dome is not None else None,
                          "whisker_set": dome.whisker_set if dome is not None else None,
                          "depth_min_cm": _f(pp.depth_min), "depth_ok": pp.depth_ok,
                          "positions": [k for k, _ in pp.positions],
                          "mix_check": mix_check(fl, segs)},
            "segments": [{"t0": _f(s.t0, 7), "t1": _f(s.t1, 7), "class": s.cls, "axes": s.axes,
                          "minor": s.minor,
                          "level": [_f(v, 3) for v in s.level]} for s in segs],
            "events": events}


# ---------------------------------------------------------------------------
# sidecar
# ---------------------------------------------------------------------------
def manoeuvres_path(recording: Union[str, Path], out_dir: Union[str, Path, None] = None) -> Path:
    p = recname.sidecar(recording, SUFFIX)
    return Path(out_dir) / p.name if out_dir is not None else p


def write_doc(path: Union[str, Path], doc: dict) -> Path:
    """Write ``doc`` (a temporary file, then replace: a reader never sees half)."""
    path = Path(path)
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(doc, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    for i in range(20):
        try:
            os.replace(tmp, path)
            return path
        except PermissionError:          # Windows: someone has it open; briefly
            if i == 19:
                tmp.unlink(missing_ok=True)
                raise
            time.sleep(0.1 * (i + 1))
    return path


def read_doc(path: Union[str, Path]) -> Optional[dict]:
    """The sidecar at ``path``; None if missing, unreadable or of another version."""
    try:
        doc = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return doc if isinstance(doc, dict) and doc.get("v") == VERSION else None


def is_current(recording: Union[str, Path], out_dir: Union[str, Path, None] = None) -> bool:
    """A sidecar of this version exists and is newer than the recording."""
    rec, mp = Path(recording), manoeuvres_path(recording, out_dir)
    try:
        if mp.stat().st_mtime_ns < rec.stat().st_mtime_ns:
            return False
    except OSError:
        return False
    return read_doc(mp) is not None


def write_manoeuvres_for(recording: Union[str, Path], out_dir: Union[str, Path, None] = None,
                         cache: bool = True, dome: Optional[DomeMap] = None
                         ) -> tuple[Path, dict, dict]:
    """Load the recording, extract and write its sidecar next to it (or into
    ``out_dir``). ``cache``: the index / store caches next to the recording as
    replay keeps them. Returns (path, doc, timings in s: load, extract)."""
    rec = Path(recording)
    t0 = time.perf_counter()
    fl = load_flight(rec, cache=cache)
    t1 = time.perf_counter()
    doc = extract(fl, dome if dome is not None else dome_map_for(rec))
    t2 = time.perf_counter()
    if out_dir is not None:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
    out = write_doc(manoeuvres_path(rec, out_dir), doc)
    return out, doc, {"load": t1 - t0, "extract": t2 - t1}


def write_manoeuvres_in_background(recording: Union[str, Path], on_done=None) -> threading.Thread:
    """The sidecar of a recording that just closed, in a child process at a lower
    priority (``tools/make_manoeuvres.py``; as ``card.write_card_in_background``).
    ``on_done(path, error or None)`` is called from the thread."""
    rec = Path(recording)

    def run() -> None:
        err = None
        try:
            if getattr(sys, "frozen", False):
                write_manoeuvres_for(rec)
            else:
                root = Path(__file__).resolve().parents[2]
                env = dict(os.environ)
                env["PYTHONPATH"] = os.pathsep.join(
                    [str(root)] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
                env["PYTHONIOENCODING"] = "utf-8"
                flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
                r = subprocess.run(
                    [sys.executable, "-m", "wavi_py.tools.make_manoeuvres", "--quiet", "--nice",
                     "--force", "--no-csv", str(rec)],
                    cwd=str(root), env=env, stdin=subprocess.DEVNULL, capture_output=True,
                    timeout=1800, creationflags=flags)
                if r.returncode != 0:
                    err = (r.stderr.decode("utf-8", "replace").strip().splitlines() or ["?"])[-1]
        except Exception as exc:  # noqa: BLE001 — an aid: report, never raise
            err = f"{type(exc).__name__}: {exc}"
        print(f"[manoeuvres] {rec.name}: " + (f"not written ({err})" if err else
                                             f"wrote {manoeuvres_path(rec).name}"))
        if on_done is not None:
            on_done(manoeuvres_path(rec), err)

    th = threading.Thread(target=run, name="wavi-manoeuvres", daemon=True)
    th.start()
    return th


# ---------------------------------------------------------------------------
# flat table
# ---------------------------------------------------------------------------
CSV_EVENT = ("stem", "id", "kind", "type", "class", "axes", "axis", "sign", "t", "t_start",
             "t_end", "duration_s", "next_s", "mode", "depth_cm", "near_surface", "level",
             "quiet_before_s", "baseline_clean", "pilot_gain")
CSV_LEVEL = tuple(f"u_{a}" for a in AXES)
CSV_THRUST = tuple(f"thrust_{d}" for d in DOF)
CSV_MOTION = tuple(f"{m}_{q}" for m in ("yaw_rate_dps", "depth_rate_cms") for q in
                   ("peak", "steady", "rise63_s"))
CSV_POS = ("position", "alive", "railed", "band_uv") + tuple(
    f"{ax}_{q}" for ax in ("drag", "lift") for q in ("base", "peak", "steady", "tau_s", "band_uv"))
CSV_COLUMNS = CSV_EVENT + CSV_LEVEL + CSV_THRUST + CSV_MOTION + ("ref_shift_mv",
                                                                  "adc_flagged_scans") + CSV_POS


def csv_rows(doc: dict):
    """One dict per event x position (CSV_COLUMNS)."""
    for e in doc["events"]:
        c = e["context"]
        lev = e.get("level") or e.get("level_to")
        row = {"stem": doc["stem"], "id": e["id"], "kind": e["kind"], "type": e["type"],
               "class": e["class"], "axes": e["axes"], "axis": e.get("axis", ""),
               "sign": e.get("sign", ""), "t": e["t"], "t_start": e["t_start"],
               "t_end": e["t_end"], "duration_s": e.get("duration_s"), "next_s": e.get("next_s"),
               "mode": c["mode"], "depth_cm": c["depth_cm"], "near_surface": c["near_surface"],
               "level": c["level"], "quiet_before_s": c["quiet_before_s"],
               "baseline_clean": c["baseline_clean"], "pilot_gain": c["pilot_gain"],
               "ref_shift_mv": e["reference"]["shift_mv"],
               "adc_flagged_scans": e["quality"]["adc_flagged_scans"]}
        row.update({f"u_{a}": v for a, v in zip(AXES, lev)})
        row.update({f"thrust_{d}": c["thrust"][d] for d in DOF})
        for m in ("yaw_rate_dps", "depth_rate_cms"):
            for q in ("peak", "steady", "rise63_s"):
                row[f"{m}_{q}"] = e["motion"][m][q]
        for key, p in e["positions"].items():
            r = dict(row, position=key, alive=p["alive"], railed=p["railed"],
                     band_uv=p["band_uv"])
            for ax in ("drag", "lift"):
                for q in ("base", "peak", "steady", "tau_s", "band_uv"):
                    r[f"{ax}_{q}"] = p[ax][q] if p.get(ax) else None
            yield r


def write_events_csv(docs: list, path: Union[str, Path]) -> Path:
    """The flat table of several sidecars (one row per event x position)."""
    path = Path(path)
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(CSV_COLUMNS))
        w.writeheader()
        for d in docs:
            for r in csv_rows(d):
                w.writerow({k: ("" if v is None else v) for k, v in r.items()})
    os.replace(tmp, path)
    return path
