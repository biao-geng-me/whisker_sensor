"""Selections and variables for the Analysis windows: the data side (no Qt).

A ``Selection`` is a time window [t0, t1] on the session axis plus the data it
reads (``SelSource``): the whole-recording store in replay, a copy of the live
buffers taken when live was paused (so it keeps its data after the buffers move
on; adjusting it is limited to what was copied). The selection model and its
drawing are in ``views/selections.py``.

A ``Variable`` is one curve's worth of data by key: ``wsk:<col>`` (a gauge, the
series' stream-order column), ``imu:<col>`` (``IMU_CHANNELS``), ``depth`` (the
source's depth from the external pressure, which is not in the series) or
``rov:<name>`` (the vehicle telemetry, ``replay/rov.py``: pilot commands,
thrusters, current, climb, heading, vibration; not in the series either, so
read like the depth). Each has a unit group, the rows of the waveform and the
spectrum: mV, mg, mrad/s, deg, cm, as the IMU window shows them (accel and gyro
as the series has them, the attitude's rad in degrees), then the ROV's: pilot
and thrust (-1 .. +1), A, m/s, hdg (deg, unwrapped), m/s². ``rov:mode`` (the
mode spans, armed and the pilot gain) is a timeline band only (``plots``
False): it is not a curve. The gauges are
drawn as the Line window draws them, (raw - V0) * scale, so the curves line up
with what was selected.

``Dataset`` is what a tab reads: (included selections) x (checked variables) of
one Analysis window, with the panel's Despike / V0 / scale. ``traces(n)`` is
Compare's: the time relative to each selection's start, min / max decimated to
``n`` pixel columns over the longest selection (``Series.envelope``: one call per
source per selection, from the store's pyramid in replay), so a long selection
costs about what a short one does. ``samples(sel, var)`` is every sample.

The spectra read the dataset's ``span(sel, sub)``: the whole selection, or with
a sub-range ``sub = (a, b)`` (seconds from each selection's start) [t0 + a,
t0 + b] clipped to the selection, None when it is shorter than a. The window
can have several numbered sub-ranges (``subs``: ``(number, a, b)``); each is
then its own trace per (selection, variable) (``triples``: sub None is the
whole selection, used when there are none). ``sub`` (one range, no number) is
the older single sub-range, still taken when ``subs`` is empty.
``spectrum`` is the amplitude spectrum of every sample of the span (the method
of ``daq/spectra.amp_spectra``: a sine of amplitude a reads a), at the span's
own mean sample rate, so selections of different lengths each have their own
frequency resolution (1 / length); IMU rows come a little unevenly and are
taken as evenly spaced at that rate. ``spectrogram`` is the same on segments
of ``seg_s`` overlapping by ``overlap`` (Hann, mean removed, as the Spec
window), timed at their centres from the selection's start;
``spectrogram_column`` is one segment of it (the Analysis window's cursors).

``overview`` is the Analysis window's timeline: the dataset's variables over an
absolute span of a source (the whole recording), min / max decimated to the
plot width, in the same units.

Events (``replay/eventset.py`` makes such selections, each with its own
recording's ``SelSource``):

- ``Selection.align``: seconds from the selection's start to its event's onset;
  set, the time axis is relative to the onset (t = 0 at the stick), not to the
  start. ``group`` names its group, ``pinned`` keeps it drawn, ``baseline`` is
  its pre-event baseline window (absolute), ``dome_map`` its recording's map;
- the ROV variables per axis (``rov_axis_variables``): the pilot's demand
  (``rov:fwd`` ...) and the thrust projected from the eight thrusters through the
  frame's mix (``rov:th_surge`` ..., ``daq/vehicle.py``), the same keys in every
  recording;
- ``pos:<position>:<drag|lift>``: a dome position's gauge, resolved per
  selection through its ``dome_map`` (else the dataset's), polarity applied, so
  one variable reads different gauges in recordings wired or mounted
  differently;
- ``Dataset.baseline``: the gauges (``wsk`` and ``pos``) minus each selection's
  baseline (the median over its baseline window) and minus the reference drift
  where its map has a reference: the mean of the reference gauges, low-passed at
  ``REF_LP_HZ`` over the whole recording (``reference_drift``, cached per
  series; as ``replay/manoeuvres.py``), minus its own baseline. V0 does
  not apply then (the baseline replaces it); the scale does;
- ``max_traces``: ``traces`` draws at most this many selections (the pinned ones
  first, then evenly spread over the rest); ``aggregate`` uses every one: per
  group the median and the 25-75 % band of a variable on a common grid of
  relative time.
"""

from __future__ import annotations

import math
import weakref
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from wavi_py.daq.series import IMU, WSK, ArraySeries, Series, peak_decimate, span_index
from wavi_py.replay.rov import AXIS_KEYS, MODE_KEY, RovData, axis_variables

DEPTH = "depth"
ROV = "rov"
POS = "pos"
CARD = "card"                   # the timeline's activity row (the recording card's preview)
CARD_KEY = "rec:card"
# the rows, top to bottom ("mode" is the timeline's band row, "card" its heatmap row:
# the timeline puts both on top)
UNIT_GROUPS = ("mV", "mg", "mrad/s", "deg", "cm", "mode", "pilot", "thrust", "A", "m/s", "hdg",
               "m/s²", "card")
MAX_TRACES = 200                # selections ``traces`` draws at most
REF_LP_HZ = 0.5                 # the reference drift's low-pass (replay/manoeuvres.py)
AGG_DT = 0.02                   # the aggregates' grid step (s)
# the dome's positions in the plane map's order (``views/sensor_grid``: port left,
# starboard right, front columns toward the centre; rows upper, middle, lower)
DOME_COLS = ("LB", "LF", "RF", "RB")
DOME_ROWS = ("U", "M", "L")
DOME_ORDER = tuple(c + r for r in DOME_ROWS for c in DOME_COLS)
_RAD_DEG = 180.0 / math.pi

# (IMU column, group label, axis label, unit, factor): the IMU window's units
IMU_VARS = (
    (0, "Accel", "x", "mg", 1.0), (1, "Accel", "y", "mg", 1.0), (2, "Accel", "z", "mg", 1.0),
    (3, "Gyro", "x", "mrad/s", 1.0), (4, "Gyro", "y", "mrad/s", 1.0),
    (5, "Gyro", "z", "mrad/s", 1.0),
    (6, "Attitude", "roll", "deg", _RAD_DEG), (7, "Attitude", "pitch", "deg", _RAD_DEG),
    (8, "Attitude", "yaw", "deg", _RAD_DEG),
)


@dataclass
class SelSource:
    """What a selection reads: a ``Series``, the depth ``(t, cm)`` (may be empty) and
    the vehicle telemetry (``RovData``, may be empty). ``snapshot``: a copy of the
    live buffers (one version: Despike does not apply)."""
    series: Series
    depth: tuple = (np.empty(0), np.empty(0))
    snapshot: bool = False
    rov: RovData = field(default_factory=RovData)

    def aux(self, var: "Variable") -> tuple[np.ndarray, np.ndarray]:
        """(t, y) of a variable that is not in the series (depth, ROV)."""
        if var.source == DEPTH:
            return self.depth
        return self.rov.series(var.key)

    def extent(self) -> Optional[tuple[float, float]]:
        """The span the selection can cover: whisker and IMU data together."""
        ex = [e for e in (self.series.extent(WSK), self.series.extent(IMU)) if e is not None]
        if not ex:
            return None
        return min(e[0] for e in ex), max(e[1] for e in ex)


def snapshot_of(series: Series, depth=None, rov: Optional[RovData] = None) -> SelSource:
    """A copy of ``series`` (the live buffers: they move on) as a ``SelSource``."""
    data, names = {}, {}
    for src in (WSK, IMU):
        ext = series.extent(src)
        if ext is None:
            continue
        data[src] = series.get(src, slice(None), ext[0], ext[1])     # copies
        names[src] = series.names(src)
    d = (np.empty(0), np.empty(0)) if depth is None else \
        (np.array(depth[0], dtype=float), np.array(depth[1], dtype=float))
    return SelSource(ArraySeries(data, names), d, snapshot=True,
                     rov=rov.copy() if rov else RovData())


@dataclass(eq=False)
class Selection:
    """One time window (see the module doc). ``number`` labels it everywhere and
    picks its colour; ``bounds`` is the source's extent (edits are clamped to it)."""
    number: int
    t0: float
    t1: float
    source: SelSource
    bounds: Optional[tuple[float, float]] = None
    align: Optional[float] = None       # s from t0 to the event's onset (the time zero)
    group: str = ""
    pinned: bool = False
    baseline: Optional[tuple[float, float]] = None   # absolute, the pre-event baseline
    dome_map: object = None             # whisker_link.dome_map.DomeMap of its recording
    label: str = ""
    event: object = None                # replay.eventset.EventRef, for event selections

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    @property
    def zero(self) -> float:
        """The absolute time drawn as 0: the onset if aligned, else the start."""
        return self.t0 + (self.align or 0.0)

    def clamp(self, t0: float, t1: float) -> tuple[float, float]:
        if t1 < t0:
            t0, t1 = t1, t0
        if self.bounds is not None:
            lo, hi = self.bounds
            t0, t1 = min(max(t0, lo), hi), min(max(t1, lo), hi)
        return float(t0), float(t1)


@dataclass(frozen=True)
class Variable:
    key: str                 # wsk:<col>, imu:<col>, rov:<name>, pos:<name>:<axis>, depth
    label: str               # "W3 A", "Accel x", "Depth"
    unit: str                # one of UNIT_GROUPS
    source: str              # WSK, IMU, DEPTH, ROV or POS
    col: int = 0
    factor: float = 1.0
    group: str = ""          # the tree's parent: the sensor name, "Accel", ...
    plots: bool = True       # False: timeline only (the mode band)
    pos: str = ""            # POS: the position (e.g. "LFM") and axis ("drag" / "lift")
    axis: str = ""


def whisker_variables(nch: int, sensor_names: Sequence[str]) -> list[Variable]:
    """The gauges, in stream order (W1 A, W1 B, W2 A, ...), named as the Line window
    names the sensors (a dome map's names)."""
    out = []
    for c in range(nch):
        s = c // 2
        sname = sensor_names[s] if s < len(sensor_names) else f"W{s + 1}"
        out.append(Variable(f"wsk:{c}", f"{sname} {'AB'[c % 2]}", "mV", WSK, c, 1.0, sname))
    return out


def imu_variables(depth: bool = True) -> list[Variable]:
    out = [Variable(f"imu:{c}", f"{g} {a}", u, IMU, c, f, g) for c, g, a, u, f in IMU_VARS]
    if depth:
        out.append(Variable(DEPTH, "Depth", "cm", DEPTH, 0, 1.0, "Depth"))
    return out


def rov_variables(rov: Optional[RovData]) -> list[Variable]:
    """The vehicle telemetry ``rov`` has (``RovData.variables``); none without."""
    if not rov:
        return []
    return [Variable(k, lab, u, ROV, 0, 1.0, g, plots=(k != MODE_KEY))
            for k, lab, u, g in rov.variables()]


def card_variable() -> Variable:
    """The timeline's activity row: the recording card's preview (``replay/card.py``),
    timeline only."""
    return Variable(CARD_KEY, "Activity (card)", "card", CARD, 0, 1.0, "Recording", plots=False)


def card_image(source: SelSource, t0: float, t1: float, nbins: int,
               despike: bool = False) -> Optional[tuple[np.ndarray, float, float]]:
    """The recording card's preview over [t0, t1] of ``source`` (clipped to the
    whisker data): each gauge's RMS about its own mean in each of ``nbins`` equal
    spans, in dB re 1 mV quantised to 0..255 on the card's fixed scale
    (``card.rms_db`` / ``quantise``: a card's picture, re-binned to the span shown).
    (q (nch x nbins) uint8, a, b) or None (no whisker data there)."""
    from wavi_py.replay.card import quantise, rms_db
    ext = source.series.extent(WSK)
    if ext is None or nbins < 1:
        return None
    a, b = max(t0, ext[0]), min(t1, ext[1])
    if not b > a:
        return None
    t, y = source.series.get(WSK, slice(None), a, b, despike)
    if not len(t) or y.ndim != 2:
        return None
    return quantise(rms_db(np.asarray(t, np.float64) - a, y, b - a, nbins)), a, b


def rov_axis_variables() -> list[Variable]:
    """The pilot's demand and the projected thrust per axis (surge, sway, heave,
    yaw; ``replay/rov.axis_variables``): the ROV variables every recording with a
    vehicle has, so an Events window lists them without a recording open."""
    return [Variable(k, lab, u, ROV, 0, 1.0, g) for k, lab, u, g in axis_variables()]


def _aux_samples(t: np.ndarray, y: np.ndarray, t0: float, t1: float):
    """The samples of an aux series in [t0, t1], NaN (not sent) dropped."""
    i0, i1 = span_index(t, t0, t1)
    t, y = np.asarray(t[i0:i1], float), np.asarray(y[i0:i1], float)
    ok = np.isfinite(y)
    return (t, y) if ok.all() else (t[ok], y[ok])


def position_variables(positions: Sequence[str]) -> list[Variable]:
    """``pos:<name>:drag`` and ``pos:<name>:lift`` of each position (resolved per
    selection through its dome map)."""
    return [Variable(f"pos:{p}:{ax}", f"{p} {ax}", "mV", POS, 0, 1.0, p, pos=p, axis=ax)
            for p in positions for ax in ("drag", "lift")]


def variable_of(key: str, sensor_names: Sequence[str] = ()) -> Optional[Variable]:
    """The variable of a key (``wsk:3``, ``imu:0``, ``pos:LFM:drag``, ``depth``, a
    per-axis ``rov:`` key); None if malformed or unknown."""
    try:
        kind, _, rest = key.partition(":")
        if kind == POS:
            p, ax = rest.split(":")
            return position_variables([p])[("drag", "lift").index(ax)]
        if kind == DEPTH:
            return imu_variables()[-1]
        if kind == ROV:
            return next((v for v in rov_axis_variables() if v.key == key), None)
        c = int(rest)
        if kind == WSK:
            return whisker_variables(c + 1, sensor_names)[c]
        if kind == IMU:
            return imu_variables(False)[c]
    except (ValueError, IndexError):
        return None
    return None


def envelope_within(series: Series, source: str, cols, t0: float, t1: float, n: int,
                    despike: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """``Series.envelope`` of exactly [t0, t1]: the envelope's edge blocks run past
    the span (they are aligned to the absolute sample index), so their min / max
    may come from outside it. Those blocks are dropped and the two edge pieces put
    back as the samples themselves (each at most about a block, so still cheap)."""
    t, y = series.envelope(source, cols, t0, t1, n, despike)
    if len(t) % 2:                                  # raw samples: pad to pairs
        t, y = np.r_[t, t[-1:]], np.concatenate([y, y[-1:]])
    a, b = t[0::2], t[1::2]                         # each pair: a block's first / last time
    ok = (a >= t0) & (b <= t1)
    keep = np.repeat(ok, 2)
    tk, yk = t[keep], y[keep]
    if not len(tk):
        return series.get(source, cols, t0, t1, despike)
    th, yh = series.get(source, cols, t0, tk[0], despike)
    tt, yt = series.get(source, cols, tk[-1], t1, despike)
    h, e = th < tk[0], tt > tk[-1]
    return (np.concatenate([th[h], tk, tt[e]]),
            np.concatenate([yh[h], yk, yt[e]]))


@dataclass
class Trace:
    sel: Optional[Selection]  # None: the timeline's (absolute time)
    var: Variable
    t: np.ndarray            # relative: seconds from the selection's zero (timeline: absolute)
    y: np.ndarray            # in var.unit


@dataclass
class Aggregate:
    """A group's ``var`` over its events: median and 25-75 % band on ``t``
    (relative), ``n`` the events covering each point, ``events`` in the group."""
    group: str
    var: Variable
    t: np.ndarray
    median: np.ndarray
    q25: np.ndarray
    q75: np.ndarray
    n: np.ndarray
    events: int


def _lowpass(t: np.ndarray, y: np.ndarray, fc: float) -> np.ndarray:
    """Zero-phase 2nd-order low-pass at ``fc`` (the samples taken as evenly spaced);
    too few samples: the mean."""
    if len(y) < 16 or t[-1] <= t[0]:
        return np.full(len(y), float(np.mean(y)) if len(y) else 0.0)
    from scipy.signal import butter, sosfiltfilt
    fs = (len(t) - 1) / (t[-1] - t[0])
    if fc >= 0.45 * fs:
        return y.astype(float)
    sos = butter(2, fc, "low", fs=fs, output="sos")
    if len(y) <= 3 * (2 * len(sos) + 1):
        return np.full(len(y), float(np.mean(y)))
    return sosfiltfilt(sos, y.astype(float))


_GAUGES = (WSK, POS)            # the sources drawn from the whisker gauges
BASELINE_S = 1.0                # a selection without a baseline window: this much before its zero
_REF_PAD_S = 2.0                # the reference is low-passed over its span plus this either side


_REF_CACHE: "weakref.WeakKeyDictionary" = weakref.WeakKeyDictionary()


def reference_drift(series: Series, cols: Sequence[int]) -> Optional[tuple[np.ndarray, np.ndarray]]:
    """(t, mV): the mean of the reference gauges ``cols`` over the whole recording,
    low-passed at ``REF_LP_HZ`` (zero phase, as ``replay/manoeuvres.py``: over the
    whole recording, so an event's value does not depend on its window). Cached
    per series and columns while the series' extent is unchanged; None without
    samples."""
    ext = series.extent(WSK)
    if ext is None:
        return None
    try:
        per = _REF_CACHE.setdefault(series, {})
    except TypeError:                         # not weak-referenceable: no cache
        per = {}
    key = tuple(int(c) for c in cols)
    got = per.get(key)
    if got is not None and got[0] == ext:
        return got[1]
    t, y = series.get(WSK, list(key), ext[0], ext[1])
    if not len(t):
        return None
    y = np.asarray(y, float)
    with np.errstate(all="ignore"):
        r = np.nanmean(y, axis=1) if y.shape[1] > 1 else y[:, 0]
    r = np.where(np.isfinite(r), r, np.nanmedian(r) if np.isfinite(r).any() else 0.0)
    out = (np.asarray(t, float), _lowpass(np.asarray(t, float), r, REF_LP_HZ))
    per[key] = (ext, out)
    return out


@dataclass
class _Gauged:
    """A selection's baseline-mode terms, computed once: per gauge column its
    baseline (raw mV), and the reference drift as (t, mV - its baseline)."""
    base: dict
    ref: Optional[tuple]


@dataclass
class Dataset:
    """(selections) x (variables) with the display context (see the module doc)."""
    selections: list
    variables: list
    despike: bool = False
    v0: Optional[np.ndarray] = None     # per gauge column, raw mV
    scale: float = 1.0
    sub: Optional[tuple] = None         # one sub-range (a, b), relative (see the module doc)
    subs: tuple = ()                    # the numbered sub-ranges ((number, a, b), ...)
    baseline: bool = False              # gauges relative to each selection's baseline
    dome_map: object = None             # POS without a selection map (and the timeline)
    max_traces: Optional[int] = MAX_TRACES
    _names: dict = field(default_factory=dict, repr=False)
    _gauged: dict = field(default_factory=dict, repr=False)

    def span(self, sel: Selection, sub: Optional[tuple] = None) -> Optional[tuple[float, float]]:
        """The part of ``sel`` the spectra use: ``sub`` = (a, b) or (number, a, b), or
        None: ``self.sub`` if set, else the whole selection (see the module doc;
        relative to the selection's zero: its start, or an event's onset)."""
        sub = self.sub if sub is None else sub[-2:]
        if sub is None:
            return sel.t0, sel.t1
        a, b = sub
        z = sel.zero
        lo, hi = max(sel.t0, z + a), min(sel.t1, z + b)
        return (lo, hi) if hi > lo else None

    def pairs(self) -> list[tuple[Selection, Variable]]:
        return [(s, v) for s in self.selections for v in self.variables]

    def drawn(self) -> list[Selection]:
        """The selections ``traces`` draws: all of them up to ``max_traces``, else
        the pinned ones first, then the rest evenly spread (in their order)."""
        sels, cap = self.selections, self.max_traces
        if cap is None or len(sels) <= cap:
            return list(sels)
        pinned = [s for s in sels if s.pinned][:cap]
        rest = [s for s in sels if not s.pinned]
        k = cap - len(pinned)
        pick = set()
        if k > 0 and rest:
            pick = {int(i) for i in np.round(np.linspace(0, len(rest) - 1, k))}
        keep = {id(s) for s in pinned} | {id(rest[i]) for i in pick}
        return [s for s in sels if id(s) in keep]

    # ----- gauges: a column and a polarity per variable
    def map_of(self, sel: Optional[Selection]):
        m = getattr(sel, "dome_map", None) if sel is not None else None
        return m if m is not None else self.dome_map

    def gauge(self, sel: Optional[Selection], var: Variable,
              series: Optional[Series] = None) -> Optional[tuple[int, int]]:
        """(gauge column in the series, polarity) of a WSK or POS variable in
        ``sel`` (POS through its map); None where it has none."""
        ser = series if series is not None else sel.source.series
        names = ser.names(WSK)
        if var.source == WSK:
            return (var.col, 1) if var.col < len(names) else None
        if var.source != POS:
            return None
        m = self.map_of(sel)
        pos = m.position(var.pos) if m is not None else None
        g = pos.daq.get(var.axis) if pos is not None else None
        if g is None or g.ch is None:
            return None
        from whisker_link.channels import CHANNEL_NAMES
        try:
            return names.index(CHANNEL_NAMES[g.ch]), int(g.polarity)
        except ValueError:
            return None

    def _baseline_terms(self, sel: Selection) -> _Gauged:
        key = (id(sel), sel.t0, sel.t1, sel.align, sel.baseline, self.despike)
        got = self._gauged.get(key)
        if got is not None and got[0] is sel:
            return got[1]
        ser = sel.source.series
        bw = sel.baseline or (max(sel.t0, sel.zero - BASELINE_S), sel.zero)
        ref = None
        m = self.map_of(sel)
        rg = getattr(getattr(m, "reference", None), "gauges", None)
        if rg:
            from whisker_link.channels import CHANNEL_NAMES
            names = ser.names(WSK)
            cols = [names.index(CHANNEL_NAMES[c]) for c in rg if CHANNEL_NAMES[c] in names]
            got = reference_drift(ser, cols) if cols else None
            if got is not None:
                t, r = got
                i0, i1 = span_index(t, min(bw[0], sel.t0) - _REF_PAD_S, sel.t1 + _REF_PAD_S)
                t, r = t[i0:i1], r[i0:i1]
                if len(t):
                    inb = (t >= bw[0]) & (t <= bw[1])
                    rb = float(np.nanmedian(r[inb])) if inb.any() else float(np.nanmedian(r))
                    ref = (t, r - rb)
        g = _Gauged({}, ref)
        g.base["window"] = bw
        self._gauged[key] = (sel, g)
        return g

    def _base_of(self, sel: Selection, col: int) -> float:
        g = self._baseline_terms(sel)
        if col not in g.base:
            a, b = g.base["window"]
            _, y = sel.source.series.get(WSK, [col], a, b, self.despike)
            g.base[col] = float(np.nanmedian(y[:, 0])) if len(y) else 0.0
        return g.base[col]

    def _gauge_values(self, sel: Optional[Selection], var: Variable, col: int, pol: int,
                      t: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Raw mV of gauge ``col`` at ``t`` -> ``var``'s values (see the module doc)."""
        y = np.asarray(y, dtype=float)
        if self.baseline and sel is not None:
            g = self._baseline_terms(sel)
            y = y - self._base_of(sel, col)
            if g.ref is not None:
                y = y - np.interp(t, g.ref[0], g.ref[1])
            return pol * y * self.scale
        v0 = 0.0 if self.v0 is None or col >= len(self.v0) else float(self.v0[col])
        return pol * (y - v0) * self.scale

    def sub_list(self) -> list:
        """The sub-ranges the spectra use: ``subs``, or [None] (whole / ``sub``)."""
        return list(self.subs) or [None]

    def triples(self) -> list[tuple[Selection, Variable, Optional[tuple]]]:
        """(selection, variable, sub-range or None) of every spectrum trace."""
        return [(s, v, u) for s in self.selections for v in self.variables
                for u in self.sub_list()]

    def _convert(self, var: Variable, y: np.ndarray) -> np.ndarray:
        if var.source == WSK:
            v0 = 0.0 if self.v0 is None or var.col >= len(self.v0) else float(self.v0[var.col])
            return (np.asarray(y, dtype=float) - v0) * self.scale
        return np.asarray(y, dtype=float) * var.factor

    def _have(self, sel: Selection, source: str) -> int:
        """Columns ``source`` has in this selection's series (0: none)."""
        if source == DEPTH:
            return len(sel.source.depth[0])
        if source == POS:
            return len(sel.source.series.names(WSK))
        if source == ROV:
            return int(bool(sel.source.rov))
        return len(sel.source.series.names(source))

    @staticmethod
    def _clip(ser: Series, src: str, t0: float, t1: float, sel) -> Optional[tuple[float, float]]:
        """[t0, t1] for a selection (as it is); the timeline's clipped to the source."""
        if sel is not None:
            return t0, t1
        ext = ser.extent(src)
        if ext is None:
            return None
        a, b = max(t0, ext[0]), min(t1, ext[1])
        return (a, b) if b > a else None

    def _curves(self, sel: Optional[Selection], source: SelSource, t0: float, t1: float,
                n: int, zero: float) -> list[Trace]:
        """Every variable over [t0, t1] of ``source``, decimated to ~``n``, time -
        ``zero`` (``sel`` None: the timeline's, clipped to what the source has)."""
        out: list[Trace] = []
        ser = source.series
        gv = []
        for v in self.variables:
            if v.source in _GAUGES:
                cp = self.gauge(sel, v, ser)
                if cp is not None:
                    gv.append((v, cp))
        span = self._clip(ser, WSK, t0, t1, sel)
        if gv and span is not None:
            cols = sorted({c for _, (c, _) in gv})
            t, y = envelope_within(ser, WSK, cols, span[0], span[1], n, self.despike)
            for v, (c, pol) in gv:
                out.append(Trace(sel, v, t - zero,
                                 self._gauge_values(sel, v, c, pol, t, y[:, cols.index(c)])))
        have = len(ser.names(IMU))
        want = [v for v in self.variables if v.source == IMU and v.col < have]
        span = self._clip(ser, IMU, t0, t1, sel)
        if want and span is not None:
            t, y = envelope_within(ser, IMU, [v.col for v in want], span[0], span[1], n)
            for j, v in enumerate(want):
                out.append(Trace(sel, v, t - zero, self._convert(v, y[:, j])))
        for v in self.variables:
            if v.source not in (DEPTH, ROV) or not v.plots:
                continue
            dt, dd = source.aux(v)
            i0, i1 = span_index(dt, t0, t1)
            if i1 <= i0:
                continue
            t, y = peak_decimate(dt[i0:i1], np.asarray(dd[i0:i1], float)[:, None],
                                 max(1, (i1 - i0) // max(1, n)))
            out.append(Trace(sel, v, t - zero, y[:, 0]))
        return out

    def traces(self, n_px: int, view: Optional[tuple[float, float, float]] = None) -> list[Trace]:
        """Compare's curves: time relative to each selection's zero, decimated to
        ~``n_px`` columns over the longest selection (a shorter one gets
        proportionally fewer); at most ``max_traces`` selections (``drawn``).
        ``view`` (lo, hi, w): the waveform zoomed by hand: only [lo, hi] of each
        selection (relative time) at ``n_px`` columns per ``w`` seconds, so a zoom
        gets the detail back (raw samples once there are fewer than columns); a
        selection outside it gets a token 8 columns of the whole."""
        sels = self.drawn()
        longest = max((s.duration for s in sels), default=0.0)
        out: list[Trace] = []
        for sel in sels:
            if sel.duration <= 0:
                continue
            if view is None:
                a, b = sel.t0, sel.t1
                n = max(8, int(round(n_px * sel.duration / longest))) if longest > 0 else n_px
            else:
                lo, hi, w = view
                a, b = max(sel.t0, sel.zero + lo), min(sel.t1, sel.zero + hi)
                if b > a:
                    n = max(8, int(round(n_px * (b - a) / max(w, 1e-9))))
                else:
                    a, b, n = sel.t0, sel.t1, 8
            out += self._curves(sel, sel.source, a, b, n, sel.zero)
        return out

    def samples(self, sel: Selection, var: Variable,
                span: Optional[tuple[float, float]] = None) -> tuple[np.ndarray, np.ndarray]:
        """Every sample of ``var`` in ``sel`` (or in ``span``, absolute; absolute
        time, ``var.unit``); empty if the selection's source has none."""
        t0, t1 = span if span is not None else (sel.t0, sel.t1)
        if var.source == DEPTH:
            dt, dd = sel.source.depth
            i0, i1 = span_index(dt, t0, t1)
            return np.array(dt[i0:i1]), np.asarray(dd[i0:i1], dtype=float)
        if var.source == ROV:
            if not var.plots:
                return np.empty(0), np.empty(0)
            return _aux_samples(*sel.source.rov.series(var.key), t0, t1)
        ser = sel.source.series
        if var.source in _GAUGES:
            cp = self.gauge(sel, var)
            if cp is None:
                return np.empty(0), np.empty(0)
            c, pol = cp
            name = ser.names(WSK)[c]
            t, y = ser.channels([name], t0, t1, self.despike)[name]
            return t, self._gauge_values(sel, var, c, pol, t, y)
        names = ser.names(var.source)
        if var.col >= len(names):
            return np.empty(0), np.empty(0)
        name = names[var.col]
        t, y = ser.channels([name], t0, t1, False)[name]
        return t, self._convert(var, y)

    def aggregate(self, var: Variable, dt: float = AGG_DT) -> list[Aggregate]:
        """Per group (in order of first appearance) the median and 25-75 % band of
        ``var`` over every selection of the group (not only the drawn ones): each
        selection's samples averaged into ``dt`` bins of time relative to its zero
        (an empty bin between samples holds the previous one), on one grid for the
        whole dataset."""
        rel = [(s.t0 - s.zero, s.t1 - s.zero) for s in self.selections if s.duration > 0]
        if not rel:
            return []
        lo = np.floor(min(a for a, _ in rel) / dt) * dt
        nb = int(np.ceil((max(b for _, b in rel) - lo) / dt))
        grid = lo + (np.arange(nb) + 0.5) * dt
        rows: dict[str, list] = {}
        for sel in self.selections:
            if sel.duration <= 0:
                continue
            t, y = self.samples(sel, var)
            row = np.full(nb, np.nan)
            ok = np.isfinite(y)
            if ok.any():
                k = np.clip(((t[ok] - sel.zero - lo) / dt).astype(int), 0, nb - 1)
                cnt = np.bincount(k, minlength=nb)
                tot = np.bincount(k, y[ok], minlength=nb)
                row[cnt > 0] = tot[cnt > 0] / cnt[cnt > 0]
                # a source sparser than the grid (the ROV's ~19 /s): each empty bin within
                # the samples holds the previous value, so every event counts in every bin
                have = np.flatnonzero(cnt > 0)
                if len(have) > 1:
                    seg = slice(have[0], have[-1] + 1)
                    last = np.maximum.accumulate(np.where(cnt[seg] > 0,
                                                          np.arange(seg.stop - seg.start), 0))
                    row[seg] = row[seg][last]
            rows.setdefault(sel.group, []).append(row)
        out = []
        import warnings
        for gname, rs in rows.items():
            m = np.vstack(rs)
            n = np.isfinite(m).sum(axis=0)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                q25, med, q75 = np.nanpercentile(m, [25, 50, 75], axis=0)
            out.append(Aggregate(gname, var, grid, med, q25, q75, n, len(rs)))
        return out

    def _span_samples(self, sel: Selection, var: Variable, n_min: int, sub=None):
        sp = self.span(sel, sub)
        if sp is None:
            return None
        t, y = self.samples(sel, var, sp)
        if len(t) < n_min or t[-1] <= t[0]:
            return None
        return t, y, (len(t) - 1) / (t[-1] - t[0])

    def spectrum(self, sel: Selection, var: Variable, window: str = "hann",
                 detrend: str = "mean", sub: Optional[tuple] = None
                 ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """(f Hz, amplitude in ``var.unit``) of the span (``sub``: a sub-range, see
        ``span``); None if it holds fewer than 8 samples."""
        from wavi_py.daq.spectra import amp_spectra
        got = self._span_samples(sel, var, 8, sub)
        if got is None:
            return None
        t, y, fs = got
        amp = amp_spectra(y[:, None], 1.0, window, detrend)[:, 0]
        return np.fft.rfftfreq(len(t), 1.0 / fs), amp

    def spectrogram(self, sel: Selection, var: Variable, seg_s: float = 1.0,
                    overlap: float = 0.75, sub: Optional[tuple] = None
                    ) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """(segment centres s from the selection's zero, f Hz, amplitude (segments,
        nfreq)) of the span (``sub``: a sub-range); None if it is shorter than one
        segment."""
        from wavi_py.daq.spectra import amp_spectra
        got = self._span_samples(sel, var, 8, sub)
        if got is None:
            return None
        t, y, fs = got
        m = max(8, int(round(seg_s * fs)))
        if len(t) < m:
            return None
        hop = max(1, int(round(m * (1.0 - overlap))))
        segs = np.lib.stride_tricks.sliding_window_view(y, m)[::hop]
        amp = amp_spectra(segs[..., None], 1.0)[..., 0]
        starts = np.arange(len(segs)) * hop
        centres = 0.5 * (t[starts] + t[starts + m - 1]) - sel.zero
        return centres, np.fft.rfftfreq(m, 1.0 / fs), amp

    def overview(self, source: SelSource, t0: float, t1: float, n_px: int) -> list[Trace]:
        """The timeline's curves (``sel`` None): every variable of the dataset over
        [t0, t1] of ``source`` (absolute time, clipped to what it has), min / max
        decimated to ~``n_px`` columns (``Series.envelope``: the store's pyramid, so
        the whole recording costs about what a minute does), in ``var.unit``
        (POS through the dataset's ``dome_map``; no baseline)."""
        return self._curves(None, source, t0, t1, max(8, n_px), 0.0)


def spectrogram_column(sg: Optional[tuple], t: float) -> Optional[tuple[np.ndarray, np.ndarray, float]]:
    """One segment of a ``Dataset.spectrogram`` result ``sg``: (f Hz, amplitude, the
    segment's centre) of the segment whose centre is nearest ``t`` (seconds from the
    selection's start); None when ``t`` is outside the segments (more than half a
    hop past the first or last centre) or there is no spectrogram."""
    if sg is None:
        return None
    tc, f, amp = sg
    if not len(tc):
        return None
    half = 0.5 * (tc[1] - tc[0]) if len(tc) > 1 else np.inf
    if t < tc[0] - half or t > tc[-1] + half:
        return None
    i = int(np.argmin(np.abs(tc - t)))
    return f, amp[i], float(tc[i])
