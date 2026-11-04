"""Selections and variables for the Analysis windows: the data side (no Qt).

A ``Selection`` is a time window [t0, t1] on the session axis plus the data it
reads (``SelSource``): the whole-recording store in replay, a copy of the live
buffers taken when live was paused (so it keeps its data after the buffers move
on; adjusting it is limited to what was copied). The selection model and its
drawing are in ``views/selections.py``.

A ``Variable`` is one curve's worth of data by key: ``wsk:<col>`` (a gauge, the
series' stream-order column), ``imu:<col>`` (``IMU_CHANNELS``) or ``depth`` (the
source's depth from the external pressure, which is not in the series). Each has
a unit group, the rows of Compare (and of C2's Spectrum): mV, mg, mrad/s, deg,
cm, as the IMU window shows them (accel and gyro as the series has them, the
attitude's rad in degrees). The gauges are
drawn as the Line window draws them, (raw - V0) * scale, so the curves line up
with what was selected.

``Dataset`` is what a tab reads: (included selections) x (checked variables) of
one Analysis window, with the panel's Despike / V0 / scale. ``traces(n)`` is
Compare's: the time relative to each selection's start, min / max decimated to
``n`` pixel columns over the longest selection (``Series.envelope``: one call per
source per selection, from the store's pyramid in replay), so a long selection
costs about what a short one does. ``samples(sel, var)`` is every sample.

The spectra read the dataset's ``span(sel)``: the whole selection, or with a
Compare sub-range ``sub = (a, b)`` (seconds from each selection's start)
[t0 + a, t0 + b] clipped to the selection, None when it is shorter than a.
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
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from wavi_py.daq.series import IMU, WSK, ArraySeries, Series, peak_decimate, span_index

DEPTH = "depth"
UNIT_GROUPS = ("mV", "mg", "mrad/s", "deg", "cm")      # Compare's rows, top to bottom
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
    """What a selection reads: a ``Series`` and the depth ``(t, cm)`` (may be empty).
    ``snapshot``: a copy of the live buffers (one version: Despike does not apply)."""
    series: Series
    depth: tuple = (np.empty(0), np.empty(0))
    snapshot: bool = False

    def extent(self) -> Optional[tuple[float, float]]:
        """The span the selection can cover: whisker and IMU data together."""
        ex = [e for e in (self.series.extent(WSK), self.series.extent(IMU)) if e is not None]
        if not ex:
            return None
        return min(e[0] for e in ex), max(e[1] for e in ex)


def snapshot_of(series: Series, depth=None) -> SelSource:
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
    return SelSource(ArraySeries(data, names), d, snapshot=True)


@dataclass(eq=False)
class Selection:
    """One time window (see the module doc). ``number`` labels it everywhere and
    picks its colour; ``bounds`` is the source's extent (edits are clamped to it)."""
    number: int
    t0: float
    t1: float
    source: SelSource
    bounds: Optional[tuple[float, float]] = None

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    def clamp(self, t0: float, t1: float) -> tuple[float, float]:
        if t1 < t0:
            t0, t1 = t1, t0
        if self.bounds is not None:
            lo, hi = self.bounds
            t0, t1 = min(max(t0, lo), hi), min(max(t1, lo), hi)
        return float(t0), float(t1)


@dataclass(frozen=True)
class Variable:
    key: str                 # wsk:<col>, imu:<col>, depth
    label: str               # "W3 A", "Accel x", "Depth"
    unit: str                # one of UNIT_GROUPS
    source: str              # WSK, IMU or DEPTH
    col: int = 0
    factor: float = 1.0
    group: str = ""          # the tree's parent: the sensor name, "Accel", ...


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
    t: np.ndarray            # relative: seconds from the selection's start (timeline: absolute)
    y: np.ndarray            # in var.unit


@dataclass
class Dataset:
    """(selections) x (variables) with the display context (see the module doc)."""
    selections: list
    variables: list
    despike: bool = False
    v0: Optional[np.ndarray] = None     # per gauge column, raw mV
    scale: float = 1.0
    sub: Optional[tuple] = None         # the Compare sub-range (a, b), relative
    _names: dict = field(default_factory=dict, repr=False)

    def span(self, sel: Selection) -> Optional[tuple[float, float]]:
        """The part of ``sel`` the spectra use (see the module doc)."""
        if self.sub is None:
            return sel.t0, sel.t1
        a, b = self.sub
        lo, hi = sel.t0 + max(0.0, a), min(sel.t1, sel.t0 + b)
        return (lo, hi) if hi > lo else None

    def pairs(self) -> list[tuple[Selection, Variable]]:
        return [(s, v) for s in self.selections for v in self.variables]

    def _convert(self, var: Variable, y: np.ndarray) -> np.ndarray:
        if var.source == WSK:
            v0 = 0.0 if self.v0 is None or var.col >= len(self.v0) else float(self.v0[var.col])
            return (np.asarray(y, dtype=float) - v0) * self.scale
        return np.asarray(y, dtype=float) * var.factor

    def _have(self, sel: Selection, source: str) -> int:
        """Columns ``source`` has in this selection's series (0: none)."""
        if source == DEPTH:
            return len(sel.source.depth[0])
        return len(sel.source.series.names(source))

    def traces(self, n_px: int) -> list[Trace]:
        """Compare's curves: relative time, decimated to ~``n_px`` columns over the
        longest selection (a shorter one gets proportionally fewer)."""
        longest = max((s.duration for s in self.selections), default=0.0)
        out: list[Trace] = []
        for sel in self.selections:
            if sel.duration <= 0:
                continue
            n = max(8, int(round(n_px * sel.duration / longest))) if longest > 0 else n_px
            ser = sel.source.series
            for src in (WSK, IMU):
                want = [v for v in self.variables if v.source == src
                        and v.col < self._have(sel, src)]
                if not want:
                    continue
                t, y = envelope_within(ser, src, [v.col for v in want], sel.t0, sel.t1, n,
                                       self.despike and src == WSK)
                t = t - sel.t0
                for j, v in enumerate(want):
                    out.append(Trace(sel, v, t, self._convert(v, y[:, j])))
            for v in self.variables:
                if v.source == DEPTH and self._have(sel, DEPTH):
                    dt, dd = sel.source.depth
                    i0, i1 = span_index(dt, sel.t0, sel.t1)
                    t, y = peak_decimate(dt[i0:i1], np.asarray(dd[i0:i1], float)[:, None],
                                         max(1, (i1 - i0) // max(1, n)))
                    out.append(Trace(sel, v, t - sel.t0, y[:, 0]))
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
        ser = sel.source.series
        names = ser.names(var.source)
        if var.col >= len(names):
            return np.empty(0), np.empty(0)
        name = names[var.col]
        t, y = ser.channels([name], t0, t1, self.despike and var.source == WSK)[name]
        return t, self._convert(var, y)

    def _span_samples(self, sel: Selection, var: Variable, n_min: int):
        sp = self.span(sel)
        if sp is None:
            return None
        t, y = self.samples(sel, var, sp)
        if len(t) < n_min or t[-1] <= t[0]:
            return None
        return t, y, (len(t) - 1) / (t[-1] - t[0])

    def spectrum(self, sel: Selection, var: Variable, window: str = "hann",
                 detrend: str = "mean") -> Optional[tuple[np.ndarray, np.ndarray]]:
        """(f Hz, amplitude in ``var.unit``) of the span (see the module doc); None if
        it holds fewer than 8 samples."""
        from wavi_py.daq.spectra import amp_spectra
        got = self._span_samples(sel, var, 8)
        if got is None:
            return None
        t, y, fs = got
        amp = amp_spectra(y[:, None], 1.0, window, detrend)[:, 0]
        return np.fft.rfftfreq(len(t), 1.0 / fs), amp

    def spectrogram(self, sel: Selection, var: Variable, seg_s: float = 1.0,
                    overlap: float = 0.75) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """(segment centres s from the selection's start, f Hz, amplitude (segments,
        nfreq)); None if the span is shorter than one segment."""
        from wavi_py.daq.spectra import amp_spectra
        got = self._span_samples(sel, var, 8)
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
        centres = 0.5 * (t[starts] + t[starts + m - 1]) - sel.t0
        return centres, np.fft.rfftfreq(m, 1.0 / fs), amp

    def overview(self, source: SelSource, t0: float, t1: float, n_px: int) -> list[Trace]:
        """The timeline's curves (``sel`` None): every variable of the dataset over
        [t0, t1] of ``source`` (absolute time, clipped to what it has), min / max
        decimated to ~``n_px`` columns (``Series.envelope``: the store's pyramid, so
        the whole recording costs about what a minute does), in ``var.unit``."""
        out: list[Trace] = []
        ser = source.series
        for src in (WSK, IMU):
            have = len(ser.names(src))
            want = [v for v in self.variables if v.source == src and v.col < have]
            ext = ser.extent(src)
            if not want or ext is None:
                continue
            a, b = max(t0, ext[0]), min(t1, ext[1])
            if b <= a:
                continue
            t, y = ser.envelope(src, [v.col for v in want], a, b, max(8, n_px),
                                self.despike and src == WSK)
            for j, v in enumerate(want):
                out.append(Trace(None, v, t, self._convert(v, y[:, j])))
        dt, dd = source.depth
        if len(dt) and any(v.source == DEPTH for v in self.variables):
            i0, i1 = span_index(dt, t0, t1)
            if i1 > i0:
                t, y = peak_decimate(dt[i0:i1], np.asarray(dd[i0:i1], float)[:, None],
                                     max(1, (i1 - i0) // max(1, n_px)))
                v = next(v for v in self.variables if v.source == DEPTH)
                out.append(Trace(None, v, t, y[:, 0]))
        return out


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
