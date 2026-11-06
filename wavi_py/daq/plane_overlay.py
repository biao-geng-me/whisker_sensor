"""The Analysis window's plane map: each dome position's tip path over the
selections, as the Dome view's plane map draws a whisker (no Qt).

``plane_overlay(ds, positions, cursors)`` reads, for every selection of ``ds``
and each of its sub-ranges (``Dataset.sub_list``: the whole selection when there
are none), the drag and lift of every position (``pos:<p>:drag|lift``, through
the selection's dome map, polarity applied: drag + = tip toward the front, lift
+ = tip up) and turns them into degrees of bend with the position's sensor
calibration (``deg_per_mv``, else the placeholder gain ``dome_pose.DEG_PER_MV``),
the Dome view's step 4 (a position with one dead axis draws the other alone,
0 on the dead one). The display scale of the waveform is taken back out, so
the map reads the same at any scale.

The origin (no bend) is 0 when the values are already relative to a rest (V0
set, or an event's baseline); otherwise the median of the selection's first
``ORIGIN_S`` (so a selection starting at rest draws from the root, as the Dome
view's tare does).

Each path is thinned to at most ``n_max`` points (every k-th sample: drag and
lift stay paired, which a min / max envelope would not). The arrows go from the
root to the tip at each cursor (``(number, t)``, t relative to the
selection's zero) inside the span (Events: also at the scrub head, label
``HEAD``); with neither, one per path to its farthest point (the peak bend).
``heads``: every selection at the plane head (the Analysis window's teal line
in the waveform, relative time), read wherever the head is, with its trail. ``scale`` is the largest bend drawn (the ``SCALE_PCT`` percentile of each path, so one spike does not shrink the rest), in degrees (the map's common
scale: all cells the same, so they compare).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from wavi_py.daq.analysis import Dataset, Selection, position_variables

ORIGIN_S = 0.25                 # the origin without V0 / baseline: median of the first this
N_MAX = 300
TRAIL_S = 1.0                   # the head's trail: the last this many seconds before it
TRAIL_N = 60                    # ... drawn with at most this many points
HEAD = "head"                   # the scrub head's arrow label (drawn as the main arrow, unlabelled)
SCALE_PCT = 99.5            # the scale: this percentile of each path's bend (a spike is clipped)


@dataclass
class PlanePath:
    sel: Selection
    sub: Optional[int]          # the sub-range's number, None: the whole selection
    xy: np.ndarray              # (N, 2) deg: (drag, lift), thinned to draw
    t: np.ndarray = field(default_factory=lambda: np.empty(0), repr=False)     # every sample
    deg: np.ndarray = field(default_factory=lambda: np.empty((0, 2)), repr=False)
    span: tuple = (0.0, 0.0)    # absolute


@dataclass
class PlaneArrow:
    sel: Selection
    sub: Optional[int]
    label: str                  # the cursor's number, "" for the peak
    xy: tuple                   # (drag, lift) deg


@dataclass
class PlaneOverlay:
    paths: dict = field(default_factory=dict)     # position -> [PlanePath]
    arrows: dict = field(default_factory=dict)    # position -> [PlaneArrow]
    scale: float = 0.0                            # deg: the largest bend drawn
    relative: bool = False                        # origin from each selection's start
    head: dict = field(default_factory=dict)      # position -> [PlaneArrow] at the head, per selection
    trail: dict = field(default_factory=dict)     # position -> [(sel, (N, 2) deg)], the TRAIL_S before

    def __bool__(self) -> bool:
        return bool(self.paths) or bool(self.head)


@dataclass
class HeadState:
    """The selections at the plane head: per position each one's tip and trail
    (``heads``)."""
    head: dict = field(default_factory=dict)
    trail: dict = field(default_factory=dict)
    big: float = 0.0                              # deg: the largest bend in it


def deg_per_mv(dome_map, position: str) -> np.ndarray:
    """(2, 2) (drag, lift) mV after polarity -> deg of the position's sensor."""
    from whisker_link.dome_pose import DEG_PER_MV
    p = dome_map.position(position) if dome_map is not None else None
    cal = dome_map.sensor_cal(p) if p is not None else None
    return np.asarray(cal.deg_per_mv, dtype=float) if cal is not None else np.eye(2) * DEG_PER_MV


def _pair(ds: Dataset, sel: Selection, vd, vl, span) -> Optional[tuple[np.ndarray, np.ndarray]]:
    td, yd = ds.samples(sel, vd, span)
    tl, yl = ds.samples(sel, vl, span)
    if not len(td) and not len(tl):
        return None
    if not len(td) or not len(tl):              # one axis dead: the other alone, 0 on it
        t = td if len(td) else tl
        z = np.zeros(len(t))
        return t, np.c_[yd if len(td) else z, yl if len(tl) else z]
    if len(tl) != len(td) or not np.array_equal(tl, td):
        yl = np.interp(td, tl, yl)
    return td, np.c_[yd, yl]


def _sel_key(ds: Dataset, sel: Selection, positions) -> tuple:
    v0 = None if ds.v0 is None else id(ds.v0)
    return (id(sel), sel.t0, sel.t1, sel.align, id(sel.source), id(ds.map_of(sel)),
            tuple(positions), tuple(ds.subs), ds.sub, ds.despike, v0, ds.scale, ds.baseline)


def plane_overlay(ds: Dataset, positions: Sequence[str], cursors: Sequence = (),
                  n_max: int = N_MAX, head: Optional[float] = None,
                  cache: Optional[dict] = None, peaks: bool = True) -> PlaneOverlay:
    """The paths (and the arrows at ``cursors`` and ``head``, see ``set_arrows``).
    ``cache`` (a dict kept by the caller): each selection's paths by its inputs, so
    an edge drag re-reads the dragged selection only (the dict is pruned to the
    selections here)."""
    out = PlaneOverlay(relative=not ds.baseline and ds.v0 is None)
    keep = {}
    big = 0.0
    for sel in ds.selections:
        k = _sel_key(ds, sel, positions)
        got = cache.get(k) if cache is not None else None
        if got is None:
            got = _sel_paths(ds, sel, positions, out.relative, n_max)
        keep[k] = got
        for p, paths in got[0].items():
            out.paths.setdefault(p, []).extend(paths)
        big = max(big, got[1])
    if cache is not None:
        cache.clear()
        cache.update(keep)
    out.scale = big
    set_arrows(out, cursors, head, peaks)
    return out


def _sel_paths(ds: Dataset, sel: Selection, positions, relative: bool, n_max: int) -> tuple:
    """({position: [PlanePath]}, the largest bend) of one selection."""
    scale = 1.0 / ds.scale if ds.scale else 1.0
    big = 0.0
    paths: dict = {}
    for p in positions:
        vd, vl = position_variables([p])
        for sel in (sel,):
            m = deg_per_mv(ds.map_of(sel), p) * scale
            origin = np.zeros(2)
            if relative:
                got = _pair(ds, sel, vd, vl, (sel.t0, min(sel.t1, sel.t0 + ORIGIN_S)))
                if got is None:
                    continue
                origin = np.nanmedian(got[1], axis=0)
            for sub in ds.sub_list():
                span = ds.span(sel, sub)
                if span is None:
                    continue
                got = _pair(ds, sel, vd, vl, span)
                if got is None:
                    continue
                t, v = got
                deg = (v - origin) @ m.T
                ok = np.isfinite(deg).all(axis=1)
                t, deg = t[ok], deg[ok]
                if not len(deg):
                    continue
                n = sub[0] if sub is not None else None
                k = max(1, math.ceil(len(deg) / n_max))
                paths.setdefault(p, []).append(PlanePath(sel, n, deg[::k], t, deg, span))
                big = max(big, float(np.percentile(np.hypot(deg[:, 0], deg[:, 1]), SCALE_PCT)))
    return paths, big


def set_arrows(ov: PlaneOverlay, cursors: Sequence = (), head: Optional[float] = None,
               peaks: bool = True) -> None:
    """``ov.arrows`` for these cursors ``(number, t)`` and the scrub ``head`` (t
    relative to each selection's zero, as the cursors; its arrow has label
    ``HEAD``) (cheap: the paths are kept, so a drag re-reads nothing). Without
    either (and ``peaks``), one arrow per path at its peak."""
    ov.arrows = {}
    marks = [(str(cn), float(ct)) for cn, ct in cursors]
    if head is not None:
        marks.append((HEAD, float(head)))
    for p, paths in ov.paths.items():
        arrows = ov.arrows.setdefault(p, [])
        for path in paths:
            t, deg = path.t, path.deg
            if marks:
                for label, ct in marks:
                    at = path.sel.zero + ct
                    if path.span[0] <= at <= path.span[1] and len(t):
                        i = int(np.clip(np.searchsorted(t, at), 0, len(t) - 1))
                        arrows.append(PlaneArrow(path.sel, path.sub, label, tuple(deg[i])))
            elif peaks and len(deg):
                r = np.hypot(deg[:, 0], deg[:, 1])
                arrows.append(PlaneArrow(path.sel, path.sub, "", tuple(deg[int(np.argmax(r))])))


def _head_key(ds: Dataset, sel: Selection) -> tuple:
    """What a selection's head reads depends on (not its end: an end drag keeps it)."""
    v0 = None if ds.v0 is None else id(ds.v0)
    return (id(sel), sel.t0, sel.align, id(sel.source), id(ds.map_of(sel)), ds.despike, v0,
            ds.scale, ds.baseline)


def heads(ds: Dataset, positions: Sequence[str], t: float, cache: Optional[dict] = None,
          trail_s: float = TRAIL_S) -> HeadState:
    """Every selection of ``ds`` at the plane head ``t`` (relative to its zero, as the
    cursors): per position one ``PlaneArrow`` (label ``HEAD``) per selection and its
    trail over the ``trail_s`` before. The recording is read there whether or not
    it is inside the selection's span, with the origin of the selection's paths
    (its first ``ORIGIN_S`` without V0 / baseline, else 0), so where the head is
    inside the span its arrow ends on the path (at its last sample at or before
    the head). ``cache``: a dict the caller keeps, each selection's head by its
    inputs, so an edge drag re-reads none (pruned to the selections here)."""
    keys = [(_head_key(ds, sel), tuple(positions), float(t), trail_s) for sel in ds.selections]
    if cache is not None and cache.get(HEAD, (None,))[0] == keys:
        return cache[HEAD][1]                     # nothing changed: the same state (drawn once)
    out = HeadState()
    keep = {}
    for sel, k in zip(ds.selections, keys):
        got = cache.get(k) if cache is not None else None
        if got is None:
            got = _sel_head(ds, sel, positions, float(t), trail_s)
        keep[k] = got
        for p, (arrow, trail) in got.items():
            out.head.setdefault(p, []).append(arrow)
            out.trail.setdefault(p, []).append((sel, trail))
            out.big = max(out.big, float(np.hypot(*arrow.xy)))
    if cache is not None:
        cache.clear()
        cache.update(keep)
        cache[HEAD] = (keys, out)
    return out


def _sel_head(ds: Dataset, sel: Selection, positions, t: float, trail_s: float) -> dict:
    """{position: (its PlaneArrow at the head, its trail)} of one selection."""
    from dataclasses import replace
    relative = not ds.baseline and ds.v0 is None
    scale = 1.0 / ds.scale if ds.scale else 1.0
    at = sel.zero + t
    win = replace(sel, t0=at - trail_s, t1=at)
    out = {}
    for p in positions:
        vd, vl = position_variables([p])
        o = np.zeros(2)
        if relative:
            got = _pair(ds, sel, vd, vl, (sel.t0, min(sel.t1, sel.t0 + ORIGIN_S)))
            if got is None:
                continue
            o = np.nanmedian(got[1], axis=0)
        got = _pair(ds, win, vd, vl, (at - trail_s, at + 1e-6))
        if got is None:
            continue
        deg = (got[1] - o) @ (deg_per_mv(ds.map_of(sel), p) * scale).T
        deg = deg[np.isfinite(deg).all(axis=1)]
        if len(deg):
            k = max(1, math.ceil(len(deg) / TRAIL_N))     # thinned to draw, the head kept
            out[p] = (PlaneArrow(sel, None, HEAD, tuple(deg[-1])), deg[::-1][::k][::-1])
    return out
