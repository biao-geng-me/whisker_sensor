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
root to the tip at each cursor (``(number, t)``, t relative to the selection's
zero) inside the span; without cursors, one per path to its farthest point (the
peak bend). ``scale`` is the largest bend drawn (the ``SCALE_PCT`` percentile of each path, so one spike does not shrink the rest), in degrees (the map's common
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

    def __bool__(self) -> bool:
        return bool(self.paths)


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


def plane_overlay(ds: Dataset, positions: Sequence[str], cursors: Sequence = (),
                  n_max: int = N_MAX) -> PlaneOverlay:
    """The paths (and the arrows at ``cursors``, see ``set_arrows``)."""
    out = PlaneOverlay(relative=not ds.baseline and ds.v0 is None)
    scale = 1.0 / ds.scale if ds.scale else 1.0
    big = 0.0
    for p in positions:
        vd, vl = position_variables([p])
        for sel in ds.selections:
            m = deg_per_mv(ds.map_of(sel), p) * scale
            origin = np.zeros(2)
            if out.relative:
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
                out.paths.setdefault(p, []).append(PlanePath(sel, n, deg[::k], t, deg, span))
                big = max(big, float(np.percentile(np.hypot(deg[:, 0], deg[:, 1]), SCALE_PCT)))
    out.scale = big
    set_arrows(out, cursors)
    return out


def set_arrows(ov: PlaneOverlay, cursors: Sequence = ()) -> None:
    """``ov.arrows`` for these cursors ``(number, t)`` (cheap: the paths are kept,
    so a cursor drag re-reads nothing)."""
    ov.arrows = {}
    for p, paths in ov.paths.items():
        arrows = ov.arrows.setdefault(p, [])
        for path in paths:
            t, deg = path.t, path.deg
            if cursors:
                for cn, ct in cursors:
                    at = path.sel.zero + float(ct)
                    if path.span[0] <= at <= path.span[1] and len(t):
                        i = int(np.clip(np.searchsorted(t, at), 0, len(t) - 1))
                        arrows.append(PlaneArrow(path.sel, path.sub, str(cn), tuple(deg[i])))
            elif len(deg):
                r = np.hypot(deg[:, 0], deg[:, 1])
                arrows.append(PlaneArrow(path.sel, path.sub, "", tuple(deg[int(np.argmax(r))])))
