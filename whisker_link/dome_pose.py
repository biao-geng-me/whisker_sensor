"""Whisker pose on the dome from gauge mV: mapping, tare, calibration, shape.

Qt-free numpy, vectorised over the whiskers, so the Dome view (wavi_py), a
script or a later web front end can share it. The pipeline per block of scans:

    mV (N, 32) --Tare--> delta mV --Smoother--> smoothed delta (32,)
      --Calibration (2x2 per whisker)--> (theta_x, theta_y) per hole, rad
      --whisker_shape--> polyline (W, S, 3) mm on the dome

PLACEHOLDERS, isolated so each can be swapped alone:

* ``PLACEHOLDER_HOLE_OF``: which live whisker channel sits in which dome hole.
  The 12 live channels in order onto holes 0..11 in the asset's build order
  (left module rows 0..2 x cols 0..1, then right). Not the real wiring.
* ``Calibration.nominal``: a diagonal ``DEG_PER_MV`` from (dA, dB) mV to
  (theta_x, theta_y); gauge A bends toward e1, B toward e2. Not measured.
* ``whisker_shape``: a rigid shaft on a constant-curvature compliant root of
  length ``LR_MM`` (data_processing/bend_angle.m), the bend angle being the
  angle at the root's top. No load, no shaft deflection, no undulation.

Display aid, not physics: the scribe ``Shell`` (a sphere on the dome's
hemisphere + a cylinder down the tube axis) and the centre-pivot mode, in which
each whisker turns about the dome centre (or, on the cylinder part, the axis
point at its height) so that its tip moves on the shell; on the sphere part a
bend of phi moves the tip phi of arc, i.e. ``radius * phi`` mm. ``TrailBuffer``
keeps the tips' recent path for the trails; ``neutral_tips`` / ``tip_path`` /
``arrow_segments`` give each whisker's zero-bend mark and the arrow from it
to the current tip.

The dome geometry and each hole's bending frame come from
``wavi_py/assets/dome/whiskers.json`` (wavi_py/tools/export_dome_assets.py):
``axis`` n outward, ``e1`` meridional (toward the dome's front, +z), ``e2`` =
n x e1. theta_x tilts the tip toward +e1, theta_y toward +e2.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.signal import lfilter

from whisker_link.channels import CHANNEL_NAMES, NCH

# The gauges wired now (select 6 skipped: W3, W8, W10, W14 read railed).
LIVE_WHISKERS = (1, 2, 4, 5, 6, 7, 9, 11, 12, 13, 15, 16)

# PLACEHOLDER: whisker number -> dome hole (index into whiskers.json). Replace
# with the real wiring table; nothing else depends on the order.
PLACEHOLDER_HOLE_OF = {w: i for i, w in enumerate(LIVE_WHISKERS)}

# PLACEHOLDER calibration gain: 1 deg per mV, so a 10-15 mV hand bend shows as
# 10-15 deg before the display gain.
DEG_PER_MV = 1.0

# bend model (data_processing/bend_angle.m): compliant PDMS root length
LR_MM = 11.5

# a gauge reading at or above this (or NaN) is railed: nothing plugged in
RAIL_MV = 0.9 * 8_380_000 * (2.5 * 1000.0 / 2 ** 23)


def channels_of(whiskers) -> np.ndarray:
    """(W, 2) channel indices (gauge A, gauge B) for whisker numbers 1..16."""
    return np.array([[CHANNEL_NAMES.index(f"W{w}_A"), CHANNEL_NAMES.index(f"W{w}_B")]
                     for w in whiskers], dtype=np.intp)


# ---------------------------------------------------------------------------
# dome layout
# ---------------------------------------------------------------------------
@dataclass
class DomeLayout:
    """The holes of the dome, (H, 3) arrays in mm / unit vectors."""

    root: np.ndarray
    axis: np.ndarray
    e1: np.ndarray
    e2: np.ndarray
    length: np.ndarray
    labels: list = field(default_factory=list)
    # the hemisphere's centre (on the tube axis +z, at the equator) and radius
    center: Optional[np.ndarray] = None
    dome_radius: float = 0.0

    def __post_init__(self) -> None:
        if self.center is None:             # an older asset: the point nearest all axes
            m = np.zeros((3, 3))
            b = np.zeros(3)
            for r, a in zip(self.root, self.axis):
                pr = np.eye(3) - np.outer(a, a)
                m += pr
                b += pr @ r
            c = np.linalg.lstsq(m, b, rcond=None)[0]
            self.center = np.array([0.0, 0.0, c[2]])
        self.center = np.asarray(self.center, dtype=np.float64)

    @classmethod
    def load(cls, path) -> "DomeLayout":
        doc = json.loads(Path(path).read_text())
        ws = doc["whiskers"]
        arr = lambda k: np.array([w[k] for w in ws], dtype=np.float64)    # noqa: E731
        dome = doc.get("dome") or {}
        return cls(arr("root"), arr("axis"), arr("e1"), arr("e2"), arr("length"),
                   [f"{w['side']}{w['row']}{w['col']}" for w in ws],
                   dome.get("center"), float(dome.get("radius", 0.0)))

    def __len__(self) -> int:
        return len(self.root)


# ---------------------------------------------------------------------------
# tare + smoothing
# ---------------------------------------------------------------------------
class Tare:
    """Per-channel baseline. ``request()`` (the Tare button, and automatically
    at the start) averages the next ``n_avg`` scans into the baseline. With
    ``drift_tau_s`` > 0 the baseline then follows the signal with that time
    constant (slow drift removal; a held bend fades out over that time too)."""

    def __init__(self, nch: int = NCH, n_avg: int = 32, drift_tau_s: float = 0.0):
        self.nch = nch
        self.n_avg = int(n_avg)
        self.drift_tau_s = float(drift_tau_s)
        self.baseline = np.full(nch, np.nan)
        self._acc = np.zeros(nch)
        self._cnt = np.zeros(nch)
        self._pending = True

    @property
    def ready(self) -> bool:
        return not self._pending

    def request(self) -> None:
        self._acc[:] = 0
        self._cnt[:] = 0
        self._pending = True

    def update(self, mv: np.ndarray, fs: float) -> np.ndarray:
        """Feed a block (N, nch) of mV; returns the deltas (N, nch), NaN while
        a tare is collecting and for railed channels."""
        mv = np.asarray(mv, dtype=np.float64)
        good = np.isfinite(mv) & (np.abs(mv) < RAIL_MV)
        mv = np.where(good, mv, np.nan)
        if self._pending:
            self._acc += np.nansum(mv, axis=0)
            self._cnt += good.sum(axis=0)
            if self._cnt.max(initial=0) >= self.n_avg:
                with np.errstate(invalid="ignore", divide="ignore"):
                    self.baseline = np.where(self._cnt > 0, self._acc / self._cnt, np.nan)
                self._pending = False
            return np.full_like(mv, np.nan)
        delta = mv - self.baseline
        if self.drift_tau_s > 0 and len(mv):
            # baseline <- EMA of the block mean; exact for a constant block
            a = 1.0 - math.exp(-len(mv) / (fs * self.drift_tau_s))
            m = np.nanmean(mv, axis=0) if np.isfinite(mv).any() else self.baseline
            self.baseline = np.where(np.isfinite(m), self.baseline + a * (m - self.baseline),
                                     self.baseline)
        return delta


class Smoother:
    """First-order low-pass per channel (time constant ``tau_s``), sample by
    sample over each block via scipy's lfilter; keeps the latest value."""

    def __init__(self, nch: int = NCH, tau_s: float = 0.03):
        self.nch = nch
        self.tau_s = float(tau_s)
        self.value = np.full(nch, np.nan)

    def reset(self) -> None:
        self.value[:] = np.nan

    def update(self, x: np.ndarray, fs: float) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        if len(x) == 0:
            return self.value
        # NaN (tare collecting, railed) holds the previous value; a channel's
        # first value starts the filter where it is, not from 0
        first = x[np.isfinite(x).argmax(axis=0), np.arange(self.nch)]
        start = np.where(np.isfinite(self.value), self.value, first)
        xf = _ffill(x, start)
        if self.tau_s <= 0:
            self.value = xf[-1]
            return self.value
        a = math.exp(-1.0 / (fs * self.tau_s))
        y, _ = lfilter([1 - a], [1, -a], np.nan_to_num(xf), axis=0,
                       zi=(a * np.nan_to_num(start))[None, :])
        self.value = np.where(np.isfinite(start), y[-1], np.nan)
        return self.value


def _ffill(x: np.ndarray, start: np.ndarray) -> np.ndarray:
    """Forward-fill NaN down axis 0, seeded with ``start``."""
    x = np.vstack([start[None, :], x])
    idx = np.where(np.isfinite(x), np.arange(len(x))[:, None], 0)
    np.maximum.accumulate(idx, axis=0, out=idx)
    return x[idx, np.arange(x.shape[1])][1:]


# ---------------------------------------------------------------------------
# calibration
# ---------------------------------------------------------------------------
@dataclass
class Calibration:
    """(dA, dB) mV -> (theta_x, theta_y) rad per hole: ``theta = M @ d``.
    ``matrix`` is (H, 2, 2) in rad / mV."""

    matrix: np.ndarray

    @classmethod
    def nominal(cls, n: int, deg_per_mv: float = DEG_PER_MV) -> "Calibration":
        """PLACEHOLDER: the same diagonal gain on every whisker."""
        return cls(np.tile(np.eye(2) * math.radians(deg_per_mv), (n, 1, 1)))

    def apply(self, d_ab: np.ndarray) -> np.ndarray:
        """d_ab (H, 2) mV -> (H, 2) rad; NaN rows stay NaN."""
        return np.einsum("hij,hj->hi", self.matrix, d_ab)


# ---------------------------------------------------------------------------
# shape
# ---------------------------------------------------------------------------
def whisker_shape(layout: DomeLayout, theta: np.ndarray, n_root: int = 6, n_shaft: int = 6,
                  lr: float = LR_MM) -> np.ndarray:
    """Polyline of every whisker, (H, n_root + n_shaft + 1, 3) mm.

    ``theta`` (H, 2) rad in each hole's (e1, e2) frame. The root (length
    ``lr``) bends with constant curvature through phi = |theta| toward
    d = theta / phi; the rest of the whisker is a straight shaft along the
    root's end tangent. Zero (or NaN) theta is straight along the axis.
    """
    th = np.nan_to_num(np.asarray(theta, dtype=np.float64))
    phi = np.hypot(th[:, 0], th[:, 1])                                  # (H,)
    safe = np.where(phi > 1e-9, phi, 1.0)
    d = np.where((phi > 1e-9)[:, None],
                 (th[:, :1] * layout.e1 + th[:, 1:] * layout.e2) / safe[:, None],
                 layout.e1)                                             # (H, 3)
    n = layout.axis
    # root: s in [0, lr]; in-plane (along d, along n) coordinates of the arc
    s = np.linspace(0.0, lr, n_root + 1)                                # (Sr,)
    a = phi[:, None] * s[None, :] / lr                                  # angle at s
    small = phi[:, None] < 1e-6
    k = np.where(small, 1.0, lr / safe[:, None])                        # radius
    u = np.where(small, s * a / 2, k * (1 - np.cos(a)))                 # along d
    w = np.where(small, s + 0 * a, k * np.sin(a))                       # along n
    # shaft from the root end, along its tangent
    ls = np.maximum(layout.length - lr, 0.0)                            # (H,)
    t = np.linspace(0.0, 1.0, n_shaft + 1)[1:]                          # (Ss,)
    u = np.concatenate([u, u[:, -1:] + (ls[:, None] * t) * np.sin(phi)[:, None]], axis=1)
    w = np.concatenate([w, w[:, -1:] + (ls[:, None] * t) * np.cos(phi)[:, None]], axis=1)
    return (layout.root[:, None, :] + u[..., None] * d[:, None, :] + w[..., None] * n[:, None, :])


# ---------------------------------------------------------------------------
# scribe shell: a sphere on the dome's hemisphere + a cylinder down the tube
# ---------------------------------------------------------------------------
@dataclass
class Shell:
    """An imaginary drawing surface concentric with the dome: the sphere of
    ``radius`` round ``center`` above the equator (z >= center z) and the
    cylinder of the same radius round the tube axis (+z through ``center``)
    below it, down to ``cyl_len`` under the equator. Whisker tips scribe on it."""

    center: np.ndarray
    radius: float
    cyl_len: float = 0.0

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=np.float64)
        if self.cyl_len <= 0:
            self.cyl_len = 1.0 * self.radius
        self.base_cyl_len = self.cyl_len

    def cover(self, p: np.ndarray, step: float = 0.0) -> bool:
        """Lengthen the cylinder (in ``step`` mm increments, default R / 4) so
        it reaches down past points ``p`` (..., 3), up to the 10 R of ``hit``;
        True when it grew (redraw the grid)."""
        z = np.asarray(p)[..., 2]
        z = z[np.isfinite(z)]
        if not len(z):
            return False
        need = min(self.center[2] - float(z.min()), 10 * self.radius)
        if need <= self.cyl_len:
            return False
        step = step or self.radius / 4
        self.cyl_len = math.ceil(need / step) * step
        return True

    def uncover(self) -> bool:
        """Back to the initial cylinder length; True if that changed it."""
        grown = self.cyl_len != self.base_cyl_len
        self.cyl_len = self.base_cyl_len
        return grown

    @classmethod
    def around(cls, layout: DomeLayout, radius: float = 0.0) -> "Shell":
        """``radius`` 0: the rest tips' reach, mean root distance from its
        pivot + mean whisker length."""
        if radius <= 0:
            piv = centre_pivots(layout)
            radius = float(np.mean(np.linalg.norm(layout.root - piv, axis=1))
                           + np.mean(layout.length))
        return cls(layout.center, radius)

    def on_cylinder(self, p: np.ndarray) -> np.ndarray:
        return p[..., 2] < self.center[2]

    def distance(self, p: np.ndarray) -> np.ndarray:
        """Signed distance of points (..., 3) from the shell surface."""
        d = p - self.center
        r_sph = np.linalg.norm(d, axis=-1)
        r_cyl = np.hypot(d[..., 0], d[..., 1])
        return np.where(self.on_cylinder(p), r_cyl, r_sph) - self.radius

    def hit(self, origin: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Where rays (H, 3) from ``origin`` (inside the shell) along unit
        ``v`` leave it, (H, 3); the cylinder is taken as unbounded below."""
        o = origin - self.center
        # sphere: |o + t v| = R, the far root
        b = np.einsum("ij,ij->i", o, v)
        c = np.einsum("ij,ij->i", o, o) - self.radius ** 2
        t_s = -b + np.sqrt(np.maximum(b * b - c, 0.0))
        # cylinder: |(o + t v)_xy| = R
        a2 = v[:, 0] ** 2 + v[:, 1] ** 2
        b2 = o[:, 0] * v[:, 0] + o[:, 1] * v[:, 1]
        c2 = o[:, 0] ** 2 + o[:, 1] ** 2 - self.radius ** 2
        with np.errstate(divide="ignore", invalid="ignore"):
            t_c = (-b2 + np.sqrt(np.maximum(b2 * b2 - a2 * c2, 0.0))) / a2
        t_c = np.where(a2 > 1e-12, t_c, np.inf)
        z_s = o[:, 2] + t_s * v[:, 2]
        z_c = o[:, 2] + t_c * v[:, 2]
        # the exit is on the sphere part when that hit is above the equator
        # and the cylinder part when its hit is below it; exactly one holds
        # for an origin inside (both at the equator itself)
        t = np.where(z_s >= 0, t_s, np.where(z_c <= 0, t_c, t_s))
        t = np.minimum(t, 10 * self.radius)         # straight down the axis: no exit
        return origin + t[:, None] * v

    def grid(self, step_rad: float, n_ring: int = 96) -> np.ndarray:
        """Grid line segments (N, 2, 3), ``step_rad`` apart: latitude rings
        and meridians on the sphere (pole on the tube axis), and on the
        cylinder the meridians continued as axial lines plus rings every
        ``radius * step_rad`` mm (the same arc length as on the sphere)."""
        R, c = self.radius, self.center
        step = max(float(step_rad), math.radians(1.0))
        segs = []
        ring = np.linspace(0, 2 * np.pi, n_ring + 1)
        cr, sr = np.cos(ring), np.sin(ring)

        def poly(p):
            segs.append(np.stack([p[:-1], p[1:]], axis=1))

        # sphere: rings at polar angle k*step from the pole, equator included
        pol = list(np.arange(step, np.pi / 2 - 1e-6, step)) + [np.pi / 2]
        for a in pol:
            poly(c + np.c_[R * np.sin(a) * cr, R * np.sin(a) * sr, np.full_like(cr, R * np.cos(a))])
        # meridians, from the first ring down the sphere and the cylinder
        n_mer = max(int(round(2 * np.pi / step)), 4)
        a = np.linspace(pol[0], np.pi / 2, 24)
        zc = np.linspace(0, -self.cyl_len, 8)
        for k in range(n_mer):
            ph = 2 * np.pi * k / n_mer
            u = np.array([np.cos(ph), np.sin(ph)])
            sph = c + np.c_[R * np.sin(a)[:, None] * u, R * np.cos(a)]
            cyl = c + np.c_[np.tile(R * u, (len(zc), 1)), zc]
            poly(np.vstack([sph, cyl[1:]]))
        # cylinder rings
        dz = R * step
        for z in np.arange(dz, self.cyl_len + 1e-6, dz):
            poly(c + np.c_[R * cr, R * sr, np.full_like(cr, -z)])
        return np.concatenate(segs)


def centre_pivots(layout: DomeLayout) -> np.ndarray:
    """(H, 3) centre pivots: the dome centre for a root on the sphere part
    (at or above the equator), the point on the tube axis at the root's height
    for one on the cylinder part."""
    c = layout.center
    cyl = layout.root[:, 2] < c[2] - 0.5
    piv = np.tile(c, (len(layout), 1))
    piv[cyl, 2] = layout.root[cyl, 2]
    return piv


def centre_pivot_tips(layout: DomeLayout, shell: Shell, theta: np.ndarray) -> np.ndarray:
    """Centre-pivot tips on ``shell``, (..., H, 3) for ``theta`` (..., H, 2)
    rad: the rest direction, pivot -> root, tilted by |theta| toward the hole's
    e1 / e2 (made normal to that direction)."""
    th = np.nan_to_num(np.asarray(theta, dtype=np.float64))
    piv = centre_pivots(layout)
    u = layout.root - piv
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    e1 = layout.e1 - np.einsum("ij,ij->i", layout.e1, u)[:, None] * u
    e1 /= np.linalg.norm(e1, axis=1, keepdims=True)
    e2 = np.cross(u, e1)
    phi = np.hypot(th[..., 0], th[..., 1])[..., None]
    safe = np.where(phi > 1e-12, phi, 1.0)
    d = (th[..., :1] * e1 + th[..., 1:] * e2) / safe
    v = np.cos(phi) * u + np.sin(phi) * d
    o = np.broadcast_to(piv, v.shape)
    return shell.hit(o.reshape(-1, 3), v.reshape(-1, 3)).reshape(v.shape)


def centre_pivot_shape(layout: DomeLayout, shell: Shell, theta: np.ndarray,
                       n_pts: int = 15) -> tuple[np.ndarray, np.ndarray]:
    """Centre-pivot mode: each whisker turns about its centre pivot, so its tip
    scribes on ``shell`` (``centre_pivot_tips``). Returns (polylines (H, n_pts,
    3), tips (H, 3)): drawn straight from the hole to the tip, so the whisker
    stays attached."""
    tips = centre_pivot_tips(layout, shell, theta)
    s = np.linspace(0.0, 1.0, n_pts)[None, :, None]
    return layout.root[:, None, :] + s * (tips - layout.root)[:, None, :], tips


def root_tips(layout: DomeLayout, theta: np.ndarray, lr: float = LR_MM) -> np.ndarray:
    """Tips of ``whisker_shape`` (the physical root model), (..., H, 3) for
    ``theta`` (..., H, 2), in closed form."""
    th = np.nan_to_num(np.asarray(theta, dtype=np.float64))
    phi = np.hypot(th[..., 0], th[..., 1])[..., None]
    safe = np.where(phi > 1e-9, phi, 1.0)
    d = np.where(phi > 1e-9, (th[..., :1] * layout.e1 + th[..., 1:] * layout.e2) / safe,
                 layout.e1)
    k = lr / safe                                                       # root radius
    small = phi < 1e-6
    ls = np.maximum(layout.length - lr, 0.0)[:, None]
    u = np.where(small, lr * phi / 2, k * (1 - np.cos(phi))) + ls * np.sin(phi)
    w = np.where(small, lr, k * np.sin(phi)) + ls * np.cos(phi)
    return layout.root + u * d + w * layout.axis


def neutral_tips(layout: DomeLayout, shell: Optional[Shell] = None) -> np.ndarray:
    """(H, 3) zero-bend tips: on ``shell`` (centre-pivot mode) or, without
    one, the physical rest tips (root + length along the axis)."""
    zero = np.zeros((len(layout), 2))
    return centre_pivot_tips(layout, shell, zero) if shell is not None else root_tips(layout, zero)


def tip_path(layout: DomeLayout, theta: np.ndarray, shell: Optional[Shell] = None,
             n: int = 8) -> np.ndarray:
    """(H, n, 3) the tip's way from neutral (zero bend) to ``theta``, the
    bend scaled 0..1: on the shell in centre-pivot mode (an arc of the sphere
    / a curve on the cylinder), the root model's tips otherwise."""
    th = np.asarray(theta, dtype=np.float64)[None] * np.linspace(0.0, 1.0, n)[:, None, None]
    p = centre_pivot_tips(layout, shell, th) if shell is not None else root_tips(layout, th)
    return p.transpose(1, 0, 2)


def arrow_segments(path: np.ndarray, normal: np.ndarray, head_mm: float = 8.0,
                   head_deg: float = 25.0) -> np.ndarray:
    """Arrows along ``path`` (H, n, 3) as line segments (H, n + 1, 2, 3): the
    n - 1 pieces of the shaft, then the two barbs of the head, which lie in
    the plane normal to ``normal`` (H, 3; e.g. the shell's outward normal at
    the tip). Each barb is ``head_mm`` long, but at most 40 % of the arrow's
    chord, so a zero arrow collapses to a point at its tip."""
    shaft = np.stack([path[:, :-1], path[:, 1:]], axis=2)              # (H, n-1, 2, 3)
    tip = path[:, -1]
    t = tip - path[:, -2]
    tn = np.linalg.norm(t, axis=1, keepdims=True)
    t = t / np.maximum(tn, 1e-12)
    side = np.cross(normal, t)
    side /= np.maximum(np.linalg.norm(side, axis=1, keepdims=True), 1e-12)
    chord = np.linalg.norm(tip - path[:, 0], axis=1, keepdims=True)
    h = np.minimum(head_mm, 0.4 * chord)
    a = math.radians(head_deg)
    barbs = [tip - h * (math.cos(a) * t + sgn * math.sin(a) * side) for sgn in (1, -1)]
    head = np.stack([np.stack([tip, b], axis=1) for b in barbs], axis=1)  # (H, 2, 2, 3)
    return np.concatenate([shaft, head], axis=1)


# ---------------------------------------------------------------------------
# plane map: every whisker at once, as a pilot inside the ROV sees the arrays
# ---------------------------------------------------------------------------
@dataclass
class PlaneMap:
    """The holes laid out flat, seen from inside the ROV looking forward: the
    port array on the left, starboard on the right, each side as seen from
    inside facing it, so its front column is toward the centre line and its
    back column lateral. ``xy`` (H, 2) mm: x = the distance aft of the
    foremost root (z), offset by ``gap`` from the centre line, toward the
    hole's side; y = the root's height (+y up). ``fwd`` (H,) is the screen x
    direction of forward on each side (+1 port, -1 starboard). ``segments``
    (N, 2, 2) are the grid lines joining neighbouring roots within each side,
    up / down a column and across a row."""

    xy: np.ndarray
    fwd: np.ndarray
    segments: np.ndarray
    spacing: float
    gap: float

    @classmethod
    def of(cls, layout: DomeLayout, cluster_mm: float = 10.0) -> "PlaneMap":
        root = layout.root
        port = root[:, 0] - layout.center[0] >= 0              # +x = port
        aft = root[:, 2].max() - root[:, 2]
        cols = _clusters(root[:, 2], cluster_mm)
        rows = _clusters(root[:, 1], cluster_mm)
        # root spacing: the steps between neighbouring columns (z) and rows (y)
        sp = np.concatenate([np.diff(np.sort([v[g].mean() for g in c]))
                             for v, c in ((root[:, 2], cols), (root[:, 1], rows))])
        spacing = float(np.median(sp)) if len(sp) else 30.0
        gap = 0.6 * spacing
        side = np.where(port, -1.0, 1.0)                        # screen x sign of each side
        xy = np.c_[side * (gap + aft), root[:, 1]]
        segs = []
        for s in (True, False):
            for groups, key in ((cols, 1), (rows, 0)):
                for g in groups:
                    ids = [i for i in g if port[i] == s]
                    ids.sort(key=lambda i: xy[i, key])
                    segs += [(xy[a], xy[b]) for a, b in zip(ids[:-1], ids[1:])]
        return cls(xy, -side, np.array(segs).reshape(-1, 2, 2), spacing, gap)

    @staticmethod
    def lift_up(layout: DomeLayout) -> np.ndarray:
        """(H,) +1 where the hole's e2 points up (theta_y + = tip up), else -1."""
        return np.where(layout.e2[:, 1] < 0, -1.0, 1.0)

    def tips(self, layout: DomeLayout, theta: np.ndarray, radius: float) -> np.ndarray:
        """(..., H, 2) tip points for bends ``theta`` (..., H, 2) rad (drag,
        e2): the root moved ``radius * angle`` mm, drag toward the side's
        forward, lift up; NaN theta reads as no bend."""
        th = np.nan_to_num(np.asarray(theta, dtype=np.float64))
        d = np.stack([th[..., 0] * self.fwd, th[..., 1] * self.lift_up(layout)], axis=-1)
        return self.xy + radius * d


def _clusters(v: np.ndarray, gap: float) -> list[list[int]]:
    """Indices of ``v`` grouped where sorted neighbours are less than ``gap`` apart."""
    order = np.argsort(v)
    out = [[int(order[0])]] if len(v) else []
    for a, b in zip(order[:-1], order[1:]):
        if v[b] - v[a] < gap:
            out[-1].append(int(b))
        else:
            out.append([int(b)])
    return out


def nice_grid_step(gain: float, target_deg: float = 12.0) -> float:
    """Auto grid spacing in degrees of BEND: a 1-2-5 value whose drawn
    spacing (times the display gain) is nearest ``target_deg``."""
    vals = np.array([m * 10.0 ** e for e in range(-2, 3) for m in (1, 2, 5)])
    return float(vals[np.argmin(np.abs(np.log(vals * gain / target_deg)))])


class TrailBuffer:
    """Ring buffer of tip positions (and bend magnitudes) with their times,
    for the scribe trails. ``push`` keeps at most ``rate_hz`` samples a second,
    so the capacity covers ``max_s`` seconds whatever the frame rate."""

    def __init__(self, n_whisker: int, max_s: float = 10.0, rate_hz: float = 60.0):
        self.rate_hz = float(rate_hz)
        self.cap = int(math.ceil(max_s * rate_hz)) + 2
        self.pos = np.full((self.cap, n_whisker, 3), np.nan)
        self.mag = np.full((self.cap, n_whisker), np.nan)
        self.t = np.full(self.cap, -np.inf)
        self.head = 0                       # next slot
        self.count = 0

    def clear(self) -> None:
        self.pos[:] = np.nan
        self.mag[:] = np.nan
        self.t[:] = -np.inf
        self.head = self.count = 0

    def push(self, t: float, tips: np.ndarray, mag: np.ndarray) -> bool:
        """Store the tips (H, 3) at time ``t`` s (NaN rows: no data, a gap)."""
        if self.count:
            dt = t - self.t[(self.head - 1) % self.cap]
            if dt < 0:                          # time went back (a new clock): start over
                self.clear()
            elif dt < 0.999 / self.rate_hz:
                return False
        self.pos[self.head] = tips
        self.mag[self.head] = mag
        self.t[self.head] = t
        self.head = (self.head + 1) % self.cap
        self.count = min(self.count + 1, self.cap)
        return True

    def recent(self, now: float, window_s: float):
        """The samples of the last ``window_s`` s, oldest first: positions
        (K, H, 3), magnitudes (K, H), ages (K,) s."""
        n = min(self.count, int(math.ceil(window_s * self.rate_hz)) + 2)
        idx = (self.head - n + np.arange(n)) % self.cap
        age = now - self.t[idx]
        keep = age <= window_s
        idx = idx[keep]
        return self.pos[idx], self.mag[idx], age[keep]

    def segments(self, now: float, window_s: float):
        """Trail line segments for one combined line item: positions
        (2 * (K - 1) * H, 3) as (start, end) pairs per whisker and step, the
        bend magnitude at each end and alpha 1 -> 0 with age; segments touching
        a gap are NaN-free with alpha 0."""
        pos, mag, age = self.recent(now, window_s)
        if len(age) < 2:
            return np.zeros((0, 3)), np.zeros(0), np.zeros(0)
        a = np.clip(1.0 - age / window_s, 0.0, 1.0)                     # (K,)
        p = np.stack([pos[:-1], pos[1:]], axis=2)                       # (K-1, H, 2, 3)
        m = np.stack([mag[:-1], mag[1:]], axis=2)                       # (K-1, H, 2)
        al = np.broadcast_to(np.stack([a[:-1], a[1:]], axis=1)[:, None, :], m.shape).copy()
        ok = np.isfinite(p).all(axis=(2, 3))                            # (K-1, H)
        al[~ok] = 0.0
        p = np.where(ok[..., None, None], p, 0.0)
        # whisker-major, so each whisker's trail is contiguous
        return (p.transpose(1, 0, 2, 3).reshape(-1, 3), m.transpose(1, 0, 2).reshape(-1),
                al.transpose(1, 0, 2).reshape(-1))


# ---------------------------------------------------------------------------
# the whole pipeline
# ---------------------------------------------------------------------------
class DomePose:
    """mV blocks in, latest smoothed bend angles / polylines out.

    ``hole_of`` maps whisker number -> hole (gauges A, B of the whisker in
    the hole's (e1, e2) order); whiskers not in it are ignored, holes nobody
    maps to stay straight and report NaN angles. ``hole_channels`` (H, 2)
    instead names the two gauge channels of every hole directly (any slots,
    e.g. a dome map's drag and lift gauges); -1 = no gauge on that axis, which
    reads as zero bend there; a hole with -1 on both is unmapped."""

    def __init__(self, layout: DomeLayout, hole_of: Optional[dict] = None,
                 calibration: Optional[Calibration] = None, smooth_tau_s: float = 0.03,
                 tare_n: int = 32, drift_tau_s: float = 0.0,
                 hole_channels: Optional[np.ndarray] = None):
        self.layout = layout
        if hole_channels is not None:
            hc = np.asarray(hole_channels, dtype=np.intp).reshape(len(layout), 2)
            self.hole_of = {}
            self.whiskers = ()
            self.holes = np.flatnonzero((hc >= 0).any(axis=1))
            self.ch = hc[self.holes]
        else:
            self.hole_of = dict(PLACEHOLDER_HOLE_OF if hole_of is None else hole_of)
            self.whiskers = tuple(sorted(self.hole_of))
            self.holes = np.array([self.hole_of[w] for w in self.whiskers], dtype=np.intp)
            self.ch = channels_of(self.whiskers)                        # (W, 2)
        self.calibration = calibration or Calibration.nominal(len(layout))
        self.tare = Tare(NCH, tare_n, drift_tau_s)
        self.smoother = Smoother(NCH, smooth_tau_s)

    def request_tare(self) -> None:
        self.tare.request()
        self.smoother.reset()

    def feed(self, mv: np.ndarray, fs: float) -> None:
        """A block (N, 32) of mV scans (any N, also 0)."""
        if len(mv):
            self.smoother.update(self.tare.update(mv, fs), fs)

    def delta_ab(self) -> np.ndarray:
        """(H, 2) smoothed mV deltas per hole; NaN where unmapped / railed / taring."""
        out = np.full((len(self.layout), 2), np.nan)
        out[self.holes] = np.where(self.ch >= 0, self.smoother.value[np.maximum(self.ch, 0)], 0.0)
        return out

    def angles(self) -> np.ndarray:
        """(H, 2) rad, theta_x / theta_y per hole."""
        return self.calibration.apply(self.delta_ab())

    def shape(self, gain: float = 1.0, shell: Optional[Shell] = None, **kw) \
            -> tuple[np.ndarray, np.ndarray]:
        """(polylines (H, S, 3) mm, bend magnitude (H,) rad, NaN = no data),
        angles times the display ``gain``. With ``shell``: centre-pivot mode
        (tips on the shell, ``centre_pivot_shape``), else the physical root
        (``whisker_shape``)."""
        th = self.angles() * gain
        mag = np.hypot(th[:, 0], th[:, 1])
        if shell is not None:
            n_pts = kw.get("n_root", 6) + kw.get("n_shaft", 6) + 1
            return centre_pivot_shape(self.layout, shell, th, n_pts)[0], mag
        return whisker_shape(self.layout, th, **kw), mag
