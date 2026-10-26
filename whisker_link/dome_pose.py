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

    @classmethod
    def load(cls, path) -> "DomeLayout":
        doc = json.loads(Path(path).read_text())
        ws = doc["whiskers"]
        arr = lambda k: np.array([w[k] for w in ws], dtype=np.float64)    # noqa: E731
        return cls(arr("root"), arr("axis"), arr("e1"), arr("e2"), arr("length"),
                   [f"{w['side']}{w['row']}{w['col']}" for w in ws])

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
# the whole pipeline
# ---------------------------------------------------------------------------
class DomePose:
    """mV blocks in, latest smoothed bend angles / polylines out.

    ``hole_of`` maps whisker number -> hole; whiskers not in it are ignored,
    holes nobody maps to stay straight and report NaN angles."""

    def __init__(self, layout: DomeLayout, hole_of: Optional[dict] = None,
                 calibration: Optional[Calibration] = None, smooth_tau_s: float = 0.03,
                 tare_n: int = 32, drift_tau_s: float = 0.0):
        self.layout = layout
        self.hole_of = dict(PLACEHOLDER_HOLE_OF if hole_of is None else hole_of)
        self.whiskers = tuple(sorted(self.hole_of))
        self.holes = np.array([self.hole_of[w] for w in self.whiskers], dtype=np.intp)
        self.ch = channels_of(self.whiskers)                            # (W, 2)
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
        out[self.holes] = self.smoother.value[self.ch]
        return out

    def angles(self) -> np.ndarray:
        """(H, 2) rad, theta_x / theta_y per hole."""
        return self.calibration.apply(self.delta_ab())

    def shape(self, gain: float = 1.0, **kw) -> tuple[np.ndarray, np.ndarray]:
        """(polylines (H, S, 3) mm, bend magnitude (H,) rad, NaN = no data),
        angles times the display ``gain``."""
        th = self.angles() * gain
        mag = np.hypot(th[:, 0], th[:, 1])
        return whisker_shape(self.layout, th, **kw), mag
