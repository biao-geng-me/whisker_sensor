"""Despike: replace isolated spikes (1 to 3 samples) by the local median.

A centred Hampel test with a two-part scale. Sample i of a gauge is a spike when

    |x[i] - med7[i]| > K * max(noise, local / RES_SIGMA)   (noise > 0)

- ``med7``: the median of the 7 samples centred on i (±12 ms at 250 Hz), so a
  run of up to 3 samples can be a spike and a step or a ramp never is;
- ``noise``: 1.4826 * the median |first difference| / sqrt(2) over ±25
  samples (±100 ms): the white noise when the gauge is still, more when it
  moves (a fast deflection's sharp top is not a spike);
- ``local``: 1.4826 * MAD of the residuals x - med7 over the same ±25.
  This is what keeps real fast signal untouched: during thrust the gauges carry
  a near-Nyquist zigzag of ±1 mV that is far above the white noise but not
  above itself. ``RES_SIGMA`` is the residual's scale for white noise
  (0.83 sigma), so both parts are in the same units.

Every window is centred and ±25 wide, so the test needs 28 samples after the
one judged (``LAG``) and nothing more: replay and live run the same test.

A spike is replaced by ``med7[i]`` (on a steep slope that is a neighbour's
value, so off by at most one sample's change). K = 8. On the pilot2 / vac_1102
recordings (survey: /scratch/bigeme/whisker_dev/replay_test/clean/index.html)
this flags 3 samples in 18 min on the bench, 34 in the 13 min dive and 19 in
the 2 min one, nearly all 1-2 sample excursions at the onset of a hard
contact, and nothing in plain noise; ADC or frame glitches (rail codes, bit
flips) were not seen at all. The 12-sample block fill (``outliers.py``) changes ~2 % of the
samples on the same data, almost all of them noise, and misses most spikes.

Two drivers of the same test:

- ``find_spikes`` / ``despike``: a whole recording, every window centred
  (replay: ``replay/index.py`` keeps the replacements, sparse, in its cache).
- ``Despiker``: a live stream, block by block: a sample is judged once the
  ``LAG`` = 28 samples after it (112 ms at 250 Hz) have arrived. The newest
  ``LAG`` of each block go out raw (provisional) and their final values come
  with later blocks (the caller patches its buffer tail). A shorter lag with
  one-sided windows was tried on the pilot2 dive: it flagged hundreds of
  samples the centred test does not, nearly all at the onset of thrust
  oscillation bursts or at the top of fast deflections, i.e. real signal.

Constant gauges (zero noise, e.g. the railed dead mux slots) are never touched.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from scipy.ndimage import median_filter

K = 8.0                    # threshold, in units of the scale
HALF = 3                   # med7: 3 samples either side
SCALE_HALF = 25            # noise and local scale: ±25 samples
LAG = SCALE_HALF + HALF    # Despiker: samples judged this late
RES_SIGMA = 0.83           # std of x - med7 for white noise, in units of its sigma
_MAD = 1.4826
_FLOOR_BLOCK = 51          # find_spikes' pre-selection: noise per block of this many


_WIN = 2 * SCALE_HALF + 1


def _scales(d: np.ndarray, r: np.ndarray, rows: np.ndarray, cols: np.ndarray,
            hi: int) -> np.ndarray:
    """max(noise, local) of the samples (rows, cols): the windows of ±SCALE_HALF
    of |first differences| ``d`` and |residuals| ``r``, shifted inside [0, hi)
    at the edges (always _WIN samples, if there are)."""
    start = np.clip(rows - SCALE_HALF, 0, max(0, hi - _WIN))
    win = np.minimum(start[:, None] + np.arange(_WIN), hi - 1)
    noise = _MAD * np.median(d[win, cols[:, None]], axis=1) / np.sqrt(2)
    local = _MAD * np.median(r[win, cols[:, None]], axis=1) / RES_SIGMA
    return np.where(noise > 0, np.maximum(noise, local), np.inf)


def find_spikes(x: np.ndarray, k: float = K) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The spikes of a whole (N, C) recording, every window centred.

    Returns ``(rows, cols, values)``: the flagged samples and their
    replacements (the centred median), sorted by row."""
    x = np.asarray(x, dtype=np.float64)
    if x.ndim != 2:
        raise ValueError(f"expected (N, C) array, got shape {x.shape}")
    n = x.shape[0]
    empty = (np.empty(0, np.int64), np.empty(0, np.int64), np.empty(0))
    if n < 2 * HALF + 1:
        return empty
    med = median_filter(x, size=(2 * HALF + 1, 1), mode="nearest")
    r = np.abs(x - med)
    d = np.abs(np.diff(x, axis=0, prepend=x[1:2]))   # the first: its successor's
    # pre-selection, so the windows are only taken where they can matter: above k x a
    # floor of the noise (half the gauge's quietest 1st percentile of 51-sample blocks)
    nb = n // _FLOOR_BLOCK
    if nb:
        blk = np.median(d[:nb * _FLOOR_BLOCK].reshape(nb, _FLOOR_BLOCK, -1), axis=1)
        floor = 0.5 * _MAD * np.percentile(blk, 1, axis=0) / np.sqrt(2)
    else:
        floor = np.zeros(x.shape[1])
    rows, cols = np.nonzero(r > k * floor)
    if not len(rows):
        return empty
    keep = r[rows, cols] > k * _scales(d, r, rows, cols, n)
    rows, cols = rows[keep], cols[keep]
    return rows, cols, med[rows, cols]


def despike(x: np.ndarray, k: float = K) -> np.ndarray:
    """A copy of ``x`` (N, C) with its spikes replaced (``find_spikes``)."""
    out = np.array(x, dtype=np.float64, copy=True)
    rows, cols, vals = find_spikes(out, k)
    out[rows, cols] = vals
    return out.astype(np.asarray(x).dtype, copy=False)


class Despiker:
    """The same test on a stream, block by block (see the module notes).

    ``push(x)`` takes the next (n, C) samples and returns ``y``, (n_back + n, C):
    the ``n_back`` samples returned provisionally before, now final, then the
    new ones, of which the newest ``pending`` (at most ``LAG``) are still raw
    and provisional. ``n_back = len(y) - n``. ``reset()`` at a discontinuity
    (the pending samples stay as they went out)."""

    def __init__(self, k: float = K) -> None:
        self.k = k
        self.reset()

    def reset(self) -> None:
        self._x: Optional[np.ndarray] = None   # raw history; its last `pending` rows unjudged
        self._fresh = True                     # nothing judged yet: _x starts the stream
        self.pending = 0

    def push(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        n = len(x)
        if n == 0:
            return np.empty((0,) + x.shape[1:])
        if self._x is None or self._x.shape[1] != x.shape[1]:
            self.reset()
            self._x = np.empty((0, x.shape[1]))
        X = np.concatenate([self._x, x])
        j0 = len(self._x) - self.pending            # first unjudged
        j1 = len(X) - LAG                           # judged now: [j0, j1)
        keep = 2 * (LAG + HALF) + 1
        if self._fresh and len(X) < _WIN + HALF:
            j1 = j0                                 # the start: the first full windows
        if j1 <= j0:
            self.pending += n
            self._x = X[-(keep + self.pending):]
            return X[j0:].copy()
        y = X[j0:].copy()
        # the windows of rows [a, b): every row within SCALE_HALF of a judged one (the
        # history is long enough that only the stream start shifts them)
        a, b = max(j0 - SCALE_HALF, 0), j1 + SCALE_HALF
        if self._fresh:
            a, self._fresh = 0, False
        lo = a - HALF                               # the medians' rows: [lo, b + HALF)
        Xm = X[max(lo, 0):b + HALF]
        if lo < 0:
            Xm = np.concatenate([np.repeat(Xm[:1], -lo, axis=0), Xm])
        med = np.median(sliding_window_view(Xm, 2 * HALF + 1, axis=0), axis=-1)
        r = np.abs(X[a:b] - med)
        d = np.abs(np.diff(X, axis=0, prepend=X[1:2]))[a:b]  # a > 0 unless at the start
        m = j1 - j0
        rows = np.repeat(np.arange(j0 - a, j1 - a), x.shape[1])
        cols = np.tile(np.arange(x.shape[1]), m)
        scale = _scales(d, r, rows, cols, b - a).reshape(m, -1)
        bad = r[j0 - a:j1 - a] > self.k * scale
        y[:m][bad] = med[j0 - a:j1 - a][bad]
        self.pending = len(X) - j1
        self._x = X[-keep:]
        return y
