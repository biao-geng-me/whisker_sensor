"""Time series by name over a time range: the one read API of the plot data.

Two sources, each its own time axis (session seconds, sorted):

- ``WSK``: the whisker gauges, mV at the ADC (before V0 and the display scale),
  columns in stream order (W1_A, W1_B, ...);
- ``IMU``: the IMU rows (``daq.server_reader.IMU_CHANNELS``: accel mg, gyro
  mrad/s, attitude rad).

Two implementations of ``Series``:

- ``ArraySeries``: arrays in memory, e.g. the GUI's live rolling buffers (the
  newest ``t_buffer_s``) or a snapshot of them (the Line window's copy);
- ``replay.store.RecordingStore``: a whole recording, with a min/max pyramid
  so a draw of any span costs about the same.

Reads:

- ``get(source, cols, t0, t1, despike)``: every sample with t0 <= t <= t1;
- ``envelope(source, cols, t0, t1, n, despike)``: what a 1 px pen needs to draw
  the span ``n`` pixel columns wide: the raw samples when there are fewer than
  about two per column, else the min and max of each column's samples (in that
  order, at the column's first and last sample times), so at most ~2n points.
  Columns are aligned to the absolute sample index, so a scrolling or panned
  view does not shimmer;
- ``channels(names, t0, t1, despike)``: by name, any mix of sources
  (``{name: (t, y)}``), for pick-and-compare, select-for-FFT and the like.

``despike`` (whisker only): the spikes replaced (``daq/despike.py``); a source
that holds one version only ignores it (live: the buffers are already despiked
or not, as the panel's Despike says).
"""

from __future__ import annotations

from typing import Optional, Sequence, Union

import numpy as np

WSK, IMU = "wsk", "imu"
Cols = Union[slice, Sequence[int], np.ndarray]


def peak_decimate(t: np.ndarray, y: np.ndarray, k: int) -> tuple[np.ndarray, np.ndarray]:
    """Min and max of each ``k``-sample block of ``y`` (N, m), interleaved.

    Two points per block keep the full envelope, so a 1 px pen draws what the
    full data would. Blocks are aligned to the absolute sample index (t / dt)
    so they don't shimmer as the window scrolls; the partial blocks at either
    end are a min / max pair too.
    """
    n = len(t)
    if k <= 1 or n < 4 * k:
        return t, y
    head = -int(round(t[0] * (n - 1) / (t[-1] - t[0]))) % k
    starts = np.arange(head, n, k)
    if head:
        starts = np.r_[0, starts]
    ends = np.r_[starts[1:], n]
    yd = np.empty((2 * len(starts), y.shape[1]), dtype=y.dtype)
    yd[0::2] = np.minimum.reduceat(y, starts, axis=0)
    yd[1::2] = np.maximum.reduceat(y, starts, axis=0)
    td = np.empty(len(yd))
    td[0::2] = t[starts]
    td[1::2] = t[ends - 1]
    return td, yd


def span_index(t: np.ndarray, t0: float, t1: float, margin: int = 0) -> tuple[int, int]:
    """[i0, i1) of the samples of sorted ``t`` in [t0, t1], widened by ``margin``
    samples either side (a drawn curve then runs on past the plot's edges)."""
    i0 = int(np.searchsorted(t, t0, side="left")) - margin
    i1 = int(np.searchsorted(t, t1, side="right")) + margin
    return max(0, i0), min(len(t), i1)


class Series:
    """The read API (see the module doc). Subclasses implement ``_arrays`` or
    override ``get`` / ``envelope``."""

    def names(self, source: str) -> list[str]:
        raise NotImplementedError

    def has(self, source: str) -> bool:
        return len(self._t(source)) > 0

    def extent(self, source: str) -> Optional[tuple[float, float]]:
        """Times of the first and last sample of ``source`` (None: none)."""
        t = self._t(source)
        return (float(t[0]), float(t[-1])) if len(t) else None

    def get(self, source: str, cols: Cols, t0: float, t1: float,
            despike: bool = False) -> tuple[np.ndarray, np.ndarray]:
        t, y = self._arrays(source, despike)
        i0, i1 = span_index(t, t0, t1)
        return t[i0:i1].copy(), np.array(y[i0:i1][:, cols])

    def envelope(self, source: str, cols: Cols, t0: float, t1: float, n: int,
                 despike: bool = False) -> tuple[np.ndarray, np.ndarray]:
        t, y = self._arrays(source, despike)
        i0, i1 = span_index(t, t0, t1, margin=1)
        k = max(1, -(-(i1 - i0) // max(1, int(n))))       # samples per column, rounded up
        if k > 1:                                   # whole blocks past both edges
            i0, i1 = max(0, i0 - k), min(len(t), i1 + k)
        return peak_decimate(t[i0:i1], y[i0:i1][:, cols], k)

    def channels(self, names: Sequence[str], t0: float, t1: float,
                 despike: bool = False) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        out = {}
        for src in (WSK, IMU):
            have = self.names(src)
            want = [n for n in names if n in have]
            if want:
                t, y = self.get(src, [have.index(n) for n in want], t0, t1, despike)
                out.update({n: (t, y[:, j]) for j, n in enumerate(want)})
        missing = [n for n in names if n not in out]
        if missing:
            raise KeyError(f"no such channel(s): {', '.join(missing)}")
        return out

    def _t(self, source: str) -> np.ndarray:
        return self._arrays(source, False)[0]

    def _arrays(self, source: str, despike: bool) -> tuple[np.ndarray, np.ndarray]:
        raise NotImplementedError


class ArraySeries(Series):
    """Arrays in memory: ``{source: (t (N,), y (N, m))}`` and the column names.
    The arrays are used as given (not copied); reads return copies."""

    def __init__(self, data: Optional[dict] = None, names: Optional[dict] = None) -> None:
        self._data = {WSK: (np.empty(0), np.empty((0, 0), np.float32)),
                      IMU: (np.empty(0), np.empty((0, 0), np.float32))}
        self._names: dict[str, list[str]] = {WSK: [], IMU: []}
        for src, (t, y) in (data or {}).items():
            self.set(src, t, y)
        for src, n in (names or {}).items():
            self._names[src] = list(n)

    def set(self, source: str, t: np.ndarray, y: np.ndarray) -> None:
        self._data[source] = (t, y)

    def names(self, source: str) -> list[str]:
        n = self._names.get(source) or []
        m = self._data[source][1].shape[1] if self._data[source][1].ndim == 2 else 0
        return n if len(n) == m else [f"{source}{i}" for i in range(m)]

    def _arrays(self, source: str, despike: bool) -> tuple[np.ndarray, np.ndarray]:
        return self._data[source]
