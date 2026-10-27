"""Amplitude spectra of the rolling buffer, and the spectrogram history.

``amp_spectra`` is the one method all spectral windows use: mean removed, Hann
window, scaled so a sine of amplitude a reads a. It takes a stack of windows,
so the spectrogram columns that fell due in one GUI tick are one rfft call.

``SpecHistory`` keeps the spectrogram columns in a ring (no shifting of the
whole history per column); a dB copy is kept alongside, converted one column
at a time, once a window has asked for dB.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from wavi_py.views.spectral_base import to_db

SPEC_COL_HZ = 20.0              # spectrogram columns: at most this many a second


def amp_spectra(x: np.ndarray, scale: float = 1.0) -> np.ndarray:
    """(..., nfreq, nch) amplitude of the windows ``x`` (..., n, nch); see the module doc.

    Windows with gaps (non-finite samples) come out all zero.
    """
    # float64: a railed gauge (~1200 mV, constant) leaves ~1e-4 of float32 rounding
    # after the mean removal, which would read as a tiny peak instead of nothing
    x = np.asarray(x, dtype=np.float64)
    n = x.shape[-2]
    finite = np.isfinite(x).all(axis=(-2, -1))
    x = np.where(finite[..., None, None], x, 0.0)
    w = np.hanning(n)
    # Mean removal, not a linear detrend: with a 1 s window a 1 Hz sine is one cycle,
    # and a line fitted to one cycle takes ~45 % of its amplitude (5 mV read as 2.7).
    # Slow drift is kept out of the higher bins by the Hann window.
    x = (x - x.mean(axis=-2, keepdims=True)) * w[:, None]
    amp = np.abs(np.fft.rfft(x, axis=-2))
    amp *= 2 * scale / w.sum()
    amp[..., 0, :] /= 2
    if n % 2 == 0:
        amp[..., -1, :] /= 2
    amp[amp < 1e-9] = 0                    # round-off of a constant (railed) gauge
    return amp


def amp_spectrum(x: np.ndarray, scale: float = 1.0) -> Optional[np.ndarray]:
    """(nfreq, nch) amplitude of one window ``x`` (n, nch); None if it has gaps."""
    if not np.isfinite(x).all():
        return None
    return amp_spectra(x, scale)


class SpecHistory:
    """Spectrogram columns in a ring: ``cols`` columns of ``rows`` = nfreq * nch values,
    channel-major (row c * nfreq + f), as amplitude (and dB once asked for)."""

    def __init__(self, rows: int, cols: int):
        self.rows = int(rows)
        self._ring = np.zeros((max(1, int(cols)), self.rows), dtype=np.float32)
        self._db: Optional[np.ndarray] = None
        self._head = 0                         # next column to write
        self.version = 0                       # bumped on every change

    @property
    def cols(self) -> int:
        return self._ring.shape[0]

    def clear(self) -> None:
        self._ring.fill(0)
        if self._db is not None:
            self._db.fill(np.nan)
        self._head = 0
        self.version += 1

    def push(self, frames: np.ndarray) -> None:
        """Append amplitude frames (k, nfreq, nch), oldest first."""
        k = frames.shape[0]
        if k == 0:
            return
        cols = np.transpose(frames, (0, 2, 1)).reshape(k, self.rows)    # channel-major
        if k > self.cols:
            cols = cols[-self.cols:]
            k = self.cols
        idx = (self._head + np.arange(k)) % self.cols
        self._ring[idx] = cols
        if self._db is not None:
            self._db[idx] = to_db(cols)
        self._head = int((self._head + k) % self.cols)
        self.version += 1

    def latest(self, nfreq: int) -> np.ndarray:
        """The newest frame as (nfreq, nch)."""
        return self._ring[(self._head - 1) % self.cols].reshape(-1, nfreq).T

    def ordered(self, db: bool = False, rows: slice = slice(None)) -> np.ndarray:
        """(cols, rows) of the ``rows``, oldest first (a copy)."""
        src = self._ring
        if db:
            if self._db is None:
                self._db = to_db(self._ring).astype(np.float32)
            src = self._db
        return np.concatenate([src[self._head:, rows], src[:self._head, rows]])

    def resize(self, cols: int) -> None:
        """Keep the newest ``cols`` columns (zero-padded at the old end)."""
        cols = max(1, int(cols))
        old = self.ordered()
        new = np.zeros((cols, self.rows), dtype=np.float32)
        keep = min(cols, old.shape[0])
        new[cols - keep:] = old[old.shape[0] - keep:]
        self._ring = new
        self._head = 0
        self._db = None
        self.version += 1
