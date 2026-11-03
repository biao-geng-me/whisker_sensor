"""Sample times: the device clocks mapped onto the Pi's receive clock.

Every event carries ``t``, when daq_server received it: USB / UDP latency and
scheduling put it a few ms late, and the reader threads deliver in bursts.
Each source also stamps its own samples (ESP32 ``tus`` micros, the autopilot's
RAW_IMU ``time_usec``): precise, but on another crystal. A sample's time is

    t = a + b * t_source

on the lower envelope of the receive times (a sample can arrive late, never
early), as in ``whisker_link.convert``. Two models, one interface
(``wsk(t_rx, tus)`` and ``imu(t_rx, time_usec)``, both returning seconds on the
receive clock):

- ``LiveClock``: b = 1 and a = the smallest receive-minus-source lag over the
  last ``LIVE_WINDOW_S`` (it follows crystal drift); for a live stream.
- ``FittedClock``: an envelope fit per clock segment (a reboot restarts the
  source clock), computed over a whole recording when it is indexed
  (``replay/index.py``).
"""

from __future__ import annotations

import bisect
import collections
from typing import Optional, Sequence

LIVE_WINDOW_S = 10.0


class _WindowMinLag:
    """Sliding-window minimum of rx - src over the last ``win`` seconds of src."""

    def __init__(self, win: float) -> None:
        self._win = win
        self._q: collections.deque = collections.deque()    # (src, lag), lag increasing
        self._last_src: Optional[float] = None

    def __call__(self, rx: float, src: float) -> float:
        if self._last_src is not None and src < self._last_src - 1e-3:
            self._q.clear()                                  # the source clock restarted
        self._last_src = src
        lag = rx - src
        q = self._q
        while q and q[-1][1] >= lag:
            q.pop()
        q.append((src, lag))
        while src - q[0][0] > self._win:
            q.popleft()
        return src + q[0][1]


class LiveClock:
    def __init__(self, window_s: float = LIVE_WINDOW_S) -> None:
        self._wsk = _WindowMinLag(window_s)
        self._imu = _WindowMinLag(window_s)

    def wsk(self, t_rx: float, tus: int) -> float:
        return self._wsk(t_rx, tus * 1e-6)

    def imu(self, t_rx: float, time_usec: int) -> float:
        return self._imu(t_rx, time_usec * 1e-6)

    def restart(self) -> None:
        """A discontinuity in the stream (a replay seek): forget the history."""
        self.__init__(self._wsk._win)


class _Segments:
    """Envelope fits of one source: [(t_rx_first, src0, a, b), ...] in receive order.
    A sample is mapped with the segment it falls in by receive time, checked against
    the source time (a mismatch of more than ``tol`` s falls back to the receive time)."""

    def __init__(self, segs: Sequence[Sequence[float]], tol: float = 1.0) -> None:
        self.segs = [tuple(float(x) for x in s) for s in segs]
        self._starts = [s[0] for s in self.segs]
        self._tol = tol

    def __call__(self, rx: float, src: float) -> float:
        i = bisect.bisect_right(self._starts, rx + self._tol) - 1
        for j in (i, i - 1, i + 1):
            if 0 <= j < len(self.segs):
                _, s0, a, b = self.segs[j]
                t = a + b * (src - s0)
                if abs(t - rx) <= self._tol:
                    return t
        return rx


class FittedClock:
    def __init__(self, wsk_segs: Sequence, imu_segs: Sequence) -> None:
        self.wsk_segs = _Segments(wsk_segs)
        self.imu_segs = _Segments(imu_segs)

    def wsk(self, t_rx: float, tus: int) -> float:
        return self.wsk_segs(t_rx, tus * 1e-6)

    def imu(self, t_rx: float, time_usec: int) -> float:
        return self.imu_segs(t_rx, time_usec * 1e-6)

    def restart(self) -> None:
        pass


def fit_segments(rx: "np.ndarray", src: "np.ndarray", min_n: int = 2) -> list[tuple]:
    """Split a source at its clock restarts (src going back) and envelope-fit each
    piece: [(rx_first, src0, a, b), ...] with t = a + b * (src - src0)."""
    import numpy as np

    from whisker_link.convert import envelope_fit

    rx = np.asarray(rx, dtype=np.float64)
    src = np.asarray(src, dtype=np.float64)
    if len(rx) == 0:
        return []
    cuts = np.nonzero(np.diff(src) < -1e-3)[0] + 1
    out = []
    for lo, hi in zip(np.r_[0, cuts], np.r_[cuts, len(rx)]):
        if hi - lo < min_n:
            continue
        s0, r0 = src[lo], rx[lo]
        # convert's 5 s envelope windows; shorter ones for a short piece (a slope needs 3+)
        # (a hair over span / 8, so the last sample doesn't make a one-point ninth window)
        win = min(5.0, max(0.25, float(src[hi - 1] - s0) / 8 * 1.001))
        a, b = envelope_fit(src[lo:hi] - s0, rx[lo:hi] - r0, win_s=win)
        out.append([float(rx[lo]), float(s0), float(a + r0), float(b)])
    return out
