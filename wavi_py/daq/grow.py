"""Rows that arrive one at a time, read as one array again and again.

A live stream's telemetry (``replay/rov.RovTrack``, the pressure of
``replay/events.EventTracker``) grows a row at a time for as long as the stream
runs, and its readers (every timeline bar once a second, a Pause's copy, an
Analysis window's) want all of it as arrays each time. Converting the whole
list on each read costs in proportion to the stream's age: an hour of the
vehicle's ~80 rows a second was ~0.1 s a read, on the GUI thread.

``Rows`` keeps the rows in one NumPy buffer that grows by doubling and the
newest ones in a list: ``append`` is a list append (the feeding thread), which
moves the list into the buffer every ``FOLD`` rows, and ``array()`` converts
what is still in the list, so a read costs at most ``FOLD`` rows however long
the stream has run and whenever it was last read. What it returns is a view of
rows that never change afterwards (later rows go behind it, or into a new
buffer), so a reader may keep it without copying.

Not locked: the owner's lock covers ``append`` and ``array``.
"""

from __future__ import annotations

import numpy as np

_MIN_ROWS = 1024
FOLD = 4096                      # rows kept as a list before they go into the buffer


class Rows:
    def __init__(self, ncol: int) -> None:
        self.ncol = int(ncol)
        self._buf = np.empty((0, self.ncol))
        self._n = 0                      # rows in the buffer
        self._new: list = []             # the newest rows, not in the buffer yet

    def append(self, row) -> None:
        """One row of ``ncol`` numbers."""
        self._new.append(row)
        if len(self._new) >= FOLD:
            self.array()

    def __len__(self) -> int:
        return self._n + len(self._new)

    def array(self) -> np.ndarray:
        """All rows so far, (N, ncol) float64: a view to read, not to write."""
        new = self._new
        if new:
            a = np.asarray(new, dtype=np.float64).reshape(len(new), self.ncol)
            n = self._n + len(a)
            if n > len(self._buf):
                buf = np.empty((max(_MIN_ROWS, 2 * n), self.ncol))
                buf[:self._n] = self._buf[:self._n]
                self._buf = buf
            self._buf[self._n:n] = a
            self._n = n
            self._new = []
        return self._buf[:self._n]
