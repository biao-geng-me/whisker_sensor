"""The newest ``n`` rows of a stream, always one contiguous array, appended in
amortized O(rows appended): the GUI's live whisker buffer.

A plain "shift left, append" buffer moves all ``n`` rows on every append (5 min
of 32 gauges at 250 Hz is 9.6 MB per tick). A ring with a wrapping write
pointer moves nothing, but its newest ``n`` rows are then two pieces, and every
reader (``buf[-n:]``, the spectrogram's fancy indices, the Line window's
in-place read, ``series()``) would have to unwrap them.

``Window`` keeps both: the rows live in storage ``n + slack`` long, the window
is ``storage[w - n:w]`` (a view, as a shifted buffer would be), an append
writes its rows at ``w`` and moves ``w``; when they don't fit, the newest
rows are first copied back to the start (a compaction: ``n`` rows once every
``slack`` appended rows; with ``slack = n``, one 9.6 MB copy per 5 min instead
of one per tick).

Several arrays with the same rows (samples and their times) share the pointer,
so their windows always line up. A view taken from ``window()`` stays the
newest rows until the next append; after that it still holds valid rows of the
stream until a compaction overwrites the start of the storage, so a reader that
keeps data across appends must copy it (the Line window does when paused;
``MainWindow.series()`` hands out a copy).
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


class Window:
    def __init__(self, n: int, specs: Sequence[tuple], slack: int | None = None,
                 fill=np.nan) -> None:
        """``specs``: (row shape, dtype) of each array, e.g. ((32,), np.float32), ((), float)."""
        self.n = int(n)
        self.slack = max(1, int(self.n if slack is None else slack))
        self._bufs = [np.full((self.n + self.slack, *shape), fill, dtype=dtype)
                      for shape, dtype in specs]
        self._w = self.n
        self.compactions = 0

    def window(self, i: int) -> np.ndarray:
        """Array ``i``'s newest ``n`` rows (a view; the oldest first)."""
        return self._bufs[i][self._w - self.n:self._w]

    def append(self, *rows: np.ndarray) -> None:
        """Append the same number of rows to every array (oldest rows drop out)."""
        k = len(rows[0])
        if k == 0:
            return
        if k >= self.n:
            for b, r in zip(self._bufs, rows):
                b[:self.n] = r[-self.n:]
            self._w = self.n
            return
        if self._w + k > len(self._bufs[0]):
            keep = self.n - k                  # the rows that stay in the window
            for b in self._bufs:
                b[:keep] = b[self._w - keep:self._w]
            self._w = keep
            self.compactions += 1
        for b, r in zip(self._bufs, rows):
            b[self._w:self._w + k] = r
        self._w += k

    def fill(self, value) -> None:
        for b in self._bufs:
            b.fill(value)

    @property
    def nbytes(self) -> int:
        return sum(b.nbytes for b in self._bufs)
