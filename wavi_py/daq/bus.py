"""SampleBus — single-writer, many-reader publish/subscribe of sensor samples.

The DAQ thread publishes batches of (N, nch) float32 samples. Each subscriber
calls ``read_since(last_seq, ...)`` to pull whatever it hasn't seen yet. Each
subscriber owns its own ``last_seq`` cursor, so a slow consumer (the GUI) never
blocks a fast one (a control loop), and vice versa.

Storage is a fixed-capacity ring buffer. If a subscriber falls behind by more
than the capacity, the oldest unread samples are silently dropped — the caller
detects this by comparing ``new_seq - last_seq`` against ``len(samples)``.

A ``timed`` bus also keeps one float64 time per sample (seconds; the event
stream's session time, see ``daq/clock.py``): ``publish(samples, t)`` and
``read_since_t``. Samples published without times get NaN.
"""

from __future__ import annotations

import threading
from typing import Callable, Optional, Tuple

import numpy as np


class SampleBus:
    """Thread-safe pub/sub of (N, nch) float32 sample batches."""

    def __init__(self, capacity: int, nch: int, timed: bool = False):
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        if nch <= 0:
            raise ValueError("nch must be positive")
        self._capacity = int(capacity)
        self._nch = int(nch)
        self._buf = np.zeros((self._capacity, self._nch), dtype=np.float32)
        self._tbuf = np.full(self._capacity, np.nan) if timed else None
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._head = 0           # next write index in the ring
        self._total = 0          # total samples ever published (monotonic)
        self._closed = False
        self._meta: dict = {}    # free-form metadata (V0, Fs, nsensor, t0_unix, ...)

    # ----- properties

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def nch(self) -> int:
        return self._nch

    @property
    def timed(self) -> bool:
        return self._tbuf is not None

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def total_published(self) -> int:
        with self._lock:
            return self._total

    # ----- producer side

    def publish(self, samples: np.ndarray, t: Optional[np.ndarray] = None) -> int:
        """Append samples to the ring. Returns the new ``total_published``.

        ``samples`` must be shape ``(N, nch)`` and float32-compatible; ``t``
        (N,) their times, kept by a timed bus.
        """
        if samples.ndim != 2 or samples.shape[1] != self._nch:
            raise ValueError(
                f"expected (N, {self._nch}) array, got shape {samples.shape}"
            )
        n = int(samples.shape[0])
        if n == 0:
            return self._total
        if samples.dtype != np.float32:
            samples = samples.astype(np.float32, copy=False)

        with self._cv:
            if self._closed:
                raise RuntimeError("SampleBus is closed")

            bufs = [(self._buf, samples)]
            if self._tbuf is not None:
                bufs.append((self._tbuf, np.full(n, np.nan) if t is None
                             else np.asarray(t, dtype=np.float64)))
            head = self._head
            for buf, x in bufs:
                if n >= self._capacity:
                    # Caller dumped more than fits in the ring; keep only the tail.
                    np.copyto(buf, x[-self._capacity:])
                    self._head = 0
                else:
                    end = head + n
                    if end <= self._capacity:
                        np.copyto(buf[head:end], x)
                    else:
                        first = self._capacity - head
                        np.copyto(buf[head:], x[:first])
                        np.copyto(buf[: n - first], x[first:])
                    self._head = end % self._capacity

            self._total += n
            self._cv.notify_all()
            return self._total

    # ----- consumer side

    def read_since(
        self,
        last_seq: int,
        block: bool = False,
        timeout: Optional[float] = None,
    ) -> Tuple[Optional[np.ndarray], int]:
        """Pull all samples published after ``last_seq``.

        Returns ``(samples, new_seq)``:
          - ``samples`` is a fresh ``(N_new, nch)`` float32 array.
          - ``new_seq`` is the sequence number to pass on the next call.
          - If the caller fell behind by more than ``capacity``,
            ``new_seq - last_seq`` will exceed ``len(samples)``; the gap is
            samples that were dropped.
          - With ``block=False`` and no new data: returns ``(None, last_seq)``.
          - With ``block=True``: waits on the condition var up to ``timeout``
            seconds; on timeout or close, returns ``(None, last_seq)``.
        """
        with self._cv:
            if block and self._total <= last_seq and not self._closed:
                self._cv.wait_for(
                    lambda: self._total > last_seq or self._closed,
                    timeout=timeout,
                )
            if self._closed and self._total <= last_seq:
                return None, last_seq

            n_new = self._total - last_seq
            if n_new <= 0:
                return None, last_seq

            n_avail = min(n_new, self._capacity)
            return self._read_tail_unlocked(n_avail), self._total

    def read_since_t(
        self,
        last_seq: int,
        block: bool = False,
        timeout: Optional[float] = None,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        """``read_since`` plus the samples' times: ``(samples, t, new_seq)``
        (``t`` all NaN on an untimed bus)."""
        with self._cv:
            if block and self._total <= last_seq and not self._closed:
                self._cv.wait_for(
                    lambda: self._total > last_seq or self._closed,
                    timeout=timeout,
                )
            n_new = self._total - last_seq
            if n_new <= 0:
                return None, None, last_seq
            n_avail = min(n_new, self._capacity)
            t = (self._read_tail_unlocked(n_avail, self._tbuf) if self._tbuf is not None
                 else np.full(n_avail, np.nan))
            return self._read_tail_unlocked(n_avail), t, self._total

    def latest(self, n: int) -> np.ndarray:
        """Most recent up-to-``n`` samples regardless of cursor. Fresh subscribers."""
        with self._lock:
            n_avail = min(int(n), self._capacity, self._total)
            if n_avail <= 0:
                return np.empty((0, self._nch), dtype=np.float32)
            return self._read_tail_unlocked(n_avail)

    def _read_tail_unlocked(self, n_avail: int, buf: Optional[np.ndarray] = None) -> np.ndarray:
        """Return the last ``n_avail`` rows of ``buf`` (default the samples; caller
        holds the lock)."""
        buf = self._buf if buf is None else buf
        end = self._head
        start = (self._head - n_avail) % self._capacity
        out = np.empty((n_avail,) + buf.shape[1:], dtype=buf.dtype)
        if start < end:
            np.copyto(out, buf[start:end])
        else:
            first = self._capacity - start
            np.copyto(out[:first], buf[start:])
            np.copyto(out[first:], buf[:end])
        return out

    # ----- convenience

    def subscribe(
        self,
        callback: Callable[[np.ndarray, int], None],
        name: str = "subscriber",
        poll_timeout_s: float = 0.5,
    ) -> threading.Thread:
        """Spawn a daemon thread that calls ``callback(samples, new_seq)`` per batch.

        Exits cleanly when the bus is closed. Exceptions inside the callback
        are caught and printed so one buggy subscriber can't kill the loop.
        """
        def _loop() -> None:
            seq = 0
            while not self._closed:
                samples, seq = self.read_since(seq, block=True, timeout=poll_timeout_s)
                if samples is None:
                    continue
                try:
                    callback(samples, seq)
                except Exception as exc:  # noqa: BLE001
                    print(f"[SampleBus:{name}] callback error: {exc}")

        t = threading.Thread(target=_loop, name=f"SampleBus-{name}", daemon=True)
        t.start()
        return t

    # ----- metadata (V0, Fs, etc.)

    def set_metadata(self, **kwargs) -> None:
        with self._lock:
            self._meta.update(kwargs)
            self._cv.notify_all()

    def get_metadata(self) -> dict:
        with self._lock:
            return dict(self._meta)

    # ----- lifecycle

    def close(self) -> None:
        with self._cv:
            self._closed = True
            self._cv.notify_all()
