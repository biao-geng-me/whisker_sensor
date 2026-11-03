"""Replay a daq_server recording (.jsonl) as if it were the live stream.

``ReplayTransport`` serves the recorded lines, as raw bytes, from
``read_some``, paced by the recording's own clock, so ``ServerReader`` and
everything after it (the buses, the views, the status line, the Fault log)
run as live. The file is indexed first (``replay/index.py``); the index gives
the byte offset of every line, so a seek costs one ``searchsorted``.

Pacing: the playhead is ``pos = anchor + speed * (clock() - wall_anchor)`` in
receive-clock seconds, and every line with ``t_play <= pos`` is due
(``t_play`` is the running maximum of ``t``, so the ~0.2 s backward steps of
the merged reader threads neither stall nor burst). ``speed=None`` serves as
fast as it is read (scripts, tests). ``clock`` can be swapped (tests).
``read_some`` waits at most ``WAIT_S`` for the next line, so pause, seek and
close act at once. A ``ReplayTransport`` is its own thread-safe control
surface: ``play`` / ``pause`` / ``toggle`` / ``speed`` / ``seek`` / ``set_loop``
/ ``position`` / ``state``.

A seek to T (session seconds) is a discontinuity in the stream, not a
reconnect. It emits, in order: ``{"k":"replay","op":"seek","gen":n,"pos":T}``
(``t`` is the receive clock, as in every event),
the hello, the newest info and status lines before ``T - preroll``, the lines
from ``T - preroll`` to T at full speed, ``{"k":"replay","op":"live",...}``,
then paced lines from T. The preroll (default ``t_buffer_s``) fills the
plot buffers and spectra the moment play resumes, as a live viewer at T had
them. The first seek (at open) also carries ``t0`` (session zero), ``unix0``
(its wall clock) and ``v0`` (the mean of the file's first 0.25 s of scans),
so the zero is the same wherever playback starts.

At the end without a loop, ``{"op":"eof"}`` is sent once, the state goes to
``eof`` and ``read_some`` keeps returning ``b""``: it never raises there,
because that would end ``ServerReader`` and read as a lost link. ``play``
after the end starts over from 0. With a loop set, reaching its end seeks to
its start (a new ``gen``), so the GUI sees the same thing as a user's seek.

Despike (``despike``, off by default): the whisker scans are served with the
index's spikes replaced (``daq/despike.py``, centred over the whole file;
``RecordingIndex.read_lines``). Switching it seeks to the playhead, so the
buffers refill with the other version at once.

Commands are refused (it is a recording): each JSON line written gets an ack
with ``ok: false``, except ``status`` (ok). Nothing is ever written.
"""

from __future__ import annotations

import collections
import json
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from wavi_py.config import DEFAULT_T_BUFFER_S
from wavi_py.replay.index import RecordingIndex
from wavi_py.transports.base import Transport
from whisker_link import wire

WAIT_S = 0.02                   # longest wait inside read_some
SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0)


class ReplayTransport(Transport):
    def __init__(self, index: RecordingIndex, *, speed: Optional[float] = 1.0,
                 start_s: float = 0.0, playing: bool = False,
                 preroll_s: float = DEFAULT_T_BUFFER_S, despike: bool = False,
                 clock: Callable[[], float] = time.monotonic):
        self.index = index
        self.preroll_s = float(preroll_s)
        self._clock = clock
        self._f = open(Path(index.path), "rb")
        self._cv = threading.Condition()
        self._closed = False
        self._speed = speed
        self._anchor = index.t0                  # receive-clock seconds
        self._wall = clock()
        self._cur = 0                            # next line to serve
        self._end_line = index.n_lines           # the loop end, else the file's
        self._pending: collections.deque = collections.deque()   # bytes, or (i0, i1)
        self._gen = 0
        self._loop: Optional[tuple[float, float]] = None
        self._state = "paused"
        self._first_seek = True
        self._despike = bool(despike)
        with self._cv:
            self._seek_locked(start_s, self.preroll_s)
            if playing:
                self._play_locked()

    # ----- control surface (any thread)

    @property
    def state(self) -> str:
        """playing / paused / eof"""
        with self._cv:
            return self._state

    @property
    def playing(self) -> bool:
        return self.state == "playing"

    @property
    def speed(self) -> Optional[float]:
        return self._speed

    @speed.setter
    def speed(self, v: Optional[float]) -> None:
        with self._cv:
            self._anchor = self._pos_locked()
            self._wall = self._clock()
            self._speed = None if v is None else float(v)
            self._cv.notify_all()

    @property
    def despike(self) -> bool:
        return self._despike

    @despike.setter
    def despike(self, on: bool) -> None:
        with self._cv:
            if bool(on) != self._despike:
                self._despike = bool(on)
                self._seek_locked(self._pos_locked() - self.index.t0, self.preroll_s)

    @property
    def position(self) -> float:
        """Session seconds of the playhead."""
        with self._cv:
            return self._pos_locked() - self.index.t0

    @property
    def duration(self) -> float:
        return self.index.duration

    @property
    def gen(self) -> int:
        return self._gen

    @property
    def loop(self) -> Optional[tuple[float, float]]:
        return self._loop

    def play(self) -> None:
        with self._cv:
            self._play_locked()

    def pause(self) -> None:
        with self._cv:
            if self._state == "playing":
                self._anchor = self._pos_locked()
                self._state = "paused"
            self._cv.notify_all()

    def toggle(self) -> None:
        with self._cv:
            if self._state == "playing":
                self._anchor = self._pos_locked()
                self._state = "paused"
            else:
                self._play_locked()
            self._cv.notify_all()

    def seek(self, t_s: float, preroll_s: Optional[float] = None) -> None:
        """Go to session time ``t_s``; keeps playing if it was."""
        with self._cv:
            self._seek_locked(t_s, self.preroll_s if preroll_s is None else preroll_s)

    def set_loop(self, t0_s: Optional[float], t1_s: Optional[float] = None) -> None:
        """Loop over [t0_s, t1_s) (session seconds); None clears it. Seeks into the
        loop if the playhead is outside."""
        with self._cv:
            if (t0_s is None or t1_s is None or t1_s <= t0_s
                    or self.index.line_at(t1_s) <= self.index.line_at(t0_s)):
                # no loop (also over a span with no line in it, e.g. inside a stall gap:
                # each wrap would seek and serve nothing, a hot loop)
                self._loop = None
                self._end_line = self.index.n_lines
            else:
                self._loop = (float(t0_s), float(t1_s))
                self._end_line = self.index.line_at(t1_s)
                p = self._pos_locked() - self.index.t0
                if not (t0_s <= p < t1_s):
                    self._seek_locked(t0_s, self.preroll_s)
            self._cv.notify_all()

    # ----- Transport

    def read_some(self, max_n: int) -> bytes:
        with self._cv:
            while True:
                if self._closed:
                    raise ConnectionError("replay closed")
                if self._pending:
                    return self._pop_pending(max_n)
                if self._cur >= self._end_line:
                    if self._loop is not None and self._state == "playing":
                        self._seek_locked(self._loop[0], self.preroll_s)
                        continue
                    if self._cur >= self.index.n_lines and self._state != "eof":
                        self._state = "eof"
                        self._anchor = float(self.index.t_play[-1]) if self.index.n_lines else 0
                        return self._marker("eof")
                    self._cv.wait(WAIT_S)
                    return b""
                if self._state != "playing":
                    self._cv.wait(WAIT_S)
                    return b""
                if self._speed is None:
                    j = self._end_line
                else:
                    j = min(self._end_line, int(self.index.t_play.searchsorted(
                        self._pos_locked(), side="right")))
                if j > self._cur:
                    return self._serve(self._cur, j, max_n)
                due = (self.index.t_play[self._cur] - self._pos_locked()) / self._speed
                self._cv.wait(min(max(due, 0.0), WAIT_S))
                return b""

    def read_exact(self, n: int) -> bytes:
        raise NotImplementedError("the replay transport is an event stream: use read_some")

    def write(self, data: bytes) -> None:
        with self._cv:
            for line in data.splitlines():
                try:
                    cmd = json.loads(line)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                name = cmd.get("cmd") if isinstance(cmd, dict) else None
                ack = {"k": "ack", "t": self._pos_locked(), "cmd": name, "ok": name == "status"}
                if name != "status":
                    ack["error"] = "replay: read-only"
                self._pending.append(wire.dumps(ack))
            self._cv.notify_all()

    def bytes_available(self) -> int:
        return 0

    def close(self) -> None:
        with self._cv:
            if self._closed:
                return
            self._closed = True
            self._cv.notify_all()
        try:
            self._f.close()
        except OSError:
            pass

    @property
    def is_open(self) -> bool:
        return not self._closed

    # ----- internals (caller holds the lock)

    def _pos_locked(self) -> float:
        """Receive-clock seconds of the playhead."""
        idx = self.index
        if self._state != "playing":
            return self._anchor
        if self._speed is None:
            return float(idx.t_play[self._cur - 1]) if self._cur > 0 else idx.t0
        p = self._anchor + self._speed * (self._clock() - self._wall)
        end = (idx.t0 + self._loop[1]) if self._loop is not None else (
            float(idx.t_play[-1]) if idx.n_lines else idx.t0)
        return min(p, end)

    def _play_locked(self) -> None:
        if self._state == "eof":
            self._seek_locked(self._loop[0] if self._loop else 0.0, self.preroll_s)
        if self._state != "playing":
            self._wall = self._clock()
            self._state = "playing"
        self._cv.notify_all()

    def _seek_locked(self, t_s: float, preroll_s: float) -> None:
        idx = self.index
        t_s = min(max(0.0, float(t_s)), idx.duration)
        self._gen += 1
        self._pending.clear()
        i_start = idx.line_at(t_s - max(0.0, preroll_s))
        i_end = idx.line_at(t_s)
        extra = {}
        if self._first_seek:
            self._first_seek = False
            extra = {"t0": idx.t0, "unix0": idx.unix0, "file": Path(idx.path).name}
            if len(idx.v0):
                extra["v0"] = [round(float(v), 6) for v in idx.v0]
        self._pending.append(self._marker("seek", t_s, **extra))
        state_lines = [idx.hello_line]
        for lines in (idx.info_lines, idx.status_lines):
            j = idx.last_before(lines, i_start)
            if j is not None and j != idx.hello_line:
                state_lines.append(j)
        for j in sorted(state_lines):
            self._pending.append((j, j + 1))
        lo = max(i_start, idx.hello_line + 1)
        if i_end > lo:
            self._pending.append((lo, i_end))
        self._pending.append(("live", t_s))
        self._cur = max(i_end, idx.hello_line + 1)
        self._anchor = idx.t0 + t_s
        self._wall = self._clock()
        if self._state == "eof":
            self._state = "paused"
        self._cv.notify_all()

    def _pop_pending(self, max_n: int) -> bytes:
        item = self._pending[0]
        if isinstance(item, bytes):
            self._pending.popleft()
            return item
        if item[0] == "live":
            self._pending.popleft()
            # the clock starts at T now that the preroll is in
            self._anchor = self.index.t0 + item[1]
            self._wall = self._clock()
            return self._marker("live", item[1])
        i0, i1 = item
        j = self._fit(i0, i1, max_n)
        if j >= i1:
            self._pending.popleft()
        else:
            self._pending[0] = (j, i1)
        return self.index.read_lines(self._f, i0, j, self._despike)

    def _serve(self, i0: int, i1: int, max_n: int) -> bytes:
        j = self._fit(i0, i1, max_n)
        self._cur = j
        return self.index.read_lines(self._f, i0, j, self._despike)

    def _fit(self, i0: int, i1: int, max_n: int) -> int:
        """The end of lines i0.. that fit in max_n bytes (at least one line)."""
        off = self.index.off
        j = int(off.searchsorted(off[i0] + max_n, side="right")) - 1
        return min(i1, max(i0 + 1, j))

    def _marker(self, op: str, t_s: Optional[float] = None, **extra) -> bytes:
        if t_s is None:
            t_s = self._pos_locked() - self.index.t0
        return wire.dumps({"k": "replay", "op": op, "gen": self._gen,
                           "t": self.index.t0 + float(t_s), "pos": round(float(t_s), 6),
                           **extra})
