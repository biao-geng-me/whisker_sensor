"""What the uploads look like, from the ``SyncEvent`` stream (no Qt, no I/O).

``Tracker.apply(ev)`` keeps one ``RecRow`` per recording (its files, phase,
bytes, error, finish time) and the run's totals; the Record row's status line,
the Uploads window (``remote/uploads.py``) and the CLI's lines all read it, so
they say the same thing. It runs on the GUI thread (the controller's slot) or
the CLI's thread, never in two at once.

A run is one ``push_queue``: it starts with a ``("", "run")`` event (``n``
recordings) followed by a ``queued`` event per recording with its files; the
totals (recordings k of n, MB done of MB total, rate, ETA) are over that run.
Bytes "done" count a skipped file (already on the cluster) whole, so the bar
reaches 100 %, but the rate counts only what went over the link, measured
over the last ``RATE_WINDOW_S``.

``load_queue`` takes ``sync.queue_events`` (the persistent queue, read on a
thread): rows for what is queued but not in a run, e.g. after a restart or
offline. Finished rows (done, removed) stay until ``clear_finished``, the
newest ``KEEP_FINISHED`` of them.

Order (``order()``): the queue first (a run's recordings in run order, then
the rest by when they were queued), then the finished ones, newest first.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

from wavi_py.remote.sync import SyncEvent
from wavi_py.replay import recname

RATE_WINDOW_S = 10.0
KEEP_FINISHED = 200
ACTIVE = ("hashing", "uploading", "verifying")
FINISHED = ("done", "removed")


@dataclass
class FileRow:
    name: str
    size: int = 0
    phase: str = "queued"
    done: int = 0
    error: str = ""
    info: str = ""
    t_end: Optional[float] = None

    @property
    def done_bytes(self) -> int:
        if self.phase in ("done", "skipped"):
            return self.size
        if self.phase in ("uploading", "verifying"):
            return min(self.done, self.size)
        return 0

    @property
    def pct(self) -> int:
        """-1: no bar (queued, finished, failed); else 0..100 of the phase's
        bytes (hashing counts its own pass)."""
        if self.phase in ACTIVE and self.size:
            return int(100 * min(self.done, self.size) / self.size)
        return 100 if self.phase in ACTIVE else -1


@dataclass
class RecRow:
    stem: str
    files: dict = field(default_factory=dict)     # name -> FileRow
    phase: str = "queued"
    error: str = ""
    info: str = ""
    k: int = 0                                    # place in the current run, 0: not in it
    rdir: str = ""
    bytes_sent: int = 0
    t_added: float = field(default_factory=time.time)
    t_end: Optional[float] = None

    @property
    def size(self) -> int:
        return sum(f.size for f in self.files.values())

    @property
    def done_bytes(self) -> int:
        return sum(f.done_bytes for f in self.files.values())

    @property
    def pct(self) -> int:
        """-1 (no bar) unless it is being worked on: then bytes done of its size."""
        if self.phase in FINISHED:
            return -1
        if self.phase in ("uploading", "checking") or any(
                f.phase in ACTIVE for f in self.files.values()):
            return int(100 * self.done_bytes / self.size) if self.size else 0
        return -1

    @property
    def active(self) -> Optional[FileRow]:
        for f in self.files.values():
            if f.phase in ACTIVE:
                return f
        return None

    @property
    def all_skipped(self) -> bool:
        return bool(self.files) and all(f.phase == "skipped" for f in self.files.values())

    @property
    def label(self) -> str:
        """``2026-11-02 16:44  in_water_pilot2`` (the stem when it is no
        recording name)."""
        rn = recname.parse(self.stem)
        if rn is None:
            return self.stem
        return f"{rn.start:%Y-%m-%d %H:%M:%S}  {rn.tag}" + ("  (Pi)" if rn.source == "pi" else "")

    def status(self) -> str:
        """The word in the Status column."""
        if self.phase == "done":
            return "skipped (on the cluster)" if self.all_skipped else "done"
        f = self.active
        if f is not None:
            return f.phase
        return {"checking": "checking the cluster", "waiting": "waiting (in use)",
                "offline": "offline"}.get(self.phase, self.phase)


def fmt_mb(b: float) -> str:
    if b < 1e3:
        return f"{int(b)} B"
    if b < 100e3:
        return f"{b / 1e3:.1f} kB"
    return f"{b / 1e6:.1f} MB" if b < 1e9 else f"{b / 1e9:.2f} GB"


def fmt_eta(s: float) -> str:
    s = int(max(0, s))
    if s < 60:
        return f"{s} s"
    if s < 3600:
        return f"{s // 60}:{s % 60:02d}"
    return f"{s // 3600}:{s // 60 % 60:02d}:{s % 60:02d}"


class Tracker:
    def __init__(self):
        self.recs: dict[str, RecRow] = {}
        self.run: list[str] = []                  # stems of the current / last run, in order
        self.running = False
        self.run_t0 = self.run_t1 = 0.0
        self.sent = 0                              # bytes over the link this session
        self._samples: deque = deque()             # (monotonic, sent)
        self.version = 0                           # bumps when rows are added / removed

    # ----- input

    def _rec(self, stem: str) -> RecRow:
        r = self.recs.get(stem)
        if r is None:
            r = self.recs[stem] = RecRow(stem)
            self.version += 1
        return r

    def _files(self, r: RecRow, files) -> None:
        old = r.files
        r.files = {n: FileRow(n, int(sz)) for n, sz in files}
        for n, f in old.items():                   # a file that showed up mid-push (notes)
            if n not in r.files and f.phase not in ("queued",):
                r.files[n] = f

    def apply(self, ev: SyncEvent) -> Optional[RecRow]:
        """Fold one event in; returns the row it changed (None: a run event)."""
        if ev.stem == "":
            if ev.phase == "run":
                self.running, self.run, self.run_t0, self.run_t1 = True, [], time.time(), 0.0
                self._samples.clear()
            elif ev.phase == "run_end":
                self.running, self.run_t1 = False, time.time()
                for s in self.run:                 # whatever a stopped run left mid-way
                    r = self.recs.get(s)
                    if r is not None:
                        for f in r.files.values():
                            if f.phase in ACTIVE:
                                f.phase = "queued"
                        if r.phase in ACTIVE + ("checking",):
                            r.phase = "queued"
            return None
        r = self._rec(ev.stem)
        if ev.rdir:
            r.rdir = ev.rdir
        if ev.k:
            r.k = ev.k
        if not ev.file:
            return self._apply_rec(r, ev)
        f = r.files.get(ev.file)
        if f is None:
            f = r.files[ev.file] = FileRow(ev.file, ev.total)
        if ev.total:
            f.size = ev.total if ev.phase != "hashing" else f.size or ev.total
        if ev.phase == "uploading":
            if f.phase == "uploading" and ev.done > f.done:
                self._sent(ev.done - f.done)
            elif f.phase != "uploading":
                self._sent(ev.done)
            if r.phase not in ("uploading",):
                r.phase = "uploading"
        if ev.phase in ("hashing", "uploading", "verifying"):
            f.done = ev.done
        f.phase, f.error, f.info = ev.phase, ev.error, ev.info
        if ev.phase in ("done", "skipped", "failed"):
            f.t_end = ev.t
            if ev.phase == "done":
                f.done = f.size
        return r

    def _apply_rec(self, r: RecRow, ev: SyncEvent) -> RecRow:
        if ev.phase == "queued":
            if self.running and r.stem not in self.run:
                self.run.append(r.stem)
            r.phase, r.error, r.info, r.t_end, r.bytes_sent = "queued", "", "", None, 0
            if ev.files:
                self._files(r, ev.files)
            self.version += 1                      # it may move up from the finished ones
        elif ev.phase == "checking":
            r.phase = "checking"
            if ev.files:
                self._files(r, ev.files)
        else:
            r.phase, r.error = ev.phase, ev.error
            r.info = ev.info
            if ev.phase in FINISHED or ev.phase in ("failed", "offline"):
                r.t_end = ev.t
            if ev.phase not in ("uploading", "waiting"):
                for f in r.files.values():         # what the end left half-way goes again
                    if f.phase in ACTIVE:
                        f.phase, f.done = "queued", 0
            if ev.phase == "done":
                r.bytes_sent = ev.bytes_sent
            if ev.phase in FINISHED:
                self.version += 1
                self._trim()
        return r

    def load_queue(self, events: list[SyncEvent]) -> None:
        """The persistent queue (``sync.queue_events``). A row not in it and not
        finished went away (pushed or dequeued elsewhere): dropped. While a run
        is on, only new rows are added (the run's own events are fresher)."""
        stems = {e.stem for e in events}
        for e in events:
            r = self.recs.get(e.stem)
            if r is not None and (self.running or r.phase not in FINISHED):
                if not self.running and r.phase not in ACTIVE + ("checking",):
                    r.phase = e.phase if r.phase not in ("failed", "offline") else r.phase
                    r.error = r.error or e.error
                continue
            r = self._rec(e.stem)
            r.phase, r.error, r.t_added, r.t_end, r.k = e.phase, e.error, e.t, None, 0
            if e.files:
                self._files(r, e.files)
            self.version += 1
        if not self.running:
            for s in [s for s, r in self.recs.items()
                      if s not in stems and r.phase not in FINISHED]:
                del self.recs[s]
                self.version += 1

    def clear_finished(self) -> None:
        for s in [s for s, r in self.recs.items() if r.phase in FINISHED]:
            del self.recs[s]
        self.version += 1

    def _trim(self) -> None:
        fin = sorted((r for r in self.recs.values() if r.phase in FINISHED),
                     key=lambda r: r.t_end or 0)
        for r in fin[:max(0, len(fin) - KEEP_FINISHED)]:
            del self.recs[r.stem]

    def _sent(self, n: int) -> None:
        self.sent += max(0, n)
        now = time.monotonic()
        self._samples.append((now, self.sent))
        while len(self._samples) > 2 and now - self._samples[0][0] > RATE_WINDOW_S:
            self._samples.popleft()

    # ----- output

    def order(self) -> list[str]:
        run_pos = {s: i for i, s in enumerate(self.run)}
        live = [r for r in self.recs.values() if r.phase not in FINISHED]
        live.sort(key=lambda r: (0, run_pos[r.stem]) if r.stem in run_pos else (1, r.t_added))
        fin = sorted((r for r in self.recs.values() if r.phase in FINISHED),
                     key=lambda r: -(r.t_end or 0))
        return [r.stem for r in live + fin]

    def rate(self) -> float:
        """B/s over the link in the last ``RATE_WINDOW_S`` (0: idle or too early)."""
        if len(self._samples) < 2:
            return 0.0
        (t0, b0), (t1, b1) = self._samples[0], self._samples[-1]
        if time.monotonic() - t1 > 5.0 or t1 - t0 < 0.5:
            return 0.0
        return (b1 - b0) / (t1 - t0)

    def current(self) -> Optional[RecRow]:
        for s in self.run:
            r = self.recs.get(s)
            if r is not None and (r.active is not None or r.phase in ("checking", "uploading")):
                return r
        return None

    def totals(self) -> dict:
        """Over the current (or last) run: k of n, bytes done / total, rate, ETA (s
        or None)."""
        rows = [self.recs[s] for s in self.run if s in self.recs]
        cur = self.current()
        n_end = sum(r.phase in FINISHED + ("failed", "offline", "waiting") for r in rows)
        k = cur.k if cur is not None else n_end
        total = sum(r.size for r in rows)
        done = sum(r.size if r.phase == "done" else r.done_bytes for r in rows)
        rate = self.rate() if self.running else 0.0
        eta = (total - done) / rate if rate > 0 and total > done else None
        return {"k": k, "n": len(self.run), "done": done, "total": total, "rate": rate,
                "eta": eta, "running": self.running,
                "n_done": sum(r.phase == "done" for r in rows),
                "secs": (self.run_t1 or time.time()) - self.run_t0 if self.run_t0 else 0.0}

    def status_text(self) -> Optional[tuple[str, str]]:
        """(short, tooltip) for the Record row while a run is busy, else None:
        ``uploading 3/20 st_..._tag.jsonl 45%``."""
        if not self.running:
            return None
        cur = self.current()
        t = self.totals()
        if cur is None:
            return None
        f = cur.active
        kn = f"{cur.k}/{t['n']}"
        if f is None:
            short = f"checking {kn} {cur.stem}"
        elif f.phase == "verifying":
            short = f"verifying {kn} {f.name}"
        else:
            short = f"{f.phase} {kn} {f.name} {f.pct}%"
        tip = [f"{cur.stem}  (recording {kn})"]
        if f is not None:
            tip.append(f"{f.name}: {f.phase} {fmt_mb(f.done_bytes if f.phase != 'hashing' else f.done)}"
                       f" of {fmt_mb(f.size)}")
        tip.append(f"this run: {fmt_mb(t['done'])} of {fmt_mb(t['total'])}"
                   + (f", {t['rate'] / 1e6:.1f} MB/s" if t["rate"] else "")
                   + (f", ETA {fmt_eta(t['eta'])}" if t["eta"] is not None else ""))
        return short, "\n".join(tip)


class LinePrinter:
    """The CLI's view of the events: a line when a file starts, ends, is skipped
    or fails, and a percentage line at most every ``every_s`` (no curses)."""

    def __init__(self, out=print, every_s: float = 3.0):
        self.t = Tracker()
        self._out, self._every, self._last = out, every_s, 0.0

    def __call__(self, ev: SyncEvent) -> None:
        prev = self.t.recs.get(ev.stem)
        prev_phase = prev.files[ev.file].phase if prev and ev.file in prev.files else None
        r = self.t.apply(ev)
        if r is None or not ev.file:
            return
        kn = f"[{r.k}/{len(self.t.run) or ev.n}]"
        f = r.files[ev.file]
        if ev.phase != prev_phase and ev.phase in ("hashing", "uploading") \
                and prev_phase != "verifying":
            if ev.phase == "hashing" and f.size < 50e6:
                return                             # quick: not worth a line
            self._out(f"  {kn} {ev.file}: {ev.phase} {fmt_mb(f.size)}")
            self._last = time.monotonic()
        elif ev.phase in ("done", "skipped", "failed") and ev.phase != prev_phase:
            word = {"skipped": "already on the cluster", "failed": f"FAILED: {ev.error}",
                    "done": "done" + (f" ({ev.info})" if ev.info else "")}[ev.phase]
            self._out(f"  {kn} {ev.file}: {word}")
        elif ev.phase in ("hashing", "uploading") and time.monotonic() - self._last >= self._every:
            self._last = time.monotonic()
            t = self.t.totals()
            self._out(f"      {ev.file} {f.pct}%  {fmt_mb(f.done)} of {fmt_mb(f.size)}"
                      + (f"  {t['rate'] / 1e6:.1f} MB/s" if t["rate"] else "")
                      + (f"  run {fmt_mb(t['done'])} of {fmt_mb(t['total'])}"
                         if ev.phase == "uploading" else ""))
