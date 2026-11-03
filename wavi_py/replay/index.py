"""Byte-offset index of a daq_server recording (.jsonl), for replay and seeking.

One pass over the file, pulling a few fields from the bytes of each line
without ``json.loads`` (only the rare lines that feed the event list, the
depth and the seek state are parsed):

- per line: byte offset, the receive time ``t``, the kind, and ``t_play``, the
  running maximum of ``t`` (daq_server merges its ESP32 and MAVLink threads,
  so ``t`` steps back by up to ~0.2 s in file order; pacing uses ``t_play``);
- the device clocks fitted onto the receive clock (``daq/clock.py``): ESP32
  ``tus`` for the whisker scans, RAW_IMU ``time_usec`` for the IMU;
- the scan rate (from ``tus``), and V0 (the mean of the first 0.25 s of scans;
  a clip, ``replay/export.py``, carries its source's V0 and surface pressure);
- the spikes of the whisker scans (``daq/despike.py``, every window centred):
  line, gauge and replacement of each, sparse (~30 in a 13 min dive), so the
  replay transport can serve the scans raw or despiked (``read_lines``);
- the line numbers of every hello / info / status line (the state a seek
  restores) and the events and depth (``replay/events.py``).

The kinds are written two ways: ``"k": "hello"`` (``json.dumps`` default
spacing; the GUI's tee writes the hello like that) and ``"k":"wsk"``
(``wire.dumps``, compact). Both are read. A truncated last line (a recording
cut by a power loss) is dropped; a file without a hello is refused.

Session time zero (``t0``) is the first event after the hello: a recording
the GUI tees re-stamps the hello at record start, but one written on the Pi
keeps the hello of the connection, which can be minutes earlier. The wall
clock of receive time t is ``hello.unix + (t - hello.mono)``.

The index is cached as ``<file>.idx.npz`` next to the recording (keyed by
size, mtime and ``INDEX_VERSION``), or in the user cache directory
(``%LOCALAPPDATA%/wavi_py`` on Windows, ``~/.cache/wavi_py`` elsewhere) when
the recording's folder is read-only.

``open`` also takes a ``.dat`` (no ``.jsonl``): whisker-only, converted once
into a stream in the user cache (``replay/dat_source.py``).
"""

from __future__ import annotations

import hashlib
import json
import os
import sys
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from wavi_py.daq.clock import FittedClock, fit_segments
from wavi_py.daq.despike import find_spikes
from wavi_py.replay.events import WATER_DENSITY, Event, EventTracker, TimelineQueries, hpa_per_cm
from whisker_link.esp32_frame import MV_PER_CODE

INDEX_VERSION = 2
V0_S = 0.25                    # V0 = mean of this much of the first scans

KINDS = ("other", "hello", "wsk", "mav", "status", "info", "txt", "mark", "ack")
K = {name: i for i, name in enumerate(KINDS)}
_KIND_BYTES = {name.encode(): i for i, name in enumerate(KINDS)}
_PARSED_MAV = (b"HEARTBEAT", b"STATUSTEXT", b"SCALED_PRESSURE2")
# hello.server of the GUI's own transports: no vehicle (no IMU) in those recordings
LOCAL_SERVERS = ("wavi_py-esp32", "wavi_py-sim", "wavi_py-dat")


class NotARecording(ValueError):
    """The file is not a daq_server recording."""


def _int_after(line: bytes, key: bytes, start: int = 0) -> Optional[int]:
    i = line.find(key, start)
    if i < 0:
        return None
    i += len(key)
    j = i
    n = len(line)
    while j < n and line[j] not in b",}":
        j += 1
    try:
        return int(line[i:j])
    except ValueError:
        return None


def _cache_dir() -> Path:
    if sys.platform.startswith("win"):
        base = os.environ.get("LOCALAPPDATA") or str(Path.home() / "AppData" / "Local")
    else:
        base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    return Path(base) / "wavi_py"


def cache_paths(path: Path) -> list[Path]:
    """Where the index of ``path`` is cached: next to it, else the user cache."""
    path = Path(path).resolve()
    tag = hashlib.sha1(str(path).encode("utf-8", "surrogatepass")).hexdigest()[:12]
    return [path.with_name(path.name + ".idx.npz"),
            _cache_dir() / f"{path.stem}.{tag}.idx.npz"]


class RecordingIndex(TimelineQueries):
    """Everything replay needs to serve and seek a recording. Build with
    ``RecordingIndex.open(path)`` (cached) or ``RecordingIndex.build(path)``."""

    def __init__(self) -> None:
        self.path: Path = Path()
        self.size = 0
        self.mtime_ns = 0
        self.off = np.empty(0, np.int64)       # n + 1: line starts, then the end of the last line
        self.t = np.empty(0, np.float64)
        self.t_play = np.empty(0, np.float64)
        self.kind = np.empty(0, np.uint8)
        self.hello: dict = {}
        self.hello_line = 0
        self.info_lines = np.empty(0, np.int64)
        self.status_lines = np.empty(0, np.int64)
        self.t0 = 0.0                          # session zero (receive clock)
        self.fs = 0.0                          # scan rate from tus (0: no scans)
        self.v0 = np.empty(0, np.float32)      # mV, first V0_S of scans
        self.wsk_segs: list = []               # clock fits, see daq/clock.py
        self.imu_segs: list = []
        self.n_wsk = 0
        self.n_raw_imu = 0
        self.spk_line = np.empty(0, np.int64)  # spikes: line, gauge, replacement (raw units)
        self.spk_ch = np.empty(0, np.int64)
        self.spk_val = np.empty(0, np.float64)
        self.events: list[Event] = []
        self.armed_spans: list[list[float]] = []   # session seconds
        self.press_t = np.empty(0)                 # session seconds
        self.press_hpa = np.empty(0)
        self.surface_hpa: Optional[float] = None
        self.from_cache = False

    # ----- construction

    @classmethod
    def open(cls, path, progress: Optional[Callable[[float], None]] = None,
             use_cache: bool = True) -> "RecordingIndex":
        path = Path(path)
        if path.suffix.lower() == ".dat":           # whisker only, via a converted stream
            from wavi_py.replay.dat_source import ensure_jsonl
            outer = progress
            path = ensure_jsonl(path, None if outer is None else (lambda p: outer(0.5 * p)))
            progress = None if outer is None else (lambda p: outer(0.5 + 0.5 * p))
        st = path.stat()
        if use_cache:
            for c in cache_paths(path):
                idx = cls._load_cache(c, path, st)
                if idx is not None:
                    return idx
        idx = cls.build(path, progress)
        if use_cache:
            for c in cache_paths(path):
                try:
                    c.parent.mkdir(parents=True, exist_ok=True)
                    idx._save_cache(c)
                    break
                except OSError:
                    continue
        return idx

    @classmethod
    def build(cls, path, progress: Optional[Callable[[float], None]] = None) -> "RecordingIndex":
        path = Path(path)
        st = path.stat()
        self = cls()
        self.path, self.size, self.mtime_ns = path, st.st_size, st.st_mtime_ns
        off: list[int] = []
        ts: list[float] = []
        kinds: list[int] = []
        wsk_t: list[float] = []
        wsk_tus: list[int] = []
        wsk_seq: list[int] = []
        imu_t: list[float] = []
        imu_us: list[int] = []
        v0_rows: list[list[int]] = []
        raw_parts: list[bytes] = []            # the "raw" list of every wsk line, as text
        tracker = EventTracker()
        hello = None
        pos = 0
        next_report = 0
        step = max(1 << 20, self.size // 200)
        k_wsk, k_mav, k_other = K["wsk"], K["mav"], K["other"]
        with open(path, "rb") as f:
            for line in f:
                n = len(line)
                if line[-1:] != b"\n":
                    break                               # truncated last line
                start, pos = pos, pos + n
                if progress is not None and pos >= next_report:
                    progress(pos / max(1, self.size))
                    next_report = pos + step
                if line.startswith(b'{"k":"wsk"'):
                    kind, ti = k_wsk, 10
                elif line.startswith(b'{"k":"mav"'):
                    kind, ti = k_mav, 10
                else:
                    i = line.find(b'"k":')
                    if i < 0:
                        continue
                    j = line.find(b'"', line.find(b'"', i + 4) + 1)
                    kind = _KIND_BYTES.get(line[line.find(b'"', i + 4) + 1:j], k_other)
                    ti = j
                i = line.find(b'"t":', ti)
                if i < 0:
                    continue
                e = line.find(b",", i)
                try:
                    t = float(line[i + 4:e if e > 0 else -2])
                except ValueError:
                    continue
                if kind == k_wsk:
                    seq = _int_after(line, b'"seq":', e)
                    tus = _int_after(line, b'"tus":', e)
                    if seq is not None and tus is not None:
                        wsk_t.append(t)
                        wsk_tus.append(tus)
                        wsk_seq.append(seq)
                        tracker.wsk_seq(t, seq)
                    if hello is not None and len(v0_rows) < 512:
                        v0_rows.append(json.loads(line)["raw"])
                    r0 = line.find(b'"raw":[', e)
                    raw_parts.append(line[r0 + 7:line.find(b"]", r0)] if r0 >= 0 else b"")
                elif kind == k_mav:
                    ty = line.find(b'"type":"', e)
                    if ty < 0:
                        continue
                    tend = line.find(b'"', ty + 8)
                    mtype = line[ty + 8:tend]
                    if mtype == b"RAW_IMU":
                        us = _int_after(line, b'"time_usec":', tend)
                        if us is not None:
                            imu_t.append(t)
                            imu_us.append(us)
                    elif mtype in _PARSED_MAV:
                        tracker.feed(json.loads(line))
                elif kind != k_other:
                    ev = json.loads(line)
                    if kind == K["hello"]:
                        if hello is None:
                            hello = ev
                            self.hello_line = len(off)
                    else:
                        tracker.feed(ev)
                off.append(start)
                ts.append(t)
                kinds.append(kind)
        if progress is not None:
            progress(1.0)
        if hello is None:
            raise NotARecording(f"{path}: no hello line, not a daq_server recording")
        self.off = np.asarray(off + [pos], dtype=np.int64)
        self.t = np.asarray(ts, dtype=np.float64)
        self.t_play = np.maximum.accumulate(self.t)
        self.kind = np.asarray(kinds, dtype=np.uint8)
        self.hello = hello
        self.info_lines = np.nonzero(self.kind == K["info"])[0]
        self.status_lines = np.nonzero(self.kind == K["status"])[0]
        first = self.hello_line + 1
        self.t0 = float(self.t[first]) if first < len(self.t) else float(hello.get("t", 0.0))
        self.n_wsk, self.n_raw_imu = len(wsk_t), len(imu_t)
        if len(wsk_t) >= 2:
            tus = np.asarray(wsk_tus, dtype=np.float64)
            seq = np.asarray(wsk_seq, dtype=np.float64)
            ok = (np.diff(tus) > 0) & (np.diff(seq) > 0)
            if ok.any():
                self.fs = float(np.diff(seq)[ok].sum() * 1e6 / np.diff(tus)[ok].sum())
            self.wsk_segs = fit_segments(np.asarray(wsk_t), tus * 1e-6)
        if len(imu_t) >= 2:
            self.imu_segs = fit_segments(np.asarray(imu_t), np.asarray(imu_us, np.float64) * 1e-6)
        if v0_rows:
            nv0 = max(1, int(round((self.fs or 80.0) * V0_S)))
            mv = float(hello.get("mv_per_code", MV_PER_CODE))
            self.v0 = (np.asarray(v0_rows[:nv0], dtype=np.float64).mean(axis=0) * mv
                       ).astype(np.float32)
        self._find_spikes(raw_parts)
        self._take_events(tracker)
        clip = hello.get("clip")
        if isinstance(clip, dict):          # a clip (replay/export.py): the source's zero, surface
            if clip.get("v0"):
                self.v0 = np.asarray(clip["v0"], dtype=np.float32)
            if clip.get("surface_hpa") is not None:
                self.surface_hpa = float(clip["surface_hpa"])
        return self

    def _find_spikes(self, raw_parts: list[bytes]) -> None:
        """The spikes of the scans (``daq/despike.py``), by line. Skipped when the
        scans do not all have the same gauges."""
        n = len(raw_parts)
        nch = raw_parts[0].count(b",") + 1 if n and raw_parts[0] else 0
        if n < 2 or not nch:
            return
        x = np.fromstring(b",".join(raw_parts), dtype=np.float64, sep=",")
        if len(x) != n * nch:
            print(f"[RecordingIndex] {self.path.name}: scans of mixed width, no despike")
            return
        rows, cols, vals = find_spikes(x.reshape(n, nch))
        lines = np.nonzero(self.kind == K["wsk"])[0]
        self.spk_line, self.spk_ch, self.spk_val = lines[rows], cols, vals

    def _take_events(self, tr: EventTracker) -> None:
        t0 = self.t0
        self.events = []
        for e in tr.sorted_events():
            e.t -= t0
            if e.t_last is not None:
                e.t_last -= t0
            self.events.append(e)
        end = self.duration
        self.armed_spans = [[a - t0, (b - t0) if b is not None else end]
                            for a, b in tr.armed_spans]
        self.press_t = np.asarray(tr.press_t, dtype=np.float64) - t0
        self.press_hpa = np.asarray(tr.press_hpa, dtype=np.float64)
        self.surface_hpa = tr.surface_hpa()

    # ----- cache

    def _save_cache(self, path: Path) -> None:
        meta = {"version": INDEX_VERSION, "size": self.size, "mtime_ns": self.mtime_ns,
                "hello": self.hello, "hello_line": self.hello_line, "t0": self.t0,
                "fs": self.fs, "wsk_segs": self.wsk_segs, "imu_segs": self.imu_segs,
                "n_wsk": self.n_wsk, "n_raw_imu": self.n_raw_imu,
                "events": [e.to_dict() for e in self.events],
                "armed_spans": self.armed_spans, "surface_hpa": self.surface_hpa}
        tmp = path.with_name(path.name + ".tmp")
        with open(tmp, "wb") as f:
            np.savez(f, off=self.off, t=self.t, kind=self.kind, v0=self.v0,
                     press_t=self.press_t, press_hpa=self.press_hpa,
                     spk_line=self.spk_line, spk_ch=self.spk_ch, spk_val=self.spk_val,
                     meta=np.asarray(json.dumps(meta)))
        os.replace(tmp, path)

    @classmethod
    def _load_cache(cls, cache: Path, path: Path, st) -> Optional["RecordingIndex"]:
        try:
            with np.load(cache, allow_pickle=False) as z:
                meta = json.loads(str(z["meta"]))
                if (meta.get("version") != INDEX_VERSION or meta.get("size") != st.st_size
                        or meta.get("mtime_ns") != st.st_mtime_ns):
                    return None
                self = cls()
                self.off, self.t, self.kind = z["off"], z["t"], z["kind"]
                self.v0, self.press_t, self.press_hpa = z["v0"], z["press_t"], z["press_hpa"]
                self.spk_line, self.spk_ch, self.spk_val = z["spk_line"], z["spk_ch"], z["spk_val"]
        except (OSError, ValueError, KeyError):
            return None
        self.path, self.size, self.mtime_ns = path, st.st_size, st.st_mtime_ns
        self.t_play = np.maximum.accumulate(self.t)
        for name in ("hello", "hello_line", "t0", "fs", "wsk_segs", "imu_segs", "n_wsk",
                     "n_raw_imu", "armed_spans", "surface_hpa"):
            setattr(self, name, meta[name])
        self.events = [Event(**e) for e in meta["events"]]
        self.info_lines = np.nonzero(self.kind == K["info"])[0]
        self.status_lines = np.nonzero(self.kind == K["status"])[0]
        self.from_cache = True
        return self

    # ----- queries

    @property
    def n_lines(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        """Session seconds of the last line."""
        return float(self.t_play[-1] - self.t0) if len(self.t_play) else 0.0

    @property
    def has_vehicle(self) -> bool:
        """An ROV recording (daq_server, with MAVLink), not the GUI's ESP32 / Sim tee."""
        return str(self.hello.get("server", "")) not in LOCAL_SERVERS

    @property
    def unix0(self) -> float:
        """Wall clock (unix seconds) of session time zero."""
        h = self.hello
        return float(h.get("unix", 0.0)) + (self.t0 - float(h.get("mono", h.get("t", self.t0))))

    def clock(self) -> FittedClock:
        return FittedClock(self.wsk_segs, self.imu_segs)

    def line_at(self, t_sess: float) -> int:
        """The first line whose (paced) time is after session time ``t_sess``."""
        return int(np.searchsorted(self.t_play, self.t0 + t_sess, side="right"))

    def last_before(self, lines: np.ndarray, i: int) -> Optional[int]:
        """The newest of ``lines`` (sorted line numbers) before line ``i``."""
        j = int(np.searchsorted(lines, i)) - 1
        return int(lines[j]) if j >= 0 else None

    def read_lines(self, f, i0: int, i1: int, despike: bool = False) -> bytes:
        """Bytes of lines [i0, i1) from the open (binary) file ``f``; with
        ``despike``, the scans with spikes rewritten with their replacements."""
        if i1 <= i0:
            return b""
        f.seek(int(self.off[i0]))
        data = f.read(int(self.off[i1] - self.off[i0]))
        if not despike:
            return data
        s0, s1 = np.searchsorted(self.spk_line, [i0, i1])
        if s0 == s1:
            return data
        base = int(self.off[i0])
        out, at = [], 0
        lines, chs, vals = self.spk_line[s0:s1], self.spk_ch[s0:s1], self.spk_val[s0:s1]
        for ln in np.unique(lines):
            a, b = int(self.off[ln]) - base, int(self.off[ln + 1]) - base
            ev = json.loads(data[a:b])
            raw = ev["raw"]
            is_int = isinstance(raw[0], int)
            for c, v in zip(chs[lines == ln], vals[lines == ln]):
                raw[int(c)] = int(round(v)) if is_int else round(float(v), 6)
            out += [data[at:a], (json.dumps(ev, separators=(",", ":")) + "\n").encode()]
            at = b
        out.append(data[at:])
        return b"".join(out)

    def despiked_scan(self, line: int, raw: list) -> list:
        """The ``raw`` list of scan ``line`` with its spikes replaced (in place)."""
        s0, s1 = np.searchsorted(self.spk_line, [line, line + 1])
        for c, v in zip(self.spk_ch[s0:s1], self.spk_val[s0:s1]):
            raw[int(c)] = v
        return raw

    @property
    def n_spikes(self) -> int:
        return len(self.spk_line)

    def depth(self, rho: float = WATER_DENSITY) -> tuple[np.ndarray, np.ndarray]:
        """(session seconds, depth cm) from the external pressure; empty without it."""
        if self.surface_hpa is None or not len(self.press_t):
            return np.empty(0), np.empty(0)
        return self.press_t, (self.press_hpa - self.surface_hpa) / hpa_per_cm(rho)

    def summary(self) -> str:
        n_ev = len(self.events)
        return (f"{self.path.name}: {self.duration:.1f} s, {self.n_lines} lines, "
                f"{self.n_wsk} scans at {self.fs:.2f} Hz, {self.n_raw_imu} RAW_IMU, "
                f"{n_ev} events, {self.n_spikes} spikes" + (" (cached index)" if self.from_cache else ""))
