"""The whole recording in memory, random access: ``RecordingStore``.

Replay serves the file through the live pipeline (``transports/
replay_transport.py``), which only ever holds the newest ``t_buffer_s``. The
Line and IMU windows instead draw any span of the recording, the full length
included, and follow a scrubbed playhead at mouse rate, from this store
(``daq/series.py`` is the read API; pick-and-compare and select-for-FFT read
it too):

- ``WSK``: every whisker scan, mV at the ADC (``raw * mv_per_code`` in float32,
  as ``ServerReader`` makes them), at its fitted time in session seconds (the
  index's ``FittedClock`` minus ``t0``, vectorized, the same arithmetic);
  ``despike=True`` gives the scans with the index's spikes replaced, rounded
  as the transport rewrites them (a lazily made copy);
- ``IMU``: every RAW_IMU row (accel, gyro) with the newest ATTITUDE before it
  in file order (roll, pitch, yaw), ``IMU_CHANNELS``, at its fitted time.

So the values are bit for bit what the pipeline delivers for the same span
(the tests check it), except the attitude of the first IMU rows after a seek,
which the pipeline holds over from before the seek.

Drawing: ``envelope`` reads a min / max pyramid (blocks of 16, 64, 256, ...
samples, aligned to the first scan), so a draw of the whole recording costs
about what a draw of 30 s does: the coarsest level with blocks no longer than
a pixel column's samples, grouped to one min / max pair per column.

Build: one pass over the file's scan and MAVLink lines (the index's offsets
and kinds), pulling the fields from the bytes; cached as
``<file>.store.npz`` next to the recording or in the user cache, as the index
(``replay/index.py``), keyed by size, mtime and ``STORE_VERSION``.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from wavi_py.daq.series import IMU, WSK, Cols, Series, span_index
from wavi_py.daq.server_reader import IMU_CHANNELS
from wavi_py.replay.index import K, RecordingIndex, _int_after, cache_paths
from whisker_link.esp32_frame import MV_PER_CODE

STORE_VERSION = 1
PYR_BASE = 16                   # the finest pyramid level's block (samples)
PYR_FACTOR = 4                  # each level's blocks are this many of the one below
_CHUNK_LINES = 50_000           # lines read at once while building
_IMU_FIELDS = (b'"xacc":', b'"yacc":', b'"zacc":', b'"xgyro":', b'"ygyro":', b'"zgyro":')
_ATT_FIELDS = (b'"roll":', b'"pitch":', b'"yaw":')


def _num_after(line: bytes, key: bytes, start: int = 0):
    """The JSON number after ``key`` (int or float as written; None if absent)."""
    i = line.find(key, start)
    if i < 0:
        return None
    i += len(key)
    j = i
    n = len(line)
    while j < n and line[j] not in b",}":
        j += 1
    s = line[i:j]
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return None


def map_segments(segs: list, rx: np.ndarray, src: np.ndarray, tol: float = 1.0) -> np.ndarray:
    """``daq.clock._Segments`` over arrays: each sample on the fit of the segment it
    falls in by receive time (that one, the one before, the one after, the first
    within ``tol`` s of the receive time), else its receive time. NaN ``src``
    (no source clock) gives the receive time."""
    out = rx.copy()
    if not segs or not len(rx):
        return out
    s = np.asarray(segs, dtype=np.float64)
    starts, s0, a, b = s[:, 0], s[:, 1], s[:, 2], s[:, 3]
    i = np.searchsorted(starts, rx + tol, side="right") - 1
    todo = np.isfinite(src)
    for d in (0, -1, 1):
        j = i + d
        ok = todo & (j >= 0) & (j < len(s))
        k = np.nonzero(ok)[0]
        jj = j[k]
        t = a[jj] + b[jj] * (src[k] - s0[jj])
        good = np.abs(t - rx[k]) <= tol
        out[k[good]] = t[good]
        todo[k[good]] = False
    return out


def _pyramid(y: np.ndarray) -> list[tuple[int, np.ndarray, np.ndarray]]:
    """[(block, mins, maxs), ...], finest first; blocks aligned to row 0, the last
    one partial."""
    levels = []
    b, lo, hi, step = PYR_BASE, y, y, PYR_BASE
    while len(lo) > 2 * step:
        starts = np.arange(0, len(lo), step)
        lo = np.minimum.reduceat(lo, starts, axis=0)
        hi = np.maximum.reduceat(hi, starts, axis=0)
        levels.append((b, lo, hi))
        b *= PYR_FACTOR
        step = PYR_FACTOR
    return levels


class RecordingStore(Series):
    """See the module doc. ``RecordingStore.open(index)`` (cached) or ``build``."""

    def __init__(self) -> None:
        self.wsk_t = np.empty(0)
        self.wsk = np.empty((0, 0), np.float32)
        self.imu_t = np.empty(0)
        self.imu = np.empty((0, len(IMU_CHANNELS)), np.float32)
        self.wsk_names: list[str] = []
        self.spk_row = np.empty(0, np.int64)          # despike: scan row, gauge, value (mV)
        self.spk_ch = np.empty(0, np.int64)
        self.spk_mv = np.empty(0, np.float32)
        self.from_cache = False
        self.build_s = 0.0
        self._desp: Optional[np.ndarray] = None
        self._pyr: dict = {}

    # ----- the read API (daq/series.py)

    def names(self, source: str) -> list[str]:
        return list(self.wsk_names) if source == WSK else list(IMU_CHANNELS)

    def _arrays(self, source: str, despike: bool) -> tuple[np.ndarray, np.ndarray]:
        if source == IMU:
            return self.imu_t, self.imu
        return self.wsk_t, (self.despiked() if despike and len(self.spk_row) else self.wsk)

    def despiked(self) -> np.ndarray:
        """The scans with the spikes replaced (made on first use)."""
        if self._desp is None:
            self._desp = self.wsk.copy()
            self._desp[self.spk_row, self.spk_ch] = self.spk_mv
        return self._desp

    def pyramid(self, source: str, despike: bool = False) -> list:
        key = (source, bool(despike and source == WSK and len(self.spk_row)))
        if key not in self._pyr:
            self._pyr[key] = _pyramid(self._arrays(source, key[1])[1])
        return self._pyr[key]

    def envelope(self, source: str, cols: Cols, t0: float, t1: float, n: int,
                 despike: bool = False) -> tuple[np.ndarray, np.ndarray]:
        t, y = self._arrays(source, despike)
        i0, i1 = span_index(t, t0, t1, margin=1)
        k = -(-(i1 - i0) // max(1, int(n)))           # samples per pixel column, rounded up
        if k <= 1:
            return t[i0:i1].copy(), np.array(y[i0:i1][:, cols])
        b, lo, hi = 1, y, y                           # the coarsest level with block <= k
        for level in self.pyramid(source, despike):
            if level[0] > k:
                break
            b, lo, hi = level
        g = -(-k // b)                                # level blocks per column
        a0 = (i0 // b) // g * g                       # whole columns, aligned to row 0
        blocks = -(-i1 // b)                          # level blocks up to i1, rounded up
        a1 = min(len(lo), -(-blocks // g) * g)
        starts = np.arange(0, a1 - a0, g)
        lo_c = np.minimum.reduceat(lo[a0:a1][:, cols], starts, axis=0)
        out = np.empty((2 * len(starts), lo_c.shape[1]), dtype=y.dtype)
        out[0::2] = lo_c
        out[1::2] = np.maximum.reduceat(hi[a0:a1][:, cols], starts, axis=0)
        first = (a0 + starts) * b
        last = np.minimum((a0 + starts + g) * b, len(t)) - 1
        td = np.empty(len(out))
        td[0::2] = t[first]
        td[1::2] = t[last]
        return td, out

    @property
    def duration(self) -> float:
        return float(self.wsk_t[-1]) if len(self.wsk_t) else 0.0

    @property
    def nbytes(self) -> int:
        n = self.wsk.nbytes + self.wsk_t.nbytes + self.imu.nbytes + self.imu_t.nbytes
        if self._desp is not None:
            n += self._desp.nbytes
        for levels in self._pyr.values():
            n += sum(lo.nbytes + hi.nbytes for _, lo, hi in levels)
        return n

    # ----- construction

    @classmethod
    def open(cls, index: RecordingIndex, progress: Optional[Callable[[float], None]] = None,
             use_cache: bool = True) -> "RecordingStore":
        import time
        t_start = time.perf_counter()
        path = Path(index.path)
        caches = [c.with_name(c.name.replace(".idx.npz", ".store.npz")) for c in cache_paths(path)]
        if use_cache:
            for c in caches:
                st = cls._load_cache(c, index)
                if st is not None:
                    st.build_s = time.perf_counter() - t_start
                    return st
        st = cls.build(index, progress)
        if use_cache:
            for c in caches:
                try:
                    c.parent.mkdir(parents=True, exist_ok=True)
                    st._save_cache(c, index)
                    break
                except OSError:
                    continue
        st.build_s = time.perf_counter() - t_start
        return st

    @classmethod
    def build(cls, index: RecordingIndex,
              progress: Optional[Callable[[float], None]] = None) -> "RecordingStore":
        self = cls()
        idx = index
        mv = float(idx.hello.get("mv_per_code", MV_PER_CODE))
        lines = np.arange(idx.hello_line + 1, idx.n_lines)
        kinds = idx.kind[lines]
        lines = lines[(kinds == K["wsk"]) | (kinds == K["mav"])]
        w_rx: list[float] = []
        w_tus: list[float] = []
        raw_parts: list[bytes] = []
        i_rx: list[float] = []
        i_us: list[float] = []
        i_rows: list[tuple] = []
        att = (0.0, 0.0, 0.0)
        nan = float("nan")
        k_wsk = K["wsk"]
        with open(path_of(idx), "rb") as f:
            for c0 in range(0, len(lines), _CHUNK_LINES):
                chunk = lines[c0:c0 + _CHUNK_LINES]
                base = int(idx.off[chunk[0]])
                f.seek(base)
                data = f.read(int(idx.off[chunk[-1] + 1]) - base)
                for ln in chunk.tolist():
                    a = int(idx.off[ln]) - base
                    e = data.find(b"\n", a)
                    line = data[a:e if e >= 0 else len(data)]
                    i = line.find(b'"t":')
                    j = line.find(b",", i)
                    t = float(line[i + 4:j])
                    if idx.kind[ln] == k_wsk:
                        seq = _int_after(line, b'"seq":', j)
                        tus = _int_after(line, b'"tus":', j)
                        w_rx.append(t)
                        w_tus.append(tus * 1e-6 if seq is not None and tus is not None else nan)
                        r0 = line.find(b'"raw":[', j)
                        raw_parts.append(line[r0 + 7:line.find(b"]", r0)] if r0 >= 0 else b"")
                        continue
                    ty = line.find(b'"type":"', j)
                    if ty < 0:
                        continue
                    mtype = line[ty + 8:line.find(b'"', ty + 8)]
                    if mtype == b"ATTITUDE":
                        d = line.find(b'"d":', ty)
                        att = tuple(float(_num_after(line, key, d) or 0) for key in _ATT_FIELDS)
                    elif mtype == b"RAW_IMU":
                        d = line.find(b'"d":', ty)
                        us = _num_after(line, b'"time_usec":', d)
                        i_rx.append(t)
                        i_us.append(int(us) * 1e-6 if isinstance(us, (int, float)) else nan)
                        i_rows.append(tuple(_num_after(line, key, d) or 0 for key in _IMU_FIELDS)
                                      + att)
                if progress is not None:
                    progress(min(1.0, (c0 + len(chunk)) / max(1, len(lines))))
        t0 = idx.t0
        self.wsk_names = [str(c) for c in idx.hello.get("channels", [])]
        rx = np.asarray(w_rx, dtype=np.float64)
        self.wsk_t = map_segments(idx.wsk_segs, rx, np.asarray(w_tus, np.float64)) - t0
        nch = raw_parts[0].count(b",") + 1 if raw_parts and raw_parts[0] else 0
        x = np.fromstring(b",".join(raw_parts), dtype=np.float64, sep=",") if nch else np.empty(0)
        if nch and len(x) == len(raw_parts) * nch:
            self.wsk = x.reshape(-1, nch).astype(np.float32) * mv
            is_int = b"." not in raw_parts[0]
            served = lines[idx.kind[lines] == k_wsk]       # the store's rows, by line
            keep = np.isin(idx.spk_line, served)
            vals = idx.spk_val[keep]
            vals = np.round(vals) if is_int else np.round(vals, 6)   # as read_lines writes them
            self.spk_row = np.searchsorted(served, idx.spk_line[keep]).astype(np.int64)
            self.spk_ch = idx.spk_ch[keep].astype(np.int64)
            # mV as ServerReader makes them: float32(raw) * mv, in float32
            self.spk_mv = vals.astype(np.float32) * mv
        else:                                  # mixed widths (no despike either): none
            self.wsk_t = np.empty(0)
            self.wsk = np.empty((0, len(self.wsk_names)), np.float32)
        if len(self.wsk_names) != self.wsk.shape[1]:
            self.wsk_names = [f"ch{i}" for i in range(self.wsk.shape[1])]
        self.imu_t = map_segments(idx.imu_segs, np.asarray(i_rx, np.float64),
                                  np.asarray(i_us, np.float64)) - t0
        self.imu = (np.asarray(i_rows, dtype=np.float32).reshape(-1, len(IMU_CHANNELS)))
        self._sort()
        return self

    def _sort(self) -> None:
        """Times are sorted after the fit (the device clocks are monotonic) except at
        a reboot, where a segment can start before the last one ended: a stable sort,
        so ``searchsorted`` holds. The spikes' rows follow."""
        for name in ("wsk", "imu"):
            t = getattr(self, f"{name}_t")
            if len(t) > 1 and (np.diff(t) < 0).any():
                order = np.argsort(t, kind="stable")
                setattr(self, f"{name}_t", t[order])
                setattr(self, name, getattr(self, name)[order])
                if name == "wsk" and len(self.spk_row):
                    self.spk_row = np.argsort(order)[self.spk_row]

    # ----- cache

    def _save_cache(self, path: Path, index: RecordingIndex) -> None:
        meta = {"version": STORE_VERSION, "size": index.size, "mtime_ns": index.mtime_ns,
                "names": self.wsk_names}
        tmp = path.with_name(path.name + ".tmp")
        with open(tmp, "wb") as f:
            np.savez(f, wsk_t=self.wsk_t, wsk=self.wsk, imu_t=self.imu_t, imu=self.imu,
                     spk_row=self.spk_row, spk_ch=self.spk_ch, spk_mv=self.spk_mv,
                     meta=np.asarray(json.dumps(meta)))
        os.replace(tmp, path)

    @classmethod
    def _load_cache(cls, cache: Path, index: RecordingIndex) -> Optional["RecordingStore"]:
        try:
            with np.load(cache, allow_pickle=False) as z:
                meta = json.loads(str(z["meta"]))
                if (meta.get("version") != STORE_VERSION or meta.get("size") != index.size
                        or meta.get("mtime_ns") != index.mtime_ns):
                    return None
                self = cls()
                for name in ("wsk_t", "wsk", "imu_t", "imu", "spk_row", "spk_ch", "spk_mv"):
                    setattr(self, name, z[name])
        except (OSError, ValueError, KeyError):
            return None
        self.wsk_names = list(meta["names"])
        self.from_cache = True
        return self

    def summary(self) -> str:
        return (f"store: {len(self.wsk_t)} scans x {self.wsk.shape[1]}, {len(self.imu_t)} IMU rows,"
                f" {self.nbytes / 1e6:.0f} MB, {self.build_s:.1f} s"
                + (" (cached)" if self.from_cache else ""))


def path_of(index: RecordingIndex) -> Path:
    return Path(index.path)
