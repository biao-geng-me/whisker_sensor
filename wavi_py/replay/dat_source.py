"""Whisker-only replay of a ``.dat`` (a recording without its ``.jsonl``).

The ``.dat`` (``io/dat_writer.py``: a wall clock and the mV of every gauge per
row, written after the outlier fill) holds no IMU, MAVLink, status, events or
device clock. To replay one through the same index / transport / reader path,
it is converted once into a daq_server-shaped ``.jsonl`` in the user cache
(``%LOCALAPPDATA%/wavi_py/dat`` on Windows, ``~/.cache/wavi_py/dat``
elsewhere; rebuilt when the ``.dat`` changes):

- a hello with ``server`` ``DAT_SERVER`` (a local source: no vehicle, so no IMU
  window data, no live timeline), ``mv_per_code`` 1 (the values are mV
  already), the ``.dat``'s first wall clock as ``unix`` and the source's name,
  size and mtime;
- one ``wsk`` line per row: ``t`` and ``tus`` both the row's time from the
  first (the recorder counted samples, so these are evenly spaced at its rate
  and the clock fit is the identity), ``seq`` the row number, ``raw`` the mV.

Only the 16-whisker array's 32-gauge ``.dat`` is taken (``NotARecording``
otherwise): the old 9-sensor bridge files have no array to show.
"""

from __future__ import annotations

import hashlib
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import numpy as np

DAT_SERVER = "wavi_py-dat"
N_GAUGES = 32
CONVERT_VERSION = 2                # 2: rows with "60.000" s no longer end the file


def converted_path(dat: Path) -> Path:
    from wavi_py.replay.index import _cache_dir

    dat = Path(dat).resolve()
    tag = hashlib.sha1(str(dat).encode("utf-8", "surrogatepass")).hexdigest()[:12]
    return _cache_dir() / "dat" / tag / f"{dat.name}.jsonl"


def _up_to_date(out: Path, st) -> bool:
    try:
        with open(out, "rb") as f:
            hello = json.loads(f.readline())
    except (OSError, ValueError):
        return False
    src = hello.get("dat") or {}
    return (src.get("size") == st.st_size and src.get("mtime_ns") == st.st_mtime_ns
            and src.get("version") == CONVERT_VERSION)


def ensure_jsonl(dat, progress: Optional[Callable[[float], None]] = None) -> Path:
    """The converted ``.jsonl`` of ``dat`` (converting it if missing or stale)."""
    dat = Path(dat)
    out = converted_path(dat)
    st = dat.stat()
    if not _up_to_date(out, st):
        out.parent.mkdir(parents=True, exist_ok=True)
        dat_to_jsonl(dat, out, progress)
    return out


def _stamp(day: bytes, clock: bytes) -> float:
    """Unix seconds of a .dat row's "YYYY-MM-DD HH:MM:SS.fff". Older writers
    rounded the seconds, so "60.000" occurs: taken as the next minute."""
    hms = clock.decode()
    h, m, sec = hms.split(":")
    base = datetime.fromisoformat(f"{day.decode()} {h}:{m}:00")
    return base.timestamp() + float(sec)


def read_dat(dat, progress: Optional[Callable[[float], None]] = None
             ) -> tuple[np.ndarray, np.ndarray]:
    """(unix seconds (N,), mV (N, C)) of a ``.dat``; a torn last row is dropped."""
    from wavi_py.replay.index import NotARecording

    stamps, rows = [], []
    size = max(1, Path(dat).stat().st_size)
    pos, next_report = 0, 0
    with open(dat, "rb") as f:
        for line in f:
            pos += len(line)
            if progress is not None and pos >= next_report:
                progress(pos / size)
                next_report = pos + (1 << 20)
            if not line.endswith(b"\n"):
                break
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                stamps.append(_stamp(parts[0], parts[1]))
                rows.append(parts[2:])
            except ValueError:
                if not stamps:
                    raise NotARecording(f"{dat}: not a wavi .dat (row 1: {line[:60]!r})")
                continue                          # one unreadable row: skipped, not the end
    if not rows:
        raise NotARecording(f"{dat}: no rows")
    nch = len(rows[0])
    if nch != N_GAUGES:
        raise NotARecording(f"{dat}: {nch} columns; only the whisker array's {N_GAUGES}-gauge"
                            " .dat can be replayed")
    keep, vals = [], []
    for i, r in enumerate(rows):                  # a short or garbled row: skipped
        if len(r) != nch:
            continue
        try:
            vals.append([float(v) for v in r])
        except ValueError:
            continue
        keep.append(i)
    return np.asarray(stamps)[keep], np.array(vals, dtype=np.float64)


def dat_to_jsonl(dat, out, progress: Optional[Callable[[float], None]] = None) -> int:
    """Write the converted stream of ``dat`` to ``out``; returns the scans."""
    dat = Path(dat)
    st = dat.stat()
    unix, x = read_dat(dat, None if progress is None else (lambda p: progress(0.6 * p)))
    t = unix - unix[0]
    t = np.maximum.accumulate(t)                      # a clock step back: hold
    hello = {"k": "hello", "t": 0.0, "mono": 0.0, "unix": float(unix[0]), "wire": 1,
             "server": DAT_SERVER, "mv_per_code": 1.0,
             "dat": {"name": dat.name, "size": st.st_size, "mtime_ns": st.st_mtime_ns,
                     "version": CONVERT_VERSION}}
    tmp = Path(str(out) + ".tmp")
    n = len(x)
    step = max(1, n // 40)
    with open(tmp, "w", newline="\n") as f:
        f.write(json.dumps(hello) + "\n")
        for i in range(n):
            if progress is not None and i % step == 0:
                progress(0.6 + 0.4 * i / n)
            f.write('{"k":"wsk","t":%.6f,"seq":%d,"tus":%d,"raw":[%s]}\n'
                    % (t[i], i, int(round(t[i] * 1e6)), ",".join("%.6f" % v for v in x[i])))
    os.replace(tmp, out)
    if progress is not None:
        progress(1.0)
    return n
