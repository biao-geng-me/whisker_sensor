"""Export a time range of a recording: a ``.jsonl`` clip, or a ``.dat``.

``export_jsonl``: a byte copy of the lines in [t_start, t_end) (session
seconds, by paced time as replay serves them), after a copy of the hello and of
the newest info and status line before the range, so the clip opens, replays
and converts (``whisker_link.convert``) like any recording. The copied hello
and info / status lines are re-stamped to the range's first line (the hello
the way the GUI's tee re-stamps it at record start; ``mono`` / ``unix`` stay,
so wall clocks are kept). The hello also gets a ``clip`` record: the source
file, the range, and the source's V0 and surface pressure, which the index
takes over (``RecordingIndex.build``), so the clip shows the same zero and the
same depth as the full recording (its own first 0.25 s, or its own disarmed
start, need not be at rest or at the surface).

``export_dat``: the whisker scans in the range in the GUI recorder's ``.dat``
format (``io/dat_writer.py``): mV, outliers filled per block of a 20 Hz tick
as the control panel does, each row's wall clock from the fitted scan time
(more exact than the recorder's sample counting).
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import numpy as np

from wavi_py.daq.outliers import fill_outliers_linear
from wavi_py.io.dat_writer import DatWriter
from wavi_py.replay.index import K, RecordingIndex
from whisker_link.esp32_frame import MV_PER_CODE

TICK_HZ = 20.0                     # the control panel's tick: the outlier fill's block


def _range(index: RecordingIndex, t_start: float, t_end: float) -> tuple[int, int]:
    """Lines [i0, i1) whose paced time is in [t_start, t_end) (session seconds)."""
    if t_end <= t_start:
        raise ValueError(f"empty range {t_start:.3f} .. {t_end:.3f} s")
    i0, i1 = np.searchsorted(index.t_play, index.t0 + np.array([t_start, t_end]), side="left")
    return max(int(i0), index.hello_line + 1), int(i1)


def _restamp(line: bytes, t: float, **extra) -> bytes:
    ev = json.loads(line)
    ev["t"] = t
    ev.update(extra)
    return (json.dumps(ev, separators=(",", ":")) + "\n").encode()


def export_jsonl(index: RecordingIndex, t_start: float, t_end: float, out_path) -> int:
    """Write the clip; returns the number of lines copied from the range."""
    i0, i1 = _range(index, t_start, t_end)
    if i1 <= i0:
        raise ValueError("no lines in the range")
    t_first = float(index.t[i0])
    clip = {"src": index.path.name, "t_start": round(float(index.t_play[i0] - index.t0), 6),
            "t_end": round(float(index.t_play[i1 - 1] - index.t0), 6),
            "v0": [float(x) for x in index.v0],
            "surface_hpa": index.surface_hpa}
    with open(index.path, "rb") as f:
        hello = index.read_lines(f, index.hello_line, index.hello_line + 1)
        head = [_restamp(hello, t_first, clip=clip)]
        for lines in (index.info_lines, index.status_lines):
            j = index.last_before(lines, i0)
            if j is not None:
                head.append(_restamp(index.read_lines(f, j, j + 1), t_first))
        body = index.read_lines(f, i0, i1)
    with open(out_path, "wb") as out:
        out.writelines(head)
        out.write(body)
    return i1 - i0


def export_dat(index: RecordingIndex, t_start: float, t_end: float, out_path) -> int:
    """Write the range's scans as a ``.dat``; returns the rows written."""
    i0, i1 = _range(index, t_start, t_end)
    sel = np.nonzero(index.kind[i0:i1] == K["wsk"])[0] + i0
    if not len(sel):
        raise ValueError("no whisker scans in the range")
    mv = float(index.hello.get("mv_per_code", MV_PER_CODE))
    clock = index.clock()
    raw, t = [], []
    with open(index.path, "rb") as f:
        data = index.read_lines(f, i0, i1)
    base = int(index.off[i0])
    for i in sel:
        ev = json.loads(data[int(index.off[i]) - base:int(index.off[i + 1]) - base])
        raw.append(ev["raw"])
        if "tus" in ev:
            t.append(clock.wsk(float(ev["t"]), int(ev["tus"])))
        else:
            t.append(float(ev["t"]))
    x = np.asarray(raw, dtype=np.float32) * np.float32(mv)
    unix = index.unix0 + (np.asarray(t) - index.t0)
    blk = max(1, int(round((index.fs or 250.0) / TICK_HZ)))
    with DatWriter(out_path, x.shape[1]) as w:
        for k in range(0, len(x), blk):
            w.write_batch([datetime.fromtimestamp(u) for u in unix[k:k + blk]],
                          fill_outliers_linear(x[k:k + blk]))
        return w.n_written
