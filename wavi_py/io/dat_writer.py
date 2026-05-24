"""Write the ``.dat`` recording format byte-for-byte the same as MATLAB ``wavi.m``.

Per-line format ([wavi.m:1140-1158](wavi/wavi.m#L1140-L1158)):

    YYYY-MM-DD HH:MM:SS.SSS  v1  v2  ...  vC\\n

where each ``vi`` is rendered with ``"{:12.6f}"`` (right-padded, 6 decimals,
leading space). Filename ([wavi.m:1129-1138](wavi/wavi.m#L1129-L1138)):

    st_YYYY-MM-DD_HHMM_SS.SS_<tag>.dat

Existing tools under ``data_processing/`` parse this exact format, so format
parity here means the analysis pipeline keeps working unchanged.
"""

from __future__ import annotations

import os
import re
from datetime import datetime, timedelta
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


_TAG_SAFE = re.compile(r"[^A-Za-z0-9._-]")
_TAG_MAX_LEN = 128


def sanitize_tag(tag: str) -> str:
    """Replace unsafe filename characters with ``_``; collapse repeats; trim."""
    if not tag:
        return ""
    s = _TAG_SAFE.sub("_", str(tag))
    s = re.sub(r"_+", "_", s).strip("_")
    return s[:_TAG_MAX_LEN]


def build_datalog_filename(start_time: datetime, tag: str = "") -> str:
    """Mirror ``wavi.m:build_datalog_filepath`` filename convention."""
    safe_tag = sanitize_tag(tag)
    sec_with_frac = start_time.second + start_time.microsecond / 1_000_000.0
    return (
        f"st_{start_time.year:04d}-{start_time.month:02d}-{start_time.day:02d}_"
        f"{start_time.hour:02d}{start_time.minute:02d}_"
        f"{sec_with_frac:05.2f}_{safe_tag}.dat"
    )


def _format_dt(t: datetime) -> str:
    sec = t.second + t.microsecond / 1_000_000.0
    return (
        f"{t.year:04d}-{t.month:02d}-{t.day:02d} "
        f"{t.hour:02d}:{t.minute:02d}:{sec:06.3f}"
    )


class DatWriter:
    """Append-mode writer for the legacy ``.dat`` format.

    Open on record-start, ``write_batch`` per new-sample block, ``close`` on
    record-stop. Thread-safe is not assumed — only one subscriber should hold
    a given writer.
    """

    def __init__(self, full_path: Path | str, nch: int):
        self._path = Path(full_path)
        self._nch = int(nch)
        self._fp = open(self._path, "w", buffering=1, newline="")
        # Format per row: timestamp + nch fields of " %12.6f" + newline.
        self._row_tail = " {:12.6f}" * self._nch + "\n"
        self._n_written = 0

    @property
    def path(self) -> Path:
        return self._path

    @property
    def n_written(self) -> int:
        return self._n_written

    def write_batch(self, timestamps: Iterable[datetime], samples: np.ndarray) -> int:
        """Write a block of samples. ``samples`` is (N, nch). Returns rows written."""
        if samples.ndim != 2 or samples.shape[1] != self._nch:
            raise ValueError(
                f"samples shape must be (N, {self._nch}), got {samples.shape}"
            )
        ts_list = list(timestamps)
        if len(ts_list) != samples.shape[0]:
            raise ValueError(
                f"timestamps length {len(ts_list)} != samples N {samples.shape[0]}"
            )

        fp = self._fp
        if fp is None or fp.closed:
            raise RuntimeError("DatWriter is closed")

        row_tail = self._row_tail
        for ts, row in zip(ts_list, samples):
            fp.write(_format_dt(ts))
            fp.write(row_tail.format(*row.tolist()))

        n = samples.shape[0]
        self._n_written = self._n_written + n
        return n

    def close(self) -> None:
        if self._fp is not None and not self._fp.closed:
            try:
                self._fp.flush()
                os.fsync(self._fp.fileno())
            except OSError:
                pass
            self._fp.close()

    def __enter__(self) -> "DatWriter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def make_timestamp_block(t_start: datetime, n: int, fs: float) -> list[datetime]:
    """Helper: build N evenly-spaced datetimes at ``1/fs`` cadence starting at t_start."""
    dt = 1.0 / float(fs)
    return [t_start + timedelta(seconds=i * dt) for i in range(n)]
