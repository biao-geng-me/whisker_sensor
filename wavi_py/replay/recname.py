"""Recording names: which recording a file belongs to, and when it started.

A recording is identified by its *stem*, the file name without its
extension(s). Two formats exist:

- laptop: ``st_YYYY-MM-DD_HHMM_SS.ss_<tag>`` (``io/dat_writer.py:
  build_datalog_filename``), with ``.dat``, ``.jsonl``, ``.faults.txt``;
- Pi: ``whisker_YYYYMMDD_HHMMSS[_<tag>]`` (``rpi/daq_server.py``), ``.jsonl``.

Tags have no naming rule beyond the sanitizer (``[A-Za-z0-9._-]``), so a tag
may hold ``_``, ``-`` and even ``.``; an empty laptop tag leaves the trailing
``_`` (``st_..._23.86_.dat``). That is why the stem is found by stripping the
known suffixes (``SUFFIXES``), never with ``Path.stem``: the laptop seconds
(``23.86``) already have a dot.

Sidecars sit next to the recording and share its stem: ``.notes.jsonl``
(``replay/notes.py``), ``.card.json`` (``replay/card.py``) and the caches
(``.jsonl.idx.npz``, ``.jsonl.store.npz``) that are rebuilt, never uploaded.

A laptop stem and a Pi stem whose starts are within ``SAME_S`` are the same
recording (the laptop's local copy of a vehicle recording starts once the
server's ack is in, a second or so after the Pi's): ``same_recording``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Union

SAME_S = 3.0
# longest first: ".jsonl.idx.npz" before ".jsonl"; ".notes.jsonl" before ".jsonl"
SUFFIXES = (".jsonl.store.npz", ".jsonl.idx.npz", ".dat.store.npz", ".dat.idx.npz",
            ".notes.jsonl", ".card.json", ".faults.txt", ".jsonl", ".dat")
RECORDING_SUFFIXES = (".jsonl", ".dat")     # the data; the rest are sidecars
CACHE_SUFFIXES = (".jsonl.store.npz", ".jsonl.idx.npz", ".dat.store.npz", ".dat.idx.npz")

_LAPTOP = re.compile(r"^st_(\d{4})-(\d\d)-(\d\d)_(\d\d)(\d\d)_(\d\d)\.(\d\d)_(.*)$")
_PI = re.compile(r"^whisker_(\d{4})(\d\d)(\d\d)_(\d\d)(\d\d)(\d\d)(?:_(.*))?$")


@dataclass(frozen=True)
class RecName:
    stem: str
    start: datetime          # local time, as the name has it (naive)
    tag: str
    source: str              # "laptop" | "pi"

    @property
    def date(self) -> str:
        """``YYYY-MM-DD`` of the start: the cluster folder (spec)."""
        return f"{self.start:%Y-%m-%d}"


def stem_of(name: Union[str, Path]) -> str:
    """The stem of a recording file or sidecar name (a path's folder is dropped):
    ``st_..._tag.jsonl.idx.npz`` -> ``st_..._tag``. An unknown extension stays."""
    s = Path(name).name
    if s.lower().endswith(".part"):      # an upload in flight: <file>.part
        s = s[:-5]
    low = s.lower()
    for suf in SUFFIXES:
        if low.endswith(suf):
            return s[:-len(suf)]
    return s


def sidecar(path: Union[str, Path], suffix: str) -> Path:
    """The ``suffix`` sidecar (e.g. ``".notes.jsonl"``) of the recording ``path``,
    in the same folder."""
    p = Path(path)
    return p.with_name(stem_of(p.name) + suffix)


def parse(name: Union[str, Path]) -> Optional[RecName]:
    """The recording a file name (or stem) belongs to; None if it is not a
    recording name in either format (or its date is impossible)."""
    stem = stem_of(name)
    m = _LAPTOP.match(stem)
    try:
        if m:
            y, mo, d, h, mi, s, cs, tag = m.groups()
            start = datetime(int(y), int(mo), int(d), int(h), int(mi), int(s), int(cs) * 10_000)
            return RecName(stem, start, tag, "laptop")
        m = _PI.match(stem)
        if m:
            y, mo, d, h, mi, s, tag = m.groups()
            start = datetime(int(y), int(mo), int(d), int(h), int(mi), int(s))
            return RecName(stem, start, tag or "", "pi")
    except ValueError:                  # month 13, second 61, ...
        return None
    return None


def same_recording(a: Union[str, Path, RecName, None], b: Union[str, Path, RecName, None],
                   tol_s: float = SAME_S) -> bool:
    """Whether two names are one recording: the same stem, or a laptop and a Pi
    stem whose starts are within ``tol_s`` (the laptop's copy and the Pi's file of
    one vehicle recording). Tags are not compared: the Pi sanitizes its own way.
    The two clocks are the laptop's and the Pi's: within ``tol_s`` only while the
    Pi keeps time (NTP or the laptop's)."""
    ra = a if isinstance(a, RecName) else (parse(a) if a else None)
    rb = b if isinstance(b, RecName) else (parse(b) if b else None)
    if ra is None or rb is None:
        return False
    if ra.stem == rb.stem:
        return True
    if ra.source == rb.source:          # two of one machine's files: names differ, so not
        return False
    return abs((ra.start - rb.start).total_seconds()) <= tol_s
