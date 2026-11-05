"""Stand-ins for ``replay/recname.py`` and ``replay/notes.py`` (task A, another
branch): only what sync needs, behind three names, so swapping in task A's
modules is a change of this file alone.

- ``stem_of(name)``: the recording's stem (the file name without its known
  extension; ``Path.stem`` is wrong here, ``x.faults.txt`` and the ``23.86``
  in a laptop name both have dots);
- ``start_date(stem)``: ``YYYY-MM-DD`` of the recording's start, from either
  name format, or None (laptop ``st_YYYY-MM-DD_HHMM_SS.ss_<tag>``,
  ``io/dat_writer.py``; Pi ``whisker_YYYYMMDD_HHMMSS[_<tag>]``,
  ``rpi/daq_server.py``);
- ``merge_notes(a, b)``: two copies of ``<stem>.notes.jsonl`` as one, the
  spec's rule: the union of the lines by ``id``, sorted by ``unix``. Lines are
  kept as written (bytes), so a merge that adds nothing returns the same text
  and sync can tell "no change" by comparing bytes. A torn line (a crash mid-
  write) is skipped, as ``whisker_link/rec_index.read_lines`` does.
"""

from __future__ import annotations

import json
import re
from typing import Optional

# longest first: ".notes.jsonl" before ".jsonl"
KNOWN_SUFFIXES = (".notes.jsonl", ".faults.txt", ".card.json", ".jsonl", ".dat")
# rebuildable caches next to a recording (replay/index.py, replay/store.py):
# never uploaded; pruned with the recording
CACHE_SUFFIXES = (".idx.npz", ".store.npz")

_LAPTOP = re.compile(r"^st_(\d{4})-(\d{2})-(\d{2})_\d{4}_")
_PI = re.compile(r"^whisker_(\d{4})(\d{2})(\d{2})_\d{6}")


def stem_of(name: str) -> str:
    """The stem of a recording's file name (any of its sidecars or caches)."""
    for c in CACHE_SUFFIXES:
        if name.endswith(c):
            name = name[: -len(c)]
            break
    for s in KNOWN_SUFFIXES:
        if name.endswith(s):
            return name[: -len(s)]
    return name.rsplit(".", 1)[0] if "." in name else name


def start_date(stem: str) -> Optional[str]:
    m = _LAPTOP.match(stem) or _PI.match(stem)
    return f"{m.group(1)}-{m.group(2)}-{m.group(3)}" if m else None


def _parse(data: bytes) -> list[tuple[str, float, bytes]]:
    """(key, unix, raw line) per readable line; key = the id, or the raw line."""
    out = []
    for raw in data.splitlines():
        raw = raw.strip()
        if not raw:
            continue
        try:
            ev = json.loads(raw)
        except ValueError:
            continue                              # a torn line
        if not isinstance(ev, dict):
            continue
        key = str(ev.get("id") or raw.decode("utf-8", "replace"))
        try:
            unix = float(ev.get("unix") or 0.0)
        except (TypeError, ValueError):
            unix = 0.0
        out.append((key, unix, raw))
    return out


def merge_notes(a: bytes, b: bytes) -> bytes:
    """The union of two notes files by id, sorted by unix (first copy wins a tie
    of ids with different text, which the append-only format never makes)."""
    seen: dict[str, tuple[float, bytes]] = {}
    for key, unix, raw in _parse(a) + _parse(b):
        seen.setdefault(key, (unix, raw))
    lines = sorted(seen.values(), key=lambda u: u[0])
    return b"".join(raw + b"\n" for _, raw in lines)
