"""The dome map of a recording: which sensor and whisker sat where when it was made.

The dome changes (whiskers swapped, cut, lost; a sensor replaced), so events of
different days compare only through the map each recording was made with:

- embedded: wavi_py writes the active map into the recordings it makes, as a
  line right after the hello (``dome_map_line``)::

      {"k":"dome_map","id":"2026-11-05","map":{...the map's JSON...}}

  It has no ``"t"``: the index skips it (session zero stays the first event
  after the hello), and the readers that go by kind ignore it;
- history: ``wavi_py/assets/dome/maps/*.json``, one map per configuration with
  the local time it holds ``from`` (``whisker_link/dome_map.py``); a recording
  without an embedded map (the Pi's, older ones) gets the newest entry whose
  ``from`` is not after its start (the local time in its name).

``dome_map_for(recording)`` is the one call. ``wavi_py/assets/dome/dome_map.json``
stays the live map (the GUI's default): the newest history entry.
"""

from __future__ import annotations

import json
from datetime import datetime
from functools import lru_cache
from pathlib import Path
from typing import Optional, Union

from wavi_py.replay import recname
from whisker_link.dome_map import DEFAULT_MAP, DomeMap, load_calibration

MAPS_DIR = DEFAULT_MAP.parent / "maps"
KIND = "dome_map"
_HEAD_BYTES = 1 << 18           # the embedded map is within this much of the file's start
_HEAD_LINES = 8


@lru_cache(maxsize=1)
def _history_cached(maps_dir: str) -> tuple:
    out = []
    for p in sorted(Path(maps_dir).glob("*.json")):
        m = DomeMap.load(p)
        out.append((datetime.fromisoformat(m.valid_from), m))
    out.sort(key=lambda fm: fm[0])
    return tuple(out)


def history(maps_dir: Union[str, Path, None] = None) -> list[DomeMap]:
    """The history's maps, oldest first."""
    return [m for _, m in _history_cached(str(maps_dir or MAPS_DIR))]


def map_at(when: datetime, maps_dir: Union[str, Path, None] = None) -> DomeMap:
    """The history entry in force at local time ``when`` (the oldest one before
    the history starts)."""
    hist = _history_cached(str(maps_dir or MAPS_DIR))
    if not hist:
        raise FileNotFoundError(f"no dome maps in {maps_dir or MAPS_DIR}")
    pick = hist[0][1]
    for frm, m in hist:
        if frm <= when:
            pick = m
    return pick


def dome_map_line(m: DomeMap) -> bytes:
    """The line a recording carries its map in (after the hello), from the JSON
    the map was loaded from."""
    doc = m.doc if m.doc is not None else {}
    return (json.dumps({"k": KIND, "id": m.id or (m.path.stem if m.path else ""), "map": doc},
                       separators=(",", ":")) + "\n").encode()


def from_line(ev: dict) -> Optional[DomeMap]:
    """The map of a parsed ``dome_map`` line (calibration from the live map's
    folder, as its ``calibration_file`` names it); None if it does not load."""
    doc = ev.get("map")
    if not isinstance(doc, dict):
        return None
    sensors = {}
    cal = doc.get("calibration_file")
    if isinstance(cal, str):
        try:
            sensors = load_calibration(DEFAULT_MAP.parent / Path(cal).name)
        except ValueError:
            sensors = {}
    try:
        m = DomeMap.from_dict(doc, sensors, where=f"{KIND} line")
    except ValueError:
        return None
    if not m.id:
        m.id = str(ev.get("id", ""))
    return m


def embedded(recording: Union[str, Path]) -> Optional[DomeMap]:
    """The map written into the recording (the first ``dome_map`` line within its
    first lines), or None."""
    try:
        with open(recording, "rb") as f:
            head = f.read(_HEAD_BYTES)
    except OSError:
        return None
    for line in head.split(b"\n")[:_HEAD_LINES]:
        if b'"k":"dome_map"' in line[:40] or b'"k": "dome_map"' in line[:40]:
            try:
                return from_line(json.loads(line))
            except ValueError:
                return None
    return None


def start_of(recording: Union[str, Path]) -> Optional[datetime]:
    """The recording's start, local time, from its name."""
    rn = recname.parse(Path(recording).name)
    return rn.start if rn is not None else None


def dome_map_for(recording: Union[str, Path], maps_dir: Union[str, Path, None] = None) -> DomeMap:
    """The recording's map: the embedded one, else the history entry at its start
    (the live map when the name has no start)."""
    m = embedded(recording)
    if m is not None:
        return m
    when = start_of(recording)
    if when is None:
        return DomeMap.load(DEFAULT_MAP)
    return map_at(when, maps_dir)
