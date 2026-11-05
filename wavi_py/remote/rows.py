"""The recordings browser's rows, without Qt: local recordings and the cluster
catalogue as one list, paired, grouped, numbered and filtered.

- A ``Row`` is one recording: its local copy (any of the local folders: the
  output dir, the download cache) and / or its cluster copy, matched by stem;
  then a laptop row and a Pi row whose starts are within 3 s become one
  (``recname.same_recording``: the laptop's copy of a vehicle recording and
  the Pi's own file). The card is the local one, else the cluster's; the notes
  are the union by id of the local and the cluster lines (``notes.merge``).
- ``number``: within each date -> tag group, #1..n by start time, over all the
  rows (not a tab's or a filter's subset, so a recording keeps its number).
- ``grouped``: date -> tag -> rows, newest first at every level.
- ``matches``: the filter box (tag, stem, note text, the bad reason) and
  "hide bad".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

from wavi_py.remote.sync import find_recordings
from wavi_py.replay import card as cardmod
from wavi_py.replay import notes as rnotes
from wavi_py.replay import recname

RECORDING_SUFFIXES = (".jsonl", ".dat")


@dataclass
class Row:
    stem: str                                  # the primary stem (the one shown)
    start: Optional[datetime]
    tag: str
    card: Optional[dict] = None
    notes: list = field(default_factory=list)
    local: Optional[Path] = None               # the file to replay (.jsonl, else .dat)
    local_stem: str = ""
    cluster: Optional[dict] = None             # {"date", "stem", "files": {name: [size, mtime]}}
    queued: bool = False
    stems: list = field(default_factory=list)  # every stem in the row (a pair: two)
    n: int = 0                                 # #n in its date / tag group

    @property
    def date(self) -> str:
        return f"{self.start:%Y-%m-%d}" if self.start else "(undated)"

    @property
    def state(self) -> rnotes.NotesState:
        return rnotes.state(self.notes)

    @property
    def bad(self) -> bool:
        return self.state.bad

    @property
    def where(self) -> str:
        w = ("both" if self.local and self.cluster else "local" if self.local else "cluster")
        return w + (", queued" if self.queued else "")

    @property
    def duration(self) -> Optional[float]:
        return float(self.card["duration_s"]) if self.card and "duration_s" in self.card else None

    @property
    def size(self) -> int:
        return sum(int(v[0]) for v in (self.cluster or {}).get("files", {}).values())


def _start(stem: str, fallback: Optional[float] = None) -> tuple[Optional[datetime], str]:
    rn = recname.parse(stem)
    if rn is not None:
        return rn.start, rn.tag
    return (datetime.fromtimestamp(fallback) if fallback else None), ""


def local_rows(folders: Iterable[Path], queued: Iterable[str] = ()) -> dict[str, Row]:
    """The recordings in the local folders (the first folder wins a stem)."""
    queued = set(queued)
    out: dict[str, Row] = {}
    for folder in folders:
        for stem, rec in find_recordings(Path(folder)).items():
            if stem in out:
                continue
            main = next((rec.dir / (stem + s) for s in RECORDING_SUFFIXES
                         if (rec.dir / (stem + s)).is_file()), None)
            start, tag = _start(stem, rec.mtime)
            out[stem] = Row(stem, start, tag, card=cardmod.read_card(cardmod.card_path(main)),
                            notes=rnotes.read(rnotes.notes_path(main)), local=main,
                            local_stem=stem, queued=stem in queued, stems=[stem])
    return out


def cluster_rows(folders: dict) -> dict[str, Row]:
    out: dict[str, Row] = {}
    for date, f in folders.items():
        stems: dict[str, dict] = {}
        for name, v in f.get("files", {}).items():
            stems.setdefault(recname.stem_of(name), {})[name] = v
        for stem, files in stems.items():
            if not any(stem + s in files for s in RECORDING_SUFFIXES):
                continue
            mt = max(v[1] for v in files.values())
            start, tag = _start(stem, mt)
            card = f.get("cards", {}).get(stem)
            if card is not None and card.get("v") != cardmod.VERSION:
                card = None
            out[stem] = Row(stem, start, tag, card=card, notes=list(f.get("notes", {}).get(stem, [])),
                            cluster={"date": date, "stem": stem, "files": files}, stems=[stem])
    return out


def _absorb(a: Row, b: Row) -> Row:
    """One row from two copies of a recording (``a`` stays primary)."""
    a.local = a.local or b.local
    a.local_stem = a.local_stem or b.local_stem
    a.cluster = a.cluster or b.cluster
    a.queued = a.queued or b.queued
    a.card = a.card or b.card
    a.notes = rnotes.merge(a.notes, b.notes)
    a.stems = a.stems + [s for s in b.stems if s not in a.stems]
    return a


def build_rows(local: dict[str, Row], cluster: dict[str, Row]) -> list[Row]:
    rows = dict(local)
    for stem, r in cluster.items():
        rows[stem] = _absorb(rows[stem], r) if stem in rows else r
    # pairs: a laptop row and a Pi row within SAME_S (one each way at most)
    laptop = [r for r in rows.values() if r.stem.startswith("st_")]
    pi = [r for r in rows.values() if r.stem.startswith("whisker_")]
    used = set()
    for a in laptop:
        for b in pi:
            if b.stem in used or not recname.same_recording(a.stem, b.stem):
                continue
            used.add(b.stem)
            # the laptop's name is the row's, unless only the Pi's has a card
            first, second = (b, a) if (b.card and not a.card) else (a, b)
            merged = _absorb(first, second)
            rows.pop(a.stem, None)
            rows.pop(b.stem, None)
            rows[merged.stem] = merged
            break
    out = list(rows.values())
    number(out)
    return out


def number(rows: list[Row]) -> None:
    groups: dict = {}
    for r in rows:
        groups.setdefault((r.date, r.tag), []).append(r)
    for g in groups.values():
        g.sort(key=lambda r: (r.start or datetime.min, r.stem))
        for i, r in enumerate(g, 1):
            r.n = i


def grouped(rows: Iterable[Row]) -> list[tuple[str, list[tuple[str, list[Row]]]]]:
    """[(date, [(tag, rows)])], newest first everywhere."""
    by: dict = {}
    for r in rows:
        by.setdefault(r.date, {}).setdefault(r.tag, []).append(r)
    newest = lambda rs: max((r.start or datetime.min) for r in rs)  # noqa: E731
    out = []
    for date in sorted(by, reverse=True):
        tags = sorted(by[date].items(), key=lambda kv: newest(kv[1]), reverse=True)
        out.append((date, [(t, sorted(rs, key=lambda r: (r.start or datetime.min, r.stem),
                                       reverse=True)) for t, rs in tags]))
    return out


def matches(r: Row, text: str = "", hide_bad: bool = False) -> bool:
    st = r.state
    if hide_bad and st.bad:
        return False
    text = text.strip().lower()
    if not text:
        return True
    hay = [r.tag, *r.stems, st.reason] + [str(n.get("text", "")) for n in st.notes]
    return any(text in h.lower() for h in hay if h)
