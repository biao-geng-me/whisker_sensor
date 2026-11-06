"""Event sets: manoeuvre events of many recordings, filtered, grouped, curated.

The data side of the Analysis window's event mode (no Qt). The events are the
sidecars' (``replay/manoeuvres.py``, ``<stem>.manoeuvres.json``); a query reads
only those, so it is quick over many recordings, and the traces are fetched
later, per event, from the recordings' stores:

- ``EventQuery``: the filter (a JSON-serialisable dataclass): which recordings,
  which events, how to group them, and the curation (excludes and pins by
  ``(stem, event id)``);
- ``EventSet.from_query(query)``: the matching ``EventRef``\\ s (excluded ones kept
  apart), the groups with their counts, ``rows()`` for a table;
- saved comparisons: named queries as JSON in wavi_py's user config dir
  (``list_comparisons`` / ``save_comparison`` / ``load_comparison`` /
  ``delete_comparison``);
- ``StoreCache``: the recordings' ``RecordingStore``\\ s, opened when an event's
  traces are first needed, least recently used dropped over a byte cap;
- ``event_selections(eventset, cache)``: ``daq/analysis.Selection``\\ s, one per
  event, each with its own recording's source, the event's window, ``align``
  (onset - t0: the time zero at the stick), its group, baseline window, pin and
  dome map, for a ``Dataset`` (``baseline=True`` for the event overlay).

Filters (a None / empty field does not filter):

- ``recordings``: recording paths or folders (a folder: its ``*.jsonl``);
  ``sidecar_dir``: where the sidecars are (default: next to each recording, or
  the user cache's ``manoeuvres/`` for a folder we cannot write:
  ``manoeuvres.sidecar_dir_for``);
  ``build_missing``: make a missing or outdated sidecar (``write_manoeuvres_for``;
  with ``sidecar_dir`` the recordings' folder gets no caches);
- ``kinds`` ("hold", "transition"), ``types`` ("hold", "onset", "release",
  "step", "reversal"), ``classes`` (``single:surge+``, ``combo``, ``rest``; for a
  transition the class it moves into, for a release the class it leaves);
- ``level_min`` / ``level_max`` on ``|level|``: the event's axis level (the hold's,
  the level after an onset / step / reversal, before a release; a combo: its
  largest axis);
- ``modes`` (``ALT_HOLD``, ``MANUAL``, ...), ``hold_min_s``: the hold's length
  (a transition: the time to the next stick change, ``next_s``);
- ``baseline_clean``, ``steady``: required when True;
- ``minor``: True / None any, False none, a number: the largest minor input at
  most this;
- ``depth_min_cm`` / ``depth_max_cm``: a depth filter drops runs whose depth is
  not trusted (``recording.depth_ok`` false) and events without a depth;
- ``alive``: "all" (no position lost, ``alive`` false), or a list of positions;
  undetermined (``alive`` null: no thrust, a rest) passes; ``alive_known``: only
  events where every one of those positions is known alive;
- ``whisker_sets``, ``map_ids``: the recording's dome map (``recording.whisker_set``
  / ``dome_map``; a v1 sidecar without them: ``replay/dome_maps.dome_map_for``);
- ``group_by``: "run", "day", "level" (bins of ``|level|`` by ``level_edges``),
  "mode", "whisker_set" or "none".
"""

from __future__ import annotations

import json
import os
import re
import sys
import time
from collections import OrderedDict
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Optional, Union

from wavi_py.replay import manoeuvres as mn
from wavi_py.replay import recname

GROUP_BY = ("run", "day", "level", "mode", "whisker_set", "none")
LEVEL_EDGES = (0.0, 0.25, 0.5, 0.75, 1.01)
STORE_CAP_BYTES = 1 << 30
COMPARISON_SUFFIX = ".comparison.json"


# ---------------------------------------------------------------------------
# the query
# ---------------------------------------------------------------------------
@dataclass
class EventQuery:
    """Which events (see the module doc). ``to_json`` / ``from_json`` round-trip."""
    recordings: list = field(default_factory=list)
    sidecar_dir: Optional[str] = None
    build_missing: bool = True
    kinds: list = field(default_factory=list)
    types: list = field(default_factory=list)
    classes: list = field(default_factory=list)
    level_min: Optional[float] = None
    level_max: Optional[float] = None
    modes: list = field(default_factory=list)
    hold_min_s: Optional[float] = None
    baseline_clean: bool = False
    steady: bool = False
    minor: Union[bool, float, None] = None
    depth_min_cm: Optional[float] = None
    depth_max_cm: Optional[float] = None
    alive: Union[str, list, None] = None
    alive_known: bool = False
    whisker_sets: list = field(default_factory=list)
    map_ids: list = field(default_factory=list)
    group_by: str = "none"
    level_edges: list = field(default_factory=lambda: list(LEVEL_EDGES))
    excludes: list = field(default_factory=list)    # [stem, id]
    pins: list = field(default_factory=list)        # [stem, id]

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=1)

    @classmethod
    def from_dict(cls, d: dict) -> "EventQuery":
        known = {f.name for f in fields(cls)}
        q = cls(**{k: v for k, v in d.items() if k in known})
        if q.group_by not in GROUP_BY:
            raise ValueError(f"group_by {q.group_by!r}: one of {', '.join(GROUP_BY)}")
        return q

    @classmethod
    def from_json(cls, text: str) -> "EventQuery":
        return cls.from_dict(json.loads(text))

    # curation
    def _key_list(self, which: str) -> list:
        return getattr(self, which)

    def _toggle(self, which: str, stem: str, eid: int, on: bool) -> None:
        lst = self._key_list(which)
        k = [stem, int(eid)]
        while k in lst:
            lst.remove(k)
        if on:
            lst.append(k)

    def exclude(self, stem: str, eid: int, on: bool = True) -> None:
        self._toggle("excludes", stem, eid, on)

    def pin(self, stem: str, eid: int, on: bool = True) -> None:
        self._toggle("pins", stem, eid, on)


@dataclass
class EventRef:
    """One event of a sidecar: where it is, what it is, its features."""
    recording: str                  # the recording's path
    stem: str
    id: int
    t: float                        # the onset (alignment t = 0), session s
    t_start: float                  # the window
    t_end: float
    kind: str
    type: str
    cls: str
    axis: Optional[str]
    sign: int
    level: float                    # signed, the event's axis (module doc)
    hold_s: Optional[float]
    mode: Optional[str]
    depth_cm: Optional[float]
    depth_ok: bool
    baseline_t: Optional[tuple]
    baseline_clean: bool
    steady: bool
    minor: dict                     # axis -> the largest |u| of that minor input
    alive: dict                     # position -> True / False / None
    no_reference: bool              # its map has no reference: no drift correction
    flags: dict                     # quality: adc_flagged_scans, railed, lost
    map_id: Optional[str]
    whisker_set: Optional[str]
    day: str
    positions: dict                 # the sidecar's per-position features
    motion: dict
    group: str = ""
    pinned: bool = False
    excluded: bool = False

    @property
    def key(self) -> tuple:
        return (self.stem, self.id)

    @property
    def window(self) -> tuple:
        return (self.t_start, self.t_end)

    @property
    def label(self) -> str:
        rn = recname.parse(self.stem)
        when = f"{rn.start:%m-%d %H:%M}" if rn else self.stem
        return f"{when} #{self.id} {self.type} {self.cls} {self.level:+.2f}"


def _event_level(e: dict) -> float:
    lv = e.get("level")
    if lv is None:
        lv = e.get("level_from") if e.get("type") == "release" else e.get("level_to")
    lv = [x for x in (lv or []) if x is not None]
    if not lv:
        return 0.0
    ax = e.get("axis")
    if ax in mn.AXES:
        return float(lv[mn.AXES.index(ax)])
    return float(max(lv, key=abs))


def _ref_of(rec: str, doc: dict, e: dict, map_id, wset) -> EventRef:
    ctx = e.get("context") or {}
    q = e.get("quality") or {}
    bt = ctx.get("baseline_t")
    hold = e.get("duration_s") if e.get("kind") == "hold" else e.get("next_s")
    stem = doc.get("stem") or recname.stem_of(rec)
    rn = recname.parse(stem)
    return EventRef(
        recording=str(rec), stem=stem, id=int(e["id"]), t=float(e["t"]),
        t_start=float(e.get("t_start", e["t"])), t_end=float(e.get("t_end", e["t"])),
        kind=e.get("kind", ""), type=e.get("type", ""), cls=e.get("class", ""),
        axis=e.get("axis"), sign=int(e.get("sign") or 0), level=_event_level(e),
        hold_s=None if hold is None else float(hold), mode=ctx.get("mode"),
        depth_cm=ctx.get("depth_cm"), depth_ok=bool((doc.get("recording") or {}).get("depth_ok", True)),
        baseline_t=tuple(bt) if bt else None,
        baseline_clean=bool(ctx.get("baseline_clean", q.get("baseline_clean", False))),
        steady=bool(q.get("steady", False)), minor=_minor_levels(e.get("minor")),
        alive={k: p.get("alive") for k, p in (e.get("positions") or {}).items()},
        no_reference=not (doc.get("recording") or {}).get("reference"),
        flags={k: q.get(k) for k in ("adc_flagged_scans", "railed", "lost")},
        map_id=map_id, whisker_set=wset, day=rn.date if rn else "",
        positions=e.get("positions") or {}, motion=e.get("motion") or {})


def _minor_levels(m) -> dict:
    """The sidecar's minor inputs ({axis: {"max", "share"}}, round 2; a bare level
    in older ones) as {axis: the largest |u|}."""
    out = {}
    for k, v in (m or {}).items():
        try:
            out[k] = float(v.get("max", 0.0)) if isinstance(v, dict) else float(v)
        except (TypeError, ValueError):
            continue
    return out


def _matches(q: EventQuery, r: EventRef) -> bool:
    if q.kinds and r.kind not in q.kinds:
        return False
    if q.types and r.type not in q.types:
        return False
    if q.classes and r.cls not in q.classes:
        return False
    a = abs(r.level)
    if q.level_min is not None and a < q.level_min - 1e-9:
        return False
    if q.level_max is not None and a > q.level_max + 1e-9:
        return False
    if q.modes and r.mode not in q.modes:
        return False
    if q.hold_min_s is not None and (r.hold_s is None or r.hold_s < q.hold_min_s):
        return False
    if q.baseline_clean and not r.baseline_clean:
        return False
    if q.steady and not r.steady:
        return False
    if q.minor is False and r.minor:
        return False
    if q.minor is not None and not isinstance(q.minor, bool):
        if r.minor and max(abs(float(v)) for v in r.minor.values()) > float(q.minor) + 1e-9:
            return False
    if q.depth_min_cm is not None or q.depth_max_cm is not None:
        if not r.depth_ok or r.depth_cm is None:
            return False
        if q.depth_min_cm is not None and r.depth_cm < q.depth_min_cm:
            return False
        if q.depth_max_cm is not None and r.depth_cm > q.depth_max_cm:
            return False
    if q.alive or q.alive_known:
        keys = r.alive.keys() if q.alive in ("all", None, []) else q.alive
        if any(r.alive.get(k) is False for k in keys):
            return False
        if q.alive_known and any(r.alive.get(k) is not True for k in keys):
            return False
    if q.whisker_sets and r.whisker_set not in q.whisker_sets:
        return False
    if q.map_ids and r.map_id not in q.map_ids:
        return False
    return True


def _level_bin(a: float, edges) -> str:
    e = list(edges)
    for lo, hi in zip(e[:-1], e[1:]):
        if lo <= a < hi:
            return f"|u| {lo:.2f}-{min(hi, 1.0):.2f}"
    return f"|u| >= {e[-1]:.2f}" if a >= e[-1] else f"|u| < {e[0]:.2f}"


def group_of(q: EventQuery, r: EventRef) -> str:
    g = q.group_by
    if g == "run":
        return r.stem
    if g == "day":
        return r.day
    if g == "level":
        return _level_bin(abs(r.level), q.level_edges)
    if g == "mode":
        return r.mode or "?"
    if g == "whisker_set":
        return r.whisker_set or "?"
    return "all"


# ---------------------------------------------------------------------------
# reading the sidecars
# ---------------------------------------------------------------------------
_DOCS: dict = {}                    # sidecar path -> (mtime_ns, size, doc): this process's


def expand_recordings(entries) -> list[Path]:
    """Recording paths of ``entries``: files as they are, folders' ``*.jsonl``
    (sorted, no duplicates)."""
    out, seen = [], set()
    for e in entries:
        p = Path(e)
        items = sorted(p.glob("*.jsonl")) if p.is_dir() else [p]
        for x in items:
            k = str(x.resolve()) if x.exists() else str(x)
            if k not in seen:
                seen.add(k)
                out.append(x)
    return out


def read_sidecar(recording: Union[str, Path], sidecar_dir=None, build: bool = True) -> Optional[dict]:
    """The recording's current sidecar (made when missing or outdated and
    ``build``); None if there is none and it could not be made."""
    rec = Path(recording)
    if sidecar_dir is None:
        sidecar_dir = mn.sidecar_dir_for(rec)
    mp = mn.manoeuvres_path(rec, sidecar_dir)
    try:
        st = mp.stat()
        current = not rec.exists() or st.st_mtime_ns >= rec.stat().st_mtime_ns
    except OSError:
        st, current = None, False
    if st is not None and current:
        got = _DOCS.get(str(mp))
        if got is not None and got[:2] == (st.st_mtime_ns, st.st_size):
            return got[2]
        doc = mn.read_doc(mp)
        if doc is not None:
            _DOCS[str(mp)] = (st.st_mtime_ns, st.st_size, doc)
            return doc
    if not build or not rec.exists():
        return None
    try:
        out, doc, _ = mn.write_manoeuvres_for(rec, sidecar_dir, cache=sidecar_dir is None)
    except Exception:                   # a broken recording: reported as missing
        return None
    st = out.stat()
    _DOCS[str(out)] = (st.st_mtime_ns, st.st_size, doc)
    return doc


def _map_of_doc(rec: Path, doc: dict) -> tuple:
    r = doc.get("recording") or {}
    if "dome_map" in r:
        return r.get("dome_map"), r.get("whisker_set")
    from wavi_py.replay.dome_maps import dome_map_for
    m = dome_map_for(rec)
    return m.id, m.whisker_set


@dataclass
class EventSet:
    """The events of a query: ``events`` (included, by group then recording and
    time; pinned flagged), ``excluded`` (curated out), ``groups`` (name -> count,
    in order), ``missing`` (recordings without a sidecar), ``query_s``."""
    query: EventQuery
    events: list
    excluded: list
    groups: "OrderedDict[str, int]"
    missing: list
    recordings: list
    query_s: float = 0.0

    @classmethod
    def from_query(cls, query: EventQuery) -> "EventSet":
        t0 = time.perf_counter()
        pins = {(s, int(i)) for s, i in query.pins}
        excl = {(s, int(i)) for s, i in query.excludes}
        recs = expand_recordings(query.recordings)
        got, missing, used = [], [], []
        for rec in recs:
            doc = read_sidecar(rec, query.sidecar_dir, query.build_missing)
            if doc is None:
                missing.append(str(rec))
                continue
            used.append(str(rec))
            map_id, wset = _map_of_doc(rec, doc)
            for e in doc.get("events", []):
                r = _ref_of(str(rec), doc, e, map_id, wset)
                if not _matches(query, r):
                    continue
                r.group = group_of(query, r)
                r.pinned = r.key in pins
                r.excluded = r.key in excl
                got.append(r)
        order: "OrderedDict[str, int]" = OrderedDict()
        names = sorted({r.group for r in got}, key=lambda g: _group_sort(query, g))
        for g in names:
            order[g] = 0
        got.sort(key=lambda r: (names.index(r.group), r.stem, r.t))
        inc = [r for r in got if not r.excluded]
        for r in inc:
            order[r.group] += 1
        return cls(query, inc, [r for r in got if r.excluded], order, missing, used,
                   time.perf_counter() - t0)

    def __len__(self) -> int:
        return len(self.events)

    def rows(self, include_excluded: bool = True) -> list[dict]:
        """One flat dict per event, for a table or a scatter (stage 2): the event's
        fields and per position ``<pos>_drag_peak`` / ``_steady``, ``_lift_*``,
        ``<pos>_alive``."""
        evs = self.events + (self.excluded if include_excluded else [])
        out = []
        for r in evs:
            row = {"stem": r.stem, "id": r.id, "group": r.group, "t": r.t, "kind": r.kind,
                   "type": r.type, "class": r.cls, "axis": r.axis, "level": r.level,
                   "hold_s": r.hold_s, "mode": r.mode, "depth_cm": r.depth_cm,
                   "baseline_clean": r.baseline_clean, "steady": r.steady,
                   "minor": max((abs(float(v)) for v in r.minor.values()), default=0.0),
                   "no_reference": r.no_reference,
                   "lost": [k for k, a in r.alive.items() if a is False],
                   "map_id": r.map_id, "whisker_set": r.whisker_set, "day": r.day,
                   "pinned": r.pinned, "excluded": r.excluded}
            for k, p in r.positions.items():
                for ax in ("drag", "lift"):
                    f = p.get(ax) or {}
                    row[f"{k}_{ax}_peak"] = f.get("peak")
                    row[f"{k}_{ax}_steady"] = f.get("steady")
                row[f"{k}_alive"] = p.get("alive")
            out.append(row)
        return out


def _group_sort(q: EventQuery, g: str):
    if q.group_by == "level":
        m = re.search(r"([0-9.]+)", g)
        return (0 if "<" in g else 1, float(m.group(1)) if m else 0.0)
    return (0, g)


# ---------------------------------------------------------------------------
# saved comparisons
# ---------------------------------------------------------------------------
def comparisons_dir() -> Path:
    """Where named queries live: ``$WAVI_COMPARISONS_DIR``; else next to the
    settings (``$WAVI_SETTINGS_INI``'s folder); else the user config dir
    (%APPDATA%\\wavi_py, ~/Library/Application Support/wavi_py,
    $XDG_CONFIG_HOME (~/.config)/wavi_py), ``comparisons/``."""
    env = os.environ.get("WAVI_COMPARISONS_DIR")
    if env:
        p = Path(env)
    elif os.environ.get("WAVI_SETTINGS_INI"):
        p = Path(os.environ["WAVI_SETTINGS_INI"]).parent / "comparisons"
    elif sys.platform == "win32":
        p = Path(os.environ.get("APPDATA") or Path.home() / "AppData" / "Roaming") / "wavi_py" / "comparisons"
    elif sys.platform == "darwin":
        p = Path.home() / "Library" / "Application Support" / "wavi_py" / "comparisons"
    else:
        p = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config") / "wavi_py" / "comparisons"
    return p


def _safe(name: str) -> str:
    s = re.sub(r"[^\w.\- ]+", "_", name.strip()).strip(". ")
    if not s:
        raise ValueError("a comparison needs a name")
    return s


def list_comparisons() -> list[str]:
    d = comparisons_dir()
    return sorted(p.name[:-len(COMPARISON_SUFFIX)] for p in d.glob(f"*{COMPARISON_SUFFIX}"))


def save_comparison(name: str, query: EventQuery) -> Path:
    d = comparisons_dir()
    d.mkdir(parents=True, exist_ok=True)
    p = d / f"{_safe(name)}{COMPARISON_SUFFIX}"
    doc = {"name": name, "saved": time.strftime("%Y-%m-%dT%H:%M:%S"), "query": query.to_dict()}
    return mn.write_doc(p, doc)


def load_comparison(name: str) -> EventQuery:
    p = comparisons_dir() / f"{_safe(name)}{COMPARISON_SUFFIX}"
    doc = json.loads(p.read_text(encoding="utf-8"))
    return EventQuery.from_dict(doc["query"])


def delete_comparison(name: str) -> bool:
    p = comparisons_dir() / f"{_safe(name)}{COMPARISON_SUFFIX}"
    try:
        p.unlink()
        return True
    except FileNotFoundError:
        return False


# ---------------------------------------------------------------------------
# stores and selections
# ---------------------------------------------------------------------------
class StoreCache:
    """Recording path -> (``RecordingIndex``, ``RecordingStore``), opened on first
    use, least recently used dropped while the stores' ``nbytes`` exceed
    ``cap_bytes`` (the newest is always kept). ``disk``: the replay caches,
    "next" (next to the recording, else the user cache: as replay), "user" (only
    the user cache: for folders that are not ours) or None (built each time)."""

    def __init__(self, cap_bytes: int = STORE_CAP_BYTES, disk: Optional[str] = "next") -> None:
        self.cap_bytes = int(cap_bytes)
        self.disk = disk
        self._items: "OrderedDict[str, tuple]" = OrderedDict()
        self.opened = 0                 # stores opened (not from memory)

    def __len__(self) -> int:
        return len(self._items)

    def __contains__(self, path) -> bool:
        return str(Path(path)) in self._items

    @property
    def nbytes(self) -> int:
        return sum(st.nbytes for _, st in self._items.values())

    def get(self, path: Union[str, Path]) -> tuple:
        k = str(Path(path))
        got = self._items.get(k)
        if got is not None:
            self._items.move_to_end(k)
            return got
        got = self._open(Path(path))
        self.opened += 1
        self._items[k] = got
        while len(self._items) > 1 and self.nbytes > self.cap_bytes:
            self._items.popitem(last=False)
        return got

    def _open(self, path: Path) -> tuple:
        from wavi_py.replay.index import RecordingIndex, cache_paths
        from wavi_py.replay.store import RecordingStore
        if self.disk == "next":
            idx = RecordingIndex.open(path)
            return idx, RecordingStore.open(idx)
        if self.disk is None:
            idx = RecordingIndex.build(path)
            return idx, RecordingStore.build(idx)
        c = cache_paths(path)[-1]
        sc = c.with_name(c.name.replace(".idx.npz", ".store.npz"))
        idx = RecordingIndex._load_cache(c, path, path.stat())
        if idx is None:
            idx = RecordingIndex.build(path)
            _try_save(lambda: idx._save_cache(c), c)
        st = RecordingStore._load_cache(sc, idx)
        if st is None:
            st = RecordingStore.build(idx)
            _try_save(lambda: st._save_cache(sc, idx), sc)
        return idx, st

    def clear(self) -> None:
        self._items.clear()


def _try_save(fn, path: Path) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fn()
    except OSError:
        pass


def event_selections(es: EventSet, cache: StoreCache, number0: int = 1,
                     progress=None) -> list:
    """``daq/analysis.Selection``\\ s of the set's events (see the module doc), in
    its order; an event whose recording does not open is left out.
    ``progress(k, n, stem)`` before each recording's store is fetched."""
    from wavi_py.daq.analysis import SelSource, Selection
    from wavi_py.replay.dome_maps import dome_map_for
    maps: dict = {}
    sources: dict = {}
    out = []
    recs = list(dict.fromkeys(r.recording for r in es.events))
    for r in es.events:
        if r.recording not in sources:
            if progress is not None:
                progress(len(sources), len(recs), r.stem)
            try:
                idx, st = cache.get(r.recording)
            except (OSError, ValueError, KeyError, IndexError):
                sources[r.recording] = None
                continue
            sources[r.recording] = SelSource(st, idx.depth())
            maps[r.recording] = dome_map_for(r.recording)
        src = sources[r.recording]
        if src is None:
            continue
        ext = src.extent()
        t0, t1 = r.t_start, r.t_end
        if ext is not None:
            t0, t1 = max(t0, ext[0]), min(t1, ext[1])
        if t1 <= t0:
            continue
        out.append(Selection(number0 + len(out), t0, t1, src, ext, align=r.t - t0,
                             group=r.group, pinned=r.pinned, baseline=r.baseline_t,
                             dome_map=maps[r.recording], label=r.label, event=r))
    return out


# ---------------------------------------------------------------------------
# the overlay: what the event waveform draws (computed off the GUI thread)
# ---------------------------------------------------------------------------
@dataclass
class Overlay:
    """The traces of an event set for the event waveform: ``per_event[(number,
    key)] = (t, y)`` of the drawn selections (time from the onset, decimated);
    ``thin[(group, key)] = (t, y, owner)``: a group's drawn traces of one variable
    in one curve (NaN between events; ``owner`` the selection number of each
    point); ``aggregates[key]``: ``Dataset.aggregate`` over every event."""
    selections: list
    drawn: list
    per_event: dict
    thin: dict
    aggregates: dict
    variables: list
    compute_s: float = 0.0

    @property
    def by_number(self) -> dict:
        return {s.number: s for s in self.selections}


def overlay(selections: list, variables: list, n_px: int = 800,
            max_traces: Optional[int] = None, aggregates: bool = True) -> Overlay:
    """The ``Overlay`` of ``selections`` (event selections) for ``variables``:
    baseline mode (``daq/analysis.Dataset``), at most ``max_traces`` drawn."""
    from wavi_py.daq.analysis import MAX_TRACES, Dataset
    import numpy as np
    t0 = time.perf_counter()
    ds = Dataset(selections, list(variables), baseline=True,
                 max_traces=MAX_TRACES if max_traces is None else max_traces)
    drawn = ds.drawn()
    per = {}
    for tr in ds.traces(n_px):
        per[(tr.sel.number, tr.var.key)] = (tr.t, tr.y)
    thin = {}
    groups = list(dict.fromkeys(s.group for s in drawn))
    for g in groups:
        sels = [s for s in drawn if s.group == g]
        for v in variables:
            ts, ys, os_ = [], [], []
            for s in sels:
                got = per.get((s.number, v.key))
                if got is None or not len(got[0]):
                    continue
                ts += [got[0], [np.nan]]
                ys += [got[1], [np.nan]]
                os_ += [np.full(len(got[0]) + 1, s.number, np.int32)]
            if ts:
                thin[(g, v.key)] = (np.concatenate(ts), np.concatenate(ys), np.concatenate(os_))
    ag = {v.key: ds.aggregate(v) for v in variables} if aggregates else {}
    return Overlay(list(selections), drawn, per, thin, ag, list(variables),
                   time.perf_counter() - t0)


def event_traces(sel, variables: list, n_px: int = 800) -> dict:
    """``{key: (t, y)}`` of one event selection (a highlighted one beyond the cap)."""
    from wavi_py.daq.analysis import Dataset
    ds = Dataset([sel], list(variables), baseline=True, max_traces=None)
    return {tr.var.key: (tr.t, tr.y) for tr in ds.traces(n_px)}


# ---------------------------------------------------------------------------
# features: the table's column and the scatter's axes
# ---------------------------------------------------------------------------
QUANTITIES = ("steady", "peak", "band RMS", "tau", "rise 63 %")
EVENT_FIELDS = ("level", "|level|", "hold", "depth", "t")


def feature(r: EventRef, source: str, quantity: str = "steady"):
    """A number of the event: ``source`` an ``EVENT_FIELDS`` name, a position axis
    (``"LFM drag"``) or a motion channel (``"motion acc_x"``), ``quantity`` one of
    ``QUANTITIES`` (positions: steady / peak / band RMS (uV) / tau; motion:
    steady / peak / tau / rise 63 %); None where the sidecar has none."""
    if source == "level":
        return r.level
    if source == "|level|":
        return abs(r.level)
    if source == "hold":
        return r.hold_s
    if source == "depth":
        return r.depth_cm
    if source == "t":
        return r.t
    if source.startswith("motion "):
        m = r.motion.get(source[7:]) or {}
        k = {"steady": "steady", "peak": "peak", "tau": "tau_s", "rise 63 %": "rise63_s"}.get(quantity)
        return m.get(k) if k else None
    pos, _, ax = source.partition(" ")
    p = r.positions.get(pos) or {}
    f = p.get(ax) or {}
    k = {"steady": "steady", "peak": "peak", "band RMS": "band_uv", "tau": "tau_s"}.get(quantity)
    return f.get(k) if k else None


def feature_sources(es: Optional[EventSet] = None) -> list[str]:
    """The ``feature`` sources there are: positions' axes (dome order), then motion."""
    from wavi_py.daq.analysis import DOME_ORDER
    out = [f"{p} {ax}" for p in DOME_ORDER for ax in ("drag", "lift")]
    return out + [f"motion {m}" for m in mn.MOTION]
