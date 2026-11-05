"""Recording cards: ``<stem>.card.json``, a small summary of one recording.

Written once when a recording stops (``write_card_for``, in a child process
from ``main_window._stop_recording``) and by ``tools/make_cards.py`` for old
ones, so the recording browser can list a folder from one ``ssh host cat`` of
its cards. JSON, under 20 KB (32 gauges: ~11 KB, most of it the preview)::

    {"v": 1, "stem", "start" (ISO local time), "tag", "duration_s", "fs_hz", "nch",
     "n_scans", "n_imu", "spikes", "faults": {"warn": n, "fault": n} | null,
     "depth_m": [min, max] | null, "armed_s": s | null,
     "preview": {"kind": "rms_db", "nbins", "lo_db", "hi_db", "data": base64},
     "depth_strip": base64 | null, "made_by"}

The preview: each gauge's RMS about its own mean (the standard deviation) over
each of ``nbins`` equal spans of session time, in dB re 1 mV (``20 log10``),
from the despiked scans, quantised to 0..255 between ``LO_DB`` and ``HI_DB``;
``data`` is the nch x nbins uint8 array, row = gauge (the store's column
order). The range is fixed so cards compare: on the pilot2 dives and the
vac_1102 bench run a quiet gauge sits at -26..-33 dB (0.02-0.05 mV, the ADC
noise), touches and handling reach +5..+12 dB, and the dead mux slots (railed,
constant) give no variance at all. So ``LO_DB`` -40 puts the noise floor
at a dim grey (~50), a dead gauge and a bin without scans at 0, and ``HI_DB``
+15 keeps the strongest bins of those recordings just short of white.

``depth_strip``: the deepest depth in each bin, 0..255 between ``depth_m``'s
min and max (so it reads back in metres from the card alone); null without
the external pressure sensor.

``faults`` counts the ESP32 / ADC fault marks of the stream (null for a
recording made from a .dat, which has none); ``armed_s`` the armed time (null
without the vehicle).
"""

from __future__ import annotations

import base64
import json
import os
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Union

import numpy as np

from wavi_py.replay import recname
from wavi_py.replay.index import RecordingIndex, cache_paths
from wavi_py.replay.store import RecordingStore

VERSION = 1
SUFFIX = ".card.json"
NBINS = 256
LO_DB = -40.0
HI_DB = 15.0
MADE_BY = f"wavi_py card v{VERSION}"


def card_path(recording: Union[str, Path]) -> Path:
    return recname.sidecar(recording, SUFFIX)


def _b64(a: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(a, dtype=np.uint8).tobytes()).decode("ascii")


def rms_db(t: np.ndarray, y: np.ndarray, duration: float, nbins: int = NBINS) -> np.ndarray:
    """(nch, nbins) dB re 1 mV of each column's standard deviation over ``nbins``
    equal spans of ``[0, duration]``; -inf where a bin is constant, NaN where it
    has no samples. One pass of sums (``np.add.reduceat``), in float64 about each
    column's mean, so the offsets (hundreds of mV) don't eat the precision."""
    nch = y.shape[1] if y.ndim == 2 else 0
    out = np.full((nch, nbins), np.nan)
    if not len(t) or not nch or duration <= 0:
        return out
    edges = np.searchsorted(t, np.linspace(0.0, duration, nbins + 1))
    edges[-1] = len(t)
    cnt = np.diff(edges)
    full = np.nonzero(cnt > 0)[0]
    if not len(full):
        return out
    x = y.astype(np.float64)
    x -= x.mean(axis=0)
    starts = edges[full]
    s1 = np.add.reduceat(x, starts, axis=0)
    s2 = np.add.reduceat(x * x, starts, axis=0)
    # reduceat sums from each start to the next start: the bins in between are empty
    c = cnt[full][:, None].astype(np.float64)
    var = np.maximum(s2 / c - (s1 / c) ** 2, 0.0)
    with np.errstate(divide="ignore"):
        out[:, full] = (10.0 * np.log10(var)).T
    return out


def quantise(db: np.ndarray, lo: float = LO_DB, hi: float = HI_DB) -> np.ndarray:
    q = np.round((np.nan_to_num(db, nan=lo, neginf=lo) - lo) * (255.0 / (hi - lo)))
    return np.clip(q, 0, 255).astype(np.uint8)


def _bin_max(t: np.ndarray, v: np.ndarray, duration: float, nbins: int) -> np.ndarray:
    """Max of ``v`` in each bin (NaN where empty)."""
    out = np.full(nbins, np.nan)
    if not len(t) or duration <= 0:
        return out
    b = np.clip((t / duration * nbins).astype(np.int64), 0, nbins - 1)
    ok = np.isfinite(v)
    np.fmax.at(out, b[ok], v[ok])
    return out


def build_card(store: RecordingStore, index: Optional[RecordingIndex] = None,
               name: Union[str, Path, None] = None, nbins: int = NBINS) -> dict:
    """The card of a recording from its store (and index, for the spikes, faults,
    depth and armed time). ``name`` is the recording's file (for the stem, start
    and tag); the index's path by default."""
    name = Path(name) if name is not None else (Path(index.path) if index is not None else None)
    stem = recname.stem_of(name) if name is not None else ""
    rn = recname.parse(stem) if stem else None
    duration = float(index.duration) if index is not None else store.duration
    duration = max(duration, store.duration)
    wsk = store.despiked() if len(store.spk_row) else store.wsk
    q = quantise(rms_db(store.wsk_t, wsk, duration, nbins))
    if rn is not None:
        start = rn.start.isoformat(timespec="seconds")
    elif index is not None and index.unix0 > 0:
        start = datetime.fromtimestamp(index.unix0).isoformat(timespec="seconds")
    else:
        start = None
    card = {"v": VERSION, "stem": stem, "start": start, "tag": rn.tag if rn else "",
            "duration_s": round(duration, 3),
            "fs_hz": round(float(index.fs), 4) if index is not None else None,
            "nch": int(wsk.shape[1]), "n_scans": int(len(store.wsk_t)),
            "n_imu": int(len(store.imu_t)),
            "spikes": int(index.n_spikes) if index is not None else int(len(store.spk_row)),
            "faults": None, "depth_m": None, "armed_s": None,
            "preview": {"kind": "rms_db", "nbins": nbins, "lo_db": LO_DB, "hi_db": HI_DB,
                        "data": _b64(q)},
            "depth_strip": None, "made_by": MADE_BY}
    if index is not None:
        if str(index.hello.get("server", "")) != "wavi_py-dat":
            n = {"warn": 0, "fault": 0}
            for e in index.events:
                if e.kind == "fault" and e.level in n:
                    n[e.level] += 1
            card["faults"] = n
        if index.has_vehicle:
            card["armed_s"] = round(float(sum(b - a for a, b in index.armed_spans)), 2)
        dt, dcm = index.depth()
        if len(dcm) and np.isfinite(dcm).any():
            dm = dcm / 100.0
            lo, hi = float(np.nanmin(dm)), float(np.nanmax(dm))
            card["depth_m"] = [round(lo, 3), round(hi, 3)]
            strip = _bin_max(np.asarray(dt, np.float64), dm, duration, nbins)
            span = max(hi - lo, 1e-6)
            card["depth_strip"] = _b64(np.clip(np.round(
                (np.nan_to_num(strip, nan=lo) - lo) * (255.0 / span)), 0, 255))
    return card


def decode_preview(card: dict) -> np.ndarray:
    """The preview as an (nch, nbins) uint8 array."""
    p = card["preview"]
    a = np.frombuffer(base64.b64decode(p["data"]), dtype=np.uint8)
    return a.reshape(int(card["nch"]), int(p["nbins"]))


def decode_depth(card: dict) -> Optional[np.ndarray]:
    """The depth strip in metres (nbins), or None."""
    if not card.get("depth_strip") or not card.get("depth_m"):
        return None
    lo, hi = card["depth_m"]
    a = np.frombuffer(base64.b64decode(card["depth_strip"]), dtype=np.uint8).astype(np.float64)
    return lo + a * ((hi - lo) / 255.0)


def write_card(path: Union[str, Path], card: dict) -> Path:
    """Write ``card`` (a temporary file, then replace: a reader never sees half)."""
    path = Path(path)
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(card, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    for i in range(20):
        try:
            os.replace(tmp, path)
            return path
        except PermissionError:          # Windows: someone has it open; briefly
            if i == 19:
                tmp.unlink(missing_ok=True)
                raise
            time.sleep(0.1 * (i + 1))
    return path


def read_card(path: Union[str, Path]) -> Optional[dict]:
    """The card at ``path``; None if missing, unreadable or of another version."""
    try:
        card = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return card if isinstance(card, dict) and card.get("v") == VERSION else None


def is_current(recording: Union[str, Path]) -> bool:
    """A card of this version exists and is newer than the recording."""
    rec, cp = Path(recording), card_path(recording)
    try:
        if cp.stat().st_mtime_ns < rec.stat().st_mtime_ns:
            return False
    except OSError:
        return False
    return read_card(cp) is not None


def write_card_for(recording: Union[str, Path], cache_store: bool = False) -> tuple[Path, dict, dict]:
    """Index (cached as usual) and load the recording, build its card and write it
    next to the recording. The whole-recording store is read from its cache if a
    replay left one; ``cache_store`` also saves it when built (``.store.npz``,
    tens of MB; off by default: a card needs it once). Returns
    (card path, card, timings in s: index, store, card)."""
    rec = Path(recording)
    t0 = time.perf_counter()
    index = RecordingIndex.open(rec)
    t1 = time.perf_counter()
    store = None
    for c in cache_paths(index.path):                 # a replay may have cached it already
        store = RecordingStore._load_cache(c.with_name(c.name.replace(".idx.npz", ".store.npz")),
                                           index)
        if store is not None:
            break
    if store is None:
        store = RecordingStore.open(index) if cache_store else RecordingStore.build(index)
    t2 = time.perf_counter()
    card = build_card(store, index, name=rec)
    t3 = time.perf_counter()
    out = write_card(card_path(rec), card)
    return out, card, {"index": t1 - t0, "store": t2 - t1, "card": t3 - t2}


def write_card_in_background(recording: Union[str, Path], on_done=None) -> threading.Thread:
    """The card of a recording that just closed, without touching the GUI's time:
    a thread that runs ``tools/make_cards.py`` on it in a child process at a lower
    priority (a build in this process would hold the GIL for seconds, under the
    window that may be recording the next one; the child also finishes if the
    window quits). Where no child can be started (a frozen app), the thread builds
    it itself. ``on_done(path, error or None)`` is called from that thread."""
    rec = Path(recording)

    def run() -> None:
        err = None
        try:
            if getattr(sys, "frozen", False):
                write_card_for(rec)
            else:
                root = Path(__file__).resolve().parents[2]       # the folder holding wavi_py
                env = dict(os.environ)
                env["PYTHONPATH"] = os.pathsep.join(
                    [str(root)] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
                env["PYTHONIOENCODING"] = "utf-8"
                flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
                r = subprocess.run(
                    [sys.executable, "-m", "wavi_py.tools.make_cards", "--quiet", "--nice",
                     "--force", str(rec)],
                    cwd=str(root), env=env, stdin=subprocess.DEVNULL, capture_output=True,
                    timeout=1800, creationflags=flags)
                if r.returncode != 0:
                    err = (r.stderr.decode("utf-8", "replace").strip().splitlines() or ["?"])[-1]
        except Exception as exc:  # noqa: BLE001 — a card is an aid: report, never raise
            err = f"{type(exc).__name__}: {exc}"
        print(f"[card] {rec.name}: " + (f"not written ({err})" if err else
                                       f"wrote {card_path(rec).name}"))
        if on_done is not None:
            on_done(card_path(rec), err)

    th = threading.Thread(target=run, name="wavi-card", daemon=True)
    th.start()
    return th
