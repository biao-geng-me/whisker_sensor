"""The recording index: ``trials.jsonl`` next to daq_server's recordings.

Append-only JSON lines, written by the server (``rpi/daq_server.py``), so it
survives client crashes and two clients can't overwrite each other. Three kinds
of line, all with ``file`` (the recording's name in the same folder) and
``unix`` (when the line was written)::

    {"ev": "start", "file": ..., "tag": ..., "by": ..., "duration": s or null, "unix": ...}
    {"ev": "stop",  "file": ..., "how": ..., "why": ..., "by": ..., "secs": ..., "unix": ...}
    {"ev": "flag",  "file": ..., "keep": false, "note": ..., "by": ..., "unix": ...}

``how`` is ``user`` (a client's rec_stop), ``auto`` (its duration elapsed),
``replaced`` (a forced rec_start), ``restart`` (the server was stopped, e.g.
``systemctl restart``), or ``crash`` (the server's stall watchdog, or a start
with no stop, found when the server next starts). ``flag`` lines mark a
recording bad (``keep`` false) or good again; the last one wins. Nothing is
ever deleted: a bad recording stays on disk, and ``whisker_link.convert`` skips it.

Python 3.9, stdlib only (it runs on the Pi).
"""

import json
import os
import time
from typing import Dict, List, Optional

INDEX_NAME = "trials.jsonl"


def append(index_path: str, rec: dict) -> None:
    """One line, flushed and synced (a few per recording, so the cost doesn't matter)."""
    line = json.dumps(dict(rec, unix=rec.get("unix", time.time())), separators=(",", ":"))
    d = os.path.dirname(index_path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(index_path, "a", encoding="utf-8") as f:
        f.write(line + "\n")
        f.flush()
        try:
            os.fsync(f.fileno())
        except OSError:
            pass


def read_lines(index_path: str) -> List[dict]:
    """The index's lines in order; a missing file is empty, a torn last line skipped."""
    out: List[dict] = []
    try:
        with open(index_path, encoding="utf-8") as f:
            for line in f:
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue
                if isinstance(ev, dict) and ev.get("file"):
                    out.append(ev)
    except FileNotFoundError:
        pass
    return out


def summarize(lines: List[dict]) -> Dict[str, dict]:
    """file -> one record: tag, by, duration, start, stop, secs, how, why, keep, note
    (in start order). ``stop`` None: no stop line (still running, or not yet found)."""
    recs: Dict[str, dict] = {}
    for ev in lines:
        fn = ev["file"]
        r = recs.setdefault(fn, {"file": fn, "tag": "", "by": "", "duration": None,
                                 "start": None, "stop": None, "secs": None, "how": None,
                                 "why": "", "keep": True, "note": ""})
        kind = ev.get("ev")
        if kind == "start":
            r.update(tag=ev.get("tag", ""), by=ev.get("by", ""),
                     duration=ev.get("duration"), start=ev.get("unix"))
        elif kind == "stop":
            r.update(stop=ev.get("unix"), secs=ev.get("secs"), how=ev.get("how"),
                     why=ev.get("why", ""))
            if ev.get("by"):
                r["stopped_by"] = ev["by"]
        elif kind == "flag":
            r.update(keep=bool(ev.get("keep", True)), note=ev.get("note", ""))
    return recs


def read(index_path: str) -> Dict[str, dict]:
    return summarize(read_lines(index_path))


def index_for(recording: str) -> str:
    """The index that belongs to a recording: the one in its folder."""
    return os.path.join(os.path.dirname(os.path.abspath(recording)), INDEX_NAME)


def status_of(recording: str) -> Optional[dict]:
    """The index record of ``recording`` (path or name), or None if not indexed."""
    return read(index_for(recording)).get(os.path.basename(recording))


def close_unfinished(index_path: str, running: Optional[str] = None) -> List[str]:
    """Starts with no stop (the server died with them open): append a ``crash``
    stop for each, its length estimated from the file's last write. Returns them."""
    done = []
    rec_dir = os.path.dirname(index_path)
    for fn, r in read(index_path).items():
        if r["stop"] is not None or r["start"] is None or fn == running:
            continue
        try:
            secs = round(max(0.0, os.path.getmtime(os.path.join(rec_dir, fn)) - r["start"]), 1)
        except OSError:
            secs = None
        append(index_path, {"ev": "stop", "file": fn, "how": "crash",
                            "why": "no stop recorded: the server ended without closing it"
                                   " (found at the next start)",
                            "by": "server", "secs": secs})
        done.append(fn)
    return done
