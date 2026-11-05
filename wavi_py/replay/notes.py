"""Recording notes: ``<stem>.notes.jsonl``, append-only JSON lines.

The same idea as the Pi's recording index (``whisker_link/rec_index.py``,
``trials.jsonl``): nothing is rewritten in place, so two copies (laptop and
cluster) merge without conflict as the union of their lines by ``id``. One line
per event (``v`` 1):

- ``{"ev": "note", "t": 123.4 | null, "text": ...}``: a comment, pinned to
  ``t`` seconds into the recording (the session time the replay timeline
  uses), or on the whole recording (null);
- ``{"ev": "flag", "keep": false, "text": "optional reason"}``: Mark bad;
  ``keep: true`` is Unmark bad. The newest flag by ``unix`` wins;
- ``{"ev": "del", "ref": "<id of a note>"}``: delete a note. An edit is a del
  and a new note.

Every line also has ``v``, ``id`` (12 hex), ``by`` and ``unix``.

Writes: ``append`` writes one whole line with one ``os.write`` on an
``O_APPEND`` descriptor, under an exclusive lock (``fcntl.flock`` on POSIX,
``msvcrt.locking`` of a byte far past the end on Windows, where an append is a
seek and a write and so not atomic between processes), so two windows (or a
window and the sync) appending to one file never interleave. A crash mid-write
leaves a torn last line, which ``read`` skips (and the next append starts on a
new line). ``append_missing`` (the sync bringing in the cluster's lines)
appends the lines whose id the file lacks, under the same lock and in one
write, so a merge never races an append and never rewrites the file (a
replace would orphan an appender already waiting on the old file's lock).
``write_lines`` writes a temporary file and replaces, retrying while Windows
refuses because another process has the file open; it is not safe against
concurrent appends.
"""

from __future__ import annotations

import json
import os
import secrets
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional, Union

from wavi_py.replay.recname import sidecar

VERSION = 1
SUFFIX = ".notes.jsonl"
_LOCK_AT = 1 << 40               # Windows: the byte locked (never read or written)


def notes_path(recording: Union[str, Path]) -> Path:
    """The notes file of a recording (any of its files): ``<stem>.notes.jsonl``."""
    return sidecar(recording, SUFFIX)


def new_id() -> str:
    return secrets.token_hex(6)


def _line(ev: str, by: str, **fields) -> dict:
    return {"v": VERSION, "id": new_id(), "ev": ev, **fields, "by": by, "unix": time.time()}


def note(text: str, t: Optional[float] = None, by: str = "") -> dict:
    """A note line, pinned to ``t`` (seconds into the recording) or not (None)."""
    return _line("note", by, t=None if t is None else round(float(t), 3), text=str(text))


def flag(keep: bool, text: str = "", by: str = "") -> dict:
    """A flag line: Mark bad (``keep`` False), or Unmark bad (True)."""
    return _line("flag", by, keep=bool(keep), text=str(text))


def delete(ref: str, by: str = "") -> dict:
    return _line("del", by, ref=str(ref))


# ----- read / write

def parse_lines(data: Union[bytes, str]) -> list[dict]:
    """The valid lines of a notes file's contents (a torn or foreign line is
    skipped), in file order."""
    if isinstance(data, bytes):
        data = data.decode("utf-8", errors="replace")
    out = []
    for s in data.splitlines():
        try:
            ev = json.loads(s)
        except ValueError:
            continue
        if isinstance(ev, dict) and ev.get("id") and ev.get("ev"):
            out.append(ev)
    return out


def read(path: Union[str, Path]) -> list[dict]:
    """The lines of the notes file ``path`` in file order; a missing file is empty."""
    try:
        return parse_lines(Path(path).read_bytes())
    except FileNotFoundError:
        return []


@contextmanager
def _locked(fd: int):
    if sys.platform.startswith("win"):
        import msvcrt
        pos = os.lseek(fd, 0, os.SEEK_CUR)
        os.lseek(fd, _LOCK_AT, os.SEEK_SET)
        msvcrt.locking(fd, msvcrt.LK_LOCK, 1)        # retries for ~10 s, then OSError
        try:
            os.lseek(fd, pos, os.SEEK_SET)
            yield
        finally:
            os.lseek(fd, _LOCK_AT, os.SEEK_SET)
            msvcrt.locking(fd, msvcrt.LK_UNLCK, 1)
    else:
        import fcntl
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)


def append(path: Union[str, Path], ev: dict) -> dict:
    """Append the line ``ev`` (from ``note`` / ``flag`` / ``delete``) to ``path``
    (made if missing), flushed to the OS before return; returns ``ev``."""
    path = Path(path)
    data = (json.dumps(ev, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
    flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT | getattr(os, "O_BINARY", 0)
    fd = os.open(path, flags, 0o664)
    try:
        with _locked(fd):
            end = os.lseek(fd, 0, os.SEEK_END)
            if end > 0:                          # a torn last line: start a new one
                with open(path, "rb") as f:
                    f.seek(end - 1)
                    if f.read(1) != b"\n":
                        data = b"\n" + data
            view = memoryview(data)
            while view:
                n = os.write(fd, view)
                view = view[n:]
    finally:
        os.close(fd)
    return ev


def append_missing(path: Union[str, Path], lines: Iterable[dict]) -> tuple[int, int, os.stat_result]:
    """Append to ``path`` (made if missing) the ``lines`` whose ``id`` it lacks,
    under the append lock, re-reading the file inside it: the union by id
    without a rewrite (readers sort; the order in the file means nothing).
    Returns (lines appended, the file's size before, its stat after), the last
    two taken under the lock, so the caller can tell whether the file was what
    it last read."""
    path = Path(path)
    flags = os.O_RDWR | os.O_APPEND | os.O_CREAT | getattr(os, "O_BINARY", 0)
    fd = os.open(path, flags, 0o664)
    try:
        with _locked(fd):
            end = os.lseek(fd, 0, os.SEEK_END)
            with open(path, "rb") as f:
                have = f.read(end)
            ids = {ev.get("id") for ev in parse_lines(have)}
            new, seen = [], set()
            for ev in lines:
                i = ev.get("id")
                if i and i not in ids and i not in seen:
                    seen.add(i)
                    new.append(ev)
            if new:
                data = "".join(json.dumps(ev, ensure_ascii=False, separators=(",", ":")) + "\n"
                               for ev in new).encode("utf-8")
                if have and not have.endswith(b"\n"):
                    data = b"\n" + data             # after a torn last line
                view = memoryview(data)
                while view:
                    n = os.write(fd, view)
                    view = view[n:]
            return len(new), end, os.fstat(fd)
    finally:
        os.close(fd)


def write_lines(path: Union[str, Path], lines: Iterable[dict], tries: int = 20) -> None:
    """Replace ``path`` with ``lines`` (a merged copy): a temporary file, then
    ``os.replace``, retried for a few seconds while another process holds the file
    open (Windows refuses the replace then)."""
    path = Path(path)
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    text = "".join(json.dumps(ev, ensure_ascii=False, separators=(",", ":")) + "\n"
                   for ev in lines)
    tmp.write_bytes(text.encode("utf-8"))
    for i in range(tries):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            if i == tries - 1:
                tmp.unlink(missing_ok=True)
                raise
            time.sleep(0.1 * (i + 1))


def merge(lines_a: Iterable[dict], lines_b: Iterable[dict]) -> list[dict]:
    """Two copies of one notes file as one: the union of the lines by ``id``,
    sorted by ``unix`` (then ``id``, so both sides get the same order)."""
    by_id: dict = {}
    for ev in list(lines_a) + list(lines_b):
        by_id.setdefault(ev.get("id"), ev)
    return sorted(by_id.values(), key=lambda e: (float(e.get("unix") or 0.0), str(e.get("id"))))


# ----- derived state

@dataclass
class NotesState:
    """What a notes file says now: the flag, and the notes not deleted."""
    flag: Optional[dict] = None              # the newest flag line, or None
    notes: list = field(default_factory=list)   # live note lines: pinned by t, then whole ones

    @property
    def bad(self) -> bool:
        return self.flag is not None and self.flag.get("keep", True) is False

    @property
    def reason(self) -> str:
        return str(self.flag.get("text", "")) if self.bad else ""

    @property
    def count(self) -> int:
        """The comments (live notes; the flag's reason not counted)."""
        return len(self.notes)

    @property
    def pinned(self) -> list:
        return [n for n in self.notes if n.get("t") is not None]


def state(lines: Iterable[dict]) -> NotesState:
    lines = list(lines)
    deleted = {ev.get("ref") for ev in lines if ev.get("ev") == "del"}
    flags = [ev for ev in lines if ev.get("ev") == "flag"]
    last = max(flags, key=lambda e: (float(e.get("unix") or 0.0), str(e.get("id")))) if flags else None
    live = [ev for ev in lines if ev.get("ev") == "note" and ev.get("id") not in deleted]
    live.sort(key=lambda e: (e.get("t") is None, float(e.get("t") or 0.0),
                             float(e.get("unix") or 0.0)))
    return NotesState(last, live)


def load(path: Union[str, Path]) -> NotesState:
    return state(read(path))
