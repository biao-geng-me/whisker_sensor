"""Uploading recordings to the cluster, the queue, and pruning local copies.

Layout on the cluster (the recording-manager spec)::

    <root>/YYYY-MM-DD/<stem>.{jsonl,dat,faults.txt,notes.jsonl,card.json}

``YYYY-MM-DD`` is the recording's start date (``replay/recname.py``; the
file's mtime for a name in neither format). Every file of the stem goes, except the
rebuildable caches (``.idx.npz``, ``.store.npz``) and our own temporaries.

One file's upload is one ssh call: the local file is piped into
``cat > <name>.part`` there, and the same remote script then checks the size
and the sha256 against the local ones and only then ``mv``s the part into
place (``umask 002`` and ``chmod g+rw``: the lab shares the folder). A broken
link kills the script before the check, so a part is never renamed; the next
try overwrites it. A file is "uploaded" only after that check. The remote
commands run under ``sh -c`` so the user's login shell (bash, zsh, tcsh)
doesn't matter, and every path in them is ``shlex.quote``d.

One recording (``Session.push``): one call lists the remote copies (size, and
the sha256 when the size matches: hashing costs the cluster, not the link);
files already there with the same size and hash are skipped.

``.notes.jsonl`` is the one file that changes after the recording, on more
than one machine (``replay/notes.py``: append-only lines, merged as the union
by ``id``). It is synced by one ssh call (``cmd_notes``) both ways: the local
copy goes up on stdin; on the cluster, under ``flock`` on the date folder, the
remote copy gets the lines whose id it lacks (``awk``, keeping the remote lines
first), is replaced atomically, and is printed back; here ``notes.
append_missing`` appends the lines the local copy lacks, under notes.py's own
append lock, so a note the GUI appends meanwhile is never lost and the file is
never rewritten. Two laptops syncing the same notes at once are serialized by
the remote lock, so neither loses the other's lines. The order of the lines
differs between copies (each keeps its own first), so notes are compared by
their set of ids, never by hash: "uploaded" means every local id is in the copy
the cluster printed back. The sync runs when the local file changed since its
record, or the remote size differs from the one seen last (a note added on
another machine).

The state (``SyncState``, a JSON file in ``config.state_dir()``):

- ``queue``: recordings waiting to upload (stem -> folder, tries, last error).
  It survives restarts: a recording made offline, or whose upload failed
  halfway, stays until a push succeeds;
- ``uploaded``: per recording, per file, the size, mtime and sha256 that were
  verified on the cluster. A later local change (a note added in replay, a
  card written after Stop) no longer matches, so ``changed_since_upload``
  finds it and it is queued again. It also spares re-hashing: a file whose
  size and mtime match its record reuses the recorded sha.

GUI and CLI may run at once: every change re-reads the file under a lock file,
applies one edit and writes it back atomically.

Progress (``SyncEvent``): ``push_queue`` and ``Session.push`` call
``on_event(ev)`` from the thread they run in, for the recording as a whole
(``file == ""``: queued, waiting, checking, uploading, done, failed, offline,
removed, paused), for the run (``stem == ""``: ``run`` with ``n``, ``run_end``)
and per file (hashing and uploading with bytes done / total,
verifying, done, skipped, failed). Byte events come at most ``proc.EMIT_HZ``
times a second. The GUI turns them into a Qt signal (``remote/qt.py``), the CLI
into lines. ``stop(stem)`` is asked between files: ``"pause"`` ends the run
(the recording stays queued, what went up stays recorded), ``"skip"`` leaves
that recording (it was removed from the queue). Events change nothing in what
is uploaded or verified.

Pruning (off by default): local copies beyond the newest ``keep_n`` go only if
every file is verified on the cluster: the uploaded record matches the local
file and a remote listing confirms size and sha256 right before deleting. The
recording being written and the one open in replay are never touched (the
caller passes them in ``protect``), nor anything modified in the last
``BUSY_S``. ``prune_plan`` lists what would go; ``prune_execute`` deletes it.
"""

from __future__ import annotations

import contextlib
import hashlib
import json
import os
import shlex
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path, PurePosixPath
from typing import Callable, Iterable, Optional

from wavi_py.remote.config import SyncConfig, state_dir
from wavi_py.remote.proc import Result, Runner, Throttle, classify
from wavi_py.replay import notes as rnotes
from wavi_py.replay import recname

NOTES_SUFFIX = rnotes.SUFFIX
NOTES_TIMEOUT_S = 90.0                      # incl. up to 60 s waiting for the remote lock
MAIN_SUFFIXES = (".jsonl", ".dat")          # a stem with one of these is a recording
TEMP_SUFFIXES = (".part", ".tmp", ".swp", "~")
BUSY_S = 20.0                               # files modified since: still being written
TEST_TIMEOUT_S = 20.0
LIST_TIMEOUT_S = 60.0
MIN_UPLOAD_RATE = 100e3                     # B/s: the upload timeout is size / this + 60 s
HASH_RATE = 100e6                           # B/s on the cluster, for the listing's timeout


class SyncError(RuntimeError):
    def __init__(self, reason: str, offline: bool = False):
        super().__init__(reason)
        self.reason, self.offline = reason, offline


# ---------------------------------------------------------------------------
# local recordings
# ---------------------------------------------------------------------------

@dataclass
class Recording:
    dir: Path
    stem: str
    files: list[Path]              # what is uploaded
    caches: list[Path]             # rebuildable, deleted with it on prune

    @property
    def date(self) -> str:
        rn = recname.parse(self.stem)
        if rn is not None:
            return rn.date
        mt = min((p.stat().st_mtime for p in self.files), default=time.time())
        return datetime.fromtimestamp(mt).strftime("%Y-%m-%d")

    @property
    def mtime(self) -> float:
        """The newest of its files: ordering for keep-N, and the busy check."""
        return max((_mtime(p) for p in self.files), default=0.0)

    @property
    def size(self) -> int:
        return sum(_size(p) for p in self.files)


def _mtime(p: Path) -> float:
    try:
        return p.stat().st_mtime
    except OSError:
        return 0.0


def _size(p: Path) -> int:
    try:
        return p.stat().st_size
    except OSError:
        return 0


def _is_cache(name: str) -> bool:
    return name.lower().endswith(recname.CACHE_SUFFIXES)


def _is_temp(name: str) -> bool:
    return name.endswith(TEMP_SUFFIXES) or name.startswith(".")


def find_recordings(folder: Path) -> dict[str, Recording]:
    """Recordings in ``folder`` (not recursive): stems with a .jsonl or .dat."""
    groups: dict[str, Recording] = {}
    try:
        entries = sorted(p for p in Path(folder).iterdir() if p.is_file())
    except OSError:
        return {}
    for p in entries:
        if _is_temp(p.name):
            continue
        stem = recname.stem_of(p.name)
        r = groups.setdefault(stem, Recording(Path(folder), stem, [], []))
        (r.caches if _is_cache(p.name) else r.files).append(p)
    return {s: r for s, r in groups.items()
            if any(p.name == s + x for p in r.files for x in MAIN_SUFFIXES)}


def recording_of(path: os.PathLike | str) -> Optional[Recording]:
    """The recording a path (any of its files, or the stem) belongs to."""
    p = Path(path)
    stem = recname.stem_of(p.name)
    return find_recordings(p.parent).get(stem)


def file_sha256(path: Path, chunk: int = 1 << 20,
                progress: Optional[Callable[[int, int], None]] = None) -> str:
    """``progress(done, total)`` after every chunk (the caller throttles)."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        total = os.fstat(f.fileno()).st_size
        done = 0
        while True:
            b = f.read(chunk)
            if not b:
                return h.hexdigest()
            h.update(b)
            done += len(b)
            if progress:
                progress(done, total)


def _stat_key(p: Path) -> tuple[int, int]:
    st = p.stat()
    return st.st_size, st.st_mtime_ns


# ---------------------------------------------------------------------------
# the persistent state
# ---------------------------------------------------------------------------

class SyncState:
    """``sync_state.json``: the queue and the uploaded records (see the module
    docstring). Methods re-read, edit and rewrite under the lock."""

    LOCK_WAIT_S = 10.0
    LOCK_STALE_S = 60.0

    def __init__(self, path: Optional[Path] = None):
        self.path = Path(path) if path else state_dir() / "sync_state.json"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._tlock = threading.RLock()

    def _read(self) -> dict:
        try:
            d = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            d = {}
        if not isinstance(d, dict):
            d = {}
        d.setdefault("v", 1)
        d.setdefault("queue", {})
        d.setdefault("uploaded", {})
        return d

    def _write(self, d: dict) -> None:
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(json.dumps(d, indent=1, sort_keys=True), encoding="utf-8")
        os.replace(tmp, self.path)

    @contextlib.contextmanager
    def _locked(self):
        lock = self.path.with_name(self.path.name + ".lock")
        with self._tlock:
            t0 = time.monotonic()
            while True:
                try:
                    fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
                    os.close(fd)
                    break
                except FileExistsError:
                    try:
                        if time.time() - lock.stat().st_mtime > self.LOCK_STALE_S:
                            lock.unlink()          # a crashed holder
                            continue
                    except OSError:
                        continue
                    if time.monotonic() - t0 > self.LOCK_WAIT_S:
                        raise SyncError(f"state file locked: {lock}")
                    time.sleep(0.05)
            try:
                yield
            finally:
                try:
                    lock.unlink()
                except OSError:
                    pass

    def snapshot(self) -> dict:
        with self._locked():
            return self._read()

    def edit(self, fn: Callable[[dict], None]) -> None:
        with self._locked():
            d = self._read()
            fn(d)
            self._write(d)

    # ----- queue

    def enqueue(self, rec: Recording, why: str = "") -> None:
        def f(d):
            q = d["queue"].setdefault(rec.stem, {"dir": str(rec.dir), "added": time.time(),
                                                 "tries": 0, "error": ""})
            q["dir"] = str(rec.dir)
            if why:
                q["why"] = why
        self.edit(f)

    def enqueue_many(self, recs: Iterable[Recording], why: str = "") -> None:
        recs = list(recs)

        def f(d):
            for rec in recs:
                d["queue"].setdefault(rec.stem, {"dir": str(rec.dir), "added": time.time(),
                                                 "tries": 0, "error": "", "why": why})
        self.edit(f)

    def dequeue(self, stem: str) -> None:
        self.edit(lambda d: d["queue"].pop(stem, None))

    def note_failure(self, stem: str, reason: str) -> None:
        def f(d):
            q = d["queue"].get(stem)
            if q is not None:
                q["tries"] = int(q.get("tries", 0)) + 1
                q["error"] = reason
                q["last_try"] = time.time()
        self.edit(f)

    def queued(self) -> dict[str, dict]:
        return self.snapshot()["queue"]

    # ----- uploaded records

    def record(self, rec: Recording, rdir: str, entries: dict[str, dict]) -> None:
        def f(d):
            u = d["uploaded"].setdefault(rec.stem, {"files": {}})
            u["dir"], u["remote"] = str(rec.dir), rdir
            u["files"].update(entries)
            u["t"] = time.time()
        self.edit(f)

    def uploaded(self, stem: str) -> dict[str, dict]:
        return (self.snapshot()["uploaded"].get(stem) or {}).get("files", {})


def _matches(rec_entry: Optional[dict], p: Path) -> bool:
    if not rec_entry:
        return False
    try:
        size, mt = _stat_key(p)
    except OSError:
        return False
    return rec_entry.get("size") == size and rec_entry.get("mtime_ns") == mt


def changed_since_upload(rec: Recording, uploaded: dict[str, dict]) -> list[str]:
    """Files of ``rec`` that are new or changed since their verified upload."""
    return [p.name for p in rec.files if not _matches(uploaded.get(p.name), p)]


def is_busy(rec: Recording, now: Optional[float] = None) -> bool:
    return (now or time.time()) - rec.mtime < BUSY_S


# ---------------------------------------------------------------------------
# remote commands (strings for the cluster's sh; built here so tests can read them)
# ---------------------------------------------------------------------------

def _sh(script: str) -> str:
    """Run ``script`` under sh whatever the login shell; one line (a newline in
    an argument is fragile through Windows' command line)."""
    return "sh -c " + shlex.quote(script)


def cmd_test(root: str) -> str:
    q = shlex.quote(root)
    return _sh(f"umask 002; mkdir -p {q} || exit 3; test -w {q} || exit 4; echo WAVI_OK")


def cmd_list(rdir: str, sizes: dict[str, int]) -> str:
    """Per existing file: ``F <size> <sha or -> <name>``; the hash only when the
    size is the local one."""
    parts = [f"cd {shlex.quote(rdir)} 2>/dev/null || exit 0"]
    for name, size in sizes.items():
        n = shlex.quote(name)
        parts.append(
            f"if [ -f {n} ]; then s=$(wc -c < {n} | tr -d ' '); h=-; "
            f"if [ \"$s\" = {int(size)} ]; then h=$(sha256sum < {n}); h=${{h%% *}}; fi; "
            f"printf 'F %s %s %s\\n' \"$s\" \"$h\" {n}; fi")
    return _sh("; ".join(parts))


def cmd_upload(rdir: str, name: str, size: int, sha: str) -> str:
    d = shlex.quote(rdir)
    part = shlex.quote(str(PurePosixPath(rdir) / (name + ".part")))
    final = shlex.quote(str(PurePosixPath(rdir) / name))
    return _sh(
        f"umask 002; mkdir -p {d} || exit 3; cat > {part} || exit 4; "
        f"s=$(wc -c < {part} | tr -d ' '); "
        f"if [ \"$s\" != {int(size)} ]; then rm -f {part}; echo \"size $s, expected {int(size)}\" >&2; exit 5; fi; "
        f"h=$(sha256sum < {part}); h=${{h%% *}}; "
        f"if [ \"$h\" != {shlex.quote(sha)} ]; then rm -f {part}; echo 'sha256 mismatch' >&2; exit 6; fi; "
        f"chmod g+rw {part}; mv -f {part} {final} || exit 7; echo WAVI_OK")


# the union by id of the remote copy ($1) and the incoming one ($2): a line
# whose "id" was seen is dropped, a torn line (not ending in "}") skipped
_AWK_UNION = (r'{ if ($0 !~ /}[ \t\r]*$/) next; k = $0; '
              'if (match($0, /"id" *: *"[^"]*"/)) { k = substr($0, RSTART, RLENGTH); gsub(/[ ]/, "", k) } '
              'if (!(k in seen)) { seen[k] = 1; print } }')


def cmd_notes(rdir: str, name: str) -> str:
    """Merge stdin into ``rdir/name`` under ``flock`` on the folder (a lock on the
    file itself would not survive its replacement), print the result."""
    d, n = shlex.quote(rdir), shlex.quote(name)
    inc, tmp = ".wavi_notes.in.$$", ".wavi_notes.tmp.$$"     # $$: the remote shell's pid
    return _sh(
        f"umask 002; mkdir -p {d} || exit 3; cd {d} || exit 3; "
        f"cat > {inc} || {{ rm -f {inc}; exit 4; }}; "
        f"( flock -w 60 9 || exit 8; touch {n} && "
        f"awk {shlex.quote(_AWK_UNION)} {n} {inc} > {tmp} && chmod g+rw {tmp} && "
        f"mv -f {tmp} {n} && cat {n} ) 9< . ; rc=$?; rm -f {inc} {tmp}; exit $rc")


def cmd_cat(rpath: str) -> str:
    return _sh(f"cat {shlex.quote(rpath)} 2>/dev/null || true")


def parse_list(out: bytes) -> dict[str, tuple[int, Optional[str]]]:
    res: dict[str, tuple[int, Optional[str]]] = {}
    for line in out.decode("utf-8", "replace").splitlines():
        parts = line.split(" ", 3)
        if len(parts) == 4 and parts[0] == "F":
            try:
                res[parts[3]] = (int(parts[1]), None if parts[2] == "-" else parts[2])
            except ValueError:
                pass
    return res


# ---------------------------------------------------------------------------
# a session: the config + one runner
# ---------------------------------------------------------------------------

@dataclass
class PushResult:
    stem: str
    uploaded: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)
    notes_merged: bool = False
    bytes: int = 0
    secs: float = 0.0

    @property
    def rate(self) -> float:
        return self.bytes / self.secs if self.secs > 0 else 0.0


Progress = Callable[[int, int, str], None]      # (file i (1-based), of n, name)

# phases of SyncEvent, per recording (file == "") and per file
REC_PHASES = ("queued", "waiting", "checking", "uploading", "done", "failed", "offline",
              "removed", "paused")
FILE_PHASES = ("queued", "hashing", "uploading", "verifying", "done", "skipped", "failed")
FINISHED = ("done", "failed", "offline", "removed")


@dataclass
class SyncEvent:
    """One step of a push, for display only (see the module docstring).
    ``files``: on a recording's ``queued`` / ``checking``, its (name, size) list;
    ``k`` / ``n``: the recording's place in this run; ``rdir``: its cluster folder;
    ``bytes_sent``: on a recording's end, what this push actually sent;
    ``info``: a remark that is not an error ("merged")."""
    stem: str
    phase: str
    file: str = ""
    done: int = 0
    total: int = 0
    error: str = ""
    info: str = ""
    k: int = 0
    n: int = 0
    rdir: str = ""
    files: tuple = ()
    bytes_sent: int = 0
    t: float = field(default_factory=time.time)


class Stopped(Exception):
    """Raised between files when ``stop(stem)`` says so ("pause" / "skip")."""

    def __init__(self, why: str):
        super().__init__(why)
        self.why = why


def _nop(ev: SyncEvent) -> None:
    pass


class Session:
    def __init__(self, cfg: SyncConfig, state: Optional[SyncState] = None,
                 runner: Optional[Runner] = None):
        self.cfg = cfg
        self.state = state or SyncState()
        self.runner = runner or Runner()
        self.on_event: Callable[[SyncEvent], None] = _nop
        self.stop: Callable[[str], str] = lambda stem: ""
        self.k = self.n = 0                       # the current recording's place in the run

    def _ev(self, stem: str, phase: str, **kw) -> None:
        try:
            self.on_event(SyncEvent(stem, phase, k=self.k, n=self.n, **kw))
        except Exception:  # noqa: BLE001 — display only
            pass

    def cancel(self) -> None:
        self.runner.cancel()

    def _run(self, cmd: str, timeout: float, stdin=None, on_bytes=None) -> Result:
        if on_bytes is None:
            return self.runner.run(self.cfg.host.strip(), cmd, timeout, stdin)
        return self.runner.run(self.cfg.host.strip(), cmd, timeout, stdin, on_bytes=on_bytes)

    def _fail(self, res: Result, what: str) -> SyncError:
        why, offline = classify(res, self.cfg.host)
        return SyncError(f"{what}: {why}" if what else why, offline)

    # ----- connection test

    def test(self) -> tuple[bool, str, bool]:
        """(ok, reason, offline): ``mkdir -p <root> && test -w <root>``."""
        if not self.cfg.configured:
            return False, "no host / root set (Cluster...)", False
        res = self._run(cmd_test(self.cfg.root), TEST_TIMEOUT_S)
        if res.ok and b"WAVI_OK" in res.out:
            return True, f"ok: {self.cfg.host}:{self.cfg.root} is writable", False
        if res.rc == 3:
            return False, f"cannot create {self.cfg.root} on {self.cfg.host}", False
        if res.rc == 4:
            return False, f"{self.cfg.root} is not writable (group / permissions)", False
        why, offline = classify(res, self.cfg.host)
        return False, why, offline

    # ----- remote paths and listing

    def remote_dir(self, rec: Recording) -> str:
        return str(PurePosixPath(self.cfg.root) / rec.date)

    def remote_state(self, rdir: str, sizes: dict[str, int]) -> dict[str, tuple[int, Optional[str]]]:
        timeout = LIST_TIMEOUT_S + sum(sizes.values()) / HASH_RATE
        res = self._run(cmd_list(rdir, sizes), timeout)
        if not res.ok:
            raise self._fail(res, "listing")
        return parse_list(res.out)

    def fetch(self, rpath: str, timeout: float = LIST_TIMEOUT_S) -> bytes:
        res = self._run(cmd_cat(rpath), timeout)
        if not res.ok:
            raise self._fail(res, "fetch")
        return res.out

    def upload_file(self, path: Path, rdir: str, size: int, sha: str,
                    stem: str = "") -> None:
        timeout = 60.0 + size / MIN_UPLOAD_RATE
        stem = stem or recname.stem_of(path.name)

        sent_all = []

        def on_bytes(done, total):
            if sent_all:                          # the closing tick after the last chunk
                return
            self._ev(stem, "uploading", file=path.name, done=done, total=total or size)
            if done >= (total or size):           # the remote size + sha256 check runs now
                sent_all.append(True)
                self._ev(stem, "verifying", file=path.name, done=done, total=total or size)
        self._ev(stem, "uploading", file=path.name, done=0, total=size)
        with open(path, "rb") as f:
            res = self._run(cmd_upload(rdir, path.name, size, sha), timeout, stdin=f,
                            on_bytes=on_bytes)
        if not (res.ok and b"WAVI_OK" in res.out):
            if res.rc in (5, 6):
                raise SyncError(f"{path.name}: verification failed on the cluster"
                                f" ({res.err.strip() or 'size / sha256'}); retried later", False)
            raise self._fail(res, path.name)

    # ----- one recording

    def _local_sha(self, p: Path, uploaded: dict[str, dict], stem: str = "") -> str:
        e = uploaded.get(p.name)
        if _matches(e, p) and e.get("sha"):
            return e["sha"]
        stem = stem or recname.stem_of(p.name)
        tick = Throttle(lambda d, t: self._ev(stem, "hashing", file=p.name, done=d, total=t))
        tick(0, _size(p), final=True)
        return file_sha256(p, progress=tick)

    def sync_notes(self, rec: Recording, rdir: str,
                   remote: dict[str, tuple[int, Optional[str]]], res: "PushResult") -> None:
        """The notes both ways in one call (``cmd_notes``; see the module
        docstring), when the local copy changed since its record or the remote
        one's size differs from the one seen last."""
        name = rec.stem + NOTES_SUFFIX
        lp, r = rec.dir / name, remote.get(name)
        e = self.state.uploaded(rec.stem).get(name)
        local_changed = lp.exists() and not _matches(e, lp)
        remote_changed = r is not None and (e is None or e.get("rsize") != r[0])
        if not (local_changed or remote_changed):
            if lp.exists():
                res.skipped.append(name)
                self._ev(rec.stem, "skipped", file=name, total=_size(lp))
            return
        mine = lp.read_bytes() if lp.exists() else b""
        self._ev(rec.stem, "uploading", file=name, done=0, total=len(mine))
        out = self._run(cmd_notes(rdir, name), NOTES_TIMEOUT_S, stdin=mine)
        if not out.ok:
            if out.rc == 8:
                raise SyncError(f"{name}: the cluster's notes lock stayed busy; retried later")
            raise self._fail(out, name)
        theirs = rnotes.parse_lines(out.out)
        missing = {ev["id"] for ev in rnotes.parse_lines(mine)} - {ev["id"] for ev in theirs}
        if missing:
            raise SyncError(f"{name}: {len(missing)} note(s) missing on the cluster after the"
                            " merge; retried later")
        n, before, st = rnotes.append_missing(lp, theirs)
        self._ev(rec.stem, "done", file=name, done=len(mine), total=len(mine),
                 info=f"merged, {n} note(s) came down" if n else "merged")
        if lp not in rec.files:
            rec.files.append(lp)
        if n:
            res.notes_merged = True
        if local_changed:
            res.uploaded.append(name)
        if before == len(mine):
            # the file was what went up (plus the lines just appended); a note
            # appended meanwhile changes its size again, so it goes next time
            self.state.record(rec, rdir, {name: {
                "size": st.st_size, "mtime_ns": st.st_mtime_ns,
                "sha": hashlib.sha256(out.out).hexdigest(), "rsize": len(out.out)}})

    def push(self, rec: Recording, progress: Optional[Progress] = None) -> PushResult:
        """Upload what of ``rec`` the cluster doesn't have (verified), then sync
        the notes. Raises SyncError; whatever was verified before stays recorded."""
        res = PushResult(rec.stem)
        rdir = self.remote_dir(rec)
        uploaded = self.state.uploaded(rec.stem)
        notes_name = rec.stem + NOTES_SUFFIX
        files = [p for p in rec.files if p.name != notes_name]
        sizes = {p.name: _size(p) for p in files}
        sizes[notes_name] = 0                     # listed (its size), never hashed
        self._ev(rec.stem, "checking", rdir=rdir, files=_file_list(rec))
        remote = self.remote_state(rdir, sizes)
        sha = {}
        for p in files:
            self._check_stop(rec.stem)
            sha[p.name] = self._local_sha(p, uploaded, rec.stem)
        todo, done = [], {}
        for p in files:
            size, mt = _stat_key(p)
            r = remote.get(p.name)
            if r is not None and r[0] == size and r[1] == sha[p.name]:
                res.skipped.append(p.name)
                done[p.name] = {"size": size, "mtime_ns": mt, "sha": sha[p.name]}
                self._ev(rec.stem, "skipped", file=p.name, total=size)
            else:
                todo.append(p)
        if done:
            self.state.record(rec, rdir, done)
        if todo:
            self._ev(rec.stem, "uploading", rdir=rdir)
        t0 = time.monotonic()
        for i, p in enumerate(todo, 1):
            self._check_stop(rec.stem)
            if progress:
                progress(i, len(todo), p.name)
            size, mt = _stat_key(p)
            try:
                self.upload_file(p, rdir, size, sha[p.name], rec.stem)
            except SyncError as exc:
                self._ev(rec.stem, "failed", file=p.name, total=size, error=exc.reason)
                raise
            self._ev(rec.stem, "done", file=p.name, done=size, total=size)
            # recorded at once: a later failure doesn't send this file again
            self.state.record(rec, rdir, {p.name: {"size": size, "mtime_ns": mt,
                                                   "sha": sha[p.name]}})
            res.uploaded.append(p.name)
            res.bytes += size
        res.secs = time.monotonic() - t0
        self._check_stop(rec.stem)
        self.sync_notes(rec, rdir, remote, res)
        return res

    def _check_stop(self, stem: str) -> None:
        why = self.stop(stem)
        if why:
            raise Stopped(why)

    def verified_on_cluster(self, rec: Recording) -> tuple[bool, str]:
        """Every file of ``rec``: the record matches it, and the cluster has the
        same size and sha256 now (one listing call); the notes: every local id is
        in the cluster's copy (one more call)."""
        up = self.state.uploaded(rec.stem)
        missing = changed_since_upload(rec, up)
        if missing:
            return False, f"not uploaded: {', '.join(missing)}"
        notes_name = rec.stem + NOTES_SUFFIX
        files = [p for p in rec.files if p.name != notes_name]
        sizes = {p.name: _size(p) for p in files}
        rdir = self.remote_dir(rec)
        remote = self.remote_state(rdir, sizes)
        for p in files:
            r = remote.get(p.name)
            if r is None or r[0] != sizes[p.name] or r[1] != up[p.name].get("sha"):
                return False, f"{p.name} differs on the cluster"
        lp = rec.dir / notes_name
        if lp.exists():
            theirs = {ev["id"] for ev in rnotes.parse_lines(
                self.fetch(str(PurePosixPath(rdir) / notes_name)))}
            if {ev["id"] for ev in rnotes.read(lp)} - theirs:
                return False, f"{notes_name}: notes missing on the cluster"
        return True, ""


# ---------------------------------------------------------------------------
# queue processing and pruning (shared by the GUI worker and the CLI)
# ---------------------------------------------------------------------------

def _file_list(rec: Recording) -> tuple:
    return tuple((p.name, _size(p)) for p in rec.files)


def queue_events(state: SyncState, protect: Iterable[str] = ()) -> list[SyncEvent]:
    """A ``queued`` (or ``waiting``: in use) event per queued recording, oldest
    first, with its files and the last error: what a window shows of the queue
    without a push running (after a restart, offline)."""
    protect = {recname.stem_of(Path(p).name) for p in protect if p}
    snap = state.snapshot()["queue"]
    out = []
    for i, (stem, q) in enumerate(sorted(snap.items(), key=lambda kv: kv[1].get("added", 0)), 1):
        rec = find_recordings(Path(q.get("dir", "."))).get(stem)
        files = _file_list(rec) if rec is not None else ()
        out.append(SyncEvent(stem, "waiting" if stem in protect else "queued",
                             k=i, n=len(snap), files=files, error=q.get("error", ""),
                             t=q.get("last_try") or q.get("added") or time.time()))
    return out


def queue_recordings(state: SyncState) -> list[tuple[str, Optional[Recording]]]:
    out = []
    for stem, q in sorted(state.queued().items(), key=lambda kv: kv[1].get("added", 0)):
        out.append((stem, find_recordings(Path(q.get("dir", "."))).get(stem)))
    return out


def requeue_changed(state: SyncState, folder: Path,
                    only_uploaded: bool = True) -> list[str]:
    """Queue the recordings in ``folder`` with files new or changed since their
    upload (``only_uploaded``: only those uploaded before, e.g. a note added in
    replay; else every recording not fully uploaded). Busy ones wait."""
    snap = state.snapshot()
    added = []
    for stem, rec in find_recordings(folder).items():
        if stem in snap["queue"] or is_busy(rec):
            continue
        up = (snap["uploaded"].get(stem) or {}).get("files")
        if only_uploaded and not up:
            continue
        if changed_since_upload(rec, up or {}):
            state.enqueue(rec, "changed" if up else "new")
            added.append(stem)
    return added


def not_uploaded(state: SyncState, folder: Path, protect: Iterable[str] = ()) -> list[Recording]:
    """Recordings in ``folder`` with files not verified on the cluster per the
    state (no network), not queued, not in use: "Upload existing..."."""
    protect = {recname.stem_of(p) for p in protect if p}
    snap = state.snapshot()
    out = []
    for stem, rec in sorted(find_recordings(folder).items()):
        if stem in snap["queue"] or stem in protect or is_busy(rec):
            continue
        if changed_since_upload(rec, (snap["uploaded"].get(stem) or {}).get("files") or {}):
            out.append(rec)
    return out


def push_queue(sess: Session, protect: Iterable[str] = (), check_busy: bool = True,
               progress: Optional[Callable[[str, int, int, str], None]] = None,
               log: Callable[[str], None] = lambda s: None,
               on_event: Optional[Callable[[SyncEvent], None]] = None,
               stop: Optional[Callable[[str], str]] = None
               ) -> tuple[list[PushResult], list[str]]:
    """Every queued recording, oldest first. A network failure stops the run
    (the rest stays queued); another failure is recorded on the item and the
    run goes on. ``check_busy``: skip recordings modified in the last ``BUSY_S``
    (the CLI, which can't know what the GUI is writing; the GUI passes what it
    writes in ``protect`` instead). ``on_event`` / ``stop``: see the module
    docstring. Returns (results, errors)."""
    protect = {recname.stem_of(Path(p).name) for p in protect if p}
    results, errors = [], []
    if on_event is not None:
        sess.on_event = on_event
    if stop is not None:
        sess.stop = stop
    items = queue_recordings(sess.state)
    sess.n = len(items)
    sess._ev("", "run")
    for k, (stem, rec) in enumerate(items, 1):
        sess.k = k
        if rec is not None:
            sess._ev(stem, "queued", files=_file_list(rec), rdir=sess.remote_dir(rec))
    try:
        for k, (stem, rec) in enumerate(items, 1):
            sess.k = k
            why = sess.stop(stem)
            if why == "pause":
                sess._ev(stem, "paused")
                break
            if why == "skip":
                sess._ev(stem, "removed")
                continue
            if rec is None:
                log(f"{stem}: no longer on disk, dropped from the queue")
                sess.state.dequeue(stem)
                sess._ev(stem, "removed", error="no longer on disk")
                continue
            if stem in protect or (check_busy and is_busy(rec)):
                log(f"{stem}: still in use, later")
                sess._ev(stem, "waiting", error="in use (recording or replay)")
                continue
            try:
                r = sess.push(rec, (lambda i, n, name: progress(stem, i, n, name))
                              if progress else None)
            except Stopped as exc:
                if exc.why == "pause":
                    sess._ev(stem, "paused")
                    break
                sess._ev(stem, "removed")
                continue
            except SyncError as exc:
                sess.state.note_failure(stem, exc.reason)
                errors.append(f"{stem}: {exc.reason}")
                sess._ev(stem, "offline" if exc.offline else "failed", error=exc.reason)
                if exc.offline:
                    raise
                continue
            except OSError as exc:                     # the local file went away mid-way
                sess.state.note_failure(stem, str(exc))
                errors.append(f"{stem}: {exc}")
                sess._ev(stem, "failed", error=str(exc))
                continue
            sess.state.dequeue(stem)
            results.append(r)
            sess._ev(stem, "done", bytes_sent=r.bytes, rdir=sess.remote_dir(rec),
                     info="notes merged" if r.notes_merged else "")
            log(f"{stem}: {len(r.uploaded)} uploaded, {len(r.skipped)} already there"
                + (", notes merged" if r.notes_merged else "")
                + (f", {r.bytes / 1e6:.1f} MB at {r.rate / 1e6:.2f} MB/s" if r.bytes else ""))
    finally:
        sess.k = sess.n = 0
        sess._ev("", "run_end")
    return results, errors


@dataclass
class PrunePlan:
    delete: list[Recording]                 # verified on the cluster: these go
    kept_unverified: list[tuple[str, str]]  # beyond keep-N but not verified: (stem, why)

    def paths(self) -> list[Path]:
        return [p for r in self.delete for p in (*r.files, *r.caches)]


def prune_plan(sess: Session, folder: Path, keep_n: int,
               protect: Iterable[str] = ()) -> PrunePlan:
    protect = {recname.stem_of(Path(p).name) for p in protect if p}
    recs = sorted(find_recordings(folder).values(), key=lambda r: r.mtime, reverse=True)
    plan = PrunePlan([], [])
    for rec in recs[max(0, keep_n):]:
        if rec.stem in protect:
            plan.kept_unverified.append((rec.stem, "in use (recording or replay)"))
            continue
        if is_busy(rec):
            plan.kept_unverified.append((rec.stem, "modified in the last seconds"))
            continue
        ok, why = sess.verified_on_cluster(rec)
        (plan.delete.append(rec) if ok else plan.kept_unverified.append((rec.stem, why)))
    return plan


def prune_execute(sess: Session, plan: PrunePlan) -> list[Path]:
    """Delete the plan's files; a recording changed since its plan was made is
    skipped (its record no longer matches)."""
    gone = []
    for rec in plan.delete:
        if changed_since_upload(rec, sess.state.uploaded(rec.stem)):
            continue
        for p in (*rec.files, *rec.caches):
            try:
                p.unlink()
                gone.append(p)
            except FileNotFoundError:
                pass
    return gone
