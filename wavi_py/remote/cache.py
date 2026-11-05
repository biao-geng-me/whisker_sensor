"""The download cache: cluster recordings opened on this machine.

``cache_dir()`` is a folder in the user data dir (``config.state_dir()/
cluster_cache``), flat like the output dir. A download (``download``) takes
every file of the recording in two ssh calls: the cluster's sha256 of each
(the listing call), then one ``cat`` of them all, split here by those sizes
into ``<name>.part`` files, hashed on the way, checked (size and sha256) and
only then renamed (all or none); mtimes are
set to the cluster's. Each file is then recorded in the sync state as uploaded
(it came from there), so sync never sends it back, while a note added in replay
changes ``.notes.jsonl`` and the normal sync (which watches this folder too:
``SyncController``) merges it up.

Size cap (default 20 GB): ``evict`` removes recordings least recently opened
first (``cache_index.json``: stem -> last opened), never a protected one (the
open replay) and never one with local changes not yet on the cluster (a note
not synced yet): those wait for the sync.
"""

from __future__ import annotations

import hashlib
import json
import os
import time
from pathlib import Path, PurePosixPath
from typing import Callable, Iterable, Optional

from wavi_py.remote.config import state_dir
from wavi_py.remote.sync import (Session, SyncError, SyncState, changed_since_upload,
                                 find_recordings, _stat_key)
from wavi_py.replay import notes as rnotes
from wavi_py.replay import recname

DEFAULT_CAP = 20e9
MIN_DOWNLOAD_RATE = 100e3       # B/s: a file's timeout is size / this + 60 s


def cache_dir() -> Path:
    p = state_dir() / "cluster_cache"
    p.mkdir(parents=True, exist_ok=True)
    return p


class Cache:
    def __init__(self, folder: Optional[Path] = None, cap: float = DEFAULT_CAP):
        self.dir = Path(folder) if folder else cache_dir()
        self.dir.mkdir(parents=True, exist_ok=True)
        self.cap = cap
        self._index = self.dir / "cache_index.json"

    def _read(self) -> dict:
        try:
            d = json.loads(self._index.read_text(encoding="utf-8"))
            return d if isinstance(d, dict) else {}
        except (OSError, ValueError):
            return {}

    def touch(self, stem: str) -> None:
        d = self._read()
        d[stem] = time.time()
        tmp = self._index.with_name(self._index.name + f".{os.getpid()}.tmp")
        tmp.write_text(json.dumps(d), encoding="utf-8")
        os.replace(tmp, self._index)

    def size(self) -> int:
        return sum(p.stat().st_size for p in self.dir.iterdir()
                   if p.is_file() and p != self._index)

    def evict(self, state: SyncState, protect: Iterable[str] = (),
              cap: Optional[float] = None) -> list[str]:
        """Remove the least recently opened recordings until the cache is under
        ``cap``; returns the stems removed."""
        cap = self.cap if cap is None else cap
        protect = {recname.stem_of(p) for p in protect if p}
        opened = self._read()
        recs = find_recordings(self.dir)
        total = self.size()
        gone = []
        for stem in sorted(recs, key=lambda s: opened.get(s, 0.0)):
            if total <= cap:
                break
            rec = recs[stem]
            if stem in protect or changed_since_upload(rec, state.uploaded(stem)):
                continue
            for p in (*rec.files, *rec.caches):
                try:
                    total -= p.stat().st_size
                    p.unlink()
                except OSError:
                    pass
            # the replay's index / store caches of it in the user cache stay: small, keyed
            opened.pop(stem, None)
            gone.append(stem)
        if gone:
            tmp = self._index.with_name(self._index.name + f".{os.getpid()}.tmp")
            tmp.write_text(json.dumps(opened), encoding="utf-8")
            os.replace(tmp, self._index)
        return gone


Progress = Callable[[int, int, str], None]       # (bytes done, bytes total, file name)


def download(sess: Session, cache: Cache, date: str, files: dict[str, list],
             progress: Optional[Progress] = None) -> Path:
    """Every file of one cluster recording (``files``: name -> [size, mtime]
    from the catalogue) into the cache, verified; returns the file to replay.
    Raises SyncError (cancel: ``sess.cancel()``, reason "cancelled")."""
    rdir = str(PurePosixPath(sess.cfg.root) / date)
    sizes = {n: int(v[0]) for n, v in files.items()}
    remote = sess.remote_state(rdir, sizes)
    for n, size in sizes.items():
        r = remote.get(n)
        if r is None or r[0] != size or r[1] is None:
            raise SyncError(f"{n} changed on the cluster since the listing: Refresh, then"
                            " open it again", False)
    total = sum(sizes.values())
    stem = recname.stem_of(next(iter(files)))
    order = sorted(files, key=lambda n: sizes[n])            # small ones first
    # one ``cat`` of every file, split here by the sizes just checked (an ssh
    # call costs ~1.5 s of connection to a login node: one per file would
    # dominate a small recording)
    parts = {n: cache.dir / (n + ".part") for n in order}
    hashes = {n: hashlib.sha256() for n in order}
    got_n = {n: 0 for n in order}
    handles = {n: open(parts[n], "wb") for n in order}
    pos = [0, 0]                                             # file index, bytes so far

    def sink(b: bytes) -> None:
        view = memoryview(b)
        while view and pos[0] < len(order):
            n = order[pos[0]]
            take = min(len(view), sizes[n] - got_n[n])
            if take:
                handles[n].write(view[:take])
                hashes[n].update(view[:take])
                got_n[n] += take
                view = view[take:]
            if got_n[n] >= sizes[n]:
                pos[0] += 1
        pos[1] += len(b)
        if progress:
            progress(min(pos[1], total), total, order[min(pos[0], len(order) - 1)])

    cmd = "cat " + " ".join(_q(str(PurePosixPath(rdir) / n)) for n in order)
    try:
        res = sess.runner.stream(sess.cfg.host.strip(), cmd, sink,
                                 60.0 + total / MIN_DOWNLOAD_RATE)
    finally:
        for f in handles.values():
            f.close()
    bad = [n for n in order if got_n[n] != sizes[n] or hashes[n].hexdigest() != remote[n][1]]
    if not res.ok or bad or pos[1] != total:
        for pth in parts.values():
            pth.unlink(missing_ok=True)
        if not res.ok:
            raise sess._fail(res, f"downloading {stem}")
        raise SyncError(f"{', '.join(bad) or stem}: the download didn't verify (size / sha256)",
                        False)
    got: dict[str, dict] = {}
    for name in order:
        dst, part = cache.dir / name, parts[name]
        if name.endswith(rnotes.SUFFIX) and dst.exists():
            # a cached copy with notes of its own: keep them (append the cluster's);
            # not recorded, so its local notes go up with the sync
            rnotes.append_missing(dst, rnotes.read(part))
            part.unlink()
            continue
        os.replace(part, dst)
        mt = float(files[name][1])
        os.utime(dst, (mt, mt))
        size, mt_ns = _stat_key(dst)
        got[name] = {"size": size, "mtime_ns": mt_ns, "sha": remote[name][1]}
        if name.endswith(rnotes.SUFFIX):
            got[name]["rsize"] = sizes[name]
    rec = find_recordings(cache.dir).get(stem)
    if rec is not None:
        sess.state.record(rec, rdir, got)
    cache.touch(stem)
    for suf in (".jsonl", ".dat"):
        if (cache.dir / (stem + suf)).is_file():
            return cache.dir / (stem + suf)
    raise SyncError(f"{stem}: no .jsonl or .dat in the download", False)


def _q(s: str) -> str:
    import shlex
    return shlex.quote(s)
