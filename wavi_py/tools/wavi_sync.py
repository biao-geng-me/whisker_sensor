"""Cluster sync from the command line, for when the GUI isn't running.

Same settings (QSettings, set in the GUI's Cluster... dialog), same queue and
uploaded state, same code (``wavi_py/remote/sync.py``) as the GUI::

    python -m wavi_py.tools.wavi_sync test
    python -m wavi_py.tools.wavi_sync push [PATH ...] [--verify]
    python -m wavi_py.tools.wavi_sync status
    python -m wavi_py.tools.wavi_sync prune [--yes]

Flags override the settings for this run only: ``--host``, ``--root``,
``--dir`` (the local recordings folder; default the GUI's output dir),
``--keep N``. ``--save`` stores the overrides.

- ``test``: ``mkdir -p <root> && test -w <root>`` over ssh; exit 0 if ok;
- ``push``: the given recordings (any file of each, or a folder), else the
  queue plus every recording in the local folder not yet verified on the
  cluster. Recordings modified in the last 20 s are left for later (the GUI may
  be writing one). A line per file (hashing a big one, uploading, done,
  already there, failed) and a percentage line at most every 3 s.
  ``--verify`` also re-checks recordings the state says are uploaded, against
  the cluster's size and sha256;
- ``status``: the settings, the queue, and per local recording: uploaded,
  changed since, or not yet;
- ``prune``: list the local recordings beyond the newest N that are verified on
  the cluster (and those that are not, with why); ``--yes`` deletes the listed
  ones. Without ``--yes`` nothing is deleted.

Exit code 0 ok, 1 failed, 2 offline (everything stays queued).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from wavi_py.remote import config as rconfig  # noqa: E402
from wavi_py.remote.progress import LinePrinter  # noqa: E402
from wavi_py.remote.sync import (Session, SyncError, SyncState, changed_since_upload,  # noqa: E402
                                 find_recordings, is_busy, prune_execute, prune_plan,
                                 push_queue, recording_of, requeue_changed)


def _cfg(a) -> rconfig.SyncConfig:
    c = rconfig.load()
    if a.host:
        c.host = a.host
    if a.root:
        c.root = a.root.rstrip("/")
    if a.dir:
        c.local_dir = str(Path(a.dir).resolve())
    if a.keep:
        c.keep_n = a.keep
    if not c.local_dir:
        c.local_dir = str(rconfig.default_local_dir())
    if a.save:
        rconfig.save(c)
    return c


def cmd_test(sess: Session, a) -> int:
    ok, why, offline = sess.test()
    print(why if ok else f"FAILED: {why}")
    return 0 if ok else (2 if offline else 1)


def _targets(paths: list[str]):
    for p in paths:
        q = Path(p)
        if q.is_dir():
            yield from find_recordings(q).values()
        else:
            rec = recording_of(q)
            if rec is None:
                print(f"{p}: not a recording (no .jsonl / .dat of its stem)")
            else:
                yield rec


def cmd_push(sess: Session, a) -> int:
    ok, why, offline = sess.test()
    if not ok:
        print(f"FAILED: {why}")
        return 2 if offline else 1
    st = sess.state
    if a.paths:
        for rec in _targets(a.paths):
            st.enqueue(rec, "cli")
    else:
        added = requeue_changed(st, Path(sess.cfg.local_dir), only_uploaded=False)
        if added:
            print(f"queued {len(added)} from {sess.cfg.local_dir}")
    if a.verify:
        for rec in find_recordings(Path(sess.cfg.local_dir)).values():
            if not is_busy(rec):
                st.enqueue(rec, "verify")
    n = len(st.queued())
    print(f"{n} recording(s) to push to {sess.cfg.host}:{sess.cfg.root}")

    lines = LinePrinter(lambda s: print(s, flush=True))
    t0 = time.monotonic()
    try:
        results, errors = push_queue(sess, log=print, on_event=lines)
    except SyncError as exc:
        print(f"OFFLINE: {exc.reason}; the rest stays queued")
        return 2
    total = sum(r.bytes for r in results)
    secs = time.monotonic() - t0
    print(f"done: {len(results)} pushed, {sum(len(r.uploaded) for r in results)} files,"
          f" {total / 1e6:.1f} MB in {secs:.1f} s"
          + (f" ({total / 1e6 / secs:.2f} MB/s overall)" if total and secs > 0 else ""))
    for e in errors:
        print(f"ERROR {e}")
    return 1 if errors else 0


def cmd_status(sess: Session, a) -> int:
    c = sess.cfg
    print(f"host {c.host or '(unset)'}  root {c.root}  local {c.local_dir}")
    print(f"auto-upload {'on' if c.auto_upload else 'off'}, prune "
          f"{'on, keep ' + str(c.keep_n) if c.prune else 'off'}; state {sess.state.path}")
    snap = sess.state.snapshot()
    q = snap["queue"]
    print(f"queue: {len(q)}")
    for stem, e in sorted(q.items(), key=lambda kv: kv[1].get("added", 0)):
        print(f"  {stem}  tries {e.get('tries', 0)}" + (f"  last error: {e['error']}"
                                                        if e.get("error") else ""))
    recs = sorted(find_recordings(Path(c.local_dir)).values(), key=lambda r: r.mtime)
    print(f"local recordings: {len(recs)}")
    for r in recs:
        up = (snap["uploaded"].get(r.stem) or {}).get("files") or {}
        ch = changed_since_upload(r, up)
        s = ("uploaded" if not ch else "not uploaded" if not up else
             f"changed since upload: {', '.join(ch)}")
        print(f"  {r.stem}  {r.size / 1e6:.1f} MB  {s}")
    return 0


def cmd_prune(sess: Session, a) -> int:
    c = sess.cfg
    try:
        plan = prune_plan(sess, Path(c.local_dir), c.keep_n)
    except SyncError as exc:
        print(f"FAILED: {exc.reason}")
        return 2 if exc.offline else 1
    for stem, why in plan.kept_unverified:
        print(f"keep {stem}: {why}")
    if not plan.delete:
        print(f"nothing to prune (keep newest {c.keep_n})")
        return 0
    paths = plan.paths()
    mb = sum(p.stat().st_size for p in paths) / 1e6
    print(f"verified on the cluster, beyond the newest {c.keep_n}: {len(plan.delete)}"
          f" recording(s), {len(paths)} files, {mb:.1f} MB")
    for p in paths:
        print(f"  {p}")
    if not a.yes:
        print("dry run: add --yes to delete these")
        return 0
    gone = prune_execute(sess, plan)
    print(f"deleted {len(gone)} files")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="wavi_sync", description=__doc__.split("\n\n")[0])
    ap.add_argument("--host")
    ap.add_argument("--root")
    ap.add_argument("--dir", help="the local recordings folder")
    ap.add_argument("--keep", type=int)
    ap.add_argument("--save", action="store_true", help="store the overrides in the settings")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("test")
    p = sub.add_parser("push")
    p.add_argument("paths", nargs="*")
    p.add_argument("--verify", action="store_true")
    sub.add_parser("status")
    p = sub.add_parser("prune")
    p.add_argument("--yes", action="store_true")
    a = ap.parse_args(argv)
    sess = Session(_cfg(a), SyncState())
    if a.cmd != "status" and not sess.cfg.configured:
        print("no host set: --host HOST (and --save), or the GUI's Cluster... dialog")
        return 1
    return {"test": cmd_test, "push": cmd_push, "status": cmd_status,
            "prune": cmd_prune}[a.cmd](sess, a)


if __name__ == "__main__":
    sys.exit(main())
