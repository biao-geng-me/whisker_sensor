"""Write the missing manoeuvre sidecars (``replay/manoeuvres.py``) under folders.

    python -m wavi_py.tools.make_manoeuvres FOLDER_OR_FILE... [--out DIR] [--force]
        [--jobs N] [--no-csv] [--dry-run] [--quiet] [--nice] [--cache | --no-cache]

Every ``.jsonl`` recording under each folder (recursive; not a ``.notes.jsonl``;
a ``.dat`` has no vehicle, so none). A recording gets a sidecar when it has none,
or its sidecar is older than it or of another version; ``--force`` rebuilds them
all. One progress line per recording (to stderr). Exit status 1 if any failed.

``--out DIR`` writes the sidecars into DIR instead of next to the recordings:
for a folder that is not ours (the cluster's ``rov-recordings`` is shared lab
data). With ``--out`` the index / store caches are not written next to the
recordings either (``--cache`` writes them anyway). Unless ``--no-csv``, each
output folder also gets ``events.csv``: every sidecar there flattened into one
row per event x position (``manoeuvres.CSV_COLUMNS``).

On the cluster, through onnode::

    ~/bin/onnode -- python -m wavi_py.tools.make_manoeuvres \\
        /shared/rc/whiskers/rov-recordings/2026-11-05 --out /scratch/.../manoeuvres --jobs 6
"""

from __future__ import annotations

import argparse
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from wavi_py.tools.make_cards import _lower_priority


def find_recordings(root: Path) -> list[Path]:
    """The ``.jsonl`` recordings under ``root`` (or ``root`` itself), sorted."""
    if root.is_file():
        return [root]
    return [p for p in sorted(root.rglob("*.jsonl"))
            if p.is_file() and not p.name.lower().endswith(".notes.jsonl")]


def _one(rec: str, out: str, cache: bool):
    from wavi_py.replay import manoeuvres as mn
    t0 = time.perf_counter()
    path, doc, tm = mn.write_manoeuvres_for(rec, out or None, cache=cache)
    return str(path), len(doc["events"]), doc["recording"].get("armed_s"), tm, \
        time.perf_counter() - t0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="+", help="folders (searched recursively) or recordings")
    ap.add_argument("--out", default="", help="write the sidecars (and events.csv) here")
    ap.add_argument("--force", action="store_true", help="rebuild sidecars that are current")
    ap.add_argument("--jobs", type=int, default=1, help="recordings at once (processes)")
    ap.add_argument("--no-csv", action="store_true", help="no events.csv")
    ap.add_argument("--dry-run", action="store_true", help="list what would be built")
    ap.add_argument("--quiet", action="store_true", help="only errors")
    ap.add_argument("--nice", action="store_true", help="run at a lower CPU priority")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--cache", dest="cache", action="store_true", default=None,
                   help="write the index / store caches next to the recordings")
    g.add_argument("--no-cache", dest="cache", action="store_false")
    a = ap.parse_args(argv)
    if a.nice:
        _lower_priority()
    from wavi_py.replay import manoeuvres as mn

    cache = a.cache if a.cache is not None else not a.out
    recs: list[Path] = []
    for p in a.paths:
        path = Path(p)
        if not path.exists():
            print(f"make_manoeuvres: no such file or folder: {p}", file=sys.stderr)
            return 2
        recs += find_recordings(path)
    out = a.out or None
    todo = [r for r in recs if a.force or not mn.is_current(r, out)]
    if not a.quiet:
        print(f"make_manoeuvres: {len(recs)} recordings, {len(todo)} to write", file=sys.stderr)
    failed = 0
    t_all = time.perf_counter()
    if a.dry_run:
        for i, rec in enumerate(todo, 1):
            print(f"[{i}/{len(todo)}] {rec.name}", file=sys.stderr)
        return 0
    with ProcessPoolExecutor(max_workers=max(1, a.jobs)) as ex:
        futs = [(rec, ex.submit(_one, str(rec), a.out, cache)) for rec in todo]
        for i, (rec, fu) in enumerate(futs, 1):
            head = f"[{i}/{len(todo)}] {rec.name}"
            try:
                path, n, armed, tm, dt = fu.result()
            except Exception as exc:  # noqa: BLE001 — not a recording, no vehicle: go on
                failed += 1
                print(f"{head}: FAILED {type(exc).__name__}: {exc}", file=sys.stderr)
                continue
            if not a.quiet:
                print(f"{head}: armed {armed or 0:.0f} s, {n} events, {dt:.1f} s"
                      f" (load {tm['load']:.1f}, extract {tm['extract']:.1f})", file=sys.stderr)
    if not a.no_csv:
        dirs = {Path(out)} if out else {r.parent for r in recs}
        for d in sorted(dirs):
            docs = [doc for p in sorted(d.glob("*" + mn.SUFFIX))
                    if (doc := mn.read_doc(p)) is not None]
            if docs:
                c = mn.write_events_csv(docs, d / "events.csv")
                if not a.quiet:
                    print(f"make_manoeuvres: {c} ({len(docs)} recordings)", file=sys.stderr)
    if not a.quiet and todo:
        print(f"make_manoeuvres: done in {time.perf_counter() - t_all:.1f} s"
              + (f", {failed} failed" if failed else ""), file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
