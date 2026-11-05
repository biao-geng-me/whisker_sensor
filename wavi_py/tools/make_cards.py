"""Write the missing recording cards (``replay/card.py``) under folders.

    python -m wavi_py.tools.make_cards FOLDER_OR_FILE... [--force] [--dry-run] [--quiet] [--nice]

Every recording under each folder (recursive): each ``.jsonl`` (not a
``.notes.jsonl``), and each ``.dat`` without a ``.jsonl`` of the same stem.
A recording gets a card when it has none, or its card is older than it or of
another version; ``--force`` rebuilds them all. One progress line per
recording (to stderr): its number, the stem and the time it took, or why it
was skipped. Exit status 1 if any failed.

The GUI runs it on the one file it just closed (``--nice --quiet``, a child
process, so the build never holds the GIL of the window that may be recording
the next one). On the cluster, through onnode::

    ~/bin/onnode -- python -m wavi_py.tools.make_cards /shared/rc/whiskers/rov-recordings

The index cache (``.idx.npz``) is kept next to each recording as replay keeps
it (or in the user cache when the folder is read-only); the whole-recording
store is not saved unless ``--cache-store``.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


def find_recordings(root: Path) -> list[Path]:
    """The recordings under ``root`` (or ``root`` itself, a file), sorted."""
    if root.is_file():
        return [root]
    out = []
    for p in sorted(root.rglob("*")):
        name = p.name.lower()
        if not p.is_file():
            continue
        if name.endswith(".jsonl") and not name.endswith(".notes.jsonl"):
            out.append(p)
        elif name.endswith(".dat") and not p.with_suffix(".jsonl").exists():
            out.append(p)
    return out


def _lower_priority() -> None:
    try:
        if hasattr(os, "nice"):
            os.nice(10)
        elif sys.platform.startswith("win"):
            import ctypes
            k = ctypes.windll.kernel32
            k.SetPriorityClass(k.GetCurrentProcess(), 0x00004000)   # BELOW_NORMAL
    except Exception:  # noqa: BLE001 — only a courtesy
        pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="+", help="folders (searched recursively) or recordings")
    ap.add_argument("--force", action="store_true", help="rebuild cards that are current")
    ap.add_argument("--dry-run", action="store_true", help="list what would be built")
    ap.add_argument("--quiet", action="store_true", help="only errors")
    ap.add_argument("--nice", action="store_true", help="run at a lower CPU priority")
    ap.add_argument("--cache-store", action="store_true",
                    help="also save the whole-recording cache (.store.npz)")
    a = ap.parse_args(argv)
    if a.nice:
        _lower_priority()
    from wavi_py.replay import card as cardmod

    recs: list[Path] = []
    for p in a.paths:
        path = Path(p)
        if not path.exists():
            print(f"make_cards: no such file or folder: {p}", file=sys.stderr)
            return 2
        recs += find_recordings(path)
    todo = [r for r in recs if a.force or not cardmod.is_current(r)]
    if not a.quiet:
        print(f"make_cards: {len(recs)} recordings, {len(todo)} cards to write", file=sys.stderr)
    failed = 0
    t_all = time.perf_counter()
    for i, rec in enumerate(todo, 1):
        head = f"[{i}/{len(todo)}] {rec.name}"
        if a.dry_run:
            print(head, file=sys.stderr)
            continue
        t0 = time.perf_counter()
        try:
            out, card, tm = cardmod.write_card_for(rec, cache_store=a.cache_store)
        except Exception as exc:  # noqa: BLE001 — not a recording, unreadable: go on
            failed += 1
            print(f"{head}: FAILED {type(exc).__name__}: {exc}", file=sys.stderr)
            continue
        if not a.quiet:
            print(f"{head}: {card['duration_s']:.0f} s, {card['nch']} ch, card"
                  f" {out.stat().st_size / 1e3:.1f} kB in {time.perf_counter() - t0:.2f} s"
                  f" (index {tm['index']:.2f}, store {tm['store']:.2f},"
                  f" card {tm['card']:.3f})", file=sys.stderr)
    if not a.quiet and todo and not a.dry_run:
        print(f"make_cards: done in {time.perf_counter() - t_all:.1f} s"
              + (f", {failed} failed" if failed else ""), file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
