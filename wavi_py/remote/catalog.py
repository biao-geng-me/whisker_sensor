"""The cluster's recordings in one ssh call, cached locally: ``Catalog``.

One call (``cmd_catalog``) walks ``<root>/YYYY-MM-DD/`` and prints, per date
folder, its mtime, then each file's name, size and mtime and the content of
every ``.card.json`` and ``.notes.jsonl`` (base64 on one line each, so a note
appended while the listing runs can't break the framing). Folders whose mtime
is the one the local catalogue already has are only named (``S``), not walked:
every change there (an upload, a notes merge) ends in a ``mv`` into the folder,
which bumps its mtime, so a refresh moves only what changed. The mtime is
``stat -c %y`` (nanoseconds), compared as a string.

The same call deletes the ``.wavi_notes.*`` temporaries older than a day that
an interrupted notes sync (``sync.cmd_notes``) can leave in a date folder.

Lines::

    S <date> <mtime>                       unchanged: keep the cached folder
    D <date> <mtime>                       listed again; its files follow
    F <date> <size> <mtime_s> <name>       one file
    B <date> <name> <base64>               a card's or notes file's content
    NOROOT                                 the root doesn't exist (yet)

The catalogue (``catalog.json`` in the state dir) keeps, per folder, the mtime,
the files and the decoded cards and notes lines, with the time of the last
successful refresh (the browser shows its age) and the last error.
"""

from __future__ import annotations

import base64
import json
import os
import shlex
import time
from pathlib import Path
from typing import Optional

from wavi_py.remote.config import state_dir
from wavi_py.remote.sync import Session, SyncError, _sh
from wavi_py.replay import notes as rnotes

CATALOG_TIMEOUT_S = 120.0
CARD_SUFFIX = ".card.json"


def cmd_catalog(root: str, known: dict[str, str]) -> str:
    """The listing script. ``known``: date -> mtime of the cached folders."""
    pats = "|".join(shlex.quote(f"{d}={m}") for d, m in sorted(known.items())) or "__none__"
    return _sh(
        f"cd {shlex.quote(root)} 2>/dev/null || {{ echo NOROOT; exit 0; }}; "
        "find . -mindepth 2 -maxdepth 2 -name '.wavi_notes.*' -mmin +1440 -delete 2>/dev/null; "
        "for d in [0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]; do "
        "[ -d \"$d\" ] || continue; "
        "m=$(stat -c %y \"$d\" | tr ' ' _); "
        f"case \"$d=$m\" in {pats}) echo \"S $d $m\"; continue;; esac; "
        "echo \"D $d $m\"; "
        "for f in \"$d\"/*; do [ -f \"$f\" ] || continue; n=${f#*/}; "
        "printf 'F %s %s %s\\n' \"$d\" \"$(stat -c '%s %Y' \"$f\")\" \"$n\"; "
        "case \"$n\" in *.card.json|*.notes.jsonl) printf 'B %s %s ' \"$d\" \"$n\"; "
        "base64 -w0 < \"$f\"; echo;; esac; "
        "done; done")


def parse_catalog(out: bytes, old: dict) -> dict:
    """The new ``folders`` from the listing and the cached ones (``old``)."""
    folders: dict = {}
    for raw in out.decode("utf-8", "replace").splitlines():
        if raw.startswith("S "):
            _, d, m = raw.split(" ", 2)
            if d in old:
                folders[d] = old[d]
        elif raw.startswith("D "):
            _, d, m = raw.split(" ", 2)
            folders[d] = {"mtime": m, "files": {}, "cards": {}, "notes": {}}
        elif raw.startswith("F "):
            parts = raw.split(" ", 4)
            if len(parts) == 5 and parts[1] in folders:
                try:
                    folders[parts[1]]["files"][parts[4]] = [int(parts[2]), int(parts[3])]
                except ValueError:
                    pass
        elif raw.startswith("B "):
            parts = raw.split(" ", 3)
            if len(parts) != 4 or parts[1] not in folders:
                continue
            f, name = folders[parts[1]], parts[2]
            try:
                data = base64.b64decode(parts[3].strip() or b"")
            except ValueError:
                continue
            if name.endswith(CARD_SUFFIX):
                try:
                    card = json.loads(data)
                except ValueError:
                    continue
                if isinstance(card, dict):
                    f["cards"][name[: -len(CARD_SUFFIX)]] = card
            elif name.endswith(rnotes.SUFFIX):
                f["notes"][name[: -len(rnotes.SUFFIX)]] = rnotes.parse_lines(data)
    return folders


class Catalog:
    def __init__(self, path: Optional[Path] = None):
        self.path = Path(path) if path else state_dir() / "catalog.json"
        self.data: dict = {"host": "", "root": "", "t": None, "folders": {}, "error": "",
                           "noroot": False}
        try:
            d = json.loads(self.path.read_text(encoding="utf-8"))
            if isinstance(d, dict):
                self.data.update(d)
        except (OSError, ValueError):
            pass

    @property
    def folders(self) -> dict:
        return self.data["folders"]

    def age_s(self) -> Optional[float]:
        t = self.data.get("t")
        return None if t is None else max(0.0, time.time() - float(t))

    def matches(self, host: str, root: str) -> bool:
        return (self.data.get("host"), self.data.get("root")) == (host, root)

    def save(self) -> None:
        tmp = self.path.with_name(self.path.name + f".{os.getpid()}.tmp")
        tmp.write_text(json.dumps(self.data), encoding="utf-8")
        os.replace(tmp, self.path)

    def refresh(self, sess: Session) -> int:
        """One listing call; returns the number of folders walked again. Raises
        SyncError (the reason in words) and keeps the old catalogue then."""
        host, root = sess.cfg.host.strip(), sess.cfg.root
        if not sess.cfg.configured:
            raise SyncError("not set up: Cluster... in the Record row", False)
        if not self.matches(host, root):
            self.data.update(host=host, root=root, folders={}, t=None)
        known = {d: f["mtime"] for d, f in self.folders.items()}
        res = sess._run(cmd_catalog(root, known), CATALOG_TIMEOUT_S)
        if not res.ok:
            err = sess._fail(res, "listing")
            self.data["error"] = err.reason
            self.save()
            raise err
        self.data["noroot"] = res.out.startswith(b"NOROOT")
        folders = parse_catalog(res.out, self.folders)
        walked = sum(1 for line in res.out.splitlines() if line.startswith(b"D "))
        self.data.update(folders=folders, t=time.time(), error="")
        self.save()
        return walked
