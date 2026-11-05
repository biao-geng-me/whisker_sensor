"""Cluster sync of recordings: ``wavi_py/remote``.

Recording is always local first; after Stop the laptop's own copies go to the
lab's cluster folder (``/shared/rc/whiskers/rov-recordings/YYYY-MM-DD/``) over
the user's OpenSSH (``ssh -o BatchMode=yes``: the app never sees credentials).
The contract is the recording-manager spec (notes, cards, sync, browser).

- ``config``: the settings (QSettings, shared by the GUI and the CLI) and the
  per-user state directory;
- ``proc``: finding and running ``ssh`` without a shell, with timeouts, no
  console window on Windows, and the failure reason parsed from its stderr;
- ``sync``: the connection test, the upload of one recording (``.part``, size
  and sha256 checked on the cluster, then renamed; notes merged), the
  persistent queue and the pruning of local copies;
- ``qt``: the worker thread, the status line in the Record row and the
  "Cluster..." dialog.

Names and dates come from ``replay/recname.py``, the notes rules from
``replay/notes.py``.

The command line twin is ``wavi_py/tools/wavi_sync.py``.
"""
