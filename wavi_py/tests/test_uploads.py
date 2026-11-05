"""Upload visibility: the chunked stdin with byte progress (``proc.Runner``),
the ``SyncEvent`` stream of a push (skipped files, a failure, pause / remove),
the tracker and the CLI's lines, and the Uploads window offscreen, filled from
a real push through the fake ssh of test_remote_sync.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_uploads.py
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wavi_py.remote import config as rconfig, proc  # noqa: E402
from wavi_py.remote.progress import LinePrinter, Tracker  # noqa: E402
from wavi_py.remote.sync import (Session, SyncState, find_recordings, push_queue,  # noqa: E402
                                 queue_events, recording_of)
from wavi_py.tests.test_remote_sync import _wait, age, app, env, make_rec, sha  # noqa: E402,F401

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="the fake ssh is a POSIX script")

SLOW_CAT = ("import sys, time\n"
            "out = open(sys.argv[1], 'wb')\n"
            "while True:\n"
            "    b = sys.stdin.buffer.read(1 << 20)\n"
            "    if not b: break\n"
            "    out.write(b); time.sleep(0.05)\n"
            "out.close(); print('WAVI_OK')\n")


# ----------------------------------------------------------------- chunked stdin

def test_chunked_stdin_progress(tmp_path):
    src = tmp_path / "big.bin"
    src.write_bytes(os.urandom(12 * (1 << 20) + 12345))
    dst = tmp_path / "copy.bin"
    seen = []
    r = proc.Runner(ssh="unused")
    with open(src, "rb") as f:
        t0 = time.monotonic()
        res = r.run_argv([sys.executable, "-c", SLOW_CAT, str(dst)], 60, stdin=f,
                         on_bytes=lambda d, t: seen.append((time.monotonic(), d, t)))
        secs = time.monotonic() - t0
    assert res.ok and b"WAVI_OK" in res.out
    assert sha(dst) == sha(src)
    size = src.stat().st_size
    assert seen[-1][1:] == (size, size)                        # the final 100 % always comes
    assert all(t == size for _, _, t in seen)
    ds = [d for _, d, _ in seen]
    assert ds == sorted(ds) and len(set(ds)) == len(ds)
    assert 3 <= len(seen) <= secs * proc.EMIT_HZ + 3, (len(seen), secs)   # capped at ~4 Hz
    gaps = [b[0] - a[0] for a, b in zip(seen[:-2], seen[1:-1])]
    assert min(gaps, default=1) >= 0.2


def test_chunked_stdin_timeout_and_cancel(tmp_path):
    src = tmp_path / "big.bin"
    src.write_bytes(os.urandom(8 << 20))
    never_reads = [sys.executable, "-c", "import time; time.sleep(30)"]
    r = proc.Runner(ssh="unused")
    with open(src, "rb") as f:
        t0 = time.monotonic()
        res = r.run_argv(never_reads, 1.0, stdin=f, on_bytes=lambda d, t: None)
    assert res.rc == -1 and time.monotonic() - t0 < 10          # the feeder didn't hang it
    r2 = proc.Runner(ssh="unused")
    threading.Timer(0.5, r2.cancel).start()
    with open(src, "rb") as f:
        t0 = time.monotonic()
        res = r2.run_argv(never_reads, 30, stdin=f, on_bytes=lambda d, t: None)
    assert res.rc == -2 and time.monotonic() - t0 < 10
    assert proc.classify(res)[0] == "cancelled"


def test_popen_flags_kept_with_a_pipe(monkeypatch, tmp_path):
    """Windows: the no-console flag still goes to Popen when stdin is fed."""
    seen = {}
    real = subprocess.Popen

    def popen(argv, **kw):
        seen.update(kw)
        kw.pop("creationflags", None)
        return real(argv, **kw)
    monkeypatch.setattr(proc.subprocess, "Popen", popen)
    monkeypatch.setattr(proc.sys, "platform", "win32")
    src = tmp_path / "x"
    src.write_bytes(b"abc" * 1000)
    with open(src, "rb") as f:
        res = proc.Runner(ssh="unused").run_argv([sys.executable, "-c", "import sys;"
                                                  "sys.stdin.read()"], 30, stdin=f)
    assert res.ok and seen["creationflags"] == 0x08000000 and seen["stdin"] == subprocess.PIPE


# ----------------------------------------------------------------- the event stream

def _three(env):
    """A (uploaded before, then a card added: its files are skipped, the card
    goes), B (new), C (its date folder on the cluster is a file: mkdir fails,
    a non-network failure, the run goes on)."""
    a, b, c = ("st_2026-11-01_1000_00.00_a", "st_2026-11-02_1000_00.00_b",
               "st_2026-11-03_1000_00.00_c")
    for s in (a, b, c):
        make_rec(env.local, s, size=400_000)
    env.sess().push(find_recordings(env.local)[a])
    card = env.local / f"{a}.card.json"
    card.write_text('{"v": 1}')
    age(card)
    env.remote.mkdir(exist_ok=True)
    (env.remote / "2026-11-03").write_text("not a folder")
    recs = find_recordings(env.local)
    for s in (a, b, c):
        env.state.enqueue(recs[s])
        time.sleep(0.01)                     # "added" orders the queue
    return a, b, c


def test_event_stream(env):
    a, b, c = _three(env)
    evs, lines = [], []
    printer = LinePrinter(lines.append, every_s=0.0)
    results, errors = push_queue(env.sess(), check_busy=False,
                                 on_event=lambda e: (evs.append(e), printer(e)))
    assert [r.stem for r in results] == [a, b] and len(errors) == 1 and c in errors[0]
    assert (evs[0].stem, evs[0].phase, evs[0].n) == ("", "run", 3)
    assert evs[-1].phase == "run_end"
    queued = [e for e in evs if e.phase == "queued"]
    assert [e.stem for e in queued] == [a, b, c] and [e.k for e in queued] == [1, 2, 3]
    assert {n for n, _ in queued[0].files} == {f"{a}{x}" for x in
                                               (".jsonl", ".dat", ".faults.txt", ".card.json")}

    def phases(stem, file=""):
        return [e.phase for e in evs if e.stem == stem and e.file == file]
    for x in (".jsonl", ".dat", ".faults.txt"):
        assert phases(a, a + x) == ["skipped"]                 # hashes recorded: no hashing
    assert phases(a, a + ".card.json")[0] == "hashing"
    assert phases(a, a + ".card.json")[-3:] == ["uploading", "verifying", "done"]
    for stem in (a, b):                                       # nothing sent after verifying
        for name in {e.file for e in evs if e.stem == stem and e.file}:
            ph = phases(stem, name)
            if "verifying" in ph:
                assert ph[ph.index("verifying"):] == ["verifying", "done"], (name, ph)
    assert phases(a)[-1] == "done" and phases(b)[-1] == "done"
    jb = phases(b, b + ".jsonl")
    assert jb[0] == "hashing" and "uploading" in jb and jb[-2:] == ["verifying", "done"]
    ups = [e for e in evs if e.stem == b and e.file == b + ".jsonl" and e.phase == "uploading"]
    assert ups[0].done == 0 and ups[-1].done == ups[-1].total == 400_000
    assert phases(c, c + ".dat")[-1] == "failed"                # the first in name order
    fail = [e for e in evs if e.stem == c and e.file == "" and e.phase == "failed"]
    assert fail and "exit 3" in fail[0].error
    assert c in env.state.queued() and a not in env.state.queued()

    t = Tracker()
    for e in evs:
        t.apply(e)
    s = t.totals()
    assert (s["n"], s["n_done"], s["running"]) == (3, 2, False)
    assert t.recs[a].status() == "done" and t.recs[c].phase == "failed"
    assert t.recs[a].files[a + ".jsonl"].phase == "skipped"
    fc = {n: f.phase for n, f in t.recs[c].files.items()}
    assert fc[c + ".dat"] == "failed" and fc[c + ".jsonl"] == "queued"   # hashed, never sent
    assert t.order()[:1] == [c]                                # the queue first, then finished
    text = "\n".join(lines)
    starts = [x for x in lines if x.endswith(" MB") and ": uploading " in x]
    assert len(starts) == len(set(starts))                    # one start line per file
    assert "already on the cluster" in text and "FAILED" in text and f"{b}.jsonl: done" in text


def test_pause_and_remove(env):
    stems = [f"st_2026-11-02_10{i}0_00.00_p{i}" for i in range(3)]
    for s in stems:
        make_rec(env.local, s, size=50_000)
        env.state.enqueue(find_recordings(env.local)[s])
        time.sleep(0.01)
    evs = []
    files_done = []

    def stop(stem):
        if stem == stems[0]:
            return "skip"                                  # removed meanwhile
        return "pause" if len(files_done) >= 1 else ""

    def on_event(e):
        evs.append(e)
        if e.file and e.phase == "done":
            files_done.append(e.file)
    results, errors = push_queue(env.sess(), check_busy=False, on_event=on_event, stop=stop)
    assert results == [] and errors == []
    assert [e.phase for e in evs if e.stem == stems[0] and not e.file] == ["queued", "removed"]
    assert files_done == [stems[1] + ".dat"]                 # paused after the file in flight
    assert ("paused" in [e.phase for e in evs if e.stem == stems[1] and not e.file])
    q = env.state.queued()
    assert set(stems) <= set(q) and q[stems[1]]["tries"] == 0     # a pause is no failure
    assert list(env.state.uploaded(stems[1])) == [stems[1] + ".dat"]
    # resumed: only the rest of it goes
    results, _ = push_queue(env.sess(), check_busy=False)
    assert len(results) == 3 and stems[1] + ".dat" in results[1].skipped


def test_queue_events_after_restart(env):
    make_rec(env.local, "st_2026-11-02_1100_00.00_q")
    rec = recording_of(env.local / "st_2026-11-02_1100_00.00_q.dat")
    env.state.enqueue(rec)
    env.state.note_failure(rec.stem, "timeout (no network)")
    evs = queue_events(SyncState(env.state.path), protect=[str(rec.files[0])])
    assert len(evs) == 1 and evs[0].phase == "waiting" and "timeout" in evs[0].error
    assert sum(sz for _, sz in evs[0].files) == rec.size


# ----------------------------------------------------------------- the window

def test_uploads_window(env, app, monkeypatch):
    from PyQt6.QtWidgets import QWidget
    from wavi_py.remote import qt as rqt
    from wavi_py.remote.uploads import C_END, C_STATUS
    a, b, c = _three(env)
    big = "st_2026-11-02_1200_00.00_big"
    make_rec(env.local, big, size=6_000_000)
    env.state.enqueue(find_recordings(env.local)[big])
    env.cfg.auto_upload = True
    rconfig.save(env.cfg)
    out = Path(os.environ.get("WAVI_SHOT_DIR", env.tmp))
    w = QWidget()
    # the first push waits until the window is open (a slow "network": the
    # fake ssh's upload is throttled by a slow feeder)
    monkeypatch.setattr(proc, "FEED_CHUNK", 256 << 10)
    real_feed = proc._feed

    def slow_feed(src, dst, on_bytes, total):
        class Slow:
            def read(self, n):
                time.sleep(0.1)
                return src.read(n)
        real_feed(Slow(), dst, on_bytes, total)
    monkeypatch.setattr(proc, "_feed", slow_feed)
    ctl = rqt.SyncController(w, env.local, state=env.state)
    sw = ctl.status_widget()
    sw.resize(420, 30)
    texts = []
    ctl.status.connect(lambda lvl, text, tip: texts.append((text, tip)))
    try:
        ctl.show_uploads()
        win = ctl._uploads
        ctl.show_uploads()
        assert ctl._uploads is win                              # one instance
        win.resize(1000, 420)
        assert _wait(app, lambda: ctl.tracker.running or ctl.tracker.run)
        mid = _wait(app, lambda: any(r.active is not None and r.active.phase == "uploading"
                                     and r.active.name == big + ".jsonl"
                                     and r.stem == big and 30 < r.active.pct < 90
                                     for r in ctl.tracker.recs.values()), 60)
        assert mid
        win._items[big].setExpanded(True)
        app.processEvents()
        assert "Uploading: recording 4 of 4" in win.overall.text(), win.overall.text()
        win.grab().save(str(out / "uploads_window_mid.png"))
        sw.grab().save(str(out / "uploads_status_line.png"))
        line = ctl._label.full_text()
        assert line.startswith("Cluster: uploading 4/4 " + big + ".jsonl ") and line.endswith("%")
        assert ctl._label.text() != line and "…" in ctl._label.text()   # elided to fit
        assert _wait(app, lambda: not ctl.tracker.running and ctl._last_status[0] == "error",
                     60), ctl._last_status
        app.processEvents()
        st = {s: win._items[s].text(C_STATUS) for s in (a, b, c, big)}
        assert st[a] == "done" and st[b] == "done" and st[big] == "done", st
        assert st[c] == "failed" and "exit 3" in win._items[c].text(C_END)
        assert win.tree.indexOfTopLevelItem(win._items[c]) == 0  # queue first
        kids = {win._items[a].child(i).text(0): win._items[a].child(i).text(C_STATUS)
                for i in range(win._items[a].childCount())}
        assert kids[".jsonl"].startswith("skipped") and kids[".card.json"] == "done"
        assert "Last run: 3 of 4 recordings done" in win.overall.text()
        win._items[a].setExpanded(True)
        win.grab().save(str(out / "uploads_window_end.png"))
        # Remove from queue (no file deleted), Clear finished
        win.tree.clearSelection()
        win._items[c].setSelected(True)
        assert win.remove_btn.isEnabled()
        win._remove_selected()
        assert _wait(app, lambda: c not in env.state.queued()
                     and ctl.tracker.recs.get(c) is not None
                     and ctl.tracker.recs[c].phase == "removed")
        assert (env.local / f"{c}.jsonl").exists()
        win._clear_finished()
        assert win.tree.topLevelItemCount() == 0
        # Pause / Retry now
        ctl.set_paused(True)
        assert ctl._last_status[1].startswith("Cluster: paused")
        win._header()
        assert win.pause_btn.text() == "Resume" and not win.retry_btn.isEnabled()
        ctl.set_paused(False)
        ctl.retry_now()
        assert _wait(app, lambda: ctl._last_status[0] == "ok"), ctl._last_status
    finally:
        ctl.close()


def test_window_shows_the_queue_after_restart(env, app):
    from PyQt6.QtWidgets import QWidget
    from wavi_py.remote import qt as rqt
    from wavi_py.remote.uploads import C_END, C_STATUS
    make_rec(env.local, "st_2026-11-02_1300_00.00_r")
    rec = find_recordings(env.local)["st_2026-11-02_1300_00.00_r"]
    env.state.enqueue(rec)
    env.state.note_failure(rec.stem, "timeout (no network)")
    rconfig.save(env.cfg)                                    # auto-upload off: nothing runs
    ctl = rqt.SyncController(QWidget(), env.local, state=env.state)
    ctl.status_widget()
    try:
        ctl.show_uploads()
        win = ctl._uploads
        assert _wait(app, lambda: rec.stem in win._items)
        it = win._items[rec.stem]
        assert it.text(C_STATUS) == "queued" and "timeout" in it.text(C_END)
        assert it.childCount() == len(rec.files)
        assert "No upload has run" in win.overall.text() and "1 in the queue" in win.overall.text()
    finally:
        ctl.close()
