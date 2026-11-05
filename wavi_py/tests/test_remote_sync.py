"""Cluster sync (wavi_py/remote) against a fake ``ssh`` first on PATH
(tests/fake_ssh.py runs the remote command locally, the "cluster" is a temp
dir), so it runs anywhere without a network: the upload (.part, size + sha256
checked, renamed, group-writable), the skip of what is there, the notes merge,
a connection dropped mid-transfer (the part stays unrenamed, the recording
stays queued), the failure reasons, pruning, the CLI, and the GUI controller.
Plus the argument building of the Windows-only parts (no console window, where
ssh.exe is). Paths with spaces and a quote on both sides.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_remote_sync.py
"""

from __future__ import annotations

import hashlib
import json
import os
import stat
import subprocess
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import threading  # noqa: E402

from wavi_py.remote import config as rconfig, proc  # noqa: E402
from wavi_py.remote.sync import (BUSY_S, PushResult, Session, SyncError, SyncState,  # noqa: E402
                                 changed_since_upload, cmd_upload, find_recordings,
                                 not_uploaded, prune_execute, prune_plan, push_queue,
                                 recording_of, requeue_changed)
from wavi_py.replay import notes as rnotes  # noqa: E402

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="the fake ssh is a POSIX script")

LAPTOP = "st_2026-11-02_1639_23.86_in_water_pilot2"
PI = "whisker_20261103_101500_bench"


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def age(p: Path, secs: float = 3600) -> None:
    t = time.time() - secs
    os.utime(p, (t, t))


def note(id_: str, unix: float, text: str = "x") -> bytes:
    return (json.dumps({"v": 1, "id": id_, "ev": "note", "t": None, "text": text,
                        "by": "t", "unix": unix}) + "\n").encode()


@pytest.fixture
def env(tmp_path, monkeypatch):
    """fake ssh on PATH, settings + state in tmp, a local dir and a remote root
    (both with spaces; the root with a quote)."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    fake = bindir / "ssh"
    fake.write_text(f"#!{sys.executable}\n" + (ROOT / "wavi_py/tests/fake_ssh.py").read_text())
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setenv("WAVI_SETTINGS_INI", str(tmp_path / "settings.ini"))
    monkeypatch.setenv("WAVI_SYNC_STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setenv("FAKE_SSH_LOG", str(tmp_path / "ssh.log"))
    monkeypatch.setenv("FAKE_SSH_COUNT", str(tmp_path / "ssh.count"))
    monkeypatch.delenv("FAKE_SSH_MODE", raising=False)
    local = tmp_path / "local data dir"
    local.mkdir()
    remote = tmp_path / "cluster root's dir"
    cfg = rconfig.SyncConfig(host="fakehost", root=str(remote), local_dir=str(local))

    class E:
        pass
    e = E()
    e.tmp, e.local, e.remote, e.cfg, e.log = tmp_path, local, remote, cfg, tmp_path / "ssh.log"
    e.state = SyncState(tmp_path / "state" / "sync_state.json")
    (tmp_path / "state").mkdir(exist_ok=True)
    e.sess = lambda: Session(cfg, e.state)
    e.calls = lambda: [json.loads(x) for x in e.log.read_text().splitlines()] \
        if e.log.exists() else []
    return e


def make_rec(folder: Path, stem: str = LAPTOP, size: int = 300_000, notes: bytes = b"",
             old: bool = True) -> list[Path]:
    rng = os.urandom
    files = [folder / f"{stem}.jsonl", folder / f"{stem}.dat", folder / f"{stem}.faults.txt"]
    files[0].write_bytes(rng(size))
    files[1].write_bytes(rng(size // 3))
    files[2].write_text("at record start: warn\n")
    if notes:
        files.append(folder / f"{stem}.notes.jsonl")
        files[-1].write_bytes(notes)
    # caches and temporaries that never go up
    (folder / f"{stem}.jsonl.idx.npz").write_bytes(b"cache")
    (folder / f"{stem}.jsonl.store.npz").write_bytes(b"cache")
    (folder / f"{stem}.dat.part").write_bytes(b"junk")
    if old:
        for p in folder.iterdir():
            age(p)
    return files


# ----------------------------------------------------------------- names, notes

def ids_of(p: Path) -> set:
    return {ev["id"] for ev in rnotes.read(p)}


def test_recording_dates(env):
    f1, f2 = env.local / f"{LAPTOP}.dat", env.local / f"{PI}.jsonl"
    f1.write_bytes(b"x")
    f2.write_bytes(b"x")
    recs = find_recordings(env.local)
    assert recs[LAPTOP].date == "2026-11-02" and recs[PI].date == "2026-11-03"


def test_append_missing_never_loses_an_append(tmp_path):
    """notes.append_missing (the sync's local merge) against another writer
    appending under notes.py's lock at the same time: every line of both ends up
    in the file once, and the file is never rewritten."""
    p = tmp_path / "x.notes.jsonl"
    theirs = [rnotes.note(f"remote {i}") for i in range(200)]
    mine = []

    def writer():
        for i in range(200):
            mine.append(rnotes.append(p, rnotes.note(f"local {i}")))
    th = threading.Thread(target=writer)
    th.start()
    for k in range(0, 200, 10):                  # the sync, in bits, while the GUI appends
        rnotes.append_missing(p, theirs[: k + 10])
    th.join()
    n, before, st = rnotes.append_missing(p, theirs)
    lines = rnotes.read(p)
    assert n == 0 and before == st.st_size == p.stat().st_size
    assert len(lines) == 400 and {e["id"] for e in lines} == {e["id"] for e in mine + theirs}
    # a torn last line: the appended lines start on a new one
    with open(p, "ab") as f:
        f.write(b'{"v":1,"id":"torn"')
    n, _, _ = rnotes.append_missing(p, [rnotes.note("after")])
    assert n == 1 and len(rnotes.read(p)) == 401


def test_find_recordings_excludes_caches(env):
    make_rec(env.local)
    (env.local / "notes_only.notes.jsonl").write_text("")    # no .jsonl/.dat: not a recording
    recs = find_recordings(env.local)
    assert list(recs) == [LAPTOP]
    names = sorted(p.name for p in recs[LAPTOP].files)
    assert names == sorted([f"{LAPTOP}.dat", f"{LAPTOP}.faults.txt", f"{LAPTOP}.jsonl"])
    assert len(recs[LAPTOP].caches) == 2
    assert recording_of(env.local / f"{LAPTOP}.dat").stem == LAPTOP


# ----------------------------------------------------------------- process layer

def test_argv_and_windows_flags(monkeypatch):
    argv = proc.ssh_argv("ssh", "sporc", "true")
    assert argv[0] == "ssh" and argv[-2:] == ["sporc", "true"]
    assert "BatchMode=yes" in argv and f"ConnectTimeout={proc.CONNECT_TIMEOUT_S}" in argv
    with pytest.raises(ValueError):
        proc.ssh_argv("ssh", "-oProxyCommand=x", "true")
    win = proc.popen_kwargs("win32")
    assert win["shell"] is False and win["creationflags"] == 0x08000000   # CREATE_NO_WINDOW
    assert "creationflags" not in proc.popen_kwargs("linux")
    assert "creationflags" not in proc.popen_kwargs("darwin")
    c = proc.windows_ssh_candidates({"SystemRoot": r"C:\Windows"})
    assert c == [r"C:\Windows\System32\OpenSSH\ssh.exe", r"C:\Windows\Sysnative\OpenSSH\ssh.exe"]
    # not on PATH: the OpenSSH Client feature's folder, Sysnative for a 32-bit Python
    monkeypatch.setattr(proc.shutil, "which", lambda n: None)
    monkeypatch.setattr(proc.os.path, "isfile", lambda p: "Sysnative" in p)
    assert "Sysnative" in proc.find_ssh("win32", {"SystemRoot": r"C:\Windows"})
    with pytest.raises(proc.SshMissing):
        proc.find_ssh("linux")


def test_remote_command_quoting():
    """Remote paths are quoted for the cluster's sh; the whole script is one
    sh -c argument without a newline (fragile through a Windows command line)."""
    cmd = cmd_upload("/x/it's a dir/2026-11-02", "a b.dat", 10, "ab" * 32)
    assert cmd.startswith("sh -c '") and "\n" not in cmd
    # what the remote login shell hands sh: the script, paths still quoted in it
    script = subprocess.run(["/bin/sh", "-c", "printf %s " + cmd[len("sh -c "):]],
                            capture_output=True, text=True).stdout
    assert "'/x/it'\"'\"'s a dir/2026-11-02/a b.dat.part'" in script


def test_classify_reasons():
    R = proc.Result
    cases = {
        "Host key verification failed.": ("host key", False),
        "u@h: Permission denied (publickey).": ("auth refused", False),
        "ssh: connect to host h port 22: Connection timed out": ("timeout", True),
        "ssh: Could not resolve hostname h: Name or service not known": ("unknown host", True),
        "ssh: connect to host h port 22: Connection refused": ("refused", True),
        "ssh: connect to host h port 22: No route to host": ("no route", True),
    }
    for err, (frag, offline) in cases.items():
        why, off = proc.classify(R(255, b"", err), "h")
        assert frag in why and off is offline, err
    assert proc.classify(R(-1, b"", ""))[1] is True
    assert "no ssh" in proc.classify(R(-3, b"", "no ssh on PATH"))[0]
    assert "exit 4" in proc.classify(R(4, b"", ""))[0]


# ----------------------------------------------------------------- connection test

def test_connection_test(env, monkeypatch):
    s = env.sess()
    ok, why, _ = s.test()
    assert ok and env.remote.is_dir(), why
    assert all("BatchMode=yes" in c for c in env.calls())
    for mode, frag, offline in (("timeout", "timeout", True), ("auth", "auth refused", False),
                                ("hostkey", "host key", False), ("resolve", "unknown host", True)):
        monkeypatch.setenv("FAKE_SSH_MODE", mode)
        ok, why, off = env.sess().test()
        assert not ok and frag in why and off is offline, (mode, why)
    monkeypatch.delenv("FAKE_SSH_MODE")
    ro = env.tmp / "ro"
    ro.mkdir()
    (ro / "root").mkdir()
    (ro / "root").chmod(0o555)
    try:
        cfg = rconfig.SyncConfig(host="fakehost", root=str(ro / "root"))
        ok, why, off = Session(cfg, env.state).test()
        assert not ok and "not writable" in why and off is False
    finally:
        (ro / "root").chmod(0o755)
    ok, why, _ = Session(env.cfg, env.state, proc.Runner(ssh=str(env.tmp / "nope"))).test()
    assert not ok and "no ssh" in why


# ----------------------------------------------------------------- upload

def test_push_uploads_verifies_and_skips(env):
    files = make_rec(env.local, notes=note("aa", 1))
    rec = recording_of(files[0])
    r = env.sess().push(rec)
    rdir = env.remote / "2026-11-02"
    assert sorted(r.uploaded) == sorted(p.name for p in files)
    for p in files:
        q = rdir / p.name
        assert sha(q) == sha(p)
        assert stat.S_IMODE(q.stat().st_mode) & 0o060 == 0o060      # group rw
    assert stat.S_IMODE(rdir.stat().st_mode) & 0o070 == 0o070       # umask 002 on mkdir
    assert not list(rdir.glob("*.part")) and not list(rdir.glob("*.npz"))
    assert not changed_since_upload(rec, env.state.uploaded(LAPTOP))
    # again: everything is there with the same size and hash, nothing is sent
    n0 = len(env.calls())
    r2 = env.sess().push(recording_of(files[0]))
    assert r2.uploaded == [] and sorted(r2.skipped) == sorted(p.name for p in files)
    assert not any("cat > " in " ".join(c) for c in env.calls()[n0:])


def test_pi_stem_and_remote_partial(env):
    """A Pi-named recording goes to its date; a remote file with the wrong
    content (same size) is re-sent, not skipped."""
    f = env.local / f"{PI}.jsonl"
    f.write_bytes(os.urandom(5000))
    age(f)
    rdir = env.remote / "2026-11-03"
    rdir.mkdir(parents=True)
    (rdir / f.name).write_bytes(os.urandom(5000))
    r = env.sess().push(recording_of(f))
    assert r.uploaded == [f.name] and sha(rdir / f.name) == sha(f)


def test_notes_merged_both_ways(env):
    files = make_rec(env.local, notes=note("aa", 1) + note("bb", 3))
    env.sess().push(recording_of(files[0]))
    rnotes_p = env.remote / "2026-11-02" / f"{LAPTOP}.notes.jsonl"
    assert rnotes_p.read_bytes() == files[-1].read_bytes()
    assert stat.S_IMODE(rnotes_p.stat().st_mode) & 0o060 == 0o060
    rnotes_p.write_bytes(rnotes_p.read_bytes() + note("cc", 2))      # added on the cluster
    lnotes = env.local / f"{LAPTOP}.notes.jsonl"
    lnotes.write_bytes(lnotes.read_bytes() + note("dd", 4))          # and here
    age(lnotes, 2 * BUSY_S)                                          # not being written
    rec = recording_of(files[0])
    assert changed_since_upload(rec, env.state.uploaded(LAPTOP)) == [lnotes.name]
    assert requeue_changed(env.state, env.local) == [LAPTOP]
    n0 = len(env.calls())
    results, errors = push_queue(env.sess(), check_busy=False)
    assert not errors and results[0].notes_merged and results[0].uploaded == [lnotes.name]
    assert ids_of(lnotes) == ids_of(rnotes_p) == {"aa", "bb", "cc", "dd"}
    # the remote keeps its lines first; the local file was only appended to
    assert [e["id"] for e in rnotes.read(rnotes_p)] == ["aa", "bb", "cc", "dd"]
    assert [e["id"] for e in rnotes.read(lnotes)] == ["aa", "bb", "dd", "cc"]
    assert len(env.calls()) - n0 == 2                 # the listing + one notes call
    assert not env.state.queued()
    assert not list(rnotes_p.parent.glob(".wavi_notes.*"))
    # nothing changed anywhere: no notes call
    n0 = len(env.calls())
    r = env.sess().push(recording_of(files[0]))
    assert r.uploaded == [] and lnotes.name in r.skipped and len(env.calls()) - n0 == 1
    # a note added on the cluster only (another laptop): pulled down by the next push
    rnotes_p.write_bytes(rnotes_p.read_bytes() + note("ee", 5))
    r = env.sess().push(recording_of(files[0]))
    assert r.notes_merged and "ee" in ids_of(lnotes) and r.uploaded == []


def test_notes_only_on_cluster_come_down(env):
    files = make_rec(env.local)
    rdir = env.remote / "2026-11-02"
    rdir.mkdir(parents=True)
    (rdir / f"{LAPTOP}.notes.jsonl").write_bytes(note("zz", 5))
    r = env.sess().push(recording_of(files[0]))
    assert r.notes_merged and ids_of(env.local / f"{LAPTOP}.notes.jsonl") == {"zz"}
    assert f"{LAPTOP}.notes.jsonl" not in r.uploaded                 # it was there already
    assert not changed_since_upload(recording_of(files[0]), env.state.uploaded(LAPTOP))


def test_two_laptops_at_once(env):
    """Two copies of one recording (two laptops) syncing their notes at the
    same moment: the remote lock serializes the merges, so the cluster ends with
    both laptops' notes after that one round, and each laptop with the union of
    itself and whatever merged before it."""
    for k in range(5):
        stem = f"st_2026-11-02_1639_2{k}.00_two"
        dirs = [env.tmp / f"laptop {i} {k}" for i in (1, 2)]
        for i, d in enumerate(dirs):
            d.mkdir()
            make_rec(d, stem, size=1000,
                     notes=b"".join(note(f"L{i}n{j}", j) for j in range(30)))
        sessions = [Session(env.cfg, SyncState(d / "state.json")) for d in dirs]
        recs = [find_recordings(d)[stem] for d in dirs]
        go = threading.Barrier(2)
        errs = []

        def run(s, r):
            go.wait()
            try:
                s.sync_notes(r, s.remote_dir(r), {}, PushResult(r.stem))
            except Exception as exc:  # noqa: BLE001
                errs.append(exc)
        ths = [threading.Thread(target=run, args=(s, r)) for s, r in zip(sessions, recs)]
        [t.start() for t in ths]
        [t.join() for t in ths]
        assert not errs
        remote = env.remote / "2026-11-02" / f"{stem}.notes.jsonl"
        assert len(ids_of(remote)) == 60 and len(rnotes.read(remote)) == 60
        # the second one to merge has everything; a second round gives both
        for s, r in zip(sessions, recs):
            s.push(find_recordings(r.dir)[stem])
        for d in dirs:
            assert ids_of(d / f"{stem}.notes.jsonl") == ids_of(remote)


def test_drop_mid_transfer_keeps_part_and_queue(env, monkeypatch):
    files = make_rec(env.local, size=2_000_000)
    rec = recording_of(files[0])
    env.state.enqueue(rec)
    monkeypatch.setenv("FAKE_SSH_MODE", "drop_after:2")    # the 2nd file's link dies halfway
    with pytest.raises(SyncError) as ei:
        push_queue(env.sess(), check_busy=False)
    assert ei.value.offline
    rdir = env.remote / "2026-11-02"
    parts = list(rdir.glob("*.part"))
    assert len(parts) == 1 and parts[0].stat().st_size > 0
    final = rdir / parts[0].name[: -len(".part")]
    assert not final.exists()                               # never renamed
    q = env.state.queued()
    assert LAPTOP in q and q[LAPTOP]["tries"] == 1 and "connection lost" in q[LAPTOP]["error"]
    up = env.state.uploaded(LAPTOP)
    assert len(up) == 1 and final.name not in up            # the 1st file is recorded
    # the queue survives a restart (a new state object on the same file), and a
    # later push finishes it: only the missing files go
    monkeypatch.delenv("FAKE_SSH_MODE")
    state2 = SyncState(env.state.path)
    results, errors = push_queue(Session(env.cfg, state2), check_busy=False)
    assert not errors and len(results[0].uploaded) == 2 and len(results[0].skipped) == 1
    assert not list(rdir.glob("*.part")) and sha(final) == sha(env.local / final.name)
    assert not state2.queued()


def test_push_queue_busy_and_missing(env):
    files = make_rec(env.local, old=False)
    rec = recording_of(files[0])
    env.state.enqueue(rec)
    env.state.edit(lambda d: d["queue"].setdefault("gone_stem", {"dir": str(env.local)}))
    results, errors = push_queue(env.sess())                # busy: modified just now
    assert results == [] and LAPTOP in env.state.queued()
    assert "gone_stem" not in env.state.queued()
    results, _ = push_queue(env.sess(), protect=[str(files[1])], check_busy=False)
    assert results == []                                    # protected: being written
    results, _ = push_queue(env.sess(), check_busy=False)
    assert len(results) == 1 and not env.state.queued()


# ----------------------------------------------------------------- prune

def test_prune(env):
    stems = [f"st_2026-11-0{d}_1000_00.00_t{d}" for d in range(1, 6)]
    for i, s in enumerate(stems):
        make_rec(env.local, s, size=1000, old=False)
        for p in env.local.glob(s + ".*"):
            age(p, 10000 - 1000 * i)                        # stems[4] is the newest
    sess = env.sess()
    for s in stems[1:]:                                     # stems[0] never uploaded
        sess.push(find_recordings(env.local)[s])
    # stems[2]'s copy on the cluster went bad: not verified any more
    bad = env.remote / "2026-11-03" / f"{stems[2]}.dat"
    bad.write_bytes(b"\0" * bad.stat().st_size)
    plan = prune_plan(sess, env.local, keep_n=1, protect=[str(env.local / f"{stems[3]}.jsonl")])
    assert [r.stem for r in plan.delete] == [stems[1]]
    why = dict(plan.kept_unverified)
    assert "in use" in why[stems[3]] and "differs" in why[stems[2]]
    assert "not uploaded" in why[stems[0]]
    paths = plan.paths()
    assert any(p.name.endswith(".idx.npz") for p in paths)  # caches go with it
    gone = prune_execute(sess, plan)
    left = [p.name for p in env.local.glob(stems[1] + ".*")]
    assert set(gone) == set(paths) and left == [f"{stems[1]}.dat.part"]   # not ours: kept
    assert len(find_recordings(env.local)) == 4
    assert (env.remote / "2026-11-02" / f"{stems[1]}.jsonl").exists()


# ----------------------------------------------------------------- CLI

def test_cli(env, capsys):
    from wavi_py.tools import wavi_sync
    for i in range(3):
        s = f"st_2026-11-02_100{i}_00.00_c{i}"
        make_rec(env.local, s, size=2000, old=False)
        for p in env.local.glob(s + ".*"):
            age(p, 5000 - 1000 * i)
    base = ["--host", "fakehost", "--root", str(env.remote), "--dir", str(env.local), "--keep", "1"]
    assert wavi_sync.main(base + ["test"]) == 0
    assert wavi_sync.main(base + ["push"]) == 0
    out = capsys.readouterr().out
    assert "3 pushed" in out
    assert wavi_sync.main(base + ["status"]) == 0
    assert capsys.readouterr().out.count("  uploaded") == 3
    assert wavi_sync.main(base + ["prune"]) == 0
    out = capsys.readouterr().out
    assert "dry run" in out and len(find_recordings(env.local)) == 3
    assert wavi_sync.main(base + ["prune", "--yes"]) == 0
    assert list(find_recordings(env.local)) == ["st_2026-11-02_1002_00.00_c2"]
    assert wavi_sync.main(base + ["push"]) == 0             # nothing left to do
    assert "0 recording(s) to push" in capsys.readouterr().out


def test_cli_offline(env, monkeypatch, capsys):
    from wavi_py.tools import wavi_sync
    monkeypatch.setenv("FAKE_SSH_MODE", "timeout")
    rc = wavi_sync.main(["--host", "fakehost", "--root", str(env.remote), "push"])
    assert rc == 2 and "timeout" in capsys.readouterr().out


def test_config_roundtrip(env):
    c = rconfig.SyncConfig(host="h", root="/r", auto_upload=True, prune=True, keep_n=3)
    rconfig.save(c)
    d = rconfig.load()
    assert (d.host, d.root, d.auto_upload, d.prune, d.keep_n) == ("h", "/r", True, True, 3)


# ----------------------------------------------------------------- GUI controller

@pytest.fixture(scope="module")
def app():
    from PyQt6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def _wait(app, cond, secs=20.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_controller_auto_upload(env, app, monkeypatch):
    from PyQt6.QtWidgets import QWidget
    from wavi_py.remote import qt as rqt
    monkeypatch.setattr(rqt, "STOP_DELAY_MS", 10)
    env.cfg.auto_upload = True
    rconfig.save(env.cfg)
    w = QWidget()
    files = make_rec(env.local, old=False)
    ctl = rqt.SyncController(w, env.local, state=env.state)
    sw = ctl.status_widget()
    seen = []
    ctl.status.connect(lambda lvl, text, tip: seen.append(text))
    try:
        ctl.recording_stopped(files[1])
        assert _wait(app, lambda: (env.remote / "2026-11-02" / files[1].name).exists()
                     and ctl._last_status[0] == "ok"), seen
        assert any("uploading" in s for s in seen), seen
        # the card, written seconds later by Stop's child process (card.py's
        # thread calls on_done): it goes up on its own, the rest is skipped
        card = env.local / f"{LAPTOP}.card.json"
        card.write_text('{"v": 1}')
        n0 = len(env.calls())
        threading.Thread(target=ctl.card_written, args=(card, None)).start()
        assert _wait(app, lambda: (env.remote / "2026-11-02" / card.name).exists()
                     and ctl._last_status[0] == "ok"), seen
        ups = [c for c in env.calls()[n0:] if "cat > " in " ".join(c)]
        assert len(ups) == 1 and card.name in " ".join(ups[0])
        ctl.card_written(env.local / "x.card.json", "failed")         # no card: nothing
        # offline: the next one stays queued
        make_rec(env.local, "st_2026-11-02_1700_00.00_b", old=False)
        monkeypatch.setenv("FAKE_SSH_MODE", "timeout")
        ctl.last_test = None
        ctl.recording_stopped(env.local / "st_2026-11-02_1700_00.00_b.dat")
        assert _wait(app, lambda: ctl._last_status[1] == "Cluster: queued (1, offline)"), seen
        sw.resize(500, 30)
        out = Path(os.environ.get("WAVI_SHOT_DIR", env.tmp))
        sw.grab().save(str(out / "sync_status_offline.png"))
        # back online: the retry pushes it
        monkeypatch.delenv("FAKE_SSH_MODE")
        ctl._retry()
        assert _wait(app, lambda: ctl._last_status[0] == "ok"), seen
        assert not env.state.queued()
    finally:
        ctl.close()


def test_controller_off_and_dialog(env, app):
    from PyQt6.QtWidgets import QWidget
    from wavi_py.remote import qt as rqt
    w = QWidget()
    ctl = rqt.SyncController(w, env.local, state=env.state)
    ctl.status_widget()
    try:
        assert ctl._last_status[1] == "Cluster: off"
        dlg = rqt.ClusterDialog(env.cfg, w)
        dlg.host_edit.setText("fakehost")
        dlg._test()
        assert _wait(app, lambda: dlg.tested is not None)
        assert dlg.tested[2][0], dlg.tested
        assert dlg.result_config().host == "fakehost"
        out = Path(os.environ.get("WAVI_SHOT_DIR", env.tmp))
        dlg.grab().save(str(out / "sync_dialog.png"))
    finally:
        ctl.close()


def test_upload_existing(env, app, monkeypatch):
    """Auto-upload never sends old recordings; "Upload existing..." counts the
    ones not on the cluster, asks, queues them, and they go after OK."""
    from PyQt6.QtWidgets import QDialog, QMessageBox, QWidget
    from wavi_py.remote import qt as rqt
    stems = [f"st_2026-11-01_100{i}_00.00_old{i}" for i in range(3)]
    for s_ in stems:
        make_rec(env.local, s_, size=1000, old=False)
    for p in env.local.iterdir():
        age(p)
    env.sess().push(find_recordings(env.local)[stems[0]])          # one is up already
    rconfig.save(env.cfg)                                           # auto-upload off
    w = QWidget()
    ctl = rqt.SyncController(w, env.local, state=env.state)
    ctl.status_widget()
    asked = []
    monkeypatch.setattr(QMessageBox, "question",
                        lambda *a, **k: asked.append(a[2]) or QMessageBox.StandardButton.Yes)

    def exec_(dlg):
        dlg._upload_existing()
        dlg.grab().save(str(Path(os.environ.get("WAVI_SHOT_DIR", env.tmp))
                            / "sync_dialog_existing.png"))
        return QDialog.DialogCode.Accepted
    monkeypatch.setattr(rqt.ClusterDialog, "exec", exec_)
    try:
        assert [r.stem for r in ctl.existing()] == stems[1:]
        assert [r.stem for r in not_uploaded(env.state, env.local,
                                             protect=[str(env.local / f"{stems[1]}.dat")])] \
            == stems[2:]                                            # in use: not offered
        ctl.open_dialog()
        assert "2 recording(s)" in asked[0]
        assert _wait(app, lambda: not env.state.queued() and ctl._last_status[0] == "off")
        for s_ in stems:
            assert (env.remote / "2026-11-01" / f"{s_}.jsonl").exists()
        assert ctl.existing() == []
    finally:
        ctl.close()
