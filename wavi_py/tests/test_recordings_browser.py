"""The recordings browser: the cluster catalogue (one ssh call, incremental,
temp cleanup), rows (pairing, grouping, numbering, filter, where), the download
cache (verified download, recorded as uploaded, cancel, short read, eviction)
and the window offscreen, all against the fake ssh (tests/fake_ssh.py).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_recordings_browser.py
"""

from __future__ import annotations

import base64
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wavi_py.remote import config as rconfig, rows as rrows  # noqa: E402
from wavi_py.remote.cache import Cache, download  # noqa: E402
from wavi_py.remote.catalog import Catalog, parse_catalog  # noqa: E402
from wavi_py.remote.sync import (Session, SyncError, SyncState,  # noqa: E402
                                 changed_since_upload, find_recordings, requeue_changed)
from wavi_py.replay import notes as rnotes  # noqa: E402

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="the fake ssh is a POSIX script")

DAY = "2026-11-02"
A1 = "st_2026-11-02_1639_23.86_pool"           # two with the same tag on one day
A2 = "st_2026-11-02_1702_10.00_pool"
B = "st_2026-11-02_1800_00.00_bench"
PI_A2 = "whisker_20261102_170208_pool"         # the Pi's file of A2 (2 s earlier)
PI_FAR = "whisker_20261102_170200_pool"        # 10 s off: not the same recording
OLD = "st_2026-11-01_0900_00.00_pool"


def make_card(stem: str, nch: int = 32, nbins: int = 256, dur: float = 120.0,
              seed: int = 0) -> dict:
    rng = np.random.default_rng(seed)
    q = rng.integers(30, 80, (nch, nbins), dtype=np.uint8)
    q[:, nbins // 3: nbins // 2] = 230                   # a "touch"
    q[4] = 0                                             # a dead gauge
    return {"v": 1, "stem": stem, "start": "", "tag": "", "duration_s": dur, "fs_hz": 250.0,
            "nch": nch, "n_scans": 1000, "n_imu": 100, "spikes": 3,
            "faults": {"warn": 0, "fault": 0}, "depth_m": [0.0, 0.4], "armed_s": 50.0,
            "preview": {"kind": "rms_db", "nbins": nbins, "lo_db": -40.0, "hi_db": 15.0,
                        "data": base64.b64encode(q.tobytes()).decode()},
            "depth_strip": None, "made_by": "test"}


def put_rec(folder: Path, stem: str, card: bool = True, notes: list = (), size: int = 3000,
            ext: str = ".jsonl", age_s: float = 3600) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    (folder / (stem + ext)).write_bytes(os.urandom(size))
    if card:
        (folder / f"{stem}.card.json").write_text(json.dumps(make_card(stem)))
    for n in notes:
        rnotes.append(folder / f"{stem}.notes.jsonl", n)
    t = time.time() - age_s
    for p in folder.glob(stem + ".*"):
        os.utime(p, (t, t))


@pytest.fixture
def env(tmp_path, monkeypatch):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    fake = bindir / "ssh"
    fake.write_text(f"#!{sys.executable}\n" + (ROOT / "wavi_py/tests/fake_ssh.py").read_text())
    fake.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bindir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setenv("WAVI_SETTINGS_INI", str(tmp_path / "settings.ini"))
    monkeypatch.setenv("WAVI_SYNC_STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setenv("FAKE_SSH_LOG", str(tmp_path / "ssh.log"))
    monkeypatch.delenv("FAKE_SSH_MODE", raising=False)

    class E:
        pass
    e = E()
    e.tmp = tmp_path
    e.remote = tmp_path / "cluster root"
    e.local = tmp_path / "local"
    e.local.mkdir()
    e.cfg = rconfig.SyncConfig(host="fakehost", root=str(e.remote), local_dir=str(e.local))
    rconfig.save(e.cfg)
    e.state = SyncState(tmp_path / "state" / "sync_state.json")
    e.sess = lambda: Session(e.cfg, e.state)
    e.calls = lambda: [json.loads(x) for x in (tmp_path / "ssh.log").read_text().splitlines()] \
        if (tmp_path / "ssh.log").exists() else []
    return e


def populate(remote: Path) -> None:
    d = remote / DAY
    put_rec(d, A1, notes=[rnotes.note("fine run", 12.0, by="a")])
    put_rec(d, A2, notes=[rnotes.flag(False, "loose whisker W3", by="a")], ext=".dat")
    put_rec(d, B, card=False)
    put_rec(remote / "2026-11-01", OLD)


# ----------------------------------------------------------------- catalogue

def test_catalog_incremental_and_cleanup(env):
    populate(env.remote)
    d = env.remote / DAY
    stale, fresh = d / ".wavi_notes.in.123", d / ".wavi_notes.in.456"
    stale.write_text("x")
    fresh.write_text("x")
    old = time.time() - 2 * 86400
    os.utime(stale, (old, old))
    os.utime(d, None)                                   # the folder's own mtime as is
    cat = Catalog(env.tmp / "catalog.json")
    assert cat.refresh(env.sess()) == 2
    f = cat.folders[DAY]
    assert set(f["files"]) >= {f"{A1}.jsonl", f"{A2}.dat", f"{B}.jsonl", f"{A1}.card.json"}
    assert f["cards"][A1]["stem"] == A1 and B not in f["cards"]
    assert f["notes"][A1][0]["text"] == "fine run"
    assert not stale.exists() and fresh.exists()        # stale temp: gone
    assert len(env.calls()) == 1                        # one ssh call
    # nothing changed: no folder walked; the cache file holds it across instances
    cat2 = Catalog(env.tmp / "catalog.json")
    assert cat2.refresh(env.sess()) == 0 and cat2.folders[DAY]["cards"][A1]["stem"] == A1
    # a file renamed into one folder (as every upload ends): only that one again
    time.sleep(0.01)
    (d / "x.tmp").write_text("{}")
    os.replace(d / "x.tmp", d / f"{B}.card.json")
    assert cat2.refresh(env.sess()) == 1 and B in cat2.folders[DAY]["cards"]
    assert cat2.age_s() < 5


def test_catalog_noroot_and_offline(env, monkeypatch):
    cat = Catalog(env.tmp / "catalog.json")
    assert cat.refresh(env.sess()) == 0 and cat.data["noroot"] and cat.folders == {}
    monkeypatch.setenv("FAKE_SSH_MODE", "timeout")
    with pytest.raises(SyncError) as ei:
        cat.refresh(env.sess())
    assert "timeout" in ei.value.reason and "timeout" in cat.data["error"]


def test_parse_catalog_framing():
    card = base64.b64encode(b'{"v": 1, "stem": "s"}').decode()
    notes = base64.b64encode(rnotes.note("a b c", by="x").__repr__().encode()).decode()
    out = (f"D {DAY} m1\nF {DAY} 10 1700000000 s.jsonl\nF {DAY} 5 1700000001 s.card.json\n"
           f"B {DAY} s.card.json {card}\nB {DAY} s.notes.jsonl {notes}\n"
           f"S 2026-11-01 m0\nF bad line\n").encode()
    old = {"2026-11-01": {"mtime": "m0", "files": {"o.jsonl": [1, 1]}, "cards": {}, "notes": {}},
           "2026-10-01": {"mtime": "gone", "files": {}, "cards": {}, "notes": {}}}
    f = parse_catalog(out, old)
    assert set(f) == {DAY, "2026-11-01"}                 # a folder gone on the cluster drops
    assert f[DAY]["files"]["s.jsonl"] == [10, 1700000000] and f[DAY]["cards"]["s"]["v"] == 1
    assert f[DAY]["notes"]["s"] == []                    # not JSON lines: nothing, no crash


# ----------------------------------------------------------------- rows

def test_rows_pairing_grouping_numbering_filter(env):
    populate(env.remote)
    put_rec(env.local, A1, notes=[rnotes.note("local only note", by="b")])   # both
    put_rec(env.local, PI_A2, card=False)               # the Pi's file of A2: one row
    put_rec(env.local, PI_FAR, card=False)              # 10 s off: its own row
    put_rec(env.local, "st_2026-11-03_1000_00.00_new")  # local only
    cat = Catalog(env.tmp / "catalog.json")
    cat.refresh(env.sess())
    env.state.enqueue(find_recordings(env.local)["st_2026-11-03_1000_00.00_new"])
    rows = rrows.build_rows(rrows.local_rows([env.local], env.state.queued()),
                            rrows.cluster_rows(cat.folders))
    by = {r.stem: r for r in rows}
    assert by[A1].where == "both" and by[B].where == "cluster"
    assert by["st_2026-11-03_1000_00.00_new"].where == "local, queued"
    # pairing: the laptop name leads (it has the cluster card); both stems listed
    assert A2 in by and PI_A2 not in by and set(by[A2].stems) == {A2, PI_A2}
    assert by[A2].local is not None and by[A2].cluster is not None
    assert PI_FAR in by
    # notes: the union of the local and the cluster copy
    assert {n["text"] for n in by[A1].state.notes} == {"fine run", "local only note"}
    assert by[A2].bad and by[A2].state.reason == "loose whisker W3"
    # numbering by start within date / tag: A1 #1, PI_FAR (17:02:00) #2, A2 #3
    assert (by[A1].n, by[PI_FAR].n, by[A2].n) == (1, 2, 3)
    assert by[B].n == 1 and by[OLD].n == 1
    g = rrows.grouped(rows)
    assert [d for d, _ in g] == ["2026-11-03", DAY, "2026-11-01"]
    tags = dict(g[1][1])
    assert list(dict(g[1][1])) == ["bench", "pool"]       # the newest group first
    assert [r.stem for r in tags["pool"]] == [A2, PI_FAR, A1]
    # filter: tag, note text, the bad reason, hide bad
    m = lambda **k: {r.stem for r in rows if rrows.matches(r, **k)}  # noqa: E731
    assert m(text="BENCH") == {B}
    assert m(text="fine run") == {A1}
    assert m(text="loose") == {A2}
    assert A2 not in m(hide_bad=True) and A1 in m(hide_bad=True)
    # numbers don't change with a filter: they are set over all rows
    assert by[A2].n == 3


# ----------------------------------------------------------------- download cache

def _cluster_files(cat: Catalog, stem: str) -> dict:
    return {n: v for n, v in cat.folders[DAY]["files"].items() if n.startswith(stem + ".")}


def test_download_verified_and_synced(env):
    populate(env.remote)
    cat = Catalog(env.tmp / "catalog.json")
    cat.refresh(env.sess())
    cache = Cache(env.tmp / "cache")
    seen = []
    path = download(env.sess(), cache, DAY, _cluster_files(cat, A1),
                    progress=lambda d, t, n: seen.append((d, t)))
    assert path == cache.dir / f"{A1}.jsonl"
    for n in _cluster_files(cat, A1):
        assert (cache.dir / n).read_bytes() == (env.remote / DAY / n).read_bytes()
    assert seen[-1][0] == seen[-1][1] and not list(cache.dir.glob("*.part"))
    # recorded as uploaded: sync sends nothing back
    rec = find_recordings(cache.dir)[A1]
    assert not changed_since_upload(rec, env.state.uploaded(A1))
    assert requeue_changed(env.state, cache.dir) == []
    # a note added in replay: the normal sync takes it up (the cache is watched)
    rnotes.append(cache.dir / f"{A1}.notes.jsonl", rnotes.note("seen in replay", 3.0))
    t = time.time() - 3600
    os.utime(cache.dir / f"{A1}.notes.jsonl", (t, t))
    assert requeue_changed(env.state, cache.dir) == [A1]
    from wavi_py.remote.sync import push_queue
    results, errors = push_queue(env.sess(), check_busy=False)
    assert not errors and results[0].uploaded == [f"{A1}.notes.jsonl"]
    texts = {e["text"] for e in rnotes.read(env.remote / DAY / f"{A1}.notes.jsonl")}
    assert texts == {"fine run", "seen in replay"}


def test_download_short_and_cancel(env, monkeypatch):
    populate(env.remote)
    cat = Catalog(env.tmp / "catalog.json")
    cat.refresh(env.sess())
    cache = Cache(env.tmp / "cache")
    monkeypatch.setenv("FAKE_SSH_MODE", "truncate_cat")
    with pytest.raises(SyncError) as ei:
        download(env.sess(), cache, DAY, _cluster_files(cat, A1))
    assert "verify" in ei.value.reason
    assert not list(cache.dir.glob(f"{A1}*"))           # nothing half-written stays
    monkeypatch.delenv("FAKE_SSH_MODE")
    sess = env.sess()
    with pytest.raises(SyncError) as ei:
        download(sess, cache, DAY, _cluster_files(cat, A1),
                 progress=lambda d, t, n: sess.cancel())
    assert ei.value.reason == "cancelled" or "cancelled" in ei.value.reason
    assert not list(cache.dir.glob("*.part")) and not (cache.dir / f"{A1}.jsonl").exists()
    # changed on the cluster since the listing: refused, not half-done
    (env.remote / DAY / f"{A1}.jsonl").write_bytes(b"other")
    with pytest.raises(SyncError) as ei:
        download(env.sess(), cache, DAY, _cluster_files(cat, A1))
    assert "Refresh" in ei.value.reason


def test_cache_eviction(env):
    cache = Cache(env.tmp / "cache")
    stems = [f"st_2026-11-0{i}_1000_00.00_c" for i in range(1, 5)]
    for i, s in enumerate(stems):
        put_rec(cache.dir, s, size=10_000)
        rec = find_recordings(cache.dir)[s]
        env.state.record(rec, "/r", {p.name: {"size": p.stat().st_size,
                                              "mtime_ns": p.stat().st_mtime_ns, "sha": "x"}
                                     for p in rec.files})
        cache.touch(s)
        time.sleep(0.01)
    rnotes.append(cache.dir / f"{stems[1]}.notes.jsonl", rnotes.note("not synced yet"))
    one = 10_000 + (cache.dir / f"{stems[0]}.card.json").stat().st_size
    # keep ~2: the oldest opened go, but not the protected one, not unsynced notes
    gone = cache.evict(env.state, protect=[str(cache.dir / f"{stems[0]}.jsonl")],
                       cap=2.5 * one)
    assert gone == [stems[2]] or gone == [stems[2], stems[3]]
    left = set(find_recordings(cache.dir))
    assert stems[0] in left and stems[1] in left and stems[2] not in left
    assert cache.size() <= 2.5 * one + 2000 or stems[3] in left


# ----------------------------------------------------------------- the window

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


def test_browser_window(env, app, monkeypatch):
    from wavi_py.views.recordings_browser import RecordingsBrowser, _walk
    populate(env.remote)
    put_rec(env.local, A1)
    opened = []
    b = RecordingsBrowser(env.local, opened.append, state=env.state,
                          catalog=Catalog(env.tmp / "catalog.json"),
                          cache=Cache(env.tmp / "cache"))
    b.resize(1100, 560)
    b.show()
    assert _wait(app, lambda: not b._listing and b.catalog.age_s() is not None)
    stems = lambda tree: [it.data(0, 0x0100) for it in _walk(tree) if it.data(0, 0x0100)]  # noqa
    assert set(stems(b.local_tree)) == {A1}
    assert set(stems(b.cluster_tree)) == {A1, A2, B, OLD}
    # a row's look: number, badge, preview icon
    b.tabs.setCurrentIndex(1)
    assert b.select(A2)
    r = b.selected()
    assert r.n == 2 and r.bad and "read-only" in b.notes_hint.text().lower()
    assert not b.add_note_btn.isEnabled()
    it = b.cluster_tree.currentItem()
    assert it.text(4) == "BAD" and "loose whisker" in it.toolTip(4)
    assert not it.icon(3).isNull()
    assert b.select(B) and "make_cards" in b.cluster_tree.currentItem().toolTip(3)
    # the filter, hide bad
    b.filter_edit.setText("bench")
    assert set(stems(b.cluster_tree)) == {B}
    b.filter_edit.setText("")
    b.hide_bad.setChecked(True)
    assert A2 not in stems(b.cluster_tree)
    b.hide_bad.setChecked(False)
    out = Path(os.environ.get("WAVI_SHOT_DIR", env.tmp))
    b.grab().save(str(out / "browser_test_cluster.png"))
    # open a local one: replayed at once
    b.tabs.setCurrentIndex(0)
    assert b.select(A1)
    b.open_selected()
    assert opened == [str(env.local / f"{A1}.jsonl")]
    # open a cluster-only one: downloaded into the cache, then opened
    b.tabs.setCurrentIndex(1)
    assert b.select(OLD)
    b.open_selected()
    assert _wait(app, lambda: len(opened) == 2)
    assert opened[1] == str(b.cache.dir / f"{OLD}.jsonl")
    assert b.last_download[0] == f"{OLD}.jsonl"
    assert next(r for r in b.rows if r.stem == OLD).where == "both"
    b.close()


def test_browser_not_set_up(env, app):
    from wavi_py.views.recordings_browser import RecordingsBrowser
    rconfig.save(rconfig.SyncConfig())
    put_rec(env.local, A1)
    b = RecordingsBrowser(env.local, lambda p: None, state=env.state,
                          catalog=Catalog(env.tmp / "catalog.json"),
                          cache=Cache(env.tmp / "cache"))
    b.show()
    app.processEvents()
    assert "not set up" in b.cluster_tree.topLevelItem(0).text(0)
    assert b.local_tree.topLevelItemCount() == 1
    b.close()
