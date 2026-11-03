"""Export range (replay/export.py): the .jsonl clip (opens, replays, converts,
keeps the source's zero and depth) and the .dat; the panel's Export...

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_replay_export.py
"""

from __future__ import annotations

import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wavi_py.daq.outliers import fill_outliers_linear  # noqa: E402
from wavi_py.replay.export import export_dat, export_jsonl  # noqa: E402
from wavi_py.replay.index import K, RecordingIndex  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402
from wavi_py.tests.test_replay_index import DIVE  # noqa: E402
from whisker_link import convert  # noqa: E402


@pytest.fixture
def src(tmp_path):
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    return RecordingIndex.build(p)


def test_jsonl_clip(src, tmp_path):
    out = tmp_path / "clip.jsonl"
    n = export_jsonl(src, 3.0, 7.0, out)
    lines = out.read_bytes().splitlines()
    i0, i1 = np.searchsorted(src.t_play, src.t0 + np.array([3.0, 7.0]))
    # hello, the newest status before the range (the fixture's info comes later), the range
    assert src.last_before(src.info_lines, i0) is None
    assert len(lines) == n + 2 and json.loads(lines[1])["k"] == "status"
    with open(src.path, "rb") as f:
        assert out.read_bytes().endswith(src.read_lines(f, int(i0), int(i1)))
    clip = RecordingIndex.build(out)
    assert clip.hello["clip"]["src"] == "rec.jsonl"
    assert abs(clip.hello["clip"]["t_start"] - 3.0) < 0.01
    # session zero at the range start; the same wall clock
    assert abs(clip.t0 - src.t[i0]) < 1e-9
    assert abs(clip.unix0 - (src.unix0 + (clip.t0 - src.t0))) < 1e-6
    assert abs(clip.duration - 4.0) < 0.02
    assert clip.kind[1] == K["status"] and abs(clip.t[1] - clip.t0) < 1e-9     # re-stamped
    # the source's zero and surface, not the clip's own (it starts at depth)
    assert np.array_equal(clip.v0, src.v0) and clip.surface_hpa == src.surface_hpa
    for t in (0.5, 2.0, 3.5):
        assert abs(clip.depth_at(t) - src.depth_at(t + 3.0)) < 0.5
    assert abs(clip.max_depth()[1] - fx.MAX_DEPTH_CM) < 0.5
    # the events in the range, at their clip times
    modes = [(round(e.t + 3.0, 1), e.text) for e in clip.events if e.kind == "mode"]
    src_modes = [(round(e.t, 1), e.text) for e in src.events
                 if e.kind == "mode" and 3.0 <= e.t < 7.0]
    assert [m[1] for m in modes][-1] == src_modes[-1][1]
    assert abs(modes[-1][0] - src_modes[-1][0]) < 0.11
    # convert reads it
    arr = convert.to_arrays(convert.load(out))
    assert len(arr["wsk_mv"]) == clip.n_wsk
    with pytest.raises(ValueError):
        export_jsonl(src, 5.0, 5.0, tmp_path / "empty.jsonl")


def test_clip_replays(src, tmp_path):
    from wavi_py.client import WaviClient

    out = tmp_path / "clip.jsonl"
    export_jsonl(src, 3.0, 7.0, out)
    c = WaviClient(transport="replay", path=str(out), replay_speed=None)
    c.start()
    try:
        assert c.wait_ready(5)
        t_end = time.monotonic() + 10
        while c.replay.state != "eof" and time.monotonic() < t_end:
            time.sleep(0.02)
        time.sleep(0.2)
        assert np.allclose(c.bus.get_metadata()["V0"], src.v0)
        assert c.bus.total_published == c.index.n_wsk
    finally:
        c.stop()


def test_dat(src, tmp_path):
    out = tmp_path / "clip.dat"
    n = export_dat(src, 3.0, 7.0, out)
    rows = out.read_text().splitlines()
    assert len(rows) == n
    i0, i1 = np.searchsorted(src.t_play, src.t0 + np.array([3.0, 7.0]))
    assert n == int(np.count_nonzero(src.kind[i0:i1] == K["wsk"]))
    stamps = [datetime.strptime(r[:23], "%Y-%m-%d %H:%M:%S.%f").timestamp() for r in rows]
    vals = np.array([[float(x) for x in r[23:].split()] for r in rows])
    assert vals.shape == (n, 32)
    dt = np.diff(stamps)
    assert np.all(dt >= 0) and abs(np.median(dt) - 1 / fx.FS) < 1.5e-3
    # the wall clock of the first row: the fitted scan time, ~3 s in
    assert abs(stamps[0] - (src.unix0 + 3.0)) < 0.01
    # values: raw x mv_per_code as the bus has them, outliers filled per 20 Hz tick's block
    # as the recorder does (which does move ~1.5 % of this sine's samples)
    with open(src.path, "rb") as f:
        evs = [json.loads(x) for x in src.read_lines(f, int(i0), int(i1)).splitlines()]
    mv = np.array([e["raw"] for e in evs if e["k"] == "wsk"], dtype=np.float32) * np.float32(fx.MV)
    blk = int(round(fx.FS / 20))
    ref = np.concatenate([fill_outliers_linear(mv[k:k + blk]) for k in range(0, len(mv), blk)])
    assert np.allclose(vals, ref, atol=2e-5)                     # float32, 6 decimals
    assert 0 < np.count_nonzero(np.abs(vals - mv) > 1e-3) < 0.03 * mv.size


def test_panel_export(tmp_path):
    from PyQt6.QtWidgets import QApplication, QMessageBox

    from wavi_py.main_window import MainWindow
    from wavi_py.tests.test_replay_gui import _pump

    app = QApplication.instance() or QApplication([])
    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, t_buffer_s=2.0, replay_path=str(p))
    w.show()
    errors = []
    orig = QMessageBox.critical
    QMessageBox.critical = lambda *a, **k: errors.append(a[2])
    try:
        w.open_replay(str(p))
        assert _pump(app, lambda: w._client is not None and w._replay.active, 10)
        bar = w._timeline_docks[0].widget()
        assert bar.export_btn.isVisible() or not w.isVisible()
        w._replay.seek(5.0)
        w._replay.set_loop((4.0, 6.0))
        out = w.export_range(str(tmp_path / "loop.jsonl"))
        assert out is not None and "Exported" in w._path_label.text()
        clip = RecordingIndex.build(out)
        assert abs(clip.hello["clip"]["t_start"] - 4.0) < 0.01 and abs(clip.duration - 2.0) < 0.02
        out = w.export_range(str(tmp_path / "loop.dat"))
        assert out is not None and out.read_text().count("\n") > 400
        assert w.export_range(str(tmp_path / "bad.jsonl"), rng=(6.0, 6.0)) is None and errors
        # the timeline button asks the panel
        called = []
        w.export_range = lambda *a, **k: called.append(1)
        w._replay.exportRequested.disconnect()
        w._replay.exportRequested.connect(w.export_range)
        bar.export_btn.click()
        assert called
    finally:
        QMessageBox.critical = orig
        w.close()


@pytest.mark.skipif(not DIVE.exists(), reason="pilot2 recordings not here")
def test_real_dive_clip(tmp_path):
    src = RecordingIndex.open(DIVE)
    out = tmp_path / "dive_clip.jsonl"
    t = time.perf_counter()
    export_jsonl(src, 80.0, 95.0, out)                   # round the deepest point
    clip = RecordingIndex.build(out)
    print(f"\nclip in {time.perf_counter() - t:.2f} s: {clip.summary()}")
    tm, dm = clip.max_depth()
    stm, sdm = src.max_depth()
    assert abs(dm - sdm) < 0.01 and abs(tm + 80.0 - stm) < 0.05
    assert abs(clip.fs - src.fs) < 0.05 and np.array_equal(clip.v0, src.v0)
    n = export_dat(src, 80.0, 82.0, tmp_path / "dive.dat")
    assert abs(n - 2 * 250) < 5
    json.loads(out.read_bytes().split(b"\n", 1)[0])      # the hello parses


# ----- .dat-only replay (replay/dat_source.py)

def test_dat_replay(src, tmp_path, monkeypatch):
    from wavi_py.client import WaviClient
    from wavi_py.replay import dat_source
    from wavi_py.replay.index import NotARecording

    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path / "cache"))
    dat = tmp_path / "st_x.dat"
    n = export_dat(src, 0.0, 10.0, dat)
    stamps = [datetime.strptime(r[:23], "%Y-%m-%d %H:%M:%S.%f").timestamp()
              for r in dat.read_text().splitlines()[:2]]
    idx = RecordingIndex.open(dat)
    assert not idx.from_cache and idx.path.name == "st_x.dat.jsonl"
    assert str(tmp_path / "cache") in str(idx.path)              # not next to the .dat
    assert idx.n_wsk == n and not idx.has_vehicle and idx.n_raw_imu == 0
    assert abs(idx.fs - fx.FS) < 0.5                             # ms stamps: ~0.1 %
    assert abs(idx.unix0 - stamps[0]) < 1e-3 and abs(idx.duration - 10.0) < 0.05
    vals = np.array([[float(v) for v in r[23:].split()] for r in
                     dat.read_text().splitlines()[:int(fx.FS / 4)]])
    assert np.allclose(idx.v0, vals.mean(axis=0), atol=1e-4)
    assert RecordingIndex.open(dat).from_cache                   # converted and indexed once
    c = WaviClient(transport="replay", path=str(dat), replay_speed=None)
    c.start()
    try:
        assert c.wait_ready(5) and not c.has_vehicle
        t_end = time.monotonic() + 10
        while c.replay.state != "eof" and time.monotonic() < t_end:
            time.sleep(0.02)
        time.sleep(0.2)
        assert c.bus.total_published == n
        x, t, _ = c.bus.read_since_t(0)
        assert np.all(np.diff(t) > 0) and abs(np.median(np.diff(t)) - 1 / fx.FS) < 1e-3
    finally:
        c.stop()
    # changed: converted again
    with open(dat, "a") as f:
        f.write(dat.read_text().splitlines()[-1].replace("2", "2", 1) + "\n")
    st0 = dat_source.converted_path(dat).stat().st_mtime_ns
    assert RecordingIndex.open(dat).n_wsk == n + 1
    assert dat_source.converted_path(dat).stat().st_mtime_ns != st0
    # not the array's 32 gauges
    bad = tmp_path / "old.dat"
    bad.write_text("2026-01-01 10:00:00.000" + "   1.000000" * 9 + "\n")
    with pytest.raises(NotARecording):
        RecordingIndex.open(bad)


@pytest.mark.skipif(not DIVE.exists(), reason="pilot2 recordings not here")
def test_real_dat(tmp_path, monkeypatch):
    from wavi_py.tests.test_replay_index import SHORT

    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    t = time.perf_counter()
    d = RecordingIndex.open(SHORT.with_suffix(".dat"))
    secs = time.perf_counter() - t
    j = RecordingIndex.open(SHORT)
    print(f"\n.dat: {d.summary()} (converted + indexed in {secs:.1f} s)\n.jsonl: {j.summary()}")
    assert abs(d.n_wsk - j.n_wsk) < 20 and abs(d.duration - j.duration) < 1.0
    assert abs(d.fs - j.fs) < 0.5
    # the same zero, within the outlier fill and the few samples' head start
    assert np.allclose(d.v0, j.v0, atol=0.5)
