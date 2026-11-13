"""The whole-recording store (replay/store.py), the series API and its min / max
decimation (daq/series.py), and the Line / IMU windows drawing from it: the
playhead, follow / unfollow, the zoom out to the full recording (replay) or the
buffer (live), and a scrub that moves them without a transport round trip.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_replay_store.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from wavi_py.client import WaviClient  # noqa: E402
from wavi_py.daq.series import IMU, WSK, ArraySeries, peak_decimate  # noqa: E402
from wavi_py.replay.index import RecordingIndex  # noqa: E402
from wavi_py.replay.store import RecordingStore  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402

DIVE = Path("/scratch/bigeme/whisker_dev/pilot2/st_2026-11-02_1644_57.91_in_water_pilot2.jsonl")


def _pipeline(path, **kw):
    """Play ``path`` through the live pipeline as fast as it reads: every bus row."""
    c = WaviClient(transport="replay", path=str(path), replay_speed=None, **kw)
    c.start()
    try:
        assert c.wait_ready(5)
        t_end = time.monotonic() + 20
        while c.replay.state != "eof" and time.monotonic() < t_end:
            time.sleep(0.02)
        time.sleep(0.2)
        x, t, _ = c.bus.read_since_t(0)
        rows, ti, _ = c.imu_bus.read_since_t(0)
        return c.index, x, t, rows, ti
    finally:
        c.stop()


@pytest.fixture(scope="module")
def rec(tmp_path_factory):
    p = tmp_path_factory.mktemp("store") / "rec.jsonl"
    fx.write_recording(p, spikes=True)
    return p


def test_store_matches_pipeline_raw_and_imu(rec):
    idx, x, t, rows, ti = _pipeline(rec)
    st = RecordingStore.build(idx)
    assert st.wsk.shape == x.shape and st.wsk.dtype == np.float32
    assert np.array_equal(st.wsk_t, t)                    # the fitted times, bit for bit
    assert np.array_equal(st.wsk, x)                      # mV as ServerReader makes them
    assert np.array_equal(st.imu_t, ti)
    assert np.array_equal(st.imu, rows)                   # accel, gyro, held attitude
    assert st.names(WSK)[:2] == ["W1_A", "W1_B"]


def test_store_matches_pipeline_despiked(rec):
    idx, x, t, _, _ = _pipeline(rec, replay_despike=True)
    st = RecordingStore.build(idx)
    assert len(st.spk_row) == len(fx.SPIKES)
    assert np.array_equal(st.despiked(), x)
    assert not np.array_equal(st.wsk, x)                  # raw keeps the spikes
    _, y = st.get(WSK, [0], 1.0, 1.4, despike=True)
    _, yr = st.get(WSK, [0], 1.0, 1.4)
    assert np.abs(y - np.median(y)).max() < np.abs(yr - np.median(yr)).max()


def test_store_cache_and_api(rec, tmp_path):
    idx = RecordingIndex.open(rec)
    a = RecordingStore.open(idx)
    b = RecordingStore.open(idx)
    assert not a.from_cache and b.from_cache
    assert np.array_equal(a.wsk, b.wsk) and np.array_equal(a.imu_t, b.imu_t)
    assert np.array_equal(a.despiked(), b.despiked())
    ch = b.channels(["W2_B", "ax_mg", "pitch_rad"], 2.0, 3.0)
    tw, yw = ch["W2_B"]
    assert tw.min() >= 2.0 and tw.max() <= 3.0 and np.array_equal(yw, b.get(WSK, [3], 2.0, 3.0)[1][:, 0])
    ti, _ = ch["ax_mg"]
    assert 90 < len(ti) < 110                               # the IMU's own axis, 100 Hz
    with pytest.raises(KeyError):
        b.channels(["nope"], 0, 1)


@pytest.mark.skipif(not DIVE.exists(), reason="pilot2 recordings not here")
def test_store_matches_pipeline_mid_dive():
    """A seek to 400 s of the real dive: the pipeline's rows (preroll + 3 s) are the
    store's rows over the same span (the IMU's attitude only after the first ATTITUDE)."""
    idx = RecordingIndex.open(DIVE)
    st = RecordingStore.open(idx)
    c = WaviClient(transport="replay", index=idx, path=str(DIVE), replay_speed=8.0,
                   replay_start_s=400.0, replay_playing=True)
    c.start()
    try:
        assert c.wait_ready(10)
        t_end = time.monotonic() + 10
        while c.replay.position < 403.0 and time.monotonic() < t_end:
            time.sleep(0.05)
        x, t, _ = c.bus.read_since_t(0)
        rows, ti, _ = c.imu_bus.read_since_t(0)
    finally:
        c.stop()
    i0 = int(np.searchsorted(st.wsk_t, t[0]))
    assert np.array_equal(st.wsk_t[i0:i0 + len(t)], t)
    assert np.array_equal(st.wsk[i0:i0 + len(t)], x)
    j0 = int(np.searchsorted(st.imu_t, ti[0]))
    assert np.array_equal(st.imu_t[j0:j0 + len(ti)], ti)
    assert np.array_equal(st.imu[j0:j0 + len(ti), :6], rows[:, :6])
    late = ti > ti[0] + 0.5                               # the held attitude has caught up
    assert np.array_equal(st.imu[j0:j0 + len(ti)][late], rows[late])


# ------------------------------------------------------------ decimation

def _synthetic_store(n=200_000, nch=4, fs=250.0, seed=0) -> RecordingStore:
    rng = np.random.default_rng(seed)
    st = RecordingStore()
    st.wsk_t = np.arange(n) / fs
    st.wsk = np.cumsum(rng.normal(size=(n, nch)), axis=0).astype(np.float32)
    st.wsk[rng.integers(0, n, 50), rng.integers(0, nch, 50)] += 500.0       # lone spikes
    st.wsk_names = [f"c{i}" for i in range(nch)]
    st.imu_t = np.arange(n // 2) / (fs / 2)
    st.imu = rng.normal(size=(n // 2, 9)).astype(np.float32)
    return st


def _check_envelope(t, y, td, yd, t0, t1, n):
    """Every column pair is the min / max of the samples between its times; the
    extremes over the span are kept; at most ~2 points per pixel column."""
    assert len(td) <= 2 * n + 8, (len(td), n)
    sel = (t >= t0) & (t <= t1)
    assert td[0] <= t[sel][0] and td[-1] >= t[sel][-1]       # covers the span
    assert (yd.max(axis=0) >= y[sel].max(axis=0)).all() and (yd.min(axis=0) <= y[sel].min(axis=0)).all()
    if len(td) < np.count_nonzero(sel):                        # decimated: pairs
        for j in np.linspace(0, len(td) // 2 - 1, 25).astype(int):
            a, b = np.searchsorted(t, td[2 * j]), np.searchsorted(t, td[2 * j + 1]) + 1
            assert np.array_equal(yd[2 * j], y[a:b].min(axis=0))
            assert np.array_equal(yd[2 * j + 1], y[a:b].max(axis=0))


@pytest.mark.parametrize("span,n", [((0, 800.0), 200), ((0, 800.0), 1600), ((300.0, 330.0), 200),
                                    ((300.0, 330.0), 1600), ((300.0, 310.0), 3000),
                                    ((123.4, 123.6), 300)])
def test_envelope_store(span, n):
    st = _synthetic_store()
    td, yd = st.envelope(WSK, slice(None), *span, n)
    _check_envelope(st.wsk_t, st.wsk, td, yd, *span, n)
    td2, yd2 = st.envelope(WSK, [1, 3], *span, n)
    assert np.array_equal(td2, td) and np.array_equal(yd2, yd[:, [1, 3]])


def test_envelope_store_aligned():
    """Panning by a fraction of a column keeps the inner columns (no shimmer)."""
    st = _synthetic_store()
    a = st.envelope(WSK, slice(None), 100.0, 400.0, 300)
    b = st.envelope(WSK, slice(None), 100.37, 400.37, 300)
    common = np.intersect1d(a[0][4:-4], b[0][4:-4])
    assert len(common) > 0.9 * len(a[0][4:-4])


@pytest.mark.parametrize("span,n", [((0, 90.0), 200), ((60.0, 90.0), 1600), ((80.0, 82.0), 1600)])
def test_envelope_live(span, n):
    st = _synthetic_store(n=22_500)
    s = ArraySeries({WSK: (st.wsk_t, st.wsk)})
    td, yd = s.envelope(WSK, slice(None), *span, n)
    _check_envelope(st.wsk_t, st.wsk, td, yd, *span, n)


def test_peak_decimate_keeps_extremes():
    t = np.arange(10_001) / 100.0
    y = np.sin(t)[:, None].astype(np.float32)
    y[5003] = 9.0
    td, yd = peak_decimate(t, y, 50)
    assert len(td) <= 2 * (10_001 // 50 + 2) and yd.max() == 9.0 and yd.min() == y.min()


# ------------------------------------------------------------ the windows

@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def line(app):
    from wavi_py.views.line_view import LineView
    v = LineView(2, 4, t_buffer_s=90.0, view_span_s=30.0)
    v.resize(900, 500)
    v.show()
    yield v
    v.close()


def _x(v):
    return tuple(round(x, 3) for x in v._visible_x_range())


def test_line_follows_playhead_unfollows_on_pan(app, line):
    st = _synthetic_store(nch=4)                         # 800 s
    line.set_store(st)
    line.set_playhead(100.0)
    line._render()
    assert _x(line) == (70.0, 100.0)
    assert all(h.isVisible() and h.value() == 100.0 for h in line._head_lines)
    line.set_playhead(10.0)                              # near the start: inside the file
    line._render()
    assert _x(line) == (0.0, 30.0)
    line.set_playhead(500.0)
    line._render()
    assert _x(line) == (470.0, 500.0)
    # a pan (mouse / arrow key) leaves following; the playhead line still moves
    line._pan(-1, 0)
    assert not line.following
    x = _x(line)
    line.set_playhead(510.0)
    line._render()
    assert _x(line) == x and line._head_lines[1].value() == 510.0
    line.go_live()                                       # L / a seek: follow at the span
    line._render()
    assert line.following and _x(line) == (480.0, 510.0)
    # zoom out while following: up to the whole recording
    for _ in range(30):
        line._zoom_x(1.25)
    assert _x(line) == (0.0, round(st.wsk_t[-1], 3))
    t, y = line._plot_data(line._tiles[0], slice(None))
    assert len(t) <= 2 * line._tiles[0].getViewBox().width() + 8
    line._reset_view()                                   # R: the opening span again
    line._render()
    assert _x(line) == (480.0, 510.0)


def test_line_live_opens_at_view_span_zooms_to_buffer(app, line):
    fs = 250.0
    t = np.arange(int(90 * fs)) / fs + 1000.0
    sig = np.random.default_rng(0).normal(size=(len(t), 4)).astype(np.float32)
    line.update_view(t, sig, np.zeros(4, np.float32), 1.0, x_range=(t[-1] - 90.0, t[-1]))
    line._render()
    x0, x1 = line._visible_x_range()
    assert abs(x1 - t[-1]) < 1e-6 and abs(x1 - x0 - 30.0) < 1e-6
    for _ in range(10):
        line._zoom_x(1.25)
    x0, x1 = line._visible_x_range()
    assert abs(x1 - x0 - 90.0) < 1e-6
    tt, _ = line._plot_data(line._tiles[0], slice(None))
    assert len(tt) <= 2 * line._tiles[0].getViewBox().width() + 8


def test_imu_view_store_and_link(app):
    from wavi_py.views.imu_view import ImuView
    st = _synthetic_store()
    v = ImuView(t_buffer_s=90.0, view_span_s=30.0)
    v.show()
    try:
        v.set_store(st)
        v.set_playhead(200.0)
        v._render()
        x0, x1 = v._plots[0].getViewBox().viewRange()[0]
        assert (round(x0, 3), round(x1, 3)) == (170.0, 200.0)
        assert v._head_lines[0].isVisible() and v._head_lines[2].value() == 200.0
        v.set_time_window((0.0, 800.0))                 # linked to the Line window: all of it
        v._render()
        n = len(v._curves[0][1].getData()[0])
        assert n <= 2 * v._plots[0].getViewBox().width() + 8
    finally:
        v.close()


# ------------------------------------------------------------ scrub in the GUI

@pytest.fixture
def win(app, rec, monkeypatch):
    from wavi_py.main_window import MainWindow

    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, replay_speed=1.0,
                   replay_path=str(rec))
    w.show()
    w.open_replay(str(rec))
    t_end = time.monotonic() + 10
    while time.monotonic() < t_end and w._store is None:
        app.processEvents()
        time.sleep(0.01)
    assert w._store is not None
    yield w
    w.close()


def test_scrub_moves_views_without_transport(app, win):
    from wavi_py.views import replay_bar
    w = win
    w._toggle_line.setChecked(True)
    w._toggle_imu.setChecked(True)
    lv, iv = w._line_view, w._imu_view
    lv._zoom_x(0.2)                                      # 2 s span: the view has to move
    app.processEvents()
    seeks = []
    tr = w._client.replay
    orig = tr.seek
    tr.seek = lambda t, pre=None: (seeks.append(t), orig(t, pre))[1]
    m = w._replay
    for i, t in enumerate(np.linspace(7.0, 9.0, 40)):  # a fast drag
        m.request_seek(float(t), False)
        assert lv._head == pytest.approx(t) and iv._head == pytest.approx(t)   # at once
        assert m.position == pytest.approx(t)
        m.poll()                                         # the transport's position: ignored
        assert m.position == pytest.approx(t)
        lv._render()                                     # what the scheduler does next
        assert lv._visible_x_range()[1] == pytest.approx(t)
        assert lv._head_lines[0].value() == pytest.approx(t)
    assert len(seeks) <= 2                               # throttled to SCRUB_S
    m.request_seek(9.0, True)                            # release: the full seek
    assert seeks[-1] == 9.0 and not m._scrubbing
    assert replay_bar.SCRUB_S > 0.05


def test_line_live_no_copy_until_paused(app, line):
    """Live, the Line window reads the caller's buffer (shifted in place each tick, no
    whole-buffer copy); Pause freezes a copy, as the buffer keeps moving under it."""
    t = np.arange(1000) / 250.0
    sig = np.zeros((1000, 4), np.float32)
    line.update_view(t, sig, np.zeros(4, np.float32), 1.0, x_range=(t[-1] - 4.0, t[-1]))
    sig[-1] = 7.0                                        # the next tick's in-place shift
    assert line.series().get(WSK, slice(None), t[-1], t[-1])[1][0, 0] == 7.0
    line.set_paused(True)
    sig[-1] = 9.0
    assert line.series().get(WSK, slice(None), t[-1], t[-1])[1][0, 0] == 7.0
    line.set_paused(False)
