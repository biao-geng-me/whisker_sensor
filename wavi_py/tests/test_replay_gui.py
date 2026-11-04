"""Replay in the GUI (offscreen): open a recording, play, seek, pause, the guards
(no Rec, no scan-mode commands), the shared time axis of the Line and IMU
windows, re-zero at a time. Screenshots go to $WAVI_TEST_SHOTS if set.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_replay_gui.py
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

from wavi_py.daq.series import WSK  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402

SHOTS = os.environ.get("WAVI_TEST_SHOTS")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


def _pump(app, pred, secs: float) -> bool:
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        app.processEvents()
        if pred():
            return True
        time.sleep(0.01)
    app.processEvents()
    return pred()


def _shot(widget, name: str) -> None:
    if SHOTS:
        Path(SHOTS).mkdir(parents=True, exist_ok=True)
        widget.grab().save(str(Path(SHOTS) / f"{name}.png"))


@pytest.fixture
def win(app, tmp_path):
    from wavi_py.main_window import MainWindow
    from wavi_py.scan_mode import parse_mode

    p = tmp_path / "rec.jsonl"
    fx.write_recording(p)
    w = MainWindow(transport="replay", nsensor=16, fs=80.0, t_buffer_s=2.0, replay_speed=4.0,
                   replay_path=str(p), scan_mode=parse_mode("wide"))
    w.show()
    w.open_replay(str(p))
    assert _pump(app, lambda: w._client is not None and w._replay.active, 10)
    sent = []
    tr = w._client.replay
    orig = tr.write
    tr.write = lambda data: (sent.append(data), orig(data))[1]
    w._sent = sent
    yield w
    w.close()


def test_v0_before_play(app, win):
    """Opened paused at 0, nothing played: the panel takes the V0 from the metadata at
    once, so Line (and the Analysis windows) are zeroed before Play."""
    w = win
    w._toggle_line.setChecked(True)
    assert _pump(app, lambda: bool(np.any(w._v0)), 5)
    assert w._replay.state == "paused" and w._replay.position == 0.0
    meta = w._client.bus.get_metadata()
    v0 = np.asarray(meta["V0"], dtype=np.float32)
    assert np.array_equal(w._v0, v0)
    # the case that broke it: the V0 comes in on a tick with no new samples (a real
    # recording opened paused serves none; this synthetic one serves its first scan)
    _pump(app, lambda: False, 0.2)                        # whatever was served: read
    assert w._client.bus.read_since_t(w._last_seq, block=False)[0] is None
    w._v0 = np.zeros_like(w._v0)
    w._last_v0_version = 0
    w._line_view.update_view(w._sig_t[:0], w._sig[:0], w._v0, w._scale)
    w._on_tick()
    lv = w._line_view
    assert np.array_equal(w._v0, v0) and np.array_equal(lv._v0, v0)
    assert w._selections.v0 is w._v0
    # Line draws from the store minus V0: the open-gauge channels aside, around zero
    assert _pump(app, lambda: w._store is not None, 5)
    _pump(app, lambda: False, 0.2)
    _t, y = lv._plot_data(lv._tiles[0], slice(0, 4))
    ok = [c for c in range(4) if c not in fx.RAILED]
    assert len(y) and np.all(np.abs(np.median(y[:, ok], axis=0)) < 1.0)


def test_open_play_seek(app, win):
    w = win
    assert w.windowTitle().endswith("rec.jsonl")
    assert w._connect_btn.text() == "Close"
    assert abs(w._fs - fx.FS) < 1e-6                     # the rate from the index
    assert w._replay.state == "paused" and w._replay.position == 0.0
    w._toggle_line.setChecked(True)
    w._toggle_imu.setChecked(True)
    w._pause_btn.setChecked(False)                       # Resume = play
    assert _pump(app, lambda: w._replay.position > 1.6, 5)
    assert w._replay.state == "playing"
    _pump(app, lambda: False, 0.2)
    n = w._samples_since_clear
    t = w._sig_t[-n:]
    # (the fitted clock puts the first scan a hair before the session zero)
    assert n > 0 and np.all(np.diff(t) > 0) and -0.01 <= t[0] and t[-1] <= w._replay.position + 0.05
    lv = w._line_view
    # the Line window draws the whole recording's store, up to the playhead
    assert _pump(app, lambda: w._store is not None, 5)
    assert lv.series() is w._store and abs(lv._head - w._replay.position) < 0.1
    _pump(app, lambda: False, 0.1)
    x0, x1 = lv._visible_x_range()               # 30 s span > the 10 s file: all of it
    assert abs(x0 - w._store.extent(WSK)[0]) < 0.01 and abs(x1 - w._store.extent(WSK)[1]) < 0.01
    status = lv._stream_status.text()
    assert status.startswith("REPLAY 4x"), status
    # the IMU window: its own times, on the same axis. Not the newest IMU row against the
    # newest scan: the fixture's MAVLink block written 0.2 s late (BACKSTEP_AT) puts the
    # IMU up to 0.2 s behind while the playhead is in it, wherever a loaded run lands.
    # Instead, every IMU row's time is exactly the store's for that row (the store's times
    # are the pipeline's, test_replay_store), and the store's IMU and whisker axes agree.
    it = w._imu_t.copy()
    assert len(it) > 0
    j0 = int(np.searchsorted(w._store.imu_t, it[0]))
    assert np.array_equal(w._store.imu_t[j0:j0 + len(it)], it)
    assert abs(w._store.imu_t[-1] - w._store.wsk_t[-1]) < 0.1     # the same session axis
    assert abs(w._store.imu_t[0] - w._store.wsk_t[0]) < 0.1
    _shot(w, "replay_panel")
    _shot(lv, "replay_line")
    # pause: the playhead holds; the views are not frozen
    w._pause_btn.setChecked(True)
    pos = w._replay.position
    _pump(app, lambda: False, 0.3)
    assert w._replay.state == "paused" and w._replay.position == pos and not w._is_paused
    # seek while paused: the buffers start over at the new position (preroll 2 s)
    # (wait for this seek to be handled, its "live" mark in: if a slow run let playback
    # pass 6.4 s before the pause, the old buffer alone would meet "> 6.4")
    gen = w._replay_gen
    w._replay.seek(6.5)
    assert _pump(app, lambda: w._replay_gen > gen and w._replay_seek_at is None
                 and w._samples_since_clear > 0 and w._sig_t[-1] > 6.4, 5)
    n = w._samples_since_clear
    assert w._sig_t[-n:].min() > 6.5 - 2.0 - 0.05           # nothing from before the seek
    assert w._replay.state == "paused"
    assert len(w._imu_t) and w._imu_t.min() > 6.5 - 2.0 - 0.05
    assert "seek to 0:06.5" in w._fault_log.text()
    # go to a depth: the next crossing of 20 cm going down is ~3.3 s
    assert w._replay.goto("0") is None
    assert w._replay.goto("20cm") is None
    assert abs(w._replay.position - (2.5 + 5 / 6)) < 0.15
    assert w._replay.goto("max") is None and abs(w._replay.position - 5.0) < 0.15
    assert w._replay.goto("nonsense") is not None


def test_guards(app, win):
    w = win
    assert not w._record_btn.isEnabled()
    assert not w._mode_combo.isEnabled() and not w._esp32_reset_btn.isEnabled()
    w._start_recording()
    assert not w._is_recording
    w._pause_btn.setChecked(False)
    assert _pump(app, lambda: w._replay.position > 4.0, 5)   # the INFO at 3.5 s went by
    w._on_scan_mode_changed(force=True)
    _pump(app, lambda: False, 0.1)
    assert not any(b'"esp32"' in d for d in w._sent), w._sent
    assert w._keeper.state != "failed"
    assert w._mode_label.text().startswith("recorded wide"), w._mode_label.text()


def test_app_args():
    from wavi_py.app import _parse_args

    a = _parse_args(["--replay", "x.jsonl", "--speed", "max", "--start", "1:02.5"])
    assert (a.transport, a.replay, a.speed, a.start, a.nsensor) == ("replay", "x.jsonl", None,
                                                                    62.5, 16)
    assert _parse_args([]).transport == "tcp"
    with pytest.raises(SystemExit):
        _parse_args(["--replay", "x.jsonl", "--transport", "server"])


def test_end_and_zero_at(app, win):
    w = win
    w._toggle_line.setChecked(True)
    w._replay.set_speed(None)
    w._pause_btn.setChecked(False)
    assert _pump(app, lambda: w._replay.state == "eof", 10)
    _pump(app, lambda: False, 0.3)
    assert w._client.is_running and w._pause_btn.isChecked()       # EOF is not a lost link
    t_end = float(w._sig_t[-1])
    assert t_end > fx.DUR_S - 0.2
    # re-zero at a time: V0 = the mean of the 0.25 s around it
    n = w._samples_since_clear
    tz = t_end - 1.0
    sel = np.abs(w._sig_t[-n:] - tz) <= 0.125
    assert w.zero_at(tz)
    assert np.allclose(w._v0, np.nanmean(w._sig[-n:][sel], axis=0))
    assert not w.zero_at(t_end + 100)
    # Play after the end starts over
    w._replay.set_speed(1.0)
    w._pause_btn.setChecked(False)
    assert _pump(app, lambda: w._replay.state == "playing" and w._replay.position < 1.0, 5)
    # close
    w._on_connect_clicked()
    assert w._client is None and not w._replay.active and w._connect_btn.text() == "Open"


def test_dome_follows_seek_and_zero(app, win):
    """The Dome (built without GL here: nothing painted) skips the preroll of a
    seek, and a re-zero at a time tares it to that V0."""
    pytest.importorskip("OpenGL")
    from wavi_py.views.dome_view import DomeView

    w = win
    dv = DomeView(fs=w._fs)
    w._dome_view = dv
    dv.set_bus(w._client.bus, w._fs)
    try:
        w._replay.seek(7.0)
        assert _pump(app, lambda: w._replay_gen >= 2 and w._replay_seek_at is None, 5)
        live = [m for m in w._client.bus.get_metadata()["replay_marks"]
                if m[0] == w._replay_gen and m[1] == "live"][0]
        assert dv._cursor >= live[2]
        assert w.zero_at(6.5)
        assert np.allclose(dv.pose.tare.baseline[np.isfinite(dv.pose.tare.baseline)],
                           w._v0[np.isfinite(dv.pose.tare.baseline)])
        assert dv.pose.tare.ready
    finally:
        w._dome_view = None
        dv.set_bus(None)
        dv.close()


def test_timeline(app, win):
    """The timeline docks: events list, click / scrub seeks, loop, panel keys."""
    from PyQt6.QtCore import Qt

    from wavi_py.views.replay_bar import EVENT_LEAD_S, TimelineBar

    w = win
    w._toggle_line.setChecked(True)
    bars = [d.widget() for d in w._timeline_docks]
    assert len(bars) == 6 and all(isinstance(b, TimelineBar) for b in bars)
    assert w._timeline_docks[0].isVisible()
    assert not w._line_view.timeline_dock.isVisible()      # windows: hidden by default
    w._line_view.timeline_dock.toggleViewAction().trigger()
    assert w._line_view.timeline_dock.isVisible()
    bar = w._line_view.timeline_dock.widget()
    evs = w._index.events
    assert bar.events.count() == len(evs) + 1 and bar.controls.isVisible()
    assert len(bar.strip.spans) == 1 and bar.strip.ticks
    # pick the fault event: 2 s before it
    i = next(k for k, e in enumerate(evs) if e.kind == "fault")
    bar.events.activated.emit(i + 1)
    assert abs(w._replay.position - (evs[i].t - EVENT_LEAD_S)) < 1e-6
    # every bar's playhead follows
    _pump(app, lambda: False, 0.1)
    assert all(abs(b.strip.head.value() - w._replay.position) < 1e-6 for b in bars)
    # next / previous event walk the list
    p0 = w._replay.position
    w._replay.event_step(True)
    nxt = next(e for e in evs if e.t > p0 + EVENT_LEAD_S + 0.5)
    assert abs(w._replay.position - (nxt.t - EVENT_LEAD_S)) < 1e-6
    w._replay.event_step(False)
    assert abs(w._replay.position - p0) < 1e-6
    # scrub: drags are rate-limited, the release seeks fully
    gen = w._client.replay.gen
    for x in (1.0, 1.1, 1.2):
        w._replay.request_seek(x, False)
    assert w._client.replay.gen == gen + 1
    w._replay.request_seek(2.0, True)
    assert w._client.replay.gen == gen + 2 and w._replay.position == 2.0
    # panel keys (offscreen windows are never active: fire the shortcuts directly)
    from PyQt6.QtGui import QKeySequence, QShortcut

    keys = {sc.key().toString(): sc for sc in w.findChildren(QShortcut)}

    def press(k):
        keys[QKeySequence(k).toString()].activated.emit()
    press(Qt.Key.Key_Right)
    assert abs(w._replay.position - 7.0) < 1e-6
    press("Shift+Left")
    assert abs(w._replay.position - 6.0) < 1e-6
    press("]")
    assert w._replay.speed == 8.0
    # loop: around the playhead, and playback wraps inside it
    w._replay.seek(3.0)
    bar.loop_btn.setChecked(True)
    assert w._replay.loop == (0.0, 8.0) and w._client.replay.loop == (0.0, 8.0)
    assert all(b.strip.loop.isVisible() and b.loop_btn.isChecked() for b in bars)
    w._replay.set_loop((3.0, 4.0))
    gen = w._client.replay.gen
    w._pause_btn.setChecked(False)
    assert _pump(app, lambda: w._client.replay.gen >= gen + 2, 5)    # wrapped twice
    assert 3.0 - 1e-6 <= w._replay.position <= 4.0 + 1e-6
    w._replay.set_loop(None)
    assert not bars[0].loop_btn.isChecked() and w._client.replay.loop is None
    w._pause_btn.setChecked(True)
    _shot(w, "timeline_panel")
    _shot(w._line_view, "timeline_line")
    # Zero at... in the Line window
    w._replay.seek(5.0)
    assert _pump(app, lambda: w._samples_since_clear > 0 and w._sig_t[-1] > 4.9, 5)
    v0 = w._v0.copy()
    w._line_view._act_zero_at.setChecked(True)
    w._line_view.zeroAtRequested.emit(4.0)
    assert not np.allclose(w._v0, v0)


def test_live_timeline_and_buffer_seek(app, win):
    """The live source: the reader's tracker in session seconds; a timeline pick
    shows that time from the buffer."""
    from wavi_py.replay.events import EventTracker, LiveTimeline

    tr = EventTracker()
    tr.feed({"k": "mav", "t": 105.0, "type": "HEARTBEAT", "src": [1, 1],
             "d": {"base_mode": 209, "custom_mode": 19}})
    tr.feed({"k": "mav", "t": 106.0, "type": "STATUSTEXT", "src": [1, 1],
             "d": {"severity": 4, "text": "hi"}})
    lt = LiveTimeline(tr, lambda: 100.0, lambda: 110.0, lambda: 1.79e9)
    assert lt.duration == 10.0
    assert [(e.kind, e.t) for e in lt.events] == [("arm", 5.0), ("mode", 5.0),
                                                  ("statustext", 6.0)]
    assert lt.armed_spans == [[5.0, 10.0]] and lt.armed_at(7.0)
    assert lt.event_near(5.4, True).text == "hi"
    w = win
    w._toggle_line.setChecked(True)
    w._replay.seek(6.0)
    assert _pump(app, lambda: w._samples_since_clear > 0 and w._sig_t[-1] > 5.9, 5)
    w._show_time_from_buffer(5.0)
    x0, x1 = w._line_view._visible_x_range()
    assert abs((x0 + x1) / 2 - 5.0) < 0.05
    w._show_time_from_buffer(100.0)
    assert "not in the buffer" in w._path_label.text()
