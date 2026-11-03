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
    assert lv._t.size > 0 and abs(lv._t[-1] - t[-1]) < 0.1
    status = lv._stream_status.text()
    assert status.startswith("REPLAY 4x"), status
    # the IMU window: its own times, on the same axis
    assert len(w._imu_t) > 0 and abs(w._imu_t[-1] - t[-1]) < 0.1
    _shot(w, "replay_panel")
    _shot(lv, "replay_line")
    # pause: the playhead holds; the views are not frozen
    w._pause_btn.setChecked(True)
    pos = w._replay.position
    _pump(app, lambda: False, 0.3)
    assert w._replay.state == "paused" and w._replay.position == pos and not w._is_paused
    # seek while paused: the buffers start over at the new position (preroll 2 s)
    w._replay.seek(6.5)
    assert _pump(app, lambda: w._samples_since_clear > 0
                 and w._sig_t[-1] > 6.4, 5)
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
