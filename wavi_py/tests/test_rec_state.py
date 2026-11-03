"""The Rec button follows the vehicle's recording (daq_server + fake ESP32, GUI
offscreen): a recording this window didn't start (another client's, or its own
before a crash) is adopted from the status record and the button only stops it;
a rec_start the server refuses (another client started one) asks and doesn't
rotate the file; auto-stop sends a duration and the server's stop ends the local
recording too, before the client's own fallback.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_rec_state.py
"""

from __future__ import annotations

import json
import math
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from whisker_link.wire import LineSplitter  # noqa: E402


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


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _ask(port: int, cmd: dict) -> dict:
    """Another client: one command, its ack."""
    with socket.create_connection(("127.0.0.1", port), timeout=3) as s:
        s.sendall((json.dumps(cmd) + "\n").encode())
        sp = LineSplitter()
        t_end = time.monotonic() + 3
        while time.monotonic() < t_end:
            for ev in sp.feed(s.recv(65536)):
                if ev.get("k") == "ack" and ev.get("cmd") == cmd["cmd"]:
                    return ev
    raise AssertionError(f"no ack for {cmd}")


@pytest.fixture
def rig(app, tmp_path):
    from wavi_py.main_window import MainWindow
    esp = subprocess.Popen([sys.executable, "-u", str(ROOT / "rpi" / "sim" / "fake_esp32.py")],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pty = esp.stdout.readline().decode().strip()
    port = _free_port()
    srv = subprocess.Popen([sys.executable, "-u", str(ROOT / "rpi" / "daq_server.py"),
                            "--serial", pty, "--mavlink", "none",
                            "--bind", f"127.0.0.1:{port}", "--rec-dir", str(tmp_path / "rec")],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    mws = []

    def window():
        mw = MainWindow(transport="server", host="127.0.0.1", port=port, nsensor=16,
                        outpath=tmp_path / f"laptop{len(mws)}")
        mws.append(mw)
        mw._connect()
        assert _pump(app, lambda: mw._last_status_rec, 5), "no status record"
        return mw
    try:
        for _ in range(50):
            try:
                socket.create_connection(("127.0.0.1", port), timeout=1).close()
                break
            except OSError:
                time.sleep(0.1)
        yield port, tmp_path, window
    finally:
        for mw in mws:
            mw._disconnect()
            mw.deleteLater()
        srv.send_signal(signal.SIGTERM)
        srv.wait(timeout=8)
        esp.kill()
        esp.wait()


def _rov_files(tmp_path) -> list[Path]:
    return sorted((tmp_path / "rec").glob("whisker_*.jsonl"))


def test_adopts_a_recording_it_did_not_start(app, rig, monkeypatch):
    port, tmp_path, window = rig
    ack = _ask(port, {"cmd": "rec_start", "tag": "before_crash", "who": "wavi_py@laptop"})
    assert ack["ok"]
    mw = window()                                     # reconnects after a crash
    assert mw._foreign_rec is not None and mw._foreign_rec["path"] == ack["path"]
    assert mw._record_btn.text().startswith("■ Stop ROV rec") and mw._record_btn.isEnabled()
    assert "wavi_py@laptop" in mw._record_btn.toolTip()
    assert "(not ours)" in mw._path_label.text() and "before_crash" in mw._path_label.text()
    assert not mw._is_recording                       # no local tee for it
    sent = []
    real = mw._client.send_command
    monkeypatch.setattr(mw._client, "send_command", lambda c: (sent.append(c), real(c))[1])
    asked = []
    monkeypatch.setattr(QMessageBox, "question",
                        lambda *a, **k: (asked.append(a[2]), QMessageBox.StandardButton.No)[1])
    mw._record_btn.click()                            # No: nothing sent
    assert asked and "didn't start it" in asked[0] and not sent
    monkeypatch.setattr(QMessageBox, "question", lambda *a, **k: QMessageBox.StandardButton.Yes)
    mw._record_btn.click()
    assert [c["cmd"] for c in sent] == ["rec_stop"] and sent[0]["path"] == ack["path"]
    assert _pump(app, lambda: mw._record_btn.text() == "● Rec" and mw._rec_cmd is None, 4)
    assert "ROV recording stopped" in mw._path_label.text()
    time.sleep(1.2)                                   # a status or two later: still off
    assert _pump(app, lambda: not mw._last_status_rec["rec"]["on"], 2)
    assert mw._record_btn.text() == "● Rec" and mw._foreign_rec is None
    assert _rov_files(tmp_path) == [Path(ack["path"])]   # never a rec_start over it
    assert not list((tmp_path / "laptop0").glob("*"))


def test_refused_start_asks_and_keeps_the_running_file(app, rig, monkeypatch):
    port, tmp_path, window = rig
    mw = window()
    assert mw._record_btn.text() == "● Rec"
    # the other client starts one right before this window's Rec (no status in between)
    other = _ask(port, {"cmd": "rec_start", "tag": "deck", "who": "wavi_py@deck"})
    asked = []
    monkeypatch.setattr(mw, "_ask_replace", lambda rec, tag: asked.append((rec, tag)))
    mw._tag_edit.setText("laptop")
    mw._record_btn.click()
    assert mw._rec_cmd is not None and not mw._record_btn.isEnabled()   # waits for the ack
    assert _pump(app, lambda: asked, 4)
    assert asked[0][0]["path"] == other["path"] and asked[0][1] == "laptop"
    assert not mw._is_recording and mw._foreign_rec["path"] == other["path"]
    assert mw._record_btn.text().startswith("■ Stop ROV rec")
    assert _rov_files(tmp_path) == [Path(other["path"])]
    assert not list((tmp_path / "laptop0").glob("*"))  # no local files for a refused start
    # Replace (the dialog's choice) forces it
    time.sleep(1.1)                                   # a new file name
    mw._foreign_rec = None
    mw._start_recording(force=True)
    assert _pump(app, lambda: mw._is_recording, 4)
    assert mw._rov_path != other["path"] and len(_rov_files(tmp_path)) == 2
    mw._record_btn.click()                            # ours: plain stop, no question
    assert _pump(app, lambda: mw._rec_cmd is None and not mw._is_recording, 4)
    assert _pump(app, lambda: not mw._last_status_rec["rec"]["on"], 3)
    assert mw._record_btn.text() == "● Rec"


def test_auto_stop_runs_on_the_server(app, rig, monkeypatch):
    port, tmp_path, window = rig
    mw = window()
    sent = []
    real = mw._client.send_command
    monkeypatch.setattr(mw._client, "send_command", lambda c: (sent.append(c), real(c))[1])
    mw._set_auto_stop(5)
    mw._tag_edit.setText("auto")
    t0 = time.monotonic()
    mw._record_btn.click()
    assert _pump(app, lambda: mw._is_recording, 4)
    assert sent[0]["cmd"] == "rec_start" and sent[0]["duration"] == 5
    assert mw._rov_duration
    assert _pump(app, lambda: not mw._is_recording, 9)
    took = time.monotonic() - t0
    # the server's stop, seen in the next status (not the client's fallback at 5 + 3 s)
    assert 5 <= took < 7.5, took
    assert [c["cmd"] for c in sent] == ["rec_start"]  # the client sent no rec_stop
    assert "stopped by the server: duration 5 s elapsed" in mw._path_label.text()
    assert mw._record_btn.text() == "● Rec"
    (dat,) = (tmp_path / "laptop0").glob("*_auto.dat")
    local = dat.with_suffix(".jsonl")
    n = local.stat().st_size
    assert n > 0
    _pump(app, lambda: False, 1.0)
    assert local.stat().st_size == n                  # the local tee is closed too
    (rov,) = _rov_files(tmp_path)
    assert json.loads(rov.read_text().splitlines()[-1])["rec"]["why"] == "duration 5 s elapsed"


def test_old_client_path_without_a_server_ack(app, tmp_path):
    """Sim transport (no vehicle): Rec starts at once, as before."""
    from wavi_py.main_window import MainWindow
    mw = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    try:
        mw._connect()
        assert _pump(app, lambda: mw._client.bus.total_published > 20, 5)
        mw._start_recording()
        assert mw._is_recording and mw._rec_cmd is None
        mw._stop_recording()
        assert not mw._is_recording and mw._record_btn.text() == "● Rec"
        assert list(tmp_path.glob("*.dat"))
    finally:
        mw._disconnect()
        mw.deleteLater()


def test_auto_stop_toggle_and_seconds_box(app, tmp_path):
    """Auto-stop is a toggle plus a whole-seconds box: the box keeps its time while
    off, typing a time (or an arrow step) turns it on, as with the FFT colour range."""
    from wavi_py.config import MIN_REC_LEN_S
    from wavi_py.main_window import MainWindow
    from PyQt6.QtWidgets import QSpinBox
    mw = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    try:
        chk, box = mw._auto_stop_chk, mw._auto_stop_spin
        assert isinstance(box, QSpinBox) and box.suffix() == " s" and box.singleStep() == 5
        assert box.minimum() == math.ceil(MIN_REC_LEN_S) and not box.keyboardTracking()
        assert not chk.isChecked() and box.value() == 60 and mw._auto_stop_s == 0
        chk.setChecked(True)
        assert mw._auto_stop_s == 60
        chk.setChecked(False)
        assert mw._auto_stop_s == 0 and box.value() == 60          # the box keeps its time
        box.setValue(90)                                           # typed + Enter / arrows
        assert chk.isChecked() and mw._auto_stop_s == 90
        box.stepUp()
        assert mw._auto_stop_s == 95
        box.setValue(1)                                            # below the minimum
        assert box.value() == math.ceil(MIN_REC_LEN_S) and mw._auto_stop_s == box.value()
        mw._set_auto_stop(0)
        assert not chk.isChecked() and mw._auto_stop_s == 0 and box.value() == 5
        mw._set_auto_stop(12.5)
        assert chk.isChecked() and box.value() == 13 and mw._auto_stop_s == 13
        # Sim: no server duration, so the controls stay editable while recording
        mw._connect()
        assert _pump(app, lambda: mw._client.bus.total_published > 20, 5)
        mw._start_recording()
        assert mw._is_recording and box.isEnabled() and chk.isEnabled()
        mw._stop_recording()
    finally:
        mw._disconnect()
        mw.deleteLater()


def test_auto_stop_is_fixed_while_the_server_holds_it(app, rig):
    port, tmp_path, window = rig
    mw = window()
    mw._set_auto_stop(30)
    mw._record_btn.click()
    assert _pump(app, lambda: mw._is_recording, 4)
    assert not mw._auto_stop_spin.isEnabled() and not mw._auto_stop_chk.isEnabled()
    assert "Fixed while" in mw._auto_stop_chk.toolTip()
    mw._record_btn.click()
    assert _pump(app, lambda: not mw._is_recording and mw._rec_cmd is None, 4)
    assert mw._auto_stop_spin.isEnabled() and mw._auto_stop_spin.value() == 30


def test_mark_last_recording_bad(app, rig, monkeypatch):
    port, tmp_path, window = rig
    mw = window()
    assert not mw._flag_btn.isEnabled()               # no recording yet
    mw._tag_edit.setText("flagme")
    mw._record_btn.click()
    assert _pump(app, lambda: mw._is_recording, 4)
    mw._record_btn.click()
    assert _pump(app, lambda: not mw._is_recording and mw._rec_cmd is None, 4)
    assert _pump(app, lambda: mw._flag_btn.isEnabled(), 3)
    (rov,) = _rov_files(tmp_path)
    assert mw._flag_btn.text() == "Mark bad" and rov.name in mw._flag_btn.toolTip()
    from PyQt6.QtWidgets import QInputDialog
    prompts = []
    monkeypatch.setattr(QInputDialog, "getText",
                        lambda *a, **k: (prompts.append(a[2]), ("drifted", True))[1])
    mw._flag_btn.click()
    assert rov.name in prompts[0]
    assert _pump(app, lambda: "marked bad" in mw._path_label.text(), 4), mw._path_label.text()
    assert _pump(app, lambda: mw._flag_btn.text() == "Unmark bad", 3)
    from whisker_link import rec_index
    r = rec_index.read(str(tmp_path / "rec" / "trials.jsonl"))[rov.name]
    assert r["keep"] is False and r["note"] == "drifted" and r["how"] == "user"
    monkeypatch.setattr(QMessageBox, "question", lambda *a, **k: QMessageBox.StandardButton.Yes)
    mw._flag_btn.click()
    assert _pump(app, lambda: mw._flag_btn.text() == "Mark bad", 4)
    assert rec_index.read(str(tmp_path / "rec" / "trials.jsonl"))[rov.name]["keep"] is True
    assert rov.exists()
