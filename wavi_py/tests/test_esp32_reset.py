"""The Reset ESP32 control: confirm, send esp32_reset, show the ack; off when
not connected. Sim transport (it reboots) and the Server transport against
daq_server + rpi/sim/fake_esp32.py (a pty has no RTS: the error is shown).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_esp32_reset.py
"""

from __future__ import annotations

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


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    """A modal error box would block the test forever: fail instead."""
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


def _pump(app, pred, secs: float) -> bool:
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        app.processEvents()
        if pred():
            return True
        time.sleep(0.02)
    return False


def _answer(monkeypatch, button):
    asked = []

    def question(parent, title, text, *a, **k):
        asked.append(text)
        return button
    monkeypatch.setattr(QMessageBox, "question", question)
    return asked


def test_reset_on_sim(app, monkeypatch, tmp_path):
    from wavi_py.main_window import MainWindow
    mw = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    try:
        assert not mw._esp32_reset_btn.isEnabled()          # not connected
        mw._connect()
        assert mw._esp32_reset_btn.isEnabled()
        tr = mw._client._transport
        assert _pump(app, lambda: mw._client.bus.total_published > 50, 5)

        sent = []
        orig = mw._client.send_command
        monkeypatch.setattr(mw._client, "send_command", lambda c: sent.append(c) or orig(c))
        asked = _answer(monkeypatch, QMessageBox.StandardButton.No)
        mw._esp32_reset_btn.click()
        assert asked and "Streaming stops" in asked[0] and not sent    # declined: nothing sent

        boot_t = tr._boot_t
        _answer(monkeypatch, QMessageBox.StandardButton.Yes)
        mw._esp32_reset_btn.click()
        assert sent == [{"cmd": "esp32_reset"}]
        assert _pump(app, lambda: mw._path_label.text().startswith("ESP32 reset: EN pulsed"), 3), \
            mw._path_label.text()
        assert _pump(app, lambda: tr._boot_t != boot_t, 2)  # the simulated ESP32 rebooted
        # it streams again (the keeper / reader kick it) and the reboot was expected
        n = mw._client.bus.total_published
        assert _pump(app, lambda: mw._client.bus.total_published > n + 50, 8)
        assert mw._fault_state.get("level", "ok") == "ok", mw._fault_state

        mw._disconnect()
        assert not mw._esp32_reset_btn.isEnabled()
    finally:
        mw._disconnect()
        mw.deleteLater()


def test_reset_warns_while_recording(app, monkeypatch, tmp_path):
    from wavi_py.main_window import MainWindow
    mw = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    try:
        mw._connect()
        assert _pump(app, lambda: mw._client.bus.total_published > 20, 5)
        mw._is_recording = True                           # only the dialog text is checked
        asked = _answer(monkeypatch, QMessageBox.StandardButton.No)
        mw._esp32_reset_btn.click()
        assert "recording keeps running and gets a gap" in asked[0]
        mw._is_recording = False
    finally:
        mw._disconnect()
        mw.deleteLater()


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_reset_over_the_server_shows_the_ack(app, monkeypatch, tmp_path):
    from wavi_py.main_window import MainWindow
    esp = subprocess.Popen([sys.executable, "-u", str(ROOT / "rpi" / "sim" / "fake_esp32.py")],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pty = esp.stdout.readline().decode().strip()
    port = _free_port()
    srv = subprocess.Popen([sys.executable, "-u", str(ROOT / "rpi" / "daq_server.py"),
                            "--serial", pty, "--mavlink", "none",
                            "--bind", f"127.0.0.1:{port}", "--rec-dir", str(tmp_path / "rec")],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    mw = None
    try:
        for _ in range(50):
            try:
                socket.create_connection(("127.0.0.1", port), timeout=1).close()
                break
            except OSError:
                time.sleep(0.1)
        mw = MainWindow(transport="server", host="127.0.0.1", port=port, nsensor=16,
                        outpath=tmp_path)
        mw._connect()
        assert mw._esp32_reset_btn.isEnabled()
        assert _pump(app, lambda: mw._client.bus.total_published > 50, 8)
        _answer(monkeypatch, QMessageBox.StandardButton.Yes)
        mw._esp32_reset_btn.click()
        # fake_esp32 is a pty: no RTS line, so daq_server's ack carries the reason
        assert _pump(app, lambda: mw._path_label.text().startswith("ESP32 reset failed"), 3), \
            mw._path_label.text()
        assert "RTS pulse failed" in mw._path_label.text()
        assert mw._client.is_running                      # the connection survived
    finally:
        if mw is not None:
            mw._disconnect()
            mw.deleteLater()
        srv.send_signal(signal.SIGTERM)
        srv.wait(timeout=8)
        esp.kill()
        esp.wait()
