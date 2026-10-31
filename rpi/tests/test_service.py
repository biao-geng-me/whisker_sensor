"""daq_server as a boot-time service (rpi/scripts/whisker-daq.service): started
before the ESP32 and the MAVLink endpoint are there it keeps serving and picks
both up when they appear; esp32_reset answers with an ack (an error on a pty,
which has no RTS) without dropping the client; SIGTERM exits cleanly.

Run: python -m pytest rpi/tests/test_service.py
The Pi's Python for the server (3.9; the tests stay in this interpreter):
DAQ_SERVER_PY="env PYTHONPATH=<3.9 site> /usr/bin/python3" python -m pytest rpi/tests/test_service.py
"""

from __future__ import annotations

import json
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "rpi"))
from whisker_link.wire import LineSplitter  # noqa: E402

FAKE_ESP32 = ROOT / "rpi" / "sim" / "fake_esp32.py"
FAKE_AP = ROOT / "rpi" / "sim" / "fake_autopilot.py"
SERVER = ROOT / "rpi" / "daq_server.py"
SERVER_PY = shlex.split(os.environ.get("DAQ_SERVER_PY", "")) or [sys.executable]


def _free_port(kind=socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Conn:
    def __init__(self, port: int):
        for _ in range(50):
            try:
                self.s = socket.create_connection(("127.0.0.1", port), timeout=1)
                break
            except OSError:
                time.sleep(0.1)
        else:
            pytest.fail("daq_server did not start")
        self.s.settimeout(0.2)
        self.sp = LineSplitter()

    def send(self, cmd: dict) -> None:
        self.s.sendall((json.dumps(cmd) + "\n").encode())

    def wait_for(self, pred, secs: float) -> dict:
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:
            try:
                data = self.s.recv(65536)
            except socket.timeout:
                continue
            if not data:
                pytest.fail("server closed the connection")
            for ev in self.sp.feed(data):
                if pred(ev):
                    return ev
        pytest.fail("timed out")


def _stop(proc: subprocess.Popen) -> str:
    proc.send_signal(signal.SIGTERM)
    try:
        out = proc.communicate(timeout=8)[0]
    except subprocess.TimeoutExpired:
        proc.kill()
        out = proc.communicate()[0]
        pytest.fail("daq_server did not exit on SIGTERM:\n" + out.decode(errors="replace"))
    return out.decode(errors="replace")


def test_boot_before_esp32_and_mavlink(tmp_path):
    """The service's start at boot: no ESP32 port yet, the MAVLink UDP port
    taken (the router / a stale process): the server serves and retries, then
    picks up both, and SIGTERM ends it with exit code 0."""
    link = tmp_path / "ttyESP"
    tcp, udp = _free_port(), _free_port(socket.SOCK_DGRAM)
    squatter = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    squatter.bind(("0.0.0.0", udp))
    srv = subprocess.Popen([*SERVER_PY, "-u", str(SERVER), "--serial", str(link),
                            "--mavlink", f"udpin:0.0.0.0:{udp}", "--mode", "wide",
                            "--selects", "0123457", "--bind", f"127.0.0.1:{tcp}",
                            "--rec-dir", str(tmp_path / "rec"), "--log-every", "1"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    esp = ap = None
    try:
        c = Conn(tcp)
        st = c.wait_for(lambda e: e["k"] == "status", 3)
        assert not st["esp32"]["streaming"] and st["mav"]["msgs"] == 0
        time.sleep(5)                                   # a few retries of both
        assert srv.poll() is None, "daq_server exited"
        squatter.close()                                # the MAVLink port frees up
        esp = subprocess.Popen([sys.executable, "-u", str(FAKE_ESP32), "--link", str(link)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        st = c.wait_for(lambda e: e["k"] == "status" and e["esp32"]["streaming"], 10)
        assert st["esp32"]["cfg"]["mode"] == 2          # the kick sent --mode / --selects
        ap = subprocess.Popen([sys.executable, "-u", str(FAKE_AP), "--target", f"127.0.0.1:{udp}"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        c.wait_for(lambda e: e["k"] == "mav" and e["type"] == "RAW_IMU", 12)
        c.s.close()
    finally:
        squatter.close()
        out = _stop(srv)
        for p in (esp, ap):
            if p:
                p.kill()
                p.wait()
    assert srv.returncode == 0, out
    # retried, but not a log line every 2 s for the same error
    assert 1 <= out.count("(retrying every 2 s)") <= 2, out
    assert out.count("(retrying every 5 s)") == 1, out
    assert "MAVLink listening" in out and f"opened {link}" in out


def test_esp32_reset_ack_on_a_pty(tmp_path):
    """A pty has no RTS line: the ack says why, and the client stays connected."""
    esp = subprocess.Popen([sys.executable, "-u", str(FAKE_ESP32)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pty = esp.stdout.readline().decode().strip()
    tcp = _free_port()
    srv = subprocess.Popen([*SERVER_PY, "-u", str(SERVER), "--serial", pty, "--mavlink",
                            "none", "--bind", f"127.0.0.1:{tcp}", "--rec-dir", str(tmp_path)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        c = Conn(tcp)
        c.wait_for(lambda e: e["k"] == "status" and e["esp32"]["streaming"], 8)
        c.send({"cmd": "esp32_reset"})
        ack = c.wait_for(lambda e: e["k"] == "ack", 3)
        assert ack["cmd"] == "esp32_reset" and ack["ok"] is False
        assert "RTS pulse failed" in ack["error"], ack
        c.wait_for(lambda e: e["k"] == "wsk", 2)        # still connected and streaming
        c.send({"cmd": "mark", "text": "after"})
        assert c.wait_for(lambda e: e["k"] == "ack", 2)["ok"] is True
        c.s.close()
    finally:
        out = _stop(srv)
        esp.kill()
        esp.wait()
    assert "ESP32 reset from" in out and "failed" in out


class _FakeSerial:
    """Records the modem-line writes of an RTS pulse."""

    def __init__(self, fail: bool = False):
        self.lines: list[tuple[str, bool]] = []
        self._fail = fail

    def __setattr__(self, name, value):
        if name in ("rts", "dtr"):
            self.lines.append((name, value))
            if self._fail and name == "rts" and value:
                raise OSError(25, "Inappropriate ioctl for device")
        object.__setattr__(self, name, value)


def _server_with_fake_serial(ser):
    import daq_server as ds
    args = ds.argparse.Namespace(rec_dir="/nonexistent", serial="/dev/fake", baud=921600,
                                 selects=None, mode=None, mavlink="none", mav_rates={},
                                 sysid=255, compid=191, bind=("127.0.0.1", 0))
    srv = ds.Server(args)
    srv.esp32._ser = ser
    sent, published = [], []
    srv.hub.send_to = lambda c, ev: sent.append(ev)
    srv.hub.publish = published.append
    return srv, sent, published


def test_esp32_reset_pulses_rts_and_marks_the_gap():
    ser = _FakeSerial()
    srv, sent, published = _server_with_fake_serial(ser)
    client = type("C", (), {"addr": ("192.168.2.1", 50000)})()
    try:
        srv.on_command({"cmd": "esp32_reset"}, client)
    finally:
        srv._ls.close()
    assert ser.lines == [("dtr", False), ("rts", True), ("rts", False)]   # EN low, then released
    assert sent == [dict(sent[0], cmd="esp32_reset", ok=True)] and "error" not in sent[0]
    assert published[0]["k"] == "mark" and "ESP32 reset" in published[0]["text"]


def test_esp32_reset_failure_releases_rts():
    ser = _FakeSerial(fail=True)
    srv, sent, published = _server_with_fake_serial(ser)
    client = type("C", (), {"addr": ("127.0.0.1", 1)})()
    try:
        srv.on_command({"cmd": "esp32_reset"}, client)
        srv.esp32._ser = None
        srv.on_command({"cmd": "esp32_reset"}, client)
    finally:
        srv._ls.close()
    assert ser.lines[-1] == ("rts", False)
    assert [a["ok"] for a in sent] == [False, False] and not published
    assert "Inappropriate ioctl" in sent[0]["error"] and "not open" in sent[1]["error"]


def test_silent_port_is_reopened(tmp_path, caplog):
    """An open port that delivers nothing (a hung ESP32, or a handle left stale
    by a USB glitch) is closed and reopened after REOPEN_S; streaming resumes."""
    import logging
    import daq_server as ds
    esp = subprocess.Popen([sys.executable, "-u", str(FAKE_ESP32)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pty = esp.stdout.readline().decode().strip()
    src = ds.Esp32Source(pty, 921600, ds.Hub(str(tmp_path)), ds.RateCounter())
    src.REOPEN_S = 1.5
    caplog.set_level(logging.WARNING, logger="daq_server")
    src.start()
    try:
        def wait(pred, secs):
            t_end = time.monotonic() + secs
            while not pred():
                assert time.monotonic() < t_end, "timed out"
                time.sleep(0.05)
        wait(lambda: src.n_scans > 10, 8)
        esp.send_signal(signal.SIGSTOP)                  # hung: nothing more comes
        wait(lambda: src.n_reopens >= 1, 5)
        assert src.loop_mono > time.monotonic() - 1      # the reader kept looping
        esp.send_signal(signal.SIGCONT)
        n = src.n_scans
        wait(lambda: src.n_scans > n + 10, 8)
    finally:
        src.stop()
        esp.kill()
        esp.wait()
    assert "reopening the port" in caplog.text


def test_stall_watchdog_spots_stuck_and_dead_threads():
    import threading
    srv, _, _ = _server_with_fake_serial(_FakeSerial())
    srv.args.stall_exit = 5.0
    hold = threading.Event()
    try:
        now = time.monotonic()
        assert srv._stalled(now) == []                  # not started yet
        srv.esp32._thread = threading.Thread(target=hold.wait, name="esp32", daemon=True)
        srv.esp32._thread.start()
        srv.esp32.loop_mono = now - 1
        assert srv._stalled(now) == []
        srv.esp32.loop_mono = now - 12
        assert srv._stalled(now) == ["esp32 thread stuck for 12 s"]
        srv.args.stall_exit = 0                         # --stall-exit 0: off
        assert srv._stalled(now) == []
        srv.args.stall_exit = 5.0
        hold.set()
        srv.esp32._thread.join()
        assert srv._stalled(now) == ["esp32 thread died"]
    finally:
        hold.set()
        srv._ls.close()


def test_die_stalled_closes_the_recording_and_exits_nonzero(tmp_path, monkeypatch, capfd):
    import daq_server as ds
    srv, _, _ = _server_with_fake_serial(_FakeSerial())
    srv.hub = ds.Hub(str(tmp_path))
    path = srv.hub.rec_start("t")
    codes = []

    def fake_exit(code):
        codes.append(code)
        raise SystemExit(code)
    monkeypatch.setattr(ds.os, "_exit", fake_exit)
    try:
        with pytest.raises(SystemExit):
            srv._die_stalled(["esp32 thread stuck for 21 s"])
    finally:
        srv._ls.close()
    assert codes == [3] and not srv.hub.recorder.active and Path(path).stat().st_size > 0
    assert "test_die_stalled" in capfd.readouterr().err    # the stacks went to stderr


def test_sigusr1_dumps_stacks_and_keeps_running(tmp_path):
    tcp = _free_port()
    srv = subprocess.Popen([*SERVER_PY, "-u", str(SERVER), "--serial", "none", "--mavlink", "none",
                            "--bind", f"127.0.0.1:{tcp}", "--rec-dir", str(tmp_path)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        Conn(tcp).s.close()                             # up
        srv.send_signal(signal.SIGUSR1)
        time.sleep(0.5)
        assert srv.poll() is None
    finally:
        out = _stop(srv)
    assert srv.returncode == 0, out
    assert "Thread 0x" in out and "_status_loop" in out, out


def test_garbled_text_is_no_sign_of_life():
    """Binary noise that the decoder passes on as Text (it splits at stray
    newlines) must not hold off esp32_hung or the port reopen."""
    import daq_server as ds
    from whisker_link import esp32_frame as ef
    src = ds.Esp32Source("/dev/null", 921600, ds.Hub("/nonexistent"), ds.RateCounter())
    src.hub.publish = lambda ev: None
    noise = ef.Text("�FhbE�~D�Zp��\x0cR�J�0")
    assert ds._garbled(noise) and not ds._garbled(ef.Text("[halt #3] recovered\tok"))
    src._handle(noise, 100.0)
    assert src.faults._last_rx < 0
    src._handle(ef.Text("whisker_daq boot"), 101.0)
    assert src.faults._last_rx == 101.0
