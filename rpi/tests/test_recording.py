"""daq_server's recording against clients that come and go: it survives a client
disconnect and a reconnecting client sees it (owner, file, elapsed); a second
rec_start doesn't replace a running recording unless forced; rec_stop with a
path stops only that recording; a rec_start duration stops it on the server
with no client left; rpi/scripts/rec.py drives all of it from a shell.

Run: python -m pytest rpi/tests/test_recording.py
(DAQ_SERVER_PY as in test_service.py runs the server under the Pi's Python.)
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_service import FAKE_ESP32, SERVER, SERVER_PY, Conn, _free_port, _stop  # noqa: E402

REC_PY = ROOT / "rpi" / "scripts" / "rec.py"


@pytest.fixture
def server(tmp_path):
    """daq_server on the fake ESP32 (so events flow), no MAVLink."""
    esp = subprocess.Popen([sys.executable, "-u", str(FAKE_ESP32)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pty = esp.stdout.readline().decode().strip()
    port = _free_port()
    srv = subprocess.Popen([*SERVER_PY, "-u", str(SERVER), "--serial", pty, "--mavlink", "none",
                            "--bind", f"127.0.0.1:{port}", "--rec-dir", str(tmp_path / "rec")],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        c = Conn(port)
        c.wait_for(lambda e: e["k"] == "status" and e["esp32"]["streaming"], 10)
        c.s.close()
        yield port, tmp_path / "rec", srv
    finally:
        out = _stop(srv)
        esp.kill()
        esp.wait()
        if srv.returncode != 0:
            print(out)


def _ack(c: Conn, cmd: dict, secs: float = 3) -> dict:
    c.send(cmd)
    return c.wait_for(lambda e: e["k"] == "ack" and e["cmd"] == cmd["cmd"], secs)


def _lines(path) -> list[dict]:
    return [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]


def test_reconnect_sees_the_running_recording(server):
    port, rec_dir, _ = server
    a = Conn(port)
    ack = _ack(a, {"cmd": "rec_start", "tag": "run1", "who": "wavi_py@laptop"})
    assert ack["ok"] and ack["path"].endswith("_run1.jsonl") and ack["duration"] is None
    a.s.close()                                       # the client crashes
    time.sleep(1.5)
    b = Conn(port)                                    # ... and comes back
    st = b.wait_for(lambda e: e["k"] == "status", 3)
    rec = st["rec"]
    assert rec["on"] and rec["path"] == ack["path"] and rec["secs"] > 1
    assert rec["by"].startswith("wavi_py@laptop (127.0.0.1:") and rec["tag"] == "run1"
    assert rec["left"] is None
    # the status command answers as an ack, with the same rec state
    s = _ack(b, {"cmd": "status"})
    assert s["k"] == "ack" and s["rec"]["path"] == ack["path"]
    stop = _ack(b, {"cmd": "rec_stop", "path": Path(ack["path"]).name})   # the name will do
    assert stop["ok"] and stop["path"] == ack["path"] and not stop["rec"]["on"]
    assert stop["rec"]["last"]["why"] == "stopped by a client"
    lines = _lines(ack["path"])
    assert lines[0]["k"] == "hello" and lines[0]["rec"]["tag"] == "run1"
    assert lines[-1]["k"] == "mark" and lines[-1]["rec"]["event"] == "stop"
    assert sum(e["k"] == "wsk" for e in lines) > 100          # kept recording while away
    assert len(list(rec_dir.glob("*.jsonl"))) == 1


def test_second_client_does_not_rotate_the_file(server):
    port, rec_dir, _ = server
    a, b = Conn(port), Conn(port)
    first = _ack(a, {"cmd": "rec_start", "tag": "laptop", "who": "wavi_py@laptop"})
    refused = _ack(b, {"cmd": "rec_start", "tag": "deck", "who": "wavi_py@deck"})
    assert refused["ok"] is False and refused["busy"] is True
    assert refused["rec"]["path"] == first["path"] and "wavi_py@laptop" in refused["rec"]["by"]
    assert "a recording is running" in refused["error"]
    assert len(list(rec_dir.glob("*.jsonl"))) == 1
    # a stale stop (another file's name) leaves it alone
    stale = _ack(b, {"cmd": "rec_stop", "path": "whisker_20000101_000000_old.jsonl"})
    assert stale["ok"] is False and stale["path"] is None and stale["rec"]["on"]
    time.sleep(1.1)                                   # a new file name (1 s resolution)
    forced = _ack(b, {"cmd": "rec_start", "tag": "deck", "force": True, "who": "wavi_py@deck"})
    assert forced["ok"] and forced["replaced"] == first["path"] and forced["path"] != first["path"]
    st = a.wait_for(lambda e: e["k"] == "status" and e["rec"]["path"] == forced["path"], 3)
    assert st["rec"]["last"]["path"] == first["path"]
    assert st["rec"]["last"]["why"] == "replaced by a new recording"
    assert _lines(first["path"])[-1]["rec"]["why"] == "replaced by a new recording"
    # an old client: rec_stop without a path stops whatever runs
    assert _ack(a, {"cmd": "rec_stop"})["path"] == forced["path"]


def test_duration_stops_on_the_server_with_no_client(server):
    port, _, _ = server
    a = Conn(port)
    ack = _ack(a, {"cmd": "rec_start", "tag": "dur", "duration": 2.0})
    assert ack["ok"] and ack["duration"] == 2.0
    st = a.wait_for(lambda e: e["k"] == "status" and e["rec"]["on"], 3)
    assert st["rec"]["duration"] == 2.0 and 0 < st["rec"]["left"] <= 2.0
    a.s.close()                                       # nobody left to send rec_stop
    time.sleep(3.0)
    b = Conn(port)
    rec = b.wait_for(lambda e: e["k"] == "status", 3)["rec"]
    assert not rec["on"] and rec["last"]["path"] == ack["path"]
    assert rec["last"]["why"] == "duration 2 s elapsed" and rec["last"]["by"] == "server"
    lines = _lines(ack["path"])
    stop = lines[-1]
    assert stop["rec"]["why"] == "duration 2 s elapsed"
    assert 2.0 <= stop["rec"]["secs"] < 2.1           # stopped on the data, not the 1 Hz status
    span = stop["t"] - lines[0]["t"]
    assert 1.95 < span < 2.15, span
    # bad durations: not a number is an error; 0 / negative mean none
    bad = _ack(b, {"cmd": "rec_start", "duration": "soon"})
    assert bad["ok"] is False and "duration" in bad["error"]
    assert _ack(b, {"cmd": "rec_start", "duration": 0})["duration"] is None
    assert _ack(b, {"cmd": "rec_stop"})["ok"]


def _rec(port, *args) -> subprocess.CompletedProcess:
    return subprocess.run([*SERVER_PY, str(REC_PY), "--port", str(port), *args],
                          capture_output=True, text=True, timeout=15)


def test_rec_cli(server):
    port, rec_dir, _ = server
    r = _rec(port, "status")
    assert r.returncode == 0 and "not recording" in r.stdout, r
    r = _rec(port, "start", "cli_run", "--duration", "30")
    assert r.returncode == 0 and "_cli_run.jsonl" in r.stdout and "stops after 30 s" in r.stdout, r
    r = _rec(port, "start", "again")
    assert r.returncode == 1 and "refused" in r.stderr and "_cli_run.jsonl" in r.stderr, r
    r = _rec(port, "status")
    assert r.returncode == 0 and "recording whisker_" in r.stdout and "rec.py@" in r.stdout, r
    assert "stops in" in r.stdout
    r = _rec(port, "mark", "tap", "W3")
    assert r.returncode == 0, r
    r = _rec(port, "stop", "--path", "nope.jsonl")
    assert r.returncode == 1 and "not the running" in r.stderr, r
    r = _rec(port, "stop")
    assert r.returncode == 0 and "stopped" in r.stdout and "_cli_run.jsonl" in r.stdout, r
    r = _rec(port, "status")
    assert "not recording" in r.stdout and "last: whisker_" in r.stdout
    assert "stopped by a client" in r.stdout, r
    r = _rec(port, "stop")
    assert r.returncode == 0 and "was not recording" in r.stdout, r
    (path,) = rec_dir.glob("*_cli_run.jsonl")
    assert any(e["k"] == "mark" and e.get("text") == "tap W3" for e in _lines(path))
    r = _rec(_free_port(), "status")                  # no server there
    assert r.returncode == 2, r
