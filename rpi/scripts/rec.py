#!/usr/bin/env python3
"""Control daq_server's recording from a shell on the Pi (over SSH): no GUI needed.

    python3 ~/whisker_sensor/rpi/scripts/rec.py status
    python3 ~/whisker_sensor/rpi/scripts/rec.py start surge_fwd --duration 60
    python3 ~/whisker_sensor/rpi/scripts/rec.py stop
    python3 ~/whisker_sensor/rpi/scripts/rec.py mark "tap W3"

``start`` refuses while a recording runs (daq_server 0.2 and later) unless
``--force``, which closes the running file and opens a new one. ``stop`` stops
whatever runs; ``stop --path FILE`` only that recording. Exit code 1 when the
server says no, 2 when it can't be reached. Python 3.9 stdlib only.
"""

import argparse
import json
import os
import socket
import sys
import time


def _fmt_s(s) -> str:
    s = int(round(float(s or 0)))
    return f"{s // 3600}:{s // 60 % 60:02d}:{s % 60:02d}" if s >= 3600 else f"{s // 60}:{s % 60:02d}"


def ask(host: str, port: int, cmd: dict, timeout: float = 5.0) -> dict:
    """Send one command and return its ack (the stream in between is skipped)."""
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall((json.dumps(cmd) + "\n").encode())
        buf = b""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            data = s.recv(65536)
            if not data:
                break
            buf += data
            *lines, buf = buf.split(b"\n")
            for line in lines:
                if b'"ack"' not in line:            # cheap skip of the data events
                    continue
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue
                if ev.get("k") == "ack" and ev.get("cmd") == cmd["cmd"]:
                    return ev
    raise TimeoutError(f"no reply to {cmd['cmd']} from {host}:{port}")


def describe(rec: dict) -> str:
    if rec.get("on"):
        out = (f"recording {os.path.basename(rec.get('path') or '?')}  {_fmt_s(rec.get('secs'))}"
               f"  {rec.get('events', 0)} events  by {rec.get('by') or '?'}")
        if rec.get("duration"):
            out += f"  stops in {_fmt_s(rec.get('left'))} (duration {rec['duration']:g} s)"
        out += f"\n  {rec.get('path')}"
    else:
        out = "not recording"
    last = rec.get("last")
    if last:
        t = time.strftime("%H:%M:%S", time.localtime(last.get("unix", 0)))
        out += (f"\nlast: {os.path.basename(last.get('path') or '?')}  {_fmt_s(last.get('secs'))}"
                f"  ended {t}: {last.get('why', '?')}"
                + (f" ({last['by']})" if last.get("by") else ""))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5600)
    sub = ap.add_subparsers(dest="what", required=True)
    sub.add_parser("status", help="is it recording, which file, how long")
    p = sub.add_parser("start", help="start a recording")
    p.add_argument("tag", nargs="?", default="")
    p.add_argument("--duration", type=float, default=None, help="the server stops it after S s")
    p.add_argument("--force", action="store_true", help="replace a running recording")
    p = sub.add_parser("stop", help="stop the recording")
    p.add_argument("--path", default=None, help="only if this file (or name) is the running one")
    p = sub.add_parser("mark", help="put a mark in the stream (and the recording)")
    p.add_argument("text", nargs="+")
    a = ap.parse_args(argv)

    who = f"rec.py@{socket.gethostname()}"
    if a.what == "status":
        cmd = {"cmd": "status"}
    elif a.what == "start":
        cmd = {"cmd": "rec_start", "tag": a.tag, "who": who}
        if a.duration:
            cmd["duration"] = a.duration
        if a.force:
            cmd["force"] = True
    elif a.what == "stop":
        cmd = {"cmd": "rec_stop", "who": who}
        if a.path:
            cmd["path"] = a.path
    else:
        cmd = {"cmd": "mark", "text": " ".join(a.text)}
    try:
        ack = ask(a.host, a.port, cmd)
    except (OSError, TimeoutError) as exc:
        print(f"daq_server at {a.host}:{a.port}: {exc}", file=sys.stderr)
        return 2

    if not ack.get("ok", True):
        print(f"refused: {ack.get('error', '?')}", file=sys.stderr)
        if ack.get("rec"):
            print(describe(ack["rec"]), file=sys.stderr)
        return 1
    if a.what == "status":
        print(describe(ack.get("rec", {})))
    elif a.what == "start":
        print(f"recording -> {ack.get('path')}"
              + (f" (stops after {a.duration:g} s)" if a.duration else ""))
        if ack.get("replaced"):
            print(f"closed {ack['replaced']}")
    elif a.what == "stop":
        print(f"stopped {ack['path']}" if ack.get("path") else "was not recording")
    else:
        print("mark sent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
