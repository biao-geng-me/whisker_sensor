"""Vehicle-side whisker + IMU server for the BlueROV2 (runs on the BlueOS Pi).

Successor to ``daq_bridge.py`` for the ESP32 whisker DAQ. Where the bridge
pumps raw bytes, the server understands both data sources, puts them on one
clock and records them together:

    ESP32 (USB serial, binary frames) ──┐
                                        ├─► stamp with Pi monotonic clock ─┬─► recording (.jsonl on the Pi)
    ArduSub (MAVLink, local endpoint) ──┘                                  └─► TCP fan-out (JSON lines)
                                                                                 wavi_py, Cockpit page, ...

The event / recording format is ``whisker_link/wire.py``; the ESP32 frame
format is ``whisker_link/esp32_frame.py``.

MAVLink: add an endpoint in BlueOS (Pirate mode -> MAVLink Endpoints):
type "UDP Client", IP 127.0.0.1, port 14670. The server listens there and
asks the autopilot for the messages in ``DEFAULT_MAV_RATES``, re-sending the
request every few seconds because a GCS (Cockpit/QGC) may override rates.

Run from the repo root on the Pi:

    python3 -u rpi/daq_server.py --serial /dev/ttyUSB0 --rec-dir ~/whisker_rec

Then on the laptop: ``python -m wavi_py --transport server --host 192.168.2.2``
or just look at it: ``ncat 192.168.2.2 5600``.
"""

from __future__ import annotations

import argparse
import logging
import os
import queue
import signal
import socket
import sys
import threading
import time
from datetime import datetime
from typing import Callable, Optional

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from whisker_link import esp32_frame as ef  # noqa: E402
from whisker_link import wire  # noqa: E402
from whisker_link.channels import CHANNEL_NAMES  # noqa: E402

log = logging.getLogger("daq_server")

SERVER_VERSION = "0.1"
DEFAULT_SERIAL_BAUD = 921600
DEFAULT_MAVLINK = "udpin:0.0.0.0:14670"

# MAVLink message -> requested rate (Hz). ArduSub on Navigator runs its main
# loop at 400 Hz; what it can actually stream is measured in the status line.
DEFAULT_MAV_RATES = {
    "RAW_IMU": 100,            # IMU0 accel (mg) / gyro (mrad/s) / mag
    "SCALED_IMU2": 100,        # IMU1
    "ATTITUDE": 50,            # roll/pitch/yaw + rates (EKF)
    "ATTITUDE_QUATERNION": 50,
    "SCALED_PRESSURE2": 20,    # Bar30 depth sensor on ArduSub
    "SERVO_OUTPUT_RAW": 20,    # thruster PWM — likely matters for the whiskers
    "LOCAL_POSITION_NED": 10,
    "SYSTEM_TIME": 1,
}


# ─────────────────────────────────────────────────────────────────────────────
#  Hub: one ordered event stream -> recorder + clients
# ─────────────────────────────────────────────────────────────────────────────

class Recorder:
    def __init__(self, rec_dir: str):
        self.rec_dir = os.path.expanduser(rec_dir)
        self._f = None
        self.path: Optional[str] = None
        self.started_mono: Optional[float] = None
        self.n_events = 0
        self._last_flush = 0.0

    @property
    def active(self) -> bool:
        return self._f is not None

    def start(self, tag: str, hello: dict) -> str:
        self.stop()
        os.makedirs(self.rec_dir, exist_ok=True)
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in tag)[:40]
        name = f"whisker_{datetime.now():%Y%m%d_%H%M%S}" + (f"_{safe}" if safe else "") + ".jsonl"
        self.path = os.path.join(self.rec_dir, name)
        self._f = open(self.path, "wb", buffering=1 << 16)
        self.started_mono = time.monotonic()
        self.n_events = 0
        self.write(wire.dumps(hello))
        return self.path

    def write(self, line: bytes) -> None:
        if self._f is None:
            return
        self._f.write(line)
        self.n_events += 1
        now = time.monotonic()
        if now - self._last_flush > 1.0:       # bound loss on power cut to ~1 s
            self._f.flush()
            self._last_flush = now

    def stop(self) -> Optional[str]:
        if self._f is None:
            return None
        self._f.flush()
        try:
            os.fsync(self._f.fileno())
        except OSError:
            pass
        self._f.close()
        self._f = None
        return self.path


class _Client:
    def __init__(self, conn: socket.socket, addr, on_command: Callable, on_close: Callable,
                 queue_max: int = 4096):
        self.conn = conn
        self.addr = addr
        self._on_command = on_command
        self._on_close = on_close
        self.q: queue.Queue = queue.Queue(maxsize=queue_max)
        self._closed = threading.Event()
        threading.Thread(target=self._writer, name=f"cl-{addr}-w", daemon=True).start()
        threading.Thread(target=self._reader, name=f"cl-{addr}-r", daemon=True).start()

    def enqueue(self, line: bytes) -> bool:
        try:
            self.q.put_nowait(line)
            return True
        except queue.Full:
            return False

    def _writer(self) -> None:
        try:
            while not self._closed.is_set():
                try:
                    first = self.q.get(timeout=0.5)
                except queue.Empty:
                    continue
                chunks = [first]
                while len(chunks) < 256:          # coalesce into fewer sends
                    try:
                        chunks.append(self.q.get_nowait())
                    except queue.Empty:
                        break
                self.conn.sendall(b"".join(chunks))
        except OSError as exc:
            log.info("client %s writer exit: %s", self.addr, exc)
        finally:
            self.close()

    def _reader(self) -> None:
        splitter = wire.LineSplitter()
        try:
            self.conn.settimeout(0.5)
            while not self._closed.is_set():
                try:
                    data = self.conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                for cmd in splitter.feed(data):
                    self._on_command(cmd, self)
        except OSError as exc:
            log.info("client %s reader exit: %s", self.addr, exc)
        finally:
            self.close()

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        for fn in (lambda: self.conn.shutdown(socket.SHUT_RDWR), self.conn.close):
            try:
                fn()
            except OSError:
                pass
        self._on_close(self)


class Hub:
    def __init__(self, rec_dir: str):
        self._lock = threading.Lock()
        self._clients: list[_Client] = []
        self.recorder = Recorder(rec_dir)
        self.extra_hello: dict = {}

    def hello(self) -> dict:
        return {
            "k": "hello", "t": time.monotonic(),
            "mono": time.monotonic(), "unix": time.time(),
            "wire": wire.WIRE_VERSION, "server": SERVER_VERSION,
            "host": socket.gethostname(),
            "channels": CHANNEL_NAMES, "mv_per_code": ef.MV_PER_CODE,
            **self.extra_hello,
        }

    def publish(self, ev: dict) -> None:
        line = wire.dumps(ev)
        with self._lock:
            self.recorder.write(line)
            clients = list(self._clients)
        for c in clients:
            if not c.enqueue(line):
                log.warning("dropping slow client %s", c.addr)
                c.close()

    def send_to(self, client: _Client, ev: dict) -> None:
        client.enqueue(wire.dumps(ev))

    def add_client(self, c: _Client) -> int:
        with self._lock:
            self._clients.append(c)
            return len(self._clients)

    def remove_client(self, c: _Client) -> int:
        with self._lock:
            if c in self._clients:
                self._clients.remove(c)
            return len(self._clients)

    @property
    def n_clients(self) -> int:
        with self._lock:
            return len(self._clients)

    def rec_start(self, tag: str = "") -> str:
        with self._lock:
            return self.recorder.start(tag, self.hello())

    def rec_stop(self) -> Optional[str]:
        with self._lock:
            return self.recorder.stop()


# ─────────────────────────────────────────────────────────────────────────────
#  Sources
# ─────────────────────────────────────────────────────────────────────────────

class RateCounter:
    def __init__(self) -> None:
        self.counts: dict[str, int] = {}
        self._lock = threading.Lock()

    def tick(self, key: str) -> None:
        with self._lock:
            self.counts[key] = self.counts.get(key, 0) + 1

    def snapshot_and_reset(self) -> dict[str, int]:
        with self._lock:
            out, self.counts = self.counts, {}
            return out


class Esp32Source:
    """Reads binary frames from the ESP32; reopens the port if it goes away."""

    def __init__(self, port: str, baud: int, hub: Hub, rates: RateCounter):
        self.port, self.baud = port, baud
        self.hub, self.rates = hub, rates
        self.decoder = ef.Decoder()
        self._unwrap = ef.TimeUnwrapper()
        self._ser = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self.last_scan_mono = 0.0
        self.last_seq: Optional[int] = None
        self.n_scans = 0
        self.n_seq_gaps = 0
        self.info: Optional[dict] = None
        self._thread = threading.Thread(target=self._run, name="esp32", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def write(self, data: bytes) -> bool:
        with self._wlock:
            if self._ser is None:
                return False
            try:
                self._ser.write(data)
                return True
            except Exception:  # noqa: BLE001
                return False

    def reset(self) -> bool:
        """Pulse EN through the USB-UART RTS line (the classic auto-reset
        circuit) — the remote equivalent of pressing the EN button."""
        with self._wlock:
            if self._ser is None:
                return False
            self._ser.dtr = False
            self._ser.rts = True
            time.sleep(0.1)
            self._ser.rts = False
        self._unwrap = ef.TimeUnwrapper()
        self.last_seq = None
        return True

    def _open(self):
        import serial  # pyserial
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = self.port, self.baud, 0.2
        # Don't toggle EN/IO0 on open: deassert both before opening.
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser

    def _run(self) -> None:
        last_kick = 0.0
        while not self._stop.is_set():
            if self._ser is None:
                try:
                    ser = self._open()
                except Exception as exc:  # noqa: BLE001
                    log.warning("serial %s: %s (retrying)", self.port, exc)
                    self._stop.wait(2.0)
                    continue
                with self._wlock:
                    self._ser = ser
                log.info("opened %s @ %d", self.port, self.baud)
                self.hub.publish({"k": "txt", "t": time.monotonic(), "src": "server",
                                  "msg": f"opened {self.port} @ {self.baud}"})
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
            except Exception as exc:  # noqa: BLE001
                log.warning("serial read failed: %s", exc)
                with self._wlock:
                    try:
                        self._ser.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self._ser = None
                continue
            now = time.monotonic()
            if data:
                for ev in self.decoder.feed(data):
                    self._handle(ev, now)
            # Not streaming (boot banner / text mode / ESP32 rebooted)? Ask again.
            if now - self.last_scan_mono > 2.0 and now - last_kick > 2.0:
                self.write(b"B")
                last_kick = now

    def _handle(self, ev, now: float) -> None:
        if isinstance(ev, ef.Scan):
            if self.last_seq is not None and ev.seq != self.last_seq + 1:
                self.n_seq_gaps += 1
            self.last_seq = ev.seq
            self.last_scan_mono = now
            self.n_scans += 1
            self.rates.tick("wsk")
            self.hub.publish({"k": "wsk", "t": now, "seq": ev.seq, "tus": self._unwrap(ev.t_us),
                              "dur": ev.dur_us, "rail": ev.railmask, "flg": ev.flags,
                              "raw": ev.raw})
        elif isinstance(ev, ef.Text):
            self.hub.publish({"k": "txt", "t": now, "src": "esp32", "msg": ev.text})
        elif isinstance(ev, ef.Info):
            self.info = dict(ev.__dict__)
            self.hub.extra_hello["esp32"] = self.info
            self.hub.publish({"k": "info", "t": now, **self.info})


class MavSource:
    def __init__(self, url: str, hub: Hub, rates: RateCounter, msg_rates: dict[str, float],
                 sysid: int, compid: int):
        self.url, self.hub, self.rates = url, hub, rates
        self.msg_rates = msg_rates
        self.sysid, self.compid = sysid, compid
        self._stop = threading.Event()
        self._conn = None
        self.target: Optional[tuple[int, int]] = None
        self.n_msgs = 0
        self._thread = threading.Thread(target=self._run, name="mavlink", daemon=True)

    def start(self) -> None:
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil
        self._mavutil = mavutil
        self._conn = mavutil.mavlink_connection(
            self.url, source_system=self.sysid, source_component=self.compid,
            dialect="ardupilotmega", autoreconnect=True)
        log.info("MAVLink listening on %s", self.url)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _request_rates(self) -> None:
        if self.target is None:
            return
        m = self._conn.mav
        mavlink = self._mavutil.mavlink
        for name, hz in self.msg_rates.items():
            msg_id = getattr(mavlink, f"MAVLINK_MSG_ID_{name}", None)
            if msg_id is None:
                continue
            interval_us = int(1e6 / hz) if hz > 0 else -1
            m.command_long_send(self.target[0], self.target[1],
                                mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, interval_us, 0, 0, 0, 0, 0)

    def _heartbeat(self) -> None:
        mavlink = self._mavutil.mavlink
        self._conn.mav.heartbeat_send(mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                      mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _run(self) -> None:
        last_hb = last_req = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_hb > 1.0:
                try:
                    self._heartbeat()
                except Exception:  # noqa: BLE001  (no peer address yet on udpin)
                    pass
                last_hb = now
            if self.target is not None and now - last_req > 5.0:
                try:
                    self._request_rates()
                except Exception as exc:  # noqa: BLE001
                    log.warning("rate request failed: %s", exc)
                last_req = now
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.2)
            except Exception as exc:  # noqa: BLE001
                log.warning("mavlink recv: %s", exc)
                continue
            if msg is None:
                continue
            t = time.monotonic()
            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue
            src = [msg.get_srcSystem(), msg.get_srcComponent()]
            if (self.target is None and mtype == "HEARTBEAT"
                    and msg.autopilot != self._mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                self.target = (src[0], src[1])
                log.info("autopilot found: sysid %d compid %d", *self.target)
                self.hub.publish({"k": "txt", "t": t, "src": "server",
                                  "msg": f"autopilot sysid {src[0]} compid {src[1]}"})
                last_req = 0.0
            d = msg.to_dict()
            d.pop("mavpackettype", None)
            self.n_msgs += 1
            self.rates.tick(mtype)
            self.hub.publish({"k": "mav", "t": t, "type": mtype, "src": src, "d": d})


# ─────────────────────────────────────────────────────────────────────────────
#  Server
# ─────────────────────────────────────────────────────────────────────────────

class Server:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.hub = Hub(args.rec_dir)
        self.rates = RateCounter()
        self.esp32 = (Esp32Source(args.serial, args.baud, self.hub, self.rates)
                      if args.serial.lower() != "none" else None)
        self.mav = (MavSource(args.mavlink, self.hub, self.rates, args.mav_rates,
                              args.sysid, args.compid)
                    if args.mavlink.lower() != "none" else None)
        self._stop = threading.Event()
        host, port = args.bind
        self._ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._ls.bind((host, port))
        self._ls.listen(8)
        self._ls.settimeout(0.5)
        log.info("listening on %s:%d", host, port)

    # ----- commands from clients

    def on_command(self, cmd: dict, client: _Client) -> None:
        name = cmd.get("cmd")
        t = time.monotonic()
        reply: dict = {"k": "ack", "t": t, "cmd": name, "ok": True}
        if name == "rec_start":
            reply["path"] = self.hub.rec_start(str(cmd.get("tag", "")))
            log.info("recording -> %s", reply["path"])
        elif name == "rec_stop":
            reply["path"] = self.hub.rec_stop()
            log.info("recording stopped: %s", reply["path"])
        elif name == "mark":
            self.hub.publish({"k": "mark", "t": t, "text": str(cmd.get("text", ""))})
        elif name == "esp32":
            reply["ok"] = bool(self.esp32 and self.esp32.write(str(cmd.get("data", "")).encode()))
        elif name == "esp32_reset":
            reply["ok"] = bool(self.esp32 and self.esp32.reset())
        elif name == "status":
            reply.update(self._status(1.0, {}))
        else:
            reply.update(ok=False, error=f"unknown command {name!r}")
        self.hub.send_to(client, reply)

    def _on_client_closed(self, c: _Client) -> None:
        log.info("client %s disconnected (%d left)", c.addr, self.hub.remove_client(c))

    # ----- status

    def _status(self, dt: float, counts: dict[str, int]) -> dict:
        rec = self.hub.recorder
        st: dict = {
            "k": "status", "t": time.monotonic(), "clients": self.hub.n_clients,
            "rec": {"on": rec.active, "path": rec.path,
                    "secs": (time.monotonic() - rec.started_mono) if rec.active else 0,
                    "events": rec.n_events},
            "hz": {k: round(v / dt, 1) for k, v in sorted(counts.items())},
        }
        if self.esp32:
            dec = self.esp32.decoder
            st["esp32"] = {"scans": self.esp32.n_scans, "seq_gaps": self.esp32.n_seq_gaps,
                           "crc_fail": dec.n_crc_fail, "skipped": dec.n_skipped,
                           "streaming": time.monotonic() - self.esp32.last_scan_mono < 1.0}
        if self.mav:
            st["mav"] = {"msgs": self.mav.n_msgs, "target": self.mav.target}
        return st

    def _status_loop(self) -> None:
        last = time.monotonic()
        n = 0
        while not self._stop.wait(1.0):
            now = time.monotonic()
            st = self._status(now - last, self.rates.snapshot_and_reset())
            last = now
            self.hub.publish(st)
            n += 1
            if n % self.args.log_every == 0:
                hz = st["hz"]
                imu = ", ".join(f"{k} {hz[k]:g}" for k in self.args.mav_rates if k in hz)
                log.info("whisker %s Hz | %s | clients %d | rec %s",
                         hz.get("wsk", 0), imu or "no MAVLink", st["clients"],
                         os.path.basename(st["rec"]["path"]) if st["rec"]["on"] else "off")

    # ----- lifecycle

    def run(self) -> None:
        if self.esp32:
            self.esp32.start()
        if self.mav:
            self.mav.start()
        threading.Thread(target=self._status_loop, name="status", daemon=True).start()
        if self.args.record:
            log.info("recording -> %s", self.hub.rec_start(self.args.tag))
        while not self._stop.is_set():
            try:
                conn, addr = self._ls.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c = _Client(conn, addr, self.on_command, self._on_client_closed)
            self.hub.send_to(c, self.hub.hello())
            log.info("client %s connected (%d total)", addr, self.hub.add_client(c))

    def stop(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        path = self.hub.rec_stop()
        if path:
            log.info("recording closed: %s", path)
        for src in (self.esp32, self.mav):
            if src:
                src.stop()
        try:
            self._ls.close()
        except OSError:
            pass


def _parse_bind(s: str) -> tuple[str, int]:
    host, _, port = s.rpartition(":")
    if not host or not port:
        raise argparse.ArgumentTypeError(f"--bind must be host:port, got {s!r}")
    return host, int(port)


def _parse_rates(s: str) -> dict[str, float]:
    """``RAW_IMU=200,ATTITUDE=50`` -> overrides on top of DEFAULT_MAV_RATES."""
    out = dict(DEFAULT_MAV_RATES)
    for item in filter(None, (x.strip() for x in s.split(","))):
        name, _, hz = item.partition("=")
        out[name.strip().upper()] = float(hz)
    return {k: v for k, v in out.items() if v != 0}


def main(argv: Optional[list[str]] = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--serial", default="/dev/ttyUSB0", help="ESP32 port, or 'none'")
    ap.add_argument("--baud", type=int, default=DEFAULT_SERIAL_BAUD)
    ap.add_argument("--mavlink", default=DEFAULT_MAVLINK,
                    help="pymavlink URL of the BlueOS endpoint, or 'none'")
    ap.add_argument("--mav-rates", type=_parse_rates, default=dict(DEFAULT_MAV_RATES),
                    help="overrides, e.g. RAW_IMU=200,SERVO_OUTPUT_RAW=0 (0 = don't request)")
    ap.add_argument("--sysid", type=int, default=255, help="our MAVLink system id")
    ap.add_argument("--compid", type=int, default=191, help="our MAVLink component id")
    ap.add_argument("--bind", type=_parse_bind, default=("0.0.0.0", wire.DEFAULT_PORT))
    ap.add_argument("--rec-dir", default="~/whisker_rec")
    ap.add_argument("--record", action="store_true", help="start recording immediately")
    ap.add_argument("--tag", default="", help="tag for the --record file name")
    ap.add_argument("--log-every", type=int, default=5, help="status log period, s")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args(argv)

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)s %(message)s")
    server = Server(args)
    signal.signal(signal.SIGINT, lambda *_: server.stop())
    signal.signal(signal.SIGTERM, lambda *_: server.stop())
    try:
        server.run()
    finally:
        server.stop()


if __name__ == "__main__":
    main()
