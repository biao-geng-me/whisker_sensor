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

ADC / ESP32 faults (``whisker_link/esp32_fault.py``): an ADC that stopped
converting (UNRECOVERED halt, CLOCK not accepted, no scans for 5 s), a lost
serial port, and warnings (recovered halts, ADC resets, watchdog reboots,
[idf] errors, flagged scans) go in the status record as ``esp32.fault``, in
the log, and as ``mark`` events (fault onset, clear, each warning) in the
recording and to the clients. A recording started during a fault gets a mark
with the fault's state first.

Self-healing: a port that is open but delivers nothing for ``REOPEN_S`` is
closed and reopened (a stale handle after a USB glitch); serial writes time
out instead of blocking. If the ESP32 or MAVLink thread stops looping (or
dies) for ``--stall-exit`` seconds, the server dumps every thread's stack to
the log and exits, so systemd restarts it. ``kill -USR1 <pid>`` dumps the
stacks without exiting.
"""

from __future__ import annotations

import argparse
import faulthandler
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
from whisker_link.esp32_fault import FaultMonitor, mark_event, mark_text  # noqa: E402
from whisker_link.channels import CHANNEL_NAMES  # noqa: E402

log = logging.getLogger("daq_server")

SERVER_VERSION = "0.1"
DEFAULT_SERIAL_BAUD = 921600
DEFAULT_MAVLINK = "udpin:0.0.0.0:14670"

# MAVLink message -> requested rate (Hz). ArduSub on Navigator runs its main
# loop at 400 Hz; what it can actually stream is measured in the status line.
DEFAULT_MAV_RATES = {
    "RAW_IMU": 100,            # IMU0 accel (mg) / gyro (mrad/s) / mag
    # SCALED_IMU2 not requested: on this vehicle its accel/gyro are all zero
    # (only the magnetometer is populated), and it cost 14% of the file.
    "ATTITUDE": 50,            # roll/pitch/yaw + rates (EKF)
    "ATTITUDE_QUATERNION": 50,
    "SCALED_PRESSURE2": 20,    # Bar30 depth sensor on ArduSub
    "SERVO_OUTPUT_RAW": 20,    # thruster PWM — likely matters for the whiskers
    "RC_CHANNELS": 20,         # pilot inputs as ArduSub sees them (joystick -> RC)
    # MANUAL_CONTROL (joystick, GCS -> vehicle) can't be requested from the
    # autopilot; it is recorded if the router forwards Cockpit's copy to us.
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
        # Blocking recv, no socket timeout: a timeout is per socket, so it would
        # also cut the writer's sendall when the laptop stalls for a moment
        # (e.g. Cockpit starting video). close() unblocks recv via shutdown().
        # A stalled client is bounded by the queue instead: publish() drops it
        # once the queue is full.
        splitter = wire.LineSplitter()
        try:
            while not self._closed.is_set():
                data = self.conn.recv(4096)
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

    def rec_stop(self, timeout: float = -1) -> Optional[str]:
        if not self._lock.acquire(timeout=timeout):
            return None
        try:
            return self.recorder.stop()
        finally:
            self._lock.release()


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


def _garbled(ev) -> bool:
    """A ``Text`` "line" that is really binary noise split at a stray newline.
    The firmware prints plain ASCII, so these are no sign of life."""
    return isinstance(ev, ef.Text) and any(
        c == "\ufffd" or (ord(c) < 32 and c != "\t") for c in ev.text)


class Esp32Source:
    """Reads binary frames from the ESP32; reopens the port if it goes away."""

    REOPEN_S = 10.0           # port open but nothing decoded this long: close and reopen it
    WRITE_TIMEOUT_S = 1.0     # a write that can't go out fails instead of blocking the reader

    def __init__(self, port: str, baud: int, hub: Hub, rates: RateCounter,
                 selects: Optional[str] = None, mode: Optional[str] = None):
        self.port, self.baud = port, baud
        # what "start streaming" sends: the select list and the scan mode first
        # (the ESP32 forgets both on reboot)
        self.kick = ((f"S{selects};".encode() if selects else b"")
                     + (f"M{mode};".encode() if mode else b"") + b"B")
        self.hub, self.rates = hub, rates
        self.decoder = ef.Decoder()
        self._unwrap = ef.TimeUnwrapper()
        self._ser = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self.loop_mono = time.monotonic()      # reader thread alive (the stall watchdog)
        self.last_event_mono = 0.0             # last decoded frame or text line
        self._opened_mono = 0.0
        self._last_wfail_log = 0.0
        self.n_reopens = 0
        self.last_scan_mono = 0.0
        self.last_seq: Optional[int] = None
        self.n_scans = 0
        self.n_seq_gaps = 0
        self.n_adc_recov = 0          # scans with FLAG_RECOVERED (ADC halt recovered)
        self.n_adc_crc = 0            # scans with FLAG_CRC_RETRY (SPI read re-read)
        self.n_adc_stale = 0          # scans with FLAG_STALE (read had no new conversion)
        self.n_adc_reinit = 0         # scans with FLAG_REINIT (ADC had reset, re-initialised)
        self.n_adc_sync = 0           # scans with FLAG_SYNC (locked scan lost/regained the grid)
        self.n_adc_empty = 0          # scans with FLAG_EMPTY (a slot had no usable conversion)
        self.health: Optional[dict] = None    # latest HEALTH frame (locked scan modes)
        self.beat: Optional[dict] = None      # latest heartbeat (BEAT, every mode)
        self.info: Optional[dict] = None
        self.faults = FaultMonitor()             # touched by this thread only, except status()
        self._fault_lock = threading.Lock()
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
            except Exception as exc:  # noqa: BLE001  (incl. the write timeout)
                now = time.monotonic()
                if now - self._last_wfail_log > 60.0:
                    log.warning("serial write to %s failed: %s", self.port, exc)
                    self._last_wfail_log = now
                return False

    def reset(self) -> Optional[str]:
        """Pulse EN through the USB-UART RTS line (the classic auto-reset
        circuit) — the remote equivalent of pressing the EN button.
        Returns None on success, else why not (no port; a pty has no RTS)."""
        with self._wlock:
            if self._ser is None:
                return f"serial port {self.port} is not open"
            try:
                self._ser.dtr = False
                self._ser.rts = True
                time.sleep(0.1)
            except Exception as exc:  # noqa: BLE001
                return f"RTS pulse failed on {self.port}: {exc}"
            finally:
                try:
                    self._ser.rts = False
                except Exception:  # noqa: BLE001
                    pass
        with self._fault_lock:
            self.faults.expect_reset()
        self._unwrap = ef.TimeUnwrapper()
        self.last_seq = None
        return None

    def fault_status(self) -> dict:
        with self._fault_lock:
            return self.faults.status()

    def _publish_faults(self, now: float) -> None:
        with self._fault_lock:
            self.faults.poll(now)
            evs = self.faults.take_events()
        for fev in evs:
            (log.error if fev["event"] == "fault" else log.warning)("%s", mark_text(fev))
            self.hub.publish(mark_event(fev, now))

    def _open(self):
        import serial  # pyserial
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = self.port, self.baud, 0.2
        ser.write_timeout = self.WRITE_TIMEOUT_S
        # Don't toggle EN/IO0 on open: deassert both before opening.
        ser.dtr = False
        ser.rts = False
        ser.open()
        return ser

    def _run(self) -> None:
        last_kick = 0.0
        last_err, last_err_log = "", 0.0
        while not self._stop.is_set():
            self.loop_mono = time.monotonic()
            if self._ser is None:
                try:
                    ser = self._open()
                except Exception as exc:  # noqa: BLE001
                    # At boot the ESP32 may not be plugged in for hours: log a
                    # new error at once, the same one once a minute.
                    now = time.monotonic()
                    if str(exc) != last_err or now - last_err_log > 60.0:
                        log.warning("serial %s: %s (retrying every 2 s)", self.port, exc)
                        last_err, last_err_log = str(exc), now
                    self._publish_faults(time.monotonic())
                    self._stop.wait(2.0)
                    continue
                with self._wlock:
                    self._ser = ser
                self._opened_mono = time.monotonic()
                last_err = ""
                with self._fault_lock:
                    self.faults.link(True)
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
                with self._fault_lock:
                    self.faults.link(False)
                continue
            now = time.monotonic()
            if data:
                for ev in self.decoder.feed(data):
                    if not _garbled(ev):
                        self.last_event_mono = now
                    self._handle(ev, now)
            quiet = now - max(self.last_event_mono, self._opened_mono)
            if quiet > self.REOPEN_S:
                # Open, but nothing decodes: a handle left stale by a USB glitch looks
                # like this. Reopening is cheap and doesn't reset the ESP32.
                self.n_reopens += 1
                log.warning("nothing from %s for %.0f s (%d junk bytes so far): reopening the port",
                            self.port, quiet, self.decoder.n_junk)
                self.hub.publish({"k": "txt", "t": now, "src": "server",
                                  "msg": f"no data for {quiet:.0f} s: reopening {self.port}"})
                with self._wlock:
                    try:
                        self._ser.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self._ser = None
                continue
            # Not streaming (boot banner / text mode / ESP32 rebooted)? Ask again.
            if now - self.last_scan_mono > 2.0 and now - last_kick > 2.0:
                self.write(self.kick)
                last_kick = now
            self._publish_faults(now)

    def _handle(self, ev, now: float) -> None:
        if isinstance(ev, ef.Scan):
            if self.last_seq is not None and ev.seq != self.last_seq + 1:
                self.n_seq_gaps += 1
            self.last_seq = ev.seq
            self.last_scan_mono = now
            with self._fault_lock:
                self.faults.on_scan(ev.flags, now)
            if ev.flags & ef.FLAG_RECOVERED:
                self.n_adc_recov += 1
            if ev.flags & ef.FLAG_CRC_RETRY:
                self.n_adc_crc += 1
            if ev.flags & ef.FLAG_STALE:
                self.n_adc_stale += 1
            if ev.flags & ef.FLAG_REINIT:
                self.n_adc_reinit += 1
            if ev.flags & ef.FLAG_SYNC:
                self.n_adc_sync += 1
            if ev.flags & ef.FLAG_EMPTY:
                self.n_adc_empty += 1
            self.n_scans += 1
            self.rates.tick("wsk")
            self.hub.publish({"k": "wsk", "t": now, "seq": ev.seq, "tus": self._unwrap(ev.t_us),
                              "dur": ev.dur_us, "rail": ev.railmask, "flg": ev.flags,
                              "raw": ev.raw})
        elif isinstance(ev, ef.Text):
            self.hub.publish({"k": "txt", "t": now, "src": "esp32", "msg": ev.text})
            if _garbled(ev):
                return                             # noise must not count as a heartbeat
            with self._fault_lock:
                self.faults.on_text(ev.text, now)
        elif isinstance(ev, ef.Info):
            self.info = dict(ev.__dict__)
            if self.info.get("mode") != 1 and self.info.get("mode") != 2:
                self.health = None                 # legacy scan: no HEALTH frames
            self.hub.extra_hello["esp32"] = self.info
            self.hub.publish({"k": "info", "t": now, **self.info})
        elif isinstance(ev, ef.Health):
            self.health = {"t": now, **ev.summary()}
        elif isinstance(ev, ef.Beat):
            self.beat = {"t": now, **ev.summary()}
            with self._fault_lock:
                self.faults.on_beat(ev, now)


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
        self.loop_mono = time.monotonic()      # thread alive (the stall watchdog)
        self._thread = threading.Thread(target=self._run, name="mavlink", daemon=True)

    def start(self) -> None:
        os.environ.setdefault("MAVLINK20", "1")
        from pymavlink import mavutil
        self._mavutil = mavutil
        self._thread.start()

    def _connect(self) -> bool:
        """Open the endpoint; on failure (e.g. the port is taken) retry every 5 s
        instead of exiting, so a service started at boot doesn't crash-loop."""
        last_err = ""
        while not self._stop.is_set():
            self.loop_mono = time.monotonic()
            try:
                self._conn = self._mavutil.mavlink_connection(
                    self.url, source_system=self.sysid, source_component=self.compid,
                    dialect="ardupilotmega", autoreconnect=True)
                log.info("MAVLink listening on %s", self.url)
                return True
            except Exception as exc:  # noqa: BLE001
                if str(exc) != last_err:
                    log.error("MAVLink %s: %s (retrying every 5 s)", self.url, exc)
                    last_err = str(exc)
                self._stop.wait(5.0)
        return False

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
        if not self._connect():
            return
        last_hb = last_req = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            self.loop_mono = now
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
        self.esp32 = (Esp32Source(args.serial, args.baud, self.hub, self.rates, args.selects,
                                  args.mode)
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
        try:
            self._dispatch(name, cmd, client, t, reply)
        except Exception as exc:  # noqa: BLE001  (a bad command must not drop the client)
            log.exception("command %r from %s failed", name, client.addr)
            reply.update(ok=False, error=f"{type(exc).__name__}: {exc}")
        self.hub.send_to(client, reply)

    def _dispatch(self, name, cmd: dict, client: _Client, t: float, reply: dict) -> None:
        if name == "rec_start":
            reply["path"] = self._rec_start(str(cmd.get("tag", "")))
        elif name == "rec_stop":
            reply["path"] = self.hub.rec_stop()
            log.info("recording stopped: %s", reply["path"])
        elif name == "mark":
            self.hub.publish({"k": "mark", "t": t, "text": str(cmd.get("text", ""))})
        elif name == "esp32":
            reply["ok"] = bool(self.esp32 and self.esp32.write(str(cmd.get("data", "")).encode()))
        elif name == "esp32_reset":
            err = self.esp32.reset() if self.esp32 else "no ESP32 (--serial none)"
            reply["ok"] = err is None
            if err:
                reply["error"] = err
                log.warning("ESP32 reset from %s failed: %s", client.addr, err)
            else:
                # the stream has a gap from here: say why, in the recording too
                log.warning("ESP32 reset (EN pulse) by client %s", client.addr[0])
                self.hub.publish({"k": "mark", "t": t,
                                  "text": f"ESP32 reset (EN pulse) by client {client.addr[0]}"})
        elif name == "status":
            reply.update(self._status(1.0, {}))
        else:
            reply.update(ok=False, error=f"unknown command {name!r}")

    def _rec_start(self, tag: str) -> str:
        path = self.hub.rec_start(tag)
        log.info("recording -> %s", path)
        if self.esp32:
            st = self.esp32.fault_status()
            if st["level"] != "ok":                # a fault from before the recording
                ev = {"event": "state", "level": st["level"], "code": st["code"],
                      "msg": st["msg"], "action": st.get("action", ""),
                      "detail": st.get("detail", ""), "count": st["count"],
                      "unix": st["since"]}
                since = time.strftime("%H:%M:%S", time.localtime(st["since"]))
                self.hub.publish({"k": "mark", "t": time.monotonic(), "fault": ev,
                                  "text": f"at record start: ESP32 {st['level']} {st['code']}:"
                                          f" {st['msg']} (since {since})"})
        return path

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
                           "junk": dec.n_junk, "reopens": self.esp32.n_reopens,
                           "adc_recov": self.esp32.n_adc_recov, "adc_crc": self.esp32.n_adc_crc,
                           "adc_stale": self.esp32.n_adc_stale,
                           "adc_reinit": self.esp32.n_adc_reinit,
                           "adc_sync": self.esp32.n_adc_sync, "adc_empty": self.esp32.n_adc_empty,
                           "streaming": time.monotonic() - self.esp32.last_scan_mono < 1.0}
            if self.esp32.info:
                st["esp32"]["cfg"] = ef.info_cfg(self.esp32.info)
            if self.esp32.health:
                h = dict(self.esp32.health)
                h["age_s"] = round(time.monotonic() - h.pop("t"), 1)
                st["esp32"]["health"] = h
            if self.esp32.beat:
                b = dict(self.esp32.beat)
                b["age_s"] = round(time.monotonic() - b.pop("t"), 1)
                st["esp32"]["beat"] = b
            st["esp32"]["fault"] = self.esp32.fault_status()
        if self.mav:
            st["mav"] = {"msgs": self.mav.n_msgs, "target": self.mav.target}
        return st

    def _stalled(self, now: float) -> list[str]:
        """Source threads that died or haven't looped for ``--stall-exit`` s."""
        limit = getattr(self.args, "stall_exit", 0)
        out = []
        for src in (self.esp32, self.mav):
            if not limit or src is None or not src._thread.ident or self._stop.is_set():
                continue                       # off, not configured, not started, stopping
            if not src._thread.is_alive():
                out.append(f"{src._thread.name} thread died")
            elif now - src.loop_mono > limit:
                out.append(f"{src._thread.name} thread stuck for {now - src.loop_mono:.0f} s")
        return out

    def _die_stalled(self, why: list[str]) -> None:
        """Nothing in-process can unstick a blocked thread: log where it is and exit
        (non-zero, so systemd's Restart=always brings up a fresh server)."""
        log.critical("%s: dumping thread stacks and exiting for a restart", "; ".join(why))
        sys.stderr.flush()
        faulthandler.dump_traceback(all_threads=True)
        path = self.hub.rec_stop(timeout=2.0)
        if path:
            log.critical("recording closed: %s", path)
        logging.shutdown()
        os._exit(3)

    def _status_loop(self) -> None:
        last = time.monotonic()
        n = 0
        while not self._stop.wait(1.0):
            now = time.monotonic()
            stalled = self._stalled(now)
            if stalled:
                self._die_stalled(stalled)
            st = self._status(now - last, self.rates.snapshot_and_reset())
            last = now
            self.hub.publish(st)
            n += 1
            if n % self.args.log_every == 0:
                hz = st["hz"]
                imu = ", ".join(f"{k} {hz[k]:g}" for k in self.args.mav_rates if k in hz)
                e = st.get("esp32", {})
                h = e.get("health")
                log.info("whisker %s Hz | %s | clients %d | rec %s | errs: gap %d link-crc %d "
                         "junk %d reopen %d adc-crc %d adc-stale %d adc-recov %d adc-reinit %d adc-sync %d%s",
                         hz.get("wsk", 0), imu or "no MAVLink", st["clients"],
                         os.path.basename(st["rec"]["path"]) if st["rec"]["on"] else "off",
                         e.get("seq_gaps", 0), e.get("crc_fail", 0), e.get("junk", 0),
                         e.get("reopens", 0),
                         e.get("adc_crc", 0), e.get("adc_stale", 0), e.get("adc_recov", 0),
                         e.get("adc_reinit", 0), e.get("adc_sync", 0),
                         f" | {e.get('cfg', {}).get('mode_name', '?')}: busy {h['busy_pct']}%"
                         f" lat max {h['lat_max_us']} us, overrun {h['overrun']}"
                         f" missed {h['missed']}" if h else "")
                f = e.get("fault")
                if f and f["level"] == "fault":       # repeated, so a log tail shows it
                    log.error("ESP32 FAULT %s since %s: %s -> %s", f["code"],
                              time.strftime("%H:%M:%S", time.localtime(f["since"])), f["msg"],
                              f.get("action", ""))

    # ----- lifecycle

    def run(self) -> None:
        if self.esp32:
            self.esp32.start()
        if self.mav:
            self.mav.start()
        threading.Thread(target=self._status_loop, name="status", daemon=True).start()
        if self.args.record:
            self._rec_start(self.args.tag)
        while not self._stop.is_set():
            try:
                conn, addr = self._ls.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(None)      # blocking; must not inherit the listener's 0.5 s
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


_MODES = {"legacy": 0, "quiet": 1, "wide": 2}


def _parse_mode(s: str) -> str:
    """``wide`` / ``2`` / ``quiet,250,512`` -> the M command's arguments, e.g. ``1 250 512``.

    Fields after the mode: settle_us, osr, period_us, spi_hz (see whisker_daq.ino, 'M');
    0 = the mode's default. ``quiet,200,256,8000`` = quiet at OSR 256 and 125 Hz.
    """
    parts = [p.strip() for p in s.replace(":", ",").split(",") if p.strip()]
    if not parts:
        raise argparse.ArgumentTypeError("mode: legacy|quiet|wide or 0|1|2")
    m = _MODES.get(parts[0].lower(), parts[0])
    try:
        nums = [int(m)] + [int(p) for p in parts[1:5]]
    except ValueError:
        raise argparse.ArgumentTypeError(f"mode: bad value {s!r}") from None
    if nums[0] not in (0, 1, 2):
        raise argparse.ArgumentTypeError("mode: legacy|quiet|wide or 0|1|2")
    return " ".join(str(n) for n in nums)


def _parse_selects(s: str) -> str:
    """``0123457`` -> the same digits, sorted and checked (mux selects 0-7)."""
    digits = sorted(set(s.replace(",", "")))
    if not digits or any(d not in "01234567" for d in digits):
        raise argparse.ArgumentTypeError("selects: digits 0-7, e.g. 0123457")
    return "".join(digits)


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
    ap.add_argument("--mode", type=_parse_mode, default=None,
                    help="ESP32 scan mode, sent as M...; before every B: legacy (rate depends on the"
                         " selects, ~89 Hz with 7), quiet (62.5 Hz, OSR 512) or wide (250 Hz),"
                         " optionally ,settle_us,osr,period_us,spi_hz (e.g. quiet,200,256 or"
                         " quiet,200,256,8000); default: leave as is (the firmware boots in"
                         " wide 250 Hz, OSR 128, selects 0123457; before 2026-10-28 quiet"
                         " 62.5 Hz, OSR 512, all selects)")
    ap.add_argument("--selects", type=_parse_selects, default=None,
                    help="mux selects the ESP32 scans, e.g. 0123457 (sent as S<digits>; before"
                         " every B, since the ESP32 forgets it on reboot); default: leave as is"
                         " (the firmware boots with 0123457, before 2026-10-28 all 8)")
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
    ap.add_argument("--stall-exit", type=float, default=20.0,
                    help="exit (for systemd to restart) when the ESP32 or MAVLink thread is"
                         " stuck this long or died, s; 0 = never")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args(argv)

    # under systemd the journal stamps each line already
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO),
                        format=("%(levelname)s %(message)s" if os.environ.get("JOURNAL_STREAM")
                                else "%(asctime)s %(levelname)s %(message)s"))
    # stacks of every thread to the log (stderr): on a crash, and on `kill -USR1 <pid>`
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    server = Server(args)
    signal.signal(signal.SIGINT, lambda *_: server.stop())
    signal.signal(signal.SIGTERM, lambda *_: server.stop())
    try:
        server.run()
    finally:
        server.stop()


if __name__ == "__main__":
    main()
