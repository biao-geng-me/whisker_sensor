"""ESP32 whisker DAQ plugged straight into the PC (no BlueOS Pi in between).

Speaks the firmware's binary frames (``whisker_link/esp32_frame.py``) on the
serial port and hands ``ServerReader`` the same JSON-lines event stream that
``rpi/daq_server.py`` sends: ``hello`` first, then ``wsk`` / ``txt`` / ``info``
events and a 1 Hz ``status``. So the GUI, the local ``.jsonl`` copy and
``whisker_link.convert`` work unchanged; there is just no IMU.

Commands written to the transport are the server's JSON commands. Handled
locally: ``esp32`` (raw bytes to the firmware), ``esp32_reset`` (RTS pulse =
pressing EN), ``mark``; ``rec_start`` / ``rec_stop`` are acknowledged and
otherwise ignored, since the local tee is the only recording here.

ADC / ESP32 faults (``whisker_link/esp32_fault.py``, as in daq_server): the
status carries ``esp32.fault``, and each fault onset / clear / warning is also
sent as a ``mark`` event (so it lands in the local .jsonl).
"""

from __future__ import annotations

import collections
import json
import socket
import threading
import time

import serial

from wavi_py.transports.base import Transport
from whisker_link import esp32_frame as ef
from whisker_link import wire
from whisker_link.esp32_fault import FaultMonitor, mark_event, mark_text
from whisker_link.channels import CHANNEL_NAMES

ESP32_BAUD = 921600          # whisker_daq.ino SERIAL_BAUD
_KICK_S = 2.0                # not streaming for this long -> send 'B' again


class Esp32SerialTransport(Transport):
    def __init__(self, port: str, baudrate: int = ESP32_BAUD, read_timeout_s: float = 0.2):
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = port, baudrate, read_timeout_s
        ser.write_timeout = 1.0
        # Don't toggle EN/IO0 on open (the auto-reset circuit): deassert first.
        ser.dtr = False
        ser.rts = False
        ser.open()
        self._ser: serial.Serial | None = ser
        self._port, self._baud = port, baudrate
        self._wlock = threading.Lock()
        self._decoder = ef.Decoder()
        self._unwrap = ef.TimeUnwrapper()
        self._out: collections.deque[bytes] = collections.deque()   # events for read_some
        self._info: dict | None = None
        self._last_seq: int | None = None
        self._last_scan = 0.0
        self._last_kick = 0.0
        self._last_status = time.monotonic()
        self._scans_since_status = 0
        self._n = {"scans": 0, "seq_gaps": 0, "adc_recov": 0, "adc_crc": 0, "adc_stale": 0,
                   "adc_reinit": 0, "adc_sync": 0, "adc_empty": 0}
        self._health: dict | None = None        # latest HEALTH frame (locked scan modes)
        self._beat: dict | None = None          # latest heartbeat (BEAT, every mode)
        self._faults = FaultMonitor()
        self._faults.link(True)
        self._emit(self._hello())
        self._kick()

    # ----- Transport

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def read_some(self, max_n: int) -> bytes:
        ser = self._ser
        if ser is None:
            raise ConnectionError("transport closed")
        data = ser.read(ser.in_waiting or 1)       # returns b"" after read_timeout_s
        now = time.monotonic()
        if data:
            for ev in self._decoder.feed(data):
                self._handle(ev, now)
        if now - self._last_scan > _KICK_S and now - self._last_kick > _KICK_S:
            self._kick()                            # boot banner / text mode / rebooted
        self._faults.poll(now)
        for fev in self._faults.take_events():
            print(f"[esp32] {mark_text(fev)}")
            self._emit(mark_event(fev, now))
        if now - self._last_status >= 1.0:
            self._emit(self._status(now))
        parts = []
        while self._out:                            # acks may arrive from the GUI thread
            parts.append(self._out.popleft())
        return b"".join(parts)

    def read_exact(self, n: int) -> bytes:
        raise NotImplementedError("the ESP32 transport is read with read_some")

    def write(self, data: bytes) -> None:
        """Accept the server's JSON commands, one per line."""
        for line in data.splitlines():
            try:
                cmd = json.loads(line)
            except json.JSONDecodeError:
                continue
            self._command(cmd)

    def bytes_available(self) -> int:
        return sum(len(b) for b in self._out)

    def close(self) -> None:
        with self._wlock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None

    # ----- internals

    def _emit(self, ev: dict) -> None:
        self._out.append(wire.dumps(ev))

    def _raw_write(self, data: bytes) -> bool:
        with self._wlock:
            if self._ser is None:
                return False
            try:
                self._ser.write(data)
                return True
            except Exception:  # noqa: BLE001
                return False

    def _kick(self) -> None:
        self._raw_write(b"B")
        self._last_kick = time.monotonic()

    def _hello(self) -> dict:
        now = time.monotonic()
        return {"k": "hello", "t": now, "mono": now, "unix": time.time(),
                "wire": wire.WIRE_VERSION, "server": "wavi_py-esp32",
                "host": socket.gethostname(), "serial": self._port,
                "channels": CHANNEL_NAMES, "mv_per_code": ef.MV_PER_CODE}

    def _handle(self, ev, now: float) -> None:
        if isinstance(ev, ef.Scan):
            if self._last_seq is not None and ev.seq != self._last_seq + 1:
                self._n["seq_gaps"] += 1
            self._last_seq = ev.seq
            self._last_scan = now
            self._n["scans"] += 1
            self._scans_since_status += 1
            self._faults.on_scan(ev.flags, now)
            if ev.flags & ef.FLAG_RECOVERED:
                self._n["adc_recov"] += 1
            if ev.flags & ef.FLAG_CRC_RETRY:
                self._n["adc_crc"] += 1
            if ev.flags & ef.FLAG_STALE:
                self._n["adc_stale"] += 1
            if ev.flags & ef.FLAG_REINIT:
                self._n["adc_reinit"] += 1
            if ev.flags & ef.FLAG_SYNC:
                self._n["adc_sync"] += 1
            if ev.flags & ef.FLAG_EMPTY:
                self._n["adc_empty"] += 1
            self._emit({"k": "wsk", "t": now, "seq": ev.seq, "tus": self._unwrap(ev.t_us),
                        "dur": ev.dur_us, "rail": ev.railmask, "flg": ev.flags, "raw": ev.raw})
        elif isinstance(ev, ef.Text):
            self._emit({"k": "txt", "t": now, "src": "esp32", "msg": ev.text})
            self._faults.on_text(ev.text, now)
        elif isinstance(ev, ef.Info):
            self._info = dict(ev.__dict__)
            if self._info.get("mode") not in (1, 2):
                self._health = None
            self._emit({"k": "info", "t": now, **self._info})
        elif isinstance(ev, ef.Health):
            self._health = ev.summary()
        elif isinstance(ev, ef.Beat):
            self._beat = ev.summary()
            self._faults.on_beat(ev, now)

    def _status(self, now: float) -> dict:
        dt = now - self._last_status
        hz = round(self._scans_since_status / dt, 1) if dt > 0 else 0.0
        self._last_status, self._scans_since_status = now, 0
        return {"k": "status", "t": now, "clients": 1,
                "rec": {"on": False, "path": None, "secs": 0, "events": 0},
                "hz": {"wsk": hz},
                "esp32": {**self._n, "crc_fail": self._decoder.n_crc_fail,
                          "skipped": self._decoder.n_skipped,
                          "streaming": now - self._last_scan < 1.0,
                          **({"cfg": ef.info_cfg(self._info)} if self._info else {}),
                          **({"health": self._health} if self._health else {}),
                          **({"beat": self._beat} if self._beat else {}),
                          "fault": self._faults.status(now)}}

    def _command(self, cmd: dict) -> None:
        name = cmd.get("cmd")
        now = time.monotonic()
        reply = {"k": "ack", "t": now, "cmd": name, "ok": True}
        if name == "esp32":
            reply["ok"] = self._raw_write(str(cmd.get("data", "")).encode())
        elif name == "esp32_reset":
            self._faults.expect_reset(now)
            try:
                reply["ok"] = self._reset()
            except Exception as exc:  # noqa: BLE001  (no RTS line, port gone)
                reply.update(ok=False, error=f"RTS pulse failed on {self._port}: {exc}")
        elif name == "mark":
            self._emit({"k": "mark", "t": now, "text": str(cmd.get("text", ""))})
        elif name in ("rec_start", "rec_stop", "status"):
            pass                                   # local tee is the recording
        else:
            reply.update(ok=False, error=f"unknown command {name!r}")
        self._emit(reply)

    def _reset(self) -> bool:
        """Pulse EN through RTS, like daq_server's esp32_reset."""
        with self._wlock:
            if self._ser is None:
                return False
            self._ser.dtr = False
            self._ser.rts = True
            time.sleep(0.1)
            self._ser.rts = False
        self._unwrap = ef.TimeUnwrapper()
        self._last_seq = None
        return True
