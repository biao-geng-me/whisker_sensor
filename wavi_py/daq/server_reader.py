"""DAQ thread for ``rpi/daq_server.py`` (ESP32 whisker array + ArduSub IMU).

Drop-in sibling of ``Reader``: same lifecycle (start / stop / request_reset /
wait_ready) and the same ``SampleBus`` output, so the GUI's line / FFT /
spectrogram views work unchanged. Differences:

- The stream is JSON lines (``whisker_link/wire.py``), not binary frames, so
  there is no marker alignment.
- Whisker samples go on ``bus`` as mV at the ADC, OUT order (W1_A, W1_B, ...).
- IMU rows go on a second bus, ``imu_bus``, with columns ``IMU_CHANNELS``:
  one row per RAW_IMU message, attitude sample-and-held from ATTITUDE.
- The server's 1 Hz ``status`` event is kept in ``bus`` metadata as
  ``server_status``.
- ``start_tee(path)`` copies the raw stream to a local .jsonl — the same
  format as the recording on the Pi, so ``whisker_link.convert`` reads both.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Optional

import numpy as np

from wavi_py.daq.bus import SampleBus
from wavi_py.transports.base import Transport
from whisker_link.esp32_frame import MV_PER_CODE

IMU_CHANNELS = [
    "ax_mg", "ay_mg", "az_mg",               # RAW_IMU accel
    "gx_mrads", "gy_mrads", "gz_mrads",      # RAW_IMU gyro
    "roll_rad", "pitch_rad", "yaw_rad",      # ATTITUDE (held)
]


class ServerReader:
    def __init__(self, transport: Transport, bus: SampleBus, imu_bus: SampleBus, *,
                 fs: float, nch: int):
        self._transport = transport
        self._bus = bus
        self._imu_bus = imu_bus
        self._fs = float(fs)
        self._nch = int(nch)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._reset_pending = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._tee = None
        self._tee_lock = threading.Lock()
        self._attitude = np.zeros(3, dtype=np.float32)
        self._v0_rows: list[np.ndarray] = []
        self._v0_version = 0
        self._mv_per_code = MV_PER_CODE
        self.hello: dict = {}
        self._last_t = 0.0           # server clock of the newest event seen
        self.error: Optional[str] = None   # why _run ended, if not stop()

    # ----- lifecycle (same surface as Reader)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._ready.clear()
        self.error = None
        self._thread = threading.Thread(target=self._run, name="wavi-server", daemon=True)
        self._thread.start()

    def request_stop(self) -> None:
        self._stop.set()

    def request_reset(self) -> None:
        self._reset_pending.set()

    def stop(self, join_timeout_s: float = 2.0) -> None:
        self._stop.set()
        self.stop_tee()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_s)
            self._thread = None

    def wait_ready(self, timeout: Optional[float] = None) -> bool:
        return self._ready.wait(timeout=timeout)

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ----- commands / local copy

    def send(self, cmd: dict) -> None:
        self._transport.write((json.dumps(cmd) + "\n").encode())

    def start_tee(self, path: str) -> None:
        with self._tee_lock:
            if self._tee is not None:
                self._tee.close()
            self._tee = open(path, "wb")
            # A recording must start with hello (clock reference, channels);
            # re-stamp it "now" so the file's time zero is the record start.
            hello = dict(self.hello, t=self._last_t or self.hello.get("t", 0.0))
            self._tee.write((json.dumps(hello) + "\n").encode())

    def stop_tee(self) -> None:
        with self._tee_lock:
            if self._tee is not None:
                self._tee.close()
                self._tee = None

    # ----- main loop

    def _run(self) -> None:
        buf = bytearray()
        rows: list[np.ndarray] = []
        imu_rows: list[np.ndarray] = []
        last_flush = time.monotonic()
        try:
            while not self._stop.is_set():
                buf += self._transport.read_some(65536)
                end = buf.rfind(b"\n")
                if end < 0:
                    continue
                chunk = bytes(buf[:end + 1])
                del buf[:end + 1]
                with self._tee_lock:
                    if self._tee is not None:
                        self._tee.write(chunk)
                for line in chunk.splitlines():
                    if not line:
                        continue
                    try:
                        ev = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    self._handle(ev, rows, imu_rows)
                now = time.monotonic()
                if now - last_flush >= 0.02:
                    self._flush(rows, imu_rows)
                    last_flush = now
        except Exception as exc:  # noqa: BLE001
            if not self._stop.is_set():
                self.error = ("no data for a while (timed out)"
                              if isinstance(exc, socket.timeout) else str(exc) or type(exc).__name__)
                print(f"[ServerReader] terminating: {self.error}")
        finally:
            self.stop_tee()
            self._bus.close()
            self._imu_bus.close()

    def _handle(self, ev: dict, rows: list, imu_rows: list) -> None:
        k = ev.get("k")
        self._last_t = ev.get("t", self._last_t)
        if k == "wsk":
            rows.append(np.asarray(ev["raw"], dtype=np.float32) * self._mv_per_code)
        elif k == "mav":
            mtype = ev.get("type")
            d = ev.get("d", {})
            if mtype == "ATTITUDE":
                self._attitude[:] = (d.get("roll", 0), d.get("pitch", 0), d.get("yaw", 0))
            elif mtype == "RAW_IMU":
                imu_rows.append(np.array(
                    [d.get(f, 0) for f in ("xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro")]
                    + list(self._attitude), dtype=np.float32))
        elif k == "status":
            self._bus.set_metadata(server_status=ev)
        elif k == "hello":
            self.hello = ev
            self._mv_per_code = float(ev.get("mv_per_code", MV_PER_CODE))
            self._bus.set_metadata(server_hello=ev, t0_unix=time.time())
        elif k == "txt":
            print(f"[{ev.get('src', '?')}] {ev.get('msg', '')}")
        elif k == "ack":
            self._bus.set_metadata(server_ack=ev)
            if not ev.get("ok", True):
                print(f"[ServerReader] command failed: {ev}")

    def _flush(self, rows: list, imu_rows: list) -> None:
        if imu_rows:
            self._imu_bus.publish(np.vstack(imu_rows))
            imu_rows.clear()
        if not rows:
            return
        block = np.vstack(rows)
        rows.clear()
        if self._reset_pending.is_set():
            self._reset_pending.clear()
            self._v0_rows = []
            self._ready.clear()
        if not self._ready.is_set():
            # V0 = mean of the first ~Fs/4 samples, as in Reader._compute_v0.
            self._v0_rows.append(block)
            have = sum(len(r) for r in self._v0_rows)
            if have >= max(1, int(round(self._fs / 4))):
                self._v0_version += 1
                v0 = np.vstack(self._v0_rows).mean(axis=0).astype(np.float32)
                self._bus.set_metadata(V0=v0, v0_version=self._v0_version, fs=self._fs,
                                       nch=self._nch)
                self._v0_rows = []
                self._ready.set()
            return
        self._bus.publish(block)
