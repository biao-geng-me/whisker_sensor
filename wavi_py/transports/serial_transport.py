"""Serial transport — direct USB connection to the Arduino.

Reuses defaults from ``hx711_array/arduino_reader.py`` (same baud, same DTR
behavior, same post-open boot sleep).
"""

from __future__ import annotations

import time

import serial

from wavi_py.config import DEFAULT_BAUDRATE
from wavi_py.transports.base import Transport


_POST_OPEN_SLEEP_S = 1.5   # Arduino auto-resets when DTR toggles on open


class SerialTransport(Transport):
    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        read_timeout_s: float | None = None,
        write_timeout_s: float = 1.0,
    ):
        self._serial: serial.Serial | None = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=read_timeout_s,         # None = block forever
            write_timeout=write_timeout_s,
            dsrdtr=False,
            rtscts=False,
            xonxoff=False,
        )
        time.sleep(_POST_OPEN_SLEEP_S)

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def read_exact(self, n: int) -> bytes:
        if self._serial is None:
            raise ConnectionError("transport closed")
        data = self._serial.read(n)
        if len(data) != n:
            raise ConnectionError(f"transport closed after {len(data)}/{n} bytes")
        return data

    def read_some(self, max_n: int) -> bytes:
        if self._serial is None:
            raise ConnectionError("transport closed")
        n = self._serial.in_waiting or 1
        return self._serial.read(min(n, max_n))

    def write(self, data: bytes) -> None:
        if self._serial is None:
            raise ConnectionError("transport closed")
        self._serial.write(data)
        self._serial.flush()

    def bytes_available(self) -> int:
        if self._serial is None:
            return 0
        return self._serial.in_waiting

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
