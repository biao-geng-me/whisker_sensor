"""JSON-lines event stream: ``daq_server`` -> clients, and the recording format.

One JSON object per line, UTF-8, ``\\n`` terminated. The TCP stream and the
on-disk ``.jsonl`` recording are byte-for-byte the same format, so any client
can replay a recording by reading the file instead of the socket.

Every event has ``k`` (kind) and ``t`` (server receive time, seconds on the
Pi's monotonic clock). Kinds:

``hello``   first line on connect / of a recording. Clock reference
            (``mono``/``unix`` sampled together), channel names, versions.
``wsk``     one whisker scan: ``seq``, ``tus`` (ESP32 micros, unwrapped),
            ``dur``, ``rail`` (railmask), ``flg``, ``raw`` (32 ADC codes).
            mV at the ADC = raw * ``mv_per_code`` from ``hello``.
``mav``     one MAVLink message: ``type`` (e.g. ``RAW_IMU``), ``src``
            ``[sysid, compid]``, ``d`` (message fields).
``txt``     a log line: ``src`` (``esp32`` / ``server``), ``msg``.
``mark``    user annotation: ``text``.
``status``  ~1 Hz health: rates, counters, recording state.
``info``    ESP32 INFO frame fields.

Commands, client -> server, are JSON lines too::

    {"cmd": "rec_start", "tag": "tank_run1"}
    {"cmd": "rec_stop"}
    {"cmd": "mark", "text": "tap W3"}
    {"cmd": "esp32", "data": "b"}        raw bytes to the firmware
"""

from __future__ import annotations

import json
from typing import Any, Iterator, TextIO

WIRE_VERSION = 1
DEFAULT_PORT = 5600


def _default(o: Any) -> Any:
    if isinstance(o, (bytes, bytearray)):
        return o.hex()
    return str(o)


def dumps(ev: dict) -> bytes:
    return (json.dumps(ev, separators=(",", ":"), default=_default) + "\n").encode()


def iter_file(f: TextIO) -> Iterator[dict]:
    """Yield events from a recording, skipping a truncated last line."""
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            continue


class LineSplitter:
    """Reassemble newline-delimited JSON from arbitrary byte chunks."""

    def __init__(self, max_line: int = 1 << 20) -> None:
        self._buf = bytearray()
        self._max = max_line
        self.n_bad = 0

    def feed(self, data: bytes) -> Iterator[dict]:
        self._buf += data
        while True:
            i = self._buf.find(b"\n")
            if i < 0:
                if len(self._buf) > self._max:
                    self._buf.clear()
                    self.n_bad += 1
                return
            line = bytes(self._buf[:i])
            del self._buf[:i + 1]
            if not line.strip():
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                self.n_bad += 1
