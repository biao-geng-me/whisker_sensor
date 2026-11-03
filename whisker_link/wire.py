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
``mark``    user annotation: ``text``. A recording's last line is a mark with
            ``rec`` = ``{event: stop, why, by, secs}`` (how it ended).
``status``  ~1 Hz health: rates, counters, recording state (``rec``: ``on``,
            ``path``, ``secs``, ``events``; server 0.2 also ``tag``, ``by``,
            ``duration``, ``left``, ``unix`` while on, and ``last`` = the
            previous one's ``path``, ``why``, ``by``, ``secs``, ``unix``).
``info``    ESP32 INFO frame fields.
``ack``     the reply to a command: ``cmd``, ``ok``, ``error`` if not ok.

Commands, client -> server, are JSON lines too::

    {"cmd": "rec_start", "tag": "tank_run1"}
    {"cmd": "rec_start", "tag": "run2", "duration": 60, "who": "wavi_py@laptop"}
    {"cmd": "rec_start", "tag": "run3", "force": true}
    {"cmd": "rec_stop"}
    {"cmd": "rec_stop", "path": "whisker_20261103_120000_run2.jsonl"}
    {"cmd": "mark", "text": "tap W3"}
    {"cmd": "esp32", "data": "b"}        raw bytes to the firmware
    {"cmd": "status"}                    the status (without rates) in the ack
    {"cmd": "rec_flag", "keep": false, "note": "overshot"}   the newest recording: bad
    {"cmd": "rec_flag", "file": "whisker_....jsonl", "keep": true}   good again

``rec_start`` (server 0.2) is refused (``ok`` false, ``busy`` true, the running
one in ``rec``) while a recording runs, unless ``force``, which closes it and
opens the new file (the ack's ``replaced``). ``duration`` (s): the server stops
the recording itself then. ``who`` names the client in the status. ``rec_stop``
with ``path`` (or just the file name) stops only that recording. ``rec_flag``
(server 0.3) adds a line to the recording index (``whisker_link/rec_index.py``);
the ack has ``file`` and ``keep``.
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
