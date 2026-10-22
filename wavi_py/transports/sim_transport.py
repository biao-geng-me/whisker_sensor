"""Simulated 16-whisker array: test the GUI and scripts without sensors.

``SimTransport`` hands ``ServerReader`` the same JSON-lines event stream as the
ESP32 transport (``hello``, ``wsk`` scans with raw ADC codes, a 1 Hz
``status`` with the error counters), so everything downstream (V0, Re-zero,
the measured rate, the status line, Rec with its local .jsonl copy) runs as
with the real array. There is no IMU.

What it sends is set by a ``SimSignals`` object, which can be changed while
streaming (the GUI's signal panel, or a script):

- per gauge: live or open (railed at full scale, rail bit set), a baseline
  (mV), and a sine (amplitude mV, frequency Hz, phase);
- for all gauges: white noise (mV RMS) and a linear drift (mV/s);
- taps: a decaying ring on one whisker (amplitude, frequency, decay time,
  direction split between gauges A and B), like a contact;
- faults: the scan flags (ADC recovered / CRC retry / stale / re-init), a
  sequence gap, a link CRC error, and a dropout (no scans for a while);
- the scan rate (a change restarts the scan counters, as a re-flashed ESP32
  would, so the reader re-measures the rate in one step).

Scans are generated on demand in ``read_some`` from the monotonic clock, in
blocks, with the scan timestamps (``tus``) at the nominal rate.
"""

from __future__ import annotations

import collections
import json
import math
import socket
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from wavi_py.transports.base import Transport
from whisker_link import esp32_frame as ef
from whisker_link import wire
from whisker_link.channels import CHANNEL_NAMES, NCH

RAIL_CODE = (1 << 23) - 1           # a railed (open) gauge: positive full scale
RATES_HZ = (40.0, 80.0, 200.0, 400.0, 1000.0)
_MAX_BLOCK = 400                    # scans per read_some call at most (catch-up after a stall)


@dataclass
class Tap:
    whisker: int                    # 0-based
    amp_mv: float
    freq_hz: float
    decay_s: float
    angle_rad: float                # gauge A gets cos, gauge B sin
    t0: float                       # monotonic start


@dataclass
class SimSignals:
    """What the simulated array sends. Safe to change from another thread
    between scans (each block reads a consistent copy under ``lock``)."""

    rate_hz: float = 80.0
    noise_mv: float = 0.2
    drift_mv_s: float = 0.0
    live: np.ndarray = field(default_factory=lambda: np.zeros(NCH, bool))
    baseline_mv: np.ndarray = field(default_factory=lambda: np.zeros(NCH))
    amp_mv: np.ndarray = field(default_factory=lambda: np.zeros(NCH))
    freq_hz: np.ndarray = field(default_factory=lambda: np.ones(NCH))
    phase_rad: np.ndarray = field(default_factory=lambda: np.zeros(NCH))
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    taps: list[Tap] = field(default_factory=list)
    pending_flags: int = 0          # OR-ed into the next scan
    pending_seq_gap: int = 0        # sequence numbers to skip before the next scan
    pending_link_crc: int = 0       # link CRC errors to count
    dropout_until: float = 0.0      # monotonic; no scans before this
    t_ref: float = field(default_factory=time.monotonic)    # drift is zero here

    def __post_init__(self) -> None:
        rng = np.random.default_rng(1)
        self.baseline_mv = rng.uniform(400, 650, NCH)      # where a real bridge sits
        self.phase_rad = rng.uniform(0, 2 * math.pi, NCH)
        self.preset("demo")

    # ----- presets and edits (any thread)

    PRESETS = ("demo", "all live", "all live, same", "all open")

    def preset(self, name: str) -> None:
        """demo: a few gauges with distinct frequencies; all live: every gauge,
        1..16 Hz by whisker; all live, same: every gauge 2 Hz 3 mV; all open."""
        with self.lock:
            self.live[:] = False
            self.amp_mv[:] = 0
            self.freq_hz[:] = 1
            if name == "demo":
                for ch, amp, f in (("W1_A", 5, 1), ("W1_B", 3, 1), ("W3_B", 4, 2), ("W4_A", 2, 5),
                                   ("W4_B", 2, 5), ("W8_B", 1, 12), ("W9_A", 6, 0.5)):
                    i = CHANNEL_NAMES.index(ch)
                    self.live[i], self.amp_mv[i], self.freq_hz[i] = True, amp, f
            elif name == "all live":
                self.live[:] = True
                self.amp_mv[:] = 3
                self.freq_hz[:] = np.repeat(np.arange(1, NCH // 2 + 1), 2)
            elif name == "all live, same":
                self.live[:] = True
                self.amp_mv[:] = 3
                self.freq_hz[:] = 2
            elif name != "all open":
                raise ValueError(f"unknown preset {name!r}")

    def set_channel(self, ch: int, *, live: bool | None = None, amp_mv: float | None = None,
                    freq_hz: float | None = None) -> None:
        with self.lock:
            if live is not None:
                self.live[ch] = live
            if amp_mv is not None:
                self.amp_mv[ch] = amp_mv
            if freq_hz is not None:
                self.freq_hz[ch] = freq_hz

    def tap(self, whisker: int, amp_mv: float = 20.0, freq_hz: float = 8.0, decay_s: float = 0.5,
            angle_rad: float | None = None) -> None:
        """A decaying ring on ``whisker`` (0-based), split between its gauges by angle."""
        if angle_rad is None:
            angle_rad = float(np.random.uniform(0, 2 * math.pi))
        with self.lock:
            self.taps.append(Tap(whisker, amp_mv, freq_hz, decay_s, angle_rad, time.monotonic()))

    def fault(self, kind: str, n: int = 1, secs: float = 3.0) -> None:
        """Inject: recovered / crc_retry / stale / reinit (scan flags), seq_gap,
        link_crc, or dropout (no scans for ``secs``)."""
        flags = {"recovered": ef.FLAG_RECOVERED, "crc_retry": ef.FLAG_CRC_RETRY,
                 "stale": ef.FLAG_STALE, "reinit": ef.FLAG_REINIT}
        with self.lock:
            if kind in flags:
                self.pending_flags |= flags[kind]
            elif kind == "seq_gap":
                self.pending_seq_gap += n
            elif kind == "link_crc":
                self.pending_link_crc += n
            elif kind == "dropout":
                self.dropout_until = time.monotonic() + secs
            else:
                raise ValueError(f"unknown fault {kind!r}")

    # ----- generation (the reader thread)

    def block(self, t: np.ndarray) -> tuple[np.ndarray, int]:
        """mV (len(t), NCH) at monotonic times ``t`` (NaN = open), and the railmask."""
        with self.lock:
            live = self.live.copy()
            mv = self.baseline_mv + self.drift_mv_s * (t[:, None] - self.t_ref)
            mv = mv + self.amp_mv * np.sin(2 * math.pi * self.freq_hz * t[:, None] + self.phase_rad)
            now = t[-1]
            self.taps = [tp for tp in self.taps if now - tp.t0 < 6 * tp.decay_s]
            for tp in self.taps:
                dt = t - tp.t0
                ring = np.where(dt >= 0, tp.amp_mv * np.exp(-np.maximum(dt, 0) / tp.decay_s)
                                * np.sin(2 * math.pi * tp.freq_hz * dt), 0.0)
                mv[:, 2 * tp.whisker] += math.cos(tp.angle_rad) * ring
                mv[:, 2 * tp.whisker + 1] += math.sin(tp.angle_rad) * ring
            noise = self.noise_mv
        if noise > 0:
            mv += np.random.normal(0, noise, mv.shape)
        rail = int(sum(1 << i for i in range(NCH) if not live[i]))
        return np.where(live, mv, np.nan), rail

    def take_faults(self) -> tuple[int, int, int]:
        with self.lock:
            out = (self.pending_flags, self.pending_seq_gap, self.pending_link_crc)
            self.pending_flags = self.pending_seq_gap = self.pending_link_crc = 0
        return out


class SimTransport(Transport):
    def __init__(self, signals: SimSignals | None = None):
        self.signals = signals if signals is not None else SimSignals()
        self._open = True
        self._out: collections.deque[bytes] = collections.deque()
        self._t0 = time.monotonic()
        self._next_t = self._t0                    # monotonic time of the next scan
        self._rate = self.signals.rate_hz
        self._seq = 0
        self._tus = 0                              # simulated micros(), wraps at 2**32
        self._last_scan = 0.0
        self._last_status = self._t0
        self._scans_since_status = 0
        self._n = {"scans": 0, "seq_gaps": 0, "adc_recov": 0, "adc_crc": 0, "adc_stale": 0,
                   "adc_reinit": 0}
        self._crc_fail = 0
        self._gap_pending = False                  # a dropout ended: count one seq gap
        self._emit(self._hello())

    # ----- Transport

    @property
    def is_open(self) -> bool:
        return self._open

    def read_some(self, max_n: int) -> bytes:
        if not self._open:
            raise ConnectionError("transport closed")
        now = time.monotonic()
        if self._next_t > now:
            time.sleep(min(self._next_t - now, 0.01))
            now = time.monotonic()
        if self.signals.rate_hz != self._rate:
            # like re-flashing the ESP32 at a new rate: counters restart, so the reader
            # drops its rate history and re-measures once instead of ramping over 10 s
            self._rate = self.signals.rate_hz
            self._next_t = now
            self._seq = self._tus = 0
            self._emit({"k": "txt", "t": now, "src": "sim", "msg": f"rate {self._rate:g} Hz"})
        n = int((now - self._next_t) * self._rate) + 1 if now >= self._next_t else 0
        if n:
            n = min(n, _MAX_BLOCK)
            t = self._next_t + np.arange(n) / self._rate
            self._next_t = t[-1] + 1 / self._rate
            if now >= self.signals.dropout_until:
                self._scans(t, now)
            else:                                  # the array keeps counting, nothing arrives
                self._seq += n
                self._tus = (self._tus + n * int(round(1e6 / self._rate))) & 0xFFFFFFFF
                self._gap_pending = True
        if now - self._last_status >= 1.0:
            self._emit(self._status(now))
        parts = []
        while self._out:                           # acks may arrive from the GUI thread
            parts.append(self._out.popleft())
        return b"".join(parts)

    def read_exact(self, n: int) -> bytes:
        raise NotImplementedError("the sim transport is read with read_some")

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
        self._open = False

    # ----- internals

    def _emit(self, ev: dict) -> None:
        self._out.append(wire.dumps(ev))

    def _hello(self) -> dict:
        now = time.monotonic()
        return {"k": "hello", "t": now, "mono": now, "unix": time.time(),
                "wire": wire.WIRE_VERSION, "server": "wavi_py-sim",
                "host": socket.gethostname(), "serial": "sim",
                "channels": CHANNEL_NAMES, "mv_per_code": ef.MV_PER_CODE}

    def _scans(self, t: np.ndarray, now: float) -> None:
        mv, rail = self.signals.block(t)
        codes = np.where(np.isnan(mv), RAIL_CODE, np.rint(np.nan_to_num(mv) / ef.MV_PER_CODE))
        codes = np.clip(codes, -RAIL_CODE - 1, RAIL_CODE).astype(np.int64)
        flags, gap, link_crc = self.signals.take_faults()
        if gap:
            self._seq += gap
        if gap or self._gap_pending:
            self._n["seq_gaps"] += 1
            self._gap_pending = False
        self._crc_fail += link_crc
        for name, bit in (("adc_recov", ef.FLAG_RECOVERED), ("adc_crc", ef.FLAG_CRC_RETRY),
                          ("adc_stale", ef.FLAG_STALE), ("adc_reinit", ef.FLAG_REINIT)):
            if flags & bit:
                self._n[name] += 1
        dt_us = int(round(1e6 / self._rate))
        for k in range(len(t)):
            self._emit({"k": "wsk", "t": now, "seq": self._seq, "tus": self._tus,
                        "dur": 12_300, "rail": rail, "flg": flags if k == 0 else 0,
                        "raw": codes[k].tolist()})
            self._seq += 1
            self._tus = (self._tus + dt_us) & 0xFFFFFFFF
        self._n["scans"] += len(t)
        self._scans_since_status += len(t)
        self._last_scan = now

    def _status(self, now: float) -> dict:
        dt = now - self._last_status
        hz = round(self._scans_since_status / dt, 1) if dt > 0 else 0.0
        self._last_status, self._scans_since_status = now, 0
        return {"k": "status", "t": now, "clients": 1,
                "rec": {"on": False, "path": None, "secs": 0, "events": 0},
                "hz": {"wsk": hz},
                "esp32": {**self._n, "crc_fail": self._crc_fail, "skipped": 0,
                          "streaming": now - self._last_scan < 1.0}}

    def _command(self, cmd: dict) -> None:
        name = cmd.get("cmd")
        now = time.monotonic()
        reply = {"k": "ack", "t": now, "cmd": name, "ok": True}
        if name == "mark":
            self._emit({"k": "mark", "t": now, "text": str(cmd.get("text", ""))})
        elif name not in ("rec_start", "rec_stop", "status"):
            reply.update(ok=False, error=f"unknown command {name!r}")   # local tee records
        self._emit(reply)
