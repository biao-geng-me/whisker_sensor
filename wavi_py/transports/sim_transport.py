"""Simulated 16-whisker array: test the GUI and scripts without sensors.

``SimTransport`` hands ``ServerReader`` the same JSON-lines event stream as the
ESP32 transport (``hello``, ``wsk`` scans with raw ADC codes, a 1 Hz
``status`` with the error counters), so everything downstream (V0, Re-zero,
the measured rate, the status line, Rec with its local .jsonl copy) runs as
with the real array. There is no IMU.

What it sends is set by a ``SimSignals`` object, which can be changed while
streaming (the GUI's signal panel, or a script):

- per gauge: live or open (railed at full scale, rail bit set), a baseline
  (mV), a sine (amplitude mV, frequency Hz, phase) and white noise (mV RMS);
- for all gauges: a linear drift (mV/s) and mains hum (amplitude mV at 50 or
  60 Hz; each gauge picks it up with its own fixed coupling 0.3..1.5 and
  phase). The hum is evaluated at the scan times, so a scan rate below 2x the
  mains frequency aliases it (50 Hz at 80 Hz scans shows at 30 Hz);
- taps: a decaying ring on one whisker (amplitude, frequency, decay time,
  direction split between gauges A and B), like a contact;
- faults: the scan flags (ADC recovered / CRC retry / stale / re-init), a
  sequence gap, a link CRC error, and a dropout (no scans for a while);
- the scan rate (a change restarts the scan counters, as a re-flashed ESP32
  would, so the reader re-measures the rate in one step);
- a reboot (the ``reboot`` fault): the array comes back in the firmware's
  boot default (wide 250 Hz, OSR 128, selects 0123457; it also starts in it),
  and streams again ~2 s later, as when the ESP32 transport kicks it with ``B``;
- an ADC lock-up (``adc_lockup``, as on the ROV: the ADC stops converting
  until its PCB is power-cycled): no scans; the legacy scan reports
  ``[halt #N] UNRECOVERED ...`` every 1.5 s, a locked mode five
  ``[halt] no conversion ...`` 0.45 s apart and then ``[halt #N] UNRECOVERED``
  every 5 s (firmware 2e9d8dc), and a reboot meanwhile reports
  ``CLOCK readback = 0x00FF (NOT accepted)``. ``pcb_power_cycle`` ends it:
  ``[halt #N] recovered at L3`` (legacy) and scans again, the first one with
  the RECOVERED flag.

A 1 Hz heartbeat (the firmware's BEAT: uptime, DRDY rate, halt counts, reset
reason) goes into the status as ``esp32.beat`` and to the fault model, also
while nothing streams; a locked-up ADC shows 0 DRDY/s and, after an
UNRECOVERED, the ADC-down flag.

The ADC / ESP32 fault model (``whisker_link/esp32_fault.py``) runs on these
events as in the ESP32 transport: ``esp32.fault`` in the status, and a
``mark`` event per fault onset / clear / warning.

The ``esp32`` command (raw bytes to the firmware) understands the firmware's
``S<digits>;`` (mux selects; the outputs of a skipped select read railed),
``M<mode> [settle_us] [osr] [period_us] [spi_hz];`` (scan mode, checked like
the firmware: a bad combination is refused with an ``M: ...`` text line) and
``B`` (an INFO frame). Legacy mode scans at ``SimSignals.rate_hz``; the locked
modes at their own rate (62.5 / 125 / 250 Hz), with a 1 Hz HEALTH record in
the status. INFO is sent when streaming starts and after each M, as the
firmware does.

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

from wavi_py.scan_mode import BOOT_MODE, BOOT_SEL_MASK, LOCK_DEFAULTS, info_for, lock_params
from wavi_py.transports.base import Transport
from whisker_link import esp32_frame as ef
from whisker_link import wire
from whisker_link.esp32_fault import FaultMonitor, mark_event
from whisker_link.channels import CHANNEL_NAMES, NCH, channels_of_selects
from whisker_link.dome_pose import LIVE_WHISKERS

RAIL_CODE = (1 << 23) - 1           # a railed (open) gauge: positive full scale
RATES_HZ = (40.0, 80.0, 200.0, 400.0, 1000.0)
_MAX_BLOCK = 400                    # scans per read_some call at most (catch-up after a stall)
_REBOOT_S = 2.0                     # a rebooted array streams again after this (the host's kick)


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
    drift_mv_s: float = 0.0
    hum_mv: float = 0.0
    hum_hz: float = 50.0
    noise_mv: np.ndarray = field(default_factory=lambda: np.full(NCH, 0.2))
    hum_coupling: np.ndarray = field(default_factory=lambda: np.ones(NCH))
    hum_phase_rad: np.ndarray = field(default_factory=lambda: np.zeros(NCH))
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
    pending_reboot: bool = False
    adc_locked: bool = False        # the ADC stopped converting (until a PCB power cycle)
    t_ref: float = field(default_factory=time.monotonic)    # drift is zero here

    def __post_init__(self) -> None:
        rng = np.random.default_rng(1)
        self.baseline_mv = rng.uniform(400, 650, NCH)      # where a real bridge sits
        self.phase_rad = rng.uniform(0, 2 * math.pi, NCH)
        self.hum_coupling = rng.uniform(0.3, 1.5, NCH)
        self.hum_phase_rad = rng.uniform(-0.3, 0.3, NCH)   # same mains, small wiring differences
        self._noise_spread = np.exp(rng.uniform(np.log(0.05), np.log(0.5), NCH))
        self.preset("demo")

    # ----- presets and edits (any thread)

    PRESETS = ("demo", "all live", "all live, same", "all open",
               "noise floor", "noisy gauges", "hum 50 Hz", "hum 60 Hz", "bench", "dome")
    PRESET_HELP = (
        "demo: 7 gauges, 0.5..12 Hz, 0.2 mV noise\n"
        "all live: every gauge, 1..16 Hz by whisker\n"
        "all live, same: every gauge 2 Hz 3 mV\n"
        "all open: nothing plugged in\n"
        "noise floor: every gauge live, no sine, noise 0.05..0.5 mV (fixed per gauge)\n"
        "noisy gauges: demo plus W3_B 3 mV, W6_A 2 mV, W12 A/B 5 mV of noise\n"
        "hum 50 Hz / hum 60 Hz: demo plus 2 mV of mains hum\n"
        "bench: every gauge live with a 0.5 mV sine at 1..16 Hz, noise 0.05..0.5 mV, "
        "1 mV of 50 Hz hum, 0.05 mV/s drift\n"
        "dome: the 12 gauges wired on the ROV (W3, W8, W10, W14 open), 12 mV sines at\n"
        "0.2..0.75 Hz with B a quarter cycle after A, so each whisker tip circles in the Dome view")
    _DEMO = (("W1_A", 5, 1), ("W1_B", 3, 1), ("W3_B", 4, 2), ("W4_A", 2, 5),
             ("W4_B", 2, 5), ("W8_B", 1, 12), ("W9_A", 6, 0.5))

    def preset(self, name: str) -> None:
        """Replace the signals with a named set (see ``PRESET_HELP``)."""
        if name not in self.PRESETS:
            raise ValueError(f"unknown preset {name!r}")
        with self.lock:
            self.live[:] = False
            self.amp_mv[:] = 0
            self.freq_hz[:] = 1
            self.noise_mv[:] = 0.2
            self.hum_mv, self.hum_hz, self.drift_mv_s = 0.0, 50.0, 0.0
            if name in ("demo", "noisy gauges", "hum 50 Hz", "hum 60 Hz"):
                for ch, amp, f in self._DEMO:
                    i = CHANNEL_NAMES.index(ch)
                    self.live[i], self.amp_mv[i], self.freq_hz[i] = True, amp, f
            if name == "noisy gauges":
                for ch, noise in (("W3_B", 3), ("W6_A", 2), ("W12_A", 5), ("W12_B", 5)):
                    i = CHANNEL_NAMES.index(ch)
                    self.live[i], self.noise_mv[i] = True, noise
            elif name in ("hum 50 Hz", "hum 60 Hz"):
                self.hum_mv, self.hum_hz = 2.0, 50.0 if name == "hum 50 Hz" else 60.0
            elif name == "noise floor":
                self.live[:] = True
                self.noise_mv[:] = self._noise_spread
            elif name == "bench":
                self.live[:] = True
                self.amp_mv[:] = 0.5
                self.freq_hz[:] = np.repeat(np.arange(1, NCH // 2 + 1), 2)
                self.noise_mv[:] = self._noise_spread
                self.hum_mv, self.drift_mv_s = 1.0, 0.05
            elif name == "dome":
                for k, w in enumerate(LIVE_WHISKERS):
                    a, b = 2 * (w - 1), 2 * (w - 1) + 1
                    self.live[a] = self.live[b] = True
                    self.amp_mv[a] = self.amp_mv[b] = 12.0
                    self.freq_hz[a] = self.freq_hz[b] = 0.2 + 0.05 * k
                    self.phase_rad[b] = self.phase_rad[a] - math.pi / 2
                    self.noise_mv[a] = self.noise_mv[b] = 0.3
            elif name == "all live":
                self.live[:] = True
                self.amp_mv[:] = 3
                self.freq_hz[:] = np.repeat(np.arange(1, NCH // 2 + 1), 2)
            elif name == "all live, same":
                self.live[:] = True
                self.amp_mv[:] = 3
                self.freq_hz[:] = 2

    def set_channel(self, ch: int, *, live: bool | None = None, amp_mv: float | None = None,
                    freq_hz: float | None = None, noise_mv: float | None = None) -> None:
        with self.lock:
            if noise_mv is not None:
                self.noise_mv[ch] = noise_mv
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
        link_crc, dropout (no scans for ``secs``), reboot (back in the boot default),
        adc_lockup (the ADC stops converting) or pcb_power_cycle (ends it)."""
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
            elif kind == "reboot":
                self.pending_reboot = True
            elif kind == "adc_lockup":
                self.adc_locked = True
            elif kind == "pcb_power_cycle":
                self.adc_locked = False
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
            if self.hum_mv:
                mv += self.hum_mv * self.hum_coupling * np.sin(
                    2 * math.pi * self.hum_hz * t[:, None] + self.hum_phase_rad)
            noise = self.noise_mv.copy()
        if noise.any():
            mv += np.random.standard_normal(mv.shape) * noise
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
        # the emulated firmware: scan mode, selects, binary streaming
        self._fw_in: collections.deque[str] = collections.deque()   # from the GUI thread
        self._fw_cmd = ""                          # an M / S command waiting for its ';'
        self._mode, self._sel_mask, self._lk = BOOT_MODE, BOOT_SEL_MASK, lock_params(BOOT_MODE)
        self._rate = 1e6 / self._lk[3]
        self._boot_t = self._t0                    # the simulated ESP32's uptime starts here
        self._reset_reason = 1                     # poweron
        self._next_beat = self._t0 + 1.0
        self._beat_scans = 0
        self._n_unrec = 0
        self._lk_halts = 0                         # locked-mode halts in a row (UNRECOVERED after 5)
        self._adc_down = False                     # an UNRECOVERED since the lock-up began
        self._streaming_at = 0.0                   # rebooted: not streaming before this (monotonic)
        self._last_health = self._t0
        self._locked = False                       # ADC lock-up in effect (seen by the firmware)
        self._n_halt = 0
        self._next_halt_msg = 0.0
        self._beat_summary: dict | None = None     # latest heartbeat (status esp32.beat)
        self._faults = FaultMonitor()
        self._faults.link(True, self._t0)
        self._emit(self._hello())
        self._emit(self._info())                   # the transport's 'B' on connect

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
        while self._fw_in:
            self._firmware(self._fw_in.popleft(), now)
        if self.signals.pending_reboot:
            self._reboot(now)
        if now >= self._next_beat:
            self._next_beat = now + 1.0
            self._beat(now)
        self._faults.poll(now)
        for fev in self._faults.take_events():
            self._emit(mark_event(fev, now))
        if now < self._streaming_at:               # rebooted, not kicked yet
            self._next_t = self._streaming_at
            if now - self._last_status >= 1.0:
                self._emit(self._status(now))
            return self._drain()
        if self._adc_locked(now):                  # nothing converts: no scans
            self._next_t = now + 0.05
            if now - self._last_status >= 1.0:
                self._emit(self._status(now))
            return self._drain()
        if self._streaming_at:                     # the kick: streaming again
            self._streaming_at = 0.0
            self._emit(self._info())
        rate = 1e6 / self._lk[3] if self._mode else self.signals.rate_hz
        if rate != self._rate:
            # like re-flashing the ESP32 at a new rate: counters restart, so the reader
            # drops its rate history and re-measures once instead of ramping over 10 s
            self._rate = rate
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
        return self._drain()

    def _drain(self) -> bytes:
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

    # ----- the emulated firmware (reader thread)

    def _info(self) -> dict:
        info = info_for(self._mode, self._sel_mask, self._lk)
        return {"k": "info", "t": time.monotonic(), **info.__dict__}

    def _say(self, msg: str, now: float) -> None:
        self._emit({"k": "txt", "t": now, "src": "esp32", "msg": msg})
        self._faults.on_text(msg, now)

    def _firmware(self, data: str, now: float) -> None:
        """Bytes written to the ESP32: S<digits>; M<args>; B (others are ignored)."""
        for c in data:
            if self._fw_cmd:
                if c in ";\n":
                    self._fw_arg_cmd(self._fw_cmd, now)
                    self._fw_cmd = ""
                elif c != "\r":
                    self._fw_cmd += c
            elif c in "MS":
                self._fw_cmd = c
            elif c == "B" and now >= self._streaming_at:
                self._emit(self._info())

    def _fw_arg_cmd(self, c: str, now: float) -> None:
        if c[0] == "S":
            digits = {int(d) for d in c[1:] if d in "01234567"}
            self._sel_mask = sum(1 << d for d in digits) or 0xFF
            return
        usage = "M: usage M<0|1|2> [settle_us] [osr] [period_us] [spi_hz];"
        try:
            v = [int(x) for x in c[1:].split()[:5]]
        except ValueError:
            v = []
        if not v or v[0] not in (0, 1, 2):
            self._say(usage, now)
            return
        if v[0] in LOCK_DEFAULTS:
            v += [0] * (5 - len(v))
            lk = lock_params(*v)
            if isinstance(lk, str):
                self._say(f"M: {lk}", now)
                return
            self._lk = lk
        else:
            self._lk = None
        self._mode = v[0]
        self._say(f"mode {self._mode} (simulated)", now)
        if now >= self._streaming_at:
            self._emit(self._info())

    def _reboot(self, now: float) -> None:
        self.signals.pending_reboot = False
        self._mode, self._sel_mask, self._fw_cmd = BOOT_MODE, BOOT_SEL_MASK, ""
        self._lk = lock_params(BOOT_MODE)
        self._seq = self._tus = 0
        self._boot_t, self._reset_reason = now, 3     # sw (esp_restart)
        self._next_beat = now + 1.0
        self._beat_scans = self._n_unrec = self._lk_halts = 0
        self._streaming_at = now + _REBOOT_S
        self._say("whisker_daq (SIMULATED) rebooted", now)
        self._say("Device ID      = 0x2405  (ADS131M04)", now)
        self._say("CLOCK readback = 0x00FF  (NOT accepted)" if self._locked
                  else "CLOCK readback = 0x0F02  (config ACCEPTED)", now)

    def _adc_locked(self, now: float) -> bool:
        """Follow SimSignals.adc_locked; while locked, say what the firmware says."""
        locked = self.signals.adc_locked
        if locked != self._locked:
            self._locked = locked
            self._next_halt_msg = now
            if not locked:                         # PCB power-cycled: the firmware recovers
                if not self._mode:
                    self._n_halt += 1
                    self._say(f"[halt #{self._n_halt}] recovered at L3  (L1=0 L2=0 L3=1"
                              f" fail={self._n_halt - 1} crcbad=0 stale=0)", now)
                self.signals.fault("recovered")
                self._next_t = now
            self._lk_halts, self._adc_down = 0, False
        if locked and now >= self._next_halt_msg:
            self._n_halt += 1
            if self._mode and self._lk_halts < 5:
                self._lk_halts += 1
                self._say(f"[halt] no conversion for 50 ms in "
                          f"{ef.MODE_NAMES[self._mode]} mode: CLKIN restart + re-init", now)
                self._next_halt_msg = now + 0.45       # a recovery takes ~0.45 s
            else:
                self._n_unrec += 1
                self._adc_down = True
                self._say(f"[halt #{self._n_halt}] UNRECOVERED  ID=0x2405 CLOCKrb=0x00FF"
                          " crcbad=0  -> power-cycle the PCB with the ESP32 already running", now)
                self._next_halt_msg = now + (5.0 if self._mode else 1.5)
        return locked

    def _beat(self, now: float) -> None:
        """The firmware's 1 Hz heartbeat, to the fault model and the status."""
        dead = self._locked
        b = ef.Beat(1, int((now - self._boot_t) * 1000), self._mode, 1,
                    "B" if now >= self._streaming_at else "g",
                    (ef.BEAT_ADC_DOWN if dead and self._adc_down else 0)
                    | (0 if dead else ef.BEAT_BOOT_CLOCK_OK), 1000,
                    0 if dead else (3125 if not self._mode else 8_000_000 // 2 // self._lk[0]),
                    self._n_halt, self._n_unrec, 0, 0x00FF if dead else 0x0F0A, 0x2405,
                    self._beat_scans, self._reset_reason, 0x0F0A)
        self._beat_summary = b.summary()
        self._faults.on_beat(b, now)

    def _health(self) -> dict | None:
        if not self._mode:
            return None
        osr, _, n, period = self._lk[:4]
        return ef.Health(1, self._mode, 380 if self._mode == 2 else 250, 30, 180, 160, 17, 4,
                         int(1e6 / period * n), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000).summary()

    def _scans(self, t: np.ndarray, now: float) -> None:
        mv, rail = self.signals.block(t)
        if self._sel_mask != 0xFF:                 # outputs of a skipped select read railed
            skipped = sorted(set(range(NCH)) - set(channels_of_selects(self._sel_mask)))
            mv[:, skipped] = np.nan
            rail |= sum(1 << i for i in skipped)
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
            self._faults.on_scan(flags if k == 0 else 0, now)
            self._emit({"k": "wsk", "t": now, "seq": self._seq, "tus": self._tus,
                        "dur": int(1e6 / self._rate) if self._mode else 12_300, "rail": rail, "flg": flags if k == 0 else 0,
                        "raw": codes[k].tolist()})
            self._seq += 1
            self._tus = (self._tus + dt_us) & 0xFFFFFFFF
        self._n["scans"] += len(t)
        self._beat_scans += len(t)
        self._scans_since_status += len(t)
        self._last_scan = now

    def _status(self, now: float) -> dict:
        dt = now - self._last_status
        hz = round(self._scans_since_status / dt, 1) if dt > 0 else 0.0
        self._last_status, self._scans_since_status = now, 0
        health = self._health()
        return {"k": "status", "t": now, "clients": 1,
                "rec": {"on": False, "path": None, "secs": 0, "events": 0},
                "hz": {"wsk": hz},
                "esp32": {**self._n, "crc_fail": self._crc_fail, "skipped": 0,
                          "streaming": now - self._last_scan < 1.0,
                          "cfg": ef.info_cfg(info_for(self._mode, self._sel_mask,
                                                      self._lk).__dict__),
                          **({"health": health} if health else {}),
                          **({"beat": self._beat_summary} if self._beat_summary else {}),
                          "fault": self._faults.status(now)}}

    def _command(self, cmd: dict) -> None:
        name = cmd.get("cmd")
        now = time.monotonic()
        reply = {"k": "ack", "t": now, "cmd": name, "ok": True}
        if name == "esp32":
            self._fw_in.append(str(cmd.get("data", "")))    # read by the reader thread
        elif name == "mark":
            self._emit({"k": "mark", "t": now, "text": str(cmd.get("text", ""))})
        elif name not in ("rec_start", "rec_stop", "status"):
            reply.update(ok=False, error=f"unknown command {name!r}")   # local tee records
        self._emit(reply)
