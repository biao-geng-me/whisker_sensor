"""ESP32 / ADC fault model: one place that turns the firmware's signals into an
alarm the operator sees (``rpi/daq_server.py``, the wavi_py ESP32 and Sim
transports all feed one ``FaultMonitor``).

Why: on the ROV the whisker PCB (ADS131M04) runs from the vehicle 5 V and the
ESP32 from the Pi's USB. The ADC can lock up (SPI answers, the ID reads fine,
the CLOCK write doesn't stick, no conversions); its SYNC/RESET pin is tied
high, so only a PCB power cycle clears it. Until now that was a TEXT line
nobody saw.

Levels:

``fault`` (red, latched)
    ``adc_unrecovered``  ``[halt #N] UNRECOVERED ...`` (legacy scan), ``Boot recovery: FAILED``;
    ``clock_rejected``   ``CLOCK readback = 0x.... (NOT accepted)`` at boot, ``quiet mode
                         CLOCK not accepted``, ``M: CLOCK config not accepted``, an
                         ``[adc reset #N]`` re-init that was ``NOT accepted``;
    ``adc_no_response``  ``Device ID = 0x.... (NO RESPONSE)``; ``DRDY edges/200ms = 0``
                         and ``CLKIN: *** FAILED`` are ``adc_not_converting``;
    ``adc_not_converting`` locked modes: repeated ``[halt] no conversion ...`` lines
                         and no scan for ``HALT_STALL_S``;
    ``adc_unrecovered`` / ``adc_not_converting`` also from the heartbeat (BEAT frame,
                         firmware since 2026-10-23 brief 6): its ADC-down flag, or no
                         DRDY edges in ``DEAD_BEATS`` beats in a row (the ESP32 is
                         alive, the ADC isn't). The heartbeat summary is the
                         fault's detail only until a firmware line (``UNRECOVERED ...``)
                         gives one; a later heartbeat never replaces that line;
    ``esp32_hung``       the firmware sends heartbeats, the link is up, and nothing
                         (no beat, scan or text) came for ``BEAT_LOST_S``;
    ``no_scans``         no scan for ``NO_SCAN_S`` while the link is up;
    ``link_down``        (daq_server) the serial port has been gone for ``NO_SCAN_S``.
    A later INFO does not clear it (the transports re-send ``B`` while nothing
    streams, and each ``B`` gets an INFO). It clears only after ``CLEAR_S`` of
    clean streaming: scans with no gap over 1 s and no halt / ADC reset (text
    or scan flag). That happens after a PCB power cycle with the ESP32 running
    (the firmware then recovers by itself) or after a full power cycle.
``warn`` (amber, for ``WARN_HOLD_S`` after the last one)
    recovered halts (text, or a RECOVERED / REINIT scan flag with no text),
    ADC resets, reader-watchdog reboots, other ESP32 reboots (the banner, or
    the heartbeat's uptime going back, with its reset reason), ``[idf] E``
    lines (and raw ``E (...)`` lines from older firmware), dropped IDF log
    lines, and SYNC / EMPTY flagged scans above ``FLAG_WARN_FRAC`` over
    ``FLAG_WINDOW_S``.

Feed it ``link``, ``on_scan``, ``on_text``, ``on_beat``, ``expect_reset`` and ``poll``;
``take_events`` returns what happened since the last call (fault onsets and
clears, warnings, at most one warning per code per ``WARN_EVERY_S``), each
ready to go out as a ``mark`` (``mark_event``). ``status()`` is the
``esp32.fault`` field of the 1 Hz status record.

Pure stdlib, so it runs on the BlueOS Pi.
"""

from __future__ import annotations

import collections
import re
import time
from typing import Callable, Optional

from whisker_link import esp32_frame as ef

NO_SCAN_S = 5.0          # no scans this long while connected: fault
HALT_STALL_S = 1.5       # locked modes: halts and no scan this long: fault
HALT_STALL_N = 3         # ... after at least this many halt lines
CLEAR_S = 10.0           # clean streaming this long clears a fault
GAP_S = 1.0              # a scan gap this long restarts the clean-streaming clock
WARN_HOLD_S = 60.0       # level stays "warn" this long after the last warning
WARN_EVERY_S = 10.0      # at most one warning event per code this often
FLAG_WINDOW_S = 10.0
FLAG_WARN_FRAC = 0.01
FLAG_MIN_SCANS = 100
HISTORY_N = 10           # events kept in status()["history"]
_BEAT_DETAIL = "heartbeat: "   # detail of a heartbeat-raised fault (a firmware line beats it)
BEAT_LOST_S = 5.0        # heartbeat firmware: nothing at all this long: ESP32 hung
DEAD_BEATS = 2           # beats in a row with no DRDY edge: ADC not converting

ACTION_PCB = "power-cycle the whisker PCB (vehicle power)"
ACTION_LINK = "check the ESP32 USB cable to the Pi; if it persists, power-cycle the vehicle"
ACTION_HUNG = "reset the ESP32 (esp32_reset / EN); if it persists, power-cycle the vehicle"

# (regex, level, code, message); first match wins. Lines starting with '#' are
# the bench diagnostics' machine output (#BURSTERR ...), not the scan.
_TEXT_RULES = [
    (r"\[halt #\d+\] UNRECOVERED", "fault", "adc_unrecovered",
     "ADC not converting (halt not recovered)"),
    (r"Boot recovery: FAILED", "fault", "adc_unrecovered",
     "ADC stuck at boot (recovery failed)"),
    (r"\[adc reset #\d+\].*NOT accepted", "fault", "clock_rejected",
     "ADC CLOCK config not accepted after an ADC reset"),
    (r"CLOCK readback = 0x[0-9A-Fa-f]+\s+\(NOT accepted\)", "fault", "clock_rejected",
     "ADC CLOCK config not accepted at boot"),
    (r"CLOCK (config )?not accepted", "fault", "clock_rejected",
     "ADC CLOCK config not accepted"),
    (r"Device ID\s*= 0x[0-9A-Fa-f]+\s+\(NO RESPONSE\)", "fault", "adc_no_response",
     "ADC not answering on SPI"),
    (r"DRDY edges/200ms = 0\b", "fault", "adc_not_converting", "ADC not converting at boot"),
    (r"CLKIN: \*\*\* FAILED", "fault", "adc_not_converting", "ADC clock (CLKIN) failed"),
    (r"\[halt #\d+\] recovered at L(\d)", "warn", "adc_halt", "ADC halt, recovered at L{0}"),
    (r"\[halt\] no conversion for \d+ ms in (\w+) mode", "warn", "adc_halt",
     "ADC halt in {0} mode, re-initialised"),
    (r"\[adc reset #\d+\]", "warn", "adc_reset", "ADC had reset, re-initialised"),
    (r"^(previous reset: )?\[reader watchdog\]", "warn", "reader_watchdog",
     "ESP32 reader watchdog: ESP32 restarted"),
    (r"^\[idf\] \d+ log lines dropped", "warn", "idf_error", "ESP-IDF log lines dropped"),
    (r"^(\[idf\] )?E \(\d+\) ", "warn", "idf_error", "ESP-IDF error"),
]
_RULES = [(re.compile(rx), lvl, code, msg) for rx, lvl, code, msg in _TEXT_RULES]
_BANNER = re.compile(r"^\s*whisker_daq\b")
_RESET_REASON = re.compile(r"^reset reason (\d+)")
# esp_reset_reason(): the ones worth a warning on their own
_BAD_RESETS = {4: "panic", 5: "interrupt watchdog", 6: "task watchdog", 7: "watchdog",
               9: "brownout"}
# a more specific fault replaces a vaguer one
_PRIORITY = {"link_down": 0, "no_scans": 1, "esp32_hung": 2, "adc_not_converting": 3,
             "adc_no_response": 4, "clock_rejected": 4, "adc_unrecovered": 4}
_ACTIONS = {"no_scans": ACTION_LINK, "link_down": ACTION_LINK, "esp32_hung": ACTION_HUNG}


def classify(msg: str) -> Optional[tuple[str, str, str]]:
    """(level, code, message) of a firmware text line, or None."""
    s = msg.strip()
    if s.startswith("#"):
        return None
    for rx, lvl, code, text in _RULES:
        m = rx.search(s)
        if m:
            return lvl, code, text.format(*m.groups())
    return None


class FaultMonitor:
    def __init__(self, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time):
        self._clock, self._wall = clock, wall
        now = clock()
        self._up = False
        self._up_since = now            # link up (or down) since
        self._last_scan: Optional[float] = None
        self._streamed = False          # scans seen since the link came up
        self._clean_since: Optional[float] = None
        self._fault: Optional[dict] = None
        self._n_faults = 0
        self._warn_counts: collections.Counter = collections.Counter()
        self._last_warn: Optional[dict] = None
        self._warn_sent: dict[str, float] = {}       # code -> last event time
        self._warn_unsent: collections.Counter = collections.Counter()
        self._halts_since_scan = 0
        self._last_halt_text = -1e9
        self._flag_pending: Optional[float] = None   # RECOVERED/REINIT flag seen, no text yet
        self._flags = collections.deque()            # (t, flagged) per scan
        self._n_flagged = 0
        self._expect_reset_until = -1e9
        self._alive = -1e9                           # last reboot banner / watchdog line
        self._reboot_pending: Optional[float] = None  # unexpected banner, reason line not seen
        self._last_flag_check = -1e9
        self._history: collections.deque = collections.deque(maxlen=HISTORY_N)
        self._events: list[dict] = []
        self._beats = False                          # the firmware sends heartbeats
        self._last_rx = -1e9                         # last beat, scan or text
        self._last_beat = -1e9
        self._beat: Optional[ef.Beat] = None         # the latest heartbeat
        self._zero_beats = 0                         # beats in a row with no DRDY edge

    # ----- inputs

    def link(self, up: bool, now: Optional[float] = None) -> None:
        """The serial port / transport opened (True) or went away (False)."""
        now = self._clock() if now is None else now
        if up != self._up:
            self._up, self._up_since = up, now
            self._streamed = False
            self._clean_since = None
            self._halts_since_scan = 0

    def expect_reset(self, now: Optional[float] = None) -> None:
        """We reset the ESP32 on purpose (EN pulse): its reboot is no warning."""
        self._expect_reset_until = (self._clock() if now is None else now) + 10.0

    def on_scan(self, flags: int, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        self._last_rx = now
        if self._last_scan is None or now - self._last_scan > GAP_S or not self._streamed:
            self._clean_since = now
        self._last_scan = now
        self._streamed = True
        self._halts_since_scan = 0
        if flags & (ef.FLAG_RECOVERED | ef.FLAG_REINIT):
            self._clean_since = now
            if now - self._last_halt_text > 3.0 and self._flag_pending is None:
                self._flag_pending = now        # the text may still come; see poll()
        bad = bool(flags & (ef.FLAG_SYNC | ef.FLAG_EMPTY))
        self._flags.append((now, bad))
        self._n_flagged += bad
        while self._flags and now - self._flags[0][0] > FLAG_WINDOW_S:
            self._n_flagged -= self._flags.popleft()[1]

    def on_text(self, msg: str, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        self._last_rx = now
        s = msg.strip()
        if _BANNER.match(s):
            self._clean_since = None
            if (self._streamed and now > self._expect_reset_until
                    and now - self._warn_sent.get("reader_watchdog", -1e9) > 15.0):
                self._reboot_pending = now           # reported with its reset reason
            self._alive = now                    # booting: the no-scan clock starts here
            self._streamed = False
            return
        m = _RESET_REASON.match(s)
        if m:
            why = _BAD_RESETS.get(int(m.group(1)))
            if why or self._reboot_pending is not None:
                why = why or f"reset reason {m.group(1)}"
                self._warn(now, "esp32_reboot", f"ESP32 rebooted ({why})", s)
            self._reboot_pending = None
            return
        c = classify(s)
        if c is None:
            return
        level, code, text = c
        if code in ("adc_halt", "adc_reset"):
            self._last_halt_text = now
            self._flag_pending = None
            self._clean_since = None
            if "no conversion" in s:
                self._halts_since_scan += 1
        if code == "reader_watchdog":
            self._alive = now
        if code == "reader_watchdog" and s.startswith("previous reset"):
            if now - self._warn_sent.get("reader_watchdog", -1e9) < 15.0:
                return                           # the same restart, reported before it
        if level == "fault":
            self._raise(now, code, text, s)
        else:
            self._warn(now, code, text, s)

    def on_beat(self, b: ef.Beat, now: Optional[float] = None) -> None:
        """A heartbeat (BEAT frame, or a text-mode ``#BEAT`` line)."""
        now = self._clock() if now is None else now
        self._last_rx = self._last_beat = now
        self._beats = True
        prev, self._beat = self._beat, b
        if prev is not None and b.uptime_ms + 1000 < prev.uptime_ms:
            # it rebooted; the banner (if we saw it) reports it, else this does
            if (now > self._expect_reset_until and now - self._alive > 30.0
                    and now - self._warn_sent.get("esp32_reboot", -1e9) > 15.0):
                name = ef.RESET_REASONS.get(b.reset_reason, str(b.reset_reason))
                self._warn(now, "esp32_reboot", f"ESP32 rebooted (reset reason {name})",
                           f"uptime {b.uptime_ms} ms")
            self._alive = now
        detail = (f"{_BEAT_DETAIL}{ef.MODE_NAMES.get(b.mode, b.mode)} mode, {b.conv_hz:.0f} DRDY/s,"
                  f" halts {b.halts}, unrecovered {b.unrecovered}, CLOCKrb=0x{b.clock_rb:04X}")
        if b.adc_down:
            self._raise(now, "adc_unrecovered", "ADC not converting (halt not recovered)", detail)
        if b.drdy_edges == 0 and b.window_ms >= 500:
            self._zero_beats += 1
            if self._zero_beats >= DEAD_BEATS:
                self._raise(now, "adc_not_converting",
                            f"ADC not converting: no DRDY for {self._zero_beats} s (ESP32 alive)",
                            detail)
        else:
            self._zero_beats = 0

    def poll(self, now: Optional[float] = None) -> None:
        """Time-based rules; call a few times a second."""
        now = self._clock() if now is None else now
        if self._reboot_pending is not None and now - self._reboot_pending > 3.0:
            self._reboot_pending = None              # older firmware: no reset reason line
            self._warn(now, "esp32_reboot", "ESP32 rebooted", "")
        if self._flag_pending is not None and now - self._flag_pending > 1.0:
            self._flag_pending = None
            if now - self._last_halt_text > 3.0:
                self._warn(now, "adc_halt", "ADC halt recovered (scan flag; no text line)", "")
        silent = now - max(self._last_scan or -1e9, self._up_since, self._alive)
        quiet = now - max(self._last_rx, self._up_since, self._alive)
        if self._up and self._beats and quiet > BEAT_LOST_S:
            self._raise(now, "esp32_hung", f"ESP32 not responding: no heartbeat for {quiet:.0f} s"
                        " (serial link up)", "")
        if self._up:
            if self._halts_since_scan >= HALT_STALL_N and silent > HALT_STALL_S:
                self._raise(now, "adc_not_converting",
                            f"ADC not converting: {self._halts_since_scan} halts and no scan"
                            f" for {silent:.0f} s", "")
            elif silent > NO_SCAN_S and not (self._beats and now - self._last_beat > 2.0):
                # (a heartbeat overdue: wait for the hung / not-hung verdict)
                self._raise(now, "no_scans", f"no whisker scans for {silent:.0f} s"
                            " (ESP32 connected)", "")
        elif now - self._up_since > NO_SCAN_S:
            self._raise(now, "link_down", f"ESP32 serial link down for {now - self._up_since:.0f} s",
                        "")
        if (self._fault is not None and self._up and self._streamed
                and self._clean_since is not None and now - self._last_scan < GAP_S
                and now - self._clean_since >= CLEAR_S):
            f = self._fault
            self._fault = None
            self._event(now, "clear", "ok", f["code"],
                        f"cleared after {CLEAR_S:g} s of clean streaming (was {f['msg']},"
                        f" {now - f['t']:.0f} s)", "", "")
        if now - self._last_flag_check < 1.0:
            return
        self._last_flag_check = now
        if len(self._flags) >= FLAG_MIN_SCANS and self._n_flagged > FLAG_WARN_FRAC * len(self._flags):
            self._warn(now, "scan_flags", f"{100 * self._n_flagged / len(self._flags):.0f} % of"
                       f" scans flagged SYNC/EMPTY over {FLAG_WINDOW_S:g} s", "")

    # ----- outputs

    @property
    def level(self) -> str:
        if self._fault is not None:
            return "fault"
        if self._last_warn is not None and self._clock() - self._last_warn["t"] < WARN_HOLD_S:
            return "warn"
        return "ok"

    def take_events(self) -> list[dict]:
        out, self._events = self._events, []
        return out

    def status(self, now: Optional[float] = None) -> dict:
        """``esp32.fault`` of the status record."""
        now = self._clock() if now is None else now
        st: dict = {"level": "ok", "count": self._n_faults,
                    "warns": dict(self._warn_counts), "history": list(self._history)}
        if self._beats:
            st["beat_age_s"] = round(now - self._last_beat, 1)
        cur = self._fault
        if cur is None and self._last_warn is not None and now - self._last_warn["t"] < WARN_HOLD_S:
            cur = self._last_warn
        if cur is not None:
            st.update(level=cur["level"], code=cur["code"], msg=cur["msg"],
                      action=cur["action"], detail=cur["detail"], since=cur["unix"],
                      age_s=round(now - cur["t"], 1))
        return st

    # ----- internals

    def _raise(self, now: float, code: str, msg: str, detail: str) -> None:
        self._clean_since = None
        f = self._fault
        if f is not None:
            if code == f["code"]:
                f["repeats"] += 1
                # the firmware's own line (e.g. "[halt #N] UNRECOVERED ... CLOCKrb") stays:
                # a later heartbeat summary says less (and reads 0 DRDY/s for as long as it lasts)
                if detail and not (detail.startswith(_BEAT_DETAIL) and f["detail"]
                                   and not f["detail"].startswith(_BEAT_DETAIL)):
                    f["detail"] = detail
                if code in ("no_scans", "link_down", "adc_not_converting", "esp32_hung"):
                    f["msg"] = msg                 # the silence keeps growing
                return
            if _PRIORITY[code] <= _PRIORITY[f["code"]]:
                return                             # no more specific than what we know
        else:
            self._n_faults += 1
        ev = self._event(now, "fault", "fault", code, msg, _ACTIONS.get(code, ACTION_PCB), detail)
        self._fault = dict(ev, repeats=0)
        if f is not None:                          # same fault, better diagnosis: keep the onset
            self._fault.update(t=f["t"], unix=f["unix"])

    def _warn(self, now: float, code: str, msg: str, detail: str) -> None:
        self._warn_counts[code] += 1
        self._last_warn = {"t": now, "unix": self._wall(), "level": "warn", "code": code,
                           "msg": msg, "action": "", "detail": detail}
        if now - self._warn_sent.get(code, -1e9) < WARN_EVERY_S:
            self._warn_unsent[code] += 1
            return
        n = self._warn_unsent.pop(code, 0)
        self._warn_sent[code] = now
        self._event(now, "warn", "warn", code,
                    msg + (f" (+{n} more since the last report)" if n else ""), "", detail)

    def _event(self, now: float, event: str, level: str, code: str, msg: str, action: str,
               detail: str) -> dict:
        ev = {"event": event, "level": level, "code": code, "msg": msg, "action": action,
              "detail": detail, "count": self._n_faults, "unix": self._wall(), "t": now}
        self._events.append(ev)
        self._history.append({k: ev[k] for k in ("event", "code", "msg", "unix")})
        return ev


def mark_text(ev: dict) -> str:
    """One line for a ``mark`` / a log: ``ESP32 FAULT: ... -> action``."""
    if ev["event"] == "fault":
        s = f"ESP32 FAULT {ev['code']}: {ev['msg']}"
        if ev.get("detail"):
            s += f" [{ev['detail']}]"
        return s + (f" -> {ev['action']}" if ev.get("action") else "")
    if ev["event"] == "clear":
        return f"ESP32 fault {ev['code']} {ev['msg']}"
    s = f"ESP32 warning {ev['code']}: {ev['msg']}"
    return s + (f" [{ev['detail']}]" if ev.get("detail") else "")


def mark_event(ev: dict, t: float) -> dict:
    """The wire ``mark`` event for a fault event (``t`` = the stream's clock)."""
    return {"k": "mark", "t": t, "text": mark_text(ev),
            "fault": {k: v for k, v in ev.items() if k != "t"}}
