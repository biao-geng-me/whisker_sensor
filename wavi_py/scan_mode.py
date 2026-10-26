"""ESP32 scan modes (``whisker_daq.ino`` command ``M``) for the GUI's Scan mode
dropdown, ``--mode`` and the Sim transport.

The firmware (since 9e57703) boots in quiet 62.5 Hz, OSR 512, core C1, with
all 8 selects, and forgets any M / S setting on reboot. A ``ScanRequest`` is what the user picked: the M arguments plus the mux selects,
sent as ``S<selects>;M<args>;B`` (the ``esp32`` command of the server and ESP32
transports). ``ModeKeeper`` checks the INFO frames that come back:

- after a request, the mode counts as set only when an INFO reports it (an
  INFO that still shows the old mode is ignored for ~2 s, since the firmware
  can send one before it reads the command); no matching INFO within
  ``CONFIRM_S`` is reported;
- later, an INFO with another mode or selects means the ESP32 rebooted (port
  open with DTR auto-reset, a watchdog) and came back in its boot default: the
  request is re-sent once, and the note says what it came back in. If that
  doesn't stick either, it is reported and not sent again until the user picks
  a mode. A request equal to the boot default (quiet 62.5 Hz, OSR 512, all
  selects) survives a reboot unchanged.

Qt-free, so it can be tested headless.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from whisker_link import esp32_frame as ef

MODE_IDS = {"legacy": 0, "quiet": 1, "wide": 2}
# mode -> firmware defaults (osr, spi_hz, scan period us), as in whisker_daq.ino (M)
LOCK_DEFAULTS = {1: (512, 8_000_000, 16_000), 2: (128, 11_428_571, 4_000)}
SETTLE_DEFAULT_US = 200
DEFAULT_SELECTS = "0123457"          # select 6 is dead on the current board
BOOT_MODE = 1                        # the firmware's boot default: quiet 62.5 Hz, OSR 512
LEGACY_MS_PER_SELECT = 1.6           # legacy scan time per select (r7: 11.2 ms for 7 selects)
LOCKED_CLKIN_HZ = 8_000_000
_OSR_CODE = {128: 0, 256: 1, 512: 2, 1024: 3, 2048: 4, 4096: 5, 8192: 6, 16384: 7}

# (dropdown label, M arguments)
PRESETS = (
    ("legacy", (0,)),
    ("quiet 62.5 Hz, OSR 512", (1,)),
    ("quiet 62.5 Hz, OSR 256", (1, 200, 256)),
    ("quiet 125 Hz, OSR 256", (1, 200, 256, 8000)),
    ("wide 250 Hz, OSR 128", (2,)),
)


def parse_mode(s: str) -> tuple[int, ...]:
    """``wide`` / ``2`` / ``quiet,200,256,8000`` -> the M arguments, e.g. (1, 200, 256, 8000).

    Same spelling as ``rpi/daq_server.py --mode``: the mode, then settle_us, osr,
    period_us, spi_hz; 0 = the mode's default.
    """
    parts = [p.strip() for p in s.replace(":", ",").split(",") if p.strip()]
    if not parts:
        raise ValueError("mode: legacy|quiet|wide or 0|1|2")
    m = MODE_IDS.get(parts[0].lower(), parts[0])
    try:
        nums = (int(m),) + tuple(int(p) for p in parts[1:5])
    except ValueError:
        raise ValueError(f"mode: bad value {s!r}") from None
    if nums[0] not in (0, 1, 2):
        raise ValueError("mode: legacy|quiet|wide or 0|1|2")
    return nums


def parse_selects(s: str) -> str:
    """``0123457`` -> sorted digits; ``""`` = all 8."""
    digits = sorted(set(s.replace(",", "").replace(" ", "")))
    if any(d not in "01234567" for d in digits):
        raise ValueError("selects: digits 0-7, e.g. 0123457 (empty = all)")
    return "".join(digits)


def legacy_nominal_hz(sel_mask: int) -> float:
    """Rough legacy scan rate for these selects (the measured rate replaces it):
    the scan time grows with the number of selects (r7: 11.2 ms for 7, ~89 Hz)."""
    return 1000.0 / (LEGACY_MS_PER_SELECT * (bin(sel_mask & 0xFF).count("1") or 8))


def lock_params(mode: int, settle: int = 0, osr: int = 0, period: int = 0,
                spi: int = 0) -> tuple | str:
    """(osr, spi, n_conv, period_us, settle, discard) of a locked mode, or the
    firmware's error message (mirror of ``lkConfigure``)."""
    d_osr, d_spi, d_period = LOCK_DEFAULTS[mode]
    osr, period, spi = osr or d_osr, period or d_period, spi or d_spi
    settle = settle or SETTLE_DEFAULT_US
    if osr not in _OSR_CODE:
        return "osr must be a power of 2, 128..16384"
    if mode == 2 and period != 4000:
        return "wide mode has a fixed 4000 us period"
    if mode == 1 and (period <= 0 or period > 64000 or period % 8000):
        return "period must be a multiple of 8000 us, up to 64000"
    t = osr // 4                          # conversion period at CLKIN 8 MHz, us
    if period % t:
        return "scan period not a whole number of conversions at this OSR"
    return osr, spi, period // t, period, settle, math.ceil(3 + (settle + 10) / t) - 1


def info_for(mode: int, sel_mask: int, lk: tuple | None = None) -> ef.Info:
    """The INFO the firmware sends for this mode (for simulators)."""
    if mode in LOCK_DEFAULTS:
        osr, spi, n, _, settle, discard = lk or lock_params(mode)
        sels = [s for s in range(8) if sel_mask >> s & 1]
        avg = [0] * 8
        for i, s in enumerate(sels):
            avg[s] = n // len(sels) + (1 if i < n % len(sels) else 0) - discard
        return ef.Info(ef.PROTO_VERSION, 32, 0x2405, 0x0F00 | _OSR_CODE[osr] << 2 | 2,
                       LOCKED_CLKIN_HZ, LOCKED_CLKIN_HZ // 2 // osr, 921600, osr, settle,
                       discard, sel_mask, mode, spi, n, tuple(avg))
    return ef.Info(ef.PROTO_VERSION, 32, 0x2405, 0xFF02, 800_000, 3125, 921600, 128, 250, 3,
                   sel_mask, 0, 600_000, 0, tuple(1 if sel_mask >> s & 1 else 0 for s in range(8)))


def describe_cfg(cfg: dict) -> str:
    """``quiet 125 Hz OSR 256`` from an ``info_cfg`` dict."""
    name = cfg.get("mode_name")
    if name is None:
        return "no scan modes (firmware before INFO v4)"
    if not cfg.get("scan_hz"):
        return name
    return f"{name} {cfg['scan_hz']:g} Hz OSR {cfg.get('osr', '?')}"


@dataclass(frozen=True)
class ScanRequest:
    args: tuple[int, ...]             # M arguments: mode [settle_us] [osr] [period_us] [spi_hz]
    selects: str = DEFAULT_SELECTS    # "" = all 8

    @property
    def mode(self) -> int:
        return self.args[0]

    @property
    def sel_mask(self) -> int:
        return sum(1 << int(d) for d in self.selects) or 0xFF

    def command(self) -> str:
        return f"S{self.selects};M{' '.join(str(a) for a in self.args)};B"

    @property
    def label(self) -> str:
        for name, args in PRESETS:
            if args == self.args:
                return name.split(" (")[0]
        return f"M{' '.join(str(a) for a in self.args)}"

    def expected(self) -> dict:
        """What the INFO should report: mode, sel_mask, and for a locked mode osr,
        settle_us and scan_hz."""
        exp = {"mode": self.mode, "sel_mask": self.sel_mask}
        if self.mode in LOCK_DEFAULTS:
            a = list(self.args[1:]) + [0] * 4
            d_osr, _, d_period = LOCK_DEFAULTS[self.mode]
            exp.update(settle_us=a[0] or SETTLE_DEFAULT_US, osr=a[1] or d_osr,
                       scan_hz=1e6 / (a[2] or d_period))
        return exp

    def mismatch(self, cfg: dict) -> list[str]:
        """How an ``info_cfg`` dict differs from this request (empty: it matches)."""
        out = []
        for k, v in self.expected().items():
            got = cfg.get(k)
            if k == "scan_hz":
                same = got is not None and abs(got - v) <= 1e-3 * v
            else:
                same = got == v
            if not same:
                if k == "mode":
                    got = cfg.get("mode_name", got)
                    v = {0: "legacy", 1: "quiet", 2: "wide"}[v]
                elif k == "sel_mask":
                    got = cfg.get("selects", got)
                    v = self.selects or "01234567"
                out.append(f"{k} {got} (wanted {v:g})" if isinstance(v, float)
                           else f"{k} {got} (wanted {v})")
        return out


class ModeKeeper:
    """Keeps the ESP32 in the requested scan mode; see the module docstring.

    Feed it ``connected``, ``on_info`` (each INFO, as an ``info_cfg`` dict),
    ``on_text`` (firmware text lines) and ``tick``; each returns the command
    string to send to the ESP32, or None. ``note`` is the status-line text.
    """

    CONFIRM_S = 2.0          # an INFO must confirm a request within this
    FIRST_INFO_S = 1.5       # after connecting, wait this long for the current mode

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self.want: Optional[ScanRequest] = None
        self.state = "idle"  # idle | first (waiting for the first INFO) | pending | ok | failed
        self.note = ""
        self._t = 0.0
        self._reapplied = False      # a re-send since the last confirmation
        self._reapply_note = ""
        self._last_cfg: Optional[dict] = None
        self._infos_since_send = 0
        self._fw_msg = ""            # firmware text after the send (e.g. "M: period must ...")

    def set_wanted(self, req: Optional[ScanRequest], connected: bool) -> Optional[str]:
        """The user picked a mode (None: leave the ESP32 as it is)."""
        self.want = req
        self._reapplied = False
        self._reapply_note = ""
        if req is None:
            self.state, self.note = "idle", ""
            return None
        if not connected:
            self.state, self.note = "idle", f"{req.label} (sent on connect)"
            return None
        return self._send("setting")

    def connected(self) -> None:
        """A new connection: compare the first INFO before sending anything."""
        self._last_cfg = None
        self._reapplied = False
        self._reapply_note = ""
        if self.want is not None:
            self.state, self._t = "first", self._clock()
            self.note = f"{self.want.label}: waiting for the ESP32's INFO"

    def on_info(self, cfg: dict) -> Optional[str]:
        self._last_cfg = cfg
        self._infos_since_send += 1
        if self.want is None:
            return None
        if not self.want.mismatch(cfg):
            self.state = "ok"
            self._reapplied = False
            self.note = f"{describe_cfg(cfg)} confirmed" + self._reapply_note
            return None
        if self.state == "pending":
            return None              # maybe sent before the command was read; see tick()
        if self.state in ("first", "idle"):
            return self._send("setting")
        if not self._reapplied:
            self._reapplied = True
            back = describe_cfg(cfg)
            if cfg.get("sel_mask") != self.want.sel_mask:   # e.g. the boot default's all 8
                back += f" selects {cfg.get('selects', '?')}"
            self._reapply_note = (f"; re-applied {time.strftime('%H:%M:%S')} after the ESP32 "
                                  f"came back in {back}")
            return self._send(f"ESP32 came back in {back} (reboot?), re-sending")
        self.state = "failed"
        self.note = (f"ESP32 reports {describe_cfg(cfg)}, wanted {self.want.label}: "
                     f"{', '.join(self.want.mismatch(cfg))}; not re-sent (pick the mode to retry)")
        return None

    def on_text(self, msg: str) -> None:
        if self.state == "pending" and msg.lstrip().startswith(("M:", "S:")):
            self._fw_msg = msg.strip()

    def tick(self) -> Optional[str]:
        now = self._clock()
        if self.state == "first" and now - self._t > self.FIRST_INFO_S:
            return self._send("setting")         # no INFO yet (old server, quiet port)
        if self.state == "pending" and now - self._t > self.CONFIRM_S:
            self.state = "failed"
            assert self.want is not None
            if not self._infos_since_send:
                why = f"no INFO reply within {self.CONFIRM_S:g} s"
            else:
                cfg = self._last_cfg or {}
                why = (f"ESP32 reports {describe_cfg(cfg)}: "
                       + ", ".join(self.want.mismatch(cfg)))
            if self._fw_msg:
                why += f" [{self._fw_msg}]"
            self.note = f"NOT SET {self.want.label}: {why}"
        return None

    def _send(self, what: str) -> str:
        assert self.want is not None
        self.state, self._t = "pending", self._clock()
        self._infos_since_send = 0
        self._fw_msg = ""
        self.note = f"{what} {self.want.label}..." if what == "setting" else what
        return self.want.command()
