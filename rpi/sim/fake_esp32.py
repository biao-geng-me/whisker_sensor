"""Fake whisker_daq ESP32 on a pseudo-terminal, for testing without hardware.

Creates a pty, prints its path, and behaves like the firmware: boot banner
in text, then binary SCAN frames at ~80 Hz once it receives ``B``. Any of the
text-mode command letters stops the binary stream again.

Scan modes like the firmware: ``M1;`` quiet (62.5 Hz, OSR 512, the boot
default), ``M0;`` legacy (--rate, default 80 Hz) and ``M2;`` wide (250 Hz),
with INFO version 4 and 1 Hz HEALTH frames in M1/M2 (``--boot-mode 0`` starts
in legacy); ``S<digits>;`` limits the mux selects (skipped outputs read
railed). In M1/M2, t_us advances by exactly 16000 / 4000 us per scan.

Signal: the "live" gauges carry ``amp * sin(2*pi*f*t)`` on a shared wall clock
(``time.time()``) plus noise, so ``fake_autopilot.py`` can put the same motion
on the IMU and the pipeline's time alignment can be checked end to end.
Every other gauge reads full scale (railed = nothing plugged in).

    python rpi/sim/fake_esp32.py                    # prints e.g. /dev/pts/7
    python rpi/daq_server.py --serial /dev/pts/7 --mavlink none
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from whisker_link import esp32_frame as ef  # noqa: E402
from whisker_link.channels import CHANNEL_NAMES, channels_of_selects  # noqa: E402

RAIL = (1 << 23) - 1
# mode -> default (osr, spi_hz, scan period us), as in whisker_daq.ino (M)
MODES = {1: (512, 8_000_000, 16_000), 2: (128, 11_428_571, 4_000)}
OSR_CODE = {128: 0, 256: 1, 512: 2, 1024: 3, 2048: 4, 4096: 5, 8192: 6, 16384: 7}


def lock_params(mode: int, settle: int = 200, osr: int = 0, period: int = 0,
                spi: int = 0) -> tuple | str:
    """(osr, spi, n_conv, scan_us, settle, discard) of a locked mode, or the firmware's error."""
    d_osr, d_spi, d_period = MODES[mode]
    osr, period, spi = osr or d_osr, period or d_period, spi or d_spi
    if osr not in OSR_CODE:
        return "osr must be a power of 2, 128..16384"
    if mode == 2 and period != 4000:
        return "wide mode has a fixed 4000 us period"
    if mode == 1 and (period <= 0 or period > 64000 or period % 8000):
        return "period must be a multiple of 8000 us, up to 64000"
    t = osr // 4                         # conversion period at CLKIN 8 MHz, us
    if period % t:
        return "scan period not a whole number of conversions at this OSR"
    return osr, spi, period // t, period, settle, math.ceil(3 + (settle + 10) / t) - 1


def info_for(mode: int, sel_mask: int, lk: tuple | None = None) -> ef.Info:
    if mode in MODES:
        osr, spi, n, _, settle, discard = lk or lock_params(mode)
        nsel = bin(sel_mask).count("1")
        sels = [s for s in range(8) if sel_mask >> s & 1]
        avg = [0] * 8
        for i, s in enumerate(sels):
            avg[s] = n // nsel + (1 if i < n % nsel else 0) - discard
        return ef.Info(ef.PROTO_VERSION, 32, 0x2405, 0x0F00 | OSR_CODE[osr] << 2 | 2, 8_000_000,
                       8_000_000 // 2 // osr, 921600, osr, settle, discard, sel_mask,
                       mode, spi, n, tuple(avg))
    return ef.Info(ef.PROTO_VERSION, 32, 0x2405, 0xFF02, 800_000, 3125, 921600, 128, 250, 3,
                   sel_mask, 0, 600_000, 0, tuple(1 if sel_mask >> s & 1 else 0 for s in range(8)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rate", type=float, default=80.0, help="legacy mode: scans per second")
    ap.add_argument("--boot-mode", type=int, choices=(0, 1), default=1,
                    help="mode after boot: 1 quiet (as the firmware), 0 legacy")
    ap.add_argument("--live", default="W3_B,W4_B,W8_B", help="connected gauges")
    ap.add_argument("--freq", type=float, default=0.5, help="motion frequency, Hz")
    ap.add_argument("--amp-mv", type=float, default=5.0)
    ap.add_argument("--noise-mv", type=float, default=0.2)
    ap.add_argument("--t0-us", type=int, default=None,
                    help="initial micros() value; e.g. 4294000000 to test the wrap")
    ap.add_argument("--link", default=None, help="also create this symlink to the pty")
    args = ap.parse_args()

    live = {CHANNEL_NAMES.index(n.strip()) for n in args.live.split(",") if n.strip()}
    master, slave = os.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    if args.link:
        try:
            os.unlink(args.link)
        except FileNotFoundError:
            pass
        os.symlink(path, args.link)
    print(path, flush=True)
    os.set_blocking(master, False)

    def out(b: bytes) -> None:
        try:
            os.write(master, b)
        except BlockingIOError:
            pass        # nobody reading; drop like a UART would
        except OSError:
            pass

    boot_mono = time.monotonic()
    t_off = args.t0_us if args.t0_us is not None else 0

    def micros() -> int:
        return (int((time.monotonic() - boot_mono) * 1e6) + t_off) & 0xFFFFFFFF

    out(b"\r\n=================================================\r\n"
        b"  whisker_daq (SIMULATED)\r\n"
        b"Baseline captured from 16 scans.\r\n")
    binary = False
    seq = 0
    mode, sel_mask = args.boot_mode, 0xFF
    lk = lock_params(1) if mode else None   # lock_params() of the current locked mode
    period = lk[3] / 1e6 if lk else 1.0 / args.rate
    next_t = time.monotonic()
    grid_us = None                       # locked modes: t_us on an exact grid
    last_health = time.monotonic()
    n_health = 0
    offsets = [random.randint(-200_000, 200_000) for _ in range(len(CHANNEL_NAMES))]
    pending = ""                         # an M / S command waiting for its ';'

    def say(msg: str) -> None:
        out(ef.encode_text(msg) if binary else (msg + "\r\n").encode())

    def run_arg_cmd(c: str) -> None:
        nonlocal mode, sel_mask, period, grid_us, n_health, lk
        if c[0] == "S":
            digits = {int(ch) for ch in c[1:] if ch in "01234567"}
            sel_mask = sum(1 << d for d in digits) or 0xFF
            say(f"selects: {''.join(str(d) for d in range(8) if sel_mask >> d & 1)}")
        else:
            usage = "M: usage M<0|1|2> [settle_us] [osr] [period_us] [spi_hz];"
            try:
                v = [int(x) for x in c[1:].split()[:5]]
                m = v[0]
            except (ValueError, IndexError):
                say(usage)
                return
            if m not in (0, 1, 2):
                say(usage)
                return
            if m in MODES:
                v += [0] * (5 - len(v))
                p = lock_params(m, v[1] if len(v) > 1 and v[1] else 200, v[2], v[3], v[4])
                if isinstance(p, str):
                    say(f"M: {p}")
                    return
                lk = p
            mode = m
            period = lk[3] / 1e6 if m in MODES else 1.0 / args.rate
            grid_us = None
            n_health = 0
            say(f"mode {m} (simulated)")
        if binary:
            out(ef.encode_info(info_for(mode, sel_mask, lk)))

    while True:
        try:
            cmds = os.read(master, 256)
        except (BlockingIOError, OSError):
            cmds = b""
        for k in cmds.decode("latin-1"):
            if pending:
                if k in ";\n":
                    run_arg_cmd(pending)
                    pending = ""
                elif k != "\r":
                    pending += k
                continue
            if k in "MS":
                pending = k
            elif k == "B":
                binary = True
                out(ef.encode_info(info_for(mode, sel_mask, lk)))
            elif k == "b":
                out(ef.encode_text("Re-capturing baseline...") if binary
                    else b"Re-capturing baseline...\r\n")
            elif k in "gxscnm":
                binary = False
                out(b"\r\n-- TEXT MODE (simulated) --\r\n")
        now = time.monotonic()
        if now < next_t:
            time.sleep(min(next_t - now, 0.005))
            continue
        next_t += period
        if next_t < now - 0.5:           # fell far behind (suspended): don't burst
            next_t = now + period
        if not binary:
            continue
        if mode in MODES:
            grid_us = micros() if grid_us is None else grid_us + lk[3]
            t_us, dur = grid_us & 0xFFFFFFFF, lk[3]
        else:
            t_us, dur = micros(), 12_300
        phase = math.sin(2 * math.pi * args.freq * time.time())
        scanned = set(channels_of_selects(sel_mask))
        raw, rail = [], 0
        for i in range(len(CHANNEL_NAMES)):
            if i in live and i in scanned:
                mv = args.amp_mv * phase * (1 + 0.3 * (i % 3)) + random.gauss(0, args.noise_mv)
                raw.append(offsets[i] + int(mv / ef.MV_PER_CODE))
            else:
                raw.append(RAIL)
                rail |= 1 << i
        out(ef.encode_scan(seq, t_us, dur, rail, 0, raw))
        seq += 1
        if mode in MODES and now - last_health >= 1.0:
            last_health = now
            osr, _, n, scan_us = lk[:4]
            conv = int(1e6 / scan_us * n)
            out(ef.encode_health(ef.Health(1, mode, 380 if mode == 2 else 250, 30, 180, 160, 17,
                                           4, conv, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000)))
            n_health += 1


if __name__ == "__main__":
    main()
