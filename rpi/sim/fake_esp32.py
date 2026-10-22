"""Fake whisker_daq ESP32 on a pseudo-terminal, for testing without hardware.

Creates a pty, prints its path, and behaves like the firmware: boot banner
in text, then binary SCAN frames at ~80 Hz once it receives ``B``. Any of the
text-mode command letters stops the binary stream again.

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
from whisker_link.channels import CHANNEL_NAMES  # noqa: E402

RAIL = (1 << 23) - 1


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rate", type=float, default=80.0, help="scans per second")
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
    period = 1.0 / args.rate
    next_t = time.monotonic()
    offsets = [random.randint(-200_000, 200_000) for _ in range(len(CHANNEL_NAMES))]
    while True:
        try:
            cmds = os.read(master, 256)
        except (BlockingIOError, OSError):
            cmds = b""
        for k in cmds.decode("latin-1"):
            if k == "B":
                binary = True
                out(ef.encode_info(ef.Info(ef.PROTO_VERSION, 32, 0x2240, 0xFF02,
                                           800_000, 3125, 921600, 128, 250, 3)))
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
        if not binary:
            continue
        t_us = micros()
        phase = math.sin(2 * math.pi * args.freq * time.time())
        raw, rail = [], 0
        for i in range(len(CHANNEL_NAMES)):
            if i in live:
                mv = args.amp_mv * phase * (1 + 0.3 * (i % 3)) + random.gauss(0, args.noise_mv)
                raw.append(offsets[i] + int(mv / ef.MV_PER_CODE))
            else:
                raw.append(RAIL)
                rail |= 1 << i
        out(ef.encode_scan(seq, t_us, 12_300, rail, 0, raw))
        seq += 1


if __name__ == "__main__":
    main()
