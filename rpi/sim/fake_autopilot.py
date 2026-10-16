"""Fake ArduSub autopilot over MAVLink UDP, for testing ``daq_server.py``.

Plays the part of BlueOS's mavlink-router endpoint ("UDP Client" to
127.0.0.1:14670): sends HEARTBEAT at 1 Hz and honours
MAV_CMD_SET_MESSAGE_INTERVAL for the IMU-type messages the server asks for.

Motion: pitch and x-accel follow ``sin(2*pi*f*t)`` on the shared wall clock,
matching ``fake_esp32.py``, delayed by ``--lag-ms`` to mimic a physical lag.

    python rpi/sim/fake_autopilot.py --target 127.0.0.1:14670
"""

from __future__ import annotations

import argparse
import math
import os
import random
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

mavlink = mavutil.mavlink


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--target", default="127.0.0.1:14670")
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--lag-ms", type=float, default=0.0)
    ap.add_argument("--max-hz", type=float, default=400.0, help="autopilot loop rate cap")
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(f"udpout:{args.target}", source_system=1,
                                      source_component=1, dialect="ardupilotmega")
    boot = time.monotonic()
    intervals: dict[int, float] = {}          # msg id -> seconds
    next_due: dict[int, float] = {}
    last_hb = 0.0
    loop_dt = 1.0 / args.max_hz

    def tboot_ms() -> int:
        return int((time.monotonic() - boot) * 1000)

    def motion() -> float:
        return math.sin(2 * math.pi * args.freq * (time.time() - args.lag_ms / 1000))

    senders = {
        mavlink.MAVLINK_MSG_ID_RAW_IMU: lambda: conn.mav.raw_imu_send(
            int((time.monotonic() - boot) * 1e6),
            int(200 * motion() + random.gauss(0, 5)), int(random.gauss(0, 5)),
            int(-1000 + random.gauss(0, 5)), int(30 * motion()), 0, 0, 200, 0, -400),
        mavlink.MAVLINK_MSG_ID_SCALED_IMU2: lambda: conn.mav.scaled_imu2_send(
            tboot_ms(), int(200 * motion()), 0, -1000, int(30 * motion()), 0, 0, 200, 0, -400),
        mavlink.MAVLINK_MSG_ID_ATTITUDE: lambda: conn.mav.attitude_send(
            tboot_ms(), 0.0, 0.1 * motion(), 1.0, 0.0, 0.03 * motion(), 0.0),
        mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2: lambda: conn.mav.scaled_pressure2_send(
            tboot_ms(), 1013.0 + 10 * motion(), 0.0, 1500),
        mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: lambda: conn.mav.servo_output_raw_send(
            int((time.monotonic() - boot) * 1e6), 0, *([1500] * 8)),
        mavlink.MAVLINK_MSG_ID_SYSTEM_TIME: lambda: conn.mav.system_time_send(
            int(time.time() * 1e6), tboot_ms()),
    }
    print(f"fake autopilot -> {args.target}", flush=True)
    while True:
        now = time.monotonic()
        if now - last_hb >= 1.0:
            conn.mav.heartbeat_send(mavlink.MAV_TYPE_SUBMARINE,
                                    mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
            last_hb = now
        while True:
            m = conn.recv_match(blocking=False)
            if m is None:
                break
            if (m.get_type() == "COMMAND_LONG"
                    and m.command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL):
                mid, us = int(m.param1), m.param2
                if us > 0 and mid in senders:
                    intervals[mid] = max(us / 1e6, loop_dt)
                    next_due.setdefault(mid, now)
                elif us < 0:
                    intervals.pop(mid, None)
                conn.mav.command_ack_send(m.command, mavlink.MAV_RESULT_ACCEPTED)
        for mid, iv in intervals.items():
            if now >= next_due[mid]:
                senders[mid]()
                next_due[mid] = max(next_due[mid] + iv, now - iv)
        time.sleep(loop_dt / 2)


if __name__ == "__main__":
    main()
