"""A synthetic daq_server recording for the replay tests (about 10 s).

- the hello in ``json.dumps`` spacing (as the GUI's tee writes it; ``compact``
  writes it as ``wire.dumps`` does), with the ESP32 cfg, and ``t != mono``;
- 250 Hz scans with seq / tus on a crystal 80 ppm slow against the Pi, 1-3 ms
  receive latency in 8 ms bursts, one seq gap (``SEQ_GAP_AT``), railed channels;
- RAW_IMU 100 Hz (``time_usec``), ATTITUDE 50 Hz, SCALED_PRESSURE2 20 Hz with a
  dive to ``MAX_DEPTH_CM`` and back;
- autopilot HEARTBEATs 3 Hz: armed ``ARM_AT``..``DISARM_AT``, MANUAL -> ALT_HOLD at
  ``MODE_AT``; a ``[1,194]`` heartbeat with base_mode 128 throughout;
- STATUSTEXT "Lost manual control" (critical) three times 3 s apart and one
  "#Gain is 50%" (info); one info line; one fault mark; a 1 Hz status whose
  adc_crc counter goes up at ``CRC_AT``;
- with ``rov``: the vehicle telemetry at 20 Hz (``rov_truth``): RC_CHANNELS
  (ch5 a 0.5 Hz sine of +-200 us), SERVO_OUTPUT_RAW (servo1 a ramp, servo8
  unused = 0), VFR_HUD (heading through north, climb), BATTERY_STATUS (1.5 A),
  VIBRATION, NAMED_VALUE_FLOAT PilotGain (0.25, then 0.5 from ``MODE_AT``), and
  an RC_CHANNELS of another component (``[1,2]``) that must be ignored;
- a block of MAVLink lines written 0.2 s late (``t`` steps back in file order);
- a truncated last line.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from whisker_link import wire
from whisker_link.channels import CHANNEL_NAMES

MV = 0.0001430511474609375
FS = 250.0
DUR_S = 10.0
T_HELLO = 1000.0                 # receive clock of the hello (the session zero is just after)
MONO = 900.0                     # hello.mono: the connection, earlier
UNIX = 1.79e9
PPM = -80.0                      # ESP32 crystal vs the Pi
SEQ_GAP_AT = 6.0                 # 5 scans lost here
ARM_AT, DISARM_AT, MODE_AT = 2.0, 8.0, 5.0
CRC_AT = 4.0
MAX_DEPTH_CM = 40.0
RAILED = (4, 5, 14, 15)          # channel indices that read railed
BACKSTEP_AT = 3.0                # the late MAVLink block
HPA_SURF = 1001.7
# spikes=True: (scan index, channel, codes added); a 3-scan run on channel 9
SPIKES = ((300, 0, 300_000), (301, 7, -250_000), (1200, 20, 400_000),
          (2000, 9, 300_000), (2001, 9, 310_000), (2002, 9, 290_000))


def depth_profile(t: float) -> float:
    """cm: 0 until 2.5 s, down to MAX_DEPTH_CM at 5 s, back to 0 at 7.5 s."""
    if t < 2.5 or t > 7.5:
        return 0.0
    return MAX_DEPTH_CM * math.sin(math.pi * (t - 2.5) / 5.0)


def rov_truth(s: float) -> tuple[float, float, float]:
    """(pilot forward -1..1, thruster 1 -1..1, heading deg unwrapped) at ``s``."""
    return 0.5 * math.sin(2 * math.pi * 0.5 * s), -0.5 + s / DUR_S, 350.0 + 20.0 * s


def raw_of(seq: int) -> list[int]:
    r = [1_800_000 + 3000 * c + int(20000 * math.sin(2 * math.pi * 3.0 * seq / FS + c))
         for c in range(32)]
    for c in RAILED:
        r[c] = 8388607
    return r


def write_recording(path, compact_hello: bool = False, truncated: bool = True,
                    vehicle: bool = True, spikes: bool = False, rov: bool = False) -> dict:
    """Write the recording; returns the ground truth for the tests."""
    rng = np.random.default_rng(1)
    lines: list[tuple[float, bytes]] = []          # (sort key, line)
    t_first = T_HELLO + 0.002

    late0 = t_first + BACKSTEP_AT

    def add(t, ev):
        ev = {"k": ev.pop("k"), "t": t, **ev}
        key = t
        if ev["k"] == "mav" and late0 <= t < late0 + 0.2:
            # the MAVLink thread stalled: these are written after the scans up to
            # late0 + 0.2, in their own order, so t steps back in the file
            key = late0 + 0.2 + (t - late0) * 1e-3
        lines.append((key, wire.dumps(ev)))

    # whisker scans: device time s, receive = s*(1+ppm) + latency, in 8 ms bursts
    seq0, tus0 = 5000, 120_000_000
    true_t, seqs = [], []
    i_gap = int(SEQ_GAP_AT * FS)
    prev_rx = 0.0
    for i in range(int(DUR_S * FS)):
        if i_gap <= i < i_gap + 5:
            continue                               # 5 scans lost
        s_dev = i / FS
        seq = seq0 + i
        t_dev = t_first + s_dev * (1 + PPM * 1e-6)
        # sent in 8 ms bursts (on the device's clock), then 1-3 ms of latency
        burst = t_first + math.ceil(s_dev / 0.008 - 1e-9) * 0.008 * (1 + PPM * 1e-6)
        t_rx = burst + 0.001 + 0.002 * rng.random() ** 6    # mostly ~1 ms, up to 3
        t_rx = prev_rx = max(t_rx, prev_rx + 1e-6)     # one reader thread: in order
        flg = 2 if abs(s_dev - CRC_AT) < 0.01 else 0
        raw = raw_of(seq)
        for at, c, d in (SPIKES if spikes else ()):
            if at == i:
                raw[c] += d
        add(t_rx, {"k": "wsk", "seq": seq, "tus": tus0 + int(round(s_dev * 1e6)), "dur": 4000,
                   "rail": 0, "flg": flg, "raw": raw})
        true_t.append(t_dev)
        seqs.append(seq)
    truth = {"wsk_t_dev": np.asarray(true_t), "wsk_seq": np.asarray(seqs), "n_wsk": len(seqs),
             "t_first": t_first}
    if vehicle:
        boot_us = 995_000_000
        n_imu = 0
        for i in range(int(DUR_S * 100)):
            s = i / 100
            t_dev = t_first + s * (1 + 20e-6)
            t_rx = t_dev + 0.003 + 0.004 * rng.random()
            add(t_rx, {"k": "mav", "type": "RAW_IMU", "src": [1, 1],
                       "d": {"time_usec": boot_us + int(s * 1e6), "xacc": int(100 * math.sin(s)),
                             "yacc": 0, "zacc": -1000, "xgyro": 1, "ygyro": 2, "zgyro": 3,
                             "xmag": 0, "ymag": 0, "zmag": 0, "id": 0, "temperature": 5000}})
            n_imu += 1
            if i % 2 == 0:
                add(t_rx + 0.0005, {"k": "mav", "type": "ATTITUDE", "src": [1, 1],
                                    "d": {"time_boot_ms": boot_us // 1000 + int(s * 1e3),
                                          "roll": 0.1, "pitch": 0.2 * math.sin(s), "yaw": 1.0,
                                          "rollspeed": 0, "pitchspeed": 0, "yawspeed": 0}})
            if i % 5 == 0:
                add(t_rx + 0.0007, {"k": "mav", "type": "SCALED_PRESSURE2", "src": [1, 1],
                                    "d": {"time_boot_ms": boot_us // 1000 + int(s * 1e3),
                                          "press_abs": HPA_SURF + depth_profile(s) * 0.981,
                                          "press_diff": 0.0, "temperature": 1964,
                                          "temperature_press_diff": 0}})
        truth["n_raw_imu"] = n_imu
        for i in range(int(DUR_S * 3)):
            s = i / 3
            armed = ARM_AT <= s < DISARM_AT
            add(t_first + s + 0.01, {"k": "mav", "type": "HEARTBEAT", "src": [1, 1],
                                     "d": {"type": 12, "autopilot": 3,
                                           "base_mode": 81 + 128 * armed,
                                           "custom_mode": 2 if s >= MODE_AT else 19,
                                           "system_status": 4, "mavlink_version": 3}})
            add(t_first + s + 0.02, {"k": "mav", "type": "HEARTBEAT", "src": [1, 194],
                                     "d": {"type": 18, "autopilot": 8, "base_mode": 128,
                                           "custom_mode": 0, "system_status": 4,
                                           "mavlink_version": 3}})
        for s in (1.0, 4.0, 7.0):
            add(t_first + s, {"k": "mav", "type": "STATUSTEXT", "src": [1, 1],
                              "d": {"severity": 2, "text": "Lost manual control", "id": 0,
                                    "chunk_seq": 0}})
        add(t_first + 5.5, {"k": "mav", "type": "STATUSTEXT", "src": [1, 1],
                            "d": {"severity": 6, "text": "#Gain is 50%", "id": 0, "chunk_seq": 0}})
    if vehicle and rov:
        for i in range(int(DUR_S * 20)):
            s = i / 20
            t = t_first + s + 0.004
            fwd, ramp, hdg = rov_truth(s)
            add(t, {"k": "mav", "type": "RC_CHANNELS", "src": [1, 1],
                    "d": {"time_boot_ms": i, "chancount": 0,
                          **{f"chan{c}_raw": 1500 for c in range(1, 12)},
                          "chan5_raw": int(round(1500 + 400 * fwd)), "rssi": 255}})
            add(t + 0.001, {"k": "mav", "type": "SERVO_OUTPUT_RAW", "src": [1, 1],
                            "d": {"time_usec": i, "port": 0,
                                  **{f"servo{c}_raw": 1500 for c in range(1, 9)},
                                  "servo1_raw": int(round(1500 + 400 * ramp)), "servo8_raw": 0}})
            add(t + 0.002, {"k": "mav", "type": "VFR_HUD", "src": [1, 1],
                            "d": {"airspeed": 0.0, "groundspeed": 0.0, "heading": int(hdg) % 360,
                                  "throttle": 0, "alt": 0.0, "climb": 0.1}})
            add(t + 0.003, {"k": "mav", "type": "NAMED_VALUE_FLOAT", "src": [1, 1],
                            "d": {"time_boot_ms": i, "name": "PilotGain",
                                  "value": 0.5 if s >= MODE_AT else 0.25}})
            add(t + 0.0035, {"k": "mav", "type": "NAMED_VALUE_FLOAT", "src": [1, 1],
                             "d": {"time_boot_ms": i, "name": "CamTilt", "value": 0.9}})
            if i % 7 == 0:
                add(t + 0.0037, {"k": "mav", "type": "RC_CHANNELS", "src": [1, 2],
                                 "d": {"time_boot_ms": i, "chan5_raw": 1900}})
            if i % 6 == 0:
                add(t + 0.0040, {"k": "mav", "type": "BATTERY_STATUS", "src": [1, 1],
                                 "d": {"id": 0, "current_battery": 150, "voltages": [16000]}})
                add(t + 0.0042, {"k": "mav", "type": "VIBRATION", "src": [1, 1],
                                 "d": {"time_usec": i, "vibration_x": 0.5, "vibration_y": 0.25,
                                       "vibration_z": 1.0}})
    # the pilot2 hello's INFO (wide 250 Hz, OSR 128, selects 0123457)
    cfg = {"proto": 4, "nch": 32, "device_id": 9221, "clock_rb": 3842, "clkin_hz": 8000000,
           "fdata_sps": 31250, "baud": 921600, "osr": 128, "settle_us": 200,
           "discard_samples": 9, "sel_mask": 191, "mode": 2, "spi_hz": 11428571, "n_conv": 125,
           "avg_per_sel": [9, 9, 9, 9, 9, 9, 0, 8]}
    add(t_first + 3.5, {"k": "info", **cfg})
    add(t_first + 6.5, {"k": "mark", "text": "ADC fault: test",
                        "fault": {"event": "fault", "code": "adc_down", "msg": "test",
                                  "unix": UNIX + 106.5}})
    for i in range(int(DUR_S)):
        s = i + 0.5
        add(t_first + s, {"k": "status", "clients": 1,
                          "rec": {"on": True, "secs": s, "events": 0},
                          "hz": {"wsk": 250.0, "RAW_IMU": 100.0},
                          "esp32": {"scans": i * 250, "seq_gaps": int(s > SEQ_GAP_AT),
                                    "crc_fail": 0, "adc_crc": 3 * int(s > CRC_AT), "adc_stale": 0,
                                    "adc_recov": 0, "adc_reinit": 0, "streaming": True,
                                    "cfg": {**cfg, "mode_name": "wide"}}})
    lines.sort(key=lambda kl: kl[0])
    hello = {"k": "hello", "t": T_HELLO, "mono": MONO, "unix": UNIX, "wire": 1,
             "server": "0.1" if vehicle else "wavi_py-esp32", "host": "test",
             "channels": CHANNEL_NAMES, "mv_per_code": MV, "esp32": cfg}
    head = wire.dumps(hello) if compact_hello else (json.dumps(hello) + "\n").encode()
    path = Path(path)
    with open(path, "wb") as f:
        f.write(head)
        for _, line in lines:
            f.write(line)
        if truncated:
            f.write(b'{"k":"wsk","t":1011.0,"seq":99999,"tus":1,"raw":[1,2')
    # session zero: the first line after the hello, in file order
    truth.update(n_lines=len(lines) + 1, hello=hello, t0=json.loads(lines[0][1])["t"])
    return truth
