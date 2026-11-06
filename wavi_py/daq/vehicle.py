"""The pilot's demand and the thrusters: RC sticks and SERVO outputs in axis terms.

- demand ``u = (raw - 1500) / 400`` per axis from RC_CHANNELS: ch5 surge, ch6
  sway, ch3 heave, ch4 yaw (the RC values already include the pilot gain);
- thrust per DOF: SERVO1..8 (1500 neutral, +-400 full) projected through ArduSub's
  AP_Motors6DOF ``SUB_FRAME_VECTORED_6DOF`` mix (the BlueROV2 Heavy) with this
  vehicle's motor directions (``MOTOR_DIRECTION``: MOT_n_DIRECTION is not in the
  recordings; it is the sign each servo takes against the mixed demand, the same
  on every 2026-11-05 run, and ``replay/manoeuvres.mix_check`` checks it per run).

The Analysis window reads the demand and the thrust per axis as ROV variables
(``replay/rov.py``: ``rov:fwd`` ..., ``rov:th_surge`` ...); the manoeuvres
sidecar (``replay/manoeuvres.py``) takes the same pilot and thruster tracks.
"""

from __future__ import annotations

import numpy as np

AXES = ("surge", "sway", "heave", "yaw")
RC_CHANNEL = {"surge": 5, "sway": 6, "heave": 3, "yaw": 4}
NEUTRAL_US = 1500
FULL_US = 400                   # full stick (after the pilot gain): u = +-1
# per motor 1..8: roll, pitch, yaw, throttle (heave), forward (surge), lateral (sway)
DOF = ("roll", "pitch", "yaw", "heave", "surge", "sway")
MIX = np.array([[0, 0, 1, 0, -1, 1], [0, 0, -1, 0, -1, -1], [0, 0, -1, 0, 1, 1],
                [0, 0, 1, 0, 1, -1], [1, -1, 0, -1, 0, 0], [-1, -1, 0, -1, 0, 0],
                [1, 1, 0, -1, 0, 0], [-1, 1, 0, -1, 0, 0]], dtype=np.float64)
MOTOR_DIRECTION = (-1, -1, 1, 1, -1, 1, 1, -1)


def demand(rc_raw: np.ndarray) -> np.ndarray:
    """(n, 4) demand from (n, 4) raw RC values in ``AXES`` order."""
    return (np.asarray(rc_raw, np.float64) - NEUTRAL_US) / FULL_US


def thrust(servo_us: np.ndarray) -> np.ndarray:
    """(n, 6) thrust per ``DOF`` from (n, 8) SERVO1..8 microseconds."""
    th = (np.asarray(servo_us, np.float64) - NEUTRAL_US) / FULL_US
    a = MIX * np.asarray(MOTOR_DIRECTION, np.float64)[:, None]
    return th @ np.linalg.pinv(a).T

