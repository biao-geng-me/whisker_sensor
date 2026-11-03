"""ROV pose and translational acceleration from the IMU rows (``imu_bus``).

Pure numpy on the rows ``ServerReader`` publishes (columns ``IMU_CHANNELS``:
RAW_IMU accel in mg, gyro, ATTITUDE roll / pitch / yaw held per row) and their
times, so it works on whatever buffer a view holds: the newest rows live, the
replayed ones, or a paused buffer at a picked time.

Frames. ArduSub's body frame is FRD (x forward, y starboard, z down) and its
world frame NED. The 3D views draw in rov-cad's collar frame (+z forward along
the tube axis, +y up, +x port); ``FRD_TO_SCENE`` maps one onto the other, for
the body and (north -> +z, east -> -x, down -> -y) for the world alike.

Translational acceleration. RAW_IMU reads the specific force f = a - g (at
rest, level: z = -1000 mg). The ATTITUDE estimate gives gravity in the body
frame, so a = f + g_body. What is left at rest is the accelerometer's offset
against the EKF's level (about 30 mg on pilot2: the attitude and the raw accel
disagree by ~1.5 deg of pitch), so ``ImuPose`` subtracts a bias taken at rest
(``tare``: the mean over a window at a picked time) before reporting. The rows
are noisy with thruster vibration (~25 mg rms on pilot2), hence the moving
mean over ``smooth_s``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

G_MG = 1000.0                              # 1 g in the RAW_IMU accel unit
ACC, ATT = slice(0, 3), slice(6, 9)        # the IMU_CHANNELS columns used here
# FRD body / NED world -> the scene (collar) frame: forward +z, up +y, port +x
FRD_TO_SCENE = np.array([[0.0, -1.0, 0.0],
                         [0.0, 0.0, -1.0],
                         [1.0, 0.0, 0.0]])


def body_to_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """The rotation body (FRD) -> NED for ArduPilot's Z-Y-X Euler angles (rad)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp]])


def scene_rotation(roll: float, pitch: float, yaw: float = 0.0) -> np.ndarray:
    """``body_to_ned`` expressed in the scene frame: turns a mesh drawn in the
    collar frame (the ROV level, facing +z) into the vehicle's attitude."""
    return FRD_TO_SCENE @ body_to_ned(roll, pitch, yaw) @ FRD_TO_SCENE.T


def gravity_body_mg(roll: np.ndarray, pitch: np.ndarray) -> np.ndarray:
    """Gravity (+g toward NED down) in the body frame, mg, one row per angle pair."""
    roll, pitch = np.asarray(roll, float), np.asarray(pitch, float)
    return G_MG * np.stack([-np.sin(pitch), np.sin(roll) * np.cos(pitch),
                            np.cos(roll) * np.cos(pitch)], axis=-1)


def linear_accel_mg(rows: np.ndarray) -> np.ndarray:
    """Translational acceleration (body FRD, mg) per IMU row: specific force +
    gravity. Includes the accelerometer's offset (see ``ImuPose.tare``)."""
    rows = np.asarray(rows, float)
    return rows[:, ACC] + gravity_body_mg(rows[:, 6], rows[:, 7])


@dataclass
class PoseSample:
    t: float                       # session seconds of the newest row used
    roll: float                    # rad, as ATTITUDE
    pitch: float
    yaw: float
    accel_body: np.ndarray         # (3,) mg, FRD: surge, sway, heave (down +)
    n: int                         # rows averaged

    @property
    def accel_world(self) -> np.ndarray:
        """The acceleration in NED (mg)."""
        return body_to_ned(self.roll, self.pitch, self.yaw) @ self.accel_body


class ImuPose:
    """The pose and the smoothed, de-biased translational acceleration at a time,
    from a buffer of IMU rows. Holds only the settings and the bias."""

    def __init__(self, smooth_s: float = 0.3):
        self.smooth_s = float(smooth_s)
        self.bias = np.zeros(3)                    # mg, body
        self.bias_t: Optional[float] = None        # where it was taken (None: not tared)

    def at(self, t: np.ndarray, rows: np.ndarray,
           t_at: Optional[float] = None) -> Optional[PoseSample]:
        """The state at ``t_at`` (None: the newest row): the attitude of the last
        row at or before it, the acceleration averaged over the ``smooth_s``
        before it. None if the buffer has no row there."""
        if len(t) == 0:
            return None
        i1 = len(t) if t_at is None else int(np.searchsorted(t, t_at, side="right"))
        if i1 == 0 or (t_at is not None and t_at - t[i1 - 1] > max(self.smooth_s, 0.5)):
            return None
        i0 = int(np.searchsorted(t, t[i1 - 1] - self.smooth_s, side="right"))
        i0 = min(i0, i1 - 1)
        a = linear_accel_mg(rows[i0:i1]).mean(axis=0) - self.bias
        r = rows[i1 - 1, ATT].astype(float)
        return PoseSample(float(t[i1 - 1]), float(r[0]), float(r[1]), float(r[2]), a, i1 - i0)

    def tare(self, t: np.ndarray, rows: np.ndarray, t_at: Optional[float] = None,
             window_s: float = 1.0) -> bool:
        """Take the mean acceleration over ``window_s`` centred on ``t_at`` (None:
        the newest ``window_s``) as the zero: the ROV at rest there. False (bias
        kept) if the buffer has no row in that window."""
        if len(t) == 0:
            return False
        if t_at is None:
            t_at = float(t[-1]) - window_s / 2
        i0, i1 = np.searchsorted(t, [t_at - window_s / 2, t_at + window_s / 2])
        if i1 <= i0:
            return False
        self.bias = linear_accel_mg(rows[i0:i1]).mean(axis=0)
        self.bias_t = float(t_at)
        return True

    def clear_tare(self) -> None:
        self.bias = np.zeros(3)
        self.bias_t = None
