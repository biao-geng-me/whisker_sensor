from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np


_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from deploy_v2_path7 import V2Path7HardwareRunner


class V2Path7PolicyAdapter:
    """Object-policy adapter for V2Path7HardwareRunner.

    Compatible with agent_tester object-policy interface:
    - reset()
    - act(observation, reward, done, truncated, info) -> np.ndarray([x_vel, y_vel])
    """

    def __init__(
        self,
        package_dir: str | Path = _THIS_DIR,
        device: str = "cpu",
        signal_shape: tuple[int, int, int] = (3, 3, 2),
        default_action: tuple[float, float] = (0.0, 0.0),
    ):
        self.runner = V2Path7HardwareRunner(package_dir=package_dir, device=device)
        self.signal_shape = tuple(signal_shape)
        self.signal_size = int(np.prod(self.signal_shape))
        self.default_action = np.asarray(default_action, dtype=np.float64).reshape(2)
        self.last_action = self.default_action.copy()

    def reset(self) -> None:
        self.runner.reset_episode()
        self.last_action = self.default_action.copy()

    def _extract_signal_frame(self, row: np.ndarray) -> np.ndarray:
        expected_min_cols = 5 + self.signal_size
        if row.shape[0] < expected_min_cols:
            raise ValueError(
                f"Observation row has {row.shape[0]} columns, expected at least "
                f"{expected_min_cols} (5 kinematics + {self.signal_size} signals)."
            )
        signal_vec = np.asarray(row[5 : 5 + self.signal_size], dtype=np.float32)
        return signal_vec.reshape(self.signal_shape)

    def act(
        self,
        observation: np.ndarray,
        reward: float,
        done: bool,
        truncated: bool,
        info: dict[str, Any],
    ) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32)
        if obs.ndim != 2:
            raise ValueError(f"Expected 2D observation, got shape {obs.shape}.")

        latest_command = None
        for row in obs:
            sensor_frame = self._extract_signal_frame(row)
            self.runner.push_sensor_frame(sensor_frame)
            latest_command = self.runner.compute_command(
                x_mm=float(row[1]),
                y_mm=float(row[2]),
                vx_mm_per_ms=float(row[3]),
                vy_mm_per_ms=float(row[4]),
                time_ms=float(row[0]),
            )

        if latest_command is not None:
            self.last_action = np.array(
                [
                    float(latest_command["command_vx_mm_per_ms"]),
                    float(latest_command["command_vy_mm_per_ms"]),
                ],
                dtype=np.float64,
            )

        return self.last_action.copy()


def build_policy(
    package_dir: str | Path = _THIS_DIR,
    device: str = "cpu",
    signal_shape: tuple[int, int, int] = (3, 3, 2),
    default_action: tuple[float, float] = (0.0, 0.0),
) -> V2Path7PolicyAdapter:
    """Factory for agent_tester --policy object --policy-object ...:build_policy."""
    return V2Path7PolicyAdapter(
        package_dir=package_dir,
        device=device,
        signal_shape=signal_shape,
        default_action=default_action,
    )
