"""Stub runtime for hardware-only training.

This file is meant to be copied or used as a reference when wiring the real
tank controller into the trainer. A first-time implementer should start here,
replace the placeholder methods with hardware I/O, and then point
`SACV2PathblindConfig.hardware_runtime_factory` to the new module:function.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .config import SACV2PathblindConfig
from .hardware_adapter import HardwarePose, HardwareResetResult, HardwareStepResult


class StubHardwareRuntime:
    """Template runtime showing the required hardware interface.

    To build a real runtime, keep the same public methods and make them do the
    following:
    - `start_episode(spec)`: reset the tank state and return initial frames plus pose
    - `step(...)`: apply one command interval and return newly collected frames plus pose
    - `close()`: cleanly release hardware resources

    The SAC code never imports lab-specific drivers directly; it only talks to
    the runtime through this object.
    """

    def __init__(self, cfg: SACV2PathblindConfig):
        self.cfg = cfg

    def start_episode(self, spec: dict[str, Any]) -> HardwareResetResult:
        """Reset hardware for a new episode.

        `spec` contains the episode setup chosen by the trainer, including path
        name, start pose, timing, motion limits, and object-gap safety values.

        Return a `HardwareResetResult` with:
        - `sensor_frames`: `(3,3,C)` or `(N,3,3,C)`
        - `pose`: current `HardwarePose` after reset
        - optional `info`: extra measurements or diagnostics
        """
        raise NotImplementedError(
            'StubHardwareRuntime is a template only. Implement your runtime and set '
            'cfg.hardware_runtime_factory to your module:function.'
        )

    def step(self, cmd_vx_mm_per_ms: float, cmd_vy_mm_per_ms: float, hold_frames: int) -> HardwareStepResult:
        """Run one control interval on hardware.

        Apply the requested `vx` and `vy` command, wait or stream for
        `hold_frames` sensor frames, and then return a `HardwareStepResult`.

        Recommended `info` flags are `command_failed`, `sensor_timeout`,
        `state_timeout`, `hardware_error`, and optionally object-gap values.
        """
        _ = (cmd_vx_mm_per_ms, cmd_vy_mm_per_ms, hold_frames)
        raise NotImplementedError(
            'StubHardwareRuntime is a template only. Implement your runtime and set '
            'cfg.hardware_runtime_factory to your module:function.'
        )

    def close(self) -> None:
        """Release hardware resources. It is fine for this to be a no-op."""
        return None


# Optional tiny fake factory for shape debugging only.
def create_debug_runtime(cfg: SACV2PathblindConfig):
    class _DebugRuntime(StubHardwareRuntime):
        def __init__(self, cfg_: SACV2PathblindConfig):
            super().__init__(cfg_)
            self._t = 0.0
            self._x = float(cfg_.xloc_start_mm)
            self._y = float(cfg_.yloc_start_mm)
            self._obj_x = float(self._x + cfg_.initial_object_gap_mm)
            self._obj_vx = float(cfg_.object_tangential_speed_mm_per_ms)

        def start_episode(self, spec: dict[str, Any]) -> HardwareResetResult:
            self._t = 0.0
            self._x = float(spec['start_x_mm'])
            self._y = float(spec['start_y_mm'])
            self._obj_x = float(self._x + cfg.initial_object_gap_mm)
            frames = np.zeros((int(cfg.rl_interval), 3, 3, cfg.num_signal_channels), dtype=np.float32)
            pose = HardwarePose(
                x_mm=self._x,
                y_mm=self._y,
                vx_mm_per_ms=0.0,
                vy_mm_per_ms=0.0,
                time_ms=self._t,
            )
            return HardwareResetResult(
                sensor_frames=frames,
                pose=pose,
                info={
                    'debug_runtime': True,
                    'object_x_mm': float(self._obj_x),
                    'object_x_gap_mm': float(self._obj_x - self._x),
                },
            )

        def step(self, cmd_vx_mm_per_ms: float, cmd_vy_mm_per_ms: float, hold_frames: int) -> HardwareStepResult:
            dt = float(cfg.sensor_frame_period_ms) * float(hold_frames)
            self._t += dt
            self._x += float(cmd_vx_mm_per_ms) * dt
            self._y += float(cmd_vy_mm_per_ms) * dt
            self._obj_x += self._obj_vx * dt
            frames = np.zeros((int(hold_frames), 3, 3, cfg.num_signal_channels), dtype=np.float32)
            pose = HardwarePose(
                x_mm=self._x,
                y_mm=self._y,
                vx_mm_per_ms=float(cmd_vx_mm_per_ms),
                vy_mm_per_ms=float(cmd_vy_mm_per_ms),
                time_ms=self._t,
            )
            truncated = bool(self._t >= float(cfg.episode_time_ms))
            return HardwareStepResult(
                sensor_frames=frames,
                pose=pose,
                done=False,
                truncated=truncated,
                info={
                    'debug_runtime': True,
                    'object_x_mm': float(self._obj_x),
                    'object_x_gap_mm': float(self._obj_x - self._x),
                },
            )

    return _DebugRuntime(cfg)


def create_runtime(cfg: SACV2PathblindConfig):
    """Default factory used by config.

    In a real deployment you will usually replace this function with your own
    `create_runtime(cfg)` living in a separate module, then set
    `hardware_runtime_factory` to `your_module:create_runtime`.
    """
    return StubHardwareRuntime(cfg)
