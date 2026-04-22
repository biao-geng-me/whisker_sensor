"""Hardware-only training environment adapter.

This module is the bridge between three pieces of the system:
- your hardware runtime, which actually talks to the tank
- the SAC trainer, which expects a simple reset/step environment interface
- the path and safety logic, which turns raw hardware state into rewards and
  termination decisions

For a first-time reader, this is the best place to understand what one RL step
means on hardware and where safety checks such as `too_far` and `too_close`
are computed.

No simulator backend is used here.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from .config import SACV2PathblindConfig
from .path_utils import calc_path_data, local_path_frame


@dataclass
class HardwarePose:
    """Latest physical state reported by the runtime.

    Every field uses millimeters or milliseconds so the trainer never has to
    guess units:
    - `x_mm`, `y_mm`: current array position in the tank frame
    - `vx_mm_per_ms`, `vy_mm_per_ms`: measured or estimated current velocities
    - `time_ms`: elapsed time since the current episode started

    The adapter uses this object for reward computation, finish-line checks,
    and safety logic such as `too_close` and `too_far`.
    """

    x_mm: float
    y_mm: float
    vx_mm_per_ms: float
    vy_mm_per_ms: float
    time_ms: float


@dataclass
class HardwareResetResult:
    """Payload returned by `HardwareRuntime.start_episode()`.

    The runtime should return enough data for the trainer to build the first
    observation of the new episode:
    - `sensor_frames`: either `(3,3,C)` for one latest frame or `(N,3,3,C)`
      for a short history collected during reset
    - `pose`: the latest hardware pose after reset completes
    - `info`: optional debug or failure fields; may include `object_x_mm`,
      `object_x_gap_mm`, or implementation-specific metadata
    """

    sensor_frames: np.ndarray
    pose: HardwarePose
    info: dict[str, Any] | None = None


@dataclass
class HardwareStepResult:
    """Payload returned by `HardwareRuntime.step()`.

    The runtime should report what happened after holding the commanded action
    for `hold_frames` sensor frames:
    - `sensor_frames`: newly collected frames since the last action
    - `pose`: latest state sample at the end of the action hold
    - `done`: set only if the runtime itself wants to end the episode
    - `truncated`: set if the runtime stopped for a non-task reason such as
      its own timeout handling
    - `info`: optional status flags and extra measurements
    """

    sensor_frames: np.ndarray
    pose: HardwarePose
    done: bool = False
    truncated: bool = False
    info: dict[str, Any] | None = None


class HardwareRuntime(Protocol):
    """Protocol your real hardware runtime must satisfy.

    Think of this as the only boundary between SAC code and lab-specific I/O.
    If a new person wants to integrate a tank controller, this is the contract
    they need to implement.

    Expected lifecycle:
    1. trainer creates the runtime once via `factory(cfg)`
    2. trainer calls `start_episode(spec)` before each episode
    3. trainer repeatedly calls `step(cmd_vx_mm_per_ms, cmd_vy_mm_per_ms, hold_frames)`
    4. trainer calls `close()` when the run ends

    The runtime may use any transport internally: direct driver calls, shared
    memory, serial, DAQ, ROS, subprocesses, etc. The trainer does not care as
    long as this interface is respected.
    """

    def start_episode(self, spec: dict[str, Any]) -> HardwareResetResult:
        """Prepare hardware for a fresh episode and return the initial observation.

        Parameters
        ----------
        spec:
            Episode specification assembled by the adapter. It contains the path
            name, episode timing, start pose, commanded x velocity, y limits,
            object motion assumptions, and safety thresholds.

        Returns
        -------
        HardwareResetResult
            Must contain sensor frames with shape `(3,3,C)` or `(N,3,3,C)`, a
            valid `HardwarePose`, and optional `info`. Returning more than one
            frame is allowed and helps seed the initial history buffer.

        Notes
        -----
        Raise an exception if reset truly failed. The training loop will retry
        according to `reset_retry_attempts`.
        """
        ...

    def step(self, cmd_vx_mm_per_ms: float, cmd_vy_mm_per_ms: float, hold_frames: int) -> HardwareStepResult:
        """Execute one control interval and return the newest data.

        Parameters
        ----------
        cmd_vx_mm_per_ms, cmd_vy_mm_per_ms:
            Velocity commands the policy wants applied during this interval.
        hold_frames:
            Number of sensor frames for which the command should be held. With
            the default config this is 4 frames, i.e. one RL step at 20 Hz.

        Returns
        -------
        HardwareStepResult
            Must contain all frames gathered during the interval plus the final
            pose. Set `done` or `truncated` only when the runtime itself needs
            to stop; task-level checks like `too_far` and `too_close` are added
            later by the adapter.

        Recommended `info` keys
        -----------------------
        `command_failed`, `sensor_timeout`, `state_timeout`, `hardware_error`,
        `infrastructure_failure`, `object_x_mm`, `object_x_gap_mm`.
        """
        ...

    def close(self) -> None:
        """Release devices, processes, or connections held by the runtime."""
        ...


def _resolve_runtime(cfg: SACV2PathblindConfig) -> HardwareRuntime:
    """Load the runtime factory declared in config and validate its interface."""
    spec = str(cfg.hardware_runtime_factory).strip()
    if ':' not in spec:
        raise ValueError(
            'hardware_runtime_factory must have format "module:function" '
            f'but got {spec!r}'
        )

    module_name, func_name = spec.split(':', 1)
    module = importlib.import_module(module_name)
    factory = getattr(module, func_name, None)
    if factory is None:
        raise AttributeError(f'Runtime factory {func_name!r} not found in module {module_name!r}.')

    runtime = factory(cfg)
    for meth in ('start_episode', 'step', 'close'):
        if not hasattr(runtime, meth):
            raise TypeError(f'Runtime object from {spec!r} is missing required method {meth!r}.')
    return runtime


def _candidate_path_files(cfg: SACV2PathblindConfig, path_sub: str) -> list[Path]:
    """Return the path file locations we are willing to try for one path name."""
    root = Path(cfg.target_paths_root)
    template_path = root / cfg.path_file_template.format(path_sub=path_sub)
    candidates = [
        template_path,
        root / f'{path_sub}_xy.npy',
        root / f'{path_sub}.npy',
        root / f'{path_sub}_xy.csv',
        root / f'{path_sub}.csv',
        root / path_sub / 'path_xy.npy',
        root / path_sub / 'path_xy.csv',
    ]

    seen = set()
    uniq = []
    for c in candidates:
        if c not in seen:
            seen.add(c)
            uniq.append(c)
    return uniq


def _load_path_xy(cfg: SACV2PathblindConfig, path_sub: str) -> np.ndarray:
    """Load one target path as an `N x 2` array of `[x_mm, y_mm]` samples."""
    last_error = None
    for path in _candidate_path_files(cfg, path_sub):
        if not path.exists():
            continue
        try:
            if path.suffix.lower() == '.npy':
                arr = np.load(path)
            else:
                arr = np.loadtxt(path, delimiter=',')
        except Exception as exc:  # pragma: no cover - file parsing guard
            last_error = exc
            continue

        arr = np.asarray(arr, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 2 or arr.shape[0] < 2:
            raise ValueError(f'Invalid path file {path}: expected shape [N,2+] with N>=2, got {arr.shape}')
        return arr[:, :2]

    msg = (
        f'Could not load target trajectory for {path_sub!r} under {cfg.target_paths_root}. '
        f'Tried: {[str(p) for p in _candidate_path_files(cfg, path_sub)]}'
    )
    if last_error is not None:
        msg += f' Last parse error: {last_error}'
    raise FileNotFoundError(msg)


def _normalize_sensor_frames(sensor_frames: np.ndarray, channels: int) -> np.ndarray:
    """Normalize runtime sensor output to `(N, 3, 3, C)` for the trainer."""
    arr = np.asarray(sensor_frames, dtype=np.float32)
    if arr.shape == (3, 3, channels):
        arr = arr[None, ...]
    if arr.ndim != 4 or arr.shape[1:] != (3, 3, channels):
        raise ValueError(
            'sensor_frames must have shape (3,3,C) or (N,3,3,C). '
            f'Got {arr.shape} with C={channels}'
        )
    return arr


class HardwareTrainingEnv:
    """Single-env hardware wrapper with path-based reward shaping.

    The trainer sees this object as its environment. Internally it:
    - samples a path for each episode
    - asks the runtime to reset and step the tank
    - keeps a rolling sensor history for the policy input
    - computes reward, finish conditions, and safety termination flags
    """

    def __init__(self, cfg: SACV2PathblindConfig):
        self.cfg = cfg
        self.history_steps = int(cfg.history_steps)
        self.kin_dim = int(cfg.kin_dim)
        self.action_dim = int(cfg.action_dim)
        self.sensor_shape = cfg.sensor_shape
        self.training_paths = cfg.training_paths()
        self.rng = np.random.default_rng(cfg.seed)

        self.path_xy_by_sub: dict[str, np.ndarray] = {}
        self.path_data_by_sub: dict[str, np.ndarray] = {}
        for path_sub in self.training_paths:
            xy = _load_path_xy(cfg, path_sub)
            self.path_xy_by_sub[path_sub] = xy
            self.path_data_by_sub[path_sub] = calc_path_data(xy)

        self.current_path_sub = self.training_paths[0]
        self.path_xy = self.path_xy_by_sub[self.current_path_sub]
        self.path_data = self.path_data_by_sub[self.current_path_sub]

        self.sensor_history = np.zeros((1, *self.sensor_shape), dtype=np.float32)
        self.kinematics = np.zeros((1, self.kin_dim), dtype=np.float32)
        self.prev_y_velocity = 0.0
        self.prev_heading_angle = 0.0
        self.episode_index = 0
        self._episode_array_start_x_mm = float(self.cfg.xloc_start_mm)
        self._episode_object_start_x_mm = float(self._episode_array_start_x_mm + self.cfg.initial_object_gap_mm)

        self.runtime = _resolve_runtime(cfg)

    def _sample_path_sub(self) -> str:
        idx = int(self.rng.integers(len(self.training_paths)))
        return str(self.training_paths[idx])

    def _start_xy(self, path_sub: str) -> tuple[float, float]:
        if self.cfg.start_on_path_initial_point:
            path_xy = self.path_xy_by_sub[path_sub]
            return float(path_xy[0, 0]), float(path_xy[0, 1])
        return float(self.cfg.xloc_start_mm), float(self.cfg.yloc_start_mm)

    def _delta_angle_to_command(self, norm_action: float) -> tuple[float, float, float]:
        prev_vy = float(self.prev_y_velocity)
        delta_angle_norm = float(np.clip(norm_action, -1.0, 1.0))

        cmd_vx = float(np.clip(
            self.cfg.fixed_x_speed_mm_per_ms,
            -self.cfg.vel_max_mm_per_ms,
            self.cfg.vel_max_mm_per_ms,
        ))
        if abs(cmd_vx) <= 1e-9:
            return prev_vy, 0.0, 0.0

        prev_theta = float(np.arctan2(prev_vy, cmd_vx))
        max_step_rad = float(np.deg2rad(max(0.0, float(self.cfg.rotation_change_limit_deg_per_control_step))))
        requested_delta_angle_deg = float(np.rad2deg(delta_angle_norm * max_step_rad))
        candidate_theta = prev_theta + float(np.deg2rad(requested_delta_angle_deg))
        candidate_vy = float(cmd_vx * np.tan(candidate_theta))
        limited_vy = float(np.clip(
            candidate_vy,
            -float(self.cfg.y_speed_limit_mm_per_ms),
            float(self.cfg.y_speed_limit_mm_per_ms),
        ))
        final_theta = float(np.arctan2(limited_vy, cmd_vx))
        applied_rotation_change_deg = float(np.rad2deg(final_theta - prev_theta))
        limited_vy = float(np.clip(
            limited_vy,
            -float(self.cfg.y_speed_limit_mm_per_ms),
            float(self.cfg.y_speed_limit_mm_per_ms),
        ))
        return limited_vy, requested_delta_angle_deg, applied_rotation_change_deg

    @staticmethod
    def _heading_from_velocity(vx: float, vy: float, speed_floor: float = 1e-6) -> float:
        if float(np.hypot(vx, vy)) <= float(speed_floor):
            return 0.0
        return float(np.arctan2(vy, vx))

    @staticmethod
    def _wrap_angle_delta(angle_now: float, angle_prev: float) -> float:
        return float(np.arctan2(np.sin(angle_now - angle_prev), np.cos(angle_now - angle_prev)))

    def _make_kinematics(self, pose: HardwarePose) -> np.ndarray:
        heading_angle = self._heading_from_velocity(float(pose.vx_mm_per_ms), float(pose.vy_mm_per_ms))
        prev_delta_heading_angle = self._wrap_angle_delta(heading_angle, self.prev_heading_angle)
        return np.array(
            [
                float(pose.x_mm),
                float(pose.y_mm),
                float(pose.vx_mm_per_ms),
                float(pose.vy_mm_per_ms),
                float(heading_angle),
                float(prev_delta_heading_angle),
            ],
            dtype=np.float32,
        )

    def _tracking_info(self, pose: HardwarePose, runtime_info: dict[str, Any] | None = None) -> dict[str, Any]:
        """Compute path-tracking and object-gap metrics from the latest hardware state."""
        frame = local_path_frame(
            self.path_data,
            np.array([float(pose.x_mm), float(pose.y_mm)], dtype=np.float64),
        )
        finish_line_reached = float(pose.x_mm) >= float(self.cfg.finish_line_mm)

        runtime_data = dict(runtime_info or {})

        object_x_mm = runtime_data.get('object_x_mm')
        try:
            object_x_mm = None if object_x_mm is None else float(object_x_mm)
        except (TypeError, ValueError):
            object_x_mm = None
        if object_x_mm is None:
            object_x_mm = float(
                self._episode_object_start_x_mm
                + float(self.cfg.object_tangential_speed_mm_per_ms) * float(pose.time_ms)
            )

        object_x_gap_mm = runtime_data.get('object_x_gap_mm')
        try:
            object_x_gap_mm = None if object_x_gap_mm is None else float(object_x_gap_mm)
        except (TypeError, ValueError):
            object_x_gap_mm = None
        if object_x_gap_mm is None:
            object_x_gap_mm = float(object_x_mm - float(pose.x_mm))

        return {
            'path_sub': self.current_path_sub,
            'target_path_x_mm': float(frame['point'][0]),
            'target_path_y_mm': float(frame['point'][1]),
            'signed_lateral_error_mm': float(frame['signed_lateral_error']),
            'path_progress_mm': float(frame['s']),
            'object_x_mm': float(object_x_mm),
            'object_x_gap_mm': float(object_x_gap_mm),
            'finish_line_reached': bool(finish_line_reached),
        }

    def _reward_done(
        self,
        pose: HardwarePose,
        runtime_done: bool,
        runtime_truncated: bool,
        runtime_info: dict[str, Any] | None = None,
    ) -> tuple[float, bool, bool, dict[str, Any]]:
        """Turn tracking metrics into reward plus done/truncated flags."""
        info = self._tracking_info(pose, runtime_info=runtime_info)
        lateral = abs(float(info['signed_lateral_error_mm']))

        reward = 1.0 - (lateral / float(self.cfg.reward_corridor_half_width_mm))
        reward = float(np.clip(reward, -1.0, 1.0))

        too_far = lateral > float(self.cfg.terminate_corridor_half_width_mm)
        too_close = float(info.get('object_x_gap_mm', np.inf)) < float(self.cfg.min_object_x_gap_terminate_mm)
        time_limit_reached = bool(float(pose.time_ms) >= float(self.cfg.episode_time_ms))

        finish_line_reached = bool(info.get('finish_line_reached', False))

        done = bool(runtime_done or too_far or too_close or finish_line_reached)
        truncated = bool(runtime_truncated or time_limit_reached)

        if too_far:
            reward -= 2.0
        if too_close:
            reward -= 2.0

        info.update({
            'too_far': bool(too_far),
            'too_close': bool(too_close),
            'finish_line_reached': bool(finish_line_reached),
            'time_limit_reached': bool(time_limit_reached),
            'runtime_done': bool(runtime_done),
            'runtime_truncated': bool(runtime_truncated),
        })
        return reward, done, truncated, info

    def reset(self):
        """Start one new hardware episode and rebuild the observation history."""
        self.episode_index += 1
        self.current_path_sub = self._sample_path_sub()
        self.path_xy = self.path_xy_by_sub[self.current_path_sub]
        self.path_data = self.path_data_by_sub[self.current_path_sub]

        start_x, start_y = self._start_xy(self.current_path_sub)
        spec = {
            'episode_index': int(self.episode_index),
            'path_sub': self.current_path_sub,
            'episode_time_ms': float(self.cfg.episode_time_ms),
            'sensor_frame_period_ms': float(self.cfg.sensor_frame_period_ms),
            'rl_interval': int(self.cfg.rl_interval),
            'start_x_mm': float(start_x),
            'start_y_mm': float(start_y),
            'fixed_x_speed_mm_per_ms': float(self.cfg.fixed_x_speed_mm_per_ms),
            'y_speed_limit_mm_per_ms': float(self.cfg.y_speed_limit_mm_per_ms),
            'rotation_change_limit_deg_per_control_step': float(self.cfg.rotation_change_limit_deg_per_control_step),
            'vel_max_mm_per_ms': float(self.cfg.vel_max_mm_per_ms),
            'object_tangential_speed_mm_per_ms': float(self.cfg.object_tangential_speed_mm_per_ms),
            'initial_object_gap_mm': float(self.cfg.initial_object_gap_mm),
            'min_object_x_gap_terminate_mm': float(self.cfg.min_object_x_gap_terminate_mm),
        }

        reset_out = self.runtime.start_episode(spec)
        frames = _normalize_sensor_frames(reset_out.sensor_frames, self.cfg.num_signal_channels)

        history = np.repeat(frames[-1:, ...], self.history_steps, axis=0)
        take = min(self.history_steps, frames.shape[0])
        history[-take:] = frames[-take:]
        self.sensor_history[0] = history

        self.prev_y_velocity = 0.0
        self.prev_heading_angle = self._heading_from_velocity(
            float(reset_out.pose.vx_mm_per_ms),
            float(reset_out.pose.vy_mm_per_ms),
        )
        self.kinematics[0] = self._make_kinematics(reset_out.pose)

        self._episode_array_start_x_mm = float(reset_out.pose.x_mm)
        self._episode_object_start_x_mm = float(self._episode_array_start_x_mm + float(self.cfg.initial_object_gap_mm))

        info = self._tracking_info(reset_out.pose, runtime_info=reset_out.info)
        info.setdefault('infrastructure_failure', False)

        return (self.sensor_history.copy(), self.kinematics.copy()), [info]

    def step(self, action: np.ndarray):
        """Apply one policy action, advance hardware, and compute RL outputs."""
        action = np.asarray(action, dtype=np.float32).reshape(1, 1)
        norm_action = float(np.clip(action[0, 0], -1.0, 1.0))
        prev_y_velocity = float(self.prev_y_velocity)

        cmd_vy, requested_delta_angle_deg, applied_rotation_change_deg = self._delta_angle_to_command(norm_action)
        cmd_vx = float(np.clip(
            self.cfg.fixed_x_speed_mm_per_ms,
            -self.cfg.vel_max_mm_per_ms,
            self.cfg.vel_max_mm_per_ms,
        ))

        step_out = self.runtime.step(
            cmd_vx_mm_per_ms=cmd_vx,
            cmd_vy_mm_per_ms=cmd_vy,
            hold_frames=int(self.cfg.rl_interval),
        )

        frames = _normalize_sensor_frames(step_out.sensor_frames, self.cfg.num_signal_channels)
        merged = np.concatenate([self.sensor_history[0], frames], axis=0)
        self.sensor_history[0] = merged[-self.history_steps:]

        self.prev_y_velocity = cmd_vy
        self.kinematics[0] = self._make_kinematics(step_out.pose)
        self.prev_heading_angle = float(self.kinematics[0, 4])

        runtime_info = dict(step_out.info or {})

        reward, done, truncated, info = self._reward_done(
            pose=step_out.pose,
            runtime_done=bool(step_out.done),
            runtime_truncated=bool(step_out.truncated),
            runtime_info=runtime_info,
        )
        command_failed = bool(runtime_info.get('command_failed', False))
        sensor_timeout = bool(runtime_info.get('sensor_timeout', False))
        state_timeout = bool(runtime_info.get('state_timeout', False))
        hardware_error = bool(runtime_info.get('hardware_error', False))
        infrastructure_failure = bool(
            runtime_info.get('infrastructure_failure', False)
            or command_failed
            or sensor_timeout
            or state_timeout
            or hardware_error
        )

        info.update(runtime_info)
        info.update({
            'command_vx_mm_per_ms': float(cmd_vx),
            'command_vy_mm_per_ms': float(cmd_vy),
            'command_delta_vy_mm_per_ms': float(cmd_vy - prev_y_velocity),
            'requested_rotation_change_deg': float(requested_delta_angle_deg),
            'applied_rotation_change_deg': float(applied_rotation_change_deg),
            'rotation_change_limit_deg_per_control_step': float(self.cfg.rotation_change_limit_deg_per_control_step),
            'command_failed': bool(command_failed),
            'sensor_timeout': bool(sensor_timeout),
            'state_timeout': bool(state_timeout),
            'hardware_error': bool(hardware_error),
            'infrastructure_failure': bool(infrastructure_failure),
        })

        obs = (self.sensor_history.copy(), self.kinematics.copy())
        rewards = np.array([reward], dtype=np.float32)
        dones = np.array([done], dtype=bool)
        truncated_arr = np.array([truncated], dtype=bool)
        infos = [info]
        return obs, rewards, dones, truncated_arr, infos

    def close(self):
        self.runtime.close()


def create_training_env(cfg: SACV2PathblindConfig) -> HardwareTrainingEnv:
    """Create the hardware-only training environment."""
    return HardwareTrainingEnv(cfg)
