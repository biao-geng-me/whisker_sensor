"""Configuration for hardware-only path-blind SAC training.

This dataclass is the central place where a new reader can understand what the
hardware trainer assumes about timing, motion limits, safety checks, replay
settings, and run control. The rest of the package mostly reads values from
this object rather than hard-coding behavior in multiple places.

This package assumes direct tank interaction only.
There is no simulator backend in this package.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_TARGET_PATHS_ROOT = REPO_ROOT / 'hardware_target_paths'

@dataclass
class SACV2PathblindConfig:
    """Tunable parameters for hardware-only path-blind SAC training.

    A few fields are especially important when bringing up a new hardware run:
    - `path_subs`: training paths the tank should sample from
    - `eval_path_subs`: prescribed paths used when running evaluation
    - `fixed_x_speed_mm_per_ms`: commanded forward x velocity of the whisker array
    - `object_tangential_speed_mm_per_ms`: expected forward velocity of the cylinder
    - `min_object_x_gap_terminate_mm`: hard safety stop when the whisker gets too close
    - `target_update_to_data_ratio`: how many SAC updates to run between episodes
    """

    # ------------------------------------------------------------------
    # Scenario selection
    # ------------------------------------------------------------------
    path_sub: str = 'path1'
    path_subs: tuple[str, ...] | list[str] | None = tuple(f'path{i}' for i in range(1, 17))
    eval_path_subs: tuple[str, ...] | list[str] | None = ('path1', 'path5', 'path9', 'path13')

    # ------------------------------------------------------------------
    # Hardware timing
    # ------------------------------------------------------------------
    # Sensor frame period, typically 12.5 ms (80 Hz).
    sensor_frame_period_ms: float = 12.5
    # Policy control interval in sensor frames, typically 4 (20 Hz control).
    rl_interval: int = 4
    # History horizon used by the policy input.
    history_ms: float = 200.0

    # ------------------------------------------------------------------
    # Motion model
    # ------------------------------------------------------------------
    fixed_x_speed_mm_per_ms: float = 0.16
    y_speed_limit_mm_per_ms: float = 0.15
    rotation_change_limit_deg_per_control_step: float = 1.5
    vel_max_mm_per_ms: float = 0.4

    # ------------------------------------------------------------------
    # Path / reward geometry
    # ------------------------------------------------------------------
    target_paths_root: Path = DEFAULT_TARGET_PATHS_ROOT
    # Preferred file naming at target_paths_root: {path_sub}_xy.npy
    path_file_template: str = '{path_sub}_xy.npy'

    object_tangential_speed_mm_per_ms: float = 0.2
    initial_object_gap_mm: float = 200.0
    min_object_x_gap_terminate_mm: float = 25.0
    reward_corridor_half_width_mm: float = 180.0
    terminate_corridor_half_width_mm: float = 200.0

    # ------------------------------------------------------------------
    # Episode settings
    # ------------------------------------------------------------------
    episode_time_ms: float = 38000.0
    finish_line_mm: float = 3800.0
    xloc_start_mm: float = 200.0
    yloc_start_mm: float = 500.0
    start_on_path_initial_point: bool = True

    io_timeout_s: int = 180
    reset_retry_attempts: int = 2
    reset_retry_delay_s: float = 2.0
    settle_time_seconds: float = 0.0

    # ------------------------------------------------------------------
    # Hardware runtime plug-in
    # ------------------------------------------------------------------
    # Expected format: "module.submodule:function_name"
    # The factory is called as: factory(cfg) -> runtime object.
    hardware_runtime_factory: str = 'rl_sac_v4_pathblind_hardware.hardware_runtime_stub:create_runtime'

    # ------------------------------------------------------------------
    # Observation shape
    # ------------------------------------------------------------------
    num_whiskers: int = 9
    num_signal_channels: int = 2
    kin_dim: int = 6

    # ------------------------------------------------------------------
    # SAC hyperparameters
    # ------------------------------------------------------------------
    replay_size: int = 50_000
    batch_size: int = 128
    gamma: float = 0.99
    tau: float = 0.005
    actor_lr: float = 1e-4
    critic_lr: float = 1e-4
    alpha_lr: float = 1e-4
    init_temperature: float = 0.2

    # ------------------------------------------------------------------
    # Training loop
    # ------------------------------------------------------------------
    total_env_steps: int = 10_000_000
    max_runtime_seconds: float | None = None
    start_steps: int = 0
    update_after: int = 2_000

    # Between-episode update budget:
    # updates_this_episode = round(target_update_to_data_ratio * episode_length)
    target_update_to_data_ratio: float = 0.5
    max_updates_per_episode: int | None = None

    checkpoint_every_episodes: int = 1

    save_episode_plots: bool = True
    plot_every_episodes: int = 1

    seed: int = 7
    device: str = 'cpu'

    @property
    def history_steps(self) -> int:
        steps = int(round(self.history_ms / self.sensor_frame_period_ms))
        return max(steps, self.rl_interval)

    @property
    def sensor_shape(self) -> tuple[int, int, int, int]:
        return (self.history_steps, 3, 3, self.num_signal_channels)

    @property
    def action_dim(self) -> int:
        return 1

    def training_paths(self) -> tuple[str, ...]:
        """Return the configured training paths.

        The hardware trainer samples one path per episode from this list.
        Empty strings are ignored so that simple comma-separated CLI parsing
        can stay forgiving.
        """
        paths = tuple(self.path_subs) if self.path_subs else (self.path_sub,)
        filtered = tuple(path for path in paths if path)
        if not filtered:
            raise ValueError('No training paths were configured.')
        return filtered

