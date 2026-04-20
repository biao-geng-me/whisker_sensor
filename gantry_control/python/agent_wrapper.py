import numpy as np
import logging
import os
import shutil
from dataclasses import asdict
from pathlib import Path

logger = logging.getLogger('server')

try:
    from agents.hardware_handoff_v2.path7_object_adapter import build_policy
except Exception:
    build_policy = None

# SAC train-mode imports — only loaded when mode == "train" with the matching policy package.
_sac_imports_ok = False
try:
    from agents.rl_sac_v4_pathblind_hardware.models import SACAgent
    from agents.rl_sac_v4_pathblind_hardware.replay_buffer import ReplayBuffer
    from agents.rl_sac_v4_pathblind_hardware.config import SACV2PathblindConfig
    from agents.rl_sac_v4_pathblind_hardware.train import run_between_episode_updates
    from agents.rl_sac_v4_pathblind_hardware.path_utils import calc_path_data, local_path_frame
    import torch
    import random
    _sac_imports_ok = True
except Exception as _sac_import_err:
    SACAgent = None
    ReplayBuffer = None
    SACV2PathblindConfig = None
    run_between_episode_updates = None

class AgentWrapper:
    """Wraps the local DRL inference engine."""
    
    def __init__(self, config):
        self.config = config
        self.state_dim = config.get("state_dim")
        self.action_dim = config.get("action_dim", 2)
        self.n_rl_interval = config.get("n_rl_interval")
        self.n_ch_total = config.get("n_ch_total")
        reward_source = str(config.get("reward_source", "matlab")).strip().lower()
        self.reward_source = reward_source if reward_source in {"matlab", "server"} else "matlab"
        self.policy = None
        self.use_object_policy = False
        self.use_sac_train = False
        
        logger.info(f"[Agent] Initializing with config: mode={config.get('mode')}, state_dim={self.state_dim}, action_dim={self.action_dim}")

        self._init_policy()
        
        # Memory for the current episode's trajectory
        self.trajectory = [] 

    def _init_policy(self):
        """Initialize deployed policy adapter when available."""
        mode = self.config.get("mode", "infer")
        policy_dir = self.config.get("policy_package_dir", "")
        if mode == "train" and "rl_sac_v4_pathblind_hardware" in str(policy_dir):
            logger.debug("[Agent] Initializing SAC train mode...")
            self._init_sac_train()
            return

        if build_policy is None:
            logger.info("[Agent] Deployed object policy unavailable; using built-in fallback.")
            return

        try:
            package_dir = self.config.get(
                "policy_package_dir",
                "agents/hardware_handoff_v2",
            )
            device = self.config.get("policy_device", "cpu")
            signal_shape = tuple(self.config.get("signal_shape", (3, 3, 2)))
            self.policy = build_policy(
                package_dir=package_dir,
                device=device,
                signal_shape=signal_shape,
            )
            self.use_object_policy = True
            logger.info(f"[Agent] Loaded deployed object policy from {package_dir} (device={device}).")
        except Exception as ex:
            logger.error(f"[Agent] Failed to load deployed object policy: {ex}")
            logger.info("[Agent] Falling back to built-in dummy controller.")
            self.policy = None
            self.use_object_policy = False

    def reset(self, initial_state, episode_meta=None):
        """Clears trajectory memory and resets episode state for a new episode."""
        logger.debug("[Agent] reset() called")
        self.trajectory = []
        
        logger.debug("[Agent] Resetting policy...")
        if self.use_object_policy:
            logger.debug("[Agent] Calling policy.reset()...")
            self.policy.reset()
            logger.debug("[Agent] policy.reset() complete")
        
        logger.debug("[Agent] Resetting SAC train mode state...")
        if self.use_sac_train:
            self._sac_begin_episode(initial_state, episode_meta=episode_meta)
        
        logger.debug("[Agent] Computing first action...")
        action = self._compute_action(initial_state)
        logger.debug(f"[Agent] reset() computed action: {action}")
        return action

    def step(self, state, reward, done, truncated, record=True):
        """Logs the transition and computes the next action."""
        logger.debug(
            "[Agent.step] Called with reward=%.6f done=%s truncated=%s use_sac_train=%s",
            float(reward),
            done,
            truncated,
            self.use_sac_train,
        )
        try:
            if record and not self.use_sac_train:
                # Infer mode trajectory recording for HPC (legacy path)
                self.trajectory.append({
                    "state": state,
                    "reward": reward,
                    "done": done,
                    "truncated": truncated
                })

            is_terminal = (done > 0.5) or (truncated > 0.5)

            if self.use_sac_train:
                self._sac_env_steps_seen += 1

            if is_terminal:
                logger.debug("[Agent.step] Terminal step received")
                if self.use_sac_train and self._sac_prev_sensor is not None:
                    self._sac_store_transition(state, reward, done=True, truncated=bool(truncated > 0.5))
                return None # Episode over, no action needed

            if self.use_sac_train:
                logger.debug("[Agent.step] Storing SAC transition before next action")
                self._sac_store_transition(state, reward, done=False, truncated=bool(truncated > 0.5))

            action = self._compute_action(state)
            logger.debug(f"[Agent.step] Returning action: {action}")
            return action
        except Exception as ex:
            logger.error(f"[Agent.step] Exception: {ex}", exc_info=True)
            return [0.0] * self.action_dim

    def between_episode_update(self, episode_length: int) -> bool:
        """Run SAC gradient updates between episodes (train mode only).

        Returns True if the agent suggests a hardware reset is needed
        (e.g., replay is too sparse to have learned anything — not currently used
        as a reset trigger, kept False for now).
        """
        if not self.use_sac_train:
            return False
        updates_run, update_seconds, metrics = run_between_episode_updates(
            agent=self._sac_agent,
            replay=self._sac_replay,
            cfg=self._sac_cfg,
            episode_length=episode_length,
        )
        if updates_run > 0:
            msg = f"[Agent] {updates_run} updates in {update_seconds:.1f}s"
            if metrics:
                msg += (
                    f" | actor_mean={metrics['actor_loss']:.3f}"
                    f" q1_mean={metrics['q1_loss']:.3f}"
                    f" alpha_mean={metrics['alpha']:.3f}"
                )
                self._sac_last_update_metrics = metrics
            logger.info(msg)
        else:
            logger.debug(f"[Agent] No updates (replay size={self._sac_replay.size}, need>={self._sac_cfg.update_after})")
        return False

    def _compute_action(self, state):
        """Compute control action from deployed policy or fallback agent."""
        logger.debug("[Agent] _compute_action() called")
        try:
            if self.use_sac_train:
                logger.debug("[Agent] Using SAC train mode")
                action = self._sac_act(state)
                logger.debug(f"[Agent] _sac_act() returned: {action}")
                return action

            if self.use_object_policy:
                logger.debug("[Agent] Using object policy")
                obs = np.array(state, dtype=np.float32).reshape(self.n_rl_interval, self.n_ch_total)
                action = self.policy.act(
                    observation=obs,
                    reward=0.0,
                    done=False,
                    truncated=False,
                    info={},
                )
                action_arr = np.asarray(action, dtype=np.float64).reshape(-1)
                return action_arr[: self.action_dim].tolist()

            # Fallback: dummy action based on state
            logger.debug("[Agent] Using dumb agent fallback")
            dummy_action = self.dumb_agent(state)
            # Ensure it matches expected dimension
            while len(dummy_action) < self.action_dim:
                dummy_action.append(0.0)
                
            return dummy_action[:self.action_dim]
        
        except Exception as ex:
            logger.error(f"[Agent] Exception in _compute_action: {ex}", exc_info=True)
            # Return zero action as last resort
            logger.error(f"[Agent] Returning zero action fallback")
            return [0.0] * self.action_dim

    def get_trajectory(self):
        """Returns the rollout data collected this episode."""
        return self.trajectory

    def update_weights(self, new_weights):
        """Legacy: loads weights from HPC bytes (infer mode only, no-op in train mode)."""
        if self.use_sac_train:
            return
        print(f"[Agent] Updating local weights... (Size: {len(new_weights)} bytes)")

    def dumb_agent(self, state):
        """Dumb control"""
        state_arr = np.array(state).reshape(self.n_rl_interval, self.n_ch_total)
        t    = state_arr[:,0]
        xloc = state_arr[:,1]
        yloc = state_arr[:,2]
        xvel = state_arr[:,3]
        yvel = state_arr[:,4]
        vel = 0.2
        T = 5000
        ycent = 400
        a1 = 1.309  # 75 degrees
        t1 = ycent / np.sin(a1) / vel
        if t[-1] < t1:
            angle = a1
            u_act = np.abs(np.cos(angle)) * vel
            v_act = np.sin(angle) * vel
        else:
            angle = 2 * np.pi * ((t[-1] - t1) / T) + a1
            u_act = np.abs(np.cos(angle)) * vel
            v_act = np.sin(angle) * vel
        print(f'{t[-1]:12.1f},x={xloc[-1]:8.3f},y={yloc[-1]:8.3f},{angle:8.3f},{u_act:5.3f},{v_act:6.3f}')
        return [u_act.item(), v_act.item()]

    @staticmethod
    def _finite_float_or_default(value, default: float, min_value: float | None = None) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = float(default)
        if not np.isfinite(parsed):
            parsed = float(default)
        if min_value is not None:
            parsed = max(float(min_value), parsed)
        return parsed

    @staticmethod
    def _summary_log_value(value):
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, float):
            return f"{value:.6g}"
        if isinstance(value, (list, tuple)):
            return repr(value)
        return str(value)

    def _log_sac_resolved_config(self):
        cfg_values = asdict(self._sac_cfg)
        summary = {
            **{f"cfg.{key}": self._summary_log_value(value) for key, value in cfg_values.items()},
            "derived.history_steps": self._summary_log_value(self._sac_cfg.history_steps),
            "derived.sensor_shape": self._summary_log_value(self._sac_cfg.sensor_shape),
            "derived.action_dim": self._summary_log_value(self._sac_cfg.action_dim),
            "resolved.fixed_vx_mm_per_ms": self._summary_log_value(self._sac_fixed_vx),
            "resolved.y_speed_limit_mm_per_ms": self._summary_log_value(self._sac_y_speed_limit),
            "resolved.rotation_change_limit_deg_per_control_step": self._summary_log_value(self._sac_rotation_change_limit_deg),
            "resolved.reward_source": self._summary_log_value(self.reward_source),
            "resolved.output_dir": self._summary_log_value(self._sac_output_dir),
            "resolved.resume": self._summary_log_value(self.config.get("resume", False)),
            "resolved.resume_path": self._summary_log_value(self.config.get("resume_path", "")),
            "resolved.keep_checkpoints": self._summary_log_value(self._sac_keep_checkpoints),
            "resolved.checkpoint_every_episodes": self._summary_log_value(self._sac_checkpoint_every),
            "resolved.loaded_path_count": self._summary_log_value(len(self._sac_path_data_list)),
            "resolved.use_random_paths_from_matlab": self._summary_log_value(
                getattr(self, '_sac_expect_episode_paths_from_matlab', False)
            ),
            "resolved.state_dim": self._summary_log_value(self.state_dim),
            "resolved.n_rl_interval": self._summary_log_value(self.n_rl_interval),
            "resolved.n_ch_total": self._summary_log_value(self.n_ch_total),
            "resolved.action_semantics": "delta_angle",
            "resolved.kin_features": "[x, y, vx, vy, heading_angle, prev_delta_heading_angle]",
            "resolved.min_warmup_episodes": self._summary_log_value(self._sac_min_warmup_episodes),
        }
        lines = ["[Agent] Resolved RL config:"] + [
            f"  {key}={summary[key]}" for key in sorted(summary.keys())
        ]
        logger.info("\n".join(lines))

    @staticmethod
    def _move_optimizer_to_device(optim, device: str) -> None:
        for state in optim.state.values():
            for key, value in state.items():
                if torch.is_tensor(value):
                    state[key] = value.to(device)

    @staticmethod
    def _heading_from_velocity(vx: float, vy: float, speed_floor: float = 1e-6) -> float:
        if float(np.hypot(vx, vy)) <= float(speed_floor):
            return 0.0
        return float(np.arctan2(vy, vx))

    @staticmethod
    def _wrap_angle_delta(angle_now: float, angle_prev: float) -> float:
        return float(np.arctan2(np.sin(angle_now - angle_prev), np.cos(angle_now - angle_prev)))

    # ------------------------------------------------------------------
    # SAC train-mode internals
    # ------------------------------------------------------------------

    def _init_sac_train(self):
        if not _sac_imports_ok:
            logger.error(f"[Agent] SAC imports failed; falling back to dummy. Error: {_sac_import_err}")
            return
        num_whiskers = self.config.get("num_whiskers", (self.n_ch_total - 5) // 2)
        n_channels = 2
        cfg = SACV2PathblindConfig()
        cfg.num_whiskers = num_whiskers
        cfg.num_signal_channels = n_channels
        cfg.device = self.config.get("policy_device", "cpu")
        cfg.episode_time_ms = float(self.config.get("episode_time_ms", cfg.episode_time_ms))
        cfg.start_steps = int(self.config.get("start_steps", cfg.start_steps))
        self._sac_cfg = cfg
        self._sac_fixed_vx = float(self.config.get("fixed_vx", cfg.fixed_x_speed_mm_per_ms))
        self._sac_y_speed_limit = float(self.config.get("y_speed_limit", cfg.y_speed_limit_mm_per_ms))
        self._sac_rotation_change_limit_deg = self._finite_float_or_default(
            self.config.get(
                "rotation_change_limit_deg_per_control_step",
                cfg.rotation_change_limit_deg_per_control_step,
            ),
            cfg.rotation_change_limit_deg_per_control_step,
            min_value=0.0,
        )
        self._sac_agent = SACAgent(kin_dim=cfg.kin_dim, action_dim=cfg.action_dim, cfg=cfg)
        self._sac_replay = ReplayBuffer(
            sensor_shape=cfg.sensor_shape,
            kin_dim=cfg.kin_dim,
            action_dim=cfg.action_dim,
            capacity=cfg.replay_size,
        )
        self._sac_sensor_history = np.zeros(
            (1, cfg.history_steps, 3, 3, n_channels), dtype=np.float32
        )
        self._sac_prev_vy = 0.0
        self._sac_prev_sensor = None
        self._sac_prev_kin = None
        self._sac_prev_action = None
        self._sac_episode_rotation_change_limit_deg = float(self._sac_rotation_change_limit_deg)

        # --- Path data for server-side reward ---
        self._sac_path_data_list = []
        self._sac_current_path_data = None
        self._sac_current_path_idx = 0
        self._sac_expect_episode_paths_from_matlab = bool(self.config.get("use_random_paths", False))
        raw_paths = self.config.get("path_data", [])
        if raw_paths:
            rng = np.random.default_rng(cfg.seed)
            self._sac_path_rng = rng
            for p in raw_paths:
                xy = np.asarray(p, dtype=np.float64)
                if xy.ndim == 2 and xy.shape[1] >= 2:
                    self._sac_path_data_list.append(calc_path_data(xy[:, :2]))
            logger.info(f"[Agent] Loaded {len(self._sac_path_data_list)} path(s) for server-side reward")
        else:
            self._sac_path_rng = np.random.default_rng(cfg.seed)
            if self._sac_expect_episode_paths_from_matlab:
                logger.info("[Agent] Random per-episode paths enabled; waiting for MATLAB to send each episode path.")
            else:
                logger.warning("[Agent] No path data received from MATLAB; server-side reward disabled")

        self._sac_reward_corridor = float(cfg.reward_corridor_half_width_mm)
        self._sac_terminate_corridor = float(cfg.terminate_corridor_half_width_mm)
        self._sac_min_gap_mm = float(cfg.min_object_x_gap_terminate_mm)
        self._sac_object_speed = float(cfg.object_tangential_speed_mm_per_ms)
        self._sac_initial_gap = float(cfg.initial_object_gap_mm)
        self._sac_y_min_mm = self._finite_float_or_default(self.config.get("y_min_mm", 0.0), 0.0)
        self._sac_y_max_mm = self._finite_float_or_default(self.config.get("y_max_mm", 900.0), 900.0)
        self._sac_boundary_margin_mm = self._finite_float_or_default(
            self.config.get("boundary_margin_mm", 50.0),
            50.0,
            min_value=0.0,
        )
        self._sac_episode_start_x = 0.0  # set on reset
        self._sac_episode_object_speed = float(self._sac_object_speed)
        self._sac_episode_delay_ms = 0.0

        # --- Checkpoint / resume ---
        self._sac_output_dir = self.config.get("output_dir", "")
        self._sac_keep_checkpoints = int(self.config.get("keep_checkpoints", 5))
        self._sac_checkpoint_every = int(self.config.get("checkpoint_every_episodes", 1))
        self._sac_total_env_steps = 0
        self._sac_episodes_completed = 0
        self._sac_last_update_metrics = {}
        self._sac_env_steps_seen = 0
        self._sac_min_warmup_episodes = 6
        self._sac_effective_start_steps = int(max(0, cfg.start_steps))
        self._sac_explore_rng = np.random.default_rng(cfg.seed)

        # Resume from checkpoint if requested
        resume = self.config.get("resume", False)
        resume_path = self.config.get("resume_path", "")
        if resume and resume_path and os.path.isfile(resume_path):
            self._load_checkpoint(resume_path)
        self._sac_env_steps_seen = int(self._sac_total_env_steps)

        self.use_sac_train = True
        logger.info(f"[Agent] SAC train mode ready. history_steps={cfg.history_steps} "
              f"sensor_shape={cfg.sensor_shape} device={cfg.device} reward_source={self.reward_source} "
              f"action_semantics=delta_angle rotation_step_limit_deg={self._sac_rotation_change_limit_deg:.3f} "
              f"configured_start_steps={cfg.start_steps}")
        self._log_sac_resolved_config()

    def _sac_begin_episode(self, initial_state, episode_meta=None):
        logger.debug("[Agent] Clearing SAC sensor history and state...")
        self._sac_sensor_history[:] = 0.0
        self._sac_prev_vy = 0.0
        self._sac_prev_sensor = None
        self._sac_prev_kin = None
        self._sac_prev_action = None

        meta = dict(episode_meta or {})
        episode_path_xy = meta.get("path_xy")
        if episode_path_xy is not None:
            xy = np.asarray(episode_path_xy, dtype=np.float64)
            if xy.ndim == 2 and xy.shape[0] >= 2 and xy.shape[1] >= 2:
                self._sac_current_path_data = calc_path_data(xy[:, :2])
                try:
                    self._sac_current_path_idx = int(meta.get("path_index"))
                except (TypeError, ValueError):
                    self._sac_current_path_idx = -1
                logger.debug(
                    "[Agent] Using episode-specific path with %d points",
                    xy.shape[0],
                )
            else:
                logger.warning("[Agent] Invalid episode-specific path received; falling back to configured paths")
                self._sac_select_path(path_index=meta.get("path_index"))
        else:
            if self._sac_expect_episode_paths_from_matlab and not self._sac_path_data_list:
                logger.warning("[Agent] Expected MATLAB to provide an episode path, but none was received.")
            self._sac_select_path(path_index=meta.get("path_index"))

        arr = np.asarray(initial_state, dtype=np.float64).reshape(self.n_rl_interval, self.n_ch_total)
        default_start_x = float(arr[-1, 1]) + self._sac_initial_gap

        try:
            self._sac_episode_start_x = float(meta.get("front_start_x_mm"))
        except (TypeError, ValueError):
            self._sac_episode_start_x = default_start_x

        try:
            self._sac_episode_object_speed = float(meta.get("object_speed_mm_per_ms"))
        except (TypeError, ValueError):
            self._sac_episode_object_speed = float(self._sac_object_speed)

        try:
            self._sac_episode_delay_ms = max(0.0, float(meta.get("delay_ms")))
        except (TypeError, ValueError):
            self._sac_episode_delay_ms = 0.0
        self._sac_episode_rotation_change_limit_deg = self._finite_float_or_default(
            meta.get("rotation_change_limit_deg_per_control_step"),
            self._sac_rotation_change_limit_deg,
            min_value=0.0,
        )
        control_step_ms = float(self._sac_cfg.sensor_frame_period_ms) * float(self._sac_cfg.rl_interval)
        episode_control_window_ms = max(control_step_ms, float(self._sac_cfg.episode_time_ms) - self._sac_episode_delay_ms)
        steps_per_episode = max(1, int(np.ceil(episode_control_window_ms / max(control_step_ms, 1e-9))))
        min_warmup_steps = self._sac_min_warmup_episodes * steps_per_episode
        self._sac_effective_start_steps = max(int(self._sac_cfg.start_steps), int(min_warmup_steps))

        logger.info(
            "[Agent] Episode setup: path_index=%s start_x=%.1f object_speed=%.4f delay_ms=%.1f rotation_step_limit_deg=%.3f warmup_start_steps=%d env_steps_seen=%d",
            self._sac_current_path_idx,
            self._sac_episode_start_x,
            self._sac_episode_object_speed,
            self._sac_episode_delay_ms,
            self._sac_episode_rotation_change_limit_deg,
            self._sac_effective_start_steps,
            self._sac_env_steps_seen,
        )

    def _sac_parse_state(self, state):
        try:
            cfg = self._sac_cfg
            arr = np.asarray(state, dtype=np.float32).reshape(self.n_rl_interval, self.n_ch_total)
            sensor_flat = arr[:, 5:]
            new_frames = sensor_flat.reshape(self.n_rl_interval, 3, 3, cfg.num_signal_channels)
            last = arr[-1]
            heading_angle = self._heading_from_velocity(float(last[3]), float(last[4]))
            prev_delta_heading_angle = 0.0
            if arr.shape[0] > 1:
                prev_vel = arr[:-1, 3:5]
                prev_speeds = np.hypot(prev_vel[:, 0], prev_vel[:, 1])
                valid_prev = np.flatnonzero(prev_speeds > 1e-6)
                if valid_prev.size > 0:
                    prev_heading_angle = self._heading_from_velocity(
                        float(prev_vel[valid_prev[0], 0]),
                        float(prev_vel[valid_prev[0], 1]),
                    )
                    prev_delta_heading_angle = self._wrap_angle_delta(
                        heading_angle,
                        prev_heading_angle,
                    )
            kin_vec = np.array([[last[1], last[2], last[3], last[4],
                                 heading_angle, prev_delta_heading_angle]], dtype=np.float32)
            return new_frames, kin_vec
        except Exception as ex:
            logger.error(f"[Agent] Error parsing state: {ex}")
            logger.error(f"[Agent] State shape: {np.asarray(state).shape}, expected: ({self.n_rl_interval}, {self.n_ch_total})")
            logger.error(f"[Agent] Traceback: ", exc_info=True)
            # Return dummy frames and kin_vec
            cfg = self._sac_cfg
            dummy_frames = np.zeros((self.n_rl_interval, 3, 3, cfg.num_signal_channels), dtype=np.float32)
            dummy_kin = np.zeros((1, 6), dtype=np.float32)
            return dummy_frames, dummy_kin

    def _sac_roll_history(self, new_frames):
        n = self.n_rl_interval
        self._sac_sensor_history[0] = np.roll(self._sac_sensor_history[0], -n, axis=0)
        self._sac_sensor_history[0, -n:] = new_frames

    def _sac_delta_angle_to_command(self, delta_angle_norm: float) -> float:
        delta_angle_norm = float(np.clip(delta_angle_norm, -1.0, 1.0))
        vx = float(self._sac_fixed_vx)
        if abs(vx) <= 1e-9:
            return float(self._sac_prev_vy)

        prev_theta = float(np.arctan2(self._sac_prev_vy, vx))
        max_step_rad = float(np.deg2rad(max(0.0, self._sac_episode_rotation_change_limit_deg)))
        candidate_theta = prev_theta + (delta_angle_norm * max_step_rad)
        candidate_vy = float(vx * np.tan(candidate_theta))
        limited_vy = float(np.clip(candidate_vy, -self._sac_y_speed_limit, self._sac_y_speed_limit))
        return float(np.clip(limited_vy, -self._sac_y_speed_limit, self._sac_y_speed_limit))

    def _sac_random_warmup_action(self) -> np.ndarray:
        return self._sac_explore_rng.uniform(-1.0, 1.0, size=(1, self._sac_cfg.action_dim)).astype(np.float32)

    def _sac_act(self, state):
        try:
            logger.debug("[Agent._sac_act] Starting SAC action computation")
            
            logger.debug("[Agent._sac_act] Parsing state...")
            new_frames, kin_vec = self._sac_parse_state(state)
            logger.debug(f"[Agent._sac_act] new_frames shape: {new_frames.shape}, kin_vec shape: {kin_vec.shape}")
            
            logger.debug("[Agent._sac_act] Rolling history...")
            self._sac_roll_history(new_frames)
            logger.debug("[Agent._sac_act] History rolled")
            
            logger.debug("[Agent._sac_act] Copying sensor and kin state...")
            self._sac_prev_sensor = self._sac_sensor_history.copy()
            self._sac_prev_kin = kin_vec.copy()
            logger.debug(f"[Agent._sac_act] prev_sensor shape: {self._sac_prev_sensor.shape}, prev_kin shape: {self._sac_prev_kin.shape}")

            use_random_warmup = self._sac_env_steps_seen < self._sac_effective_start_steps
            if use_random_warmup:
                sac_action = self._sac_random_warmup_action()
                logger.debug(
                    "[Agent._sac_act] Using random warmup action at env_step=%d/%d: %s",
                    self._sac_env_steps_seen,
                    self._sac_effective_start_steps,
                    sac_action,
                )
            else:
                logger.debug("[Agent._sac_act] Calling sac_agent.act()...")
                sac_action = self._sac_agent.act(self._sac_prev_sensor, self._sac_prev_kin, deterministic=False)
                logger.debug(f"[Agent._sac_act] sac_agent.act() returned, type: {type(sac_action)}, value: {sac_action}")
            
            if sac_action is None:
                logger.warning("[Agent._sac_act] SAC agent.act() returned None; using zero action")
                self._sac_prev_action = np.array([[0.0]], dtype=np.float32)
                logger.debug("[Agent._sac_act] Returning current command after zero delta fallback")
                return [self._sac_fixed_vx, self._sac_prev_vy]
            
            logger.debug(f"[Agent._sac_act] Extracting action values from shape {getattr(sac_action, 'shape', 'N/A')}")
            self._sac_prev_action = sac_action.copy()
            
            # The policy action is a normalized delta-angle command per control step.
            if hasattr(sac_action, 'shape'):
                if len(sac_action.shape) >= 2 and sac_action.shape[0] > 0 and sac_action.shape[1] > 0:
                    delta_angle_norm = float(sac_action[0, 0])
                    logger.debug(f"[Agent._sac_act] Extracted delta_angle_norm: {delta_angle_norm}")
                else:
                    logger.warning(f"[Agent._sac_act] Unexpected sac_action shape {sac_action.shape}; using zero action")
                    delta_angle_norm = 0.0
            else:
                logger.warning("[Agent._sac_act] sac_action is not a numpy array; using zero action")
                delta_angle_norm = 0.0
            
            self._sac_prev_vy = self._sac_delta_angle_to_command(delta_angle_norm)
            result = [self._sac_fixed_vx, self._sac_prev_vy]
            logger.debug(f"[Agent._sac_act] Final action: {result}")
            return result
        
        except Exception as ex:
            logger.error(f"[Agent._sac_act] Exception: {ex}", exc_info=True)
            logger.error(f"[Agent._sac_act] Returning fallback action")
            return [self._sac_fixed_vx, self._sac_prev_vy]

    def _sac_store_transition(self, next_state, reward: float, done: bool, truncated: bool):
        if self._sac_prev_sensor is None or self._sac_prev_action is None or self._sac_prev_kin is None:
            logger.debug("[Agent._sac_store_transition] Skipping store: previous state/action not initialized")
            return

        try:
            logger.debug("[Agent._sac_store_transition] Building next observation tensors")
            next_frames, next_kin = self._sac_parse_state(next_state)
            next_sensor = self._sac_sensor_history.copy()
            n = self.n_rl_interval
            next_sensor[0] = np.roll(next_sensor[0], -n, axis=0)
            next_sensor[0, -n:] = next_frames

            reward_arr = np.array([[reward]], dtype=np.float32)
            done_flag = np.array([[float(done or truncated)]], dtype=np.float32)

            logger.debug(
                "[Agent._sac_store_transition] Storing batch with sensor=%s kin=%s action=%s reward=%s next_sensor=%s next_kin=%s done=%s",
                self._sac_prev_sensor.shape,
                self._sac_prev_kin.shape,
                self._sac_prev_action.shape,
                reward_arr.shape,
                next_sensor.shape,
                next_kin.shape,
                done_flag.shape,
            )

            self._sac_replay.store_batch(
                sensor=self._sac_prev_sensor,
                kin=self._sac_prev_kin,
                action=self._sac_prev_action,
                reward=reward_arr,
                next_sensor=next_sensor,
                next_kin=next_kin,
                done=done_flag,
            )
            logger.debug(
                "[Agent._sac_store_transition] Store complete. replay_size=%d ptr=%d",
                self._sac_replay.size,
                self._sac_replay.ptr,
            )
        except Exception as ex:
            logger.error(f"[Agent._sac_store_transition] Exception: {ex}", exc_info=True)
            raise

    # ------------------------------------------------------------------
    # Server-side reward
    # ------------------------------------------------------------------

    def _sac_select_path(self, path_index=None):
        """Pick a random path for this episode (called from reset)."""
        if not self._sac_path_data_list:
            self._sac_current_path_data = None
            return
        idx = None
        try:
            if path_index is not None:
                idx_candidate = int(path_index)
                if 0 <= idx_candidate < len(self._sac_path_data_list):
                    idx = idx_candidate
        except (TypeError, ValueError):
            idx = None
        if idx is None:
            idx = int(self._sac_path_rng.integers(len(self._sac_path_data_list)))
        self._sac_current_path_data = self._sac_path_data_list[idx]
        self._sac_current_path_idx = idx
        logger.debug(f"[Agent] Selected path index {idx} for this episode")

    def compute_reward(self, state):
        """Compute reward from state using path data and the hardware_adapter reward logic.

        Returns (reward, done, info_dict).
        If no path data is loaded, returns (0.0, False, {}).
        """
        if self._sac_current_path_data is None:
            return 0.0, False, {}

        try:
            arr = np.asarray(state, dtype=np.float64).reshape(self.n_rl_interval, self.n_ch_total)
            last = arr[-1]
            t_ms = float(last[0])
            x_mm = float(last[1])
            y_mm = float(last[2])

            position_xy = np.array([x_mm, y_mm], dtype=np.float64)
            frame = local_path_frame(self._sac_current_path_data, position_xy)
            lateral = abs(float(frame['signed_lateral_error']))

            reward = 1.0 - (lateral / self._sac_reward_corridor)
            reward = float(np.clip(reward, -1.0, 1.0))

            # Object gap approximation, aligned with the front-carriage path start
            # and the pre-control delay used by the MATLAB hardware loop.
            elapsed_ms = self._sac_episode_delay_ms + t_ms
            object_x = self._sac_episode_start_x + elapsed_ms * self._sac_episode_object_speed
            object_gap = object_x - x_mm

            hit_y_min = bool(
                np.isfinite(self._sac_y_min_mm)
                and y_mm <= (self._sac_y_min_mm + self._sac_boundary_margin_mm)
            )
            hit_y_max = bool(
                np.isfinite(self._sac_y_max_mm)
                and y_mm >= (self._sac_y_max_mm - self._sac_boundary_margin_mm)
            )
            y_boundary_hit = bool(hit_y_min or hit_y_max)
            too_far = lateral > self._sac_terminate_corridor
            too_close = object_gap < self._sac_min_gap_mm
            finish_line_reached = x_mm >= float(self._sac_cfg.finish_line_mm)

            done = too_far or too_close or finish_line_reached or y_boundary_hit
            if y_boundary_hit:
                reward -= 2.0
            if too_far:
                reward -= 2.0
            if too_close:
                reward -= 2.0

            info = {
                'path_index': int(self._sac_current_path_idx),
                'target_path_x_mm': float(frame['point'][0]),
                'target_path_y_mm': float(frame['point'][1]),
                'path_progress_mm': float(frame['s']),
                'signed_lateral_error_mm': float(frame['signed_lateral_error']),
                'object_x_gap_mm': float(object_gap),
                'y_boundary_hit': y_boundary_hit,
                'y_boundary_side': 'min' if hit_y_min else ('max' if hit_y_max else ''),
                'too_far': too_far,
                'too_close': too_close,
                'finish_line_reached': bool(finish_line_reached),
            }
            return reward, done, info
        except Exception as ex:
            logger.error(f"[Agent.compute_reward] Exception: {ex}", exc_info=True)
            return 0.0, False, {}

    # ------------------------------------------------------------------
    # Checkpoint save / load
    # ------------------------------------------------------------------

    def save_checkpoint(self, episode_num, total_steps):
        """Save model weights every episode (latest) and full checkpoint+replay every N episodes."""
        if not self.use_sac_train or not self._sac_output_dir:
            return None
        try:
            output_dir = Path(self._sac_output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)
            latest_checkpoint_path = output_dir / 'latest_checkpoint.pt'
            latest_replay_path = output_dir / 'latest_replay.npz'

            checkpoint = {
                'actor': self._sac_agent.actor.state_dict(),
                'q1': self._sac_agent.q1.state_dict(),
                'q2': self._sac_agent.q2.state_dict(),
                'q1_target': self._sac_agent.q1_target.state_dict(),
                'q2_target': self._sac_agent.q2_target.state_dict(),
                'log_alpha': self._sac_agent.log_alpha.detach().cpu(),
                'actor_optim': self._sac_agent.actor_optim.state_dict(),
                'q1_optim': self._sac_agent.q1_optim.state_dict(),
                'q2_optim': self._sac_agent.q2_optim.state_dict(),
                'alpha_optim': self._sac_agent.alpha_optim.state_dict(),
                'counters': {
                    'total_env_steps': int(total_steps),
                    'episodes_completed': int(episode_num),
                },
                'cfg': {key: self._summary_log_value(value) for key, value in asdict(self._sac_cfg).items()},
                'rng': {
                    'python': random.getstate(),
                    'numpy': np.random.get_state(),
                    'torch': torch.get_rng_state(),
                    'cuda': torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
                    'path_rng': self._sac_path_rng.bit_generator.state if hasattr(self, '_sac_path_rng') else None,
                    'explore_rng': self._sac_explore_rng.bit_generator.state if hasattr(self, '_sac_explore_rng') else None,
                },
                'replay_path': str(latest_replay_path),
            }

            # Keep latest checkpoint/replay as a consistent resume pair.
            torch.save(checkpoint, latest_checkpoint_path)
            self._sac_replay.save(str(latest_replay_path))

            # Save full checkpoint + replay every N episodes
            if episode_num % self._sac_checkpoint_every == 0:
                ckpt_path = output_dir / f'checkpoint_{total_steps:07d}.pt'
                replay_path = output_dir / f'replay_{total_steps:07d}.npz'
                archive_checkpoint = dict(checkpoint)
                archive_checkpoint['replay_path'] = str(replay_path)
                torch.save(archive_checkpoint, ckpt_path)
                self._sac_replay.save(str(replay_path))
                self._prune_checkpoints(output_dir, self._sac_keep_checkpoints)
                logger.info(f"[Agent] Full checkpoint saved: {ckpt_path}")
                return str(ckpt_path)
            else:
                logger.debug(
                    f"[Agent] Latest model updated (episode {episode_num}, "
                    f"next full save at episode divisible by {self._sac_checkpoint_every})"
                )
                return None
        except Exception as ex:
            logger.error(f"[Agent.save_checkpoint] Exception: {ex}", exc_info=True)
            return None

    def _load_checkpoint(self, checkpoint_path_str):
        """Restore model + replay from a checkpoint file."""
        try:
            checkpoint_path = Path(checkpoint_path_str)
            device = self._sac_cfg.device
            checkpoint = torch.load(checkpoint_path, map_location=device,weights_only=False)

            self._sac_agent.actor.load_state_dict(checkpoint['actor'])
            self._sac_agent.q1.load_state_dict(checkpoint['q1'])
            self._sac_agent.q2.load_state_dict(checkpoint['q2'])
            self._sac_agent.q1_target.load_state_dict(checkpoint['q1_target'])
            self._sac_agent.q2_target.load_state_dict(checkpoint['q2_target'])
            self._sac_agent.log_alpha.data.copy_(checkpoint['log_alpha'].to(device))

            self._sac_agent.actor_optim.load_state_dict(checkpoint['actor_optim'])
            self._sac_agent.q1_optim.load_state_dict(checkpoint['q1_optim'])
            self._sac_agent.q2_optim.load_state_dict(checkpoint['q2_optim'])
            self._sac_agent.alpha_optim.load_state_dict(checkpoint['alpha_optim'])

            self._move_optimizer_to_device(self._sac_agent.actor_optim, device)
            self._move_optimizer_to_device(self._sac_agent.q1_optim, device)
            self._move_optimizer_to_device(self._sac_agent.q2_optim, device)
            self._move_optimizer_to_device(self._sac_agent.alpha_optim, device)

            self._sac_agent.actor.train()
            self._sac_agent.q1.train()
            self._sac_agent.q2.train()
            self._sac_agent.q1_target.train()
            self._sac_agent.q2_target.train()

            # Restore replay, tolerating old capacities and bad latest files.
            replay_path = checkpoint.get('replay_path')
            requested_replay = Path(replay_path) if replay_path else (checkpoint_path.parent / 'latest_replay.npz')
            candidates = []
            for candidate in [requested_replay, checkpoint_path.parent / 'latest_replay.npz']:
                if candidate not in candidates:
                    candidates.append(candidate)
            for candidate in sorted(checkpoint_path.parent.glob('replay_*.npz'), reverse=True):
                if candidate not in candidates:
                    candidates.append(candidate)

            replay_loaded_from = None
            replay_errors = []
            for candidate in candidates:
                if not candidate.exists():
                    continue
                try:
                    self._sac_replay.load(str(candidate))
                    replay_loaded_from = candidate
                    break
                except Exception as ex:
                    replay_errors.append(f"{candidate}: {ex}")
                    logger.warning(f"[Agent] Replay load failed from {candidate}: {ex}")

            if replay_loaded_from is None:
                logger.warning("[Agent] Resume checkpoint loaded without replay; starting from an empty replay buffer")
                if replay_errors:
                    logger.warning("[Agent] Replay load attempts: " + " | ".join(replay_errors))
            elif replay_loaded_from != requested_replay:
                logger.warning(f"[Agent] Replay restored from fallback file: {replay_loaded_from}")

            # Restore counters
            counters = checkpoint.get('counters', {})
            self._sac_total_env_steps = counters.get('total_env_steps', 0)
            self._sac_episodes_completed = counters.get('episodes_completed', 0)

            # Restore RNG state
            rng = checkpoint.get('rng')
            if rng:
                random.setstate(rng['python'])
                np.random.set_state(rng['numpy'])
                torch.set_rng_state(rng['torch'])
                if torch.cuda.is_available() and rng.get('cuda') is not None:
                    torch.cuda.set_rng_state_all(rng['cuda'])
                if hasattr(self, '_sac_path_rng') and rng.get('path_rng') is not None:
                    self._sac_path_rng.bit_generator.state = rng['path_rng']
                if hasattr(self, '_sac_explore_rng') and rng.get('explore_rng') is not None:
                    self._sac_explore_rng.bit_generator.state = rng['explore_rng']

            logger.info(
                f"[Agent] Resumed from checkpoint: {checkpoint_path} "
                f"(steps={self._sac_total_env_steps}, episodes={self._sac_episodes_completed}, "
                f"replay_size={self._sac_replay.size})"
            )
        except Exception as ex:
            logger.error(f"[Agent._load_checkpoint] Failed to load {checkpoint_path_str}: {ex}", exc_info=True)

    def _prune_checkpoints(self, output_dir, keep):
        """Keep only the N most recent checkpoints."""
        if keep <= 0:
            return
        ckpts = sorted(output_dir.glob('checkpoint_*.pt'))
        if len(ckpts) <= keep:
            return
        for ckpt in ckpts[:-keep]:
            step = ckpt.stem.split('_', 1)[1]
            replay = output_dir / f'replay_{step}.npz'
            ckpt.unlink(missing_ok=True)
            if replay.exists():
                replay.unlink()
