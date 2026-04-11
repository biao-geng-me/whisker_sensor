import numpy as np
import logging
import os
import shutil
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

    def reset(self, initial_state):
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
            logger.debug("[Agent] Clearing SAC sensor history and state...")
            self._sac_sensor_history[:] = 0.0
            self._sac_prev_vy = 0.0
            self._sac_prev_sensor = None
            self._sac_prev_kin = None
            self._sac_prev_action = None
            self._sac_select_path()
            # Extract start_x from initial_state for object gap calculation
            arr = np.asarray(initial_state, dtype=np.float64).reshape(self.n_rl_interval, self.n_ch_total)
            self._sac_episode_start_x = float(arr[-1, 1]) + self._sac_initial_gap
            logger.debug("[Agent] SAC state cleared, start_x=%.1f", self._sac_episode_start_x)
        
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

            if done > 0.5:
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
                msg += f" | actor={metrics['actor_loss']:.3f} q1={metrics['q1_loss']:.3f} alpha={metrics['alpha']:.3f}"
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
        self._sac_cfg = cfg
        self._sac_fixed_vx = float(self.config.get("fixed_vx", 0.2))
        self._sac_y_speed_limit = float(self.config.get("y_speed_limit", 0.15))
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

        # --- Path data for server-side reward ---
        self._sac_path_data_list = []
        self._sac_current_path_data = None
        self._sac_current_path_idx = 0
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
            logger.warning("[Agent] No path data received from MATLAB; server-side reward disabled")

        self._sac_reward_corridor = float(cfg.reward_corridor_half_width_mm)
        self._sac_terminate_corridor = float(cfg.terminate_corridor_half_width_mm)
        self._sac_min_gap_mm = float(cfg.min_object_x_gap_terminate_mm)
        self._sac_object_speed = float(cfg.object_tangential_speed_mm_per_ms)
        self._sac_initial_gap = float(cfg.initial_object_gap_mm)
        self._sac_episode_start_x = 0.0  # set on reset

        # --- Checkpoint / resume ---
        self._sac_output_dir = self.config.get("output_dir", "")
        self._sac_keep_checkpoints = int(self.config.get("keep_checkpoints", 5))
        self._sac_total_env_steps = 0
        self._sac_episodes_completed = 0

        # Resume from checkpoint if requested
        resume = self.config.get("resume", False)
        resume_path = self.config.get("resume_path", "")
        if resume and resume_path and os.path.isfile(resume_path):
            self._load_checkpoint(resume_path)

        self.use_sac_train = True
        logger.info(f"[Agent] SAC train mode ready. history_steps={cfg.history_steps} "
              f"sensor_shape={cfg.sensor_shape} device={cfg.device}")

    def _sac_parse_state(self, state):
        try:
            cfg = self._sac_cfg
            arr = np.asarray(state, dtype=np.float32).reshape(self.n_rl_interval, self.n_ch_total)
            sensor_flat = arr[:, 5:]
            new_frames = sensor_flat.reshape(self.n_rl_interval, 3, 3, cfg.num_signal_channels)
            last = arr[-1]
            time_norm = float(last[0]) / max(float(cfg.episode_time_ms), 1.0)
            kin_vec = np.array([[last[1], last[2], last[3], last[4],
                                 self._sac_prev_vy, time_norm]], dtype=np.float32)
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
            
            logger.debug("[Agent._sac_act] Calling sac_agent.act()...")
            sac_action = self._sac_agent.act(self._sac_prev_sensor, self._sac_prev_kin, deterministic=False)
            logger.debug(f"[Agent._sac_act] sac_agent.act() returned, type: {type(sac_action)}, value: {sac_action}")
            
            if sac_action is None:
                logger.warning("[Agent._sac_act] SAC agent.act() returned None; using zero action")
                self._sac_prev_action = np.array([[0.0]], dtype=np.float32)
                logger.debug("[Agent._sac_act] Returning [fixed_vx, 0.0]")
                return [self._sac_fixed_vx, 0.0]
            
            logger.debug(f"[Agent._sac_act] Extracting action values from shape {getattr(sac_action, 'shape', 'N/A')}")
            self._sac_prev_action = sac_action.copy()
            
            # Safely extract vy_norm with bounds checking
            if hasattr(sac_action, 'shape'):
                if len(sac_action.shape) >= 2 and sac_action.shape[0] > 0 and sac_action.shape[1] > 0:
                    vy_norm = float(sac_action[0, 0])
                    logger.debug(f"[Agent._sac_act] Extracted vy_norm: {vy_norm}")
                else:
                    logger.warning(f"[Agent._sac_act] Unexpected sac_action shape {sac_action.shape}; using zero action")
                    vy_norm = 0.0
            else:
                logger.warning("[Agent._sac_act] sac_action is not a numpy array; using zero action")
                vy_norm = 0.0
            
            self._sac_prev_vy = vy_norm * self._sac_y_speed_limit
            result = [self._sac_fixed_vx, self._sac_prev_vy]
            logger.debug(f"[Agent._sac_act] Final action: {result}")
            return result
        
        except Exception as ex:
            logger.error(f"[Agent._sac_act] Exception: {ex}", exc_info=True)
            logger.error(f"[Agent._sac_act] Returning fallback action")
            return [self._sac_fixed_vx, 0.0]

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

    def _sac_select_path(self):
        """Pick a random path for this episode (called from reset)."""
        if not self._sac_path_data_list:
            self._sac_current_path_data = None
            return
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

            # Object gap approximation
            object_x = self._sac_episode_start_x + t_ms * self._sac_object_speed
            object_gap = object_x - x_mm

            too_far = lateral > self._sac_terminate_corridor
            too_close = object_gap < self._sac_min_gap_mm

            done = too_far or too_close
            if too_far:
                reward -= 2.0
            if too_close:
                reward -= 2.0

            info = {
                'signed_lateral_error_mm': float(frame['signed_lateral_error']),
                'object_x_gap_mm': float(object_gap),
                'too_far': too_far,
                'too_close': too_close,
            }
            return reward, done, info
        except Exception as ex:
            logger.error(f"[Agent.compute_reward] Exception: {ex}", exc_info=True)
            return 0.0, False, {}

    # ------------------------------------------------------------------
    # Checkpoint save / load
    # ------------------------------------------------------------------

    def save_checkpoint(self, episode_num, total_steps):
        """Save model + replay to output_dir following train.py pattern."""
        if not self.use_sac_train or not self._sac_output_dir:
            return None
        try:
            output_dir = Path(self._sac_output_dir)
            output_dir.mkdir(parents=True, exist_ok=True)

            ckpt_path = output_dir / f'checkpoint_{total_steps:07d}.pt'
            replay_path = output_dir / f'replay_{total_steps:07d}.npz'

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
                'rng': {
                    'python': random.getstate(),
                    'numpy': np.random.get_state(),
                    'torch': torch.get_rng_state(),
                    'cuda': torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
                },
                'replay_path': str(replay_path),
            }
            torch.save(checkpoint, ckpt_path)
            self._sac_replay.save(str(replay_path))

            # Copy to latest
            shutil.copyfile(ckpt_path, output_dir / 'latest_checkpoint.pt')
            shutil.copyfile(replay_path, output_dir / 'latest_replay.npz')

            # Prune old checkpoints
            self._prune_checkpoints(output_dir, self._sac_keep_checkpoints)

            logger.info(f"[Agent] Checkpoint saved: {ckpt_path}")
            return str(ckpt_path)
        except Exception as ex:
            logger.error(f"[Agent.save_checkpoint] Exception: {ex}", exc_info=True)
            return None

    def _load_checkpoint(self, checkpoint_path_str):
        """Restore model + replay from a checkpoint file."""
        try:
            checkpoint_path = Path(checkpoint_path_str)
            device = self._sac_cfg.device
            checkpoint = torch.load(checkpoint_path, map_location=device)

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

            self._sac_agent.actor.train()
            self._sac_agent.q1.train()
            self._sac_agent.q2.train()

            # Restore replay
            replay_path = checkpoint.get('replay_path')
            rp = Path(replay_path) if replay_path else (checkpoint_path.parent / 'latest_replay.npz')
            if rp.exists():
                self._sac_replay.load(str(rp))
            elif (checkpoint_path.parent / 'latest_replay.npz').exists():
                self._sac_replay.load(str(checkpoint_path.parent / 'latest_replay.npz'))
                logger.warning(f"[Agent] Replay not found at {rp}; loaded latest_replay.npz instead")

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
