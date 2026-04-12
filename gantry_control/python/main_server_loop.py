from connection_manager import ConnectionManager
from agent_wrapper import AgentWrapper
from hpc_client import HPCClient
from pathlib import Path
import time
import csv
import os
import datetime
import numpy as np
import logging
import sys

# Set interactive backend before any matplotlib import so live plots work.
# TkAgg is the standard interactive backend on Windows; falls back gracefully.
_matplotlib_ok = False
try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as _plt
    _matplotlib_ok = True
except Exception:
    _plt = None

_analyze_ok = False
try:
    from agents.rl_sac_v4_pathblind_hardware.analyze_run import (
        make_training_curves, make_loss_curves_log, make_episode_curves,
    )
    _analyze_ok = True
except Exception:
    pass


_TERM_COLORS = {
    'too_far': 'tab:red',
    'too_close': 'tab:orange',
    'time_limit': 'tab:blue',
    'done': 'tab:olive',
}


class LivePlotter:
    """Persistent interactive matplotlib window updated after each episode."""

    def __init__(self):
        self._ok = False
        if not _matplotlib_ok:
            return
        try:
            _plt.ion()
            self.fig, (self.ax_ret, self.ax_lat) = _plt.subplots(2, 1, figsize=(10, 7))
            self.fig.suptitle('Training Progress (live)')
            self._setup_axes()
            _plt.tight_layout()
            _plt.show(block=False)
            _plt.pause(0.1)
            self._ok = True
        except Exception as ex:
            logging.getLogger('server').warning(f'[LivePlotter] Init failed: {ex}')

    def _setup_axes(self):
        self.ax_ret.set_title('Episode Return')
        self.ax_ret.set_xlabel('Episode')
        self.ax_ret.set_ylabel('Return')
        self.ax_ret.grid(True, alpha=0.3)

        self.ax_lat.set_title('Episode-End Signed Lateral Error')
        self.ax_lat.set_xlabel('Episode')
        self.ax_lat.set_ylabel('Error (mm)')
        for level, color in [( 180, 'tab:orange'), (-180, 'tab:orange'),
                              ( 300, 'tab:red'),    (-300, 'tab:red')]:
            self.ax_lat.axhline(level, ls='--', color=color, alpha=0.5)
        self.ax_lat.grid(True, alpha=0.3)

    def update(self, episode_log_rows: list):
        if not self._ok or not episode_log_rows:
            return
        try:
            ep_idx   = list(range(1, len(episode_log_rows) + 1))
            returns  = [float(r['episode_return'])          for r in episode_log_rows]
            laterals = [float(r['signed_lateral_error_mm']) for r in episode_log_rows]
            reasons  = [r['termination_reason']             for r in episode_log_rows]
            colors   = [_TERM_COLORS.get(r, 'tab:gray')     for r in reasons]

            for ax, vals in [(self.ax_ret, returns), (self.ax_lat, laterals)]:
                ax.cla()
            self._setup_axes()

            self.ax_ret.plot(ep_idx, returns,  color='0.6', alpha=0.5, lw=1)
            self.ax_lat.plot(ep_idx, laterals, color='0.6', alpha=0.5, lw=1)
            self.ax_ret.scatter(ep_idx, returns,  c=colors, s=35, zorder=3)
            self.ax_lat.scatter(ep_idx, laterals, c=colors, s=35, zorder=3)

            self.fig.suptitle(f'Training Progress (live) — episode {len(episode_log_rows)}')
            _plt.tight_layout()
            self.fig.canvas.draw_idle()
            _plt.pause(0.05)
        except Exception as ex:
            logging.getLogger('server').debug(f'[LivePlotter] Update error: {ex}')

# Protocol Header Bytes
CMD_START    = 0x01
CMD_STEP     = 0x02
CMD_END_SYNC = 0x03
CMD_SHUTDOWN = 0x04

def setup_logging():
    """Configure logging to file and console."""
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(log_dir, f'server_{timestamp}.log')
    
    # Create logger
    logger = logging.getLogger('server')
    logger.setLevel(logging.DEBUG)
    
    # File handler (DEBUG level - capture everything)
    fh = logging.FileHandler(log_file)
    fh.setLevel(logging.DEBUG)
    
    # Console handler (INFO level - less verbose on console)
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    
    # Formatter with timestamp
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', 
                                  datefmt='%Y-%m-%d %H:%M:%S')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    
    logger.addHandler(fh)
    logger.addHandler(ch)
    
    logger.info(f"Logging initialized. Log file: {log_file}")
    return logger

def main():
    logger = setup_logging()
    logger.info("=== Robotics DRL Server Starting ===")
    
    # Initialize network manager and wait for MATLAB
    net = ConnectionManager()
    logger.info("Waiting for MATLAB client connection...")
    net.wait_for_client()
    logger.info("MATLAB client connected!")

    # PHASE 1: Configuration Handshake
    logger.info("[PHASE 1] Waiting for configuration...")
    config = net.receive_json()
    if not config:
        logger.error("No configuration received. Exiting.")
        net.close()
        return
        
    logger.info(f"[PHASE 1] Configuration received: Mode = {config.get('mode')}")
    logger.debug(f"[PHASE 1] Full config: {config}")
    
    agent = AgentWrapper(config)

    # Live training plot (train mode only, requires interactive display)
    live_plotter = None
    if config.get('mode') == 'train' and _matplotlib_ok:
        live_plotter = LivePlotter()
        if not live_plotter._ok:
            live_plotter = None
            logger.warning('[LivePlotter] Could not open interactive window; live plots disabled.')
        else:
            logger.info('[LivePlotter] Live training plot window opened.')

    net.send_string("READY")
    logger.info("[PHASE 1] Sent READY to MATLAB")
    
    # Variables derived from config
    state_dim = config.get("state_dim")
    action_dim = config.get("action_dim")
    n_rl_interval = config.get("n_rl_interval")
    n_ch_total = config.get("n_ch_total")
    num_whiskers = config.get("num_whiskers", (n_ch_total - 5)//2)  # Assuming first 5 are t, x, y, x_vel, y_vel
    obs_var_names = make_obs_var_names(num_whiskers)
    # Total received per step = states + 1 reward + 1 done flag + 1 truncated flag
    step_msg_size = state_dim + 3 
    record_trajectory = config.get("record_trajectory", False)  # Optional trajectory recording

    # Trajectory output directory (episode_trajectories/ relative to this script)
    traj_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'episode_trajectories')
    os.makedirs(traj_dir, exist_ok=True)

    # Per-episode trajectory buffer: list of observation rows using obs_var_names
    episode_trajectory = []
    episode_step = 0
    
    # Latency tracking
    step_latencies = []  # Track latency for each step in current episode
    recv_times = []
    agent_times = []
    send_times = []
    episode_num = 0

    # Convergence logging (train mode): in-memory rows + CSV output
    output_dir_str = config.get('output_dir', '')
    ckpt_output_dir = Path(output_dir_str) if output_dir_str else None
    train_log_rows: list[dict] = []
    episode_log_rows: list[dict] = []
    # Pre-load existing logs if resuming
    if ckpt_output_dir and ckpt_output_dir.is_dir():
        for fname, target in [('train_log.csv', train_log_rows), ('episode_log.csv', episode_log_rows)]:
            p = ckpt_output_dir / fname
            if p.exists():
                with p.open(newline='') as f:
                    target.extend(csv.DictReader(f))
        if train_log_rows:
            logger.info(f"[Convergence] Loaded {len(train_log_rows)} existing rows from {ckpt_output_dir}")

    # Per-episode reward / lateral tracking (reset in CMD_START)
    episode_rewards: list[float] = []
    episode_signed_laterals: list[float] = []
    last_signed_lateral = 0.0
    last_reward_info: dict = {}
    last_done = 0.0
    last_truncated_flag = False

    # PHASE 2 & 3: The State Machine Loop
    logger.info("[PHASE 2] Starting control loop...")
    running = True
    while running:
        net.set_timeout(10*60)  # timeout for receiving header
        header = net.receive_header()
        
        if header is None:
            logger.error("[PHASE 2] MATLAB disconnected unexpectedly.")
            break
            
        logger.debug(f"[PHASE 2] Received header: 0x{header:02x}")
            
        if header == CMD_START:
            # 0x01: Start Episode
            episode_num += 1
            step_latencies = []
            recv_times = []
            agent_times = []
            send_times = []
            episode_trajectory = []
            episode_step = 0
            episode_rewards = []
            episode_signed_laterals = []
            last_signed_lateral = 0.0
            last_reward_info = {}
            last_done = 0.0
            last_truncated_flag = False
            logger.info(f"\n[Episode {episode_num}] STARTED")
            
            logger.debug(f"[Episode {episode_num}] Waiting for initial state data (expecting {step_msg_size} doubles)...")
            initial_data = net.receive_doubles(step_msg_size)
            logger.debug(f"[Episode {episode_num}] Received initial data, shape: {len(initial_data)}")
            
            initial_state = initial_data[0:state_dim]
            logger.debug(f"[Episode {episode_num}] Extracted initial state, shape: {len(initial_state)}")
            logger.debug(f"[Episode {episode_num}] First 5 state values: {initial_state[:5] if len(initial_state) >= 5 else initial_state}")
            
            # Reset agent memory and get first action
            try:
                logger.debug(f"[Episode {episode_num}] Calling agent.reset()...")
                action = agent.reset(initial_state)
                logger.debug(f"[Episode {episode_num}] agent.reset() returned, action type: {type(action)}")
                
                if action is None:
                    logger.warning("[Episode %d] agent.reset() returned None; sending zero action", episode_num)
                    action = [0.0] * action_dim
                    logger.debug(f"[Episode {episode_num}] Set action to zero: {action}")
                
                logger.debug(f"[Episode {episode_num}] Action to send: {action} (length: {len(action)})")
                logger.debug(f"[Episode {episode_num}] Calling net.send_doubles() with {len(action)} values...")
                net.send_doubles(action)
                logger.debug(f"[Episode {episode_num}] net.send_doubles() completed successfully")
                logger.info(f"[Episode {episode_num}] First action sent: {action}")
                
            except Exception as e:
                logger.error(f"[Episode {episode_num}] Error during agent.reset(): {e}", exc_info=True)
                logger.error(f"[Episode {episode_num}] Attempting to send zero action as fallback...")
                try:
                    net.send_doubles([0.0] * action_dim)
                    logger.error(f"[Episode {episode_num}] Fallback action sent")
                except Exception as e2:
                    logger.error(f"[Episode {episode_num}] Fallback action send FAILED: {e2}", exc_info=True)
            
        elif header == CMD_STEP:
            net.set_timeout(1)
            # 0x02: Fast Control Loop Step
            step_start = time.perf_counter()
            
            # Time: receive data from MATLAB
            recv_start = time.perf_counter()
            step_data = net.receive_doubles(step_msg_size)
            recv_ms = (time.perf_counter() - recv_start) * 1000
            recv_times.append(recv_ms)
            
            state = step_data[0:state_dim]
            matlab_reward = step_data[state_dim]
            done = step_data[state_dim + 1]
            truncated = step_data[state_dim + 2]

            # Server-side reward replaces MATLAB reward when path data is available.
            # Done/truncated signals still come from MATLAB to keep the protocol in sync.
            if agent.use_sac_train and agent._sac_current_path_data is not None:
                reward, _, reward_info = agent.compute_reward(state)
                last_signed_lateral = float(reward_info.get('signed_lateral_error_mm', 0.0))
                episode_signed_laterals.append(last_signed_lateral)
                last_reward_info = reward_info
            else:
                reward = matlab_reward
                reward_info = {}

            episode_rewards.append(reward)
            last_done = done
            last_truncated_flag = bool(truncated >= 0.5)

            episode_done = (done >= 0.5) or bool(truncated)
            
            logger.debug(f"[Episode {episode_num} Step {episode_step:4d}] Received: reward={reward:.4f}, done={done:.1f}, truncated={truncated:.1f}")
            
            # Time: agent inference and trajectory recording
            agent_start = time.perf_counter()
            try:
                logger.debug(f"[Episode {episode_num} Step {episode_step:4d}] Calling agent.step()...")
                action = agent.step(state, reward, done, truncated, record=record_trajectory)
                logger.debug(f"[Episode {episode_num} Step {episode_step:4d}] agent.step() returned: {action}")
            except Exception as ex:
                logger.error(
                    f"[Episode {episode_num} Step {episode_step:4d}] agent.step() failed: {ex}",
                    exc_info=True,
                )
                action = [0.0] * action_dim if not episode_done else None
            agent_ms = (time.perf_counter() - agent_start) * 1000
            agent_times.append(agent_ms)

            # Record step observations to episode trajectory (state only)
            episode_step += 1
            state_arr = np.asarray(state, dtype=np.float64)
            expected_state_size = n_rl_interval * n_ch_total
            if state_arr.size != expected_state_size:
                logger.warning(
                    f"[Episode {episode_num} Step {episode_step}] "
                    f"State size mismatch: got {state_arr.size}, expected {expected_state_size}. Skipping trajectory row(s)."
                )
            else:
                state_mat = state_arr.reshape(n_rl_interval, n_ch_total)
                for obs_row in state_mat:
                    episode_trajectory.append(dict(zip(obs_var_names, obs_row.tolist())))

            # Time: send action back to MATLAB
            send_start = time.perf_counter()
            if (not episode_done) and action is not None:
                try:
                    net.send_doubles(action)
                    logger.debug(f"[Episode {episode_num} Step {episode_step}] Sent action: {action}")
                except Exception as ex:
                    logger.error(
                        f"[Episode {episode_num} Step {episode_step}] Failed to send action: {ex}",
                        exc_info=True,
                    )
                    raise
            send_ms = (time.perf_counter() - send_start) * 1000
            send_times.append(send_ms)
            
            step_latency = (time.perf_counter() - step_start) * 1000  # ms
            step_latencies.append(step_latency)
            
            # Debug output every 100 steps
            if len(step_latencies) % 100 == 0:
                avg_total = sum(step_latencies[-100:]) / 100
                avg_recv = sum(recv_times[-100:]) / 100
                avg_agent = sum(agent_times[-100:]) / 100
                avg_send = sum(send_times[-100:]) / 100
                logger.info(f"[Episode {episode_num} Step {len(step_latencies):5d}] Total: {avg_total:.3f}ms | Recv: {avg_recv:.3f}ms | Agent: {avg_agent:.3f}ms | Send: {avg_send:.3f}ms")
                
        elif header == CMD_END_SYNC:
            # 0x03: End Episode & Sync with HPC — save trajectory CSV
            logger.info(f"[Episode {episode_num}] Episode ended. Saving trajectory and running update...")

            if episode_trajectory:
                ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                csv_path = os.path.join(traj_dir, f'traj_{ts}_ep{episode_num:04d}.csv')
                fieldnames = obs_var_names
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(episode_trajectory)
                logger.info(f"[Episode {episode_num}] Trajectory saved: {csv_path} ({len(episode_trajectory)} rows)")
            
            # Print detailed latency statistics for this episode
            if step_latencies:
                import statistics
                logger.info(f"\n[Episode {episode_num} Latency Stats] ({len(step_latencies)} steps):")
                logger.info(f"  Total | Mean: {statistics.mean(step_latencies):.3f}ms | Max: {max(step_latencies):.3f}ms | Min: {min(step_latencies):.3f}ms | Std: {statistics.stdev(step_latencies) if len(step_latencies) > 1 else 0:.3f}ms")
                logger.info(f"  Recv  | Mean: {statistics.mean(recv_times):.3f}ms | Max: {max(recv_times):.3f}ms | Min: {min(recv_times):.3f}ms")
                logger.info(f"  Agent | Mean: {statistics.mean(agent_times):.3f}ms | Max: {max(agent_times):.3f}ms | Min: {min(agent_times):.3f}ms")
                logger.info(f"  Send  | Mean: {statistics.mean(send_times):.3f}ms | Max: {max(send_times):.3f}ms | Min: {min(send_times):.3f}ms")
            
            # Run local SAC update (train mode) or no-op (infer mode).
            needs_reset = agent.between_episode_update(episode_step)

            # Save checkpoint after each episode (train mode only)
            if agent.use_sac_train:
                agent._sac_total_env_steps += episode_step
                agent._sac_episodes_completed = episode_num
                ckpt = agent.save_checkpoint(episode_num, agent._sac_total_env_steps)
                if ckpt:
                    logger.info(f"[Episode {episode_num}] Checkpoint saved at step {agent._sac_total_env_steps}")

            # --- Convergence logging (train mode only) ---
            if ckpt_output_dir and agent.use_sac_train:
                ep_return = sum(episode_rewards)
                mean_rwd = ep_return / len(episode_rewards) if episode_rewards else 0.0
                mean_lat = (
                    sum(episode_signed_laterals) / len(episode_signed_laterals)
                    if episode_signed_laterals else 0.0
                )
                # Termination reason
                if last_reward_info.get('too_far'):
                    term_reason = 'too_far'
                elif last_reward_info.get('too_close'):
                    term_reason = 'too_close'
                elif last_truncated_flag:
                    term_reason = 'time_limit'
                else:
                    term_reason = 'done'

                sac_m = getattr(agent, '_sac_last_update_metrics', {})
                train_row = {
                    'total_env_steps': str(agent._sac_total_env_steps),
                    'mean_reward_this_episode': f'{mean_rwd:.6f}',
                    'mean_lateral_error_mm': f'{mean_lat:.4f}',
                    'actor_loss': str(sac_m.get('actor_loss', '')),
                    'q1_loss': str(sac_m.get('q1_loss', '')),
                    'q2_loss': str(sac_m.get('q2_loss', '')),
                    'alpha': str(sac_m.get('alpha', '')),
                    'episode_return': f'{ep_return:.6f}',
                }
                ep_row = {
                    'episode_return': f'{ep_return:.6f}',
                    'signed_lateral_error_mm': f'{last_signed_lateral:.4f}',
                    'termination_reason': term_reason,
                }
                train_log_rows.append(train_row)
                episode_log_rows.append(ep_row)

                try:
                    ckpt_output_dir.mkdir(parents=True, exist_ok=True)
                    _write_log_csv(ckpt_output_dir / 'train_log.csv', train_log_rows,
                                   list(train_row.keys()))
                    _write_log_csv(ckpt_output_dir / 'episode_log.csv', episode_log_rows,
                                   list(ep_row.keys()))
                    if _analyze_ok and len(train_log_rows) >= 1:
                        make_training_curves(train_log_rows, ckpt_output_dir)
                        make_loss_curves_log(train_log_rows, ckpt_output_dir)
                    if _analyze_ok and len(episode_log_rows) >= 2:
                        make_episode_curves(episode_log_rows, ckpt_output_dir)
                    logger.debug(f"[Episode {episode_num}] Convergence logs updated")
                except Exception as log_ex:
                    logger.warning(f"[Episode {episode_num}] Convergence logging failed: {log_ex}")

                # Update live plot window
                if live_plotter is not None:
                    live_plotter.update(episode_log_rows)

            if needs_reset:
                logger.info(f"[Episode {episode_num}] Agent requested reset")
                net.send_string("RESET_NEEDED")
            else:
                net.send_string("SYNC_COMPLETE")
            logger.info(f"[Episode {episode_num}] COMPLETE")
            
        elif header == CMD_SHUTDOWN:
            # 0x04: Teardown
            logger.info("Shutdown command received from MATLAB.")
            # Save final checkpoint before exiting
            if agent.use_sac_train:
                ckpt = agent.save_checkpoint(episode_num, agent._sac_total_env_steps)
                if ckpt:
                    logger.info(f"Final checkpoint saved at step {agent._sac_total_env_steps}")
            running = False
            
        else:
            logger.error(f"Unknown header received: 0x{header:02x}")

    # PHASE 3: Teardown
    logger.info("=== Shutting Down ===")
    net.close()
    logger.info("Server shut down complete.")

def _write_log_csv(path: Path, rows: list, fieldnames: list):
    """Rewrite a log CSV from in-memory rows."""
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def make_obs_var_names(num_whiskers: int) -> list[str]:
    return ['t', 'x', 'y', 'x_vel', 'y_vel'] + [
        f'M{component}{index}'
        for index in range(1, num_whiskers + 1)
        for component in ('L', 'D') # whisker simulators send lift moment and drag moment in order
    ]

if __name__ == "__main__":
    main()