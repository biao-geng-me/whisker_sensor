from connection_manager import ConnectionManager
from agent_wrapper import AgentWrapper
from hpc_client import HPCClient
import time
import csv
import os
import datetime
import numpy as np
import logging
import sys

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
            reward = step_data[state_dim]
            done = step_data[state_dim + 1]
            truncated = step_data[state_dim + 2]
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
            if needs_reset:
                logger.info(f"[Episode {episode_num}] Agent requested reset")
                net.send_string("RESET_NEEDED")
            else:
                net.send_string("SYNC_COMPLETE")
            logger.info(f"[Episode {episode_num}] COMPLETE")
            
        elif header == CMD_SHUTDOWN:
            # 0x04: Teardown
            logger.info("Shutdown command received from MATLAB.")
            running = False
            
        else:
            logger.error(f"Unknown header received: 0x{header:02x}")

    # PHASE 3: Teardown
    logger.info("=== Shutting Down ===")
    net.close()
    logger.info("Server shut down complete.")

def make_obs_var_names(num_whiskers: int) -> list[str]:
    return ['t', 'x', 'y', 'x_vel', 'y_vel'] + [
        f'M{component}{index}'
        for index in range(1, num_whiskers + 1)
        for component in ('L', 'D') # whisker simulators send lift moment and drag moment in order
    ]

if __name__ == "__main__":
    main()