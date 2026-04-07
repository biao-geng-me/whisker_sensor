from connection_manager import ConnectionManager
from agent_wrapper import AgentWrapper
from hpc_client import HPCClient
import time
import csv
import os
import datetime
import numpy as np

# Protocol Header Bytes
CMD_START    = 0x01
CMD_STEP     = 0x02
CMD_END_SYNC = 0x03
CMD_SHUTDOWN = 0x04

def main():
    print("--- Robotics DRL Server Starting ---")
    
    # Initialize network manager and wait for MATLAB
    net = ConnectionManager()
    net.wait_for_client()

    # PHASE 1: Configuration Handshake
    print("[Main] Waiting for configuration...")
    config = net.receive_json()
    if not config:
        print("[Error] No configuration received. Exiting.")
        net.close()
        return
        
    print(f"[Main] Configuration received: Mode = {config.get('mode')}")
    
    agent = AgentWrapper(config)
    hpc = HPCClient(port=config.get("hpc_port", 5555))
    
    # If training, ensure HPC tunnel is active before telling MATLAB we're ready
    if config.get("mode") == "train":
        if not hpc.connect():
            # In a real scenario, you might want to abort here if tunnel fails
            pass 

    net.send_string("READY")
    
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
    running = True
    while running:
        net.set_timeout(10*60)  # timeout for receiving header
        header = net.receive_header()
        
        if header is None:
            print("[Main] MATLAB disconnected unexpectedly.")
            break
            
        if header == CMD_START:
            # 0x01: Start Episode
            episode_num += 1
            step_latencies = []
            recv_times = []
            agent_times = []
            send_times = []
            episode_trajectory = []
            episode_step = 0
            print(f"\n[Main] --- New Episode {episode_num} Started ---")
            initial_data = net.receive_doubles(step_msg_size)
            initial_state = initial_data[0:state_dim]
            
            # Reset agent memory and get first action
            action = agent.reset(initial_state)
            net.send_doubles(action)
            
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
            
            # Time: agent inference and trajectory recording
            agent_start = time.perf_counter()
            action = agent.step(state, reward, done, truncated, record=record_trajectory)
            agent_ms = (time.perf_counter() - agent_start) * 1000
            agent_times.append(agent_ms)

            # Record step observations to episode trajectory (state only)
            episode_step += 1
            state_arr = np.asarray(state, dtype=np.float64)
            expected_state_size = n_rl_interval * n_ch_total
            if state_arr.size != expected_state_size:
                print(
                    f"[Warn] State size mismatch at step {episode_step}: "
                    f"got {state_arr.size}, expected {expected_state_size}. Skipping trajectory row(s)."
                )
            else:
                state_mat = state_arr.reshape(n_rl_interval, n_ch_total)
                for obs_row in state_mat:
                    episode_trajectory.append(dict(zip(obs_var_names, obs_row.tolist())))

            # Time: send action back to MATLAB
            send_start = time.perf_counter()
            if action is not None:
                net.send_doubles(action)
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
                print(f"[Step {len(step_latencies):5d}] Total: {avg_total:.3f}ms | Recv: {avg_recv:.3f}ms | Agent: {avg_agent:.3f}ms | Send: {avg_send:.3f}ms")
                
        elif header == CMD_END_SYNC:
            # 0x03: End Episode & Sync with HPC — save trajectory CSV
            print("[Main] Episode ended. Saving trajectory and syncing...")

            if episode_trajectory:
                ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                csv_path = os.path.join(traj_dir, f'traj_{ts}_ep{episode_num:04d}.csv')
                fieldnames = obs_var_names
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(episode_trajectory)
                print(f"[Main] Trajectory saved: {csv_path} ({len(episode_trajectory)} rows)")
            
            # Print detailed latency statistics for this episode
            if step_latencies:
                import statistics
                print(f"\n[Latency Stats] Episode {episode_num} ({len(step_latencies)} steps):")
                print(f"  Total | Mean: {statistics.mean(step_latencies):.3f}ms | Max: {max(step_latencies):.3f}ms | Min: {min(step_latencies):.3f}ms | Std: {statistics.stdev(step_latencies) if len(step_latencies) > 1 else 0:.3f}ms")
                print(f"  Recv  | Mean: {statistics.mean(recv_times):.3f}ms | Max: {max(recv_times):.3f}ms | Min: {min(recv_times):.3f}ms")
                print(f"  Agent | Mean: {statistics.mean(agent_times):.3f}ms | Max: {max(agent_times):.3f}ms | Min: {min(agent_times):.3f}ms")
                print(f"  Send  | Mean: {statistics.mean(send_times):.3f}ms | Max: {max(send_times):.3f}ms | Min: {min(send_times):.3f}ms")
            
            if config.get("mode") == "train":
                traj = agent.get_trajectory()
                new_weights = hpc.sync_trajectory(traj)
                if new_weights:
                    agent.update_weights(new_weights)
            
            # Tell MATLAB it is safe to reset physical hardware
            net.send_string("SYNC_COMPLETE")
            
        elif header == CMD_SHUTDOWN:
            # 0x04: Teardown
            print("[Main] Shutdown command received from hardware.")
            running = False
            
        else:
            print(f"[Error] Unknown header received: {header}")

    # PHASE 3: Teardown
    print("--- Shutting Down ---")
    hpc.close()
    net.close()

def make_obs_var_names(num_whiskers: int) -> list[str]:
    return ['t', 'x', 'y', 'x_vel', 'y_vel'] + [
        f'M{component}{index}'
        for index in range(1, num_whiskers + 1)
        for component in ('L', 'D') # whisker simulators send lift moment and drag moment in order
    ]

if __name__ == "__main__":
    main()