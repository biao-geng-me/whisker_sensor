% --- Main Robotics Control Loop Example ---
clear all; close all; clc;

% 1. Setup Configuration
HOST = '127.0.0.1';
PORT = 65432;
n_rl_interval = 4;
n_ch_total = 5+18; % t,x,y,u,v + 18 sensor channels
STATE_DIM = n_rl_interval*n_ch_total;
ACTION_DIM = 2;
MAX_EPISODES = 3;
MAX_STEPS = 1000;

config.mode = 'infer';
config.hpc_port = 5555;
config.state_dim = STATE_DIM;
config.action_dim = ACTION_DIM;
config.max_episodes = MAX_EPISODES;

% 2. Initialize Network connection to Python
try
    net = NetworkClient(HOST, PORT, STATE_DIM, ACTION_DIM);
catch
    return; % Exit if Python server isn't running
end

% Phase 1: Handshake
net.sendConfig(config);

% Phase 2: The Episode Loop
for ep = 1:MAX_EPISODES
    fprintf('\n--- Starting Episode %d ---\n', ep);
    latencies = []; % Track round-trip latency for this episode
    
    % Hardware specific: Move robot to starting position
    % hardware.resetToHome();
    currentState = rand(n_ch_total,n_rl_interval); % Dummy hardware read
    currentState = currentState(:)';
    % Get first action from Python
    action = net.startEpisode(currentState);
    
    for step = 1:MAX_STEPS
        
        % --- HARDWARE EXECUTION ---
        % hardware.applyTorques(action);
        % pause(0.05); % 20 Hz physical loop timing
        % currentState = hardware.readSensors();
        % --------------------------
        
        % Dummy Physics Update
        currentState = currentState + norm(action) * 0.1; 
        reward = rand(); 
        
        % Check terminal condition (e.g., robot fell over or reached goal)
        done = 0.0;

        % Check if episode time up
        truncated = 0.0;
        if step == MAX_STEPS
            done = 1.0;
        end
        
        % Query Python for the next action (with latency measurement)
        tic;
        action = net.stepRL(currentState, reward, done, truncated);
        latency_ms = toc * 1000; % Convert to milliseconds
        latencies = [latencies, latency_ms];
        
        if done > 0.5
            fprintf('Episode finished at step %d\n', step);
            break;
        end
    end
    
    % Print latency statistics for this episode
    fprintf('\n[Latency Stats] Episode %d:\n', ep);
    fprintf('  Mean: %.2f ms\n', mean(latencies));
    fprintf('  Max:  %.2f ms\n', max(latencies));
    fprintf('  Min:  %.2f ms\n', min(latencies));
    fprintf('  Std:  %.2f ms\n', std(latencies));
    
    % Phase 2 (State C): Hardware parks, wait for HPC to train
    % hardware.applyBrakes();
    if strcmp(config.mode,'train')
        net.syncWithHPC();
    end
    
end

% Phase 3: Teardown
net.shutdown();
disp('Experiment Complete.');