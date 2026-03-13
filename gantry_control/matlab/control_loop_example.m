% --- Main Robotics Control Loop Example ---
clear all; close all; clc;

% 1. Setup Configuration
HOST = '127.0.0.1';
PORT = 65432;
n_rl_interval = 4;
n_ch_total = 5+18; % t,x,y,u,v + 18 sensor channels
STATE_DIM = n_rl_interval*n_ch_total;
ACTION_DIM = 2;
MAX_EPISODES = 1;
MAX_STEPS = 1000;
SAMPLE_RATE = 80;

config.mode = 'infer';
config.hpc_port = 5555;
config.n_rl_interval = n_rl_interval;
config.n_ch_total = n_ch_total;
config.state_dim = STATE_DIM;
config.action_dim = ACTION_DIM;
config.max_episodes = MAX_EPISODES;
config.sample_rate = SAMPLE_RATE;
config.dt = 1/SAMPLE_RATE;

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
    currentState = zeros(n_ch_total,n_rl_interval); % Dummy hardware read
    % currentState = currentState(:)';
    % Get first action from Python
    action = net.startEpisode(currentState(:)');
    traj_hist = zeros(5,MAX_STEPS);
    
    for step = 1:MAX_STEPS
        
        % --- HARDWARE EXECUTION ---
        % hardware.applyTorques(action);
        % pause(0.05); % 20 Hz physical loop timing
        % currentState = hardware.readSensors();
        % --------------------------
        
        % Dummy Physics Update
        currentState = circshift(currentState, [0 -1]);
        currentState(1,end) = step*config.dt*1000;
        currentState(2:3,end) = action(1:2)' * config.dt * 1000 + currentState(2:3,end-1);
        currentState(4:5,end) = action(1:2)';
        currentState(6:end,end) = rand(config.n_ch_total-5,1);
        traj_hist(:, step) = currentState(1:5, end); % Store trajectory history
        
        reward = rand(); 
        
        % Check terminal condition (e.g., robot fell over or reached goal)
        done = 0.0;

        % Check if episode time up
        truncated = 0.0;
        if step == MAX_STEPS
            done = 1.0;
        end
        
        % Query Python for the next action (with latency measurement)
        if mod(step,config.n_rl_interval)==0
            tic;
            action = net.stepRL(currentState(:)', reward, done, truncated);
            latency_ms = toc * 1000; % Convert to milliseconds
            latencies = [latencies, latency_ms];
        end
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
    
    
    t=traj_hist(1,:);
    xt = traj_hist(2,:);
    yt = traj_hist(3,:);
    ut = traj_hist(4,:);
    vt = traj_hist(5,:);
    figure; plot(xt,yt);
    figure; plot(t,ut); hold on; plot(t,vt);
end

% Phase 3: Teardown
net.shutdown();
disp('Experiment Complete.');