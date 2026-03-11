% --- Configuration ---
host = '127.0.0.1';
port = 65432;

disp(['Connecting to Python server at ', host, ':', num2str(port), '...']);

% 1. Create TCP client instance
% tcpclient handles little-endian 'double' translation automatically natively
try
    t = tcpclient(host, port, 'Timeout', 10);
catch ME
    disp('Error connecting to Python. Make sure the Python server is running first.');
    rethrow(ME);
end

disp('Connected! Starting control loop...');

% --- Dummy Hardware / Simulation Variables ---
num_steps = 1000;
current_state = [1.0, 2.0, 3.0, 4.0]; % 4D State
current_reward = 0.0;
done = 0.0;

% 2. Send Initial State to kick off the loop
% Format: [State(1:4), Reward, Done]
initial_msg = [current_state, current_reward, done];
write(t, initial_msg, 'double');

% 3. Main Control Loop
for step = 1:num_steps
    
    % A. Read Action from Python (Expect 2 doubles)
    % This will block until data is received
    action = read(t, 2, 'double');
    
    % B. Step the "Environment" (Hardware/Sim)
    % -> Apply motor torques (action)
    % -> Read sensors (current_state)
    
    % Simulating hardware delay (20Hz = 0.05 seconds)
    % In reality, your hardware reading will act as the physical delay.
    % pause(0.0001);
    
    % Dummy state update
    current_state = current_state + (norm(action) * 0.1) .* [1, 1, 1, 1]; % Just making numbers change
    current_reward = current_reward + 1.0;
    
    % Check termination condition
    if step == num_steps
        done = 1.0;
    end
    
    % C. Send New State back to Python
    msg = [current_state, current_reward, done];
    write(t, msg, 'double');
    
end

disp('Loop completed. Closing connection.');
clear t; % Close the socket