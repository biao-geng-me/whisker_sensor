% --- Configuration ---
host = '127.0.0.1';
port = 65432;

% Import Java networking and IO classes
import java.net.Socket
import java.io.*

disp('Connecting using raw Java Sockets...');

try
    % 1. Create socket and KILL THE DELAY
    socket = Socket(host, port);
    socket.setTcpNoDelay(true); % <--- The Magic Bullet
    
    % Get data streams
    outStream = DataOutputStream(socket.getOutputStream());
    inStream  = DataInputStream(socket.getInputStream());
catch ME
    disp('Failed to connect.');
    rethrow(ME);
end

disp('Connected! Firing 1000 rapid steps...');

% Dummy variables
num_steps = 1000;
current_state = [1.0, 2.0, 3.0, 4.0];
current_reward = 0.0;
done = 0.0;

% 2. Send Initial State
initial_msg = [current_state, current_reward, done];
for i = 1:length(initial_msg)
    outStream.writeDouble(initial_msg(i));
end
outStream.flush(); % Push immediately

% 3. Main Loop
for step = 1:num_steps
    
    % A. Read Action (2 doubles)
    action = [inStream.readDouble(), inStream.readDouble()];
    
    % B. Dummy update (NO PAUSE)
    current_state = current_state + (norm(action) * 0.1) .* [1, 1, 1, 1];
    if step == num_steps
        done = 1.0;
    end
    
    % C. Send New State (6 doubles)
    msg = [current_state, current_reward, done];
    for i = 1:length(msg)
        outStream.writeDouble(msg(i));
    end
    outStream.flush(); % Push immediately
    
end

disp('Loop completed. Closing socket.');
socket.close();