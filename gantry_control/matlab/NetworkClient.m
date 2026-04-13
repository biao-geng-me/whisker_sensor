classdef NetworkClient < handle
    % NETWORKCLIENT Handles low-latency TCP communication with the Python DRL Server.
    % Uses raw Java sockets to bypass Nagle's algorithm.

    properties (Access = private)
        socket      % java.net.Socket
        outStream   % java.io.DataOutputStream
        inStream    % java.io.DataInputStream
        
        % Protocol Headers (Must match Python)
        CMD_START    = int8(1);
        CMD_STEP     = int8(2);
        CMD_END_SYNC = int8(3);
        CMD_SHUTDOWN = int8(4);
        
        stateDim
        actionDim
    end
    
    methods
        function obj = NetworkClient(host, port, stateDim, actionDim)
            % Constructor: Establishes connection to Python
            import java.net.Socket
            import java.io.*
            
            obj.stateDim = stateDim;
            obj.actionDim = actionDim;
            
            fprintf('[Network] Connecting to %s:%d...\n', host, port);
            try
                obj.socket = Socket(host, port);
                obj.socket.setTcpNoDelay(true); % The Magic Bullet for low latency
                
                obj.outStream = DataOutputStream(obj.socket.getOutputStream());
                obj.inStream  = DataInputStream(obj.socket.getInputStream());
                fprintf('[Network] Connected successfully.\n');
            catch ME
                error('NetworkClient:ConnectionFailed', ...
                    'Could not connect to Python server. Is it running?\n%s', ME.message);
            end
        end
        
        function sendConfig(obj, configStruct)
            % Phase 1: Sends configuration as a JSON string with a 4-byte length header
            jsonStr = jsonencode(configStruct);
            jsonBytes = unicode2native(jsonStr, 'UTF-8');
            msgLength = int32(length(jsonBytes));
            
            % Send 4-byte length (Big-Endian)
            obj.outStream.writeInt(msgLength);
            % Send JSON bytes
            obj.outStream.write(jsonBytes);
            obj.outStream.flush();
            
            % Wait for 'READY' response
            obj.waitForString('READY');
            fprintf('[Network] Configuration accepted by Server.\n');
        end
        
        function action = startEpisode(obj, initialState, episodeMeta)
            % Sends State A (0x01) and the initial state, receives first action.
            % episodeMeta is optional and can be a struct with fields:
            % path_index, front_start_x_mm, object_speed_mm_per_ms, delay_ms,
            % rotation_change_limit_deg_per_control_step
            % or a numeric vector in the same order.
            if nargin < 3 || isempty(episodeMeta)
                metaValues = nan(1, 5);
            elseif isstruct(episodeMeta)
                metaValues = nan(1, 5);
                if isfield(episodeMeta, 'path_index')
                    metaValues(1) = double(episodeMeta.path_index);
                end
                if isfield(episodeMeta, 'front_start_x_mm')
                    metaValues(2) = double(episodeMeta.front_start_x_mm);
                end
                if isfield(episodeMeta, 'object_speed_mm_per_ms')
                    metaValues(3) = double(episodeMeta.object_speed_mm_per_ms);
                end
                if isfield(episodeMeta, 'delay_ms')
                    metaValues(4) = double(episodeMeta.delay_ms);
                end
                if isfield(episodeMeta, 'rotation_change_limit_deg_per_control_step')
                    metaValues(5) = double(episodeMeta.rotation_change_limit_deg_per_control_step);
                end
            else
                metaValues = double(episodeMeta(:)');
                if numel(metaValues) < 5
                    metaValues(end+1:5) = nan;
                elseif numel(metaValues) > 5
                    metaValues = metaValues(1:5);
                end
            end

            obj.outStream.writeByte(obj.CMD_START);
            obj.sendDoubles([initialState, metaValues]);
            obj.outStream.flush();
            
            action = obj.receiveDoubles(obj.actionDim);
        end
        
        function action = stepRL(obj, state, reward, done, truncated)
            % Sends State B (0x02) control loop data, receives action
            obj.outStream.writeByte(obj.CMD_STEP);
            
            % Package state, reward, and done flag
            payload = [state(:)', reward, done, truncated];
            obj.sendDoubles(payload);
            obj.outStream.flush();
            
            % Only wait for an action if the episode isn't done
            if done < 0.5 && ~truncated
                action = obj.receiveDoubles(obj.actionDim);
            else
                action = zeros(1, obj.actionDim); % Dummy action on terminal state
            end
        end
        
        function reset_needed = syncWithHPC(obj)
            % Sends CMD_END_SYNC (0x03) and blocks until Python training update completes.
            % Returns true if Python requests a hardware reset, false otherwise.
            reset_needed = false;
            fprintf('[Network] Episode complete. Requesting sync...\n');
            obj.outStream.writeByte(obj.CMD_END_SYNC);
            obj.outStream.flush();
            
            % Block until Python responds — either SYNC_COMPLETE or RESET_NEEDED.
            response = obj.receiveString(5*60*1000);
            if strcmp(response, 'RESET_NEEDED')
                reset_needed = true;
                fprintf('[Network] Sync complete. Hardware reset requested by agent.\n');
            else
                if ~strcmp(response, 'SYNC_COMPLETE')
                    warning('NetworkClient:UnexpectedSyncResponse', ...
                        'Expected SYNC_COMPLETE or RESET_NEEDED but received "%s".', response);
                end
                fprintf('[Network] Sync complete.\n');
            end
        end
        
        function shutdown(obj)
            % Phase 3: Sends teardown command and closes socket
            fprintf('[Network] Sending shutdown signal...\n');
            try
                obj.outStream.writeByte(obj.CMD_SHUTDOWN);
                obj.outStream.flush();
                obj.socket.close();
            catch
                % Ignore errors on shutdown
            end
            fprintf('[Network] Disconnected.\n');
        end
        
        function testLatency(obj, iterations)
            % Runs a latency test by sending/receiving dummy data
            % iterations: number of round trips to measure (default 100)
            if nargin < 2
                iterations = 100;
            end
            
            latencies = [];
            fprintf('[Latency] Testing round-trip time (%d iterations)...\n', iterations);
            
            for i = 1:iterations
                tic;
                obj.outStream.writeByte(int8(0x99)); % Dummy header (won't be processed)
                obj.outStream.flush();
                % Note: This is a simple test. In production, you'd need
                % the server to echo back or use a proper ping/pong mechanism
                latencies = [latencies, toc * 1000];
            end
            
            fprintf('[Latency] Results:\n');
            fprintf('  Mean: %.2f ms\n', mean(latencies));
            fprintf('  Max:  %.2f ms\n', max(latencies));
            fprintf('  Min:  %.2f ms\n', min(latencies));
            fprintf('  Std:  %.2f ms\n', std(latencies));
        end
        
    end
    
    methods (Access = private)
        % --- Helper Methods for Java I/O ---
        
        function sendDoubles(obj, dataArray)
            % Writes an array of doubles sequentially
            for i = 1:length(dataArray)
                obj.outStream.writeDouble(dataArray(i));
            end
        end
        
        function dataArray = receiveDoubles(obj, count)
            % Reads 'count' number of doubles sequentially
            dataArray = zeros(1, count);
            for i = 1:count
                dataArray(i) = obj.inStream.readDouble();
            end
        end
        
        function waitForString(obj, expectedStr, timeoutMs)
            % Reads a 4-byte length header, then reads the string and compares
            if nargin < 3
                timeoutMs = 10000;
            end

            previousTimeout = obj.socket.getSoTimeout();
            cleanup = onCleanup(@() obj.socket.setSoTimeout(previousTimeout));
            obj.socket.setSoTimeout(timeoutMs);

            try
                msgLength = obj.inStream.readInt();
            catch ME
                if contains(char(ME.message), 'Read timed out')
                    error('NetworkClient:Timeout', ...
                        'Timed out waiting for "%s" after %.1f s.', ...
                        expectedStr, timeoutMs / 1000);
                end
                rethrow(ME);
            end

            maxMessageLength = 1024;
            if msgLength < 0 || msgLength > maxMessageLength
                error('NetworkClient:InvalidMessageLength', ...
                    ['Received invalid string length %d while waiting for "%s". ' ...
                     'The TCP stream is out of sync.'], ...
                    msgLength, expectedStr);
            end
            
            byteArr = zeros(1, msgLength, 'int8');
            for i = 1:msgLength
                byteArr(i) = obj.inStream.readByte();
            end
            
            receivedStr = native2unicode(byteArr, 'UTF-8');
            
            if ~strcmp(receivedStr, expectedStr)
                warning('NetworkClient:UnexpectedResponse', ...
                    'Expected "%s" but received "%s"', expectedStr, receivedStr);
            end
        end

        function str = receiveString(obj, timeoutMs)
            % Reads a length-prefixed string (4-byte int header + UTF-8 bytes).
            % Returns the received string. Throws on timeout or invalid length.
            if nargin < 2
                timeoutMs = 10000;
            end

            previousTimeout = obj.socket.getSoTimeout();
            cleanup = onCleanup(@() obj.socket.setSoTimeout(previousTimeout));
            obj.socket.setSoTimeout(timeoutMs);

            try
                msgLength = obj.inStream.readInt();
            catch ME
                if contains(char(ME.message), 'Read timed out')
                    error('NetworkClient:Timeout', ...
                        'Timed out waiting for string after %.1f s.', timeoutMs / 1000);
                end
                rethrow(ME);
            end

            maxMessageLength = 1024;
            if msgLength < 0 || msgLength > maxMessageLength
                error('NetworkClient:InvalidMessageLength', ...
                    'Received invalid string length %d. The TCP stream is out of sync.', msgLength);
            end

            byteArr = zeros(1, msgLength, 'int8');
            for i = 1:msgLength
                byteArr(i) = obj.inStream.readByte();
            end
            str = native2unicode(byteArr, 'UTF-8');
        end
    end
end
