classdef Jakiro < handle
    % Jakiro Top-level app that composes two CarriageApp instances
    properties
        CC1
        CC2
        WA
        UIFigure
        GridLayout
        ExpPanel
        pathpathTimer = [] % timer for path tracking both carriages
        net % agent server connection
    end

    properties
        run_start_time = [] % time when experiment started
        cc1_done = false; % flag indicating carriage 1 finished path tracking
        cc2_done = false; % flag indicating carriage 2 finished path tracking
        pathpath_tick_period_ms = 10; % period of path tracking timer in ms
        pathpath_redraw_interval = 20; % update interval for path tracking
        outpath % output path for wavi data
        n_rl_interval = 4; % number of samples between RL agent action updates (for rl agent control modes)
        n_ch_total = 23; % total number of channels in the state sent to the agent (for rl agent control modes)
        agent_server_address = '127.0.0.1'; % address of the agent server (for rl agent control modes)
        agent_server_port = 65432; % port of the agent server (for rl agent control modes)
        currentState = []; % current state for RL agent control
    end
    methods
        function app = Jakiro()
            % create main UI
            delete(timerfindall);
            delete_serial_ports;
            close all; clc;

            app.UIFigure = uifigure('Position',[100 60 1200 980]);
            % ensure clicking the window X triggers app cleanup
            app.UIFigure.CloseRequestFcn = @(src,evt) app.onFigureClose(src,evt);
            app.GridLayout = uigridlayout(app.UIFigure,[3 3]);
            uigl = app.GridLayout;
            uigl.RowHeight = {320,240,420};

            ax = create_ax(parent=uigl);
            ax.Layout.Column = [1 3];
            ax.Layout.Row = 3;

            % Front carriage
            cc1_panel = uipanel(uigl,BackgroundColor=[0 0.4470 0.7410]);
            cc1_panel.Layout.Column = 1;
            cc1_panel.Layout.Row = [1 2];
            app.CC1 = CarriageApp(cc1_panel,ax);
            app.CC1.Car.origin = [5010, 0]; % parking position for carriage 1 (steps, 250 mm offset)
            app.CC1.Car.origin_mm = app.CC1.Car.origin * app.CC1.Car.step2mm;
            app.CC1.Car.path_dx_max = 3800; % max x movement range (mm)
            app.CC1.Car.motor_settings.ACC = 10000;
            app.CC1.Car.name = 'Front Carriage';
            app.CC1.Car.control_aoa = false;
            app.CC1.Car.path_CMD_INTERVAL = app.pathpath_tick_period_ms;
            app.CC1.CarView.mark.DisplayName = 'Front Carriage';
            
            % Back carriage
            cc2_panel = uipanel(uigl);
            cc2_panel.Layout.Column = 2;
            cc2_panel.Layout.Row = [1 2];
            app.CC2 = CarriageApp(cc2_panel,ax);
            app.CC2.Car.joystick_side = 'R';
            app.CC2.Car.path_dx_max = 3500; % max x movement range (mm)
            app.CC2.Car.motor_settings.ACC = 10000;
            app.CC2.Car.name = 'Back Carriage';
            app.CC2.Car.control_aoa = true;
            app.CC2.Car.path_CMD_INTERVAL = app.pathpath_tick_period_ms;
            app.CC2.Car.x_min_mm = 0;
            app.CC2.Car.x_max_mm = 3500;
            app.CC2.Car.y_min_mm = 0;
            app.CC2.Car.y_max_mm = 900;
            app.CC2.Car.boundary_margin_mm = 50;

            app.CC2.hArrow.Visible = 'on';
            app.CC2.CarView.mark.DisplayName = 'Back Carriage';
            app.CC2.CarView.mark.Marker = 's';


            % Data acquisition
            wa_panel = uipanel(uigl);
            wa_panel.Layout.Column = 3;
            wa_panel.Layout.Row = 1;
            user_dir = getenv('USERPROFILE');
            if isempty(user_dir)
                user_dir = getenv('HOME');
            end
            if isempty(user_dir)
                user_dir = '.';
            end
            outpath = fullfile(user_dir,'wavi_data');
            if ~isfolder(outpath)
                mkdir(outpath);
            end
            app.outpath = outpath;
            app.WA = wavi(wa_panel,false,n_update=1,ns_read=4,ns_fill=8,...
                            ch_map =load('channel_map.txt'),... % not standalone mode
                            outpath = outpath,...
                            scale = 10);

            % Experiment control panel in row 2, column 3
            exp_panel_parent = uipanel(uigl,'Title','Experiment');
            exp_panel_parent.Layout.Column = 3;
            exp_panel_parent.Layout.Row = 2;
            app.ExpPanel = ExpControlPanel(exp_panel_parent);
            % listen for path-mode events
            addlistener(app.ExpPanel,'PathPath',@(src,evt) app.onPathPath(src,evt));
            addlistener(app.ExpPanel,'PathHuman',@(src,evt) app.onPathHuman(src,evt));
            addlistener(app.ExpPanel,'PathAgentPre',@(src,evt) app.onPathAgentPre(src,evt));
            addlistener(app.ExpPanel,'PathAgentLive',@(src,evt) app.onPathAgentLive(src,evt));
            app.connect_agent_server(); % connect to agent server at startup (for RL agent control modes)
        end
    
        function connect_agent_server(app)

            STATE_DIM = app.n_rl_interval*app.n_ch_total;
            ACTION_DIM = 2;
            MAX_EPISODES = 1;
            SAMPLE_RATE = 80;

            config.mode = 'infer';
            config.hpc_port = 5555;
            config.n_rl_interval = app.n_rl_interval;
            config.n_ch_total = app.n_ch_total;
            config.state_dim = STATE_DIM;
            config.action_dim = ACTION_DIM;
            config.max_episodes = MAX_EPISODES;
            config.sample_rate = SAMPLE_RATE;
            config.dt = 1/SAMPLE_RATE;
            % connect_agent_server Connect to the agent server at the specified address and port
            try
                app.net = NetworkClient(app.agent_server_address, app.agent_server_port,STATE_DIM, ACTION_DIM);
                % Phase 1: Handshake
                app.net.sendConfig(config);
                fprintf('Connected to agent server at %s:%d\n', app.agent_server_address, app.agent_server_port);
            catch ME
                warning('AgentConnectionError');
                fprintf('Failed to connect to agent server at %s:%d: \n%s', app.agent_server_address, app.agent_server_port, ME.message);
                app.net = [];
            end
            app.currentState = zeros(app.n_ch_total,app.n_rl_interval); % hardware state buffer
        end
    end

    methods % event handlers

        function onFigureClose(app, src, ~)
            % Disable further CloseRequest callbacks to avoid recursion
            try
                src.CloseRequestFcn = [];
            catch
            end
            % Call the app close routine
            try
                app.close();
            catch
                % fallback: delete figure directly
                try
                    delete(src);
                catch
                end
            end
        end

        function start(app)
            % start Bring UIFigure to front and attempt to start child apps
            try
                figure(app.UIFigure);
            catch
            end
            % if child apps have start methods, call them
            if isprop(app,'CC1') && ~isempty(app.CC1)
                try
                    if ismethod(app.CC1,'start')
                        app.CC1.start();
                    end
                catch
                end
            end
            if isprop(app,'CC2') && ~isempty(app.CC2)
                try
                    if ismethod(app.CC2,'start')
                        app.CC2.start();
                    end
                catch
                end
            end
        end

        function close(app)
            % close Stop child apps, delete timers and serial ports, and close UI
            % Stop/cleanup CC1
            try
                if isprop(app,'CC1') && ~isempty(app.CC1)
                    app.CC1.onAppClose();
                end
            catch
            end
            % Stop/cleanup CC2
            try
                if isprop(app,'CC2') && ~isempty(app.CC2)
                    app.CC2.onAppClose();
                end
            catch
            end

            % Stop/cleanup Wavi (data acquisition)
            try
                if isprop(app,'WA') && ~isempty(app.WA)
                    app.WA.cleanup();
                end
            catch
            end

            % close UIFigure
            try
                if isvalid(app.UIFigure)
                    delete(app.UIFigure);
                end
            catch
            end
        end

        function onPathPath(app, ~, ~)
            % coordinate the two carriages using parameters from ExpPanel (formerly StartExperiment)
            try
                [v1,v2,delay_s,run_tag,~] = app.ExpPanel.getParameters();
            catch
                warning('Failed to read experiment parameters.');
                return
            end

            % set velocities
            app.CC1.Car.vel_max = round(v1*1000/app.CC1.Car.step2mm);
            app.CC2.Car.vel_max = round(v2*1000/app.CC2.Car.step2mm);

            % move back carriage first
            [xp,yp,rp,thetap2,L,start_x,start_y,start_s,pathtag2] = app.CC2.prepare_pathtracking_data(); % generate data from selected file
            try
                start(app.CC2.redrawTimer);
            catch
            end
            app.CC2.Car.sendCommand('NUL,NUL,ABS0>');
            pause(0.2);
            app.CC2.Car.moveToPositionMM(start_x-app.CC2.Car.origin_mm(1), start_y-app.CC2.Car.origin_mm(2), 20, 1, false); 
            app.CC2.Car.init_pathtracking_variables(xp,yp,rp,L,start_s,thetap2);
            stop(app.CC2.redrawTimer);
            
            %
            [xp,yp,rp,thetap1,L,start_x,start_y,start_s,pathtag1] = app.CC1.prepare_pathtracking_data(); % generate data from selected file
            try
                start(app.CC1.redrawTimer);
            catch
            end
            app.CC1.Car.moveToPositionMM(start_x-app.CC1.Car.origin_mm(1), start_y-app.CC1.Car.origin_mm(2), 20, 1, false); 
            app.CC1.Car.init_pathtracking_variables(xp,yp,rp,L,start_s,thetap1);
            stop(app.CC1.redrawTimer);
            
            % create or reconfigure the timer
            if isempty(app.pathpathTimer) || ~isvalid(app.pathpathTimer)
                app.pathpathTimer = timer('TimerFcn',@(src,evt) app.path_path_tick(src,evt),...
                    'Period', app.pathpath_tick_period_ms/1000, 'ExecutionMode','fixedrate', 'Name','Path Path timer', 'BusyMode','drop');
            else
                try
                    stop(app.pathpathTimer);
                catch
                end
            end
            
            % prepare daq
            try
                if ~isempty(app.WA.s)
                    app.WA.tag = sprintf('%s_%s-v1=%.2f_%s-v2=%.2f_delay=%.1f',run_tag,pathtag1,v1,pathtag2,v2,delay_s);
                    app.WA.is_recording = true;
                    app.WA.init_datalog_file();
                    app.WA.align_data_read();
                    app.WA.average_signal_as_offset(round(app.WA.Fs));
                    app.WA.reset_data_buffers();
                end
            catch
            end
            app.run_start_time = datetime('now');
            % start(app.pathpathTimer);

            % blocking path tracking loop
            period = app.pathpath_tick_period_ms/1000; % seconds
            t0 = tic;
            n = 0;
            app.CC1.hArrow.Visible = 'on';
            app.CC2.hArrow.Visible = 'on';

            % Main loop: build src/event and call pathTrackingTick(app,src,event)
            while true
                t1 = tic;
                n = n + 1;

                % synthetic src/event objects to mimic timer callback
                src = struct();
                src.TasksExecuted = n;
                ev = struct();
                ev.Data.time = datetime('now');

                % call tick; it returns true when done
                try
                    is_done = app.path_path_tick(src, ev);
                catch ME
                    warning('pathTrackingTick:error','Error in pathTrackingTick: %s', ME.message);
                    is_done = true;
                end

                if is_done
                    break;
                end

                % schedule next tick using tic/toc to reduce drift
                next_time = n * period;
                elapsed = toc(t0);

                fprintf('PathPath Tick %d: elapsed total=%.3f s, frame=%.1f ms, avg FPS=%.1f\n',...
                 n, elapsed, toc(t1)*1000, n/elapsed);
                
                % busy-wait the final few ms to reduce jitter
                while toc(t0) < next_time
                    pause(0.0005); % yield briefly to keep UI responsive, NOTE pause <=1ms triggers higher resolution OS timer.
                end
            end

            % ensure any requested stop is applied and notify listeners
            try
                app.CC1.Car.stopPathTracking();
                app.CC2.Car.stopPathTracking();
            catch
            end
            app.CC1.hArrow.Visible = 'off';
            app.CC2.hArrow.Visible = 'off';
            app.WA.is_recording = false;
            app.WA.close_datafile();
        end

        % additional mode handlers (placeholders)
        function onPathHuman(app, ~, ~)
            % PathHuman: CC1 follows a prescribed path, CC2 is human-controlled
            % Both run in the same blocking loop to avoid timer jitter
            
            try
                [v1,v2,delay_s,run_tag,episode_time_s] = app.ExpPanel.getParameters();
            catch
                warning('Failed to read experiment parameters.');
                return
            end

            % set velocity limits for both carriages
            app.CC1.Car.vel_max = round(v1*1000/app.CC1.Car.step2mm);
            app.CC2.Car.vel_max = round(v2*1000/app.CC2.Car.step2mm);

            % prepare CC1 for path tracking
            [xp,yp,rp,thetap1,L,start_x,start_y,start_s,~] = app.CC1.prepare_pathtracking_data();
            try
                start(app.CC1.redrawTimer);
            catch
            end
            app.CC1.Car.moveToPositionMM(start_x-app.CC1.Car.origin_mm(1), start_y-app.CC1.Car.origin_mm(2), 20, 1, false);
            app.CC1.Car.init_pathtracking_variables(xp,yp,rp,L,start_s,thetap1);
            stop(app.CC1.redrawTimer);

            % prepare CC2 for interactive control (human)
            app.CC2.Car.poll_gamepad = 1;
            app.CC2.Car.poll_keyboard = 1;

            % prepare daq
            try
                if ~isempty(app.WA.s)
                    app.WA.tag = sprintf('%s_PathHuman-v1=%.2f_v2max=%.2f_delay=%.1f',run_tag,v1,v2,delay_s);
                    app.WA.is_recording = true;
                    app.WA.init_datalog_file();
                    app.WA.align_data_read();
                    app.WA.average_signal_as_offset(round(app.WA.Fs));
                    app.WA.reset_data_buffers();
                end
            catch
            end
            
            app.run_start_time = datetime('now');

            % blocking path+human control loop
            period = app.pathpath_tick_period_ms/1000; % seconds
            t0 = tic;
            n = 0;
            app.CC1.hArrow.Visible = 'on';
            app.CC2.hArrow.Visible = 'on';
            app.CC2.Car.cmd_npoll = 0; % reset poll counter

            is_done_cc1 = false; % track when CC1 finishes path tracking
            while true
                t1 = tic;
                n = n + 1;

                % synthetic src/event objects to mimic timer callback
                src = struct();
                src.TasksExecuted = n;
                ev = struct();
                ev.Data.time = datetime('now');

                % CC1: path tracking
                if ~is_done_cc1
                    try
                        is_done_cc1 = app.CC1.Car.pathTrackingTick(src, ev);
                    catch ME
                        warning(ME.identifier, '%s', ME.message);
                        is_done_cc1 = true;
                    end
                end
                now_dt = datetime('now');
                elapsed_time_sec = seconds(now_dt - app.run_start_time);

                % stop condition: episode time exceeded
                if elapsed_time_sec > episode_time_s
                    break;
                end
                
                % CC2: interactive control (human controlled, no timer)
                if elapsed_time_sec > delay_s
                    try
                        app.CC2.Car.interactControlTick(src, ev);
                    catch ME
                        warning(ME.identifier, '%s', ME.message);
                    end
                end

                % data acquisition
                try
                    if ~isempty(app.WA.s)
                        app.WA.read_update_tick();
                    end
                catch
                end

                % update visuals periodically
                if mod(n, app.pathpath_redraw_interval) == 0
                    app.CC1.update_view();
                    app.CC2.update_view();
                    drawnow limitrate
                end

                % schedule next tick
                next_time = n * period;
                elapsed = toc(t0);

                fprintf('PathHuman Tick %d: elapsed total=%.3f s, frame=%.1f ms, avg FPS=%.1f\n',...
                    n, elapsed, toc(t1)*1000, n/elapsed);

                % busy-wait with sleep to reduce jitter
                while toc(t0) < next_time
                    pause(0.0005);
                end
            end

            % cleanup
            try
                app.CC1.Car.stopPathTracking();
            catch
            end
            app.CC2.Car.poll_gamepad = 0;
            app.CC2.Car.poll_keyboard = 0;
            app.CC1.hArrow.Visible = 'off';
            app.CC2.hArrow.Visible = 'off';
            app.WA.is_recording = false;
            app.WA.close_datafile();
        end

        function onPathAgentPre(app, ~, ~)
            % PathAgentPre: CC1 follows a prescribed path, CC2 is agent-controlled
            % Both run in the same blocking loop to avoid timer jitter
            
            try
                [v1,v2,delay_s,run_tag,episode_time_s] = app.ExpPanel.getParameters();
            catch
                warning('Failed to read experiment parameters.');
                return
            end

            % set velocity limits for both carriages
            app.CC1.Car.vel_max = round(v1*1000/app.CC1.Car.step2mm);
            app.CC2.Car.vel_max = round(v2*1000/app.CC2.Car.step2mm);
            cc2_state_buffer = zeros(5, app.n_rl_interval); % back carriage state buffer for constructing state to send to agent
            % prepare daq
            try
                if ~isempty(app.WA.s)
                    app.WA.ns_read = app.n_rl_interval; % set number of samples to read per tick to match agent action update interval
                    app.WA.n_update = 2;
                    app.WA.n_fill = 1;
                    app.WA.tag = sprintf('%s_PathAgentPre-v1=%.2f_v2max=%.2f_delay=%.1f',run_tag,v1,v2,delay_s);
                    app.WA.is_recording = true;
                    app.WA.init_datalog_file();
                    app.WA.align_data_read();
                    app.WA.average_signal_as_offset(round(app.WA.Fs));
                    app.WA.reset_data_buffers();
                end
            catch
                % error('DataAcquisitionError', 'Error during data acquisition setup: %s', e.message);
            end

            % connect to agent server

            % prepare CC1 for path tracking
            [xp,yp,rp,thetap1,L,start_x,start_y,start_s,~] = app.CC1.prepare_pathtracking_data();
            try
                start(app.CC1.redrawTimer);
            catch
            end
            app.CC1.Car.moveToPositionMM(start_x-app.CC1.Car.origin_mm(1), start_y-app.CC1.Car.origin_mm(2), 20, 1, false);
            app.CC1.Car.init_pathtracking_variables(xp,yp,rp,L,start_s,thetap1);
            stop(app.CC1.redrawTimer);

            % prepare CC2 for interactive control (human)
            app.CC2.Car.poll_gamepad = 0;
            app.CC2.Car.poll_keyboard = 1; % for interupting the agent control

            
            % blocking path+ agent control loop
            period = 12.5/1000; % seconds, hardcoded control loop period for agent control (80 Hz)
            t0 = tic;
            n = 0;
            app.CC1.hArrow.Visible = 'on';
            app.CC2.hArrow.Visible = 'on';
            app.CC2.Car.cmd_npoll = 0; % reset poll counter
            
            is_done_cc1 = false; % track when CC1 finishes path tracking
            is_done = 0; % track when episode is done for agent (currently only time-based truncation, no early termination condition)
            truncated = 0;
            action = app.net.startEpisode(app.currentState(:)'); % get initial action from agent
            app.run_start_time = datetime('now');
            while true
                t1 = tic;
                n = n + 1;

                % synthetic src/event objects to mimic timer callback
                src = struct();
                src.TasksExecuted = n;
                ev = struct();
                ev.Data.time = datetime('now');

                % CC1: path tracking
                if ~is_done_cc1
                    try
                        is_done_cc1 = app.CC1.Car.pathTrackingTick(src, ev);
                    catch ME
                        warning(ME.identifier, '%s', ME.message);
                        is_done_cc1 = true;
                    end
                end
                now_dt = datetime('now');
                elapsed_time_sec = seconds(now_dt - app.run_start_time);

                % stop condition: episode time exceeded
                if elapsed_time_sec > episode_time_s
                    truncated = 1;
                end
                
                % CC2: agent control
                if elapsed_time_sec > delay_s
                    cc2_state_buffer = circshift(cc2_state_buffer, [0 -1]);
                    cc2_state_buffer(1,end) = (elapsed_time_sec - delay_s) * 1000; % timestamp in ms relative to agent control start
                    cc2_state_buffer(2:3,end) = app.CC2.Car.real_loc; % current position of back carriage
                    cc2_state_buffer(4:5,end) = app.CC2.Car.real_vel; % current velocity of back carriage
                    vx_in = round(action(1)*1000/app.CC2.Car.step2mm);
                    vy_in = round(action(2)*1000/app.CC2.Car.step2mm);
                    try
                        app.CC2.Car.agentControlStep(ev, vx_in, vy_in);
                    catch ME
                        warning(ME.identifier, '%s', ME.message);
                    end
                    reward = 0; % placeholder reward
                end

                % data acquisition
                try
                    if ~isempty(app.WA.s)
                        new_samples = app.WA.rl_read_update_tick();
                    end
                catch ME
                    % error('DataAcquisitionError', 'Error during data acquisition: %s', ME.message);
                end

                if elapsed_time_sec > delay_s && ~isempty(new_samples)
                    % construct state for agent
                    app.currentState = [cc2_state_buffer; new_samples']; % 5 rows of carriage state + 18 rows of daq data, each with n_rl_interval columns
                    % send state to agent and receive action
                    action = app.net.stepRL(app.currentState(:)', reward, is_done, truncated); % flatten state to 1D array for sending to agent
                end

                % update visuals periodically
                if mod(n, app.pathpath_redraw_interval) == 0
                    app.CC1.update_view();
                    app.CC2.update_view();
                    % drawnow limitrate
                end

                % schedule next tick
                next_time = n * period;
                elapsed = toc(t0);

                fprintf('PathAgentPre Tick %d: elapsed total=%.3f s, frame=%.1f ms, avg FPS=%.1f\n',...
                    n, elapsed, toc(t1)*1000, n/elapsed);

                % busy-wait with sleep to reduce jitter
                while toc(t0) < next_time
                    pause(0.0005);
                end

                if truncated
                    fprintf('Episode truncated after %.2f seconds\n', elapsed_time_sec);
                    break;
                end
            end

            % cleanup
            try
                app.CC1.Car.stopPathTracking();
            catch
            end
            app.CC2.Car.poll_gamepad = 0;
            app.CC2.Car.poll_keyboard = 0;
            app.CC1.hArrow.Visible = 'off';
            app.CC2.hArrow.Visible = 'off';
            app.WA.is_recording = false;
            app.WA.close_datafile();
        end

        function onPathAgentLive(app, ~, ~)
            disp('PathAgentLive triggered (placeholder)');
        end

    end

    methods %
        function is_done = path_path_tick(app,src,event)
            is_done = false;

            fprintf('Path path tick %d\n', src.TasksExecuted);
            if src.TasksExecuted <= 1
                app.cc1_done = false;
                app.cc2_done = false;
            end

            % both carriages run in path tracking mode
            if ~app.cc1_done
                app.cc1_done = app.CC1.Car.pathTrackingTick(src,event);
            end
            
            now_dt = datetime('now');
            elapsed_time_sec = seconds(now_dt - app.run_start_time);
            
            if elapsed_time_sec > app.ExpPanel.DelayField.Value && ~app.cc2_done
                app.cc2_done = app.CC2.Car.pathTrackingTick(src,event);
            end

            % app.WA.read_serial_data(app.WA.ns_read, app.WA.ns_fill);
            try
                if ~isempty(app.WA.s)
                    app.WA.read_update_tick();
                end
            catch
            end

            if mod(src.TasksExecuted,app.pathpath_redraw_interval)==0
                app.CC1.update_view();
                app.CC2.update_view();
                % app.WA.update_visuals();
                drawnow limitrate
            end
            
            if app.cc1_done && app.cc2_done
                is_done = true;
                % finished path tracking for carriage 1
                try
                    stop(src);
                    fclose(app.WA.fout);
                catch
                end
            end
        end
    end
end
