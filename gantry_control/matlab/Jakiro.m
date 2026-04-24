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
        wa_Fs = 80; % sampling rate for wavi data acquisition (Hz)
        num_whiskers = 9;
        n_rl_interval = 4; % number of samples between RL agent action updates (for rl agent control modes)
        n_ch_total = 23; % total number of channels in the state sent to the agent (for rl agent control modes)
        agent_server_address = '127.0.0.1'; % address of the agent server (for rl agent control modes)
        agent_server_port = 65432; % port of the agent server (for rl agent control modes)
        agent_server_hpc_port = 5555;
        agent_server_mode = 'train';
        agent_server_max_episodes = 1;
        agent_server_record_trajectory = false;
        agent_server_use_random_paths = false;
        agent_policy_package_dir = 'agents/rl_sac_v4_pathblind_hardware';
        agent_server_pid = [];
        server_config_window = [];
        currentState = []; % current state for RL agent control
        calibration_data_file = 'sensor_calibration/calibration_sim_v1.csv' % calibration data file for converting raw sensor readings to bending moments (for rl agent control modes)
        exp2sim_channel_map_file = 'sensor_calibration/ch_map_sim_v1.csv'; % mapping from hardware channels to simulated sensor channels for RL agent control modes
        signal_to_bending_moment = [];
        episode_path_plot_handle = [];
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
            uigl.RowHeight = {260,300,420};

            ax = create_ax(parent=uigl);
            ax.Layout.Column = [1 3];
            ax.Layout.Row = 3;

            % Front carriage
            cc1_panel = uipanel(uigl,BackgroundColor=[0 0.4470 0.7410]);
            cc1_panel.Layout.Column = 1;
            cc1_panel.Layout.Row = [1 2];
            app.CC1 = CarriageApp(cc1_panel,ax);
            app.CC1.Car.origin = [4008, 0]; % parking position for carriage 1 (steps, 200 mm offset)
            app.CC1.Car.origin_mm = app.CC1.Car.origin * app.CC1.Car.step2mm;
            app.CC1.Car.path_dx_max = 4100; % max x movement range (mm)
            app.CC1.Car.x_max_mm = 4100; % hard x boundary for interactive/manual control
            app.CC1.Car.y_min_mm = 0; % hard y boundaries for interactive/manual control
            app.CC1.Car.y_max_mm = 900;
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
            app.CC2.Car.path_dx_max = 3850; % max x movement range (mm)
            app.CC2.Car.motor_settings.ACC = 10000;
            app.CC2.Car.name = 'Back Carriage';
            app.CC2.Car.control_aoa = true;
            app.CC2.Car.path_CMD_INTERVAL = app.pathpath_tick_period_ms;
            app.CC2.Car.x_min_mm = 0;
            app.CC2.Car.x_max_mm = 3850;
            app.CC2.Car.y_min_mm = 0;
            app.CC2.Car.y_max_mm = 900;
            app.CC2.Car.boundary_margin_mm = 50;

            app.CC2.hArrow.Visible = 'on';
            app.CC2.CarView.mark.DisplayName = 'Back Carriage';
            app.CC2.CarView.mark.Marker = 's';

            % Inter-carriage velocity limiting
            app.CC1.Car.peer = app.CC2.Car;
            app.CC2.Car.peer = app.CC1.Car;

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
            sig_filter_options = struct('filterType','lowpass-iir',...
                                        'fs',app.wa_Fs,...
                                        'order',3,...
                                        'cutoffHz',2,...
                                        'highpassHz',0.01,...
                                        'gain',1.2,...
                                        'nChannels',app.num_whiskers*2);
            app.WA = wavi(wa_panel,false,nsensor=app.num_whiskers,...
                            n_update=1,ns_read=app.n_rl_interval,ns_fill=6,...
                            ch_map =load('channel_map.txt'),... % not standalone mode
                            sig_filter = SigFilter(sig_filter_options),...
                            filter_add_offset_back = false,... % remove offset for rl
                            outpath = outpath,...
                            t_buffer = 120,...
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
            addlistener(app.ExpPanel,'PathAgentTrain',@(src,evt) app.onPathAgentTrain(src,evt));
            addlistener(app.ExpPanel,'FilterConfigRequested',@(src,evt) app.onFilterConfigRequested(src,evt));
            addlistener(app.ExpPanel,'ServerConfigRequested',@(src,evt) app.onServerConfigRequested(src,evt));

            try
                app.signal_to_bending_moment = SignalToBendingMoment( ...
                    app.calibration_data_file, app.exp2sim_channel_map_file);
                fprintf('Bending moment conversion configured.\n');
            catch me
                error('SignalConversionError:InitFailed', 'Failed to initialize SignalToBendingMoment: %s', me.message);
            end
        end
    
        function [ok,msg] = connect_agent_server(app, userConfig)

            if nargin < 2 || isempty(userConfig)
                userConfig = app.getServerConfig();
            end

            STATE_DIM = app.n_rl_interval*app.n_ch_total;
            ACTION_DIM = 2;
            SAMPLE_RATE = 80;
            use_random_paths = isfield(userConfig, 'use_random_paths') && logical(userConfig.use_random_paths);

            config.mode = userConfig.mode;
            config.hpc_port = app.agent_server_hpc_port;
            config.n_rl_interval = app.n_rl_interval;
            config.n_ch_total = app.n_ch_total;
            config.state_dim = STATE_DIM;
            config.action_dim = ACTION_DIM;
            config.max_episodes = userConfig.max_episodes;
            config.sample_rate = SAMPLE_RATE;
            config.dt = 1/SAMPLE_RATE;
            config.num_whiskers = app.num_whiskers;
            config.record_trajectory = logical(userConfig.record_trajectory);
            config.use_random_paths = use_random_paths;
            config.policy_package_dir = userConfig.policy_package_dir;
            config.visualize = isfield(userConfig, 'visualize') && logical(userConfig.visualize);
            % Parameters used by the Python agent for observation normalisation and action scaling
            try
                [v1,v2,~,~,episode_time_s,~,rotation_step_deg] = app.ExpPanel.getParameters();
            catch
                v1 = 0.2;
                v2 = 0.16;
                episode_time_s = 38.0; % default fallback
                rotation_step_deg = 1.5;
            end
            config.episode_time_ms = episode_time_s * 1000;
            config.fixed_vx = v2;          % mm/ms — fixed forward speed for the RL-controlled back carriage
            config.rotation_change_limit_deg_per_control_step = rotation_step_deg;
            config.reward_source = 'matlab'; % hardware loop remains the reward source of truth

            % Pass path data so the server can compute reward
            if ~use_random_paths
                try
                    pathFiles = app.CC1.ModePanel.SelectedFiles;
                    if ~isempty(pathFiles)
                        paths_cell = cell(1, numel(pathFiles));
                        for ip = 1:numel(pathFiles)
                            pd = PathData(pathFiles{ip});
                            paths_cell{ip} = [pd.x(:), pd.y(:)];
                        end
                        config.path_data = paths_cell;
                    end
                catch pathErr
                    fprintf('[connect_agent_server] Warning: could not read path files: %s\n', pathErr.message);
                end
            end

            % Checkpoint / resume settings from server config window
            if isfield(userConfig, 'output_dir')
                config.output_dir = userConfig.output_dir;
            end
            if isfield(userConfig, 'resume')
                config.resume = userConfig.resume;
            end
            if isfield(userConfig, 'resume_path')
                config.resume_path = userConfig.resume_path;
            end
            if isfield(userConfig, 'keep_checkpoints')
                config.keep_checkpoints = userConfig.keep_checkpoints;
            end
            if isfield(userConfig, 'checkpoint_every_episodes')
                config.checkpoint_every_episodes = userConfig.checkpoint_every_episodes;
            end

            % connect_agent_server Connect to the agent server at the specified address and port
            try
                app.net = NetworkClient(app.agent_server_address, app.agent_server_port,STATE_DIM, ACTION_DIM);
                % Phase 1: Handshake
                app.net.sendConfig(config);
                fprintf('Connected to agent server at %s:%d\n', app.agent_server_address, app.agent_server_port);
                ok = true;
                msg = sprintf('Connected to agent server at %s:%d.', app.agent_server_address, app.agent_server_port);
            catch ME
                warning('AgentConnectionError');
                fprintf('Failed to connect to agent server at %s:%d: \n%s', app.agent_server_address, app.agent_server_port, ME.message);
                app.net = [];
                ok = false;
                msg = sprintf('Failed to connect to server: %s', ME.message);
            end
            app.currentState = zeros(app.n_ch_total,app.n_rl_interval); % hardware state buffer
        end

        function config = getServerConfig(app)
            config = struct();
            config.mode = app.agent_server_mode;
            config.hpc_port = app.agent_server_hpc_port;
            config.max_episodes = app.agent_server_max_episodes;
            config.record_trajectory = app.agent_server_record_trajectory;
            config.use_random_paths = app.agent_server_use_random_paths;
            config.policy_package_dir = app.agent_policy_package_dir;
        end

        function state = makeInitialRlState(app, x_mm, y_mm)
            n_q = app.n_rl_interval;
            dt_q_ms = 1000 / app.wa_Fs;
            t_query = (-(n_q-1):0) * dt_q_ms;

            cc2_state_buffer = zeros(5, n_q);
            cc2_state_buffer(1,:) = t_query;
            cc2_state_buffer(2,:) = x_mm;
            cc2_state_buffer(3,:) = y_mm;

            state = [cc2_state_buffer; zeros(app.n_ch_total - 5, n_q)];
        end

        function meta = makeEpisodeStartMeta(~, path_idx, start_x_mm, object_speed_mm_per_ms, delay_s, rotation_step_deg)
            meta = struct();
            meta.path_index = double(path_idx) - 1; % zero-based for Python list indexing
            meta.front_start_x_mm = double(start_x_mm);
            meta.object_speed_mm_per_ms = double(object_speed_mm_per_ms);
            meta.delay_ms = double(delay_s) * 1000;
            meta.rotation_change_limit_deg_per_control_step = double(rotation_step_deg);
        end

        function [start_s, start_x, start_y, s_grid, x_vals] = findPathStartForCarriage(~, carObj, xp, L, s_grid, x_vals)
            if nargin < 5 || isempty(s_grid)
                s_grid = linspace(0, L, 4001);
            end
            if nargin < 6 || isempty(x_vals)
                x_vals = xp(s_grid);
            end

            try
                origin_x_mm = carObj.origin(1) * carObj.step2mm;
            catch
                origin_x_mm = 0;
            end

            [~, idx_min] = min(abs(x_vals - origin_x_mm));
            start_s = max(0, min(L, s_grid(idx_min)));
            start_x = xp(start_s);
            start_y = [];
            try
                start_y = carObj.path_yp(start_s); %#ok<NASGU>
            catch
            end
        end

        function episodePath = buildEpisodePathContext(app, pd, path_idx, pathtag, source_identifier)
            [xp, yp, rp, thetap, L] = pd.getInterpolants();
            [start_s1, start_x1, ~, s_grid, x_vals] = app.findPathStartForCarriage(app.CC1.Car, xp, L);
            start_y1 = yp(start_s1);
            [start_s2, start_x2, ~] = app.findPathStartForCarriage(app.CC2.Car, xp, L, s_grid, x_vals);
            start_y2 = yp(start_s2);

            episodePath = struct();
            episodePath.pd = pd;
            episodePath.xy = [pd.x(:), pd.y(:)];
            episodePath.xp = xp;
            episodePath.yp = yp;
            episodePath.rp = rp;
            episodePath.thetap = thetap;
            episodePath.L = L;
            episodePath.path_index = path_idx;
            episodePath.pathtag = pathtag;
            episodePath.source_identifier = source_identifier;
            episodePath.start_s1 = start_s1;
            episodePath.start_x1 = start_x1;
            episodePath.start_y1 = start_y1;
            episodePath.start_s2 = start_s2;
            episodePath.start_x2 = start_x2;
            episodePath.start_y2 = start_y2;
        end

        function episodePath = resolvePathAgentPreEpisodePath(app)
            if app.agent_server_use_random_paths
                generator = RandomPathGenerator();
                generator.Seed = double(randi(intmax('int32')));
                [xy, seedPoints, genMeta] = generator.generate();
                pd = PathData(xy);
                pathtag = sprintf('random_ep%03d_seed%d', 1, generator.Seed);
                episodePath = app.buildEpisodePathContext(pd, 1, pathtag, ...
                    sprintf('random seed %d', generator.Seed));
                episodePath.seed_points = seedPoints;
                episodePath.generator_meta = genMeta;
            else
                [pd, fullpath] = app.CC1.ModePanel.createPathDataFromSelection();
                [~, name1, ext1] = fileparts(fullpath);
                pathtag = erase([name1 ext1], 'xy_');
                episodePath = app.buildEpisodePathContext(pd, 1, pathtag, fullpath);
            end
        end

        function episodePath = resolvePathAgentTrainEpisodePath(app, episodeIndex, pathFiles)
            if app.agent_server_use_random_paths
                generator = RandomPathGenerator();
                generator.Seed = double(randi(intmax('int32')));
                [xy, seedPoints, genMeta] = generator.generate();
                pd = PathData(xy);
                pathtag = sprintf('random_ep%03d_seed%d', episodeIndex, generator.Seed);
                episodePath = app.buildEpisodePathContext(pd, episodeIndex, pathtag, ...
                    sprintf('random seed %d', generator.Seed));
                episodePath.seed_points = seedPoints;
                episodePath.generator_meta = genMeta;
            else
                path_idx = randi(numel(pathFiles));
                chosen_file = pathFiles{path_idx};
                pd = PathData(chosen_file);
                [~, fname, fext] = fileparts(chosen_file);
                pathtag = erase([fname fext], 'xy_');
                episodePath = app.buildEpisodePathContext(pd, path_idx, pathtag, chosen_file);
            end
        end

        function updateEpisodePathPlot(app, path_xy, path_label)
            ax = app.CC1.Ax;
            if isempty(ax) || ~isvalid(ax)
                return
            end

            if ~isempty(app.episode_path_plot_handle)
                try
                    if isgraphics(app.episode_path_plot_handle)
                        delete(app.episode_path_plot_handle);
                    end
                catch
                end
            end

            if nargin < 3 || isempty(path_label)
                path_label = 'Episode Path';
            end

            hold(ax, 'on');
            app.episode_path_plot_handle = plot(ax, path_xy(:,1), path_xy(:,2), ...
                'LineWidth', 2.0, ...
                'LineStyle', '--', ...
                'Color', [0.1 0.1 0.1], ...
                'DisplayName', path_label);
            try
                legend(ax, 'show');
            catch
            end
            drawnow limitrate
        end

        function options = discoverPolicyPackages(app)
            agentsDir = fullfile(app.getPythonRootDir(), 'agents');
            if ~isfolder(agentsDir)
                options = {app.agent_policy_package_dir};
                return
            end

            d = dir(agentsDir);
            names = {};
            for i = 1:numel(d)
                if ~d(i).isdir
                    continue
                end
                n = d(i).name;
                if strcmp(n, '.') || strcmp(n, '..')
                    continue
                end
                names{end+1} = ['agents/', n]; %#ok<AGROW>
            end

            if isempty(names)
                options = {app.agent_policy_package_dir};
            else
                options = sort(names);
            end
        end

        function rootDir = getPythonRootDir(~)
            matlabDir = fileparts(mfilename('fullpath'));
            rootDir = char(java.io.File(fullfile(matlabDir, '..', 'python')).getCanonicalPath());
        end

        function [ok,msg] = startAgentServer(app, userConfig)
            ok = false;

            if nargin < 2 || isempty(userConfig)
                userConfig = app.getServerConfig();
            end

            app.agent_server_mode = userConfig.mode;
            app.agent_server_hpc_port = app.agent_server_hpc_port; % hpc_port no longer in UI; kept as property default (5555)
            app.agent_server_max_episodes = userConfig.max_episodes;
            app.agent_server_record_trajectory = logical(userConfig.record_trajectory);
            app.agent_server_use_random_paths = isfield(userConfig, 'use_random_paths') && logical(userConfig.use_random_paths);
            app.agent_policy_package_dir = userConfig.policy_package_dir;

            app.shutdownAgentServer();

            [pidOk, pid, pidMsg] = app.launchServerProcess();
            if ~pidOk
                msg = pidMsg;
                return
            end

            app.agent_server_pid = pid;

            maxAttempts = 8;
            lastMsg = '';
            for i = 1:maxAttempts
                pause(0.35);
                [connected, connectMsg] = app.connect_agent_server(userConfig);
                if connected
                    ok = true;
                    msg = sprintf('Server started (PID %d) and connected.', pid);
                    return
                end
                lastMsg = connectMsg;
            end

            app.shutdownAgentServer();
            msg = sprintf('Started Python server but failed to connect: %s', lastMsg);
        end

        function [ok,msg] = shutdownAgentServer(app)
            ok = true;
            msg = 'Server is shut down.';

            if ~isempty(app.net)
                try
                    app.net.shutdown();
                catch
                end
                app.net = [];
            end

            if ~isempty(app.agent_server_pid)
                pid = app.agent_server_pid;
                [status, out] = system(sprintf('taskkill /PID %d /T /F', pid));
                if status ~= 0
                    if contains(out, 'not found', 'IgnoreCase', true)
                        % Process already exited, treat as success.
                    else
                        ok = false;
                        msg = sprintf('Failed to kill server process %d: %s', pid, strtrim(out));
                    end
                end
                app.agent_server_pid = [];
            end
        end

        function [ok,pid,msg] = launchServerProcess(app)
            ok = false;
            pid = [];

            activateScript = fullfile(getenv('USERPROFILE'), 'py_envs', 'rl', 'Scripts', 'Activate.ps1');
            if ~isfile(activateScript)
                msg = sprintf('Activation script not found: %s', activateScript);
                return
            end

            pyRoot = app.getPythonRootDir();
            scriptName = 'main_server_loop.py';
            if ~isfile(fullfile(pyRoot, scriptName))
                msg = sprintf('Server script not found: %s', fullfile(pyRoot, scriptName));
                return
            end

            activateQ = strrep(activateScript, '''', '''''');
            pyRootQ = strrep(pyRoot, '''', '''''');

            psCmd = [ ...
                '$ErrorActionPreference = ''Stop''; ' ...
                '& ''' activateQ '''; ' ...
                '$p = Start-Process -FilePath ''python.exe'' ' ...
                '-ArgumentList ''-u'',''main_server_loop.py'' ' ...
                '-WorkingDirectory ''' pyRootQ ''' -PassThru; ' ...
                'Write-Output $p.Id'];

            fullCmd = ['powershell -NoProfile -ExecutionPolicy Bypass -Command "' psCmd '"'];
            [status, out] = system(fullCmd);
            if status ~= 0
                msg = sprintf('Failed to launch server process: %s', strtrim(out));
                return
            end

            pid = str2double(strtrim(out));
            if isnan(pid)
                tokens = regexp(out, '\d+', 'match');
                if ~isempty(tokens)
                    pid = str2double(tokens{end});
                end
            end

            if isnan(pid) || pid <= 0
                msg = sprintf('Could not parse server PID from output: %s', strtrim(out));
                return
            end

            ok = true;
            msg = sprintf('Started server process with PID %d.', pid);
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

            % Disconnect and stop agent server
            try
                app.shutdownAgentServer();
            catch
            end

            try
                if ~isempty(app.server_config_window) && isvalid(app.server_config_window)
                    if isvalid(app.server_config_window.UIFigure)
                        delete(app.server_config_window.UIFigure);
                    end
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
                [v1,v2,delay_s,run_tag,episode_time_s,settle_delay_s] = app.ExpPanel.getParameters();
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

            % start(app.pathpathTimer);

            % blocking path tracking loop
            period = app.pathpath_tick_period_ms/1000; % seconds
            app.CC1.hArrow.Visible = 'on';
            app.CC2.hArrow.Visible = 'on';
            
            pause(settle_delay_s); % allow time for water to settle after carriage movement before starting
            % prepare daq
            try
                if ~isempty(app.WA.s)
                    % app.WA.is_recording = true;
                    % app.WA.init_datalog_file();
                    app.WA.reset_data_buffers();
                    app.WA.align_data_read();
                    app.WA.average_signal_as_offset(round(app.WA.Fs));
                    app.WA.tag = sprintf('%s_%s-v1=%.2f_%s-v2=%.2f_delay=%.1f',run_tag,pathtag1,v1,pathtag2,v2,delay_s);
                end
            catch
            end
            % if ~isempty(app.WA.s)
            %     app.WA.tag = sprintf('%s_%s-v1=%.2f_%s-v2=%.2f_delay=%.1f',run_tag,pathtag1,v1,pathtag2,v2,delay_s);
            %     app.WA.align_data_read(); % clear samples during carriage movement before starting
            % end

            n = 0;
            app.run_start_time = datetime('now');
            t0 = tic;

            % Viz episode start
            if ~isempty(app.net)
                try
                    if ~app.net.sendVizStart(app.buildVizFrame())
                        app.net = [];
                    end
                catch
                    app.net = [];
                end
            end

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

                % stop condition: episode time exceeded
                elapsed = toc(t0);
                if elapsed > episode_time_s
                    fprintf('Episode time %.1f s exceeded, stopping.\n', elapsed);
                    break;
                end

                % schedule next tick using tic/toc to reduce drift
                next_time = n * period;

                fprintf('PathPath Tick %d: elapsed total=%.3f s, frame=%.1f ms, avg FPS=%.1f\n',...
                 n, elapsed, toc(t1)*1000, n/elapsed);
                
                % busy-wait the final few ms to reduce jitter
                while toc(t0) < next_time
                    pause(0.0005); % yield briefly to keep UI responsive, NOTE pause <=1ms triggers higher resolution OS timer.
                end
            end

            % Viz episode end
            if ~isempty(app.net)
                try
                    app.net.sendVizEnd();
                catch
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
            app.WA.write_buffer_to_file(app.run_start_time, datetime('now'), app.WA.tag);
            app.WA.close_datafile();
        end

        % additional mode handlers (placeholders)
        function onPathHuman(app, ~, ~)
            % PathHuman: CC1 follows a prescribed path, CC2 is human-controlled
            % Both run in the same blocking loop to avoid timer jitter
            
            try
                [v1,v2,delay_s,run_tag,episode_time_s,settle_delay_s] = app.ExpPanel.getParameters();
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

            pause(settle_delay_s); % allow time for water to settle
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

            % Viz episode start
            if ~isempty(app.net)
                try
                    if ~app.net.sendVizStart(app.buildVizFrame())
                        app.net = [];
                    end
                catch
                    app.net = [];
                end
            end

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
                new_samples_ph = [];
                try
                    if ~isempty(app.WA.s)
                        if ~isempty(app.net)
                            new_samples_ph = app.WA.rl_read_update_tick();
                        else
                            app.WA.read_update_tick();
                        end
                    end
                catch
                end
                if ~isempty(app.net) && ~isempty(new_samples_ph)
                    viz_state = app.buildVizFrame(new_samples_ph);
                    if ~app.net.sendVizFrame(viz_state)
                        app.net = [];
                    end
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

            % Viz episode end
            if ~isempty(app.net)
                try
                    app.net.sendVizEnd();
                catch
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
            modeCleanup = onCleanup(@() agentPathModeCleanup(app)); %#ok<NASGU>

            if isempty(app.net)
                warning('AgentConnectionError:NotConnected', ...
                    'Agent server is not connected. Use Config server -> Start first.');
                return
            end
            
            try
                [v1,v2,delay_s,run_tag,episode_time_s,settle_delay_s,rotation_step_deg] = app.ExpPanel.getParameters();
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
                app.WA.ns_read = app.n_rl_interval; % set number of samples to read per tick to match agent action update interval
                app.WA.n_update = 2;
                app.WA.n_fill = 1;
                app.WA.align_data_read();
                app.WA.average_signal_as_offset(round(app.WA.Fs));
                app.WA.reset_data_buffers();
            catch e
                error('DataAcquisitionError:SetupFailed', 'Error during data acquisition setup: %s', e.message);
            end

            % Resolve the episode path, update the plot, and prepare both carriages.
            try
                episodePath = app.resolvePathAgentPreEpisodePath();
            catch ME
                warning('PathAgentPre:PathSetupFailed', ...
                    'Failed to prepare the episode path: %s', ME.message);
                return
            end

            xp = episodePath.xp;
            yp = episodePath.yp;
            rp = episodePath.rp;
            thetap1 = episodePath.thetap;
            L = episodePath.L;
            start_s = episodePath.start_s1;
            start_x = episodePath.start_x1;
            start_y = episodePath.start_y1;
            start_s2 = episodePath.start_s2;
            start_x2 = episodePath.start_x2;
            start_y2 = episodePath.start_y2;
            pathtag1 = episodePath.pathtag;

            app.updateEpisodePathPlot(episodePath.xy, sprintf('Episode Path (%s)', pathtag1));

            % prepare CC2 for agent control
            app.CC2.Car.poll_gamepad = 0;
            app.CC2.Car.poll_keyboard = 1; % for interupting the agent control
            
            try
                start(app.CC2.redrawTimer);
            catch
            end
            app.CC2.Car.moveToPositionMM(start_x2-app.CC2.Car.origin_mm(1), start_y2-app.CC2.Car.origin_mm(2), 20, 1, false); % move to path start
            stop(app.CC2.redrawTimer);

            try
                start(app.CC1.redrawTimer);
            catch
            end
            app.CC1.Car.moveToPositionMM(start_x-app.CC1.Car.origin_mm(1), start_y-app.CC1.Car.origin_mm(2), 20, 1, false);
            app.CC1.Car.init_pathtracking_variables(xp,yp,rp,L,start_s,thetap1);
            app.CC2.Car.init_pathtracking_variables(xp,yp,rp,L,start_s2,thetap1);
            stop(app.CC1.redrawTimer);
            
            % blocking path+agent control loop
            period = 1/app.wa_Fs; % seconds, control loop period for agent control (80 Hz)
            app.CC1.hArrow.Visible = 'on';
            app.CC2.hArrow.Visible = 'on';
            app.CC2.Car.cmd_npoll = 0; % reset poll counter
            
            is_done_cc1 = false; % track when CC1 finishes path tracking
            is_done = 0; % track when episode is done for agent (currently only time-based truncation, no early termination condition)
            truncated = 0;
            connection_lost = false;
            app.currentState = app.makeInitialRlState(start_x2, start_y2);
            try
                action = app.net.startEpisode( ...
                    app.currentState(:)', ...
                    app.makeEpisodeStartMeta(episodePath.path_index, start_x, v1, delay_s, rotation_step_deg), ...
                    episodePath.xy); % get initial action from agent
            catch ME
                handleAgentConnectionFailure(app, 'PathAgentPre startEpisode', ME);
                return
            end
            
            n = 0;
            num_agent_interactions = 0;
            pause(settle_delay_s); % allow some time for water to calm down from carriages moving to start positions.
            try
                app.WA.tag = sprintf('%s_PathAgentPre-%s_front=%.2f_rlvx=%.2f_rotstep=%.1f_delay=%.1f', ...
                    run_tag, pathtag1, v1, v2, rotation_step_deg, delay_s);
                app.WA.align_data_read(); % clear samples during carriage movement before starting
            catch

            end
            app.run_start_time = datetime('now');
            t0 = tic;
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
                
                % CC2: path tracking during delay, then agent control
                if elapsed_time_sec > delay_s
                    vx_in = round(action(1)*1000/app.CC2.Car.step2mm);
                    vy_in = round(action(2)*1000/app.CC2.Car.step2mm);
                    try
                        app.CC2.Car.agentControlStep(ev, vx_in, vy_in);
                    catch ME
                        warning(ME.identifier, '%s', ME.message);
                    end
                else
                    try
                        app.CC2.Car.pathTrackingTick(src, ev);
                    catch ME
                        warning(ME.identifier, '%s', ME.message);
                    end
                end
                reward = 0; % placeholder reward

                % data acquisition
                new_samples = [];
                try
                    if ~isempty(app.WA.s)
                        new_samples = app.WA.rl_read_update_tick(); % offset subtracted
                    end
                catch ME
                    % error('DataAcquisitionError', 'Error during data acquisition: %s', ME.message);
                end

                if elapsed_time_sec > delay_s && ~isempty(new_samples) % only get new action when there are enough data samples
                    num_agent_interactions = num_agent_interactions + 1;
                    % construct state for agent

                    % Interpolate carriage state from built-in rolling buffer using
                    % controller-relative time: latest sample at 0 ms, older samples negative.
                    n_q = app.n_rl_interval;
                    dt_q_ms = 1000 / app.wa_Fs;
                    t_query = (-(n_q-1):0) * dt_q_ms; % ms, oldest -> newest

                    sb = app.CC2.Car.state_buffer;
                    n_valid = min(app.CC2.Car.state_buffer_count, size(sb,1));
                    cc2_state_buffer(1,:) = t_query + num_agent_interactions * app.n_rl_interval * dt_q_ms; % controller-relative time for each sample, accounting for multiple agent interactions
                    if n_valid <= 0
                        cc2_state_buffer(2:5,:) = zeros(4, n_q);
                    else
                        sb_valid = sb(end-n_valid+1:end,:); % [time, x, y, vx, vy], oldest -> newest
                        t_rel = sb_valid(:,1) - sb_valid(end,1); % newest is 0 ms

                        % Remove duplicate timestamps to keep interp1 well-defined.
                        [t_rel_u, iu] = unique(t_rel, 'stable');
                        state_u = sb_valid(iu, 2:5);

                        if isscalar(t_rel_u)
                            % Single sample: nearest hold for all query points.
                            cc2_state_buffer(2:5,:) = repmat(state_u(1,:)', 1, n_q);
                        else
                            % Linear interpolation inside range only (no extrapolation).
                            state_q = interp1(t_rel_u, state_u, t_query, 'linear', 0); % n_q x 4
                            cc2_state_buffer(2:5,:) = state_q';
                        end
                    end

                    bending_moments = app.signal_to_bending_moment.convertSamples( ...
                        new_samples, reorderToSimulation=true);

                    app.currentState = [cc2_state_buffer; bending_moments']; % 5 rows of carriage state + 18 rows of daq data, each with n_rl_interval columns
                    % send state to agent and receive action
                    try
                        action = app.net.stepRL(app.currentState(:)', reward, is_done, truncated); % flatten state to 1D array for sending to agent
                    catch ME
                        handleAgentConnectionFailure(app, sprintf('PathAgentPre step %d', num_agent_interactions), ME);
                        connection_lost = true;
                        break;
                    end
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

            app.WA.is_recording = false;
            try
                app.WA.close_datafile();
            catch
            end
            try
                app.WA.write_buffer_to_file(app.run_start_time, datetime('now'), app.WA.tag);
            catch
            end

            % Signal episode end to the agent server and wait for sync
            if ~connection_lost
                try
                    if ~isempty(app.net)
                        app.net.syncWithHPC();
                    end
                catch ME
                    handleAgentConnectionFailure(app, 'PathAgentPre syncWithHPC', ME);
                end
            end
        end

        function onPathAgentLive(~, ~, ~)
            disp('PathAgentLive triggered (placeholder)');
        end

        function onPathAgentTrain(app, ~, ~)
            % PathAgentTrain: CC1 follows a randomly selected path each episode,
            % CC2 is controlled by a SAC agent that trains locally between episodes.
            modeCleanup = onCleanup(@() agentPathModeCleanup(app)); %#ok<NASGU>

            if isempty(app.net)
                warning('AgentConnectionError:NotConnected', ...
                    'Agent server is not connected. Use Config server -> Start first.');
                return
            end

            try
                [v1,v2,delay_s,run_tag,episode_time_s,settle_delay_s,rotation_step_deg] = app.ExpPanel.getParameters();
            catch
                warning('Failed to read experiment parameters.');
                return
            end

            pathFiles = app.CC1.ModePanel.SelectedFiles;
            if ~app.agent_server_use_random_paths && isempty(pathFiles)
                warning('PathAgentTrain:NoPaths', ...
                    'No path files loaded in CC1 panel. Load at least one path before training.');
                return
            end

            max_episodes = app.agent_server_max_episodes;

            % set velocity limits for both carriages
            app.CC1.Car.vel_max = round(v1*1000/app.CC1.Car.step2mm);
            app.CC2.Car.vel_max = round(v2*1000/app.CC2.Car.step2mm);

            % DAQ setup (same as PathAgentPre)
            try
                app.WA.ns_read = app.n_rl_interval;
                app.WA.n_update = 2;
                app.WA.n_fill = 1;
                app.WA.align_data_read();
                app.WA.average_signal_as_offset(round(app.WA.Fs));
                app.WA.reset_data_buffers();
            catch e
                error('DataAcquisitionError:SetupFailed', 'Error during data acquisition setup: %s', e.message);
            end

            app.CC2.Car.poll_gamepad = 0;
            app.CC2.Car.poll_keyboard = 1;

            period = 1/app.wa_Fs;
            connection_lost = false;

            for ep = 1:max_episodes
                fprintf('\n[PathAgentTrain] === Episode %d / %d ===\n', ep, max_episodes);

                try
                    episodePath = app.resolvePathAgentTrainEpisodePath(ep, pathFiles);
                catch ME
                    warning('PathAgentTrain:PathSetupFailed', ...
                        'Failed to prepare the episode path: %s', ME.message);
                    break;
                end

                pd = episodePath.pd;
                xp_interp = episodePath.xp;
                yp_interp = episodePath.yp;
                rp_interp = episodePath.rp;
                thetap1 = episodePath.thetap;
                L = episodePath.L;
                path_idx = episodePath.path_index;
                pathtag1 = episodePath.pathtag;
                start_s = episodePath.start_s1;
                start_x = episodePath.start_x1;
                start_y = episodePath.start_y1;
                start_s2 = episodePath.start_s2;
                start_x2 = episodePath.start_x2;
                start_y2 = episodePath.start_y2;

                app.updateEpisodePathPlot(episodePath.xy, sprintf('Episode Path Ep %03d', ep));
                fprintf('[PathAgentTrain] Path source: %s\n', episodePath.source_identifier);

                % --- Move CC2 first, then CC1 (back carriage always first) ---
                try; start(app.CC2.redrawTimer); catch; end
                app.CC2.Car.moveToPositionMM(start_x2 - app.CC2.Car.origin_mm(1), start_y2 - app.CC2.Car.origin_mm(2), 20, 1, false);
                try; stop(app.CC2.redrawTimer); catch; end

                try; start(app.CC1.redrawTimer); catch; end
                app.CC1.Car.moveToPositionMM(start_x - app.CC1.Car.origin_mm(1), ...
                    start_y - app.CC1.Car.origin_mm(2), 20, 1, false);
                app.CC1.Car.init_pathtracking_variables(xp_interp, yp_interp, rp_interp, L, start_s, thetap1);
                app.CC2.Car.init_pathtracking_variables(xp_interp, yp_interp, rp_interp, L, start_s2, thetap1);
                try; stop(app.CC1.redrawTimer); catch; end

                % Settle pause
                pause(settle_delay_s);

                % DAQ: align after carriage movement
                try
                    app.WA.tag = sprintf('%s_PathAgentTrain-%s_front=%.2f_rlvx=%.2f_rotstep=%.1f_delay=%.1f_ep%03d', ...
                        run_tag, pathtag1, v1, v2, rotation_step_deg, delay_s, ep);
                    app.WA.align_data_read();
                catch
                end

                app.currentState = app.makeInitialRlState(start_x2, start_y2);
                cc2_state_buffer = app.currentState(1:5,:);
                is_done_cc1 = false;
                cc1_complete_logged = false;
                is_done = 0;
                truncated = 0;
                terminal_sent = false;
                num_agent_interactions = 0;
                reward = 0;

                app.run_start_time = datetime('now');
                try
                    action = app.net.startEpisode( ...
                        app.currentState(:)', ...
                        app.makeEpisodeStartMeta(path_idx, start_x, v1, delay_s, rotation_step_deg), ...
                        episodePath.xy);
                catch ME
                    handleAgentConnectionFailure(app, sprintf('PathAgentTrain episode %d startEpisode', ep), ME);
                    connection_lost = true;
                    break;
                end

                app.CC1.hArrow.Visible = 'on';
                app.CC2.hArrow.Visible = 'on';
                app.CC2.Car.cmd_npoll = 0;

                n = 0;
                t0 = tic;

                while true
                    t1 = tic;
                    n = n + 1;

                    src = struct(); src.TasksExecuted = n;
                    ev  = struct(); ev.Data.time = datetime('now');

                    % CC1: path tracking
                    if ~is_done_cc1
                        try
                            is_done_cc1 = app.CC1.Car.pathTrackingTick(src, ev);
                            if is_done_cc1 && ~cc1_complete_logged
                                fprintf('[PathAgentTrain] Front path complete; continuing back-carriage control.\n');
                                cc1_complete_logged = true;
                            end
                        catch ME
                            warning(ME.identifier, '%s', ME.message);
                            is_done_cc1 = true;
                            if ~cc1_complete_logged
                                fprintf('[PathAgentTrain] Front path tracking stopped unexpectedly; continuing back-carriage control.\n');
                                cc1_complete_logged = true;
                            end
                        end
                    end

                    now_dt = datetime('now');
                    elapsed_time_sec = seconds(now_dt - app.run_start_time);

                    if elapsed_time_sec > episode_time_s
                        truncated = 1;
                    end

                    % CC2: path tracking during delay, then agent control
                    if elapsed_time_sec > delay_s
                        vx_in = round(action(1)*1000/app.CC2.Car.step2mm);
                        vy_in = round(action(2)*1000/app.CC2.Car.step2mm);
                        try
                            app.CC2.Car.agentControlStep(ev, vx_in, vy_in);
                        catch ME
                            warning(ME.identifier, '%s', ME.message);
                        end
                    else
                        try
                            app.CC2.Car.pathTrackingTick(src, ev);
                        catch ME
                            warning(ME.identifier, '%s', ME.message);
                        end
                    end

                    % DAQ
                    new_samples = [];
                    try
                        if ~isempty(app.WA.s)
                            new_samples = app.WA.rl_read_update_tick();
                        end
                    catch
                    end

                    if elapsed_time_sec > delay_s && ~isempty(new_samples)
                        num_agent_interactions = num_agent_interactions + 1;

                        % Build CC2 kinematic state (same interpolation as PathAgentPre)
                        n_q = app.n_rl_interval;
                        dt_q_ms = 1000 / app.wa_Fs;
                        t_query = (-(n_q-1):0) * dt_q_ms;

                        sb = app.CC2.Car.state_buffer;
                        n_valid = min(app.CC2.Car.state_buffer_count, size(sb,1));
                        cc2_state_buffer(1,:) = t_query + num_agent_interactions * n_q * dt_q_ms;
                        if n_valid <= 0
                            cc2_state_buffer(2:5,:) = zeros(4, n_q);
                        else
                            sb_valid = sb(end-n_valid+1:end,:);
                            t_rel = sb_valid(:,1) - sb_valid(end,1);
                            [t_rel_u, iu] = unique(t_rel, 'stable');
                            state_u = sb_valid(iu, 2:5);
                            if isscalar(t_rel_u)
                                cc2_state_buffer(2:5,:) = repmat(state_u(1,:)', 1, n_q);
                            else
                                state_q = interp1(t_rel_u, state_u, t_query, 'linear', 0);
                                cc2_state_buffer(2:5,:) = state_q';
                            end
                        end

                        % Compute reward from lateral error of CC2 vs CC1's current path.
                        % Use the measured front-carriage position when available so
                        % random/curved paths do not rely on a straight-line x estimate.
                        x2 = cc2_state_buffer(2, end);  % latest CC2 x position (mm)
                        y2 = cc2_state_buffer(3, end);  % latest CC2 y position (mm)
                        front_x_mm = NaN;
                        sb1 = app.CC1.Car.state_buffer;
                        n_valid1 = min(app.CC1.Car.state_buffer_count, size(sb1,1));
                        if n_valid1 > 0
                            sb1_valid = sb1(end-n_valid1+1:end,:);
                            valid_rows1 = all(isfinite(sb1_valid(:,1:3)), 2);
                            if any(valid_rows1)
                                front_x_mm = sb1_valid(find(valid_rows1, 1, 'last'), 2);
                            end
                        end
                        if ~isfinite(front_x_mm)
                            front_x_mm = app.CC1.Car.real_loc(1);
                        end
                        if ~isfinite(front_x_mm)
                            front_x_mm = start_x + elapsed_time_sec*1000*v1;
                        end
                        x2_gap = front_x_mm - x2;

                        % Closest point on discrete path
                        dists_sq = (pd.x - x2).^2 + (pd.y - y2).^2;
                        [~, kp] = min(dists_sq);
                        k_prev = max(kp-1, 1);
                        k_next = min(kp+1, numel(pd.x));
                        tx = (pd.x(k_next) - pd.x(k_prev));
                        ty = (pd.y(k_next) - pd.y(k_prev));
                        tnorm = sqrt(tx^2 + ty^2);
                        if tnorm > 0
                            tx = tx/tnorm; ty = ty/tnorm;
                        end
                        % Signed lateral error: positive = left of path direction
                        signed_err = (x2 - pd.x(kp))*(-ty) + (y2 - pd.y(kp))*(tx);

                        reward_corridor = 180;   % mm
                        terminate_corridor = 240; % mm
                        min_gap_mm = 25;          % mm
                        finish_line_mm = 3800;    % mm

                        reward = max(-1.0, min(1.0, 1.0 - abs(signed_err)/reward_corridor));

                        if abs(signed_err) > terminate_corridor
                            reward = reward - 2.0;
                            is_done = 1;
                        end
                        if x2_gap < min_gap_mm
                            reward = reward - 2.0;
                            is_done = 1;
                        end
                        y_boundary_hit = false;
                        boundary_margin = app.CC2.Car.boundary_margin_mm;
                        if isfinite(app.CC2.Car.y_min_mm)
                            y_boundary_hit = y_boundary_hit || (y2 <= app.CC2.Car.y_min_mm + boundary_margin);
                        end
                        if isfinite(app.CC2.Car.y_max_mm)
                            y_boundary_hit = y_boundary_hit || (y2 >= app.CC2.Car.y_max_mm - boundary_margin);
                        end
                        if y_boundary_hit
                            reward = reward - 2.0;
                            is_done = 1;
                            try
                                app.CC2.Car.sendStopCommand();
                            catch
                            end
                        end
                        if x2 >= finish_line_mm
                            is_done = 1;
                        end

                        % Build full state and send to agent
                        bending_moments = app.signal_to_bending_moment.convertSamples( ...
                            new_samples, reorderToSimulation=true);
                        app.currentState = [cc2_state_buffer; bending_moments'];
                        fprintf('Episode %d, Step %d, geting action ...', ...
                            ep, num_agent_interactions);
                        episode_done_flag = logical(is_done);
                        prev_action = action;
                        try
                            action = app.net.stepRL(app.currentState(:)', reward, episode_done_flag, truncated);
                        catch ME
                            handleAgentConnectionFailure(app, ...
                                sprintf('PathAgentTrain episode %d step %d', ep, num_agent_interactions), ME);
                            connection_lost = true;
                            break;
                        end
                        if truncated || episode_done_flag
                            terminal_sent = true;
                        end
                        norm_action_str = 'n/a';
                        max_step_rad = deg2rad(max(0.0, rotation_step_deg));
                        if ~terminal_sent && max_step_rad > 0 && numel(prev_action) >= 2 && numel(action) >= 2 ...
                                && abs(prev_action(1)) > eps && abs(action(1)) > eps
                            prev_theta = atan2(prev_action(2), prev_action(1));
                            curr_theta = atan2(action(2), action(1));
                            applied_norm_action = max(-1.0, min(1.0, (curr_theta - prev_theta) / max_step_rad));
                            norm_action_str = sprintf('%.3f', applied_norm_action);
                        end
                        fprintf('done. action=[%.3f, %.3f], norm_action=%s\n', ...
                            action(1), action(2), norm_action_str);
                    end

                    if connection_lost
                        break;
                    end

                    % Visuals
                    if mod(n, app.pathpath_redraw_interval) == 0
                        app.CC1.update_view();
                        app.CC2.update_view();
                    end

                    % Timing
                    next_time = n * period;
                    elapsed = toc(t0);
                    fprintf('PathAgentTrain ep%d Tick %d: total=%.3f s, frame=%.1f ms, avg FPS=%.1f\n', ...
                        ep, n, elapsed, toc(t1)*1000, n/elapsed);
                    while toc(t0) < next_time
                        pause(0.0005);
                    end

                    if (truncated || is_done) && terminal_sent
                        if truncated
                            fprintf('[PathAgentTrain] Episode truncated after %.2f s\n', elapsed_time_sec);
                        else
                            fprintf('[PathAgentTrain] Episode ended (is_done) after %.2f s\n', elapsed_time_sec);
                        end
                        break;
                    end
                end % tick loop

                % --- Episode cleanup ---
                try; app.CC1.Car.stopPathTracking(); catch; end
                app.CC1.hArrow.Visible = 'off';
                app.CC2.hArrow.Visible = 'off';
                try
                    app.WA.write_buffer_to_file(app.run_start_time, datetime('now'), app.WA.tag);
                    app.WA.close_datafile();
                catch
                end

                if connection_lost
                    break;
                end

                if ep == max_episodes
                    try
                        if ~isempty(app.net)
                            app.net.syncWithHPC();
                        end
                    catch ME
                        handleAgentConnectionFailure(app, sprintf('PathAgentTrain episode %d syncWithHPC', ep), ME);
                        connection_lost = true;
                    end
                end

                if connection_lost
                    break;
                end

                if ep < max_episodes
                    % CC2 (back carriage): reset angle to 0, then start moving non-blocking
                    try 
                        for i=1:3
                            app.CC2.Car.sendCommand('NUL,NUL,ABS0>');
                            pause(0.01);
                        end
                    catch
                    end
                    pause(0.2);
                    try start(app.CC2.redrawTimer); catch; end
                    app.CC2.Car.moveToPositionMM(start_x2 - app.CC2.Car.origin_mm(1), start_y2 - app.CC2.Car.origin_mm(2), 20, 0, true);
                    try stop(app.CC2.redrawTimer); catch; end

                    % CC1 (front carriage): start after 2 s without waiting for CC2 to finish
                    pause(2.0);
                    try start(app.CC1.redrawTimer); catch; end
                    app.CC1.Car.moveToPositionMM(start_x - app.CC1.Car.origin_mm(1), ...
                        start_y - app.CC1.Car.origin_mm(2), 20, 1, true);
                    try stop(app.CC1.redrawTimer); catch; end

                    % Sync with agent server while the carriages are returning.
                    reset_needed = false;
                    try
                        if ~isempty(app.net)
                            reset_needed = app.net.syncWithHPC();
                        end
                    catch ME
                        handleAgentConnectionFailure(app, sprintf('PathAgentTrain episode %d inter-episode sync', ep), ME);
                        connection_lost = true;
                        break;
                    end
                    if reset_needed
                        fprintf('[PathAgentTrain] Hardware reset requested.\n');
                    end
                end

            end % episode loop

            % --- Final cleanup ---
            app.WA.is_recording = false;

            % Send shutdown signal to agent server
            try
                if ~isempty(app.net)
                    app.net.shutdown();
                    app.net = [];
                    app.agent_server_pid = [];
                end
            catch ME
                warning('AgentTrainShutdown:Failed', 'Failed to shut down agent server: %s', ME.message);
            end

            fprintf('[PathAgentTrain] Training complete (%d episodes).\n', max_episodes);
        end

        function onFilterConfigRequested(app, ~, ~)
            [filterSpec, wasCancelled] = app.promptFilterConfig();
            if wasCancelled
                return;
            end

            try
                app.WA.set_filter(filterSpec);
                if isempty(filterSpec)
                    fprintf('Wavi filter disabled (type=none).\n');
                else
                    fprintf('Wavi filter updated: %s\n', char(string(filterSpec.filterType)));
                end
            catch me
                warning('FilterConfig:ApplyFailed', 'Failed to apply filter configuration: %s', me.message);
            end
        end

        function onServerConfigRequested(app, ~, ~)
            cfg = app.getServerConfig();
            defaults = cfg;
            defaults.python_root = app.getPythonRootDir();
            defaults.policy_options = app.discoverPolicyPackages();

            try
                if isempty(app.server_config_window) || ~isvalid(app.server_config_window) || ~isvalid(app.server_config_window.UIFigure)
                    app.server_config_window = ServerConfigWindow( ...
                        @(newCfg) app.startAgentServer(newCfg), ...
                        @() app.shutdownAgentServer(), ...
                        defaults);
                else
                    app.server_config_window.UIFigure.Visible = 'on';
                end

                app.server_config_window.setServerRunning(~isempty(app.net), app.serverStatusMessage());
            catch me
                warning('ServerConfig:OpenFailed', 'Failed to open server config window: %s', me.message);
            end
        end

        function msg = serverStatusMessage(app)
            if ~isempty(app.net)
                if ~isempty(app.agent_server_pid)
                    msg = sprintf('Server connected. PID %d.', app.agent_server_pid);
                else
                    msg = 'Server connected.';
                end
            else
                msg = 'Server is not running.';
            end
        end

        function [filterSpec, wasCancelled] = promptFilterConfig(app)
            filterSpec = [];

            defaults = struct('filterType','lowpass-iir',...
                              'order',3,...
                              'cutoffHz',2,...
                              'highpassHz',0.01,...
                              'gain',1.2);
            try
                if ~isempty(app.WA) && ~isempty(app.WA.sig_filter)
                    sf = app.WA.sig_filter;
                    defaults.filterType = char(string(sf.filterType));
                    defaults.order = sf.order;
                    defaults.cutoffHz = sf.cutoffHz;
                    defaults.highpassHz = sf.highpassHz;
                    defaults.gain = sf.gain;
                end
            catch
            end

            dlg = uifigure('Name','Filter Config',...
                           'Position',[360 220 360 260],...
                           'WindowStyle','modal');
            gl = uigridlayout(dlg,[7 2]);
            gl.RowHeight = {'fit','fit','fit','fit','fit','1x','fit'};
            gl.ColumnWidth = {'1x','1x'};

            lblType = uilabel(gl,'Text','Type');
            lblType.Layout.Row = 1;
            ddType = uidropdown(gl,'Items',{'none','lowpass-iir','highpass-iir','moving-average','biquad-lowpass'},...
                                   'Value',char(string(defaults.filterType)));
            ddType.Layout.Row = 1;
            ddType.Layout.Column = 2;

            lblOrder = uilabel(gl,'Text','Order');
            lblOrder.Layout.Row = 2;
            efOrder = uieditfield(gl,'numeric','Value',double(defaults.order));
            efOrder.Layout.Row = 2;
            efOrder.Layout.Column = 2;

            lblCutoff = uilabel(gl,'Text','Cutoff Hz');
            lblCutoff.Layout.Row = 3;
            efCutoff = uieditfield(gl,'numeric','Value',double(defaults.cutoffHz));
            efCutoff.Layout.Row = 3;
            efCutoff.Layout.Column = 2;

            lblHighpass = uilabel(gl,'Text','Highpass Hz');
            lblHighpass.Layout.Row = 4;
            efHighpass = uieditfield(gl,'numeric','Value',double(defaults.highpassHz));
            efHighpass.Layout.Row = 4;
            efHighpass.Layout.Column = 2;

            lblGain = uilabel(gl,'Text','Gain');
            lblGain.Layout.Row = 5;
            efGain = uieditfield(gl,'numeric','Value',double(defaults.gain));
            efGain.Layout.Row = 5;
            efGain.Layout.Column = 2;

            btnGl = uigridlayout(gl,[1 2]);
            btnGl.RowHeight = {'fit'};
            btnGl.ColumnWidth = {'1x','1x'};
            btnGl.Layout.Row = 7;
            btnGl.Layout.Column = [1 2];

            uibutton(btnGl,'Text','Cancel','ButtonPushedFcn',@(~,~) onCancel());
            uibutton(btnGl,'Text','Apply','ButtonPushedFcn',@(~,~) onApply());

            ddType.ValueChangedFcn = @(~,~) updateFieldEnableState();
            dlg.CloseRequestFcn = @(~,~) onCancel();
            updateFieldEnableState();

            uiwait(dlg);

            if isvalid(dlg)
                ud = dlg.UserData;
                delete(dlg);
            else
                ud = struct('action','cancel');
            end

            if ~isstruct(ud) || ~isfield(ud,'action') || ~strcmp(ud.action,'apply')
                wasCancelled = true;
                return;
            end

            wasCancelled = false;
            if strcmp(ud.type, 'none')
                filterSpec = [];
            else
                filterSpec = struct('filterType',ud.type,...
                                    'fs',app.wa_Fs,...
                                    'order',ud.order,...
                                    'cutoffHz',ud.cutoffHz,...
                                    'highpassHz',ud.highpassHz,...
                                    'gain',ud.gain,...
                                    'nChannels',app.WA.nch);
            end

            function updateFieldEnableState()
                enabled = 'on';
                if strcmp(ddType.Value, 'none')
                    enabled = 'off';
                end
                efOrder.Enable = enabled;
                efCutoff.Enable = enabled;
                efHighpass.Enable = enabled;
                efGain.Enable = enabled;
            end

            function onApply()
                dlg.UserData = struct('action','apply',...
                                      'type',char(string(ddType.Value)),...
                                      'order',max(1, round(double(efOrder.Value))),...
                                      'cutoffHz',double(efCutoff.Value),...
                                      'highpassHz',double(efHighpass.Value),...
                                      'gain',double(efGain.Value));
                uiresume(dlg);
            end

            function onCancel()
                dlg.UserData = struct('action','cancel');
                uiresume(dlg);
            end
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

            new_samples_pp = [];
            try
                if ~isempty(app.WA.s)
                    if ~isempty(app.net)
                        new_samples_pp = app.WA.rl_read_update_tick();
                    else
                        app.WA.read_update_tick();
                    end
                end
            catch
            end
            if ~isempty(app.net) && ~isempty(new_samples_pp)
                viz_state = app.buildVizFrame(new_samples_pp);
                if ~app.net.sendVizFrame(viz_state)
                    app.net = [];
                end
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
                    % fclose(app.WA.fout);
                catch
                end
            end
        end

        function frame = buildVizFrame(app, new_samples)
            % Build an n_rl_interval-frame state in the same layout as PathAgentPre/Train.
            % new_samples is optional (omit or pass [] for episode-start frames).
            n_q      = app.n_rl_interval;
            dt_q_ms  = 1000 / app.wa_Fs;
            t_query  = (-(n_q-1):0) * dt_q_ms; % ms offsets, oldest→newest

            % Episode-relative elapsed time in ms (consistent across all modes)
            if ~isempty(app.run_start_time)
                elapsed_ms = seconds(datetime('now') - app.run_start_time) * 1000;
            else
                elapsed_ms = 0;
            end

            cc2_state_buf      = zeros(5, n_q);
            cc2_state_buf(1,:) = elapsed_ms + t_query;

            sb      = app.CC2.Car.state_buffer;
            n_valid = min(app.CC2.Car.state_buffer_count, size(sb, 1));
            if n_valid > 0
                sb_valid = sb(end-n_valid+1:end, :);
                t_rel    = sb_valid(:,1) - sb_valid(end,1);
                [t_rel_u, iu] = unique(t_rel, 'stable');
                state_u = sb_valid(iu, 2:5);
                if isscalar(t_rel_u)
                    cc2_state_buf(2:5,:) = repmat(state_u(1,:)', 1, n_q);
                else
                    state_q = interp1(t_rel_u, state_u, t_query, 'linear', 0);
                    cc2_state_buf(2:5,:) = state_q';
                end
            end

            if nargin >= 2 && ~isempty(new_samples)
                bending_moments = app.signal_to_bending_moment.convertSamples( ...
                    new_samples, reorderToSimulation=true)'; % convertSamples returns n_q×18; transpose to 18×n_q
            else
                bending_moments = zeros(app.n_ch_total - 5, n_q);
            end

            % n_ch_total × n_rl_interval; flatten column-major to match currentState(:)' in agent modes.
            state_mat = [cc2_state_buf; bending_moments];
            frame = state_mat(:)';
        end
    end
end
