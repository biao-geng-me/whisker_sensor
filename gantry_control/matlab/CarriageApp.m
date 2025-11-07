% Carriage.m
% This UI component controls a motorized carriage and integrates the ControlModePanel.
% Optional parent allows testing in isolation or embedding in the main app.

classdef CarriageApp < handle
    properties % UI elements
        ui_parent
        isStandAlone 
        Fig
        gl
        MainPanelWidth = 360;
        MainPanelHeight = 480;

        % Subcomponents
        ModePanel   % ControlModePanel instance
        SerialPanel % just UI
        Car         % carriage control object

        
        % Serial interface app
        SerialApp

        % Visualization
        Ax % uiaxes
        CarView
        redrawTimer
        redrawRate = 20; % Hz
        hArrow % connect current position and target position for path tracking debugging
    end

    properties % parameters

    end

    methods
        function app = CarriageApp(parent,ax)

            arguments
                parent = [] % optional
                ax = []
            end

            if isempty(parent)
                pW = app.MainPanelWidth;
                pH = app.MainPanelHeight;
                app.Fig = uifigure('Name','Carriage App','Position',[100 100 pW+10*2 pH+10*2]);
                app.Fig.CloseRequestFcn = @(src,evt) app.onAppClose(src,evt);
                app.ui_parent = app.Fig;
                app.isStandAlone = 1;
            else
                app.ui_parent = parent;
                app.isStandAlone = 0;
                app.Fig = ancestor(parent,'matlab.ui.Figure','toplevel');
            end
            
            % visualization
            if isempty(ax)
                app.Ax = create_ax();
            else
                app.Ax = ax;
            end

            % Main layout using uigridlayout for easier alignment
            app.gl = uigridlayout(app.ui_parent,[3,1], ...
                        'RowHeight',{'4x','7x','3x'}, ...
                        'Padding',[5 5 5 5], ...
                        'BackgroundColor',[0.9290 0.6940 0.1250]);

            % Mode selection panel
            app.ModePanel = ControlModePanel(app.gl, app.Ax);
            app.ModePanel.Panel.Layout.Row = 2;
            app.ModePanel.Panel.Layout.Column = 1;
            addlistener(app.ModePanel, "StartPolling", @(src,evt) app.onStartPolling(src,evt));
            addlistener(app.ModePanel, "StopPolling", @(src,evt) app.onStopPolling(src,evt));
            addlistener(app.ModePanel, "StartPathtracking", @(src,evt) app.onStartPathtracking(src,evt));
            addlistener(app.ModePanel, "StopPathtracking", @(src,evt) app.onStopPathtracking(src,evt));
            addlistener(app.ModePanel, "HomeRequested", @(src,evt) app.onHomeRequested(src,evt));

            % Serial panel
            app.SerialPanel = uipanel(app.gl,'Title','Serial');
            app.SerialPanel.Layout.Row = 1;
            app.SerialPanel.Layout.Column = 1;
            app.SerialApp = SerialPortApp(app.SerialPanel);
            app.SerialApp.baudrate = 2000000;
            % listen for serial connect/disconnect events so Car gets the handle
            addlistener(app.SerialApp, 'SerialConnected', @(src,evt) app.onSerialConnected(src,evt));
            addlistener(app.SerialApp, 'SerialDisconnected', @(src,evt) app.onSerialDisconnected(src,evt));

            % Carriage control
            app.Car = CarriageControl([],app.gl);
            addlistener(app.Car, "TimerFcnStop", @(src,evt) app.onTimerFcnStop(src,evt));
            % listen for pathtracking stop so UI can be updated
            addlistener(app.Car, "PathtrackingStopped", @(src,evt) app.onPathtrackingStopped(src,evt));

            car1 = CarriageTrack(app.Ax);
            car1.set_xy(app.Car.current_pos(1),app.Car.current_pos(2));
            car1.set_lines_color([0.5 0.5 0.5]);
            car1.redraw;
            app.CarView = car1;
            app.redrawTimer = timer('TimerFcn', @(src,evt) app.update_view(), ...
                           'Name', 'Redraw timer',...
                           'Period', 1/app.redrawRate, ...% Period is in seconds
                           'ExecutionMode', 'fixedSpacing');

            app.hArrow = line(app.Ax,[0,0],[0,0],'Color','red','LineWidth',2,'Marker','+');
            app.hArrow.Visible = 'off';
            app.hArrow.Annotation.LegendInformation.IconDisplayStyle = 'off';

            if app.isStandAlone

            else
                % 
            end

        end
    end

    methods % event handlers
        function onStartPolling(app,src,evt)
            try
                disp('Activate button pressed')
                app.Car.poll_gamepad = 1;

                stat = app.Car.start_interact_timer;
                if stat~=0
                    error('Polling failed to start.')
                end
                start(app.redrawTimer);

                if app.isStandAlone % manage timer callback by self

                else
                    % disp('Polling enbled waiting for main program to start timer function.')
                end
            catch ME
                disp(ME.message)
                app.ModePanel.turnOffInteractive;
            end
        end

        function onSerialConnected(app,src,evt)
            try
                if isprop(src,'s') && ~isempty(src.s)
                    app.Car.s = src.s;
                elseif isprop(evt,'SerialPort') && ~isempty(evt.SerialPort)
                    app.Car.s = evt.SerialPort;
                end
                app.Car.update_status_from_controller();
                app.update_view();
            catch ME
                warning('CarriageApp:SerialConnectFailed','Failed to set Car.s: %s', ME.message);
            end
        end

        function onSerialDisconnected(app,src,evt)
            try
                app.Car.s = [];
            catch
            end
        end

        function onStartPathtracking(app,src,evt)
            % This starts pathtracking routine
            try
                disp('Pathtracking start requested');
                [xp,yp,rp,thetap,L,start_x,start_y,start_s,~] = app.prepare_pathtracking_data();

                % Start redraw timer if not running
                if strcmp(app.redrawTimer.Running,'off')
                    start(app.redrawTimer);
                end
                
                fprintf('Moving to start position (%.1f, %.1f) at arc s=%.1f mm\n', start_x, start_y, start_s);
                % smooth move to start (velocity profile planned by ClearCore)
                % moveToPosition use motor local coordinates
                app.Car.moveToPositionMM(start_x-app.Car.origin_mm(1), start_y-app.Car.origin_mm(2)); 
                while(~strcmp(app.Car.move_status, "SUCCESS"))
                    pause(0.1);
                    if(app.Car.move_status == "FAIL" || app.Car.move_status == "INTERRUPTED")
                        error('Failed to move to start position. Pathtracking aborted.');
                    end
                end

                % start path tracking on CarriageControl at start_s
                flush(app.Car.s); % clear controller buffer (it takes time to read serial data)
                stop(app.redrawTimer); % stop timer during blocking pathtracking
                app.runPathTracking(xp,yp,rp,thetap,L,start_s);

            catch ME
                disp(ME.message)
            end
        end

        function onStopPathtracking(app,src,evt)
            % Stop pathtracking routine (placeholder)
            try
                disp('Pathtracking stop requested');
                app.Car.stopPathTracking();
                % stop redraw timer
                if strcmp(app.redrawTimer.Running,'on')
                    stop(app.redrawTimer);
                end
            catch ME
                disp(ME.message)
            end
        end

        function onHomeRequested(app,src,evt)
            % Move carriage to home (0,0) when Home requested from ModePanel
            try
                if ~isempty(app.Car)

                    % enable visual updates if not already running
                    if strcmp(app.redrawTimer.Running,'off')
                        start(app.redrawTimer);
                    end
                    if app.Car.control_aoa
                        app.Car.sendCommand('NUL,NUL,ABS0>');
                    end
                    pause(0.2);
                    app.Car.moveToPosition(0,0,20);
                end
            catch ME
                warning('CarriageApp:HomeFailed','Home move failed: %s', ME.message);
            end
        end

        function onStopPolling(app,src,evt)
            app.Car.poll_gamepad = 0;
            app.Car.stop_interact_timer;
            disp('Deactivate, control timer fcn stopped');
            stop(app.redrawTimer);
            if app.isStandAlone
                app.Car.stop_interact_timer;
            end
        end

        function onTimerFcnStop(app,src,evt)
            % disp('CarriageApp: control timer fcn error')
            app.ModePanel.turnOffInteractive;
            app.Car.stop_interact_timer;
            stop(app.redrawTimer);
        end

        function onAppClose(app,src,evt)
            % Be defensive: src or evt may be invalid/deleted; don't assume
            try
                fprintf('Received event %s from %s.\n',evt.EventName,src.Name);
            catch
                % ignore printing errors
            end

            % Delete serial port if it exists and is a valid handle
            try
                if isprop(app,'SerialApp') && ~isempty(app.SerialApp) && isprop(app.SerialApp,'s') && ~isempty(app.SerialApp.s) && isvalid(app.SerialApp.s)
                    try
                        delete(app.SerialApp.s);
                    catch
                        % ignore
                    end
                end
            catch
                % ignore access errors
            end

            % Delete axes parent (figure) if valid
            try
                if ~isempty(app.Ax) && isvalid(app.Ax) && isvalid(app.Ax.Parent)
                    try
                        delete(app.Ax.Parent);
                    catch
                        % ignore
                    end
                end
            catch
                % ignore
            end

            % Stop and delete timers if present
            try
                if isprop(app,'redrawTimer') && ~isempty(app.redrawTimer) && isvalid(app.redrawTimer)
                    try
                        if strcmp(app.redrawTimer.Running,'on')
                            stop(app.redrawTimer);
                        end
                    catch
                    end
                    try
                        delete(app.redrawTimer);
                    catch
                    end
                end
                
                if isprop(app.Car,'iTimer') && ~isempty(app.Car.iTimer) && isvalid(app.Car.iTimer)
                    try
                        if strcmp(app.Car.iTimer.Running,'on')
                            stop(app.Car.iTimer);
                        end
                    catch
                    end
                    try
                        delete(app.Car.iTimer);
                    catch
                    end
                end
            catch
            end

            % Finally delete main figure if still valid
            try
                if ~isempty(app.Fig) && isvalid(app.Fig)
                    delete(app.Fig);
                end
            catch
                % ignore
            end
        end

        function onPathtrackingStopped(app,src,evt)
            % keep visual update for a short while after pathtracking stops
            % this shows the inertial drift after motor stops
            % tstart = tic;
            % while toc(tstart) < 0.5
            %     app.Car.update_status_from_controller();
            %     app.update_view();
            % end

            % Ensure Play button text is reset if pathtracking finished
            try
                if ~isempty(app.ModePanel) && isvalid(app.ModePanel.FileListBox)
                    app.ModePanel.PlayBtn.Text = '▶ Start';
                end
            catch
            end
        end

    end

    methods 
        function [xp,yp,rp,thetap,L,start_x,start_y,start_s,pathtag] = prepare_pathtracking_data(app)
            % Build PathData from selected file in ModePanel (must be single file)
            try
                [pd, fullpath] = app.ModePanel.createPathDataFromSelection();
                [~, name1, ~] = fileparts(fullpath);
                pathtag= erase(name1,'xy_');
            catch ME
                uialert(app.Fig, sprintf('Failed to load path: %s', ME.message), 'Path Error');
                return
            end

            [xp,yp,rp,thetap,L] = pd.getInterpolants();

            % compute starting arc-length where path x(s) matches this carriage origin x
            try
                origin_x_mm = app.Car.origin(1) * app.Car.step2mm;
            catch
                origin_x_mm = 0;
            end
            % sample arc to find a close index
            s_grid = linspace(0, L, 4001);
            x_vals = xp(s_grid);

            [~, idx_min] = min(abs(x_vals - origin_x_mm));
            s0_guess = s_grid(idx_min);

            % ensure start_s is in valid range
            start_s = max(0, min(L, s0_guess));

            % compute start coordinates and move carriage there (in mm)
            start_x = xp(start_s);
            start_y = yp(start_s);
        
            % return the precomputed tangent interpolant so controllers can use it
        end

        function runPathTracking(app,xp,yp,rp,thetap,Ltot,start_s)
            % Initialize path tracking and start timer-driven ticks.
            % xp,yp,rp - interpolants mapping arc length (mm) -> x,y, curvature radius
            % thetap - angle interpolant mapping arc length (mm) -> tangent angle (radians)
            % Ltot - total arc length (mm)
            % start_s - starting arc length (mm)

            app.Car.init_pathtracking_variables(xp,yp,rp,Ltot,start_s,thetap);
            % start the timer (init_pathtracking_variables ensures pathTimer exists)
            try
                flush(app.Car.s);
            catch
            end

            period = app.Car.path_CMD_INTERVAL/1000; % seconds
            t0 = tic;
            n = 0;
            app.hArrow.Visible = 'on';
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
                    is_done = app.Car.pathTrackingTick(src, ev);
                catch ME
                    warning('pathTrackingTick:error','Error in pathTrackingTick: %s', ME.message);
                    is_done = true;
                end

                if is_done
                    break;
                end
                if mod(n,round(50/app.Car.path_CMD_INTERVAL))==0
                    app.update_view(); % keep UI responsive every 50 ms
                end

                % schedule next tick using tic/toc to reduce drift
                next_time = n * period;
                elapsed = toc(t0);

                fprintf('Tick %d: elapsed total=%.3f s, frame=%.1f ms, avg FPS=%d\n', n, elapsed, toc(t1)*1000, round(n/elapsed));
                
                % busy-wait the final few ms to reduce jitter
                while toc(t0) < next_time
                    pause(0.0005); % yield briefly to keep UI responsive, NOTE pause <=1ms triggers higher resolution OS timer.
                end
            end

            % ensure any requested stop is applied and notify listeners
            try
                app.Car.stopPathTracking();
            catch
            end
            app.hArrow.Visible = 'off';
        end
    end

    methods % 
        function update_view(app)
            try
                x = app.Car.real_loc(1);
                y = app.Car.real_loc(2);
                app.CarView.set_xy(x,y);
                app.CarView.redraw();

                app.hArrow.XData = [x, app.Car.path_target_loc(1)];
                app.hArrow.YData = [y, app.Car.path_target_loc(2)];

                drawnow limitrate
            catch ME
                disp(ME.identifier)
                disp(ME.message)
                disp(ME.stack.file)   % shows file + line numbers
                disp(ME.stack.line)
                error('Cant draw.')
            end
        end

    end
end

