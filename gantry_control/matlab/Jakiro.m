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
    end

    properties
        run_start_time = [] % time when experiment started
        cc1_done = false; % flag indicating carriage 1 finished path tracking
        cc2_done = false; % flag indicating carriage 2 finished path tracking
        pathpath_tick_period = 25; % period of path tracking timer in ms
        pathpath_redraw_interval = 2; % update interval for path tracking
        outpath % output path for wavi data
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
            app.CC1.Car.origin = [3870, 0]; % parking position for carriage 1 (steps)
            app.CC1.Car.origin_mm = app.CC1.Car.origin * app.CC1.Car.step2mm;
            app.CC1.Car.path_dx_max = 3800; % max x movement range (mm)
            app.CC1.Car.motor_settings.ACC = 10000;
            app.CC1.Car.name = 'Front Carriage';
            app.CC1.Car.control_aoa = false;
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
            app.WA = wavi(wa_panel,false,n_update=4,...
                            ch_map =load('channel_map.txt'),... % not standalone mode
                            outpath = outpath);

            % Experiment control panel in row 2, column 3
            exp_panel_parent = uipanel(uigl,'Title','Experiment');
            exp_panel_parent.Layout.Column = 3;
            exp_panel_parent.Layout.Row = 2;
            app.ExpPanel = ExpControlPanel(exp_panel_parent);
            % listen for start events
            addlistener(app.ExpPanel,'StartExperiment',@(src,evt) app.onStartExperiment(src,evt));
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

        function onStartExperiment(app, ~, ~)
            % coordinate the two carriages using parameters from ExpPanel
            try
                [v1,v2,delay_s,run_tag] = app.ExpPanel.getParameters();
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
                    'Period', app.pathpath_tick_period/1000, 'ExecutionMode','fixedrate', 'Name','Path Path timer', 'BusyMode','drop');
                else
                    try
                        stop(app.pathpathTimer);
                    catch
                    end
            end
            
            % prepare daq
            if ~isempty(app.WA.s)
                app.WA.tag = sprintf('%s_%s-v1=%.2f_%s-v2=%.2f_delay=%.1f',run_tag,pathtag1,v1,pathtag2,v2,delay_s);
                app.WA.init_datalog_file();
                app.WA.align_data_read();
                app.WA.average_signal_as_offset(round(app.WA.Fs));
                app.WA.spec_data = nan(size(app.WA.spec_data)); % reset spectrogram
            end
            app.run_start_time = datetime('now');
            start(app.pathpathTimer);
        end
    end

    methods %
        function path_path_tick(app,src,event)
            tic;
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
                % update path target location arrow
                app.CC2.hArrow.XData = [app.CC2.Car.real_loc(1), app.CC2.Car.path_target_loc(1)];
                app.CC2.hArrow.YData = [app.CC2.Car.real_loc(2), app.CC2.Car.path_target_loc(2)];
            end

            % app.WA.read_serial_data(app.WA.ns_read, app.WA.ns_fill);
            app.WA.read_update_tick();

            if mod(src.TasksExecuted,app.pathpath_redraw_interval)==0
                app.CC1.update_view();
                app.CC2.update_view();
                % app.WA.update_visuals();
                drawnow limitrate
            end
            
            if app.cc1_done && app.cc2_done
                % finished path tracking for carriage 1
                try
                    stop(src);
                    fclose(app.WA.fout);
                catch
                end
            end
            toc;
        end
    end
end
