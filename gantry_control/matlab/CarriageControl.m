classdef CarriageControl < handle
    % CarriageControl
    % UI + functions for receiving data and sending commands to ClearCore
    % 
    
    properties % UI
        UIFigure
        Parent
        gl
        inspectBtn
    end
    
    properties % handles
        s % serialport handle
        kb
        joy
        iTimer % interactive timer
        ser_log_file

        % visuals
        hGV
        hVelText % for velocity information
        hStatusText % for controller feedback info
        hInputText
    end

    properties % motor and control parameters
        motor_settings
        name = 'Carriage Control'
        END_MARKER = '>'; % end marker for motor control commands
        poll_gamepad
        poll_keyboard
        joystick_side
        step2mm = 0.0499; % clearpath motor steps to mm conversion, measured from installation
        polling_interval = 25 % ms
        aoa_motor_type = 'ClearPath' % 'Stepper' or 'ClearPath'
        control_aoa = true % whether to control AOA automatically (keep 0 aoa)

        wait_prev = 0
        vx_current = 0 % current x vel, command value
        vy_current = 0 % current y vel, command value

        % controller feedback
        current_pos = [0 0 0 0 0] % [x, y, aoa_pos, aoa_angle, controller time]
        prev_pos = [0 0 0 0 0]
        
        vel_max = 10000
        vx_max = 10000
        vy_max = 10000

        % target velocity
        vx_cruise = 0
        vy_cruise = 0
        
        % actual velocity calculated from position and time
        vx_t = 0
        vy_t = 0
        
        % time returned by controller
        controller_time1 = 0
        controller_time2 = 0
        last_status_time = []   % datetime of last status update
        
        % status
        real_loc = [0, 0] % physical location
        real_vel = [0, 0] % physical velocity
        aoa_old = 0        % previous aoa (radians), for smoothing

        % gantry origin in the tank coordinate system
        origin = [0,0]
        origin_mm = [0,0] 

        % path tracking state
        pathTimer = []            % timer used for path tracking ticks
        path_xp                  % griddedInterpolant for x(arc)
        path_yp                  % griddedInterpolant for y(arc)
        path_rp                  % griddedInterpolant for radius(arc)
        path_thetap              % griddedInterpolant or function handle for tangent angle(arc)
        path_Ltot = 0            % total arc length
        path_arc_len = 0         % current arc length travelled (mm)
        path_d_trav = 0          % cumulative travelled distance (mm)
        path_xy_old = [0 0]      % previous position steps
        path_npoll = 0           % poll counter
        path_CMD_INTERVAL =  10  % ms (default)
        path_stopRequested = 0   % flag to request stop from outside
        path_dx_max = 3600       % max x movement range (mm)
        path_target_loc = [0,0]  % target location for path tracking (mm)

        % movement boundaries (mm). Leave empty (Inf) for no bound.
        x_min_mm = -Inf
        x_max_mm = Inf
        y_min_mm = -Inf
        y_max_mm = Inf
        boundary_margin_mm = 20    % margin (mm) near boundary to start blocking motion
    end

    properties
        % asynchronous move state
        moveTimer = []           % timer used for asynchronous moveToPosition
        move_target              % current async move target in steps
        move_tol_steps = 1       % tolerance for async move
        move_timeout_s = 20      % timeout seconds for async move
        move_status = 'NULL'      % async move status: 'NULL', 'INITIATED', 'SUCCESS', 'FAIL', 'INTERRUPTED'
    end

    events
        TimerFcnStop
        PathtrackingStopped
        MoveCompleted
    end

    methods
        function obj = CarriageControl(s,parent,opt)
            arguments
                s = []
                % s (1,1) serialport % required, must be a serialport object
                parent = [] % optional
                opt.name = "Gantry carriage"
                opt.X_Sign = 1;
                opt.Y_Sign = 1;
                opt.joystick_side = 'L';
                opt.origin = [0,0];
            end

            % create UI
            if isempty(parent)
                obj.UIFigure = uifigure('Name','Control Mode','Position',[100 100 360 240]);
                obj.Parent = obj.UIFigure;
            else
                obj.Parent = parent;
                obj.UIFigure = ancestor(parent,'matlab.ui.Figure','toplevel');
            end

            obj.gl = uigridlayout(obj.Parent,[3,2]);
            obj.gl.RowHeight = {'1x','2x','1x'};
            obj.gl.ColumnWidth = {'6x','1x'};

            hInputText = uilabel(obj.gl);
            hInputText.Layout.Row = 1;
            hInputText.Layout.Column = [1 2];
            hInputText.Text = sprintf('Start polling to update input');
            hInputText.FontSize = 12;
            hInputText.HorizontalAlignment = 'left';
            obj.hInputText = hInputText;

            hVelText = uilabel(obj.gl);
            hVelText.Layout.Row = 2;
            hVelText.Layout.Column = [1 2];
            hVelText.Text = sprintf('Start polling to update velocity');
            hVelText.FontSize = 12;
            hVelText.HorizontalAlignment = 'left';
            obj.hVelText = hVelText;

            hStatusText = uilabel(obj.gl);
            hStatusText.Layout.Row = 3;
            hStatusText.Layout.Column = 1;
            hStatusText.Text = sprintf('Start polling to update status');
            hStatusText.FontSize = 12;
            hStatusText.HorizontalAlignment = 'left';
            obj.hStatusText = hStatusText;

            % Inspect button to open Property Inspector for this object
            inspectBtn = uibutton(obj.gl, 'push');
            inspectBtn.Layout.Row = 3;
            inspectBtn.Layout.Column = 2;
            inspectBtn.Text = 'Inspect';
            inspectBtn.Tooltip = 'Open Property Inspector for this CarriageControl object';
            inspectBtn.ButtonPushedFcn = @(src,evt) inspect(obj);
            obj.inspectBtn = inspectBtn;

            % set properties
            obj.s = s;
            obj.name = opt.name;

            motor_settings.MAX_SPEED = 20000; % max speed steps/sec
            motor_settings.INC_SPEED = 200; % to do: base this on FPS
            motor_settings.X_SIGN = opt.X_Sign;
            motor_settings.Y_SIGN = opt.Y_Sign;
            motor_settings.ACC = 20000; % acceleration steps/sec^2
            % motor_settings.STEPS_PER_REV = 6400; % for generic stepper
            motor_settings.STEPS_PER_REV = 800*20; % for ClearPath with 20:1 gearbox
            obj.motor_settings = motor_settings;

            % 'native' is necessary. WARNING: control will be active even when MATLAB is out of focus
            obj.kb = HebiKeyboard('native');
            % joy = HebiJoystick(1); % this 3rd party function doesnot work with uifigure .
            obj.joy = vrjoystick(1);
            obj.poll_gamepad = 0;
            obj.poll_keyboard = 0;
            obj.joystick_side = opt.joystick_side;
            obj.origin = opt.origin;
            obj.origin_mm = opt.origin*obj.step2mm;

            obj.iTimer = timer('TimerFcn', @(src,evt) obj.interactControlTick(src,evt), ...
                               'Name', 'Interactive control timer',...
                               'Period', obj.polling_interval/1000, ...% Period is in seconds
                               'ExecutionMode', 'fixedSpacing',...
                               'ErrorFcn', @(src,evt) obj.onTimerFcnError(src,evt));

            obj.ser_log_file = fopen('serialport_received.dat','w');

        end

        function stat = start_interact_timer(obj)
            stat = 1; % assume failure
            if isempty(obj.s)
                uialert(obj.UIFigure,'Gantry controller not connected.','Error');
                return
            end
            flush(obj.s);
            fprintf('Starting timer ..\n');
            start(obj.iTimer);
            % fprintf('Polling started. Press q in the figure window to stop.\n');
            stat = 0;
        end

        function stop_interact_timer(obj)
            stop(obj.iTimer);
        end

        function onTimerFcnError(obj,src,evt)
            stop(obj.iTimer);
            disp('Control stopped due to error.');
            notify(obj,'TimerFcnStop')
        end
        function v = clamp(obj,v,vmax)
            v = max(0,v);
            v = min(v,vmax);
        end
    end

    methods % control functions
        function interactControlTick(obj, src, event)
            % interactive control, poll input devices and send commands
            % obj - CarriageControl class object
            % src - timer object
            % event i timer event
            try
                tStart = tic;
                obj.update_status_from_controller();

                % get user inputs    
                state = read(obj.kb); % to do return empty state when not polling keyboard
    
                if obj.poll_gamepad
                    % returns axes positions, button status, and POVs (d-pad)
                    [axx, buttons, ~] = read(obj.joy);
                else
                    axx = zeros(1,5);
                    buttons = zeros(1,10);
                end
    
                % joystick notes
                % axis values are normalized
                %      		     ^ (up:-1.0)
                %        	     |
                % left: -1.0 <---0---> right: 1.0
                %                |
                %                v (down: 1.0)
            
                % axx is for [LX LY Z RX RY]
                % xbox controller has 10 buttons
                % dpad value is a scalar, -1 if not pressed, value indicates  of the 8 directions, clockwise 0:45:315
                
                LXA = 1; % left x axis (left analog stick)
                LYA = 2; % left y axis
                ZAX = 3; % z axis (triggers)
                RXA = 4; % right x axis
                RYA = 5; % right y axis
            
                if obj.joystick_side == 'L'
                    iXA = LXA;
                    iYA = LYA;
                elseif  obj.joystick_side == 'R'
                    iXA = RXA;
                    iYA = RYA;
                else
                    error('Wrong joystick side.')
                end
    
                DEAD_ZONE = 0.25; % because drifting in controller analog sticks
                % note that left and right trigger both control the z axis LT:1.0 RT:-1.0
                
                % Buttons index
                RTN_BTN = 7; % select button, send "return to home position" command
                SET_BTN = 9; % left stick press, send "set as home position" command
                L_BTN = 5; % AOA turn left
                R_BTN = 6; % AOA turn right
                
                % key = get(fig, 'CurrentKey');
                poll_time = datetime(event.Data.time);
                poll_time.Format = 'dd-MMM-uuuu HH:mm:ss.SSS';
                input_str = sprintf('%10d %s Inputs: ',src.TasksExecuted, poll_time);
            
                INC_SPEED = obj.motor_settings.INC_SPEED;
                MAX_SPEED = obj.motor_settings.MAX_SPEED;
            
                % check for interactive inputs
                % interactive movement
                xdir = 0;
                ydir = 0;
                joystick_control = false;
                if any(abs(axx)>DEAD_ZONE)
                    joystick_control = true;
                end
            
                if state.LEFT || axx(iXA)<-DEAD_ZONE
                    fprintf('← ');
                    input_str = [input_str '← '];
                    if ~state.RIGHT
                        xdir = 1;
                    end
                end
            
                if state.RIGHT || axx(iXA)> DEAD_ZONE
                    fprintf('→ ');
                    input_str = [input_str '→ '];
                    if ~state.LEFT
                        xdir = -1;
                    end
                end
            
                if state.UP || axx(iYA) < -DEAD_ZONE
                    fprintf('↑ ');
                    input_str = [input_str '↑ '];
                    if ~state.DOWN
                        ydir = -1;
                    end
                end
            
                if state.DOWN || axx(iYA) > DEAD_ZONE
                    fprintf('↓ ');
                    input_str = [input_str '↓ '];
                    if ~state.UP
                        ydir = 1;
                    end
                end
            
                % acceleration
                if state.keys('s')
                    fprintf('s ');
                    obj.vx_max = obj.vx_max + INC_SPEED;
                end
            
                if state.keys('f')
                    fprintf('f ');
                    obj.vx_max = obj.vx_max - INC_SPEED;
                end
            
                if state.keys('e')
                    fprintf('e ');
                    obj.vy_max = obj.vy_max + INC_SPEED;
                end
                if state.keys('d')
                    fprintf('d ');
                    obj.vy_max = obj.vy_max - INC_SPEED;
                end
            
                if axx(ZAX) < -DEAD_ZONE
                    fprintf('🎮RT ');
                    obj.vel_max = obj.vel_max + INC_SPEED;
                end
            
                if axx(ZAX) > DEAD_ZONE
                    fprintf('🎮LT ');
                    obj.vel_max = obj.vel_max - INC_SPEED;
                end
            
                if any(state.keys) || any(abs(axx)>DEAD_ZONE) || any(buttons)
                    fprintf('\n');
                end
            
                if state.keys('q') % using ascii indexing
                    % --- stop input polling ---
                    stop(src);
                    fprintf('Input polling stopped by user.\n');
                    notify(obj,'TimerFcnStop')
                end
            
                obj.vx_max = obj.clamp(obj.vx_max,MAX_SPEED);
                obj.vy_max = obj.clamp(obj.vy_max,MAX_SPEED);
                obj.vel_max = obj.clamp(obj.vel_max,MAX_SPEED);
            
                if joystick_control % override for joystick control
                    aoa = atan2(axx(iYA),axx(iXA));
                    obj.vx_max = abs(round(obj.vel_max*cos(aoa)));
                    obj.vy_max = abs(round(obj.vel_max*sin(aoa)));
                end
                
                % calculate velocity components
                obj.vx_cruise = obj.vx_max*xdir*obj.motor_settings.X_SIGN;
                obj.vy_cruise = obj.vy_max*ydir*obj.motor_settings.Y_SIGN;
                aoa = atan2(obj.vy_cruise,obj.vx_cruise);
                aoa_d = round(aoa/pi*180);
            
                % fprintf('\n');
                if xdir ~= 0
                    % ramp
                    obj.vx_current = obj.vx_current ...
                        + sign(obj.vx_cruise - obj.vx_current)*round(INC_SPEED*abs(cos(aoa)));
                    if abs(obj.vx_current)>obj.vx_max
                        obj.vx_current = sign(obj.vx_current)*obj.vx_max;
                    end
                end
            
                if ydir ~= 0
                    % ramp
                    obj.vy_current = obj.vy_current + ...
                        sign(obj.vy_cruise - obj.vy_current)*round(INC_SPEED*abs(sin(aoa)));
                    if abs(obj.vy_current)>obj.vy_max
                        obj.vy_current = sign(obj.vy_current)*obj.vy_max;
                    end
                end

                % enforce movement boundaries (zero velocity component if at/near boundary)
                [obj.vx_current, obj.vy_current] = obj.limit_velocity_by_bounds(obj.vx_current, obj.vy_current);
                
                % compose commands
                if xdir ~= 0
                    cmd_x = sprintf('VEL%d',obj.vx_current);
                    obj.wait_prev = 0; % previous cmd interrupted
                elseif obj.wait_prev
                    cmd_x = 'PRE'; % keep previous action
                elseif obj.vx_current ~=0
                    obj.vx_current = round(obj.vx_current*0.49);
                    cmd_x = sprintf('VEL%d',obj.vx_current);
                else
                    cmd_x = 'NUL'; % NULL command (stop). The command string can't be empty!!!
                end
            
                if ydir ~= 0
                    cmd_y = sprintf('VEL%d',obj.vy_current);
                    obj.wait_prev = 0;
                elseif obj.wait_prev
                    cmd_y = 'PRE';
                elseif obj.vy_current ~=0
                    obj.vy_current = round(obj.vy_current*0.49);
                    cmd_y = sprintf('VEL%d',obj.vy_current);
                else
                    cmd_y = 'NUL';
                end
                
                % -------------
                % AOA
                % -------------
                if obj.control_aoa && (xdir ~= 0 || ydir~=0)
                    aoa = atan(obj.vy_current/obj.vx_current);
                    spr = obj.motor_settings.STEPS_PER_REV;
                    tgt_aoa_pos = round(aoa/(2*pi)*spr);
                    if strcmpi(obj.aoa_motor_type,'Stepper')
                        % dist = round(tgt_aoa_pos - obj.current_pos(3)*(-1));
                        % limit range due to buggy control
                        % dist = min(dist,spr/2);
                        % dist = max(-spr/2,dist);
                        % if dist ~= 0
                            % cmd_a = sprintf('REL%d',-dist);
                        cmd_a = sprintf('ABS%d',tgt_aoa_pos*(-1));
    
                        % else
                        %     cmd_a = 'NUL';
                        % end
    
                    elseif strcmpi(obj.aoa_motor_type,'ClearPath')
                        cmd_a = sprintf('ABS%d',tgt_aoa_pos);
                    end
                elseif obj.wait_prev
                    cmd_a = 'PRE';
                else
                    cmd_a = 'NUL';
                end
            
                if buttons(L_BTN)
                    % for generic stepper, move RELative steps
                    cmd_a = sprintf('VEL%d',-160); 
                    fprintf('🎮LB ');
                elseif buttons(R_BTN)
                    cmd_a = sprintf('VEL%d',160);
                    fprintf('🎮RB ');
                end
            
                % return to home
                if buttons(RTN_BTN) || (state.CTRL && state.ALT && state.keys('0'))
                    cmd_x = 'ABS0';
                    cmd_y = 'ABS0';
                    cmd_a = 'ABS0';
                    obj.wait_prev = 1;
                end
            
                if buttons(SET_BTN) || (state.CTRL && state.ALT && state.keys('s'))
                    cmd_x = 'SET';
                    cmd_y = 'SET';
                    cmd_a = 'SET';
                    obj.wait_prev = 0;
                end
            
                if ~strcmp(cmd_x,'NUL') || ~strcmp(cmd_y,'NUL') || ~strcmp(cmd_a,'NUL')
                    command = sprintf('%s,%s,%s%c',cmd_x,cmd_y,cmd_a,obj.END_MARKER);
                    % fprintf(s, command); % this somehow doesn't work
                    write(obj.s,command,"char");
                    fprintf('%s\t%9d\tSent command: %s\n',poll_time, obj.controller_time2,command);
                end
            
                if joystick_control
                    input_str = [input_str '🎮'];
                end
                set(obj.hInputText, 'Text', input_str);
            
                % frame_time moved to update_status_from_controller
    
                % fmt = 'Status: Pos=[%5d,%5d,%5d,%5d], t=%s';
                % t_str = ms_to_hms_string(obj.current_pos(5));
                % set(obj.hStatusText, 'Text',sprintf(fmt,obj.current_pos(1:4),t_str));
                
                % hGV.redraw(4400,1400,current_pos(1)*step2mm,current_pos(2)*step2mm);
                drawnow limitrate
            catch me
                disp(me.message);
                fprintf('%s %s line%d\n',me.stack(1).file, me.stack(1).name, me.stack(1).line);
                error('Timer callback function error');
            end
        end

        function is_done = pathTrackingTick(obj, src, event)
            % follow prescribed path
            % Minimal path tracking tick: read controller position, compute
            % target along interpolants, send velocity commands. This
            % function is designed to be called from a timer with period
            % obj.path_CMD_INTERVAL/1000.
            
            tStart = tic;
            is_done = false;

            poll_time = datetime(event.Data.time);
            poll_time.Format = 'dd-MMM-uuuu HH:mm:ss.SSS';
            dt_str = sprintf('%10d %s',src.TasksExecuted, poll_time);

            % If stop requested externally, stop timer and return
            if obj.path_stopRequested || obj.check_user_interrupt()
                try
                    fprintf('%s Path tracking stop requested.\n',obj.name);
                    obj.sendStopCommand();
                    obj.stopPathTracking();
                catch
                end
                is_done = true;
                return
            end
            
            % read data from serial port
            try
                if(obj.path_npoll==0) % first call, flush serial buffer (microcontroller sends data non stop)
                    flush(obj.s);
                end
                obj.update_status_from_controller();

            catch ME
                disp(ME.message);
                obj.stopPathTracking();
                is_done = true;
                return
            end

            % update internal counters
            obj.path_npoll = obj.path_npoll + 1;

            % compute traveled distance in steps and mm
            prev = obj.path_xy_old;
            curr = obj.current_pos(1:2)+obj.origin;
            prev_vec = curr - prev;
            prev_dist_steps = norm(prev_vec);
            obj.path_d_trav = obj.path_d_trav + prev_dist_steps*obj.step2mm;
            obj.path_xy_old = curr;
            

            % convert to mm
            if obj.path_npoll == 1
                look_ahead_dist = obj.vel_max * obj.path_CMD_INTERVAL/1000 * obj.step2mm*2.0;
            else
                % the previous distance travelled is a good estimate of lookahead distance
                look_ahead_dist = prev_dist_steps * obj.step2mm * obj.vel_max/1000; % scale lookahead with speed, 800 steps/sec is a good reference
                look_ahead_dist = max(look_ahead_dist, obj.vel_max*obj.path_CMD_INTERVAL/1000*obj.step2mm*0.8); % limit minimum to avoid overshoot
                look_ahead_dist = min(look_ahead_dist, obj.vel_max*obj.path_CMD_INTERVAL/1000*obj.step2mm*2.0); % limit maximum to improve accuracy
            end
            % Set new target position along path. 
            % Note that travelled distance is only an approximate of the arc.
            % A better way is to find the closest point on the path to the current position.
            new_path_arc_len = obj.path_d_trav + look_ahead_dist;

            if obj.path_d_trav > obj.path_Ltot
                % reached end
                fprintf('%s Path tracking reached the end of path.\n',obj.name);
                obj.stopPathTracking();
                is_done = true;
                return
            end

            if abs(curr(1)*obj.step2mm) > obj.path_dx_max
                % reached end
                fprintf('%s Path tracking reached the max X range.\n',obj.name);
                obj.stopPathTracking();
                is_done = true;
                return
            end

            % don't decrease arc length (no going backwards, this is to avoid AOA oscillations)
            if new_path_arc_len > obj.path_arc_len
                obj.path_arc_len = new_path_arc_len;
            else
                obj.path_arc_len = obj.path_arc_len + prev_dist_steps*obj.step2mm; % advance a little
            end

            % compute target in steps
            obj.path_target_loc = [obj.path_xp(obj.path_arc_len), obj.path_yp(obj.path_arc_len)];
            target_x = round(obj.path_target_loc(1)/obj.step2mm);
            target_y = round(obj.path_target_loc(2)/obj.step2mm);
            target_r = obj.path_rp(obj.path_arc_len)/obj.step2mm;
            curve_speed_limit = round(sqrt(obj.motor_settings.ACC*0.5*target_r));

            % compute direction and velocity
            xvec = target_x - curr(1);
            yvec = target_y - curr(2);
            aoa = atan2(double(yvec), double(xvec));
            vel_amp = obj.vel_max*obj.path_npoll/20; % ramp up speed over first calls

            vel_amp = min([vel_amp, curve_speed_limit, obj.vel_max]);
            vx_amp = abs(round(vel_amp*cos(aoa)));
            vy_amp = abs(round(vel_amp*sin(aoa)));

            if xvec < 0
                xdir = -1;
            else
                xdir = 1;
            end
            if yvec < 0
                ydir = -1;
            else
                ydir = 1;
            end

            % calculate velocity components
            obj.vx_cruise = vx_amp*xdir*obj.motor_settings.X_SIGN;
            obj.vy_cruise = vy_amp*ydir*obj.motor_settings.Y_SIGN;

            % ramping is skipped in path tracking mode
            obj.vx_current = obj.vx_cruise;
            obj.vy_current = obj.vy_cruise;

            % enforce movement boundaries for path tracking
            [obj.vx_current, obj.vy_current] = obj.limit_velocity_by_bounds(obj.vx_current, obj.vy_current);

            % compose velocity commands (simple direct set)
            cmd_x = sprintf('VEL%d',round(obj.vx_current));
            cmd_y = sprintf('VEL%d',round(obj.vy_current));
            if obj.control_aoa
                
                if ~isempty(obj.path_thetap) % use tangent interpolant
                % if 1==0
                    aoa_tmp = (obj.path_thetap(obj.path_arc_len)+obj.path_thetap(obj.path_d_trav))/2;
                else
                    aoa_tmp = atan(obj.vy_current/obj.vx_current); % use instantaneous velocity vector
                end
                aoa = 0.5*obj.aoa_old + 0.5*aoa_tmp; % smooth AOA changes
                obj.aoa_old = aoa;

                fprintf('%s path tangent angle: %.1f %.1f\n',dt_str, aoa/pi*180,obj.path_arc_len);
                spr = obj.motor_settings.STEPS_PER_REV;
                aoa = max(min(pi/2,aoa),-pi/2); % limit AOA (hardware limitation, to avoid cable issues)

                tgt_aoa_pos = round(aoa/(2*pi)*spr);
                if strcmpi(obj.aoa_motor_type,'Stepper')
                    cmd_a = sprintf('ABS%d',tgt_aoa_pos*(-1));
                elseif strcmpi(obj.aoa_motor_type,'ClearPath')
                    % todo
                    cmd_a = sprintf('ABS%d',tgt_aoa_pos);
                end
            else
                cmd_a = 'NUL'; % keep previous AOA command
            end

            command = sprintf('%s,%s,%s%c',cmd_x,cmd_y,cmd_a,obj.END_MARKER);
            try
                write(obj.s,command,'char');
                fprintf('%s path tick: %.1f %.1f %.1f\n',dt_str, obj.path_arc_len, ...
                                        obj.path_d_trav, look_ahead_dist);
                fprintf('PathTick: Sent command: %s\n',command);
            catch me
                warning(me.identifier, '%s', me.message);
            end

            % update input display
            set(obj.hInputText, 'Text', sprintf('Path tracking %6d call time %5.2f ms',obj.path_npoll,toc(tStart)*1000));
        end

        function init_pathtracking_variables(obj,xp,yp,rp,Ltot,start_s,thetap)
            % Initialize internal path tracking state and
            % create (but do not start) the path timer. Accepts the same

            obj.path_xp = xp;
            obj.path_yp = yp;
            obj.path_rp = rp;
            obj.path_thetap = thetap;
            obj.path_Ltot = Ltot;
            obj.path_d_trav = start_s;

            % initialize arc-length counters
            obj.path_arc_len = 0;
            obj.path_xy_old = obj.current_pos(1:2)+obj.origin;
            obj.path_npoll = 0;
            obj.path_stopRequested = 0;

            % create or reconfigure the path timer (but do not start it)
            if isempty(obj.pathTimer) || ~isvalid(obj.pathTimer)
                obj.pathTimer = timer('TimerFcn',@(src,evt) obj.pathTrackingTick(src,evt),...
                    'Period', obj.path_CMD_INTERVAL/1000, 'ExecutionMode','fixedSpacing', 'Name','Path tracking timer');
            else
                try
                    stop(obj.pathTimer);
                catch
                end
            end
        end

        function stopPathTracking(obj)
            % Request stop and clean up timer
            obj.path_stopRequested = 1;
            if ~isempty(obj.pathTimer) && isvalid(obj.pathTimer)
                try
                    stop(obj.pathTimer);
                    delete(obj.pathTimer);
                catch
                end
                obj.pathTimer = [];
            end

            % notify listeners that pathtracking stopped
            try
                notify(obj,'PathtrackingStopped');
            catch
            end
        end

        function ok = moveToPosition(obj, xTarget, yTarget, timeout_s, tol_steps, async)
            % Send ABS commands and wait until controller reports target position

            % xTarget, yTarget - target positions in steps
            % timeout_s - optional timeout in seconds (default 10s)
            % tol_steps - optional tolerance in steps (default 1)
            % async - optional logical, if true (default) the function returns immediately and
            %         the move is monitored in the background. If false, the function blocks until
            %         the move is complete or timeout occurs.
            % ok - true if target reached within tolerance, false otherwise, only valid if async=false

            arguments
                obj
                xTarget (1,1) double
                yTarget (1,1) double
                timeout_s (1,1) double = 10000
                tol_steps (1,1) double = 1
                async (1,1) logical = true
            end

            ok = false;
            if isempty(obj.s)
                warning('moveToPosition:NoSerial','Serial port not available.');
                return
            end

            % compose ABS commands (controller expects ABS in steps)
            cmd_x = sprintf('ABS%d',round(xTarget));
            cmd_y = sprintf('ABS%d',round(yTarget));

            % keep AOA as PRE to avoid changing angle todo: can this be configured?
            cmd_a = 'PRE';
            command = sprintf('%s,%s,%s%c',cmd_x,cmd_y,cmd_a,obj.END_MARKER);

            try
                obj.sendCommand(command);
            catch me
                warning(me.identifier,'%s',me.message);
                return
            end

            obj.move_status = 'INITIATED';
            fprintf('%s moveToPosition: Target [%d,%d], tol=%d steps, timeout=%d s, async=%d. Initiated.\n', ...
                obj.name, round(xTarget),round(yTarget),tol_steps,timeout_s,async);
            if async
                % configure async move
                obj.move_target = [round(xTarget) round(yTarget)];
                obj.move_tol_steps = tol_steps;
                obj.move_timeout_s = timeout_s;

                % create moveTimer if needed
                if isempty(obj.moveTimer) || ~isvalid(obj.moveTimer)
                    try delete(obj.moveTimer); catch, end
                    obj.moveTimer = timer('TimerFcn',@(src,evt) obj.positionMoveCheckTick(src,evt),...
                        'Period',0.1,'ExecutionMode','fixedSpacing','Name','MoveCheckTimer');
                else
                    try stop(obj.moveTimer); catch, end
                    obj.moveTimer.Period = 0.1;
                end
                obj.moveTimer.UserData.tStart = tic;
                start(obj.moveTimer);
                return
            else
                % blocking (legacy) behavior
                tStart = tic;
                while true
                    try
                        flush(obj.s);
                        obj.update_status_from_controller();
                    catch me
                        warning('moveToPosition:ReadFailed','Failed to read controller status: %s',me.message);
                        break
                    end
                    if obj.check_user_interrupt()
                        warning('moveToPosition:Interrupted','Move interrupted by user.');
                        break
                    end
                    curr = obj.current_pos(1:2);
                    dx = abs(double(curr(1)) - double(xTarget));
                    dy = abs(double(curr(2)) - double(yTarget));
                    if dx <= tol_steps && dy <= tol_steps
                        ok = true;
                        break
                    end
                    if toc(tStart) > timeout_s
                        try write(obj.s,command,'char'); catch, end
                        warning('moveToPosition:Timeout','Timeout waiting for position. dx=%d dy=%d',round(dx),round(dy));
                        break
                    end
                    pause(0.2);
                    try write(obj.s,command,'char'); catch, end
                end

                if ok
                    statusStr = 'Success';
                else
                    statusStr = 'Failed';
                end
                fprintf('moveToPosition: Target [%d,%d], Reached [%d,%d], tol=%d steps, %s\n', ...
                    round(xTarget),round(yTarget),curr(1),curr(2),tol_steps, statusStr);
            end

        end

        function ok = moveToPositionMM(obj, x_mm, y_mm, timeout_s, tol_mm, async)
            % Wrapper that accepts millimetres and converts to steps
            % ok = moveToPositionMM(obj, x_mm, y_mm, timeout_s, tol_mm, async)
            arguments
                obj
                x_mm (1,1) double
                y_mm (1,1) double
                timeout_s (1,1) double = 10
                tol_mm (1,1) double = 0.5 % default tolerance in mm
                async (1,1) logical = true
            end

            % convert to steps using obj.step2mm
            xSteps = round(double(x_mm) / obj.step2mm);
            ySteps = round(double(y_mm) / obj.step2mm);
            tol_steps = max(1, round(double(tol_mm) / obj.step2mm));

            ok = obj.moveToPosition(xSteps, ySteps, timeout_s, tol_steps, async);
        end

        function positionMoveCheckTick(obj, src, ~)
            % Timer callback for asynchronous checking if a position move is complete
            function reset_timer()
                try stop(src); catch, end
                try delete(src); catch, end
                obj.moveTimer = [];
            end
            try
                % read controller status once
                flush(obj.s);
                obj.update_status_from_controller();
                if(obj.check_user_interrupt())
                    reset_timer();
                    obj.move_status = 'INTERRUPTED';
                    fprintf('moveToPosition: Move interrupted by user.\n');
                    notify(obj,'MoveCompleted');
                    return
                end

                curr = obj.current_pos(1:2);
                dx = abs(double(curr(1)) - double(obj.move_target(1)));
                dy = abs(double(curr(2)) - double(obj.move_target(2)));
                if dx <= obj.move_tol_steps && dy <= obj.move_tol_steps
                    % reached target
                    reset_timer();
                    obj.move_status = 'SUCCESS';
                    fprintf('moveToPosition: Target [%d,%d], Reached [%d,%d], tol=%d steps, Success\n', ...
                        obj.move_target,curr,obj.move_tol_steps);
                    notify(obj,'MoveCompleted');
                    return
                end
                % check timeout
                tElapsed = toc(src.UserData.tStart);
                if tElapsed > obj.move_timeout_s
                    reset_timer();
                    obj.move_status = 'FAIL';
                    warning('moveToPosition:Timeout','Asynchronous move timed out.');
                    notify(obj,'MoveCompleted');
                    return
                end
                % resend PRE commands to keep previous move active
                try
                    write(obj.s,'PRE,PRE,PRE>','char');
                catch
                end
            catch me
                warning(me.identifier,'%s',me.message);
            end
        end

        function interrupted = check_user_interrupt(obj)
            % check_user_interrupt Check if user requested to stop current move
            % by pressing 'q' key. If so, send immediate stop command to controller.
            state = read(obj.kb);
            interrupted = false;
            if state.keys('q')
                obj.sendStopCommand();
                fprintf('User interrupt: Sent immediate stop command to controller.\n');
                interrupted = true;
            end
        end

        function [vx_out, vy_out] = limit_velocity_by_bounds(obj, vx_in, vy_in)
            % limit_velocity_by_bounds Zeroes velocity components if movement would
            % drive the carriage beyond configured x/y bounds (within margin).
            % vx_in, vy_in are in steps/sec. Returns possibly modified velocities.

            vx_out = vx_in;
            vy_out = vy_in;

            % convert to mm/sec
            vx_mm = vx_in * obj.step2mm;
            vy_mm = vy_in * obj.step2mm;

            % current position in mm
            pos = obj.real_loc; % [x y]
            margin = obj.boundary_margin_mm;

            % X axis
            if isfinite(obj.x_max_mm) && (pos(1) >= obj.x_max_mm - margin) && (vx_mm > 0)
                vx_out = 0;
            end
            if isfinite(obj.x_min_mm) && (pos(1) <= obj.x_min_mm + margin) && (vx_mm < 0)
                vx_out = 0;
            end

            % Y axis
            if isfinite(obj.y_max_mm) && (pos(2) >= obj.y_max_mm - margin) && (vy_mm > 0)
                vy_out = 0;
            end
            if isfinite(obj.y_min_mm) && (pos(2) <= obj.y_min_mm + margin) && (vy_mm < 0)
                vy_out = 0;
            end

            % If velocities were zeroed, also update cruise/current values to keep UI consistent
            % (callers assign returned values back into obj.vx_current / obj.vy_current)
        end

        function sendStopCommand(obj)
            % sendStopCommand Send immediate stop command to controller
            if isempty(obj.s)
                warning('sendStopCommand:NoSerial','Serial port not available.');
                return
            end
            obj.sendCommand('NUL0,NUL0,NUL0>');
        end

        function sendCommand(obj,cmd)
            if cmd(end) ~= '>'
                warning("Command %s not sent because missing end marker '>'\n",cmd);
                return
            end
            try
                write(obj.s,cmd,'char');
                pause(0.001);
                write(obj.s,cmd,'char');
                pause(0.001);
                write(obj.s,cmd,'char');
            catch me
                warning(me.identifier,'%s',me.message);
            end
        end

        function update_status_from_controller(obj)
            tStart = tic;
            position_received = false;
            nlines_received = 0;
            obj.prev_pos = obj.current_pos;
            % flush(obj.s); % clear any stale data, timing of this inconistent, do it outside
            while ~position_received
                while obj.s.NumBytesAvailable>0

                    txt = readline(obj.s);
                    fprintf(obj.ser_log_file,'%s\n',txt);
                    if contains(txt,'current position:')
                        nlines_received = nlines_received + 1;
                        obj.current_pos = sscanf(txt,'\tcurrent position: [%d,%d,%d,%d],%d.')'; % sscanf returns column vector
                        position_received = true;
                    end
                    if nlines_received>200
                        error(['Too many data in serial buffer.' newline ...
                               'Ensure to flush the buffer first and keep up command rate with the microcontroller.']);
                    end
                end
                % pause(0.005); % busy wait 
                if toc(tStart)>5
                    error('Controller communication timed out while getting motor positions.');
                end
            end
            
            obj.controller_time1 = obj.controller_time2;
            obj.controller_time2 = obj.current_pos(5);
            dt = obj.controller_time2 - obj.controller_time1; % in ms

            % compute real location and velocity (using controller time difference)
            obj.real_loc = (obj.current_pos(1:2) + obj.origin)*obj.step2mm;
            if dt ~= 0
                obj.real_vel = (obj.current_pos(1:2) - obj.prev_pos(1:2))*obj.step2mm/dt;
            end

            % compute frame_time using wall-clock since last status update
            now_dt = datetime('now');
            if isempty(obj.last_status_time)
                frame_time_ms = NaN; % first update, no prior timestamp
            else
                frame_time_ms = seconds(now_dt - obj.last_status_time) * 1000;
            end
            obj.last_status_time = now_dt;

            % Update status and velocity display
            t_str = ms_to_hms_string(obj.current_pos(5));
            set(obj.hStatusText, 'Text', sprintf('Status: Pos=[%5d,%5d,%5d,%5d], t=%s', obj.current_pos(1:4), t_str));

            % aoa degrees from current velocities; guard against division by zero
            if any(isnan(obj.real_vel)) || all(obj.real_vel==0)
                aoa_d = 0;
            else
                aoa = atan2(obj.real_vel(2), obj.real_vel(1));
                aoa_d = round(aoa/pi*180);
            end

            fmt = ['Speed: max %5d, real [%4.2f,%4.2f,%4.2f], AOA %4d°\n' ...
                   'Loc: [%5.1f,%5.1f], Frame time:%5.1f'];
            try
                set(obj.hVelText, 'Text', ...
                sprintf(fmt, obj.vel_max, obj.real_vel, norm(obj.real_vel), aoa_d, ...
                         obj.real_loc, frame_time_ms));
            catch
                % ignore UI update errors
            end

        end

    end
end
