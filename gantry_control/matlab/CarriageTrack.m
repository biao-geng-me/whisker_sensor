classdef CarriageTrack < handle
    %CARRIAGETRACK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name = 'Carriage'; % name of the carriage
        ax  % parent ax
        x = 0.5;
        y = 0.5;
        vx = 0;
        vy =0;
        hor_line
        ver_line
        mark
        traj % trajectory plot handle
        big_num = 1e6;
        marker_symbol = '+';
        marker_size = 50;
        traj_marker_size = 10;
        istart_traj = 2;
        marker_lw = 3;
        nt = 500; % number of time points in the trajectory
        xt  % for trajectory
        yt  % for trajectory
        it = 0;
    end
    
    methods
        function obj = CarriageTrack(ax)
            %CARRIAGETRACK Construct an instance of this class
            %   Detailed explanation goes here
            obj.ax = ax;
            obj.init();
        end
        
        function set_xy(obj,x,y)
            obj.x = x;
            obj.y = y;
            obj.append_location(x,y);
        end

        function set_vel(obj,vx,vy)
            obj.vx = vx;
            obj.vy = vy;
        end

        function set_states(obj,x,y,vx,vy)
            obj.x = x;
            obj.y = y;
            obj.vx = vx;
            obj.vy = vy;
        end

        function init(obj)
            b = obj.big_num;
            obj.hor_line = line(obj.ax,[-b b],[obj.y, obj.y]);
            obj.ver_line = line(obj.ax,[obj.x, obj.x], [-b, b]);
            obj.hor_line.Annotation.LegendInformation.IconDisplayStyle = 'off';
            obj.ver_line.Annotation.LegendInformation.IconDisplayStyle = 'off';

            obj.xt = zeros(obj.nt,1); % for trajectory
            obj.yt = zeros(obj.nt,1); % for trajectory

            hold(obj.ax,'on');
            obj.mark = scatter(obj.ax,obj.x,obj.y,obj.marker_size, ...
                "red",obj.marker_symbol,...
                "MarkerEdgeColor",[1 0 0], ...
                "MarkerFaceColor",[1 0 0], ...
                "LineWidth",obj.marker_lw, ...
                "DisplayName",obj.name);

            obj.traj = scatter(obj.ax,obj.xt(1:end-obj.istart_traj), ...
                obj.yt(1:end-obj.istart_traj), ...
                obj.traj_marker_size, ...
                "red","filled","o",...
                "MarkerEdgeColor",'None', ...
                "MarkerFaceColor",[0.2 0.2 0.2]);
            obj.traj.Annotation.LegendInformation.IconDisplayStyle = 'off';
            alpha = 1:obj.nt;
            obj.traj.MarkerFaceAlpha = 'flat';
            obj.traj.AlphaData = alpha(1:end-obj.istart_traj);
        end

        function redraw(obj)
            obj.hor_line.YData = [obj.y obj.y];
            obj.ver_line.XData = [obj.x obj.x];
            obj.mark.XData = obj.x;
            obj.mark.YData = obj.y;

            obj.traj.XData = obj.xt(1:end-obj.istart_traj);
            obj.traj.YData = obj.yt(1:end-obj.istart_traj);
        end

        function set_lines_color(obj,rgb)
            obj.hor_line.Color = rgb;
            obj.ver_line.Color = rgb;
        end

        function set_mark_color(obj,rgb)
            obj.mark.MarkerEdgeColor = rgb;
            obj.mark.CData = rgb;
        end

        function obj=append_location(obj,x,y)
            obj.it = obj.it+1;
            if obj.it> obj.nt
                obj.it = obj.nt;
                obj.xt(1:end-1) = obj.xt(2:end);
                obj.yt(1:end-1) = obj.yt(2:end);
            end
            obj.xt(obj.it) = x;
            obj.yt(obj.it) = y;
        end
    end
end

