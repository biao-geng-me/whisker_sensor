classdef GantryView < handle
    %GANTRYVIEW Visualize the motion of a xy(a) gantry
    %   Detailed explanation goes here
    
    properties
        UIFigure
        ax
        car1
        car2
    end
    
    methods
        function obj = GantryView(varargin)
            %GANTRYVIEW Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 0
                % Create separate UI figure.
                obj.UIFigure = uifigure;
                obj.UIFigure.Name = 'Gantry View App';
                % obj.UIFigure.Position = [100 100 300 150];
                parent = obj.UIFigure;
            elseif nargin == 1
                % create the UI in the parent container.
                parent = varargin{1};
            else
                error('Too many arguments.');
            end

            ax = uiaxes(parent);
            ax.Box = 'on';
            ax.Position = [50 50 540 270];
            ax.DataAspectRatio = [ 1 1 1];
            ax.XDir = "reverse";
            ax.YDir = "reverse";
            ax.XAxisLocation = "top";
            ax.YAxisLocation = "right";
            ax.XLim = [-500 4500]; % tank length mm
            ax.YLim = [-300 1200];
            disableDefaultInteractivity(ax);
            hold(ax,"on");
            rectangle(ax,'Position',[0,0,3800,840], ...
                        'EdgeColor','r',...
                        'LineWidth',2)
            L = 3600;
            H = 420;
            x = 0:L;
            y = -x.*(x-L)/L^2*4*H;
            y = y + H*sin(2*pi*x/1000).*y/max(y);
            plot(ax,x,y);
            
            car1 = CarriageTrack(ax);
            car1 = car1.set_xy(0,0);
            car1.set_lines_color([0.5 0.5 0.5]);
            car1.redraw;
            obj.car1 = car1;

            car2 = CarriageTrack(ax);
            car2 = car2.set_xy(500,1000);
            car2.set_lines_color([0.0 0.5 0.5]);
            car2.set_mark_color([0 0 0])
            car2.redraw;
            obj.car2 = car2;
            
            obj.ax = ax;


            if nargout == 0
                clear obj;
            end
        end

        function redraw(obj,x1,y1,x2,y2)
            obj.car1 = obj.car1.set_xy(x1,y1);
            obj.car2 = obj.car2.set_xy(x2,y2);

            obj.car1.redraw;
            obj.car2.redraw;
        end
        
    end
end

