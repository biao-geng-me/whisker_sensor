classdef ExpControlPanel < handle
    % ExpControlPanel UI to configure and start an experiment coordinating two carriages
    events
        StartExperiment
    end
    properties
        Parent
        Grid
        UIFigure
        Vel1Field
        Vel2Field
        DelayField
        StartBtn
    end
    methods
        function obj = ExpControlPanel(parent)
            arguments
                parent = []
            end
            if isempty(parent)
                obj.UIFigure = uifigure('Name','Experiment Control','Position',[200 200 360 180]);
                obj.Parent = obj.UIFigure;
                parent = obj.UIFigure;
            else
                obj.Parent = parent;
            end

            % Use a grid layout to arrange controls
            obj.Grid = uigridlayout(parent,[4,2]);
            obj.Grid.RowHeight = {'1x','1x','1x','1x'};
            obj.Grid.ColumnWidth = {'2x','1x'};

            % Velocity fields (row 1)
            lbl1 = uilabel(obj.Grid,'Text','Velocity Carriage 1 (m/s):');
            lbl1.Layout.Row = 1; lbl1.Layout.Column = 1;
            obj.Vel1Field = uieditfield(obj.Grid,'numeric','Value',0.2);
            obj.Vel1Field.Layout.Row = 1; obj.Vel1Field.Layout.Column = 2;

            % Velocity fields (row 2)
            lbl2 = uilabel(obj.Grid,'Text','Velocity Carriage 2 (m/s):');
            lbl2.Layout.Row = 2; lbl2.Layout.Column = 1;
            obj.Vel2Field = uieditfield(obj.Grid,'numeric','Value',0.2);
            obj.Vel2Field.Layout.Row = 2; obj.Vel2Field.Layout.Column = 2;

            % Delay (row 3)
            lbl3 = uilabel(obj.Grid,'Text','Delay start (s):');
            lbl3.Layout.Row = 3; lbl3.Layout.Column = 1;
            obj.DelayField = uieditfield(obj.Grid,'numeric','Value',5);
            obj.DelayField.Layout.Row = 3; obj.DelayField.Layout.Column = 2;

            % Start button spans the two columns in row 4
            obj.StartBtn = uibutton(obj.Grid,'push','Text','Start Experiment',...
                'ButtonPushedFcn',@(btn,evt) obj.onStartPressed());
            obj.StartBtn.Layout.Row = 4; obj.StartBtn.Layout.Column = [1 2];
        end

        function onStartPressed(obj)
            % notify listeners that the experiment should start
            try
                notify(obj,'StartExperiment');
            catch
            end
        end

        function [v1,v2,delay_s] = getParameters(obj)
            % Return configured parameters
            v1 = obj.Vel1Field.Value;
            v2 = obj.Vel2Field.Value;
            delay_s = obj.DelayField.Value;
        end
    end
end
