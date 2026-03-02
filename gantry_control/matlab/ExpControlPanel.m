classdef ExpControlPanel < handle
    % ExpControlPanel UI to configure and start an experiment coordinating two carriages
    events
        PathPath
        PathHuman
        PathAgentPre
        PathAgentLive
    end
    properties
        Parent
        Grid
        UIFigure
        Vel1Field
        Vel2Field
        DelayField
        TagField
        % buttons for various path modes
        PathPathBtn
        PathHumanBtn
        PathAgentPreBtn
        PathAgentLiveBtn
    end
    methods
        function obj = ExpControlPanel(parent)
            arguments
                parent = []
            end
            if isempty(parent)
                obj.UIFigure = uifigure('Name','Experiment Control','Position',[200 200 360 220]);
                obj.Parent = obj.UIFigure;
                parent = obj.UIFigure;
            else
                obj.Parent = parent;
            end

            % Use a grid layout to arrange controls
            obj.Grid = uigridlayout(parent,[6,2]);
            obj.Grid.RowHeight = {'1x','1x','1x','1x','1x','1x'};
            obj.Grid.ColumnWidth = {'2x','1x'};

            % Velocity fields (row 1)
            lbl1 = uilabel(obj.Grid,'Text','Velocity Carriage 1 (m/s):');
            lbl1.Layout.Row = 1; lbl1.Layout.Column = 1;
            obj.Vel1Field = uieditfield(obj.Grid,'numeric','Value',0.3);
            obj.Vel1Field.Layout.Row = 1; obj.Vel1Field.Layout.Column = 2;

            % Velocity fields (row 2)
            lbl2 = uilabel(obj.Grid,'Text','Velocity Carriage 2 (m/s):');
            lbl2.Layout.Row = 2; lbl2.Layout.Column = 1;
            obj.Vel2Field = uieditfield(obj.Grid,'numeric','Value',0.3);
            obj.Vel2Field.Layout.Row = 2; obj.Vel2Field.Layout.Column = 2;

            % Delay (row 3)
            lbl3 = uilabel(obj.Grid,'Text','Delay start (s):');
            lbl3.Layout.Row = 3; lbl3.Layout.Column = 1;
            obj.DelayField = uieditfield(obj.Grid,'numeric','Value',1.0);
            obj.DelayField.Layout.Row = 3; obj.DelayField.Layout.Column = 2;

            % Tag (row 4)
            lbl4 = uilabel(obj.Grid,'Text','Run tag:');
            lbl4.Layout.Row = 4; lbl4.Layout.Column = 1;
            obj.TagField = uieditfield(obj.Grid,'text','Value','test');
            obj.TagField.Layout.Row = 4; obj.TagField.Layout.Column = 2;

            % buttons for path modes (rows 5 & 6)
            obj.PathPathBtn = uibutton(obj.Grid,'push','Text','Path Path',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathPathPressed());
            obj.PathPathBtn.Layout.Row = 5; obj.PathPathBtn.Layout.Column = 1;

            obj.PathHumanBtn = uibutton(obj.Grid,'push','Text','Path Human',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathHumanPressed());
            obj.PathHumanBtn.Layout.Row = 5; obj.PathHumanBtn.Layout.Column = 2;

            obj.PathAgentPreBtn = uibutton(obj.Grid,'push','Text','Path Agent Pre',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathAgentPrePressed());
            obj.PathAgentPreBtn.Layout.Row = 6; obj.PathAgentPreBtn.Layout.Column = 1;

            obj.PathAgentLiveBtn = uibutton(obj.Grid,'push','Text','Path Agent Live',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathAgentLivePressed());
            obj.PathAgentLiveBtn.Layout.Row = 6; obj.PathAgentLiveBtn.Layout.Column = 2;
        end

        function onPathPathPressed(obj)
            % notify listeners that the path‑path routine should start
            try
                notify(obj,'PathPath');
            catch
            end
        end

        function onPathHumanPressed(obj)
            try
                notify(obj,'PathHuman');
            catch
            end
        end

        function onPathAgentPrePressed(obj)
            try
                notify(obj,'PathAgentPre');
            catch
            end
        end

        function onPathAgentLivePressed(obj)
            try
                notify(obj,'PathAgentLive');
            catch
            end
        end

        function [v1,v2,delay_s,tag] = getParameters(obj)
            % Return configured parameters
            v1 = obj.Vel1Field.Value;
            v2 = obj.Vel2Field.Value;
            delay_s = obj.DelayField.Value;
            tag = obj.getTag();
        end

        function tag = getTag(obj)
            % Return the tag string
            if isprop(obj,'TagField') && ~isempty(obj.TagField)
                tag = char(obj.TagField.Value);
            else
                tag = '';
            end
        end
    end
end
