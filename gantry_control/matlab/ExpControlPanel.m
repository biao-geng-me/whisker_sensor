classdef ExpControlPanel < handle
    % ExpControlPanel UI to configure and start an experiment coordinating two carriages
    events
        PathPath
        PathHuman
        PathAgentPre
        FilterConfigRequested
        ServerConfigRequested
        PathAgentTrain
    end
    properties
        Parent
        Grid
        UIFigure
        Vel1Field
        Vel2Field
        RotationStepField
        EpisodeTimeField
        SettleDelayField
        DelayField
        TagField
        % buttons for various path modes
        PathPathBtn
        PathHumanBtn
        PathAgentPreBtn
        PathAgentTrainBtn
        FilterConfigBtn
        ConfigServerBtn
    end
    methods
        function obj = ExpControlPanel(parent)
            arguments
                parent = []
            end
            if isempty(parent)
                obj.UIFigure = uifigure('Name','Experiment Control','Position',[200 200 360 260]);
                obj.Parent = obj.UIFigure;
                parent = obj.UIFigure;
            else
                obj.Parent = parent;
            end

            % Use a grid layout to arrange controls
            obj.Grid = uigridlayout(parent,[7,3]);
            obj.Grid.RowHeight = {'fit','fit','1x','1x','1x','1x','1x'};
            obj.Grid.ColumnWidth = {'1x','1x','1x'};

            % Velocity fields (row 1)
            lbl1 = uilabel(obj.Grid,'Text','Front Speed / RL Back X (m/s):', ...
                'Tooltip', sprintf(['Left field: front carriage path speed.\n', ...
                                    'Right field: fixed forward speed for the RL-controlled back carriage.\n', ...
                                    'RL lateral speed limit comes from the Python config file.']));
            lbl1.Layout.Row = 1; lbl1.Layout.Column = 1;
            obj.Vel1Field = uieditfield(obj.Grid,'numeric','Value',0.2, ...
                'Tooltip', sprintf(['Front carriage path speed.\n', ...
                                    'Also used as the front/object forward speed in RL reward geometry.']));
            obj.Vel1Field.Layout.Row = 1; obj.Vel1Field.Layout.Column = 2;
            
            obj.Vel2Field = uieditfield(obj.Grid,'numeric','Value',0.16, ...
                'Tooltip', sprintf(['Fixed forward speed for the RL-controlled back carriage\n', ...
                                    'in PathAgentPre / PathAgentTrain.\n', ...
                                    'The RL lateral speed limit is not taken from this field.']));
            obj.Vel2Field.Layout.Row = 1; obj.Vel2Field.Layout.Column = 3;

            lbl2 = uilabel(obj.Grid,'Text','Max Rotation Change / Control Step (deg):', ...
                'Tooltip', sprintf(['Maximum allowed change in implied whisker/carriage rotation\n', ...
                                    'between consecutive RL control steps.\n', ...
                                    'This is not an absolute angle limit.']));
            lbl2.Layout.Row = 2; lbl2.Layout.Column = [1 2];
            obj.RotationStepField = uieditfield(obj.Grid,'numeric','Value',2.0, ...
                'Limits',[0 Inf], ...
                'Tooltip', sprintf(['Per control-step rotation-change limit used by the RL command wrapper.\n', ...
                                    'Cumulative turning beyond this is still allowed.']));
            obj.RotationStepField.Layout.Row = 2; obj.RotationStepField.Layout.Column = 3;

            % Row 3: Episode time
            lbl3 = uilabel(obj.Grid,'Text','Episode Time (s):');
            lbl3.Layout.Row = 3; lbl3.Layout.Column = 1;
            % Col 2 (row 3) left empty for spacing
            obj.EpisodeTimeField = uieditfield(obj.Grid,'numeric','Value',20.0);
            obj.EpisodeTimeField.Layout.Row = 3; obj.EpisodeTimeField.Layout.Column = 3;

            % Row 4: Delay start
            lbl4 = uilabel(obj.Grid,'Text','Delay start (s):');
            lbl4.Layout.Row = 4; lbl4.Layout.Column = 1;
            obj.SettleDelayField = uieditfield(obj.Grid,'numeric','Value',13.0,...
                'Tooltip',sprintf(['Delay after both carriages move to start position.\n', ...
                                   ' It is to settle down water motion due to the initial movement.']));
            obj.SettleDelayField.Layout.Row = 4; obj.SettleDelayField.Layout.Column = 2;
            obj.DelayField = uieditfield(obj.Grid,'numeric','Value',3.0,...
                'Tooltip',sprintf(['Delay before agent control starts.\n', ...
                                   ' It is to make sure the sensor array starts in the wake.']));
            obj.DelayField.Layout.Row = 4; obj.DelayField.Layout.Column = 3;

            % Row 5: Run tag
            lbl5 = uilabel(obj.Grid,'Text','Run tag:');
            lbl5.Layout.Row = 5; lbl5.Layout.Column = 1;
            % Col 2 (row 5) left empty for spacing
            obj.TagField = uieditfield(obj.Grid,'text','Value','test');
            obj.TagField.Layout.Row = 5; obj.TagField.Layout.Column = [2 3];

            % Row 6: buttons for path modes
            obj.PathPathBtn = uibutton(obj.Grid,'push','Text','Path Path',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathPathPressed());
            obj.PathPathBtn.Layout.Row = 6; obj.PathPathBtn.Layout.Column = 2;

            obj.FilterConfigBtn = uibutton(obj.Grid,'push','Text','Config Filter',...
                'ButtonPushedFcn',@(btn,evt) obj.onFilterConfigPressed());
            obj.FilterConfigBtn.Layout.Row = 6; obj.FilterConfigBtn.Layout.Column = 1;

            obj.PathHumanBtn = uibutton(obj.Grid,'push','Text','Path Human',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathHumanPressed());
            obj.PathHumanBtn.Layout.Row = 6; obj.PathHumanBtn.Layout.Column = 3;

            % Row 7: RL buttons and server config
            obj.PathAgentPreBtn = uibutton(obj.Grid,'push','Text','Path Agent Pre',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathAgentPrePressed());
            obj.PathAgentPreBtn.Layout.Row = 7; obj.PathAgentPreBtn.Layout.Column = 2;

            obj.PathAgentTrainBtn = uibutton(obj.Grid,'push','Text','Path Agent Train',...
                'ButtonPushedFcn',@(btn,evt) obj.onPathAgentTrainPressed());
            obj.PathAgentTrainBtn.Layout.Row = 7; obj.PathAgentTrainBtn.Layout.Column = 3;

            obj.ConfigServerBtn = uibutton(obj.Grid,'push','Text','Config server',...
                'ButtonPushedFcn',@(btn,evt) obj.onServerConfigPressed());
            obj.ConfigServerBtn.Layout.Row = 7; obj.ConfigServerBtn.Layout.Column = 1;
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

        function onPathAgentTrainPressed(obj)
            try
                notify(obj,'PathAgentTrain');
            catch
            end
        end

        function onFilterConfigPressed(obj)
            try
                notify(obj,'FilterConfigRequested');
            catch
            end
        end

        function onServerConfigPressed(obj)
            try
                notify(obj,'ServerConfigRequested');
            catch
            end
        end

        function [v1,v2,delay_s,tag,episode_time_s,settle_delay_s,rotation_step_deg] = getParameters(obj)
            % Return configured parameters
            v1 = obj.Vel1Field.Value;
            v2 = obj.Vel2Field.Value;
            delay_s = obj.DelayField.Value;
            tag = obj.getTag();
            episode_time_s = obj.EpisodeTimeField.Value;
            settle_delay_s = obj.SettleDelayField.Value;
            rotation_step_deg = obj.RotationStepField.Value;
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
