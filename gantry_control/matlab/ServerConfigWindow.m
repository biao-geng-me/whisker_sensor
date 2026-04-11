classdef ServerConfigWindow < handle
    % ServerConfigWindow UI for configuring and launching the Python agent server

    properties
        UIFigure
        Grid

        ModeDropdown
        EpisodesField
        HpcPortField
        RecordTrajectoryCheck
        PolicyDropdown

        StartBtn
        ShutdownBtn
        BrowseBtn
        RefreshBtn
        StatusLabel

        StartCallback
        ShutdownCallback
        PythonRoot
        PolicyOptions
    end

    methods
        function obj = ServerConfigWindow(startCallback, shutdownCallback, defaults)
            if nargin < 1
                startCallback = [];
            end
            if nargin < 2
                shutdownCallback = [];
            end
            if nargin < 3 || isempty(defaults)
                defaults = struct();
            end

            obj.StartCallback = startCallback;
            obj.ShutdownCallback = shutdownCallback;

            obj.PythonRoot = obj.readDefault(defaults, 'python_root', fullfile(pwd, 'gantry_control', 'python'));
            obj.PolicyOptions = obj.readDefault(defaults, 'policy_options', {'agents/hardware_handoff_v2'});
            selectedPolicy = obj.readDefault(defaults, 'policy_package_dir', obj.PolicyOptions{1});

            obj.UIFigure = uifigure('Name', 'Server Config', 'Position', [280 180 520 330]);
            obj.Grid = uigridlayout(obj.UIFigure, [8, 3]);
            obj.Grid.RowHeight = {'fit', 'fit', 'fit', 'fit', 'fit', '1x', 'fit', 'fit'};
            obj.Grid.ColumnWidth = {'1x', '1.2x', 'fit'};

            lblMode = uilabel(obj.Grid, 'Text', 'Mode');
            lblMode.Layout.Row = 1;
            obj.ModeDropdown = uidropdown(obj.Grid, 'Items', {'infer', 'train'}, ...
                'Value', obj.readDefault(defaults, 'mode', 'infer'));
            obj.ModeDropdown.Layout.Row = 1;
            obj.ModeDropdown.Layout.Column = [2 3];

            lblEpisodes = uilabel(obj.Grid, 'Text', 'Number of episodes');
            lblEpisodes.Layout.Row = 2;
            obj.EpisodesField = uieditfield(obj.Grid, 'numeric', ...
                'Value', obj.readDefault(defaults, 'max_episodes', 1), ...
                'RoundFractionalValues', 'on', 'LowerLimit', 1, 'LowerLimitInclusive', 'on');
            obj.EpisodesField.Layout.Row = 2;
            obj.EpisodesField.Layout.Column = [2 3];

            lblHpc = uilabel(obj.Grid, 'Text', 'HPC port');
            lblHpc.Layout.Row = 3;
            obj.HpcPortField = uieditfield(obj.Grid, 'numeric', ...
                'Value', obj.readDefault(defaults, 'hpc_port', 5555), ...
                'RoundFractionalValues', 'on', 'LowerLimit', 1, 'UpperLimit', 65535);
            obj.HpcPortField.Layout.Row = 3;
            obj.HpcPortField.Layout.Column = [2 3];

            lblPolicy = uilabel(obj.Grid, 'Text', 'Policy package');
            lblPolicy.Layout.Row = 4;
            obj.PolicyDropdown = uidropdown(obj.Grid, ...
                'Items', obj.PolicyOptions, ...
                'Editable', 'on', ...
                'Value', selectedPolicy);
            obj.PolicyDropdown.Layout.Row = 4;
            obj.PolicyDropdown.Layout.Column = 2;

            obj.BrowseBtn = uibutton(obj.Grid, 'push', 'Text', 'Browse...', ...
                'ButtonPushedFcn', @(~,~) obj.onBrowsePolicy());
            obj.BrowseBtn.Layout.Row = 4;
            obj.BrowseBtn.Layout.Column = 3;

            obj.RefreshBtn = uibutton(obj.Grid, 'push', 'Text', 'Refresh models', ...
                'ButtonPushedFcn', @(~,~) obj.onRefreshPolicies());
            obj.RefreshBtn.Layout.Row = 5;
            obj.RefreshBtn.Layout.Column = 3;

            obj.RecordTrajectoryCheck = uicheckbox(obj.Grid, ...
                'Text', 'Record trajectory', ...
                'Value', obj.readDefault(defaults, 'record_trajectory', false));
            obj.RecordTrajectoryCheck.Layout.Row = 5;
            obj.RecordTrajectoryCheck.Layout.Column = [1 2];

            obj.StatusLabel = uilabel(obj.Grid, 'Text', 'Server is not running.', 'WordWrap', 'on');
            obj.StatusLabel.Layout.Row = 7;
            obj.StatusLabel.Layout.Column = [1 3];

            obj.StartBtn = uibutton(obj.Grid, 'push', 'Text', 'Start', ...
                'ButtonPushedFcn', @(~,~) obj.onStartPressed());
            obj.StartBtn.Layout.Row = 8;
            obj.StartBtn.Layout.Column = 2;

            obj.ShutdownBtn = uibutton(obj.Grid, 'push', 'Text', 'Shutdown', ...
                'ButtonPushedFcn', @(~,~) obj.onShutdownPressed(), ...
                'Enable', 'off');
            obj.ShutdownBtn.Layout.Row = 8;
            obj.ShutdownBtn.Layout.Column = 3;
        end

        function setServerRunning(obj, isRunning, message)
            if nargin < 3 || isempty(message)
                if isRunning
                    message = 'Server is running.';
                else
                    message = 'Server is not running.';
                end
            end

            obj.ShutdownBtn.Enable = obj.onoff(isRunning);
            obj.StatusLabel.Text = message;
        end
    end

    methods (Access = private)
        function onStartPressed(obj)
            config = obj.collectConfig();
            if isempty(obj.StartCallback)
                obj.setServerRunning(false, 'Start callback is not configured.');
                return
            end

            try
                [ok, msg] = obj.StartCallback(config);
            catch me
                ok = false;
                msg = sprintf('Failed to start server: %s', me.message);
            end

            if ok
                obj.setServerRunning(true, msg);
            else
                obj.setServerRunning(false, msg);
            end
        end

        function onShutdownPressed(obj)
            if isempty(obj.ShutdownCallback)
                obj.setServerRunning(false, 'Shutdown callback is not configured.');
                return
            end

            try
                [ok, msg] = obj.ShutdownCallback();
            catch me
                ok = false;
                msg = sprintf('Failed to shut down server: %s', me.message);
            end

            if ok
                obj.setServerRunning(false, msg);
            else
                obj.StatusLabel.Text = msg;
            end
        end

        function onBrowsePolicy(obj)
            startDir = fullfile(obj.PythonRoot, 'agents');
            if ~isfolder(startDir)
                startDir = obj.PythonRoot;
            end

            selectedDir = uigetdir(startDir, 'Select policy package directory');
            if isequal(selectedDir, 0)
                return
            end

            relPolicy = obj.makePolicyRelative(selectedDir);
            obj.ensurePolicyOption(relPolicy);
            obj.PolicyDropdown.Value = relPolicy;
        end

        function onRefreshPolicies(obj)
            options = obj.discoverPolicies();
            obj.PolicyOptions = options;
            obj.PolicyDropdown.Items = options;
            if ~any(strcmp(options, obj.PolicyDropdown.Value))
                obj.PolicyDropdown.Value = options{1};
            end
            obj.StatusLabel.Text = sprintf('Found %d policy package option(s).', numel(options));
        end

        function config = collectConfig(obj)
            config = struct();
            config.mode = char(string(obj.ModeDropdown.Value));
            config.max_episodes = max(1, round(double(obj.EpisodesField.Value)));
            config.hpc_port = max(1, min(65535, round(double(obj.HpcPortField.Value))));
            config.record_trajectory = logical(obj.RecordTrajectoryCheck.Value);
            config.policy_package_dir = char(string(obj.PolicyDropdown.Value));
        end

        function ensurePolicyOption(obj, value)
            if isempty(value)
                return
            end
            if ~any(strcmp(obj.PolicyOptions, value))
                obj.PolicyOptions{end+1} = value;
                obj.PolicyDropdown.Items = obj.PolicyOptions;
            end
        end

        function relPath = makePolicyRelative(obj, selectedDir)
            pyRoot = obj.PythonRoot;
            selectedDir = strrep(selectedDir, '/', filesep);
            pyRoot = strrep(pyRoot, '/', filesep);
            pyRootNorm = [char(java.io.File(pyRoot).getCanonicalPath()), filesep];
            selectedNorm = char(java.io.File(selectedDir).getCanonicalPath());

            if startsWith(selectedNorm, pyRootNorm, 'IgnoreCase', true)
                relPath = selectedNorm(length(pyRootNorm)+1:end);
                relPath = strrep(relPath, filesep, '/');
            else
                relPath = selectedNorm;
            end
        end

        function options = discoverPolicies(obj)
            agentsDir = fullfile(obj.PythonRoot, 'agents');
            if ~isfolder(agentsDir)
                options = {'agents/hardware_handoff_v2'};
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
                options = {'agents/hardware_handoff_v2'};
            else
                options = sort(names);
            end
        end

        function value = readDefault(~, defaults, fieldName, fallback)
            if isstruct(defaults) && isfield(defaults, fieldName)
                value = defaults.(fieldName);
                if isempty(value)
                    value = fallback;
                end
            else
                value = fallback;
            end
        end

        function s = onoff(~, tf)
            if tf
                s = 'on';
            else
                s = 'off';
            end
        end
    end
end
