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

        OutputDirField
        OutputDirBrowseBtn
        ResumeCheck
        ResumePathField
        ResumePathBrowseBtn
        KeepCheckpointsField
        CheckpointEveryField

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
            obj.PolicyOptions = obj.readDefault(defaults, 'policy_options', {'agents/rl_sac_v4_pathblind_hardware'});
            selectedPolicy = obj.readDefault(defaults, 'policy_package_dir', obj.PolicyOptions{1});

            obj.UIFigure = uifigure('Name', 'Server Config', 'Position', [280 180 520 520]);
            obj.Grid = uigridlayout(obj.UIFigure, [13, 3]);
            obj.Grid.RowHeight = {'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', '1x', 'fit', 'fit'};
            obj.Grid.ColumnWidth = {'1x', '1.2x', 'fit'};

            lblMode = uilabel(obj.Grid, 'Text', 'Mode');
            lblMode.Layout.Row = 1;
            obj.ModeDropdown = uidropdown(obj.Grid, 'Items', {'infer', 'train'}, ...
                'Value', obj.readDefault(defaults, 'mode', 'train'));
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

            % --- Checkpoint / resume controls ---
            lblOutput = uilabel(obj.Grid, 'Text', 'Output directory');
            lblOutput.Layout.Row = 6;
            defaultOutputDir = obj.readDefault(defaults, 'output_dir', fullfile(obj.PythonRoot, 'checkpoints'));
            obj.OutputDirField = uieditfield(obj.Grid, 'text', 'Value', defaultOutputDir);
            obj.OutputDirField.Layout.Row = 6;
            obj.OutputDirField.Layout.Column = 2;
            obj.OutputDirBrowseBtn = uibutton(obj.Grid, 'push', 'Text', 'Browse...', ...
                'ButtonPushedFcn', @(~,~) obj.onBrowseOutputDir());
            obj.OutputDirBrowseBtn.Layout.Row = 6;
            obj.OutputDirBrowseBtn.Layout.Column = 3;

            defaultLatestCkpt = fullfile(defaultOutputDir, 'latest_checkpoint.pt');
            defaultResumeExists = isfile(defaultLatestCkpt);
            obj.ResumeCheck = uicheckbox(obj.Grid, ...
                'Text', 'Resume from checkpoint', ...
                'Value', obj.readDefault(defaults, 'resume', defaultResumeExists));
            obj.ResumeCheck.Layout.Row = 7;
            obj.ResumeCheck.Layout.Column = [1 2];

            lblResume = uilabel(obj.Grid, 'Text', 'Resume checkpoint');
            lblResume.Layout.Row = 8;
            if defaultResumeExists
                defaultResumeFallback = defaultLatestCkpt;
            else
                defaultResumeFallback = '';
            end
            defaultResumePath = obj.readDefault(defaults, 'resume_path', defaultResumeFallback);
            obj.ResumePathField = uieditfield(obj.Grid, 'text', 'Value', defaultResumePath);
            obj.ResumePathField.Layout.Row = 8;
            obj.ResumePathField.Layout.Column = 2;
            obj.ResumePathBrowseBtn = uibutton(obj.Grid, 'push', 'Text', 'Browse...', ...
                'ButtonPushedFcn', @(~,~) obj.onBrowseResumePath());
            obj.ResumePathBrowseBtn.Layout.Row = 8;
            obj.ResumePathBrowseBtn.Layout.Column = 3;

            lblKeep = uilabel(obj.Grid, 'Text', 'Keep checkpoints');
            lblKeep.Layout.Row = 9;
            obj.KeepCheckpointsField = uieditfield(obj.Grid, 'numeric', ...
                'Value', obj.readDefault(defaults, 'keep_checkpoints', 5), ...
                'RoundFractionalValues', 'on', 'LowerLimit', 1, 'LowerLimitInclusive', 'on');
            obj.KeepCheckpointsField.Layout.Row = 9;
            obj.KeepCheckpointsField.Layout.Column = [2 3];

            lblCkptEvery = uilabel(obj.Grid, 'Text', 'Save every N episodes');
            lblCkptEvery.Layout.Row = 10;
            obj.CheckpointEveryField = uieditfield(obj.Grid, 'numeric', ...
                'Value', obj.readDefault(defaults, 'checkpoint_every_episodes', 1), ...
                'RoundFractionalValues', 'on', 'LowerLimit', 1, 'LowerLimitInclusive', 'on');
            obj.CheckpointEveryField.Layout.Row = 10;
            obj.CheckpointEveryField.Layout.Column = [2 3];

            obj.StatusLabel = uilabel(obj.Grid, 'Text', 'Server is not running.', 'WordWrap', 'on');
            obj.StatusLabel.Layout.Row = 12;
            obj.StatusLabel.Layout.Column = [1 3];

            obj.StartBtn = uibutton(obj.Grid, 'push', 'Text', 'Start', ...
                'ButtonPushedFcn', @(~,~) obj.onStartPressed());
            obj.StartBtn.Layout.Row = 13;
            obj.StartBtn.Layout.Column = 2;

            obj.ShutdownBtn = uibutton(obj.Grid, 'push', 'Text', 'Shutdown', ...
                'ButtonPushedFcn', @(~,~) obj.onShutdownPressed(), ...
                'Enable', 'off');
            obj.ShutdownBtn.Layout.Row = 13;
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

        function onBrowseOutputDir(obj)
            startDir = obj.OutputDirField.Value;
            if isempty(startDir) || ~isfolder(startDir)
                startDir = obj.PythonRoot;
            end
            selectedDir = uigetdir(startDir, 'Select output directory for checkpoints');
            if isequal(selectedDir, 0)
                return
            end
            obj.OutputDirField.Value = selectedDir;
        end

        function onBrowseResumePath(obj)
            startDir = obj.OutputDirField.Value;
            if isempty(startDir) || ~isfolder(startDir)
                startDir = obj.PythonRoot;
            end
            [file, path] = uigetfile({'*.pt', 'Checkpoint files (*.pt)'}, ...
                'Select checkpoint file to resume from', startDir);
            if isequal(file, 0)
                return
            end
            obj.ResumePathField.Value = fullfile(path, file);
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
            config.output_dir = char(string(obj.OutputDirField.Value));
            config.resume = logical(obj.ResumeCheck.Value);
            config.resume_path = char(string(obj.ResumePathField.Value));
            config.keep_checkpoints = max(1, round(double(obj.KeepCheckpointsField.Value)));
            config.checkpoint_every_episodes = max(1, round(double(obj.CheckpointEveryField.Value)));
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
