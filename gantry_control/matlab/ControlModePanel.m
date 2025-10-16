classdef ControlModePanel < handle
    properties
        Parent
        UIFigure % main app figure window
        Ax % uiaxes for path drawing
        PathObj % PathData instance for selected path
        Panel
        BG
        R1
        R2
        FileListBox
        FileBtn
        PlotBtn
        PlottedHandles % containers.Map: file -> graphics handle
        ClearBtn
        RemoveBtn
        HomeBtn
        PlayBtn
        InteractiveToggle % uibutton
        InteractiveLED %
        SelectedFiles
        DefaultPathFolder % optional folder to auto-load files from on startup
    end

    events
        StartPolling
        StopPolling

        StartPathtracking
        StopPathtracking
        HomeRequested
    end

    methods
        function app = ControlModePanel(parent, ax)
            arguments
                parent = [] % optional
                ax = [] % for drawing paths
            end
            % If no parent given, create standalone figure
            if isempty(parent)
                app.UIFigure = uifigure('Name','Control Mode','Position',[100 100 380 250]);
                app.Parent = app.UIFigure;
            else
                app.Parent = parent;
                app.UIFigure = ancestor(parent,'matlab.ui.Figure','toplevel');
            end

            % visualization
            if isempty(ax)
                app.Ax = create_ax();
            else
                app.Ax = ax;
            end

            % Wrap everything in a panel for tighter integration
            app.Panel = uipanel(app.Parent,'Title','Control mode');
            if ~isa(app.Parent,'matlab.ui.container.GridLayout')
                app.Panel.Position = [10 10 360 230];
            end

            % Button group inside panel
            app.BG = uibuttongroup(app.Panel,'Position',[10 150 200 50],...
                'BorderType','none',...
                'SelectionChangedFcn',@(bg,event) app.selectionChanged(bg,event));

            % Radio buttons (children of button group) with the same visual positions
            % as your tweaked layout (positions are relative to the uibuttongroup)
            app.R1 = uiradiobutton(app.BG,'Text','Interactive','Position',[5 25 120 20],...
                'Tooltip','Interactive mode: control the system using Keyboard (front) or Joystick (back)');
            app.R2 = uiradiobutton(app.BG,'Text','Pathtracking','Position',[5 0 120 20],...
                'Tooltip','Pathtracking mode: select one or more path data files from a folder');

            % Interactive controls (kept at your panel positions)
            app.InteractiveToggle = uibutton(app.Panel,'Push','Text','Activate','Position',[110 175 70 22],...
                'Enable','on',...
                'ButtonPushedFcn',@(btn,event) app.toggleInteractive());

            app.InteractiveLED = uilamp(app.Panel,'Position',[190 177 18 18],...
                'Color',[0.5 0.5 0.5]);

            % Pathtracking controls (kept at your panel positions)
            app.PlayBtn = uibutton(app.Panel,'Text','▶ Start','Position',[110 150 70 22],...
                'Enable','off',...
                'ButtonPushedFcn',@(btn,event) app.togglePlay());

            % File list (scrollable)
            app.FileListBox = uilistbox(app.Panel,'Position',[10 60 340 80],...
                'Enable','off','Items',{},...
                'Multiselect','on',...
                'ValueChangedFcn',@(lb,event) app.fileSelectionChanged(event));

            % Browse and remove buttons
            app.FileBtn = uibutton(app.Panel,'Text','Browse...','Position',[10 25 80 25],...
                'Enable','off',...
                'ButtonPushedFcn',@(btn,event) app.browseFiles());

            % Plot button next to Browse - disabled until a file is selected
            app.PlotBtn = uibutton(app.Panel,'Text','Plot','Position',[100 25 50 25],...
                'Enable','off',...
                'ButtonPushedFcn',@(btn,event) app.plotSelectedFiles());

            % Clear plots button
            app.ClearBtn = uibutton(app.Panel,'Text','Clear','Position',[160 25 50 25],...
                'Enable','off',...
                'ButtonPushedFcn',@(btn,event) app.clearPlots());

            app.RemoveBtn = uibutton(app.Panel,'Text','❌','Position',[220 25 40 25],...
                'Enable','off',...
                'ButtonPushedFcn',@(btn,event) app.removeSelected());

            % Home button to return carriage to origin (0,0)
            app.HomeBtn = uibutton(app.Panel,'Text','Home','Position',[265 25 60 25],...
                'Enable','off',...
                'ButtonPushedFcn',@(btn,event) app.onHomePressed());

            % Initialize file list
            app.SelectedFiles = {};

            % default folder can be set externally before constructing or
            % defaults to a subfolder called 'Paths' if present
            app.DefaultPathFolder = fullfile(pwd,'Paths');
            % attempt to load files from default folder
            try
                app.loadFilesFromDefaultFolder();
            catch
                % ignore failures; panel will start empty
            end

            % Map to track plotted graphics per file (avoid duplicates)
            app.PlottedHandles = containers.Map('KeyType','char','ValueType','any');

            % Ensure LED starts grey
            app.InteractiveLED.Color = [0.5 0.5 0.5];

            % Optional: set a default selected mode (none) or choose one
            % bg.SelectedObject = app.R1; % uncomment to default to Interactive
        end

        function selectionChanged(app,~,event)
            % Grey/disable controls for unselected modes. Note: uilamp has no 'Enable' property
            % so we change its color to indicate disabled state.

            % Disable interactive controls by default
            app.InteractiveToggle.Enable = 'off';
            app.InteractiveLED.Color = [0.5 0.5 0.5];

            % Disable pathtracking controls by default
            app.FileListBox.Enable = 'off';
            app.FileBtn.Enable = 'off';
            app.RemoveBtn.Enable = 'off';
            app.PlotBtn.Enable = 'off';
            app.PlayBtn.Enable = 'off';

            % Enable relevant controls based on selection
            switch event.NewValue.Text
                case 'Interactive'
                    app.InteractiveToggle.Enable = 'on';
                    % keep LED color according to whether interactive is active or not
                    if strcmp(app.InteractiveToggle.Text,'Deactivate')
                        app.InteractiveLED.Color = [0 1 0];
                    else
                        app.InteractiveLED.Color = [0.5 0.5 0.5];
                    end
                case 'Pathtracking'
                    % deactivate input timer
                    if strcmp(app.InteractiveToggle.Text,'Deactivate')
                        % todo: stop timer
                        app.InteractiveToggle.Text = 'Activate';
                        app.InteractiveLED.Color = [0.5 0.5 0.5];
                    end

                    % enable file list
                    app.FileListBox.Enable = 'on';
                    app.FileBtn.Enable = 'on';
                    app.RemoveBtn.Enable = 'on';
                    app.HomeBtn.Enable = 'on';
                    % Play (start pathtracking) is only enabled when exactly one file is selected
                    if isscalar(app.FileListBox.Value)
                        app.PlayBtn.Enable = 'on';
                    else
                        app.PlayBtn.Enable = 'off';
                    end
                    % If there's already a selection, enable Plot button
                    if ~isempty(app.FileListBox.Value)
                        app.PlotBtn.Enable = 'on';
                    else
                        app.PlotBtn.Enable = 'off';
                    end
            end
        end

        function browseFiles(app)
            [files, path] = uigetfile({'*.*','All Files'}, 'Select files', 'MultiSelect', 'on');
            if isequal(files,0)
                figure(app.UIFigure);
                return; % user canceled
            end

            if ischar(files)
                files = {files}; % wrap single file into cell array
            end

            % Avoid duplicates
            newFiles = fullfile(path, files);
            app.SelectedFiles = unique([app.SelectedFiles, newFiles],'stable');

            % Update listbox items (filenames only)
            [~,names,exts] = cellfun(@fileparts, app.SelectedFiles, 'UniformOutput', false);
            app.FileListBox.Items = strcat(names, exts);
            % no plots yet for newly added files -> clear button off
            app.ClearBtn.Enable = 'off';
            
            figure(app.UIFigure);
        end

        function removeSelected(app)
            idx = app.FileListBox.Value;
            if isempty(idx), return; end

            items = app.FileListBox.Items;
            toRemove = ismember(items, idx);
            % store removed full paths so we can delete any plotted graphics
            removedPaths = app.SelectedFiles(toRemove);
            app.SelectedFiles(toRemove) = [];

            % remove plotted graphics for removed files
            for k = 1:numel(removedPaths)
                fp = removedPaths{k};
                if isKey(app.PlottedHandles, fp)
                    h = app.PlottedHandles(fp);
                    try
                        if isgraphics(h)
                            delete(h);
                        end
                    catch me
                        warning('Failed to delete plotted handle for %s: %s', fp, me.message);
                    end
                    remove(app.PlottedHandles, fp);
                end
            end

            [~,names,exts] = cellfun(@fileparts, app.SelectedFiles, 'UniformOutput', false);
            app.FileListBox.Items = strcat(names, exts);

            % Clear selection and disable Plot button when items removed
            app.FileListBox.Value = {};
            app.PlotBtn.Enable = 'off';
            % disable clear button if no plotted handles remain
            if isempty(keys(app.PlottedHandles))
                app.ClearBtn.Enable = 'off';
            end
        end

        function fileSelectionChanged(app, event)
            % Enable Plot button only when something is selected
            sel = event.Value;
            if isempty(sel)
                app.PlotBtn.Enable = 'off';
            else
                app.PlotBtn.Enable = 'on';
            end
            % Play button is enabled only when exactly one file is selected
            if isscalar(sel)
                app.PlayBtn.Enable = 'on';
            else
                app.PlayBtn.Enable = 'off';
            end

        end

        function plotSelectedFiles(app)
            % Load selected filenames from the listbox and plot XY data
            selItems = app.FileListBox.Value;
            if isempty(selItems)
                uialert(app.UIFigure,'No file selected to plot.','Plot');
                return;
            end

            % Map selected display names back to full paths in SelectedFiles
            % Build a mapping of display name -> full file
            [~,names,exts] = cellfun(@fileparts, app.SelectedFiles, 'UniformOutput', false);
            displayNames = strcat(names, exts);

            selectedPaths = {};
            for i=1:numel(selItems)
                idx = find(strcmp(displayNames, selItems{i}),1);
                if ~isempty(idx)
                    selectedPaths{end+1} = app.SelectedFiles{idx}; %#ok<AGROW>
                end
            end

            if isempty(selectedPaths)
                uialert(app.UIFigure,'Selected files not found.','Plot');
                return;
            end

            hold(app.Ax,'on');
            for k = 1:numel(selectedPaths)
                file = selectedPaths{k};

                % skip if already plotted
                if isKey(app.PlottedHandles, file)
                    % already plotted, skip to avoid duplicates
                    continue;
                end

                % files contain simple x y data loadable with load()
                try
                    data = load(file);
                catch me
                    warning('Could not load file: %s (%s)',file, me.message);
                    continue;
                end

                % data might be a struct if file contains variables, or a numeric matrix
                if isstruct(data)
                    % take first numeric field
                    fn = fieldnames(data);
                    val = data.(fn{1});
                else
                    val = data;
                end

                if isnumeric(val) && size(val,2) >= 2
                    x = val(:,1);
                    y = val(:,2);
                    [~,nm,~] = fileparts(file);
                    h = plot(app.Ax,x,y,'DisplayName',sprintf('%s',nm));
                    % store handle so we can delete it later
                    try
                        app.PlottedHandles(file) = h;
                        % enable Clear button when we have at least one plotted handle
                        app.ClearBtn.Enable = 'on';
                    catch me
                        warning('Could not store plotted handle for %s: %s', file, me.message);
                    end
                else
                    warning('File does not contain Nx2 numeric data: %s',file);
                end
            end
            lg = legend(app.Ax,'show');
            try
                lg.Interpreter = 'none';
            catch
                % in case legend handle doesn't support Interpreter in some contexts
            end
            hold(app.Ax,'off');

        end

        function pd = createPathDataFromSelection(app)
            % Return a PathData object built from the single selected file.
            selItems = app.FileListBox.Value;
            if isempty(selItems) || numel(selItems)~=1
                error('ControlModePanel:InvalidSelection','Exactly one file must be selected to create PathData.');
            end

            % map display name to full path
            [~,names,exts] = cellfun(@fileparts, app.SelectedFiles, 'UniformOutput', false);
            displayNames = strcat(names, exts);
            idx = find(strcmp(displayNames, selItems{1}),1);
            if isempty(idx)
                error('ControlModePanel:FileNotFound','Selected file not found in internal list.');
            end

            fullpath = app.SelectedFiles{idx};
            % create PathData
            pd = PathData(fullpath);
            app.PathObj = pd;
        end

        function clearPlots(app)
            % Delete all plotted graphics and clear the map
            keysList = keys(app.PlottedHandles);
            for i = 1:numel(keysList)
                k = keysList{i};
                h = app.PlottedHandles(k);
                try
                    if isgraphics(h)
                        delete(h);
                    end
                catch me
                    warning('Failed to delete plotted handle for %s: %s', k, me.message);
                end
            end
            remove(app.PlottedHandles, keysList);
        end

        function togglePlay(app)
            if strcmp(app.PlayBtn.Text,'▶ Start')
                app.PlayBtn.Text = '■ Stop';
                % notify listeners that pathtracking should start
                notify(app,'StartPathtracking');
            else
                app.PlayBtn.Text = '▶ Start';
                % notify listeners that pathtracking should stop
                notify(app,'StopPathtracking');
            end
        end

        function toggleInteractive(app)
            if strcmp(app.InteractiveToggle.Text,'Activate')
                app.turnOnInteractive();
                notify(app,'StartPolling');
            else
                app.turnOffInteractive();
                notify(app,'StopPolling');
            end
        end

        function turnOffInteractive(app)
            app.InteractiveToggle.Text = 'Activate';
            app.InteractiveLED.Color = [0.5 0.5 0.5];
        end

        function turnOnInteractive(app)
            app.InteractiveToggle.Text = 'Deactivate';
            app.InteractiveLED.Color = [0 1 0];
        end

        function onHomePressed(app)
            % User pressed Home button. Notify listeners and try a best-effort
            % direct call to a Car object if available as app.Parent.App or similar.
            try
                % Notify listeners first
                notify(app,'HomeRequested');
            catch
            end
        end

        function loadFilesFromDefaultFolder(app)
            % loadFilesFromDefaultFolder Populate the file list from DefaultPathFolder
            if isempty(app.DefaultPathFolder) || ~isfolder(app.DefaultPathFolder)
                return
            end

            % look for common data file types
            files = [dir(fullfile(app.DefaultPathFolder,'*.mat')); dir(fullfile(app.DefaultPathFolder,'*.dat')); dir(fullfile(app.DefaultPathFolder,'*.csv'))];
            if isempty(files)
                return
            end

            fullpaths = fullfile({files.folder}, {files.name});
            app.SelectedFiles = fullpaths;

            % Update listbox items (filenames only)
            [~,names,exts] = cellfun(@fileparts, app.SelectedFiles, 'UniformOutput', false);
            app.FileListBox.Items = strcat(names, exts);
            % enable plot and browse controls since we have files
            % app.FileListBox.Enable = 'on';
            % app.FileBtn.Enable = 'on';
            % app.PlotBtn.Enable = 'on';
        end
    end
end
