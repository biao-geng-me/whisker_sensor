classdef FilterTester < handle
% FilterTester  Interactive real-time filter tester for small block processing (1-4 samples).
%
% Usage:
%   app = FilterTester();   % launch the GUI
%
% Features:
%   - Load CSV/MAT or use a built-in synthetic test signal
%   - Select filter type: moving-average, lowpass-IIR, highpass-IIR, biquad-lowpass
%   - Tune cutoff, order, block size (1-4), and output gain
%   - Apply one block at a time or run a real-time simulation
%   - Run simulation scrolls the time window right-to-left with a shaded
%     "peek" region showing upcoming (future) samples
%   - Freeze / unfreeze the x-axis
%   - Metrics table: MSE, max error, RMS raw/filtered, estimated latency, SNR
%   - Auto-tune: sets a sensible 2nd-order Butterworth lowpass
%   - Export filter coefficients as MATLAB code or C arrays (copied to clipboard)
%   - Generate a standalone real-time loop script with latency logging
%   - Block consistency validation: verifies block sizes 1 and 4 produce identical output

	properties (Access = private)
		% --- figure & UI handles ---
		fig            % uifigure
		% controls
		ddData         % uidropdown  â€“ data source
		btnLoad        % uibutton   â€“ load file
		efFs           % uieditfield(numeric) â€“ sample rate
		efDuration     % uieditfield(numeric) â€“ signal duration
		ddFilter       % uidropdown  â€“ filter type
		efOrder        % uieditfield(numeric) â€“ filter order
		efCutoff       % uieditfield(numeric) â€“ low-pass cutoff (Hz)
		efHighpass     % uieditfield(numeric) â€“ high-pass cutoff (Hz)
		ddBlock        % uidropdown  â€“ block size
		efGain         % uieditfield(numeric) â€“ output gain
		efPeek         % uieditfield(numeric) â€“ peek window (s)
		btnReset       % uibutton
		btnStep        % uibutton
		btnRun         % uibutton  (acts as toggle via isRunning flag)
		chkFreeze      % uicheckbox
		btnAutoTune    % uibutton
		btnExport      % uibutton  â€“ export as MATLAB coefficients
		btnExportC     % uibutton  â€“ export as C arrays
		btnGenLoop     % uibutton  â€“ generate real-time loop script
		btnValidate    % uibutton  – block consistency validation
		btnSynthConfig % uibutton  – configure synthetic signal or datalog channel
		lblStatus      % uilabel
		tblMetrics     % uitable
		% axes
		axTime         % Raw vs Filtered (time domain)
		axResidual     % Residual signal
		axPSD          % Power spectral density + theoretical magnitude response
		axPhase        % Phase response

		% --- application state ---
		fs        = 80
		t                   % time vector (column)
		raw                 % raw signal (column)
		filtered            % filtered output (column)
		residual            % residual = raw - filtered (column)
		idx       = 1       % next sample index to process
		blockSize = 1
		isRunning = false
		filterType = 'lowpass-iir'
		order     = 3
		cutoffHz  = 8
		highpassHz = 0.2
		gain      = 1.0
		peekSec   = 0.5     % peek-ahead shading window (seconds)
		a         = 1
		b         = 1
		zi                  % filter state vector
		realtimeUiPeriodSec = 0.05
		realtimeHeavyPlotPeriodSec = 0.5
		freezeX   = false
		xlimFrozen = [0, 3]
		% Synthetic signal components: each row = [amplitude, frequency_Hz, phase_rad, noise_amp]
		synthComponents   % Nx4 matrix
		loadedDataType = 'synthetic'
		loadedSourcePath = ''
		datalogTable = table()
		datalogDtStr = ''
		datalogTag = ''
		datalogChannelNames = {}
		datalogSelectedChannel = 1
		csvData = []
		csvChannelNames = {}
		csvSelectedChannel = 1
	end

	methods
		function app = FilterTester()
			app.initState();
			app.buildUi();
			app.updateDataSourceControls();
			app.resetFilterState();
			app.replayAll();
		end
	end

	% =========================================================================
	%  Initialisation
	% =========================================================================
	methods (Access = private)

		function initState(app)
			% Default components mirror the original hard-coded signal:
			%  amp   freq(Hz)  phase(rad)  noise_amp
			app.synthComponents = [
				0.80,  1.8,  0,     0;     % low-freq swell
				0.35,  7.0,  0,     0;     % useful band
				0.20, 35.0,  0,     0.12;  % high-freq noise
				0.08,  0.15, 0,     0;     % slow drift
			];
			app.loadedDataType = 'synthetic';
			dur      = 10;
			app.t    = (0 : 1/app.fs : dur).';
			app.raw  = FilterTester.synthSignal(app.t, app.synthComponents);
			app.filtered = zeros(size(app.raw));
			app.residual = app.raw;
			app.idx  = 1;
			app.xlimFrozen = [app.t(1), app.t(min(numel(app.t), app.fs * 3))];
		end

		% =====================================================================
		%  UI construction  (uifigure + uigridlayout for resize-friendly layout)
		% =====================================================================
		function buildUi(app)
			app.fig = uifigure('Name', 'FilterTester', ...
				'Position', [50 50 1400 820], ...
				'Color', [0.96 0.96 0.96], ...
				'WindowState', 'maximized', ...
				'CloseRequestFcn', @(~,~) app.onClose());

			% Top-level grid: narrow fixed controls column | stretching plots column
			figGrid = uigridlayout(app.fig, [1 2], ...
				'ColumnWidth', {400, '1x'}, ...
				'RowHeight',   {'1x'}, ...
				'Padding',     [8 8 8 8], ...
				'ColumnSpacing', 8, ...
				'BackgroundColor', [0.96 0.96 0.96]);

			% ------ left control panel ----------------------------------------
			cp = uipanel(figGrid, 'Title', 'Controls', ...
				'BackgroundColor', [0.96 0.96 0.96]);
			cp.Layout.Row = 1;  cp.Layout.Column = 1;

			% Control grid inside panel:
			%   20 rows × 2 columns (label | field).
			%   Rows 1-19 are fixed height; row 20 (metrics table) stretches.
			rH = [repmat({26}, 1, 19), {'1x'}];
			cpGrid = uigridlayout(cp, [20 2], ...
				'RowHeight',    rH, ...
				'ColumnWidth',  {'1x', 100}, ...
				'Padding',      [6 6 6 6], ...
				'RowSpacing',   4, ...
				'Scrollable',   'on', ...
				'BackgroundColor', [0.96 0.96 0.96]);

			% helper: place a label spanning both columns (section header)
			function lbl = hdr(row, txt)
				lbl = uilabel(cpGrid, 'Text', txt, 'FontWeight', 'bold');
				lbl.Layout.Row = row;  lbl.Layout.Column = [1 2];
			end
			% helper: label in col-1
			function lbl = lbl1(row, txt)
				lbl = uilabel(cpGrid, 'Text', txt);
				lbl.Layout.Row = row;  lbl.Layout.Column = 1;
			end

			% Row 1 – Data source header
			hdr(1, 'Data source');

			% Row 2 – data source dropdown (spans both cols)
			app.ddData = uidropdown(cpGrid, ...
				'Items', {'Synthetic signal', 'Load file (CSV / MAT / DAT)'}, ...
				'ValueChangedFcn', @(~,~) app.onDataSourceChange());
			app.ddData.Layout.Row = 2;  app.ddData.Layout.Column = [1 2];

			% Row 3 – Configure/Tune (synthetic) | Load file
			app.btnSynthConfig = uibutton(cpGrid, 'Text', 'Configure / Tune', ...
				'ButtonPushedFcn', @(~,~) app.onSynthConfig());
			app.btnSynthConfig.Layout.Row = 3;  app.btnSynthConfig.Layout.Column = 1;
			app.btnLoad = uibutton(cpGrid, 'Text', 'Load file...', ...
				'ButtonPushedFcn', @(~,~) app.onLoadData(), 'Enable', 'off');
			app.btnLoad.Layout.Row = 3;  app.btnLoad.Layout.Column = 2;

			% Row 4 – sample rate
			lbl1(4, 'Sample rate (Hz)');
			app.efFs = uieditfield(cpGrid, 'numeric', 'Value', app.fs, ...
				'Limits', [2 100000], ...
				'ValueChangedFcn', @(~,~) app.onSignalSettingsChange());
			app.efFs.Layout.Row = 4;  app.efFs.Layout.Column = 2;

			% Row 5 – duration
			lbl1(5, 'Duration (s)');
			app.efDuration = uieditfield(cpGrid, 'numeric', 'Value', 10, ...
				'Limits', [0.1 3600], ...
				'ValueChangedFcn', @(~,~) app.onSignalSettingsChange());
			app.efDuration.Layout.Row = 5;  app.efDuration.Layout.Column = 2;

			% Row 6 – Filter type header
			hdr(6, 'Filter type');

			% Row 7 – filter dropdown (spans both cols)
			app.ddFilter = uidropdown(cpGrid, ...
				'Items', {'moving-average', 'lowpass-iir', 'highpass-iir', 'biquad-lowpass'}, ...
				'ValueChangedFcn', @(~,~) app.onFilterParamChange());
			app.ddFilter.Layout.Row = 7;  app.ddFilter.Layout.Column = [1 2];

			% Row 8 – order
			lbl1(8, 'Order');
			app.efOrder = uieditfield(cpGrid, 'numeric', 'Value', app.order, ...
				'Limits', [1 20], 'RoundFractionalValues', true, ...
				'ValueChangedFcn', @(~,~) app.onFilterParamChange());
			app.efOrder.Layout.Row = 8;  app.efOrder.Layout.Column = 2;

			% Row 9 – cutoff
			lbl1(9, 'Cutoff (Hz)');
			app.efCutoff = uieditfield(cpGrid, 'numeric', 'Value', app.cutoffHz, ...
				'Limits', [0.001 1e6], ...
				'ValueChangedFcn', @(~,~) app.onFilterParamChange());
			app.efCutoff.Layout.Row = 9;  app.efCutoff.Layout.Column = 2;

			% Row 10 – highpass
			lbl1(10, 'Highpass (Hz)');
			app.efHighpass = uieditfield(cpGrid, 'numeric', 'Value', app.highpassHz, ...
				'Limits', [0.001 1e6], ...
				'ValueChangedFcn', @(~,~) app.onFilterParamChange());
			app.efHighpass.Layout.Row = 10;  app.efHighpass.Layout.Column = 2;

			% Row 11 – block size
			lbl1(11, 'Block size');
			app.ddBlock = uidropdown(cpGrid, 'Items', {'1','2','3','4'}, ...
				'ValueChangedFcn', @(~,~) app.onBlockChange());
			app.ddBlock.Layout.Row = 11;  app.ddBlock.Layout.Column = 2;

			% Row 12 – output gain
			lbl1(12, 'Output gain');
			app.efGain = uieditfield(cpGrid, 'numeric', 'Value', app.gain, ...
				'Limits', [0.001 1000], ...
				'ValueChangedFcn', @(~,~) app.onFilterParamChange());
			app.efGain.Layout.Row = 12;  app.efGain.Layout.Column = 2;

			% Row 13 – peek window
			lbl1(13, 'Peek window (s)');
			app.efPeek = uieditfield(cpGrid, 'numeric', 'Value', app.peekSec, ...
				'Limits', [0 60]);
			app.efPeek.Layout.Row = 13;  app.efPeek.Layout.Column = 2;

			% Row 14 – Reset | Apply one step
			app.btnReset = uibutton(cpGrid, 'Text', 'Reset', ...
				'ButtonPushedFcn', @(~,~) app.onReset());
			app.btnReset.Layout.Row = 14;  app.btnReset.Layout.Column = 1;
			app.btnStep = uibutton(cpGrid, 'Text', 'Apply one step', ...
				'ButtonPushedFcn', @(~,~) app.onStep());
			app.btnStep.Layout.Row = 14;  app.btnStep.Layout.Column = 2;

			% Row 15 – Run simulation | Freeze x
			app.btnRun = uibutton(cpGrid, 'Text', 'Run simulation', ...
				'ButtonPushedFcn', @(~,~) app.onRunToggle());
			app.btnRun.Layout.Row = 15;  app.btnRun.Layout.Column = 1;
			app.chkFreeze = uicheckbox(cpGrid, 'Text', 'Freeze x', ...
				'Value', false, 'ValueChangedFcn', @(~,~) app.onFreezeToggle());
			app.chkFreeze.Layout.Row = 15;  app.chkFreeze.Layout.Column = 2;

			% Row 16 – Auto-tune | Validate blocks
			app.btnAutoTune = uibutton(cpGrid, 'Text', 'Auto-tune LPF', ...
				'ButtonPushedFcn', @(~,~) app.onAutoTune());
			app.btnAutoTune.Layout.Row = 16;  app.btnAutoTune.Layout.Column = 1;
			app.btnValidate = uibutton(cpGrid, 'Text', 'Validate blocks', ...
				'ButtonPushedFcn', @(~,~) app.onValidateBlocks());
			app.btnValidate.Layout.Row = 16;  app.btnValidate.Layout.Column = 2;

			% Row 17 – Export MATLAB | Export C
			app.btnExport = uibutton(cpGrid, 'Text', 'Export MATLAB coeffs', ...
				'ButtonPushedFcn', @(~,~) app.onExport());
			app.btnExport.Layout.Row = 17;  app.btnExport.Layout.Column = 1;
			app.btnExportC = uibutton(cpGrid, 'Text', 'Export C arrays', ...
				'ButtonPushedFcn', @(~,~) app.onExportC());
			app.btnExportC.Layout.Row = 17;  app.btnExportC.Layout.Column = 2;

			% Row 18 – Generate loop script (spans both cols)
			app.btnGenLoop = uibutton(cpGrid, 'Text', 'Generate real-time loop script', ...
				'ButtonPushedFcn', @(~,~) app.onGenerateLoopScript());
			app.btnGenLoop.Layout.Row = 18;  app.btnGenLoop.Layout.Column = [1 2];

			% Row 19 – Status label (spans both cols)
			app.lblStatus = uilabel(cpGrid, 'Text', 'Status: Ready', ...
				'WordWrap', 'on', 'FontColor', [0.2 0.4 0.7]);
			app.lblStatus.Layout.Row = 19;  app.lblStatus.Layout.Column = [1 2];

			% Row 20 – Metrics table (spans both cols, stretches vertically)
			app.tblMetrics = uitable(cpGrid, ...
				'ColumnName', {'Metric', 'Value'}, ...
				'ColumnWidth', {'fit', 'fit'}, ...
				'RowName', {}, ...
				'Data', cell(6, 2));
			app.tblMetrics.Layout.Row = 20;  app.tblMetrics.Layout.Column = [1 2];

			% ------ right plot panel (2×2 tiled layout) ----------------------
			pp = uipanel(figGrid, 'Title', 'Plots', ...
				'BackgroundColor', [1 1 1]);
			pp.Layout.Row = 1;  pp.Layout.Column = 2;

			tl = tiledlayout(pp, 2, 2, ...
				'TileSpacing', 'compact', 'Padding', 'compact');

			app.axTime     = nexttile(tl, 1);
			app.axResidual = nexttile(tl, 2);
			app.axPSD      = nexttile(tl, 3);
			app.axPhase    = nexttile(tl, 4);
		end

		% =====================================================================
		%  Event callbacks
		% =====================================================================

		function onClose(app)
			app.isRunning = false;
			delete(app.fig);
		end

		function onDataSourceChange(app)
			app.updateDataSourceControls();
		end

		function onLoadData(app)
			if strcmp(app.ddData.Value, 'Synthetic signal')
				app.setStatus('Switch to the load-file source before choosing a file');
				return;
			end

			[fileName, pathName] = uigetfile( ...
				{'*.csv;*.mat;*.dat;*.txt', 'CSV, MAT, or sensor datalog files'; ...
				 '*.csv', 'CSV files'; ...
				 '*.mat', 'MAT files'; ...
				 '*.dat;*.txt', 'Sensor datalog files'; ...
				 '*.*', 'All files'}, ...
				'Load data file');
			if isequal(fileName, 0), return; end

			fullPath = fullfile(pathName, fileName);
			[~, ~, ext] = fileparts(fullPath);

			try
				if strcmpi(ext, '.csv')
					app.clearDatalogState();
					app.loadCsvSource(fullPath);
				elseif strcmpi(ext, '.mat')
					s    = load(fullPath);
					vars = fieldnames(s);
					if isempty(vars), error('MAT file has no variables.'); end
					signal = s.(vars{1});
					signal = signal(:);
					app.clearDatalogState();
					app.clearCsvState();
					app.loadedDataType = 'mat';
					app.loadedSourcePath = fullPath;
					app.applyLoadedSignal(signal, [], max(1, app.efFs.Value));
					app.setStatus(sprintf('Loaded %s  (%d samples)', fileName, numel(app.raw)));
				elseif any(strcmpi(ext, {'.dat', '.txt'}))
					app.clearCsvState();
					app.loadDatalogSource(fullPath);
				else
					error('Unsupported file type: %s', ext);
				end
			catch me
				app.updateDataSourceControls();
				app.setStatus(['Load error: ', me.message]);
			end
		end

		function onSignalSettingsChange(app)
			if ~strcmp(app.ddData.Value, 'Synthetic signal'), return; end
			fs  = app.efFs.Value;
			dur = app.efDuration.Value;
			app.fs       = fs;
			app.t        = (0 : 1/fs : dur).';
			app.raw      = FilterTester.synthSignal(app.t, app.synthComponents);
			app.filtered = zeros(size(app.raw));
			app.residual = app.raw;
			app.idx      = 1;
			app.loadedDataType = 'synthetic';
			app.loadedSourcePath = '';
			app.xlimFrozen = [app.t(1), app.t(min(numel(app.t), app.fs * 3))];
			app.resetFilterState();
			app.replayAll();
			app.updateDataSourceControls();
			app.setStatus('Synthetic signal regenerated');
		end

		function onFilterParamChange(app)
			app.filterType  = app.ddFilter.Value;
			app.order       = max(1, min(20, round(app.efOrder.Value)));
			app.cutoffHz    = min(max(app.efCutoff.Value,   0.001), app.fs * 0.48);
			app.highpassHz  = min(max(app.efHighpass.Value, 0.001), app.fs * 0.45);
			app.gain        = app.efGain.Value;
			% clamp displayed values
			app.efOrder.Value    = app.order;
			app.efCutoff.Value   = app.cutoffHz;
			app.efHighpass.Value = app.highpassHz;
			app.replayAll();
			app.setStatus('Filter updated and replayed');
		end

		function onBlockChange(app)
			app.blockSize = str2double(app.ddBlock.Value);
			app.replayAll();
			app.setStatus(sprintf('Block size = %d', app.blockSize));
		end

		function onReset(app)
			app.filtered  = zeros(size(app.raw));
			app.residual  = app.raw;
			app.idx       = 1;
			app.isRunning = false;
			app.btnRun.Text = 'Run simulation';
			app.resetFilterState();
			app.refreshAll();
			app.setStatus('Reset stream state');
		end

		function onStep(app)
			app.applyNextBlock();
		end

		function onRunToggle(app)
			if app.isRunning
				app.isRunning   = false;
				app.btnRun.Text = 'Run simulation';
				app.setStatus('Simulation paused');
			else
				app.isRunning   = true;
				app.btnRun.Text = 'Stop';
				app.setStatus('Running real-time simulation...');
				app.runLoop();
			end
		end

		function onFreezeToggle(app)
			app.freezeX = app.chkFreeze.Value;
			if app.freezeX
				app.xlimFrozen = xlim(app.axTime);
				app.setStatus('X-axis frozen');
			else
				app.setStatus('X-axis unfrozen');
			end
			app.refreshAll();
		end

		function onAutoTune(app)
			app.ddFilter.Value  = 'lowpass-iir';
			app.efOrder.Value   = 2;
			app.efCutoff.Value  = max(0.5, round(app.fs * 0.06, 3));
			app.onFilterParamChange();
			app.setStatus('Auto-tune applied: 2nd-order Butterworth lowpass');
		end

		function onExport(app)
			[b, a] = FilterTester.design_filter( ...
				app.filterType, app.fs, app.order, app.cutoffHz, app.highpassHz);
			txt = sprintf('%% Filter: %s  fs=%.6g Hz  cutoff=%.6g Hz  order=%d\nb = [%s];\na = [%s];\n', ...
				app.filterType, app.fs, app.cutoffHz, app.order, ...
				num2str(b, '%.12g '), num2str(a, '%.12g '));
			disp(txt);
			try
				clipboard('copy', txt);
				app.setStatus('MATLAB coefficients exported to workspace + clipboard');
			catch
				app.setStatus('MATLAB coefficients exported to command window');
			end
		end

		function onExportC(app)
			[b, a] = FilterTester.design_filter( ...
				app.filterType, app.fs, app.order, app.cutoffHz, app.highpassHz);
			bStr = strtrim(sprintf('%.12g, ', b));  bStr = bStr(1:end-1);
			aStr = strtrim(sprintf('%.12g, ', a));  aStr = aStr(1:end-1);
			txt = sprintf( ...
				['/* Filter: %s  fs=%.6g Hz  cutoff=%.6g Hz  order=%d */\n' ...
				 '#define FILTER_B_LEN  %d\n' ...
				 '#define FILTER_A_LEN  %d\n' ...
				 'static const double filter_b[FILTER_B_LEN] = { %s };\n' ...
				 'static const double filter_a[FILTER_A_LEN] = { %s };\n'], ...
				app.filterType, app.fs, app.cutoffHz, app.order, ...
				numel(b), numel(a), bStr, aStr);
			disp(txt);
			try
				clipboard('copy', txt);
				app.setStatus('C arrays exported to command window + clipboard');
			catch
				app.setStatus('C arrays exported to command window');
			end
		end

		function onGenerateLoopScript(app)
			scriptPath = fullfile(fileparts(mfilename('fullpath')), ...
				'FilterTester_real_time_loop.m');
			fid = fopen(scriptPath, 'w');
			if fid < 0
				app.setStatus('Could not create real-time loop script');
				return;
			end
			lines = { ...
				'function FilterTester_real_time_loop(raw, fs)'; ...
				'% Auto-generated real-time loop template from FilterTester.'; ...
				'% Processes samples in small blocks and logs per-block latency.'; ...
				'%'; ...
				'% Usage:'; ...
				'%   FilterTester_real_time_loop()          built-in test signal'; ...
				'%   FilterTester_real_time_loop(sig, fs)   your own data'; ...
				'if nargin < 2, fs = 200; end'; ...
				'if nargin < 1'; ...
				'    t   = (0:1/fs:10).'';'; ...
				'    raw = sin(2*pi*2*t) + 0.4*sin(2*pi*30*t) + 0.05*randn(size(t));'; ...
				'end'; ...
				['b         = ' mat2str(app.b, 8) ';']; ...
				['a         = ' mat2str(app.a, 8) ';']; ...
				['blockSize = ' num2str(app.blockSize) ';']; ...
				['gain      = ' num2str(app.gain) ';']; ...
				'zi        = zeros(max(length(a), length(b)) - 1, 1);'; ...
				'out       = zeros(size(raw));'; ...
				'idx       = 1;'; ...
				'latMs     = zeros(ceil(numel(raw) / blockSize), 1);'; ...
				'k         = 0;'; ...
				'while idx <= numel(raw)'; ...
				'    i2      = min(idx + blockSize - 1, numel(raw));'; ...
				'    t0      = tic;'; ...
				'    [y, zi] = filter(b, a, raw(idx:i2), zi);'; ...
				'    k       = k + 1; latMs(k) = toc(t0) * 1e3;'; ...
				'    out(idx:i2) = gain * y;'; ...
				'    pause((i2 - idx + 1) / fs);  % simulate sample rate'; ...
				'    idx = i2 + 1;'; ...
				'end'; ...
				'latMs = latMs(1:k);'; ...
				'fprintf(''Mean block latency: %.4f ms   (n = %d blocks)\n'', mean(latMs), k);'; ...
				'figure;'; ...
				'tVec = (0:numel(raw)-1)'' / fs;'; ...
				'plot(tVec, raw, ''Color'', [0.7 0.7 0.7]); hold on;'; ...
				'plot(tVec, out, ''b'', ''LineWidth'', 1.4);'; ...
				'legend(''Raw'', ''Filtered''); xlabel(''Time (s)''); ylabel(''Amplitude'');'; ...
				'title(''FilterTester real-time loop output'');'; ...
				'end'; ...
			};
			for k = 1:numel(lines)
				fprintf(fid, '%s\n', lines{k});
			end
			fclose(fid);
			app.setStatus(['Generated: ', scriptPath]);
		end

		function onSynthConfig(app)
			if ~strcmp(app.ddData.Value, 'Synthetic signal')
				if strcmp(app.loadedDataType, 'csv')
					app.configureCsvChannel();
				else
					app.configureDatalogChannel();
				end
				return;
			end

			% Open a modal-style uifigure for editing the synthetic signal components.
			dlg = uifigure('Name', 'Configure Synthetic Signal', ...
				'Position', [200 200 580 420], ...
				'WindowStyle', 'Normal', ...
				'Resize', 'on');

			g = uigridlayout(dlg, [3 1], ...
				'RowHeight', {'1x', 32, 32}, ...
				'Padding', [10 10 10 10], 'RowSpacing', 6);

			% ---- editable table of components --------------------------------
			tbl = uitable(g, ...
				'ColumnName',     {'Amplitude', 'Frequency (Hz)', 'Phase (rad)', 'Noise amp'}, ...
				'ColumnEditable', [true true true true], ...
				'ColumnWidth',    {110, 130, 110, 100}, ...
				'RowName',        {}, ...
				'Data',           num2cell(app.synthComponents));
			tbl.Layout.Row = 1;  tbl.Layout.Column = 1;

			% ---- Add / Remove row buttons ------------------------------------
			btnGrid = uigridlayout(g, [1 2], ...
				'ColumnWidth', {'1x','1x'}, 'Padding', [0 0 0 0], 'RowSpacing', 0);
			btnGrid.Layout.Row = 2;

			uibutton(btnGrid, 'Text', '+ Add component', ...
				'ButtonPushedFcn', @(~,~) addRow());
			uibutton(btnGrid, 'Text', '- Remove last', ...
				'ButtonPushedFcn', @(~,~) removeRow());

			% ---- Apply / Close buttons ---------------------------------------
			applyGrid = uigridlayout(g, [1 2], ...
				'ColumnWidth', {'1x','1x'}, 'Padding', [0 0 0 0], 'RowSpacing', 0);
			applyGrid.Layout.Row = 3;

			uibutton(applyGrid, 'Text', 'Apply', ...
				'ButtonPushedFcn', @(~,~) applyComponents());
			uibutton(applyGrid, 'Text', 'Close', ...
				'ButtonPushedFcn', @(~,~) delete(dlg));

			% ---- nested helpers -------------------------------------------
			function addRow()
				d = tbl.Data;
				tbl.Data = [d; {0.1, 10.0, 0.0, 0.0}];
			end

			function removeRow()
				d = tbl.Data;
				if size(d, 1) > 1
					tbl.Data = d(1:end-1, :);
				end
			end

			function applyComponents()
				d = tbl.Data;
				if isempty(d)
					app.setStatus('No components defined — keeping previous signal.');
					return;
				end
				% Convert cell to numeric, filling blanks with 0
				mat = zeros(size(d));
				for r = 1:size(d,1)
					for c = 1:size(d,2)
						v = d{r,c};
						if isnumeric(v) && isfinite(v)
							mat(r,c) = v;
						end
					end
				end
				app.synthComponents = mat;
				% Regenerate signal
				fs  = app.efFs.Value;
				dur = app.efDuration.Value;
				app.fs       = fs;
				app.t        = (0 : 1/fs : dur).';
				app.raw      = FilterTester.synthSignal(app.t, app.synthComponents);
				app.filtered = zeros(size(app.raw));
				app.residual = app.raw;
				app.idx      = 1;
				app.loadedDataType = 'synthetic';
				app.loadedSourcePath = '';
				app.xlimFrozen = [app.t(1), app.t(min(numel(app.t), app.fs * 3))];
				app.resetFilterState();
				app.replayAll();
				app.updateDataSourceControls();
				app.setStatus(sprintf('Signal updated: %d component(s)', size(mat,1)));
			end
		end

		function onValidateBlocks(app)
			% Verifies that block sizes 1 and 4 produce numerically identical output.
			app.setStatus('Validating block consistency...');
			drawnow;
			[b, a] = FilterTester.design_filter( ...
				app.filterType, app.fs, app.order, app.cutoffHz, app.highpassHz);
			raw  = app.raw;
			zLen = max(length(a), length(b)) - 1;

			zi1  = zeros(zLen, 1);  out1 = zeros(size(raw));  idx = 1;
			while idx <= numel(raw)
				i2 = idx;
				[y, zi1] = filter(b, a, raw(idx:i2), zi1);
				out1(idx:i2) = app.gain * y;  idx = i2 + 1;
			end

			zi4  = zeros(zLen, 1);  out4 = zeros(size(raw));  idx = 1;
			while idx <= numel(raw)
				i2 = min(idx + 3, numel(raw));
				[y, zi4] = filter(b, a, raw(idx:i2), zi4);
				out4(idx:i2) = app.gain * y;  idx = i2 + 1;
			end

			maxDiff = max(abs(out1 - out4));
			if maxDiff < 1e-10
				app.setStatus(sprintf('Block validation PASSED  (max diff = %.2e)', maxDiff));
			else
				app.setStatus(sprintf('Block validation FAILED  (max diff = %.2e) â€” check filter stability', maxDiff));
			end
		end

	end   % event-callback methods

	% =========================================================================
	%  Core processing methods
	% =========================================================================
	methods (Access = private)

		function runLoop(app)
			ticStart = tic;
			nextUiUpdate = 0;
			nextHeavyUpdate = 0;
			while app.isRunning && isvalid(app.fig)
				done = app.applyNextBlock(false);

				wallElapsed = toc(ticStart);
				doUiUpdate = (wallElapsed >= nextUiUpdate) || done;
				if doUiUpdate
					doHeavyUpdate = (wallElapsed >= nextHeavyUpdate) || done;
					app.refreshAll(doHeavyUpdate);
					nextUiUpdate = wallElapsed + app.realtimeUiPeriodSec;
					if doHeavyUpdate
						nextHeavyUpdate = wallElapsed + app.realtimeHeavyPlotPeriodSec;
					end
				end

				if done
					app.isRunning   = false;
					app.btnRun.Text = 'Run simulation';
					app.setStatus('Simulation complete');
					break;
				end

				simElapsed = (app.idx - 1) / app.fs;
				lagSec = simElapsed - toc(ticStart);
				if lagSec > 0
					pause(min(lagSec, app.realtimeUiPeriodSec));
				else
					drawnow limitrate;
				end
			end
		end

		function done = applyNextBlock(app, doRefresh)
			if nargin < 2
				doRefresh = true;
			end

			if app.idx > numel(app.raw)
				done = true;
				if doRefresh
					app.refreshAll();
				end
				return;
			end
			idxEnd  = min(app.idx + app.blockSize - 1, numel(app.raw));
			samples = app.raw(app.idx : idxEnd);
			[y, app.zi] = filter(app.b, app.a, samples, app.zi);
			y = app.gain * y;
			app.filtered(app.idx : idxEnd) = y;
			app.residual(app.idx : idxEnd) = app.raw(app.idx : idxEnd) - y;
			app.idx = idxEnd + 1;
			if doRefresh
				app.refreshAll();
			end
			done = app.idx > numel(app.raw);
		end

		function replayAll(app)
			app.resetFilterState();
			app.filtered = zeros(size(app.raw));
			app.residual = app.raw;
			tmpIdx = 1;
			while tmpIdx <= numel(app.raw)
				idxEnd  = min(tmpIdx + app.blockSize - 1, numel(app.raw));
				samples = app.raw(tmpIdx : idxEnd);
				[y, app.zi] = filter(app.b, app.a, samples, app.zi);
				app.filtered(tmpIdx : idxEnd) = app.gain * y;
				app.residual(tmpIdx : idxEnd) = app.raw(tmpIdx : idxEnd) - app.gain * y;
				tmpIdx = idxEnd + 1;
			end
			app.idx = numel(app.raw) + 1;   % mark all as processed
			app.resetFilterState();
			app.refreshAll();
		end

		function resetFilterState(app)
			[b, a] = FilterTester.design_filter( ...
				app.filterType, app.fs, app.order, app.cutoffHz, app.highpassHz);
			app.b  = b;
			app.a  = a;
			app.zi = zeros(max(length(a), length(b)) - 1, 1);
		end

		function refreshAll(app, doHeavyPlots)
			if nargin < 2
				doHeavyPlots = true;
			end
			app.plotResponses(doHeavyPlots);
			app.updateMetrics();
			drawnow limitrate;
		end

		% =====================================================================
		%  Plotting
		% =====================================================================
		function plotResponses(app, doHeavyPlots)
			if nargin < 2
				doHeavyPlots = true;
			end

			t        = app.t;
			raw      = app.raw;
			filtered = app.filtered;
			fs       = app.fs;
			b        = app.b;
			a        = app.a;

			nReady = find(filtered ~= 0, 1, 'last');
			if isempty(nReady), nReady = 0; end

			% Determine the visible x window.
			% In run-simulation mode the window scrolls so the current sample
			% sits at 80% of the window width; a shaded peek region is shown
			% to the right of the current position.
			winSec  = max(3, app.efPeek.Value * 4);
			peekSec = app.efPeek.Value;
			if app.isRunning && nReady > 0
				tCurrent = t(min(nReady, numel(t)));
				xLo = tCurrent - winSec * 0.8;
				xHi = xLo + winSec;
			else
				xLo = t(1);
				xHi = t(end);
			end
			if app.freezeX
				xLo = app.xlimFrozen(1);
				xHi = app.xlimFrozen(2);
			end

			% ---- panel 1: Raw vs Filtered -----------------------------------
			ax = app.axTime;
			cla(ax);
			hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
			plot(ax, t, raw, 'Color', [0.75 0.75 0.75], 'DisplayName', 'Raw');
			if nReady > 0
				plot(ax, t(1:nReady), filtered(1:nReady), 'b-', ...
					'LineWidth', 1.5, 'DisplayName', 'Filtered');
			end
			% Peek-ahead shaded region
			if app.isRunning && peekSec > 0 && nReady > 0
				tPk0  = t(min(nReady, numel(t)));
				tPk1  = min(t(end), tPk0 + peekSec);
				yl    = ylim(ax);
				patch(ax, [tPk0 tPk1 tPk1 tPk0], [yl(1) yl(1) yl(2) yl(2)], ...
					[1 0.85 0.6], 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
					'DisplayName', sprintf('Peek (%.2gs)', peekSec));
			end
			legend(ax, 'Location', 'best');
			ylabel(ax, 'Amplitude');
			title(ax, 'Raw vs Filtered');
			xlim(ax, [xLo, xHi]);

			% ---- panel 2: Residual ------------------------------------------
			ax = app.axResidual;
			cla(ax);
			hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
			resPlot = raw;
			if nReady > 0
				resPlot(1:nReady) = raw(1:nReady) - filtered(1:nReady);
			end
			plot(ax, t, resPlot, 'r-', 'LineWidth', 1);
			ylabel(ax, 'Amplitude');
			title(ax, 'Residual (Raw - Filtered)');
			xlim(ax, [xLo, xHi]);

			if ~doHeavyPlots
				return;
			end

			% ---- PSD + theoretical magnitude --------------------------------
			nfft = 2^nextpow2(min(numel(raw), 4096));
			f    = (0 : nfft/2).' * fs / nfft;
			pr   = abs(fft(raw      - mean(raw),      nfft)).^2 / nfft;
			pf   = abs(fft(filtered - mean(filtered),  nfft)).^2 / nfft;
			[H, wHz] = freqz(b, a, nfft/2 + 1, fs);

			ax = app.axPSD;
			cla(ax);
			hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
			plot(ax, f, 10*log10(pr(1:nfft/2+1) + eps), ...
				'Color', [0.6 0.6 0.6], 'DisplayName', 'Raw PSD');
			plot(ax, f, 10*log10(pf(1:nfft/2+1) + eps), ...
				'b', 'LineWidth', 1.2, 'DisplayName', 'Filtered PSD');
			plot(ax, wHz, 20*log10(abs(H) + eps), ...
				'k--', 'LineWidth', 1.4, 'DisplayName', 'Theoretical |H(f)|');
			xlabel(ax, 'Frequency (Hz)');
			ylabel(ax, 'dB');
			title(ax, 'PSD + Theoretical Magnitude Response');
			legend(ax, 'Location', 'best');

			% ---- Phase response ---------------------------------------------
			ax = app.axPhase;
			cla(ax);
			hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
			plot(ax, wHz, unwrap(angle(H)) * 180/pi, 'k-', 'LineWidth', 1.2);
			xlabel(ax, 'Frequency (Hz)');
			ylabel(ax, 'Phase (deg)');
			title(ax, 'Phase Response');
		end

		function updateMetrics(app)
			nReady = max(1, min(numel(app.raw), app.idx - 1));
			e = app.raw(1:nReady) - app.filtered(1:nReady);

			mse         = mean(e.^2);
			maxAbsErr   = max(abs(e));
			rmsRaw      = rms(app.raw(1:nReady));
			rmsFiltered = rms(app.filtered(1:nReady));
			snrDb       = FilterTester.signalToNoiseDb(app.raw(1:nReady), e);

			if numel(app.a) == 1
				gdSamples = (numel(app.b) - 1) / 2;
			else
				gdSamples = max(numel(app.a), numel(app.b)) - 1;
			end
			latencyMs = 1000 * (gdSamples + (app.blockSize - 1)) / app.fs;

			app.tblMetrics.Data = {
				'MSE',               sprintf('%.6g', mse);
				'Max abs error',     sprintf('%.6g', maxAbsErr);
				'RMS raw',           sprintf('%.6g', rmsRaw);
				'RMS filtered',      sprintf('%.6g', rmsFiltered);
				'Est. latency (ms)', sprintf('%.3f',  latencyMs);
				'SNR (dB)',          sprintf('%.2f',  snrDb);
			};
		end

		function setStatus(app, msg)
			if isvalid(app.lblStatus)
				app.lblStatus.Text = ['Status: ', msg];
			end
		end

		function updateDataSourceControls(app)
			useLoad = ~strcmp(app.ddData.Value, 'Synthetic signal');
			app.btnLoad.Enable = FilterTester.onOff(useLoad);
			if useLoad
				app.btnSynthConfig.Text = 'Configure channel';
				hasDatalog = strcmp(app.loadedDataType, 'datalog') && ~isempty(app.datalogChannelNames);
				hasCsv = strcmp(app.loadedDataType, 'csv') && ~isempty(app.csvChannelNames);
				app.btnSynthConfig.Enable = FilterTester.onOff(hasDatalog || hasCsv);
			else
				app.btnSynthConfig.Text = 'Configure / Tune';
				app.btnSynthConfig.Enable = 'on';
			end
		end

		function clearDatalogState(app)
			app.datalogTable = table();
			app.datalogDtStr = '';
			app.datalogTag = '';
			app.datalogChannelNames = {};
			app.datalogSelectedChannel = 1;
		end

		function clearCsvState(app)
			app.csvData = [];
			app.csvChannelNames = {};
			app.csvSelectedChannel = 1;
		end

		function applyLoadedSignal(app, signal, t, fs)
			signal = double(signal(:));
			if isempty(signal)
				error('Loaded signal is empty.');
			end

			if nargin < 3 || isempty(t)
				t = [];
			end
			if nargin < 4 || isempty(fs) || ~isfinite(fs) || fs <= 0
				fs = max(1, app.efFs.Value);
			end

			if isempty(t)
				validMask = isfinite(signal);
				signal = signal(validMask);
				if isempty(signal)
					error('Loaded signal contains no finite samples.');
				end
				t = (0 : numel(signal)-1).' / fs;
			else
				t = double(t(:));
				validMask = isfinite(signal) & isfinite(t);
				signal = signal(validMask);
				t = t(validMask);
				if isempty(signal)
					error('Loaded signal contains no valid time/sample pairs.');
				end
				t = t - t(1);
				if numel(t) > 1 && t(end) > 0
					fs = (numel(t) - 1) / t(end);
				end
			end

			signal = signal - mean(signal, 'omitnan');
			signal(isnan(signal)) = 0;

			app.fs       = max(1, fs);
			app.t        = t;
			app.raw      = signal;
			app.filtered = zeros(size(signal));
			app.residual = signal;
			app.idx      = 1;
			app.efFs.Value = app.fs;
			if numel(t) > 1
				app.efDuration.Value = max(t(end), 0.1);
			end
			app.xlimFrozen = [app.t(1), app.t(min(numel(app.t), max(2, round(app.fs * 3))))];
			app.resetFilterState();
			app.replayAll();
			app.updateDataSourceControls();
		end

		function loadDatalogSource(app, fullPath)
			[datTable, dtStr, tag] = load_datalog(fullPath);
			if width(datTable) <= 2
				error('Datalog does not contain any sensor channels.');
			end

			app.datalogTable = datTable;
			app.datalogDtStr = dtStr;
			app.datalogTag = tag;
			app.datalogChannelNames = app.getDatalogChannelNames(datTable);
			app.datalogSelectedChannel = min(max(1, app.datalogSelectedChannel), ...
				numel(app.datalogChannelNames));
			app.loadedDataType = 'datalog';
			app.loadedSourcePath = fullPath;
			app.applyDatalogChannel(app.datalogSelectedChannel);
		end

		function loadCsvSource(app, fullPath)
			tbl = readtable(fullPath);
			if isempty(tbl) || width(tbl) == 0
				error('CSV is empty.');
			end

			nVars = width(tbl);
			isNumericCol = false(1, nVars);
			for idx = 1:nVars
				col = tbl{:, idx};
				isNumericCol(idx) = isnumeric(col) || islogical(col);
			end
			if ~any(isNumericCol)
				error('CSV does not contain numeric channels.');
			end

			csvData = double(tbl{:, isNumericCol});
			if isempty(csvData)
				error('CSV numeric data is empty.');
			end

			app.csvData = csvData;
			app.csvChannelNames = app.getCsvChannelNames(tbl, isNumericCol);
			app.csvSelectedChannel = min(max(1, app.csvSelectedChannel), numel(app.csvChannelNames));
			app.loadedDataType = 'csv';
			app.loadedSourcePath = fullPath;
			app.applyCsvChannel(app.csvSelectedChannel);
		end

		function configureDatalogChannel(app)
			if ~strcmp(app.loadedDataType, 'datalog') || isempty(app.datalogChannelNames)
				app.setStatus('Load a sensor datalog before configuring a channel');
				return;
			end

			dlg = uifigure('Name', 'Select Datalog Channel', ...
				'Position', [250 250 420 120], ...
				'WindowStyle', 'modal', ...
				'Resize', 'off');

			g = uigridlayout(dlg, [3 2], ...
				'RowHeight', {22, 28, 32}, ...
				'ColumnWidth', {'fit', '1x'}, ...
				'Padding', [10 10 10 10], ...
				'RowSpacing', 8, ...
				'ColumnSpacing', 8);

			uilabel(g, 'Text', 'Channel');
			ddChannel = uidropdown(g, ...
				'Items', app.datalogChannelNames, ...
				'Value', app.datalogChannelNames{app.datalogSelectedChannel});
			ddChannel.Layout.Row = 1;
			ddChannel.Layout.Column = 2;

			metaText = strtrim(sprintf('%s %s', app.datalogDtStr, app.datalogTag));
			if isempty(metaText)
				metaText = app.loadedSourcePath;
			end
			metaLabel = uilabel(g, 'Text', metaText, 'WordWrap', 'on');
			metaLabel.Layout.Row = 2;
			metaLabel.Layout.Column = [1 2];

			btnGrid = uigridlayout(g, [1 2], ...
				'ColumnWidth', {'1x', '1x'}, ...
				'Padding', [0 0 0 0], ...
				'RowSpacing', 0, ...
				'ColumnSpacing', 8);
			btnGrid.Layout.Row = 3;
			btnGrid.Layout.Column = [1 2];

			uibutton(btnGrid, 'Text', 'Apply', ...
				'ButtonPushedFcn', @(~,~) applySelection());
			uibutton(btnGrid, 'Text', 'Close', ...
				'ButtonPushedFcn', @(~,~) delete(dlg));

			function applySelection()
				idx = find(strcmp(app.datalogChannelNames, ddChannel.Value), 1);
				if isempty(idx)
					idx = app.datalogSelectedChannel;
				end
				app.applyDatalogChannel(idx);
				if isvalid(dlg)
					delete(dlg);
				end
			end
		end

		function configureCsvChannel(app)
			if ~strcmp(app.loadedDataType, 'csv') || isempty(app.csvChannelNames)
				app.setStatus('Load a CSV file before configuring a channel');
				return;
			end

			dlg = uifigure('Name', 'Select CSV Channel', ...
				'Position', [250 250 420 120], ...
				'WindowStyle', 'modal', ...
				'Resize', 'off');

			g = uigridlayout(dlg, [3 2], ...
				'RowHeight', {22, 28, 32}, ...
				'ColumnWidth', {'fit', '1x'}, ...
				'Padding', [10 10 10 10], ...
				'RowSpacing', 8, ...
				'ColumnSpacing', 8);

			uilabel(g, 'Text', 'Channel');
			ddChannel = uidropdown(g, ...
				'Items', app.csvChannelNames, ...
				'Value', app.csvChannelNames{app.csvSelectedChannel});
			ddChannel.Layout.Row = 1;
			ddChannel.Layout.Column = 2;

			metaLabel = uilabel(g, 'Text', app.loadedSourcePath, 'WordWrap', 'on');
			metaLabel.Layout.Row = 2;
			metaLabel.Layout.Column = [1 2];

			btnGrid = uigridlayout(g, [1 2], ...
				'ColumnWidth', {'1x', '1x'}, ...
				'Padding', [0 0 0 0], ...
				'RowSpacing', 0, ...
				'ColumnSpacing', 8);
			btnGrid.Layout.Row = 3;
			btnGrid.Layout.Column = [1 2];

			uibutton(btnGrid, 'Text', 'Apply', ...
				'ButtonPushedFcn', @(~,~) applySelection());
			uibutton(btnGrid, 'Text', 'Close', ...
				'ButtonPushedFcn', @(~,~) delete(dlg));

			function applySelection()
				idx = find(strcmp(app.csvChannelNames, ddChannel.Value), 1);
				if isempty(idx)
					idx = app.csvSelectedChannel;
				end
				app.applyCsvChannel(idx);
				if isvalid(dlg)
					delete(dlg);
				end
			end
		end

		function applyCsvChannel(app, channelIdx)
			if isempty(app.csvData) || size(app.csvData, 2) < channelIdx
				error('Requested CSV channel is not available.');
			end

			signal = app.csvData(:, channelIdx);
			app.csvSelectedChannel = channelIdx;
			app.loadedDataType = 'csv';
			app.applyLoadedSignal(signal, [], max(1, app.efFs.Value));
			[~, fileName, ext] = fileparts(app.loadedSourcePath);
			channelName = app.csvChannelNames{channelIdx};
			app.setStatus(sprintf('Loaded %s%s  [%s]  (%d samples)', ...
				fileName, ext, channelName, numel(app.raw)));
		end

		function applyDatalogChannel(app, channelIdx)
			if isempty(app.datalogTable) || width(app.datalogTable) < channelIdx + 2
				error('Requested datalog channel is not available.');
			end

			signal = app.datalogTable{:, channelIdx + 2};
			rawTime = app.datalogTable{:, 1} + app.datalogTable{:, 2};

			if isdatetime(rawTime)
				validMask = ~isnat(rawTime) & isfinite(signal);
				rawTime = rawTime(validMask);
				if isempty(rawTime)
					error('Selected datalog channel has no valid timestamps.');
				end
				t = seconds(rawTime - rawTime(1));
			elseif isduration(rawTime)
				validMask = ~isnat(rawTime) & isfinite(signal);
				rawTime = rawTime(validMask);
				if isempty(rawTime)
					error('Selected datalog channel has no valid timestamps.');
				end
				t = seconds(rawTime - rawTime(1));
			else
				rawTime = double(rawTime(:));
				validMask = isfinite(rawTime) & isfinite(signal);
				t = rawTime(validMask);
				if isempty(t)
					error('Selected datalog channel has no valid timestamps.');
				end
				t = t - t(1);
			end

			signal = double(signal(validMask));
			if isempty(signal)
				error('Selected datalog channel has no valid samples.');
			end

			app.datalogSelectedChannel = channelIdx;
			app.loadedDataType = 'datalog';
			app.applyLoadedSignal(signal, t, []);
			[~, fileName, ext] = fileparts(app.loadedSourcePath);
			channelName = app.datalogChannelNames{channelIdx};
			app.setStatus(sprintf('Loaded %s%s  [%s]  (%d samples)', ...
				fileName, ext, channelName, numel(app.raw)));
		end

		function names = getDatalogChannelNames(app, datTable)
			nChannels = width(datTable) - 2;
			names = cell(1, nChannels);
			varNames = datTable.Properties.VariableNames;
			if numel(varNames) >= width(datTable)
				varNames = varNames(3:end);
			else
				varNames = repmat({''}, 1, nChannels);
			end

			for idx = 1:nChannels
				fallback = sprintf('S%d-Ch%d', ceil(idx / 2), 1 + mod(idx - 1, 2));
				name = '';
				if idx <= numel(varNames)
					name = varNames{idx};
				end
				if isempty(name) || startsWith(name, 'Var')
					name = fallback;
				else
					name = strrep(name, '_', ' ');
				end
				names{idx} = sprintf('Channel %d: %s', idx, name);
			end
		end

		function names = getCsvChannelNames(app, tbl, isNumericCol)
			idxCols = find(isNumericCol);
			varNames = tbl.Properties.VariableNames;
			names = cell(1, numel(idxCols));
			for n = 1:numel(idxCols)
				idx = idxCols(n);
				name = '';
				if idx <= numel(varNames)
					name = varNames{idx};
				end
				if isempty(name) || startsWith(name, 'Var')
					name = sprintf('Column %d', idx);
				else
					name = strrep(name, '_', ' ');
				end
				names{n} = sprintf('Channel %d: %s', n, name);
			end
		end

	end   % private methods

	% =========================================================================
	%  Static helper functions
	% =========================================================================
	methods (Static, Access = private)

		function s = onOff(tf)
			if tf, s = 'on'; else, s = 'off'; end
		end

		function sig = synthSignal(t, components)
			% components: Nx4 matrix [amplitude, frequency_Hz, phase_rad, noise_amp]
			% Each row contributes: amp*sin(2*pi*freq*t + phase) + noise_amp*randn
			sig = zeros(size(t));
			for k = 1:size(components, 1)
				amp   = components(k, 1);
				freq  = components(k, 2);
				phase = components(k, 3);
				nAmp  = components(k, 4);
				sig = sig + amp * sin(2*pi*freq*t + phase) + nAmp * randn(size(t));
			end
		end

		function [b, a] = design_filter(filterType, fs, order, cutoffHz, highpassHz)
			nyq = fs / 2;
			switch lower(filterType)
				case 'moving-average'
					n = max(1, order);
					b = ones(1, n) / n;
					a = 1;

				case 'lowpass-iir'
					wn = min(max(cutoffHz  / nyq, 0.001), 0.999);
					n  = max(1, order);
					[b, a] = butter(n, wn, 'low');

				case 'highpass-iir'
					wn = min(max(highpassHz / nyq, 0.001), 0.999);
					n  = max(1, order);
					[b, a] = butter(n, wn, 'high');

				case 'biquad-lowpass'
					q    = 0.707;
					w0   = 2*pi * min(max(cutoffHz, 0.5), nyq*0.95) / fs;
					alph = sin(w0) / (2*q);
					cw   = cos(w0);
					b0 = (1 - cw)/2;  b1 = 1 - cw;  b2 = (1 - cw)/2;
					a0 =  1 + alph;   a1 = -2*cw;   a2 = 1 - alph;
					b = [b0  b1  b2] / a0;
					a = [1   a1/a0  a2/a0];

				otherwise
					b = 1; a = 1;
			end
		end

		function snr = signalToNoiseDb(raw, noise)
			ps = mean(raw.^2);
			pn = mean(noise.^2);
			if pn < eps, snr = Inf; else, snr = 10 * log10(ps / pn); end
		end

	end   % static methods

end   % classdef
