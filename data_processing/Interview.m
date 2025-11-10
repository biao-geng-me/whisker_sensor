classdef Interview < handle
	% Interview  Simple programmatic UI for interactive signal analysis
	%
	% Uses a 2x2 uigridlayout. Left column contains the dataset list and
	% an optional controls panel; right column contains the main time
	% axes (top) and FFT axes (bottom).

	properties
		UIFigure matlab.ui.Figure
		Grid matlab.ui.container.GridLayout

		% Menus
		MenuFile matlab.ui.container.Menu
		MenuLoad matlab.ui.container.Menu

		% UI components
		DataList cell = {}    % cell array of SignalData objects
		DataListTable matlab.ui.control.Table
		AxTime matlab.ui.control.UIAxes
		AxFFT matlab.ui.control.UIAxes
		% Plot management
		Plots cell = {} % cell array of structs with fields: name,dataIdx,chIdx,line
		PlotManagerFigure matlab.ui.Figure
		PlotMgrControls struct = struct()
		% Pan management
		Pans cell = {} % structs: name,start,end,roi
		PanManagerFigure matlab.ui.Figure
		PanMgrControls struct = struct()
		% Spectrogram / cursor management
		SelectedPlotIdx double = []
		AxSpec matlab.ui.control.UIAxes
		AxSpecCurve matlab.ui.control.UIAxes
		SpecCursors cell = {} % cell array of structs: {name,start,end,roi}

		% spectrogram settings (seconds, overlap)
		SpecWindowSec double = 1.0
		SpecOverlap double = 0.9
        % optional frequency range for spectrogram [fmin fmax]; empty = auto
        SpecFreqRange double = []
		% last-computed spectrogram data (t, f, S matrix)
		Spec_t double = []
		Spec_f double = []
		Spec_S double = []
		% Drag state for interactive line dragging
		DragState struct = struct('active',false,'line',[],'origX',[],'origY',[],'startPoint',[],'plotIdx',[])
	end

	methods
		function obj = Interview()
			obj.createUI();
		end

		function onPlotTableEdited(obj, src, event)
			% Called when a cell in the dataset table is edited (e.g., TimeOffset)
			% Deprecated -- replaced by plot table editing. Keep for compatibility.
			if isempty(event) || isempty(event.Indices), return; end
			row = event.Indices(1);
			col = event.Indices(2);
			% If user edited Visible/Tag/Offset directly in the plots table
			if isempty(obj.Plots) || row<1 || row>numel(obj.Plots), return; end
			p = obj.Plots{row};
			switch col
				case 1 % Visible
					newVis = logical(event.NewData);
					if isgraphics(p.line)
						if newVis, p.line.Visible = 'on'; else p.line.Visible = 'off'; end
					end
					obj.Plots{row} = p;
					obj.computeAndPlotFFT();
				case 4 % Tag
					newTag = event.NewData;
					p.tag = newTag;
					p.name = sprintf('%s - %s', newTag, obj.DataList{p.dataIdx}.chNames{p.chIdx});
					if isgraphics(p.line), p.line.DisplayName = p.name; end
					obj.Plots{row} = p;
					obj.updatePlotManagerUI();
					obj.computeAndPlotFFT();
				case 5 % Offset (signal offset)
					newOff = double(event.NewData);
					if p.dataIdx>=1 && p.dataIdx<=numel(obj.DataList)
						sd = obj.DataList{p.dataIdx};
						sd.setSigOffset(p.chIdx, newOff);
						% refresh plots for this dataset/channel
						obj.updateAllPlotsForData(p.dataIdx);
						obj.computeAndPlotFFT();
					end
				case 6 % Spectrogram single-select column
					% enforce single-select: if user set this row true, clear others
					newVal = logical(event.NewData);
					if newVal
						% clear other selections
						for j=1:numel(obj.Plots)
							if j~=row
								% set table data programmatically
								% we will update DataListTable after loop
								% nothing to change in obj.Plots here
							end
						end
						obj.SelectedPlotIdx = row;
					else
						% user cleared selection
						obj.SelectedPlotIdx = [];
					end
					% refresh table and spectrogram
					obj.updateDataListUI();
					obj.updateSpectrogram();
			end
		end

		function showDatasetProperties(obj, row)
			% show a small properties window for dataset row
			sd = obj.DataList{row};
			f = uifigure('Name','Dataset Properties','Position',[300 300 420 300]);
			uilabel(f,'Position',[10 260 400 22],'Text',sprintf('Path: %s', sd.filepath),'Interpreter','none');
			uilabel(f,'Position',[10 240 200 18],'Text',sprintf('Channels: %d', size(sd.sig,2)));
			uilabel(f,'Position',[220 240 200 18],'Text',sprintf('Samples: %d', size(sd.sig,1)));
			% channel offsets table
			tbl = uitable(f,'Position',[10 60 400 160],'Data',sd.sigOffset,'ColumnEditable',true(1,size(sd.sig,2)),'ColumnName',sd.chNames);
			uilabel(f,'Position',[10 40 200 18],'Text','Edit channel offsets (signal)');
			% time offset edit
			timeEdit = uieditfield(f,'numeric','Position',[10 10 120 22],'Value',sd.timeOffset);
			btnSave = uibutton(f,'push','Position',[140 10 80 24],'Text','Save','ButtonPushedFcn',@(s,e) onSave());
			function onSave()
				% read channel offsets from table
				vals = tbl.Data;
				for ci=1:numel(vals)
					sd.setSigOffset(ci, vals(ci));
				end
				sd.setTimeOffset(timeEdit.Value);
				% refresh plots
				obj.updateAllPlotsForData(row);
				obj.computeAndPlotFFT();
				delete(f);
			end
		end

		function updateAllPlotsForData(obj, dataIdx)
			for i=1:numel(obj.Plots)
				p = obj.Plots{i};
				if p.dataIdx==dataIdx && isgraphics(p.line)
					sd = obj.DataList{p.dataIdx};
					p.line.XData = sd.getTimeWithOffset();
					% baseline-subtracted
					if ~isempty(sd.sigOffset) && numel(sd.sigOffset)>=p.chIdx
						p.line.YData = sd.sig(:,p.chIdx) - sd.sigOffset(p.chIdx);
					else
						p.line.YData = sd.sig(:,p.chIdx);
					end
				end
			end
			% refresh time-axis legend to reflect any visibility changes
			obj.updateTimeLegend();
		end

		function updateTimeLegend(obj)
			% Show legend entries on the time axis for all plots (regardless of visibility)
			names = {};
			for i=1:numel(obj.Plots)
				p = obj.Plots{i};
				if isfield(p,'line') && isgraphics(p.line)
					if isprop(p.line,'DisplayName') && ~isempty(p.line.DisplayName)
						names{end+1} = p.line.DisplayName; %#ok<AGROW>
					else
						names{end+1} = p.name; %#ok<AGROW>
					end
				else
					names{end+1} = p.name; %#ok<AGROW>
				end
			end
			if isempty(names)
				try
					legend(obj.AxTime,'off');
				catch
					% ignore
				end
			else
				legend(obj.AxTime,names,'Interpreter','none');
			end
		end

		function editPlotPropertiesModal(obj, row)
			% Open a modal window to edit file, channel, tag, offset, visible for plot row
			if row<1 || row>numel(obj.Plots), return; end
			p = obj.Plots{row};
			f = uifigure('Name','Edit Plot Properties','Position',[120 320 1200 420],'WindowStyle','modal');
			% Visible checkbox
			cb = uicheckbox(f,'Text','Visible','Position',[10 190 120 22],'Value', isgraphics(p.line) && strcmp(p.line.Visible,'on'));
			% File dropdown
			uilabel(f,'Position',[10 160 80 18],'Text','File:');
			fileItems = cellfun(@(s) s.filepath, obj.DataList, 'UniformOutput', false);
			if isempty(fileItems), fileItems = {''}; end
			fileDd = uidropdown(f,'Position',[100 158 300 22],'Items',fileItems);
			if p.dataIdx>=1 && p.dataIdx<=numel(obj.DataList)
				fileDd.Value = obj.DataList{p.dataIdx}.filepath;
			end
			% Channel dropdown
			uilabel(f,'Position',[10 120 80 18],'Text','Channel:');
			chDd = uidropdown(f,'Position',[100 118 200 22],'Items',{});
			% Tag and offset
			uilabel(f,'Position',[10 80 80 18],'Text','Tag:');
			tagField = uieditfield(f,'text','Position',[100 78 300 22],'Value',p.tag);
			uilabel(f,'Position',[10 44 80 18],'Text','Offset:');
			offField = uieditfield(f,'numeric','Position',[100 42 120 22]);
			% populate channels based on selected file
			function refreshChannels()
				selPath = fileDd.Value;
				idx = find(strcmp(fileItems,selPath),1);
				if isempty(idx) || idx>numel(obj.DataList)
					chDd.Items = {};
					return;
				end
				sd = obj.DataList{idx};
				chDd.Items = sd.chNames;
				% set channel value
				if p.dataIdx==idx && p.chIdx>=1 && p.chIdx<=numel(sd.chNames)
					chDd.Value = sd.chNames{p.chIdx};
				else
					chDd.Value = sd.chNames{min(1,end)};
				end
				% set offset default
				if ~isempty(sd.sigOffset) && numel(sd.sigOffset)>=p.chIdx
					offField.Value = sd.sigOffset(p.chIdx);
				else
					offField.Value = 0;
				end
			end
			fileDd.ValueChangedFcn = @(s,e) refreshChannels();
			refreshChannels();
			% Save button
			btn = uibutton(f,'push','Position',[300 10 100 28],'Text','Save','ButtonPushedFcn',@(s,e) onSave());
			function onSave()
				% apply changes
				newFile = fileDd.Value;
				newFileIdx = find(strcmp(fileItems,newFile),1);
				if isempty(newFileIdx)
					uialert(f,'Invalid file selection','Error'); return;
				end
				newChName = chDd.Value;
				sdNew = obj.DataList{newFileIdx};
				newChIdx = find(strcmp(sdNew.chNames,newChName),1);
				if isempty(newChIdx), uialert(f,'Invalid channel','Error'); return; end
				% update plot struct
				p.dataIdx = newFileIdx;
				p.chIdx = newChIdx;
				p.tag = tagField.Value;
				% update offset in target SignalData
				sdNew.setSigOffset(newChIdx, offField.Value);
				% update visibility
				if cb.Value, vis = 'on'; else vis = 'off'; end
				% update line
				if isgraphics(p.line)
					p.line.XData = sdNew.getTimeWithOffset();
					p.line.YData = sdNew.sig(:,newChIdx) - sdNew.sigOffset(newChIdx);
					p.line.DisplayName = sprintf('%s - %s', p.tag, sdNew.chNames{newChIdx});
					p.line.Visible = vis;
				end
				obj.Plots{row} = p;
				obj.updateDataListUI();
				obj.updatePlotManagerUI();
				obj.computeAndPlotFFT();
				delete(f);
			end
		end

		function createUI(obj)
			% Create main UIFigure
			% enlarge main UI to accommodate an extra spectrogram row
			obj.UIFigure = uifigure('Name','Interview - Signal Analyzer','Position',[100 100 1200 900]);
			% ensure cleanup when the UIFigure is closed
			obj.UIFigure.CloseRequestFcn = @(s,e) obj.onClose();

			% Menus
			obj.MenuFile = uimenu(obj.UIFigure,'Text','File');
			obj.MenuLoad = uimenu(obj.MenuFile,'Text','Load Data...','MenuSelectedFcn',@obj.onLoadData);
			uimenu(obj.MenuFile,'Text','Save Session...','MenuSelectedFcn',@obj.onSaveSession);
			uimenu(obj.MenuFile,'Text','Load Session...','MenuSelectedFcn',@obj.onLoadSession);
			% Plot menu
			objMenu = uimenu(obj.UIFigure,'Text','Plot');
			uimenu(objMenu,'Text','Manage','MenuSelectedFcn',@obj.onManagePlots);
			uimenu(objMenu,'Text','Pan','MenuSelectedFcn',@obj.onManagePans);

            % Spectrogram menu
            specMenu = uimenu(obj.UIFigure,'Text','Spectrogram');
            uimenu(specMenu,'Text','Cursor Manager','MenuSelectedFcn',@obj.onManageSpecCursors);
            uimenu(specMenu,'Text','Settings...','MenuSelectedFcn',@obj.onSpecSettings);

			% Grid layout 3x2: top row is list spanning both columns; middle row has time and FFT;
			% bottom row contains spectrogram (left) and its cursor-spectrum (right)
			g = uigridlayout(obj.UIFigure, [3,2]);
			g.RowHeight = {'2x','4x','4x'};      % top (list) smaller, middle/bottom larger
			g.ColumnWidth = {'2x','1x'};
			obj.Grid = g;

			% Dataset / Plot properties table (top, spans both columns)
			% Columns: Visible (checkbox), File, Channel, Tag, Offset, Spec (single-select)
			colNames = {'Visible','File','Channel','Tag','Offset','Spec'};
			% make Spec editable (logical) but we'll enforce single-select behavior in the callback
			tbl = uitable(g,'ColumnName',colNames,'ColumnEditable',[true false false true true true], 'ColumnFormat',{'logical','char','char','char','numeric','logical'});
			% we'll handle File/Channel edits via a modal editor on selection
			tbl.Layout.Row = 1;
			tbl.Layout.Column = [1 2];
			tbl.CellSelectionCallback = @(s,e) obj.onPlotTableSelected(s,e);
			tbl.CellEditCallback = @(s,e) obj.onPlotTableEdited(s,e);
			obj.DataListTable = tbl;
			% Time axis (bottom-left)
			axT = uiaxes(g);
			axT.Layout.Row = 2;
			axT.Layout.Column = 1;
			title(axT,'Time-series'); xlabel(axT,'Time (s)'); ylabel(axT,'Amplitude');
			obj.AxTime = axT;
			% listen for axis limit changes to update pan ROI heights
			try
				addlistener(obj.AxTime,'XLim','PostSet',@(s,e) obj.onAxesLimitsChanged());
				addlistener(obj.AxTime,'YLim','PostSet',@(s,e) obj.onAxesLimitsChanged());
				% also keep spectrogram X axis linked to time axis
				addlistener(obj.AxTime,'XLim','PostSet',@(s,e) obj.syncSpecXLim());
			catch
				% older MATLAB versions may not allow; ignore
			end

			% FFT axis (middle-right)
			axF = uiaxes(g);
			axF.Layout.Row = 2;
			axF.Layout.Column = 2;
			title(axF,'Spectrum'); xlabel(axF,'Frequency (Hz)'); ylabel(axF,'Magnitude');
			obj.AxFFT = axF;

			% Spectrogram axis (bottom-left)
			axS = uiaxes(g);
			axS.Layout.Row = 3;
			axS.Layout.Column = 1;
			title(axS,'Spectrogram (amplitude)'); xlabel(axS,'Time (s)'); ylabel(axS,'Frequency (Hz)');
			obj.AxSpec = axS;
			% keep time and spectrogram X axes aligned
			try
				addlistener(obj.AxSpec,'XLim','PostSet',@(s,e) obj.syncTimeXLim());
			catch
				% ignore
			end

			% Cursor-selected spectrum (bottom-right)
			axC = uiaxes(g);
			axC.Layout.Row = 3;
			axC.Layout.Column = 2;
			title(axC,'Cursor-selected Spectrum'); xlabel(axC,'Frequency (Hz)'); ylabel(axC,'Magnitude');
			obj.AxSpecCurve = axC;
		end

		function onLoadData(obj, ~, ~)
			% Menu callback to load one or more files
			[files, path] = uigetfile({'*.mat;*.dat;*.csv;*.txt','Data files (*.mat,*.dat,*.txt,*.csv)'; '*.*','All files'}, 'Select data file(s)','MultiSelect','on');
			if isequal(files,0)
				return; % user cancelled
			end

			if ischar(files)
				files = {files};
			end

			for k = 1:numel(files)
				fname = files{k};
				fullp = fullfile(path, fname);
				try
					sd = SignalData(fullp);
				catch ME
					warning('Interview:LoadFailed','Failed to load %s: %s', fullp, ME.message);
					uialert(obj.UIFigure, sprintf('Failed to load %s:\n%s', fname, ME.message), 'Load error');
					continue;
				end
				% append
				obj.DataList{end+1} = sd; %#ok<AGROW>
			end

			obj.updateDataListUI();
		end

		function updateDataListUI(obj)
			% Update the top table to show current plots and their properties
			% Columns: Visible (logical), File (char), Channel (char), Tag (char), Offset (numeric)
			n = max(1,numel(obj.Plots));
			data = cell(n,6);
			for i=1:numel(obj.Plots)
				p = obj.Plots{i};
				data{i,1} = isgraphics(p.line) && strcmp(p.line.Visible,'on');
				% file path
				if p.dataIdx>=1 && p.dataIdx<=numel(obj.DataList)
					data{i,2} = obj.DataList{p.dataIdx}.filepath;
				else
					data{i,2} = '';
				end
				% channel name
				if p.dataIdx>=1 && p.dataIdx<=numel(obj.DataList)
					sd = obj.DataList{p.dataIdx};
					if p.chIdx>=1 && p.chIdx<=numel(sd.chNames)
						data{i,3} = sd.chNames{p.chIdx};
					else
						data{i,3} = '';
					end
				else
					data{i,3} = '';
				end
				% tag
				data{i,4} = p.tag;
				% offset (signal offset for this dataset/channel)
				if p.dataIdx>=1 && p.dataIdx<=numel(obj.DataList)
					sd = obj.DataList{p.dataIdx};
					if ~isempty(sd.sigOffset) && numel(sd.sigOffset)>=p.chIdx
						data{i,5} = sd.sigOffset(p.chIdx);
					else
						data{i,5} = 0;
					end
				else
					data{i,5} = 0;
				end
				% spectrogram selection column: true if this plot is the selected one
				if ~isempty(obj.SelectedPlotIdx) && i==obj.SelectedPlotIdx
					data{i,6} = true;
				else
					data{i,6} = false;
				end
			end
			obj.DataListTable.Data = data;
		end

		function onPlotTableSelected(obj, src, event)
			% called when user selects a cell in the plots/properties table
			if isempty(src) || isempty(src.Data) || isempty(event), return; end
			if isempty(event.Indices), return; end
			row = event.Indices(1);
			col = event.Indices(2);
			if row<1 || row>numel(obj.Plots), return; end
			% If user selected File or Channel column, open modal editor
			% File -> allow choosing among loaded data files; Channel -> choose channels for that dataset
			% set selected plot and update spectrogram view
			obj.SelectedPlotIdx = row;
			obj.updateSpectrogram();
			if col==2 || col==3
				obj.editPlotPropertiesModal(row);
			end
		end

		function onManagePlots(obj, ~, ~)
			% Open or bring forward the Plot Manager UI
			if ~isempty(obj.PlotManagerFigure) && isvalid(obj.PlotManagerFigure)
				figure(obj.PlotManagerFigure);
				return;
			end
			obj.createPlotManagerUI();
		end

		function onManagePans(obj, ~, ~)
			% Open or bring forward the Pan Manager UI
			if ~isempty(obj.PanManagerFigure) && isvalid(obj.PanManagerFigure)
				figure(obj.PanManagerFigure);
				return;
			end
			obj.createPanManagerUI();
		end

		function createPanManagerUI(obj)
			f = uifigure('Name','Pan Manager','Position',[250 250 420 320]);
			obj.PanManagerFigure = f;

			% Add / Remove buttons and list of pans
			btnAdd = uibutton(f,'push','Position',[10 280 100 28],'Text','Add Pan','ButtonPushedFcn',@(s,e) onAddPan());
			btnRemove = uibutton(f,'push','Position',[120 280 100 28],'Text','Remove Pan','ButtonPushedFcn',@(s,e) onRemovePan());

			uilabel(f,'Position',[10 250 80 20],'Text','Pans:');
			panList = uilistbox(f,'Position',[10 60 280 180]);
			btnZoom = uibutton(f,'push','Position',[300 200 100 28],'Text','Zoom to Pan','ButtonPushedFcn',@(s,e) onZoom());

			% store controls
			obj.PanMgrControls.PanList = panList;
			obj.PanMgrControls.BtnAdd = btnAdd;
			obj.PanMgrControls.BtnRemove = btnRemove;
			obj.PanMgrControls.BtnZoom = btnZoom;

			% populate
			obj.updatePanManagerUI();

			% nested callbacks
			function onAddPan()
				% default pan: center 10% of xlim
				xl = xlim(obj.AxTime);
				span = diff(xl);
				w = max(span*0.1, span/100); % small window at least
				x1 = xl(1) + (span-w)/2;
				x2 = x1 + w;
				obj.createPan(x1,x2);
				obj.updatePanManagerUI();
				obj.computeAndPlotFFT();
			end

			function onRemovePan()
				sel = panList.Value;
				if isempty(sel), return; end
				plist = panList.Items;
				pidx = find(strcmp(plist,sel),1);
				if isempty(pidx), return; end
				obj.removePan(pidx);
				obj.updatePanManagerUI();
				obj.computeAndPlotFFT();
			end

			function onZoom()
				sel = panList.Value;
				if isempty(sel), return; end
				plist = panList.Items;
				pidx = find(strcmp(plist,sel),1);
				if isempty(pidx), return; end
				p = obj.Pans{pidx};
				xlim(obj.AxTime,[p.start p.end]);
			end
		end

		function createPan(obj, x1, x2)
			% create pan ROI on AxTime spanning full y range
			xl = xlim(obj.AxTime);
			yl = ylim(obj.AxTime);
			x1 = min(x1,x2); x2 = max(x1,x2);
			w = x2 - x1;
			% pad in y so the rectangle appears as bookending vertical band
			% use a large multiplier so the top/bottom edges fall well outside the axis
			ypad = max(5*diff(yl), 1e-6);
			pos = [x1 yl(1)-ypad w diff(yl)+2*ypad];
			% draw rectangle ROI; suppress transient internal warning during creation
			warnState = warning('off','all');
			roi = drawrectangle(obj.AxTime,'Position',pos);
			warning(warnState);
			% style: thin black border and low face alpha so it looks like bookend lines
			try
				roi.Color = [0 0 0];
				roi.LineWidth = 0.8;
				roi.FaceAlpha = 0.06;
			catch
				% some older versions may not support LineWidth/FaceAlpha; ignore
			end
			% store pan
			p.start = x1;
			p.end = x2;
			p.roi = roi;
			p.name = sprintf('%.1f - %.1f s', p.start, p.end);
			obj.Pans{end+1} = p; %#ok<AGROW>

			% add listeners for when ROI is moved/resized (support both events)
			pidx = numel(obj.Pans);
			% ROIMoved fires after interaction; MovingROI fires during
			try
				addlistener(roi,'ROIMoved',@(src,ev) onPanMoved(src,ev,pidx));
			catch
				% ignore if event not supported
			end
			try
				addlistener(roi,'MovingROI',@(src,ev) onPanMoved(src,ev,pidx));
			catch
				% ignore if event not supported
			end

			% nested callback
			function onPanMoved(src,~,pidx_local)
				if pidx_local > numel(obj.Pans), return; end
				pos = src.Position; % [x y w h]
				obj.Pans{pidx_local}.start = pos(1);
				obj.Pans{pidx_local}.end = pos(1)+pos(3);
				obj.Pans{pidx_local}.name = sprintf('%.1f - %.1f s', obj.Pans{pidx_local}.start, obj.Pans{pidx_local}.end);
				obj.updatePanManagerUI();
				obj.computeAndPlotFFT();
			end
		end

		function onAxesLimitsChanged(obj)
			% adjust pan ROI vertical spans to match current axes YLim (with padding)
			if isempty(obj.Pans), return; end
			yl = ylim(obj.AxTime);
			ypad = max(5*diff(yl), 1e-6);
			for i=1:numel(obj.Pans)
				p = obj.Pans{i};
				if isfield(p,'roi') && isvalid(p.roi)
					pos = p.roi.Position;
					pos(2) = yl(1)-ypad;
					pos(4) = diff(yl) + 2*ypad;
					try
						p.roi.Position = pos;
					catch
						% ignore
					end
				end
			end
		end

		function onManageSpecCursors(obj, ~, ~)
			% Open or bring forward the Spectrogram Cursor Manager UI
			if ~isempty(obj.PlotManagerFigure) && isvalid(obj.PlotManagerFigure) && isvalid(obj.UIFigure)
				% continue
			end
			if ~isempty(obj.PanManagerFigure) && isvalid(obj.PanManagerFigure)
				% no-op
			end
			if ~isempty(obj.PlotManagerFigure) && isvalid(obj.PlotManagerFigure)
				% keep as is
			end
			if ~isempty(obj.PlotManagerFigure) && isvalid(obj.PlotManagerFigure)
				% nothing
			end
			if ~isempty(obj.PlotManagerFigure) && isvalid(obj.PlotManagerFigure)
				% nothing
			end
			% create manager
			f = uifigure('Name','Spectrogram Cursor Manager','Position',[300 300 420 300]);
			% list of cursors
			uilabel(f,'Position',[10 260 80 20],'Text','Cursors:');
			list = uilistbox(f,'Position',[10 60 260 200]);
			btnAdd = uibutton(f,'push','Position',[280 220 120 28],'Text','Add Cursor','ButtonPushedFcn',@(s,e) onAdd());
			btnRemove = uibutton(f,'push','Position',[280 180 120 28],'Text','Remove Cursor','ButtonPushedFcn',@(s,e) onRemove());
			% populate
			updateList();

			function updateList()
				items = {};
				for k=1:numel(obj.SpecCursors)
					try
						p = obj.SpecCursors{k};
						items{end+1} = p.name; %#ok<AGROW>
					catch
						items{end+1} = sprintf('Cursor %d',k); %#ok<AGROW>
					end
				end
				list.Items = items;
			end

			function onAdd()
				% center new cursor at middle of spectrogram time or current xlim
				if ~isempty(obj.Spec_t)
					x = mean([min(obj.Spec_t) max(obj.Spec_t)]);
				else
					xl = xlim(obj.AxSpec); x = mean(xl);
				end
				obj.addSpecCursor(x);
				updateList();
			end

			function onRemove()
				sel = list.Value;
				if isempty(sel), return; end
				idx = find(strcmp(list.Items, sel),1);
				if isempty(idx), return; end
				obj.removeSpecCursor(idx);
				updateList();
			end

		end

		function addSpecCursor(obj, xCenter)
			% create single rectangular cursor ROI at xCenter and append
			if nargin<2, xCenter = 0; end
			% fixed vertical span for cursor: -90 to 100
			fmin = -90; fmax = 100;
			w = obj.SpecWindowSec;
			h = fmax - fmin;
			pos = [xCenter - w/2, fmin, w, h];
			warnState = warning('off','all');
			try
				r = drawrectangle(obj.AxSpec,'Position',pos);
			catch
				warning('Interview:DrawRectFail','drawrectangle failed');
				warning(warnState);
				return;
			end
			warning(warnState);
			try
				r.FaceAlpha = 0.12; r.LineWidth = 0.8; r.Color = rand(1,3);
				try r.InteractionsAllowed = 'translate'; catch,end
				try r.PositionConstraintFcn = @(pos)[pos(1) fmin pos(3) h]; catch,end
			catch
				% ignore
			end
			p.start = pos(1); p.end = pos(1)+pos(3); p.roi = r; p.name = sprintf('Curs %.2f', p.start);
			obj.SpecCursors{end+1} = p; %#ok<AGROW>
			% attach listeners
			idx = numel(obj.SpecCursors);
			try addlistener(r,'ROIMoved',@(src,ev) onMoved(idx)); catch,end
			try addlistener(r,'MovingROI',@(src,ev) onMoved(idx)); catch,end
			function onMoved(localIdx)
				if localIdx>numel(obj.SpecCursors), return; end
				posn = obj.SpecCursors{localIdx}.roi.Position;
				obj.SpecCursors{localIdx}.start = posn(1);
				obj.SpecCursors{localIdx}.end = posn(1)+posn(3);
				obj.SpecCursors{localIdx}.name = sprintf('Curs %.2f - %.2f', obj.SpecCursors{localIdx}.start, obj.SpecCursors{localIdx}.end);
				obj.updateSpecCursorSpectrum();
			end
			obj.updateSpecCursorSpectrum();
		end

		function removeSpecCursor(obj, idx)
			if idx<1 || idx>numel(obj.SpecCursors), return; end
			p = obj.SpecCursors{idx};
			try
				if isvalid(p.roi), delete(p.roi); end
			catch
			end
			obj.SpecCursors(idx) = [];
			obj.updateSpecCursorSpectrum();
		end

		function onSpecSettings(obj, ~, ~)
			% Open modal settings dialog to edit SpecWindowSec, SpecOverlap and freq range
			f = uifigure('Name','Spectrogram Settings','Position',[400 320 420 220],'WindowStyle','modal');
			uilabel(f,'Position',[10 170 160 22],'Text','Window length (s):');
			winField = uieditfield(f,'numeric','Position',[180 170 120 22],'Value',obj.SpecWindowSec);
			uilabel(f,'Position',[10 130 200 22],'Text','Overlap (0-1):');
			overField = uieditfield(f,'numeric','Position',[180 130 120 22],'Value',obj.SpecOverlap);
			% Frequency range controls
			autoChk = uicheckbox(f,'Text','Auto freq range','Position',[10 95 160 22],'Value', isempty(obj.SpecFreqRange));
			uilabel(f,'Position',[10 60 120 18],'Text','Freq min:');
			minField = uieditfield(f,'numeric','Position',[140 60 120 22]);
			uilabel(f,'Position',[10 30 120 18],'Text','Freq max:');
			maxField = uieditfield(f,'numeric','Position',[140 30 120 22]);
			% initialize fields
			if isempty(obj.SpecFreqRange)
				minField.Value = 0; maxField.Value = 100;
			else
				minField.Value = obj.SpecFreqRange(1);
				maxField.Value = obj.SpecFreqRange(2);
			end
			% enable/disable numeric fields based on checkbox
			if autoChk.Value
				minField.Enable = 'off';
				maxField.Enable = 'off';
			else
				minField.Enable = 'on';
				maxField.Enable = 'on';
			end
			autoChk.ValueChangedFcn = @(s,e) onAutoChanged(s.Value);
			function onAutoChanged(v)
				if v
					minField.Enable = 'off'; maxField.Enable = 'off';
				else
					minField.Enable = 'on'; maxField.Enable = 'on';
				end
			end
			btn = uibutton(f,'push','Position',[300 10 100 28],'Text','Save','ButtonPushedFcn',@(s,e) onSave());
			function onSave()
				v1 = winField.Value; v2 = overField.Value;
				if ~(isnumeric(v1) && v1>0)
					uialert(f,'Window must be > 0 seconds','Invalid'); return;
				end
				if ~(isnumeric(v2) && v2>=0 && v2<1)
					uialert(f,'Overlap must be in [0,1)','Invalid'); return;
				end
				if autoChk.Value
					obj.SpecFreqRange = [];
				else
					fmin = minField.Value; fmax = maxField.Value;
					if ~(isnumeric(fmin) && isnumeric(fmax) && fmin < fmax)
						uialert(f,'Freq min must be < Freq max','Invalid'); return;
					end
					obj.SpecFreqRange = [fmin fmax];
				end
				obj.SpecWindowSec = v1; obj.SpecOverlap = v2;
				% reposition and resize existing cursors and update spectrogram
				obj.updateSpectrogram();
				delete(f);
			end
		end

		function removePan(obj,pidx)
			if pidx<1 || pidx>numel(obj.Pans), return; end
			p = obj.Pans{pidx};
			if isvalid(p.roi)
				delete(p.roi);
			end
			obj.Pans(pidx) = [];
		end

		function updatePanManagerUI(obj)
			if isempty(obj.PanMgrControls) || ~isfield(obj.PanMgrControls,'PanList') || isempty(obj.PanMgrControls.PanList) || ~isgraphics(obj.PanMgrControls.PanList)
				% create temporary reference if called before UI exists
				return;
			end
			pl = cell(1,numel(obj.Pans));
			for i=1:numel(obj.Pans)
				pl{i} = obj.Pans{i}.name;
			end
			obj.PanMgrControls.PanList.Items = pl;
			if ~isempty(pl)
				obj.PanMgrControls.PanList.Value = pl{end};
			end
		end

		function updateSpectrogram(obj)
			% compute spectrogram for selected plot using aspectro
			sel = obj.SelectedPlotIdx;
			if isempty(sel) || sel<1 || sel>numel(obj.Plots), return; end
			% obj.Plots is a cell array of structs, obj.DataList is a cell array of SignalData
			pl = obj.Plots{sel};
			if pl.dataIdx<1 || pl.dataIdx>numel(obj.DataList), return; end
			sd = obj.DataList{pl.dataIdx};
			sig = sd.getSigWithOffset();
            sig = sig(:,pl.chIdx);
			fs = sd.Fs;
			% call aspectro (user-supplied)
			try
				[t,f,S] = aspectro(sig,fs,window_size=obj.SpecWindowSec*fs,overlap=obj.SpecOverlap);
			catch
				% fallback: try without extra args
				try
					[t,f,S] = aspectro(sig,fs);
				catch ME
					warning('Interview:AspectroFail','%s',ME.message);
					return;
				end
			end
			obj.Spec_S = S; obj.Spec_f = f; obj.Spec_t = t;
			% plot into AxSpec
			cla(obj.AxSpec);
			C = abs(S);
			try
				contourf(obj.AxSpec,t,f,C,'LineColor','none'); axis(obj.AxSpec,'xy');
				% color map: set lowest level (0) to white
				try
					n = 256; cmap = jet(n);
					cmap(1,:) = [1 1 1];
					colormap(obj.AxSpec, cmap);
					caxis(obj.AxSpec, [0 max(C(:))]);
				catch
					% ignore if colormap ops fail
				end
			catch me
				warning('Interview:ContourfFail','%s',me.message);
				imagesc(obj.AxSpec,t,f,C); axis(obj.AxSpec,'xy');
				try
					n = 256; cmap = jet(n); cmap(1,:)=[1 1 1]; colormap(obj.AxSpec,cmap); caxis(obj.AxSpec,[0 max(C(:))]);
				catch
					% ignore
				end
			end
			% determine frequency range to display (user override or auto)
			if ~isempty(obj.SpecFreqRange) && numel(obj.SpecFreqRange)==2 && all(isfinite(obj.SpecFreqRange))
				fr = obj.SpecFreqRange(:)';
			else
				fr = [min(f) max(f)];
			end
			% create cursors if none
			% reposition rectangles to fit new y-range and width (only if cursors exist)
			if ~isempty(obj.SpecCursors)
				for i=1:numel(obj.SpecCursors)
					try
						r = obj.SpecCursors{i};
						% enforce fixed vertical span for cursors
						pos = r.roi.Position;
						pos(2) = -90; % fixed lower bound
						pos(4) = 190; % fixed height to reach 100
						pos(3) = obj.SpecWindowSec; % width remains window seconds
						r.roi.Position = pos;
					catch
					end
				end
			end
			% ensure spectrogram X limits show the computed time range
			try xlim(obj.AxSpec,[min(t) max(t)]); catch end
			% ensure spectrogram y-limits reflect chosen range
			try ylim(obj.AxSpec, fr); catch end

			% align AxSpec horizontal position/width with AxTime so inner plotting areas line up
			try
				pt = obj.AxTime.Position;
				ps = obj.AxSpec.Position;
				ps(1) = pt(1);
				ps(3) = pt(3);
				obj.AxSpec.Position = ps;
			catch
				% ignore
			end
			% update the right-hand curve for the current cursor positions
			obj.updateSpecCursorSpectrum();
			% ensure drawing is flushed so ROIs can be stacked on top
			try drawnow; catch, end
			% bring cursors to front so they're visible above contour
			obj.bringSpecCursorsToFront();
		end

		function createSpecCursors(obj, x1, x2, freqRange)
			% create two rectangular ROIs on AxSpec at centers x1 and x2
			% ignore supplied freqRange for vertical span; use fixed -90..100
			if nargin<4, freqRange = [-90 100]; end
			ymin = -90; ymax = 100;
			w = obj.SpecWindowSec;
			h = ymax - ymin;
			% compute positions centered at x1,x2
			pos1 = [x1 - w/2, ymin, w, h];
			pos2 = [x2 - w/2, ymin, w, h];
			warnState = warning('off','all');
			try
				r1 = drawrectangle(obj.AxSpec,'Position',pos1);
				r2 = drawrectangle(obj.AxSpec,'Position',pos2);
			catch
				warning('Interview:DrawRectFail','drawrectangle failed');
				warning(warnState);
				return;
			end
			warning(warnState);
			try
				r1.FaceAlpha = 0.12; r1.Color = [1 0 0]; r2.FaceAlpha = 0.12; r2.Color = [0 0 1];
				r1.LineWidth = 0.8; r2.LineWidth = 0.8;
				% restrict interaction to translation only (no resize)
				try
					r1.InteractionsAllowed = 'translate';
					r2.InteractionsAllowed = 'translate';
				catch
				end
				% constrain position so vertical span is fixed
				try
					fixedY = ymin; fixedH = h;
					r1.PositionConstraintFcn = @(pos)[pos(1) fixedY pos(3) fixedH];
					r2.PositionConstraintFcn = @(pos)[pos(1) fixedY pos(3) fixedH];
				catch
				end
			catch
				% ignore if properties not supported
			end
			% store as structs like Pans
			p1.start = pos1(1); p1.end = pos1(1)+pos1(3); p1.roi = r1; p1.name = sprintf('Curs %.2f', p1.start);
			p2.start = pos2(1); p2.end = pos2(1)+pos2(3); p2.roi = r2; p2.name = sprintf('Curs %.2f', p2.start);
			obj.SpecCursors = {p1,p2};
			% listeners
			try addlistener(r1,'ROIMoved',@(src,ev) onMoved(1)); catch,end
			try addlistener(r1,'MovingROI',@(src,ev) onMoved(1)); catch,end
			try addlistener(r2,'ROIMoved',@(src,ev) onMoved(2)); catch,end
			try addlistener(r2,'MovingROI',@(src,ev) onMoved(2)); catch,end

			function onMoved(idx)
				if idx>numel(obj.SpecCursors), return; end
				pr = obj.SpecCursors{idx};
				pos = pr.roi.Position;
				obj.SpecCursors{idx}.start = pos(1);
				obj.SpecCursors{idx}.end = pos(1)+pos(3);
				obj.SpecCursors{idx}.name = sprintf('Curs %.2f - %.2f', obj.SpecCursors{idx}.start, obj.SpecCursors{idx}.end);
				obj.updateSpecCursorSpectrum();
			end
		end

		function updateSpecCursorSpectrum(obj)
			% Use spectrogram data directly: for each vertical cursor pick the
			% closest time column in the saved spectrogram and plot the
			% amplitude vs frequency using that column (no recompute of FFT).
			if isempty(obj.SpecCursors) || numel(obj.SpecCursors)<2, return; end
			if isempty(obj.Spec_S) || isempty(obj.Spec_t) || isempty(obj.Spec_f), return; end
			l1 = obj.SpecCursors{1}; l2 = obj.SpecCursors{2};
			if ~isstruct(l1) || ~isstruct(l2) || ~isfield(l1,'roi') || ~isfield(l2,'roi'), return; end
			if ~isgraphics(l1.roi) || ~isgraphics(l2.roi), return; end
			p1 = l1.roi.Position; p2 = l2.roi.Position;
			x1 = p1(1); x2 = p2(1);
			% find nearest time columns in spectrogram
			[~, idx1] = min(abs(obj.Spec_t - x1));
			[~, idx2] = min(abs(obj.Spec_t - x2));
			% extract spectral columns (frequency x 1)
			spec1 = obj.Spec_S(:, max(1, min(size(obj.Spec_S,2), idx1)));
			spec2 = obj.Spec_S(:, max(1, min(size(obj.Spec_S,2), idx2)));
			% plot
			cla(obj.AxSpecCurve);
			h1 = plot(obj.AxSpecCurve, obj.Spec_f, spec1, '-r', 'LineWidth',1.2);
			hold(obj.AxSpecCurve,'on');
			h2 = plot(obj.AxSpecCurve, obj.Spec_f, spec2, '-b', 'LineWidth',1.2);
			hold(obj.AxSpecCurve,'off');
			xlabel(obj.AxSpecCurve,'Frequency (Hz)'); ylabel(obj.AxSpecCurve,'Magnitude');
			legend(obj.AxSpecCurve, {sprintf('Cursor1 t=%.3fs', obj.Spec_t(idx1)), sprintf('Cursor2 t=%.3fs', obj.Spec_t(idx2))}, 'Interpreter','none');
			grid(obj.AxSpecCurve,'on');
		end

		function syncSpecXLim(obj)
			% Keep spectrogram X limits aligned with time axis X limits
			try
				t = xlim(obj.AxTime);
				cur = xlim(obj.AxSpec);
				if ~isequal(t,cur)
					xlim(obj.AxSpec,t);
				end
			catch
				% ignore
			end
		end

		function syncTimeXLim(obj)
			% Keep time axis X limits aligned with spectrogram X limits
			try
				t = xlim(obj.AxSpec);
				cur = xlim(obj.AxTime);
				if ~isequal(t,cur)
					xlim(obj.AxTime,t);
				end
			catch
				% ignore
			end
		end

		function bringSpecCursorsToFront(obj)
			% Ensure rectangle ROIs on AxSpec are rendered above the spectrogram
			for k=1:numel(obj.SpecCursors)
				try
					p = obj.SpecCursors{k};
					if isstruct(p) && isfield(p,'roi') && isgraphics(p.roi)
						% try uistack first
						try
							uistack(p.roi,'top');
						catch
							% fallback: reorder AxSpec.Children so ROI children are on top
							try
								ax = obj.AxSpec;
								roiChildren = p.roi.Children;
								allChildren = ax.Children;
								% place roiChildren at top preserving order
								remaining = allChildren(~ismember(allChildren, roiChildren));
								newOrder = [remaining; roiChildren];
								ax.Children = newOrder;
							catch
								% best-effort; ignore failures
							end
						end
					end
				catch
				end
			end
		end

		function onClose(obj)
			% Cleanup all child figures and delete the UIFigure
			% Close Plot Manager figure
			if ~isempty(obj.PlotManagerFigure) && isvalid(obj.PlotManagerFigure)
				try delete(obj.PlotManagerFigure); end
			end
			% Close Pan Manager figure
			if ~isempty(obj.PanManagerFigure) && isvalid(obj.PanManagerFigure)
				try delete(obj.PanManagerFigure); end
			end
			% Delete any ROIs created on AxTime
			for k=1:numel(obj.Pans)
				p = obj.Pans{k};
				if isfield(p,'roi') && isvalid(p.roi)
					try delete(p.roi); end
				end
			end
			% Delete spec cursor ROIs if present
			for k=1:numel(obj.SpecCursors)
				try
					p = obj.SpecCursors{k};
					if isstruct(p) && isfield(p,'roi') && isgraphics(p.roi)
						delete(p.roi);
					end
				catch
				end
			end
			% Finally delete main UIFigure
			if ~isempty(obj.UIFigure) && isvalid(obj.UIFigure)
				delete(obj.UIFigure);
			end
		end

		function computeAndPlotFFT(obj)
			% Compute FFT per pan window and plot per (plot,pan) curve
			if isempty(obj.Pans) || isempty(obj.Plots)
				cla(obj.AxFFT);
				return;
			end
			cla(obj.AxFFT);
			hold(obj.AxFFT,'off');
			legendEntries = {};
			for kpan = 1:numel(obj.Pans)
				pr = obj.Pans{kpan};
				for iplot = 1:numel(obj.Plots)
					p = obj.Plots{iplot};
					if ~isgraphics(p.line) || strcmp(p.line.Visible,'off'), continue; end
					sd = obj.DataList{p.dataIdx};
					t = sd.getTimeWithOffset();
					% use baseline-subtracted signal for FFT as well
					if ~isempty(sd.sigOffset) && numel(sd.sigOffset)>=p.chIdx
						sig = sd.sig(:,p.chIdx) - sd.sigOffset(p.chIdx);
					else
						sig = sd.sig(:,p.chIdx);
					end
					% sampling rate
					if isprop(sd,'Fs') && ~isempty(sd.Fs) && ~isnan(sd.Fs)
						fs = sd.Fs;
					else
						if numel(t)>1
							fs = (numel(t)-1)/t(end);
						else
							fs = 1;
						end
					end
					idx = find(t>=pr.start & t<=pr.end);
					if isempty(idx), continue; end
					sigseg = double(sig(idx));
					% call existing FFT helper (user's fast_fourier)
					[ff,pp] = fast_fourier(sigseg, fs);
					h = plot(obj.AxFFT, ff, pp);
					h.LineWidth = 1.0;
					hold(obj.AxFFT,'on');
					% legend entry: plot tag + pan tag
					ptag = '';
					if isfield(p,'tag') && ~isempty(p.tag)
						ptag = p.tag;
					else
						ptag = sprintf('%s_Ch%d', sd.filename, p.chIdx);
					end
					legname = sprintf('%s | %s', ptag, pr.name);
					legendEntries{end+1} = legname; %#ok<AGROW>
				end
			end
			if ~isempty(legendEntries)
				legend(obj.AxFFT, legendEntries,'Interpreter','none');
			end
			xlabel(obj.AxFFT,'Frequency (Hz)'); ylabel(obj.AxFFT,'Magnitude');
			title(obj.AxFFT,sprintf('FFT (per plot x pan)'));
		end

		function createPlotManagerUI(obj)
			f = uifigure('Name','Plot Manager','Position',[200 200 1280 360]);
			obj.PlotManagerFigure = f;

			% Dataset dropdown
			uilabel(f,'Position',[10 320 120 20],'Text','Dataset:');
			% populate dataset dropdown from loaded DataList (filepaths)
			fileItems = cellfun(@(s) s.filepath, obj.DataList, 'UniformOutput', false);
			if isempty(fileItems), fileItems = {}; end
			dd = uidropdown(f,'Position',[140 320 320 22],'Items',fileItems);
			dd.ValueChangedFcn = @(s,e) onDatasetChanged(e);

			% Channel listbox
			uilabel(f,'Position',[10 210 120 20],'Text','Channel:');
			chDrop = uidropdown(f,'Position',[140 210 320 22],'Items',{});

			% Tag field and Add Plot button
			uilabel(f,'Position',[10 180 120 20],'Text','Plot tag (optional):');
			tagField = uieditfield(f,'text','Position',[140 180 200 22]);
			btnAdd = uibutton(f,'push','Position',[350 180 110 28],'Text','Add Plot', 'ButtonPushedFcn',@(s,e) onAddPlot());

			% Existing plots list
			uilabel(f,'Position',[10 100 120 20],'Text','Existing plots:');
			plotsList = uilistbox(f,'Position',[140 60 220 100]);

			btnToggle = uibutton(f,'push','Position',[370 120 90 28],'Text','Toggle Visible', 'ButtonPushedFcn',@(s,e) onToggle());
			btnRemove = uibutton(f,'push','Position',[370 90 90 28],'Text','Remove', 'ButtonPushedFcn',@(s,e) onRemove());
			% edit tag controls for selected plot
			uilabel(f,'Position',[10 40 120 16],'Text','Edit selected tag:');
			editTag = uieditfield(f,'text','Position',[140 40 200 22]);
			btnUpdateTag = uibutton(f,'push','Position',[350 40 110 28],'Text','Update Tag','ButtonPushedFcn',@(s,e) onUpdateTag());

			% when plot selection changes, populate editTag
			plotsList.ValueChangedFcn = @(s,e) onPlotSelectionChanged();

			% store controls
			obj.PlotMgrControls.DatasetDropDown = dd;
			obj.PlotMgrControls.ChannelDrop = chDrop;
			obj.PlotMgrControls.TagField = tagField;
			obj.PlotMgrControls.PlotsList = plotsList;
			obj.PlotMgrControls.EditTag = editTag;
			obj.PlotMgrControls.BtnUpdateTag = btnUpdateTag;
			obj.PlotMgrControls.BtnAdd = btnAdd;

			% populate initial lists
			updateChannels();
			obj.updatePlotManagerUI();

			% nested callbacks
			function onDatasetChanged(e)
				updateChannels();
			end

			function updateChannels()
				items = dd.Items;
				if isempty(items)
					chDrop.Items = {};
					return;
				end
				val = dd.Value;
				if isempty(val)
					idx = 1;
				else
					idx = find(strcmp(items,val),1);
					if isempty(idx), idx = 1; end
				end
				if numel(obj.DataList) >= idx && ~isempty(obj.DataList{idx})
					sd = obj.DataList{idx};
					chDrop.Items = sd.chNames;
				else
					chDrop.Items = {};
				end
			end

			function onAddPlot()
				% figure out selected dataset index
				items = dd.Items;
				if isempty(items), return; end
				val = dd.Value;
				idx = find(strcmp(items,val),1);
				if isempty(idx), idx = 1; end
				chSel = chDrop.Value;
				if isempty(chSel), return; end
				% map channel name to index
				sd = obj.DataList{idx};
				chIdx = find(strcmp(sd.chNames,chSel),1);
				if isempty(chIdx), return; end
				% get optional tag
				tagstr = strtrim(tagField.Value);
				if isempty(tagstr)
					obj.addPlot(idx,chIdx);
				else
					obj.addPlot(idx,chIdx,tagstr);
				end
				obj.updatePlotManagerUI();
				obj.updateDataListUI();
			end

			function onToggle()
				sel = plotsList.Value;
				if isempty(sel), return; end
				% find matching plot index
				plist = plotsList.Items;
				pidx = find(strcmp(plist,sel),1);
				if isempty(pidx), return; end
				obj.toggleSelectedPlotVisibility(pidx);
				obj.updatePlotManagerUI();
				obj.updateDataListUI();
			end

			function onRemove()
				sel = plotsList.Value;
				if isempty(sel), return; end
				plist = plotsList.Items;
				pidx = find(strcmp(plist,sel),1);
				if isempty(pidx), return; end
				obj.removeSelectedPlot(pidx);
				obj.updatePlotManagerUI();
				obj.updateDataListUI();
			end

			function onPlotSelectionChanged()
				sel = plotsList.Value;
				if isempty(sel), editTag.Value = ''; return; end
				plist = plotsList.Items;
				pidx = find(strcmp(plist,sel),1);
				if isempty(pidx), editTag.Value = ''; return; end
				if numel(obj.Plots) >= pidx && isfield(obj.Plots{pidx},'tag')
					editTag.Value = obj.Plots{pidx}.tag;
				else
					editTag.Value = '';
				end
			end

			function onUpdateTag()
				sel = plotsList.Value;
				if isempty(sel), return; end
				plist = plotsList.Items;
				pidx = find(strcmp(plist,sel),1);
				if isempty(pidx), return; end
				newtag = strtrim(editTag.Value);
				if isempty(newtag), return; end
				% update stored tag and line DisplayName
				obj.Plots{pidx}.tag = newtag;
				obj.Plots{pidx}.name = sprintf('%s - %s', newtag, obj.DataList{obj.Plots{pidx}.dataIdx}.chNames{obj.Plots{pidx}.chIdx});
				if isgraphics(obj.Plots{pidx}.line)
					obj.Plots{pidx}.line.DisplayName = obj.Plots{pidx}.name;
				end
				obj.updatePlotManagerUI();
				obj.updateDataListUI();
				obj.updateTimeLegend();
				obj.computeAndPlotFFT();
			end
		end

		function startLineDrag(obj, src, ~)
			% Begin interactive drag of a plotted line. We record original data
			% and set UIFigure callbacks for motion and button up.
			if obj.DragState.active
				return;
			end
			% find plot index for this line
			pidx = [];
			for i=1:numel(obj.Plots)
				if isgraphics(obj.Plots{i}.line) && obj.Plots{i}.line==src
					pidx = i; break;
				end
			end
			if isempty(pidx), return; end
			obj.DragState.active = true;
			obj.DragState.line = src;
			obj.DragState.origX = src.XData;
			obj.DragState.origY = src.YData;
			% store the starting mouse point in axes coords
			cp = obj.AxTime.CurrentPoint;
			obj.DragState.startPoint = cp(1,1:2);
			obj.DragState.plotIdx = pidx;
			% attach motion and up handlers to UIFigure
			obj.UIFigure.WindowButtonMotionFcn = @(s,e) obj.onDragMotion();
			obj.UIFigure.WindowButtonUpFcn = @(s,e) obj.endLineDrag();
		end

		function onDragMotion(obj)
			if ~obj.DragState.active, return; end
			% get current point in axes
			cp = obj.AxTime.CurrentPoint;
			cur = cp(1,1:2);
			dx = cur(1) - obj.DragState.startPoint(1);
			dy = cur(2) - obj.DragState.startPoint(2);
			% update line display
			try
				nx = obj.DragState.origX + dx;
				ny = obj.DragState.origY + dy;
				obj.DragState.line.XData = nx;
				obj.DragState.line.YData = ny;
			catch
			end
		end

		function endLineDrag(obj)
			if ~obj.DragState.active, return; end
			% compute total shift
			cp = obj.AxTime.CurrentPoint;
			cur = cp(1,1:2);
			dx = cur(1) - obj.DragState.startPoint(1);
			dy = cur(2) - obj.DragState.startPoint(2);
			pidx = obj.DragState.plotIdx;
			if isempty(pidx) || pidx>numel(obj.Plots)
				obj.DragState.active = false;
				obj.UIFigure.WindowButtonMotionFcn = '';
				obj.UIFigure.WindowButtonUpFcn = '';
				return;
			end
			p = obj.Plots{pidx};
			% update underlying SignalData offsets
			sd = obj.DataList{p.dataIdx};
			% time offset in seconds (add)
			sd.setTimeOffset(sd.timeOffset + dx);
			% signal offset for that channel: subtract drag delta so displayed trace
			% (sig - offset) shifts by dy when offset decreases by dy
			if isempty(sd.sigOffset), sd.sigOffset = zeros(1,size(sd.sig,2)); end
			sd.setSigOffset(p.chIdx, sd.sigOffset(p.chIdx) - dy);
			% refresh plotted data for all plots that use this same dataIdx & chIdx
			for i=1:numel(obj.Plots)
				pp = obj.Plots{i};
				if pp.dataIdx==p.dataIdx && pp.chIdx==p.chIdx && isgraphics(pp.line)
					% update X/Y using offsets (baseline-subtracted)
					ts = sd.getTimeWithOffset();
					if ~isempty(sd.sigOffset) && numel(sd.sigOffset)>=pp.chIdx
						yy = sd.sig(:,pp.chIdx) - sd.sigOffset(pp.chIdx);
					else
						yy = sd.sig(:,pp.chIdx);
					end
					pp.line.XData = ts;
					pp.line.YData = yy;
				end
			end
			% finalize
			obj.DragState.active = false;
			obj.UIFigure.WindowButtonMotionFcn = '';
			obj.UIFigure.WindowButtonUpFcn = '';
			% recompute FFT since offsets changed
			obj.computeAndPlotFFT();
		end

		function addPlot(obj, dataIdx, chIdx, tag)
			% create a line plot on AxTime for given dataset and channel
			if dataIdx<1 || dataIdx>numel(obj.DataList), return; end
			sd = obj.DataList{dataIdx};
			if chIdx<1 || chIdx>numel(sd.chNames), return; end
			% plot baseline-subtracted signal (signal - offset)
			t = sd.getTimeWithOffset();
			if ~isempty(sd.sigOffset) && numel(sd.sigOffset)>=chIdx
				sig = sd.sig(:,chIdx) - sd.sigOffset(chIdx);
			else
				sig = sd.sig(:,chIdx);
			end
			ax = obj.AxTime;
			% determine tag
			if nargin<4 || isempty(tag)
				% default tag from SignalData filename + channel
				tag = sprintf('%s - Ch%d', sd.filename, chIdx);
			end
			h = plot(ax, t, sig, 'DisplayName', sprintf('%s - %s', tag, sd.chNames{chIdx}));
			h.Tag = sprintf('plot_%d_%d', dataIdx, chIdx);
			h.LineWidth = 1.2;
			h.Color = rand(1,3);
			h.Visible = 'on';
			h.Parent = ax;
			% enable dragging on the plotted line
			h.ButtonDownFcn = @(s,e) obj.startLineDrag(s,e);
			hold(ax,'on');
			% store
			p.name = sprintf('%s - %s', tag, sd.chNames{chIdx});
			p.dataIdx = dataIdx;
			p.chIdx = chIdx;
			p.line = h;
			p.tag = tag;
			obj.Plots{end+1} = p; %#ok<AGROW>
			% set newly added plot as selected and update spectrogram
			obj.SelectedPlotIdx = numel(obj.Plots);
			obj.updateTimeLegend();
			obj.updateSpectrogram();
		end

		function removeSelectedPlot(obj, pidx)
			if pidx<1 || pidx>numel(obj.Plots), return; end
			p = obj.Plots{pidx};
			if isgraphics(p.line)
				delete(p.line);
			end
			obj.Plots(pidx) = [];
			% update top table and legend
			obj.updateDataListUI();
			obj.updateTimeLegend();
			% adjust selected plot index if needed
			if isempty(obj.Plots)
				obj.SelectedPlotIdx = [];
			else
				obj.SelectedPlotIdx = min(obj.SelectedPlotIdx, numel(obj.Plots));
			end
			obj.updateSpectrogram();
		end

		function toggleSelectedPlotVisibility(obj, pidx)
			if pidx<1 || pidx>numel(obj.Plots), return; end
			p = obj.Plots{pidx};
			if ~isgraphics(p.line), return; end
			if strcmp(p.line.Visible,'on')
				p.line.Visible = 'off';
			else
				p.line.Visible = 'on';
			end
			obj.Plots{pidx} = p;
			obj.updateTimeLegend();
		end

		function updatePlotManagerUI(obj)
			% Safely update the Plot Manager list. The Plot Manager UI may not
			% exist (or its controls may have been deleted), so guard for that.
			if isempty(obj.PlotMgrControls) || ~isfield(obj.PlotMgrControls,'PlotsList') || isempty(obj.PlotMgrControls.PlotsList) || ~isgraphics(obj.PlotMgrControls.PlotsList)
				return;
			end
			pl = cell(1,numel(obj.Plots));
			for i=1:numel(obj.Plots)
				pl{i} = obj.Plots{i}.name;
			end
			obj.PlotMgrControls.PlotsList.Items = pl;
			if ~isempty(pl)
				obj.PlotMgrControls.PlotsList.Value = pl{end};
			end
		end

		function onSaveSession(obj, ~, ~)
			[fn, pn] = uiputfile('*.json','Save session as');
			if isequal(fn,0), return; end
			fullp = fullfile(pn,fn);
			% build session struct
			sess.dataFiles = cell(1,numel(obj.DataList));
			for i=1:numel(obj.DataList)
				sd = obj.DataList{i};
				sess.dataFiles{i}.filepath = sd.filepath;
				sess.dataFiles{i}.timeOffset = sd.timeOffset;
				sess.dataFiles{i}.sigOffset = sd.sigOffset;
			end
			sess.plots = cell(1,numel(obj.Plots));
			for i=1:numel(obj.Plots)
				p = obj.Plots{i};
				sess.plots{i} = struct('dataIdx',p.dataIdx,'chIdx',p.chIdx,'tag',p.tag,'visible',isgraphics(p.line) && strcmp(p.line.Visible,'on'));
			end
			sess.pans = cell(1,numel(obj.Pans));
			for i=1:numel(obj.Pans)
				pr = obj.Pans{i};
				sess.pans{i} = struct('start',pr.start,'end',pr.end);
			end
			% try to produce pretty-printed JSON when supported by MATLAB
			try
				jsonstr = jsonencode(sess,'PrettyPrint',true);
			catch
				jsonstr = jsonencode(sess);
			end
			fid = fopen(fullp,'w');
			if fid==-1
				uialert(obj.UIFigure,sprintf('Failed to open %s for writing',fullp),'Save error');
				return;
			end
			% write as characters (preserves newlines if present)
			fwrite(fid, jsonstr, 'char');
			fclose(fid);
			uialert(obj.UIFigure,sprintf('Session saved to %s',fullp),'Saved');
			% bring main UIFigure to front/focus
			try
				figure(obj.UIFigure);
				drawnow;
			catch
				% ignore if not supported
			end
		end

		function onLoadSession(obj, ~, ~)
			[fn, pn] = uigetfile('*.json','Open session');
			if isequal(fn,0), return; end
			fullp = fullfile(pn,fn);
			txt = fileread(fullp);
			try
				sess = jsondecode(txt);
			catch
				uialert(obj.UIFigure,'Failed to read session file','Error');
				return;
			end
			% clear existing data
			% delete plots
			for i=numel(obj.Plots):-1:1
				obj.removeSelectedPlot(i);
			end
			% delete pans
			for i=numel(obj.Pans):-1:1
				obj.removePan(i);
			end
			obj.DataList = {};
			% load files
			for i=1:numel(sess.dataFiles)
				entry = sess.dataFiles(i);
				try
					sd = SignalData(entry.filepath);
					% set offsets
					if isfield(entry,'timeOffset'), sd.timeOffset = entry.timeOffset; end
					if isfield(entry,'sigOffset'), sd.sigOffset = entry.sigOffset; end
					obj.DataList{end+1} = sd; %#ok<AGROW>
				catch
					warning('Interview:LoadSession','Failed to load %s', entry.filepath);
				end
			end
			obj.updateDataListUI();
			% restore plots
			for i=1:numel(sess.plots)
				p = sess.plots(i);
				if p.dataIdx <= numel(obj.DataList)
					obj.addPlot(p.dataIdx, p.chIdx, p.tag);
					if isfield(p,'visible') && ~p.visible
						% hide
						obj.Plots{end}.line.Visible = 'off';
					end
				end
			end
			% restore pans
			for i=1:numel(sess.pans)
				pr = sess.pans(i);
				obj.createPan(pr.start, pr.end);
			end
			obj.updatePlotManagerUI();
			obj.updateDataListUI();
			obj.updatePanManagerUI();
			% refresh time legend after restoring plots
			obj.updateTimeLegend();
			% select first plot if available and update spectrogram
			if ~isempty(obj.Plots)
				obj.SelectedPlotIdx = 1;
				obj.updateSpectrogram();
			end
			obj.computeAndPlotFFT();
			uialert(obj.UIFigure,sprintf('Session loaded from %s',fullp),'Loaded');
			% bring main UIFigure to front/focus
			try
				figure(obj.UIFigure);
				drawnow;
			catch
				% ignore
			end
		end

        
	end
end
