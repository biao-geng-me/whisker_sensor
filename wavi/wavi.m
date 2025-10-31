classdef wavi < handle
    %WAVI Whisker Array data acquisition and Visualization
    %   A class version of the wavi_sampling function for integration with
    %   motor control

    properties % UI
        UIFigure
        ui_parent
        gl
        button_gl
        SerialPanel
        SerialApp

        line_fig
        fft_fig
        spec_fig
        is_standalone
        % runtime helpers
        readTimer % matlab timer object used for fixed-rate reading
        readCount = 0 % number of timer ticks / read iterations performed
        fout % file handle for data logging
    end
    
    properties % sampling settings
        s 
        baudrate = 2000000
        A_fft
        Fs
        ns_read % number of samples to read in each loop
        tag
        tmax % maximum run time
        outpath = '.';
        show_line = true;
        show_trace = true;
        show_fft = true; 
        show_spectrogram = true;
        show_spectrum = true;
        nsensor = 1;
        nch = 1;
        t_fft = 1;
        scale = 1;
        ch_map = [];
        t_buffer = 30; % time length for signal buffer, seconds
        t_spec = 20; % spectrogram duration
        n_update = 1; % visualization update frequency, multiple of ns_read, i.e. update every n_update read
        ns_fill % number of samples, window size for filtering outliers
    end

    properties % sensing data
        V0 % offset
        ns_tot % total number of samples in buffer
        darr  % datetime array
        sig   % signal data
        ns_spec % number of samples to do spec
        spec_data % spectrogram data (fft history)
        fft_map % latest fft results nfreq*nch
        nfreq
    end
    
    methods
        function obj = wavi(ui_parent, is_standalone, options)
            %WAVI_SAMPLER Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                ui_parent = []
                is_standalone (1,1) logical = true
                options.A_fft (1,1) double {mustBeNonnegative} = 0
                options.Fs (1,1) double {mustBePositive} = 80
                options.ns_read (1,1) double {mustBeInteger,mustBePositive} = 4 % number of samples to read in each loop
                options.tag = '';
                options.tmax = 3600*10; % maximum run time
                options.outpath = [];
                options.show_line = true;
                options.show_trace = true;
                options.show_fft = true; 
                options.show_spectrogram = true;
                options.show_spectrum = true;
                options.nsensor = 9;
                options.t_fft = 1;
                options.scale = 1;
                options.ch_map = [];
                options.ns_spec = [];
                options.n_update = 1; % visualization update frequency, multiple of ns_read, i.e. update every n_update read
                options.ns_fill = 6;
                options.baudrate = 2000000;
            end

            % if isempty(s)
            %     obj.s=serialport("COM18",2000000); % change COM number accordingly
            % else
            %     obj.s = s;
            % end
            obj.s = [];
            obj.is_standalone = is_standalone;

            if isempty(ui_parent)
                obj.UIFigure = uifigure('Name','Wavi','Position',[100 100 200 300]);
                obj.ui_parent = obj.UIFigure;
            else
                obj.ui_parent = ui_parent;
                obj.UIFigure = ancestor(ui_parent,'matlab.ui.Figure','toplevel');
            end
           
            % populate properties
            obj.A_fft            = options.A_fft;
            obj.Fs               = options.Fs;
            obj.ns_read          = options.ns_read;
            obj.tag              = options.tag;
            obj.tmax             = options.tmax;

            if isempty(options.outpath)
                obj.outpath = obj.set_outpath();
            else
                obj.outpath = options.outpath;
            end

            obj.show_line        = options.show_line;
            obj.show_trace       = options.show_trace;
            obj.show_fft         = options.show_fft;
            obj.show_spectrogram = options.show_spectrogram;
            obj.show_spectrum    = options.show_spectrum;
            obj.nsensor          = options.nsensor;
            obj.t_fft            = options.t_fft;
            obj.scale            = options.scale;
            obj.n_update         = options.n_update;
            obj.ns_fill          = options.ns_fill;

            obj.baudrate = options.baudrate;
            
            obj.nch = obj.nsensor*2;

            if isempty(options.ch_map)
                obj.ch_map = 1:obj.nch;
            else
                obj.ch_map = options.ch_map;
            end

            %%% setup data buffers
            % maximum buffered length
            obj.ns_tot = round(obj.t_buffer*obj.Fs);
            obj.sig = nan(obj.ns_tot,obj.nch);
            obj.darr = linspace(datetime('now')-seconds(obj.t_buffer),datetime('now'),obj.ns_tot);
            obj.darr.Format = 'dd-MMM-uuuu HH:mm:ss.SSS';  

            obj.ns_spec = obj.ns_read*obj.n_update;
    
            obj.t_fft = round(obj.Fs*obj.t_fft)/obj.Fs; % adjust to integer number of samples
            obj.nfreq = (round(obj.Fs/2*obj.t_fft)) + 1; % maximum frequency / resolution + 1
            obj.spec_data = zeros(obj.nfreq*obj.nch,obj.t_spec*ceil(obj.Fs/obj.ns_spec));
            obj.fft_map = reshape(obj.spec_data(:,end),obj.nfreq,obj.nch);

            % init graphics
            obj.fft_fig = fft_view(obj.nch,obj.nfreq,obj.t_fft);
            obj.spec_fig = spectrogram_view(obj.spec_data,obj.Fs,obj.ns_spec);
            obj.line_fig = line_view(obj.nsensor,obj.darr,obj.sig);
            
            set(obj.fft_fig.fh,'Visible','off');
            set(obj.spec_fig.fh,'Visible','off');
            set(obj.line_fig.fh,'Visible','off');

            % create ui
            obj.gl = uigridlayout(obj.ui_parent,[3,1],'RowHeight',{'3x','1x','1x'},'Padding',[10 10 10 10]);
                % Serial panel
            obj.SerialPanel = uipanel(obj.gl,'Title','Sensor array');
            obj.SerialPanel.Layout.Row = 1;
            obj.SerialPanel.Layout.Column = 1;
            obj.SerialApp = SerialPortApp(obj.SerialPanel);
            obj.SerialApp.baudrate = obj.baudrate;
            addlistener(obj.SerialApp,'SerialConnected',@(src,evt) obj.onSerialConnected(src,evt));
            % listen for serial disconnect to cleanup resources
            try
                addlistener(obj.SerialApp,'SerialDisconnected',@(src,evt) obj.onSerialDisconnected(src,evt));
            catch
                % SerialPortApp may not implement SerialDisconnected; ignore if absent
            end
            
            obj.button_gl = uigridlayout(obj.gl,[5,1],'ColumnWidth',{'1x','1x','1x','1x','1x'},'RowHeight',{'1x'},'Padding',[0 0 0 0]);
            fft_btn = FigureToggler(obj.fft_fig.fh, obj.button_gl, 'FFT', [0 0 20 20]);
            line_btn = FigureToggler(obj.line_fig.fh, obj.button_gl, 'Line', [0 0 20 40]);
            spec_btn = FigureToggler(obj.spec_fig.fh, obj.button_gl, 'Spec', [0 0 20 40]);

            drawnow

        end

        function outpath =set_outpath(obj)
            % set default output path to user home/wavi_data
            user_dir = getenv('USERPROFILE');
            if isempty(user_dir)
                user_dir = getenv('HOME');
            end
            if isempty(user_dir)
                user_dir = '.';
            end
            outpath = fullfile(user_dir,'wavi_data');
            if ~isfolder(outpath)
                mkdir(outpath);
            end
            obj.outpath = outpath;
        end
    end

    methods % event handlers
        function onSerialConnected(obj,src,evt)

            obj.s = obj.SerialApp.s;
            if ~obj.is_standalone
                return
            end

            try
                obj.run();
            catch me
                disp(me);
            end
        end

        function onSerialDisconnected(obj, src, ~)
            % Called when SerialPortApp emits SerialDisconnected (if available)
            try
                obj.cleanup();
            catch me
                warning('wavi:onSerialDisconnected','Cleanup failed: %s', me.message);
            end
        end

        function onAppClose(obj)
            % Called by parent app when closing
            obj.cleanup();
            % stop and delete timer if running
            try
                if ~isempty(obj.readTimer) && isvalid(obj.readTimer)
                    stop(obj.readTimer);
                    delete(obj.readTimer);
                end
            catch
            end
        end

        function cleanup(obj)
            % Close serial port and data file (if open)
            try
                if ~isempty(obj.s)
                    % if serialport object
                    try
                        if isvalid(obj.s)
                            flush(obj.s);
                            delete(obj.s);
                        end
                    catch
                        % older MATLAB cannot isvalid serialport; attempt delete
                        try
                            delete(obj.s);
                        catch
                        end
                    end
                    obj.s = [];
                end
            catch me
                warning('wavi:cleanup','Error closing serial: %s', me.message);
            end

            try
                if ~isempty(obj.fout) && obj.fout ~= -1
                    fclose(obj.fout);
                    obj.fout = [];
                end
            catch me
                % warning('wavi:cleanup','Error closing file: %s', me.message);
            end
        end

    end

    methods % data acquisition

        function run(obj)
            % Start fixed-rate timer to read serial data in chunks of ns_read
            obj.align_data_read;
            obj.average_signal_as_offset(round(obj.Fs))

            period = floor(obj.ns_read / obj.Fs*1000)/1000; % seconds between timer callbacks
            if ~isempty(obj.readTimer) && isvalid(obj.readTimer)
                stop(obj.readTimer);
                delete(obj.readTimer);
            end

            obj.readCount = 0;
            obj.readTimer = timer( ...
                'ExecutionMode','fixedRate', ...
                'Period', period, ...
                'StartDelay', 0, ...
                'TasksToExecute', Inf, ...
                'BusyMode','queue', ...
                'TimerFcn', @(src,evt) obj.read_update_tick(src,evt) ...
            );
            obj.init_datalog_file();
            start(obj.readTimer);
        end

        function read_update_tick(obj, src, ~)
            % Timer callback: read serial data and update visuals/printing
            try
                obj.readCount = obj.readCount + 1;
                obj.read_serial_data(obj.ns_read, obj.ns_fill);

                if mod(obj.readCount, obj.n_update) == 0
                    obj.update_visuals();
                end

                if mod(obj.readCount, round(obj.Fs / obj.ns_read)) == 0
                    obj.print_sig_vals();
                end
                obj.write_data_samples();
            catch me
                % Stop timer on error to avoid silent failure loop
                warning('wavi:read_update_tick','Timer callback error: %s', me.message);
                if ~isempty(src) && isvalid(src)
                    stop(src);
                    delete(src);
                end
            end
        end

        function align_data_read(obj)
            % align data read
            % the value 2024 is a marker to indicate the start of a data frame (see HX711_array Arduino code)
            % this while loop is necessary because the line feed \n can randomly appear in the binary data stream

            tic
            flush(obj.s);

            tmpv =0;
            fprintf('searching frame start\n');
            ncount = 0;
            while(tmpv~=2024)
                fprintf('.')
                ncount = ncount + 1;
                readline(obj.s);
                tmpv = read(obj.s,1,'single');
                if mod(ncount,100)==0
                    fprintf('\n');
                end
            end

            % clear old frames from serial buffer
            fprintf('clearing buffer\n');
            while(obj.s.NumBytesAvailable>(obj.nch+1)*4)
                read(obj.s,(obj.nch+1)*4+1,'char');
            end
            fprintf('%g seconds to find sample start\n',toc);
        end

        function average_signal_as_offset(obj,nsamples)
            % read nsamples and take the average as the offset voltage
            obj.V0 = zeros(1,obj.nch);
            for j=1:nsamples
                obj.V0 = obj.V0 + read(obj.s,obj.nch,'single'); % sensor data
                readline(obj.s); % \n
                read(obj.s,1,'single'); % 2024
                
            end 
            obj.V0 = obj.V0(obj.ch_map)/nsamples;
            fprintf('Sensor voltage offset:\n')
            fprintf(['\n' repmat(' %8.3f',1,obj.nch) '\n\n'],obj.V0);
        end

        function read_serial_data(obj,ns_read, n_fill)
            % read ns_read samples from serial

            % shift buffer
            obj.sig(1:end-ns_read,:) = obj.sig(ns_read+1:end,:);
            obj.darr(1:end-ns_read) = obj.darr(ns_read+1:end);
            buff = zeros(ns_read,obj.nch);

            % read serial port
            % tic
            for j=1:ns_read
                buff(j,:) = read(obj.s,obj.nch,'single');
                readline(obj.s);
                read(obj.s,1,'single');
            end
            % t_read = toc;
            
            % put new data in
            obj.sig(end-ns_read+1:end,:) = buff(:,obj.ch_map);

            % replace outliers in near the end
            tmp = obj.sig(end-n_fill+1:end,:);
            obj.sig(end-n_fill+1:end,:) = filloutliers(tmp,'linear');

            obj.darr(end-ns_read+1:end) = linspace(obj.darr(end-ns_read)+seconds(1/obj.Fs),...
                                            obj.darr(end-ns_read)+seconds(ns_read/obj.Fs),ns_read);
        end
        
        function do_fft(obj)

            % shift spectrogram
            obj.spec_data(:,1:end-1) = obj.spec_data(:,2:end);
        
            for i=1:obj.nch % channels
                [~,pp] = fast_fourier(obj.sig(floor(end-obj.t_fft*obj.Fs+1):end,i),obj.Fs);
                n = numel(pp);
                ind = (1:n) + (i-1)*obj.nfreq;
                obj.spec_data(ind,end) = pp;
            end
            obj.fft_map = reshape(obj.spec_data(:,end),obj.nfreq,obj.nch);
        end

        function print_sig_vals(obj)
            fprintf([repmat('%7.3f',1,obj.nch) '\n'],obj.sig(end,:));
        end

        function update_visuals(obj)
            
            obj.line_fig.update(obj.darr,obj.sig,obj.V0,obj.scale)

            obj.do_fft();
            obj.fft_fig.update(obj.fft_map);
            obj.spec_fig.update(obj.spec_data);

            % drawnow limitrate
        end

        function init_datalog_file(obj)
            % Get current date and time
            currentTime = datetime('now');
            
            % Extract desired format for filename (year, month, day, hour, minute, second)
            fileName = sprintf('st_%04d-%02d-%02d_%02d%02d_%05.2f_%s.dat',currentTime.Year,...
                                                                        currentTime.Month,...
                                                                        currentTime.Day,...
                                                                        currentTime.Hour,...
                                                                        currentTime.Minute,...
                                                                        currentTime.Second,...
                                                                        obj.tag);
            obj.fout = fopen(fullfile(obj.outpath, fileName), 'w');
            if obj.fout == -1
                error('wavi:init_datalog_file','Failed to open file for writing: %s', fullfile(obj.outpath, fileName));
            else
                fprintf('Logging data to file: %s\n', fullfile(obj.outpath, fileName));
            end
        end

        function write_data_samples(obj)
            for j=1:obj.ns_read
                currentTime = obj.darr(end-obj.ns_read+j);
                dtstr = sprintf('%04d-%02d-%02d %02d:%02d:%06.3f',currentTime.Year,...
                                                                currentTime.Month,...
                                                                currentTime.Day,...
                                                                currentTime.Hour,...
                                                                currentTime.Minute,...
                                                                currentTime.Second);
            
                fprintf(obj.fout,['%s' repmat(' %12.6f',1,obj.nch) '\n'],dtstr,obj.sig(end-obj.ns_read+j,:));
            end
        end
    end

end

