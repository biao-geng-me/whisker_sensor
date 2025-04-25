function wavi_sampling(s,A_fft,Fs,ns_read,options)
    % WAVI_SAMPLING  Read data from serial port and plot
    %   WAVI_SAMPLING(s,A_fft,Fs,ns_read,options) reads data from serial port s
    arguments
        s {mustBeA(s,'internal.Serialport')}
        A_fft (1,1) double {mustBeNonnegative}
        Fs (1,1) double {mustBePositive}
        ns_read (1,1) double {mustBeInteger,mustBePositive} % number of samples to read in each loop
        options.tag = '';
        options.tmax = 3600*10; % maximum run time
        options.outpath = '.';
        options.showtrace = true;
        options.showfft = true;
        options.show_spectrogram = true;
        options.show_spectrum = true;
        options.nsensor = 1;
        options.t_fft = 1;
        options.scale = 1;
    end
    nsensor = options.nsensor;
    tag = options.tag;
    tmax = options.tmax;
    set(0, 'DefaultAxesFontSize', 20);

    if ~exist(options.outpath,'dir')
        mkdir(options.outpath);
    end
    
    % settings
    % lgd_fs = 120; % legend fontsize
    tbuffer = 30; % buffer time, seconds
    tdis = 20;     % display time, seconds
    t_fft = min(tbuffer,options.t_fft);   % fft time length
    t_spec = 20; % spectrogram duration

    % an_pos = [0.5,0.6,1.0,0.3]; % position of annotation
    % an_pos2 = [0.5,0.5,1.0,0.3];

    nch = nsensor*2;

    %%% setup data buffers
    % maximum buffered length
    ns_tot = tbuffer*Fs;
    sig = nan(ns_tot,nch);
    darr = linspace(datetime('now')-seconds(tbuffer),datetime('now'),ns_tot);
    darr.Format = 'dd-MMM-uuuu HH:mm:ss.SSS';  
    
    % portion of buffer for display
    ns_dis = Fs* tdis;

    % reading buffer
    buff = nan(ns_read,nch);

    % fft result buffer
    t_fft = round(Fs*t_fft)/Fs; % adjust to integer number of samples
    nfreq = (round(Fs/2*t_fft)) + 1; % maximum frequency / resolution + 1
    spec_data = zeros(nfreq*nch,t_spec*ceil(Fs/ns_read));
    dt_spec = ns_read/Fs;

    %%% setup plots

    % time history window
    display_info=get(groot);
    pos1 = display_info.MonitorPositions(1,:);
    fh2 = figure('OuterPosition',[0,61,pos1(3)*0.5,pos1(4)-80]);hold on;grid on;box on
    for i=1:nsensor
        ln_sig(i*2-1) = line(darr,sig(:,i*2-1),'Color','r','LineWidth',2,'Marker','+');
        ln_sig(i*2) = line(darr,sig(:,i*2),'Color','b','LineWidth',2,'Marker','None');
    end

    % fft
    if options.showfft
        [fh1,sh,bh,fft_ax1,fft_ax2] = init_fft_surf(nch,nfreq,t_fft);
    end

    % spectrogram
    if options.show_spectrogram
        [fh3,mh] = init_spec_plot(spec_data,Fs,ns_read);
    end

    % trace
    if options.showtrace
        [fh4,hfar,htrace] = init_trace(Fs,sig,nsensor);
    end

    % spectrum
    if options.show_spectrum
        [fh5,fft_lines] = init_spectrum(nsensor,nfreq,t_fft);
    end
    %%% done 
    
    % Get current date and time
    currentTime = datetime('now');
    
    % Extract desired format for filename (year, month, day, hour, minute, second)
    fileName = sprintf('st_%04d-%02d-%02d_%02d%02d_%05.2f_%s.dat',currentTime.Year,...
                                                                  currentTime.Month,...
                                                                  currentTime.Day,...
                                                                  currentTime.Hour,...
                                                                  currentTime.Minute,...
                                                                  currentTime.Second,...
                                                                  tag);
    
    % Open a new file with the generated filename and '.txt' extension
    if ~exist(options.outpath,'dir')
        mkdir(options.outpath);
    end

    fid = fopen(fullfile(options.outpath,fileName), 'w');
    C = onCleanup(@()cleanUpFunc(s,fid));

    darr(end) = datetime('now'); % measurement starttime
    dt_prev = darr(end);
        
    % get initial values
    tic
    flush(s);
    % align data read
    % the value 2024 is a marker to indicate the start of a data frame (see HX711_array Arduino code)
    % this while loop is necessary because the line feed \n can randomly appear in the data stream
    tmpv =0;
    fprintf('searching frame start\n');
    ncount = 0;
    while(tmpv~=2024)
        fprintf('.')
        ncount = ncount + 1;
        readline(s);
        tmpv = read(s,1,'single');
        if mod(ncount,100)==0
            fprintf('\n');
        end
    end
    % clear old frames from serial buffer
    fprintf('clearing buffer\n');
    while(s.NumBytesAvailable>(nch+1)*4)
        read(s,(nch+1)*4+1,'char');
    end
    fprintf('%g seconds to find sample start\n',toc)

    % initial values
    for j=1:round(Fs)
        V0 = read(s,nch,'single');
        readline(s);
        read(s,1,'single');
    end 

    fprintf(['\n' repmat(' %12.6f',1,nch) '\n'],V0);

    t_loop_start = tic;
    tcycle = 0;
    t_read = 0;
    nc_avg = ceil(Fs/ns_read);
    dt_loop_start = datetime('now');

    nc_read=0;

    while (1)
        nc_read=nc_read+1;
        tc_s = tic; % cycle start
        % shift buffer
        sig(1:end-ns_read,:) = sig(ns_read+1:end,:);
        darr(1:end-ns_read) = darr(ns_read+1:end);
        
        % read serial port
        tic
        for j=1:ns_read
            buff(j,:) = read(s,nch,'single');
            readline(s);
            read(s,1,'single');
        end
        t_read = t_read+toc;
        
        sig(end-ns_read+1:end,:) = filloutliers(buff,'linear');
        darr(end-ns_read+1:end) = linspace(darr(end-ns_read)+seconds(1/Fs),...
                                           darr(end-ns_read)+seconds(ns_read/Fs),ns_read);

        if 1==1 % update plot
            % line plot
            set(0,'CurrentFigure',fh2);
            for i=1:nsensor
                for j=1:2
                    ich = (i-1)*2+j;
                    ln_sig(ich).XData = darr; 
                    ln_sig(ich).YData = (sig(:,ich)-V0(ich))*options.scale+i;
                end
            end

            xlim([darr(1) darr(end)])
            ylim([0 nsensor+1]);

            % compute fft
            if(options.showfft || options.show_spectrogram || options.show_spectrum)
                % shift spectrogram
                spec_data(:,1:end-1) = spec_data(:,2:end);
            
                for i=1:size(sig,2) % channels
                    [~,pp] = fast_fourier(sig(floor(end-t_fft*Fs+1):end,i),Fs);
                    n = numel(pp);
                    ind = (1:n) + (i-1)*nfreq;
                    spec_data(ind,end) = pp;
                end 
            end

            % fh1, fft surface
            if(options.showfft)
                set(0,'CurrentFigure',fh1);
                fft_map = reshape(spec_data(:,end),nfreq,nch);
                update_fft_surf(fh1,sh,bh,fft_map,fft_ax1,fft_ax2);
            end 

            % fh3, spectrogram
            if(options.show_spectrogram)
                set(0,'CurrentFigure',fh3);
                update_spec_plot(mh,spec_data);
            end 

            % fh4, trace
            if options.showtrace
                set(0,'CurrentFigure',fh4);
                qd = update_trace(fh4,htrace,hfar,sig(end-Fs:end,:)-V0,Fs,nsensor);
                % title(sprintf('%5.1f° %s',round(qd)),datetime('now')-dt_loop_start);
            end

            % fh5, spectrum
            if options.show_spectrum
                set(0,'CurrentFigure',fh5);
                fft_map = reshape(spec_data(:,end),nfreq,nch);
                update_spectrum(fft_lines,fft_map,A_fft);
            end
            drawnow
        end % plot
    
        % save data
        for j=1:ns_read
            currentTime = darr(end-ns_read+j);
            dtstr = sprintf('%04d-%02d-%02d %02d:%02d:%06.3f',currentTime.Year,...
                                                              currentTime.Month,...
                                                              currentTime.Day,...
                                                              currentTime.Hour,...
                                                              currentTime.Minute,...
                                                              currentTime.Second);
        
            fprintf(fid,['%s' repmat(' %12.6f',1,nch) '\n'],dtstr,sig(end-ns_read+j,:));
        end
    
        % fprintf('%s %12.5f %12.3f\n',dtstr,sig(end),toc(tc_s));
    
        if (toc(t_loop_start)>tmax)
            fprintf('maximum time reached\n');
        % cleanUpFunc(s,fid);
            return
        end
        tcycle = tcycle + toc(tc_s);

        if (nc_read==nc_avg)
            
            fprintf(['%5d %7.3f %7.3f %7.1f Hz\n\t' repmat('%7.3f',1,nch) '\n'],...
                    size(buff,1),t_read/nc_avg,tcycle/nc_avg, nc_avg/tcycle*ns_read,...
                    mean(buff) );
            tcycle = 0;
            t_read = 0;
            nc_read =0;
        end
    end
end

function cleanUpFunc(s,fid)
    fprintf('file id %d\n',fid)
    fclose(fid);
end

