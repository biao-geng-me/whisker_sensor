function fh=plot_sensor_signal(datalog,isensor,opt)
    % ------------------------------------------------------
    % plot a sensor signal hitory and spectrum for 1 sensor
    % ------------------------------------------------------
    %

    arguments
        datalog                 % full pathname of sensor datalog file
        isensor                 % sensor index in the array
        % settings
        opt.log_scale = false;
        opt.f_range = [0 44];   % for spectrogram
        opt.overlap = 0.75;     % spectrogram overlap
        opt.t_fft = 2;
        opt.fontsize = 12;
        opt.title = []
    end
    % settings
    log_scale = opt.log_scale;
    f_range   = opt.f_range;
    ol        = opt.overlap;
    t_fft     = opt.t_fft;
    fontsize  = opt.fontsize;
    if isempty(opt.title)
        tag = sprintf('Sensor %d',isensor);
    else
        tag = opt.title;
    end

    % --------- end of user input ---------------

    % load data
    dat = load_datalog(datalog);
    dtime = datetime([datestr(dat{:,1}) datestr(dat{:,2},' HH:MM:SS.FFF')]);
    ttime = dtime - dtime(1);
    t = seconds(ttime); % time in seconds
    Fs = (numel(t)-1)/t(end);

    y1 = dat{:,isensor*2+1};
    y2 = dat{:,isensor*2+2};
    
    y1 = y1-y1(1);
    y2 = y2-y2(1);
    
    % -------------
    % History plot
    % -------------
    fh = figure(WindowState="maximized");
    ax1 = subplot(3,1,1);
    hold on;box on;grid on
    set(gca, 'FontName', 'Times', 'FontSize', fontsize);
    plot(t,y1,'-','LineWidth',1.5);
    plot(t,y2,'-')

    xlim(t([1 end]))
    ylabel('Signal (mV)')
    legend('Ch1','Ch2')
    title(tag)
    
    % -------------
    % spectrogram 1
    % -------------
    [tt,ff,ss] = aspectro(y1,Fs,"overlap",ol,"window_size",Fs*t_fft);
    if log_scale
        ss = log(ss)./log(10);
    end

    f_idx = ff>=f_range(1) & ff<=f_range(2);
    ax2 = subplot(3,1,2);
    contourf(tt,ff(f_idx),ss(f_idx,:),LineColor='None')
    colormap jet
    colorbar

    xlim(t([1 end]))
    ylabel('Frequency (Hz)')
    ylim(ff([1 end]))
    if log_scale
        clim([-6 max(ss(:))])
    end
    title('Ch1')
    set(gca, 'FontName', 'Times', 'FontSize', fontsize);
    
    % -------------
    % spectrogram 2
    % -------------
    [tt,ff,ss] = aspectro(y2,Fs,"overlap",ol,"window_size",Fs*t_fft);
    if log_scale
        ss = log(ss)./log(10);
    end
    ax3 = subplot(3,1,3);
    contourf(tt,ff,ss,LineColor='None')
    colormap jet
    colorbar
    xlabel('Time (s)')
    xlim(t([1 end]))
    ylim(ff([1 end]))
    if log_scale
        clim([-6 max(ss(:))])
    end
    ylabel('Frequency (Hz)')
    title('Ch2')
    set(gca, 'FontName', 'Times', 'FontSize', fontsize);
    
    linkaxes([ax1,ax2,ax3],'x')
    ax1.Position(3) = ax3.Position(3);

end