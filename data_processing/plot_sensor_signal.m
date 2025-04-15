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
        opt.title = [];
        opt.clickable = false;
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
    plot_history(ax1)
    % hold on;box on;grid on
    % set(gca, 'FontName', 'Times', 'FontSize', fontsize);
    % plot(t,y1,'-','LineWidth',1.5);
    % plot(t,y2,'-')

    % xlim(t([1 end]))
    % ylabel('Signal (mV)')
    % legend('Ch1','Ch2')
    % title(tag)
    
    % -------------
    % spectrogram 1
    % -------------
    [tt,ff,ss1] = aspectro(y1,Fs,"overlap",ol,"window_size",Fs*t_fft);
    if log_scale
        ss = log(ss1)./log(10);
    else
        ss = ss1;
    end

    f_idx = ff>=f_range(1) & ff<=f_range(2);
    ax2 = subplot(3,1,2);
    [~,ch1] = contourf(tt,ff(f_idx),ss(f_idx,:),LineColor='None');
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
    [tt,ff,ss2] = aspectro(y2,Fs,"overlap",ol,"window_size",Fs*t_fft);
    if log_scale
        ss = log(ss2)./log(10);
    else
        ss = ss2;
    end
    ax3 = subplot(3,1,3);
    [~,ch2] = contourf(tt,ff,ss,LineColor='None');
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

    if opt.clickable
        set(ch1, 'ButtonDownFcn', @(src,evt) plot_amp_line(src,evt,1),'PickableParts','all');
        set(ch2, 'ButtonDownFcn', @(src,evt) plot_amp_line(src,evt,2),'PickableParts','all');
        % create global variables
        amp_history_fh = figure();hold on;box on;grid on
        axx1 = subplot(2,1,1);
        plot_history(axx1);
        axx2 = subplot(2,1,2);
        amp_line_legends = {};n_amp_lines = 0;
        amp_line_IDs = [];
        linkaxes([axx1,axx2],'x')
    end

    function plot_history(ax)
        axes(ax)
        hold on;box on;grid on
        set(gca, 'FontName', 'Times', 'FontSize', fontsize);
        plot(t,y1,'-r','LineWidth',1.5);
        plot(t,y2,'-b','LineWidth',1.5)
    
        xlim(t([1 end]))
        ylabel('Signal (mV)')
        legend('Ch1','Ch2')
        title(tag)
    end

    function plot_amp_line(~,~,ich)
        % pick a frequency from the contour and show the amplitude change with time
        coords = get(gca, 'CurrentPoint');
        tpick = coords(1,1);
        fpick = coords(1,2);
        fprintf('click at [%5.3f, %5.3f]\n', tpick, fpick);
        [~,ifreq] = min(abs(ff-fpick));
        fprintf('cloest resolved frequency %f4.1\n',ff(ifreq));

        if ~isvalid(amp_history_fh)
            % figure closed
            amp_history_fh = figure();hold on;box on;grid on
            axx1 = subplot(2,1,1);
            plot_history(axx1);
            axx2 = subplot(2,1,2);
            amp_line_legends = {};n_amp_lines = 0;
            amp_line_IDs = [];
            linkaxes([axx1,axx2],'x')
        else
            figure(amp_history_fh)
        end

        lgd = sprintf('Ch%d %3.1fHz',ich,ff(ifreq));

        if ich==1
            s = ss1;
        elseif ich==2
            s = ss2;
        end

        line_ID = [ich ifreq];
        if ~isempty(amp_line_IDs) && ismember(line_ID,amp_line_IDs,'rows')
            fprintf('%s already plotted\n', lgd);
            return
        end
        amp_line_IDs = [amp_line_IDs; line_ID];
        n_amp_lines = n_amp_lines + 1;
        amp_line_legends{n_amp_lines} = lgd;

        subplot(2,1,2);hold on;box on;grid on
        set(gca, 'FontName', 'Times', 'FontSize', fontsize);
        plot(tt,mean(s(ifreq-1:ifreq+1,:)),LineWidth=2)
        xlim(t([1 end]))
        xlabel('Time (s)')
        ylabel('Amp (mV)');
        legend(amp_line_legends);

    end
end