function [ff,pp1,pp2,fh] = sensor_signal_spectrum(datalog,isensor,opt)
    % compute and plot sensor signal spectrum
    arguments
        datalog
        isensor
        opt.t_range = []
        opt.show_fft = false
        opt.title = []
    end
    t_range = opt.t_range;
    % load data
    
    dat = load_datalog(datalog);

    dtime = datetime([datestr(dat{:,1}) datestr(dat{:,2},' HH:MM:SS.FFF')]);
    ttime = dtime - dtime(1);
    t = seconds(ttime); % time in seconds
    Fs = (numel(t)-1)/t(end);

    y1 = dat{:,isensor*2+1};
    y2 = dat{:,isensor*2+2};

    if ~isempty(t_range)
        idx = t>t_range(1) & t<t_range(2);
        y1 = y1(idx);
        y2 = y2(idx);
    end

    [ff,pp1] = fast_fourier(y1,Fs);
    [ff,pp2] = fast_fourier(y2,Fs);
    fh = [];
    if opt.show_fft
        fh = figure(); hold on; box on; grid on
        plot(ff,pp1,'-r','LineWidth',2);
        plot(ff,pp2,'-b','LineWidth',2);
        if ~isempty(opt.title)
            title(opt.title)
        end
        xlabel ('Frequency (Hz)')
        ylabel ('Amplitude (mV)')

        if ~isempty(t_range)
            title(sprintf('Time range: [%g,%g]',t_range));
        end
    end

