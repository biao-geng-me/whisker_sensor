function fh = sensor_signal_trajectory(datalog,isensor,opt)
    % plot sensor signal trajectory (not used yet)
    arguments
        datalog
        isensor
        opt.t_range = []
        opt.title = []
    end
    t_range = opt.t_range;
    
    % load data
    dat = load_datalog(datalog);

    dtime = datetime([datestr(dat{:,1}) datestr(dat{:,2},' HH:MM:SS.FFF')]);
    ttime = dtime - dtime(1);
    t = seconds(ttime); % time in seconds

    y1 = dat{:,isensor*2+1};
    y2 = dat{:,isensor*2+2};

    if ~isempty(t_range)
        idx = t>t_range(1) & t<t_range(2);
        y1 = y1(idx);
        y2 = y2(idx);
    end

    fh = figure(); hold on; box on; grid on
    plot(y1,y2,'-k','LineWidth',2);
    pbaspect([1 1 1]);
    daspect([1 1 1]);
    if ~isempty(opt.title)
        title(opt.title)
    end

