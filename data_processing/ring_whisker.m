% Ring-whisker interaction data analysis
% Biao Geng
% Created 2025-Dec-18
% Figures saved manually for each condition.

cfg = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), 'ring_whisker_1.json')));
data_path = cfg.data_path;

files = dir(fullfile(data_path,'st_*_90d_No_W3_0.5R*.dat'));

figure(Position=[1920+100 100 720 720]);
tiledlayout(3,2,"TileSpacing","tight");
title_txts = {'3V', '3L','2V','2L','1V','1L'};
for ifile = 1:numel(files)
    [dat_table,dt_str,tag] = load_datalog(fullfile(data_path,files(ifile).name));
    
    dat = table2dat(dat_table);
    t = dat(:,1);
    
    v = dat(:,2:end) - dat(1,2:end); % sensor voltage, remove offset
    v = flip(v,2);
    Fs = round(numel(t)/(t(end)-t(1)));
    nch = size(v,2);
    nsensors = nch/2;
    
    
    axh = gobjects(nch, 1); % Preallocate array for axes handles
    
    for ich = 1:6
        axh(ich) = nexttile(ich);
        hold on; 
        yy = v(:,ich);
        % Define the time thresholds
        t_thresholds = [1, 9];
        
        % Find indices for the segments of interest
        idx1 = t < t_thresholds(1) | t > t_thresholds(2) ;
        
        % Perform linear fit for t < 1 | t>9
        p1 = polyfit(t(idx1), yy(idx1), 1);
        fitted_line1 = polyval(p1, t);
        
        ys = smoothdata(yy-fitted_line1,1,"movmean",round(Fs/5));
        
        
        % plot(t,v(:,ich));
        % plot(t,yy - fitted_line1);
        % plot(t,ys,'-k',LineWidth=3);
        
        plot(t,ys);
    end
end

for i=1:nch
    axes(axh(i));
    grid on;
    xticks(1:10);
    title(title_txts{i});
    yline(axh(i),0);
    xlim([0 10])
    ylim([-0.0028 0.0028])
    if i==5 || i==6
        xlabel('Time (s)')
    else
        set(axh(i), 'XTickLabel', []);
    end

    if mod(i,2) == 1
        ylabel('Signal (mV)')
    else
        set(axh(i), 'YTickLabel', []);
    end
end


