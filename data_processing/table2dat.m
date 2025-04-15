function dat = table2dat(dat_table)
    % convert sensor data table to array format
    % datetime is converted to seconds

    dtime = datetime([datestr(dat_table{:,1}) datestr(dat_table{:,2},' HH:MM:SS.FFF')]);
    ttime = dtime - dtime(1);
    t = seconds(ttime); % time in seconds
    nch = size(dat_table,2)-2;
    ntime = numel(t);

    dat = zeros(ntime,nch+1);
    dat(:,1) = t;
    for i=1:nch
        dat(:,i+1) = dat_table{:,i+2};
    end

    % remove nan entries
    dat = dat(~any(isnan(dat),2),:);
end
 