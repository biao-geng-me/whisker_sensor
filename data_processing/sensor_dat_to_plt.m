% convert sensor data (.dat) to Tecplot format (.plt)

function sensor_dat_to_plt(pathname)

    dat_table = load_datalog(pathname);
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

    out_pathname = replace(pathname,'.dat','.plt');
    fid = fopen(out_pathname, 'w');
    fprintf(fid,'TITLE = "Sensor Data"\n');
    fprintf(fid,'VARIABLES = "time" "Voltage (mV)"\n');
    for i=1:nch
        fprintf(fid,'ZONE T="Channel %d"\n',i);
        fprintf(fid,'I=%d\n',ntime);
        fprintf(fid,'J=1\n');
        fprintf(fid,'DATAPACKING=POINT\n');
        fprintf(fid,'DT=(SINGLE SINGLE)\n');
        fprintf(fid,'%9.3f %12.6f\n',[dat(:,1), dat(:,i+1)]');
    end

    fclose(fid);
end