% convert sensor data (.dat) to Tecplot format (.plt)

function sensor_dat_to_plt(pathname,options)
    arguments
        pathname (1,:) char
        options.remove_offset (1,1) logical = true
    end
    [dat_table,dt_str,tag] = load_datalog(pathname);
    if isempty(dat_table)
        fprintf('no data found in %s\n',pathname);
        return
    end

    dat = table2dat(dat_table);
    nch = size(dat_table,2)-2;
    ntime = size(dat,1);

    if options.remove_offset
        for i=2:nch+1
            dat(:,i) = dat(:,i)-dat(1,i);
        end
    end

    out_pathname = [pathname(1:end-4), '.plt'];
    fid = fopen(out_pathname, 'w');
    fprintf(fid,'TITLE = "Sensor Data"\n');
    fprintf(fid,'VARIABLES = "Time (s)"');
    for i=1:nch
        fprintf(fid,' "Ch%d (mV)"',i);
    end
    fprintf(fid,'\n');
    fprintf(fid,'ZONE T="%s %s"\n',dt_str,tag);
    fprintf(fid,'I=%d, J=1\n',ntime);
    fprintf(fid,'DATAPACKING=POINT\n');
    fprintf(fid,'DT=(%s)\n',repmat('SINGLE ',1,nch));
    fprintf(fid,['%9.3f ',repmat('%12.6f',1,nch) '\n'],dat');

    fclose(fid);
end