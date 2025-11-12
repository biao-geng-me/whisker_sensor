function [dat_table,dt_str,tag] = load_datalog(pathname)

    if ~exist(pathname,'file') && ~strcmp(pathname(end-3:end),'.dat')
        pathname = [pathname '.dat'];
    end
    try
        dat_table = readtable(pathname, "FileType","fixedwidth");
    catch ME
        error('load_datalog:readtable','Failed to readtable from %s', filepath);
    end

    % extract meta data from datalog filename
    % convention is st_date_time_tag_fields
    [filepath,name,ext] = fileparts(pathname);
    try
        parts=split(name,'_');
        dt_str = [parts{2} ' ' parts{3}];
        tag = join(parts(5:end),' ');
        tag = tag{1};
    catch
        dt_str = '';
        tag = '';
    end
