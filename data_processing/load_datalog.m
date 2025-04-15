function [dat_table,dt_str,tag] = load_datalog(pathname)

    dat_table = readtable(pathname, "FileType","fixedwidth");

    % extract meta data from datalog filename
    [filepath,name,ext] = fileparts(pathname);

    parts=split(name,'_');
    dt_str = [parts{2} ' ' parts{3}];
    tag = join(parts(5:end),' ');
    tag = tag{1};
