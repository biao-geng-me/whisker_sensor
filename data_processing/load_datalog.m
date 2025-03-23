function dat_table = load_datalog(pathname)

    dat_table = readtable(pathname, "FileType","fixedwidth");
