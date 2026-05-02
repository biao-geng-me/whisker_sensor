function write_S3_mesh(pathname, co, f, varnames, zonename)
%WRITE_S3_MESH Write a 3D triangular surface mesh in Tecplot ASCII format.
%   write_S3_mesh(pathname, co, f)
%   write_S3_mesh(pathname, co, f, varnames)
%   write_S3_mesh(pathname, co, f, varnames, zonename)
%
% Inputs
%   pathname : output Tecplot file path (typically .plt or .dat)
%   co       : node coordinates, N x 3 numeric array
%   f        : triangle connectivity, E x 3 positive integer array (1-based)
%   varnames : 1 x 3 variable names (default: ["x","y","z"])
%   zonename : zone title string (default: "surface")

    arguments
        pathname {mustBeTextScalar}
        co (:,3) double
        f (:,3) double
        varnames (1,3) string = ["x","y","z"]
        zonename string = "surface"
    end

    if isempty(co) || isempty(f)
        error('write_S3_mesh:EmptyInput', 'co and f must be non-empty.');
    end

    if any(f(:) < 1) || any(f(:) ~= round(f(:)))
        error('write_S3_mesh:InvalidConnectivity', ...
              'f must contain 1-based positive integer node indices.');
    end

    if any(f(:) > size(co,1))
        error('write_S3_mesh:ConnectivityOutOfRange', ...
              'Connectivity contains node indices outside co.');
    end

    pathname = char(pathname);
    zonename = char(zonename);
    varnames = cellstr(varnames);

    fid = fopen(pathname, 'w');
    if fid < 0
        error('write_S3_mesh:OpenFailed', 'Could not open %s for writing.', pathname);
    end

    c = onCleanup(@() fclose(fid));

    fprintf(fid, 'TITLE = "S3 surface mesh"\n');
    fprintf(fid, 'VARIABLES = "%s" "%s" "%s"\n', varnames{1}, varnames{2}, varnames{3});
    fprintf(fid, 'ZONE T="%s", N=%d, E=%d, ZONETYPE=FETRIANGLE, DATAPACKING=POINT\n', ...
            zonename, size(co,1), size(f,1));

    fprintf(fid, '%.9g %.9g %.9g\n', co.');
    fprintf(fid, '%d %d %d\n', f.');
end
