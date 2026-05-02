% example script to call makeWhiskerMesh.m

clc;
addpath(fileparts(mfilename('fullpath')));

% Scale the hydraulic diameter to 1 mm, default is 0.533 mm per
% Liu et al. Bioinspiration and Biomimetics 2019, 14, doi:10.1088/1748-3190/ab34fe.
% k = 1/0.533;

k = 3;
nL = 7;
start_phase = 180; % undulation phase. 180 to start from saddle plane
nzl = 50;  % number of elements per wavelength
neq1 = 10; % number of elements along the narrow aspect
neq2 = neq1; % number of elements along the wide aspect, set to the same value for a smooth transition

[co,f] =makeWhiskerMesh("wavenumber", nL,...
                        "scale", k,...
                        "start_phase", start_phase,...
                        "nzl", nzl,...
                        "neq1", neq1,...
                        "neq2",neq2);

% Translate
% co = co + [15 15 0];

fname = sprintf('whisker_%dL_%3.1fX',nL,k);
stlwrite_([fname '.stl'],f,co);

% optionally write Tecplot format
write_S3_mesh('whisker.plt',co,f,["x","y","z"],fname);

