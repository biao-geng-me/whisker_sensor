% bgeng 2024-04-25 
% assemble dipole flow test data for 1 dipole frequency
% check peak in fft, exclude data where there is no clear peak corresponding to the
% dipole frequency

clear;clc;
%----------- user inputs ----------------

% dipole flow configuration
a = 0.0254/2; % meter, sphere radius
d = 0.024; % meter, distance from whisker tip to sphere center
f = 15; % dipole frequency
Fs = 88; % sampling frequency. The HX711 board nominal sampling rate is 80Hz,
         % but could have more than 10% variance.

% data file
datdir = './sensor_data_samples';
savdir = fullfile(datdir,'..');
dat_ch = 1; % channel index in data log to use
sav_ch = 1; % normally the same as dat_ch unless swapped during data collection
savname = sprintf('ch%d_x3_sensitivity_%dHz.dat',sav_ch,f);

% sensor data log filename pattern
% use format specifier liker %d to represent numbers
% use * to match any string
filename_fmtstr = 'st_*%dHz_*'; % used to match all data logs for 1 frequency
level_fmtstr = 'st_*%dHz_%dp*'; % used to match an amplitude level
filename_pattern = sprintf(filename_fmtstr,f);

% processing settings
t_fft = 30; % seconds, time length for fft
A_fft = 0; % fft plot y range, 0 to use automatic range
check_lvl = []; % select a subset to check fft spectrum
%------------ end of user inputs ------------

flist = dir(fullfile(datdir,filename_pattern));
if isempty(flist)
    error('no data file found using pattern "%s" in %s\n',filename_pattern,datdir);
end

% get levels
nfile = numel(flist);
lvl = zeros(nfile,1);

for i=1:numel(flist)
    txtparts = split(flist(i).name,'_');
    lvl(i) = sscanf(txtparts{end},'%d');
end

lvl = unique(lvl);
nlvl = numel(lvl);

% 
close all;
% 
fa = alias_frequency(f,Fs);  % aliased frequency
amp = zeros(nlvl,1);
vel = zeros(nlvl,1);
err = zeros(nlvl,1);

for i=1:nlvl
    pat = sprintf(level_fmtstr,f,lvl(i));
    flist = dir(fullfile(datdir,pat));

    if isempty(flist)
        error('no data file found using pattern %s in %s\n',pat,datdir);
    end
    ns = numel(flist);
    ampp = zeros(ns,1);
    for j=1:ns
        fname = fullfile(flist(j).folder,flist(j).name);
        dat = load_datalog(fname);
        y = dat{:,2+dat_ch};
        y1 = y(end-Fs*t_fft:end);
        [ff,pp] = fast_fourier(y1,Fs);
        
        ind = fa-1<ff & ff< fa+1;
        ind_s = find(ind,1);
        [pmax,imax] = max(pp(ind));
        imax = imax + ind_s -1;
        ampp(j) = pmax;
        
        if ismember(i,check_lvl)
            figure;hold on
            plot(ff,pp)
            plot(ff(imax),pmax,'or');
            
            xlim([0 Fs/2]);
            if exist('A_fft','var') && (A_fft>0)
                ylim([0 A_fft]);
            end
            ylabel('Amplitude (mV)')
            xlabel('Frequency (Hz)');
            title(sprintf('%d Hz %d%%', f, lvl(i)))
        end 
    end

    amp(i) = mean(ampp);
    err(i) = std(ampp);
end

%% calc tip flow velocity
% accelerometer data
ac_data_dir = './acceleration_data';
file = dir(fullfile(ac_data_dir,sprintf('%dHz*',f)));
ac_data = load(fullfile(file.folder,file.name));
ac = interp1(ac_data(:,1),ac_data(:,2),lvl);

% calculate amplitude
A = ac*9.8/(2*pi*f)^2; % amplitude
vtip = 2*pi()*f*A*a.^3*d^-3/2*1000; % mm/s

figure; errorbar(vtip,amp,err,'-+k')
xlim([0 max(vtip)*1.1]);
ylim([0 max(amp)*1.1]);
xlabel('whisker tip velocity (mm/s)');
ylabel('signal (mV)');

%%
if ~exist(savdir,"dir")
    mkdir(savdir);
end
fid = fopen(fullfile(savdir,savname),'w');
fprintf(fid,'%3d %12.5e %12.5e %12.5e\n',[lvl vtip amp err]');
fclose(fid);
