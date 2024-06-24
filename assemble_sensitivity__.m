% bgeng 2024-04-25 assemble dipole flow test data
% check peak in fft, exclude data where there is no clear peak corresponding to the
% dipole frequency.

clear;clc;
datdir = 'ch2-x2-dipole-1';
savdir = '.';

% accelerometer data
ac_data_dir = 'C:\Users\bigeme\Working\SealWhisker\sensor\prototype_tests\vibration-data-2024-04-24';

f = 35; % dipole frequency
Fs = 88; % sensor sample frequency

t_fft = 30; % use 30 s for fft
A_fft = 6e-4; % fft plot y range
ich = 2; % channel index

pat = sprintf('st*%dHz_*p.dat',f);
flist = dir(fullfile(datdir,pat));

% get levels
nfile = numel(flist);
lvl = zeros(nfile,1);

for i=1:numel(flist)
    txtparts = split(flist(i).name,'_');
    lvl(i) = sscanf(txtparts{end},'%d');
end

lvl = unique(lvl);
nlvl = numel(lvl);

% adjust
close all;
ilvl = 1:nlvl;
show_fft = 0;
savname = sprintf('ch%d_x2_sensitivity_%dHz.dat',ich,f);

% 
fa = alias_frequency(f,Fs);  % aliased frequency
amp = zeros(nlvl,1);
vel = zeros(nlvl,1);
err = zeros(nlvl,1);

for i=ilvl
    pat = sprintf('st*%dHz_%dp*',f,lvl(i));
    flist = dir(fullfile(datdir,pat));
    ns = numel(flist);

    ampp = zeros(ns,1);
    
    for j=1:ns
        fname = fullfile(flist(j).folder,flist(j).name);
        dat = readtable(fname, "FileType","fixedwidth");
        y = dat{:,2+ich};
        y1 = y(end-Fs*t_fft:end);
        [ff,pp] = fast_fourier(y1,Fs);
        
        ind = fa-1<ff & ff< fa+1;
        ind_s = find(ind,1);
        [pmax,imax] = max(pp(ind));
        imax = imax + ind_s -1;
        ampp(j) = pmax;
        
        if (show_fft)
            figure;hold on
            plot(ff,pp)
            plot(ff(imax),pmax,'or');
            
            xlim([0 Fs/2]);
            ylim([0 A_fft]);
            ylabel('Amplitude (mV)')
            xlabel('Frequency (Hz)');
            title(sprintf('%d Hz %d%%', f, lvl(i)))
        end 
    end

    amp(i) = mean(ampp);
    err(i) = std(ampp);
end


%% calc tip flow velocity
file = dir(fullfile(ac_data_dir,sprintf('%dHz*',f)));
ac_data = load(fullfile(file.folder,file.name));
ac = interp1(ac_data(:,1),ac_data(:,2),lvl);

% calculate amplitude
A = ac*9.8/(2*pi*f)^2; % amplitude
a = 0.0254/2; % sphere radius

d = 0.024; % distance from whisker tip to sphere center
vtip = 2*pi()*f*A*a.^3*d^-3/2*1000; % mm/s

figure; errorbar(vtip,amp,err,'-+k')
xlim([0 max(vtip)*1.1]);
ylim([0 max(amp)*1.1]);
xlabel('whisker tip velocity (mm/s)');
ylabel('Signal (mV)');

%%
fid = fopen(fullfile(savdir,savname),'w');
fprintf(fid,'%3d %12.5e %12.5e %12.5e\n',[lvl vtip amp err]');
fclose(fid);



