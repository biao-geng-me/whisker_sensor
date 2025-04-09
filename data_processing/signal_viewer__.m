
% check sensor signal using the plot_sensor_signal function

clear;clc;close all

% -------------------
% assemble data logs
% -------------------
path = 'sensor_data_samples';
pattern = sprintf("st_*_400RPM_*.dat");
files = dir(fullfile(path,pattern));

% settings
log_scale = false;
f_range = [0 44];   % for spectrogram, 
ol = 0.75;          % spectrogram overlap
t_fft = 2;
fontsize = 12;

% -------------------
%  plot
%  -------------------
for k = 1:numel(files)
    for isensor = 1:1

        % use regexp to match parameters in filename
        % list of patterns to match, e.g. 'aoa_(\d+).dat', the match between () is returned.
        params = get_params(files(k).name,{'_test_(\S+)_two'}); 
        tag = sprintf ("400 RPM, two motor, %s, sensor %d",params{1},isensor);
        file = fullfile(files(k).folder,files(k).name);
        fh(k) = plot_sensor_signal(file,isensor,title=tag,f_range=f_range);
        saveas(fh(k),sprintf('%s_%d',params{1},isensor));
    end
end

