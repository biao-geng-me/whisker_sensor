
% check sensor signal using the plot_sensor_signal function
clear;clc;close all


% assemble data log
% example data
% path = 'sensor_data_samples';
% name = 'st_2025-04-09_1115_27.00_400RPM_20x40bar_test_edge_two_motor';

data_path = fullfile(getenv('USERPROFILE'),'wavi_data');
name = 'st_2025-11-06_1044_31.17_test_path0-v1=0.30_path3-v2=0.30_delay=5.0.dat';

isensor = 9;

% settings
f_range = [0 40];   % for spectrogram, 
ol = 0.75;          % spectrogram overlap
t_fft = 2;

% plot
file = fullfile(data_path,name);
fh1 = plot_sensor_signal(file,isensor,...
                         title=[],...
                         t_fft=t_fft,...
                         f_range=f_range,...
                         clickable=true,...
                         log_scale=false,...
                         overlap=ol,...
                         line_tags={'Lift','Drag'});

%% spectrum plot
%  use the sensor_signal_spectrum function to do fft of a selected signal segment
[ff,pp1,pp2,fh] = sensor_signal_spectrum(file,isensor,show_fft=true,t_range=[2 35]);

%%
files = dir(fullfile(path,'st*.dat'));
for i=1:numel(files)
    datalog2image(path,files(i).name,t_avg=10);
    disp(i);
end

