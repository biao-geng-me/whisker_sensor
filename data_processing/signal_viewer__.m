
% check sensor signal using the plot_sensor_signal function
clear;clc;close all


% assemble data log
path = 'sensor_data_samples';
name = 'st_2025-04-09_1115_27.00_400RPM_20x40bar_test_edge_two_motor';
isensor = 1;

% settings
f_range = [0 44];   % for spectrogram, 
ol = 0.75;          % spectrogram overlap
t_fft = 2;

% plot
file = fullfile(path,name);
fh1 = plot_sensor_signal(file,isensor,...
                         title=[],...
                         t_fft=t_fft,...
                         f_range=f_range,...
                         clickable=true,...
                         log_scale=false,...
                         overlap=ol);

%% spectrum plot
%  use the sensor_signal_spectrum function to do fft of a selected signal segment
[ff,pp1,pp2,fh] = sensor_signal_spectrum(file,isensor,show_fft=true,t_range=[23 33]);

