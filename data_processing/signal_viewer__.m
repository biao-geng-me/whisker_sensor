
% check sensor signal using the plot_sensor_signal function
clear;clc;close all


% assemble data log
% path = 'sensor_data_samples';
% name = 'st_2025-04-09_1115_27.00_400RPM_20x40bar_test_edge_two_motor';    

path = fullfile(getenv('USERPROFILE'),'OneDrive - rit.edu','work','wavi_data');
name = 'st_2025-10-31_1529_40.91_water-D6cm-17cm_path2-v1=0.30_path3-v2=0.20_delay=3.0';
isensor = 5;

% settings
f_range = [0 40];   % for spectrogram, 
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

