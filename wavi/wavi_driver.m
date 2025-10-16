% WAVI_DRIVER Script to run wavi sampling
% reconnect serial port
try
    delete(s);
catch
end
%%
clear; clc; fclose all; close all

fprintf('Setting up arduino\n')
s=serialport("COM18",2000000); % change COM number accordingly
% fopen(s);


%% loop measurement
close all
ay = load('gong.mat'); % sound notification

outpath = 'test';
nrepeat = 1;
tmax = 4000;
tag = 'test';

Fs = 80;
Afft = 0;
nread = 6;
nsensor = 9;
ch_map = load('channel_map.txt');

for i=1:nrepeat
    wavi_sampling(s,Afft,Fs,nread,'tag',tag,'tmax',tmax,'outpath',outpath,'nsensor',nsensor, ...
        'showtrace',false,...
        'showfft',true, ...
        'show_spectrogram',false, ...
        'show_spectrum', false, ...
        't_fft',1, ...
        'scale',1, ...
        'ch_map',ch_map);
end

fprintf("%s all done\n",tag);
sound(ay.y,ay.Fs)

