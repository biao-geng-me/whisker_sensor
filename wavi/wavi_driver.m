% WAVI_DRIVER Script to run wavi sampling
% reconnect serial port
try
    delete(s);
catch
end
%%
clear; clc; fclose all; close all

fprintf('Setting up arduino\n')
nsensor = 3;
s=serialport("COM13",2000000); % change COM number accordingly
pause(1.5) % wait for Arduino reset
% send sensor count to Arduino before starting acquisition
try
    cmd = sprintf('N=%d', nsensor);
    writeline(s, cmd);
    pause(0.1);
catch
end


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

% ch_map = load('channel_map.txt');
ch_map = 1:nsensor*2;

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

