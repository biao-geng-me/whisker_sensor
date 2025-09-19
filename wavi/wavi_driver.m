% reconnect serial port
try
    delete(s);
catch
end
%%
clear; clc; fclose all; close all

fprintf('Setting up arduino\n')
s=serialport("COM13",2000000); % change COM number accordingly
fopen(s);


%% loop measurement
close all
ay = load('gong.mat'); % sound notification

outpath = 'test';
nrepeat = 1;
tmax = 4000;
tag = 'test';

Fs = 10;
Afft = 0;
nread = 2;
nsensor = 1;

for i=1:nrepeat
    wavi_sampling(s,Afft,Fs,nread,'tag',tag,'tmax',tmax,'outpath',outpath,'nsensor',nsensor, ...
        'showtrace',false,...
        'showfft',true, ...
        'show_spectrogram',true, ...
        'show_spectrum', false, ...
        't_fft',1, ...
        'scale',1);
end

fprintf("%s all done\n",tag);
sound(ay.y,ay.Fs)

