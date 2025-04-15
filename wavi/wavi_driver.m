% reconnect serial port
clear; clc; fclose all; close all

fprintf('Setting up arduino\n')
s=serialport("COM7",2000000); % change COM number accordingly
fopen(s);


%% loop measurement
close all
ay = load('gong.mat'); % sound notification

outpath = 'test';
nrepeat = 1;
tmax = 4000;
tag = 'test';

Fs = 12;
Afft = 1e-4;
nread = 2;
nsensor = 3;

for i=1:nrepeat
    wavi_sampling(s,Afft,Fs,nread,'tag',tag,'tmax',tmax,'outpath',outpath,'showtrace',true,...
                  'nsensor',nsensor,'showfft',true,'show_spectrogram',true,'t_fft',1,'scale',1);
end

fprintf("%s all done\n",tag);
sound(ay.y,ay.Fs)

