% reconnect serial port
clear; clc; fclose all; close all

fprintf('setting up arduino\n')
s=serialport("COM4",2000000);
fopen(s);


%% loop measurement
close all
ay = load('gong.mat'); % sound notification

outpath = 'test';
nrepeat = 1;
tmax = 4000;

tag = 'test';

Fs = 80;
Afft = 0;
nread = 8;

for i=1:nrepeat
    wavi_sampling(s,Afft,Fs,nread,'tag',tag,'tmax',tmax,'outpath',outpath,'showtrace',true,...
                  'nsensor',9,'showfft',true,'showspec',false,'t_fft',1,'scale',1);
end

fprintf("%s all done\n",tag);
sound(ay.y,ay.Fs)

