function [y,ym] = remove_drift(y,Fs,T)
% remove_drift - remove drift from signal
%   T - moveing average time in seconds
%   Fs - sampling frequency
%   y - signal
%   ym - moving average
    y = y - y(1);
    ym = movmean(y,T*Fs);
    y = y - ym;