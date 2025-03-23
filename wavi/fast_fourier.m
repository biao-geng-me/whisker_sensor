function [f,P1] = fast_fourier(ys,Fs)
% fourier transform after detrend with real amplitudes
% [f,P1] = fast_fourier(ys,Fs)
% ys - samples
% Fs - sample rate, best in Hz
% returns
% f  - frequency
% P1 - amplitude

L = length(ys);
% even number of samples is preferred
if (mod(L,2)==1)
    L=L-1;
    ys = ys(1:L);
end

Y = fft(detrend(ys));
P2 = abs(Y/L); % P2 is symmetric from index 2 to end

np = ceil((L+1)/2);
P1 = P2(1:np);

P1(2:end-1) = 2*P1(2:end-1); % one side amplitude

f = Fs*(0:(L/2))/L;

