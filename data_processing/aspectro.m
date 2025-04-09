function [t,f,s]=aspectro(ys,Fs,opt)

    % amplitude based spectrogram
    % 
    arguments
        ys              % signal sample array
        Fs              % signal sampling frequency
        opt.window_size=0
        opt.overlap=0.5
    end

    N = numel(ys);
    if opt.window_size == 0
        L = max(round(N/50),32);
    else
        L = round(opt.window_size);
    end

    % even number of samples is preferred
    if (mod(L,2)==1)
        L=L-1;
    end

    f = Fs*(0:(L/2))/L;

    step = round(L*(1-opt.overlap));

    ts = (1:numel(ys))/Fs;
    tind = L/2:step:(numel(ys)-L/2);
    t = ts(tind);
    s = nan(numel(f),numel(t));
    for i=1:numel(t)
        j = (i-1)*step + 1;
        y = ys(j:(j+L)-1);
        [~,pp] = fast_fourier(y,Fs);

        s(:,i) = pp;
    end

    return
