function fa = alias_frequency(f,Fs)

    % f input signal frequency
    % Fs siganl sampling rate

    Fm = Fs/2;
    n = floor(f/Fm);

    oe = mod(n,2);

    if oe == 1
        fa = Fm - mod(f,Fm);
    else
        fa = mod(f,Fm);
    end