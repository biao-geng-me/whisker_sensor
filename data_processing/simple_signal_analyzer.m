% this is to test ideas for an interactive signal analyzer app
function simple_signal_analyzer(t,signal)
    % Generate test signal
    % fs = 1000;
    % t = 0:1/fs:2;
    % signal = chirp(t, 10, 2, 200);
    fs = 1/diff(t(1:2));
    % Create figure and axes
    fig = figure;
    ax1 = subplot(3,1,1);
    plot(t, signal);
    title('Time Domain');
    
    ax2 = subplot(3,1,2);
    [f, Pxx] = fast_fourier(signal,fs);
    h_spectrum = plot(f, Pxx);
    title('Spectrum');
    
    ax3 = subplot(3,1,3);
    [s, f_sg, t_sg] = spectrogram(signal, 256, 250, [], fs);
    h_spectrogram = imagesc(t_sg, f_sg, 10*log10(abs(s)));
    axis xy; colorbar;
    title('Spectrogram');
    
    % Link x-axes for pan/zoom
    linkaxes([ax1, ax3], 'x');
    
    % Add sliders
    uicontrol('Style', 'slider', 'Position', [20 20 120 20], ...
        'Min', 64, 'Max', 1024, 'Value', 256, ...
        'Tag', 'winLen', 'Callback', @updatePlots);
    
    uicontrol('Style', 'slider', 'Position', [20 50 120 20], ...
        'Min', 0, 'Max', 0.9, 'Value', 0.5, ...
        'Tag', 'overlap', 'Callback', @updatePlots);
    
    % Callback function
    function updatePlots(~, ~)
        winLen = round(get(findobj('Tag', 'winLen'), 'Value'));
        overlap = get(findobj('Tag', 'overlap'), 'Value');
        
        % Update spectrum

        set(h_spectrum, 'XData', f, 'YData', Pxx);
        
        % Update spectrogram
        [s, f_sg, t_sg] = spectrogram(signal, winLen, round(overlap*winLen), [], fs);
        set(h_spectrogram, 'CData', 10*log10(abs(s)), 'XData', t_sg, 'YData', f_sg);
        drawnow;
    end
end