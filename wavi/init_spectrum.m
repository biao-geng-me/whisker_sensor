function [fh,lines] = init_spectrum(nsensor,nfreq,t_fft)
    % 
    % return line handles of spectrum plots

    monp = get(groot,'MonitorPositions');
    monp = monp(1,:);
    fig_pos = [monp(1)+monp(3)*0.6, 81, monp(3)*0.4, monp(4) - 120];

    fh = figure('OuterPosition',fig_pos);
    box on;
    %
    
    ff = (0:nfreq-1)*1/t_fft;
    y = zeros(size(ff));
    for i=1:nsensor
        lines(i*2-1)=line(ff,y+i-1,'Color','r','linewidth',3);
        lines(i*2) = line(ff,y+i-1,'Color','b','linewidth',1);
    end
    xlabel('Frequency (Hz)');
    xlim([0 ff(end)]);
    ylim([0 nsensor])
    
    yticks(1:nsensor);
    ylabel('Amplitude (mV)')
    
    




