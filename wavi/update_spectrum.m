function update_spectrum(lines,fft_map,A_fft)
    % update spectrum line plot
    
    pmax0 = max(fft_map(:));
    if A_fft > 0
        pmax = A_fft;
    else
        if pmax0 < 1e-3
            pmax=1e-3;
        elseif pmax0 < 1e-2
            pmax = 1e-2;
        elseif pmax0 < 1e-1
            pmax = 1e-1;
        else
            pmax = 1;
        end
    end

    for i=1:numel(lines)
        lines(i).YData = fft_map(:,i)/pmax + ceil(i/2)-1;
    end 

    myy = yticks;
    ytlabel = cell(size(myy));
    for i=1:numel(myy)
        ytlabel{i} = sprintf('%g',pmax);
    end
    yticklabels(ytlabel);
    title(sprintf('Max peak: %7.5f',pmax0))
    grid on;
    drawnow
        


