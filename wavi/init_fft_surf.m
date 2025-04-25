function [fh,sh,bh,ah,ah2] = init_fft_surf(nch,nfreq,t_fft)
    % plot fft as a surface
    % return figure and contour map handles
    monp = get(groot,'MonitorPositions');
    if size(monp,1)>1 % show on second monitor,
        monp = monp(2,:);
        fig_pos = [monp(1), 49, monp(3)*0.5, monp(4)-80];
    else % show on first monitor reduced size
        monp = monp(1,:);
        fig_pos = [monp(1)+monp(3)*0.5, 49, monp(3)*0.4, monp(4) - 100];
    end

    fh = figure('OuterPosition',fig_pos);
    grid on;
    box on
    %

    [X,Y] = meshgrid(0:nch+1,0:nfreq-1);
    
    colormap jet
    ah = gca;
    ah.Position(2) = 0.38; % change vertical position
    ah.Position(4) = 0.55;
    
    sh = surface(X,Y,zeros(nfreq,nch+2),'FaceColor','interp','EdgeColor','none',...
        'FaceLighting','none','FaceAlpha',0.5);

    ylabel('Frequency (Hz)')
    xlim([0 nch+1])
    ylim([0 nfreq-1])
    xticks(1:nch)
    xticklabels([]);

    myy = yticks;
    ytlabel = cell(size(myy));
    for i=1:numel(myy)
        ytlabel{i} = sprintf('%d',myy(i)/t_fft); % 1/t_fft is the frequency resolution
    end
    yticklabels(ytlabel);

    % plot amplitudes at the bottom
    pos = ah.Position;
    pos(2) = 0.07;
    pos(4) = 0.25;

    ah2 = axes(fh,"Position",pos);
    bh = bar(rand(1,nch),'FaceColor','flat');
    bh.CData(1:2:end,:) = repmat([1 0 0],nch/2,1);

    xlim([0 nch+1])
    
    xticks(1:nch)

    xtlabel = cell(1,nch);
    for i=1:nch
        isensor = ceil(i/2);
        ich = mod(i,2);
        if ich == 0
            ich = 2;
        end
        xtlabel{i} = sprintf("%d-%d",isensor,ich);
    end
    xticklabels(xtlabel);
    xlabel('Channel')
    ylabel('mV')


    return



    





    