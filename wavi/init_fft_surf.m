function [fh,sh,bh,ah,ah2] = init_fft_surf(nch,nfreq,t_fft)

    % return figure and contour map handles

    fh = figure('Position',[960+1920, 60, 960, 920]);
    grid on;
    box on
    %

    [X,Y] = meshgrid(0:nch+1,0:nfreq-1);
    
    colormap jet
    ah = gca;
    ah.Position(2) = 0.38;
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
        ytlabel{i} = sprintf('%d',myy(i)/t_fft);
    end
    yticklabels(ytlabel);

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



    





    