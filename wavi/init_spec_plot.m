function [fh,mh] = init_spec_plot(spec_data,Fs,nread)
    % initialize spectrogram plot
    % return figure and contour map handles

    monp = get(groot,'MonitorPositions');
    if size(monp,1)>1 % show on second monitor,
        monp = monp(2,:);
        fig_pos = [monp(1)+monp(3)*0.5, 49, monp(3)*0.5, monp(4)-80];
    else % show on first monitor reduced size
        monp = monp(1,:);
        fig_pos = [monp(1)+20, 49+ monp(4)*0.4, monp(3)*0.5, monp(4)*0.6 - 60];
    end

    fh = figure('OuterPosition',fig_pos);
    grid on;
    box on
    %
    mh = imagesc(spec_data);
    colormap jet
    set(mh,'alphadata',isfinite(spec_data));
%     mh.XData = (1:size(spec_data,2))*dt;
    ylabel('Frequency');
    yticks(0:Fs/2+1:Fs*20) % 20 is the number of maximum channels
    myy = yticks;
    ytlabel = cell(size(myy));
    for i=1:numel(myy)
        ytlabel{i} = sprintf('%d',round(Fs/2));
    end
    yticklabels(ytlabel);
    
    fps = round(Fs/nread);
    nsample = size(spec_data,2);
    myx = nsample:-fps*5:0;
    xticks(sort(myx))
    myx = xticks;
    xtlabel = cell(size(myx));
    for i=1:numel(myx)
        xtlabel{i} = sprintf('%4.1f',(myx(i)-nsample)/fps);
    end
    xticklabels(xtlabel);
    xlabel('Time (s)');
    % ylim([0,Fs/2*9]);





    