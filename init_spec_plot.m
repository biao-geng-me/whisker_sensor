function [fh,mh] = init_spec_plot(fft_map,Fs,nread)

    % return figure and contour map handles

    fh = figure('Position',[1920+960, 240, 640, 480]);
    grid on;
    box on
    %
    mh = imagesc(fft_map);
    colormap jet
    set(mh,'alphadata',isfinite(fft_map));
%     mh.XData = (1:size(fft_map,2))*dt;
    ylabel('Frequency');
    yticks(0:Fs/2+1:Fs*20)
    myy = yticks;
    ytlabel = cell(size(myy));
    for i=1:numel(myy)
        ytlabel{i} = sprintf('%d',round(Fs/2));
    end
    yticklabels(ytlabel);
    
    fps = round(Fs/nread);
    nsample = size(fft_map,2);
    myx = nsample:-fps*5:0;
    xticks(sort(myx))
    myx = xticks;
    xtlabel = cell(size(myx));
    for i=1:numel(myx)
        xtlabel{i} = sprintf('%4.1f',(myx(i)-nsample)/fps);
    end
    xticklabels(xtlabel);





    