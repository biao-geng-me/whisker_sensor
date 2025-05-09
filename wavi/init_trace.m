function [fh,far,pl] = init_trace(Fs,sig,nsensor)
    % return two line handles, one for filled arrow, one for the trace

    [nsensor_y,nsensor_x]=square_layout(nsensor);

    monp = get(groot,'MonitorPositions');
    if size(monp,1)>1 % show on second monitor,
        monp = monp(2,:);
        npixel = round(min(monp(3)*0.8/nsensor_x,monp(4)*0.8/nsensor_y));
        fig_pos = [monp(1)+50, 49, npixel*nsensor_x, npixel*nsensor_y];
    else % show on first monitor reduced size
        monp = monp(1,:);
        npixel = round(min(monp(3)*0.6/nsensor_x,monp(4)*0.6/nsensor_y));
        fig_pos = [monp(1)+100, monp(2)+monp(4)*0.2, npixel*nsensor_x, npixel*nsensor_y];
    end

    fh = figure('Position',fig_pos);

    pbaspect([1 1 1]);
    daspect([1 1 1]);
    xlim([0 nsensor_x+1])
    ylim([0 nsensor_y+1])
    hold on
    
    %
    trace_length = 0.5; % seconds
    npt = round(trace_length*Fs);

    for iy=1:nsensor_y
    for ix=1:nsensor_x

        i=(iy-1)*nsensor_y + ix;
        if(i>nsensor)
            break
        end
        x = sig(1:npt,i*2-1)+ix;
        y = sig(1:npt,i*2)+iy;
        pl(i)=line(x,y,'Color',[1 1 1]*0.5,'linewidth',4);
        hold on

        L = norm([x(end),y(end)]);
    
        q = atan2(y(end),x(end));
        ahp = 0.2; % arrow head length percentage
    
        h = 0.01;
        L1 = L*(1-ahp);
        xar = [0 L1 L1 L L1 L1 0];
        yar = [-h -h -L*ahp 0 L*ahp h h];
        
        T = [cos(q) -sin(q); sin(q) cos(q)];
        co = T*[xar;yar];
        far(i) = fill(co(1,:),co(2,:),[0.8,0,0],'facealpha',0.9,'EdgeColor','none');

    end
    end
    % ar = line([0 x(end)],[0 y(end)],'color','k','linewidth',1);
    
    pbaspect([1 1 1]);
    daspect([1 1 1]);

    box on
    grid on

