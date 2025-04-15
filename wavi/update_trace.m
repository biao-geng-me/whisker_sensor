function qc = update_trace(fh,pl,far,sig,Fs,nsensor)

    % 
    [nsensor_y,nsensor_x]=square_layout(nsensor);
    % correction curve data
    al = -180:15:180;
    am = [-189.68 -175.47 -160.70 -140.22 -119.42 -98.98 -82.22 -65.84 -53.22 -40.49 -30.80 -19.82  -8.67   3.93  21.91  39.02  61.31  81.08 100.44 113.55 125.90 138.31 149.54 158.49 170.32];

    set(0,'CurrentFigure',fh);
    trace_length = 0.5; % seconds
    ahp = 0.1; % arrow head length percentage
    h = 0.01;
    npt = round(trace_length*Fs);

    xt = zeros(npt,1);
    yt = zeros(npt,1);

    for iy=1:nsensor_y
    for ix=1:nsensor_x
        isensor=(iy-1)*nsensor_y + ix;
        if (isensor>nsensor)
            break
        end

        y1 = sig(end-npt+1:end,isensor*2-1);
        y2 = sig(end-npt+1:end,isensor*2);

        L = norm([y1(end) y2(end)]);
        qm = atan2(y2(end),y1(end));
    
        qd = qm/pi*180;
    
        if qd > am(end)
            qd = qd - 360;
        end
    
        qc = interp1(am,al,qd);
        q = qc/180*pi;
        T = [cos(q) -sin(q); sin(q) cos(q)];
        % h = L*awp/2;
        L1 = L*(1-ahp);
        xar = [0 L1 L1 L L1 L1 0];
        yar = [-h -h -L*ahp/3 0 L*ahp/3 h h];
        
        co = T*[xar;yar];
        far(isensor).XData = co(1,:)+ix;
        far(isensor).YData = co(2,:)+iy;
    
        for j=1:npt
            xt(j) = y1(end-j+1);
            yt(j) = y2(end-j+1);
            
            qm = atan2(yt(j),xt(j));
        
            qd = qm/pi*180;
        
            if qd > am(end)
                qd = qd - 360;
            end
    
            q = interp1(am,al,qd)/180*pi;
    
            qdel = q-qm;
        
            T = [cos(qdel) -sin(qdel); sin(qdel) cos(qdel)];
            tmp = T*[xt(j); yt(j)];
            xt(j) = tmp(1)+ix;
            yt(j) = tmp(2)+iy;
        end
    
        
        pl(isensor).XData = xt;
        pl(isensor).YData = yt;
        drawnow
    end
    end


    



