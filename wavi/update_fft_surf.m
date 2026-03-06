function update_fft_surf(fh,sh,bh,fft_map_3d,fft_ax1,fft_ax2,ah,options)
   % h - surface plot handle
   % bh - bar plot handle
   arguments
       fh
       sh
       bh
       fft_map_3d
       fft_ax1
       fft_ax2
       ah = []
       options.n_avg = 40;
   end
   
   fft_map = fft_map_3d(:,:,end);
    set(0,'CurrentFigure',fh);
    fh.CurrentAxes=fft_ax1;
    sh.ZData(:,2:end-1) = fft_map;
    sh.CData = sh.ZData;

    amax = max(fft_map(:));
    acrit = 4e-4;
    if isnan(amax) || amax <acrit
        clim([0 acrit*3]);
        zlim([0 acrit*3]);
    else
        clim([0 amax]);
        zlim([0 amax]);
    end
    
    fh.CurrentAxes=fft_ax2;
    bh.YData = max(fft_map);
    if ~isempty(ah)
        max_over_freq = squeeze(max(fft_map_3d,[],1));
        ah.YData = mean(max_over_freq(:,end-options.n_avg+1:end),2);
    end
    nch = size(fft_map,2);
    xticks(1:nch);
    if ~isnan(amax)
        ylim([0 (ceil(amax/acrit)+1)*acrit]);
    end
    grid on;
    title(sprintf("%7.5f",amax))
        


