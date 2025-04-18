function [fh_line, fh_fft, pp_max] = datalog2image(path,name,opt)
    % plot and save sensor signal and fft
    arguments
        path
        name
        opt.show_plots = false
        opt.t_avg = 5
    end

    if opt.show_plots
        visible = "on";
    else
        visible = "off";
    end
    t_avg = opt.t_avg;

    file = fullfile(path,name);
    [dat_table,dt_str,tag] = load_datalog(file);
    dat = table2dat(dat_table);
    t = dat(:,1);
    v = dat(:,2:end) - dat(1,2:end); % sensor voltage, remove offset
    v2 = zeros(size(v)); % store filtered v
    Fs = (numel(t)-1)/(t(end)-t(1));
    nch = size(v,2);
    nsensor = nch/2;
    [nrow,ncol] = square_layout(nsensor);
    
    % line history figure
    fh_line = figure(Position=[0 0 1920 1080],Visible=visible);
    tiledlayout(fh_line,nrow,ncol,TileSpacing="tight");

    for irow=1:nrow
        for icol=1:ncol
            i = (irow-1)*ncol + icol;
            if(i>nsensor)
                break;
            end
            nexttile; hold on; box on; grid on;
            y1 = v(:,i*2-1);
            y2 = v(:,i*2);
    
            [y1a,y1m] = remove_drift(y1,Fs,t_avg);
            [y2a,y2m] = remove_drift(y2,Fs,t_avg);
        
            plot(t,y1,'-r',LineWidth=2);
            plot(t,y1m,'-k')
        
            plot(t,y2,'-b',LineWidth=2);
            plot(t,y2m,'--k')
            xlim(t([1 end]));
            title(sprintf('Sensor %d',i))
    
            v2(:,i*2-1) = y1a;
            v2(:,i*2) = y2a;
            if(icol==1 && irow==1)
                ylabel('Signal (mV)')
            end
        end
        if(irow==nrow)
            xlabel('Time (s)');
        end
    end

    % fft figure
    fh_fft = figure(Position=[0 0 1920 1080], Visible=visible);
    tiledlayout(fh_fft,nrow,ncol,TileSpacing="tight");
    
    pp_max = 0;
    for irow=1:nrow
        for icol=1:ncol
            i = (irow-1)*ncol + icol;
            if(i>nsensor)
                break;
            end
            nexttile(i); hold on; box on; grid on;
            [ff1,pp1] = fast_fourier(v2(:,i*2-1),Fs);
            [ff2,pp2] = fast_fourier(v2(:,i*2),Fs);
            
            pmax = max(max(pp1),max(pp2));
            if pmax > pp_max
                pp_max = pmax;
            end
    
            plot(ff1,pp1,'-r',LineWidth=2);
            plot(ff2,pp2,'-b')
            xlim([0 Fs/2]);
            title(sprintf('Sensor %d',i))

            if(icol==1 && irow==1)
                ylabel('Signal (mV)')
            end
        end
        if(irow==nrow)
            xlabel('Frequency (Hz)');
        end
    end

    % adjust ylim
    for i=1:nsensor
        nexttile(i);
        ylim([0 pp_max*1.25]);
    end

    %
    if strcmp(name(end-3:end),'.dat')
        name = name(1:end-4);
    end

    saveas(fh_fft,fullfile(path,['fft_' name '.png']));
    saveas(fh_line,fullfile(path,['his_' name '.png']));
    
    
    