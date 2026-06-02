% Ring-whisker interaction data analysis
% Biao Geng
% Created 2026-May-29
% Figures saved manually for each condition.
% Note: for the new set of data with 3x long whiskers, the placing of the
% whiskers was rotated by 180° compared to previous, now sensor 1 sits on top,
% sensor 3 at the bottom

cfg = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), 'ring_whisker_2.json')));
data_path = cfg.data_path;

patterns = {'3.8cm.dat', '2.9cm.dat', '2.5cm.dat'};
labels   = {'3.8cm', '2.9cm', '2.5cm'};

title_txts = {'1L', '1V','2L','2V','3L','1V'};

% Alignment parameters (shared across sets)
align_ch  = [3 5];     % channels used to compute the alignment shift
t_thresholds = [2, 7]; % time window to exclude in polynomial trend fit
xc_window = [2 7];     % time window in which to evaluate xcorr
max_shift = 0.2;       % cap on |shift| in seconds, to avoid spurious peaks
smoothing_gain = 1.02;
align_sets = false;    % if true, xcorr-align each set's mean to set 1 in the comparison figure

% Per-set means stashed for the comparison figure
set_mean  = cell(numel(patterns),1);
set_ref_t = cell(numel(patterns),1);

for ipat = 1:numel(patterns)
    files = dir(fullfile(data_path, ['st_*' patterns{ipat}]));

    figure(Position=[1920+100+40*(ipat-1) 100+40*(ipat-1) 720 720]);
    tiledlayout(3,2,"TileSpacing","tight");
    sgtitle(labels{ipat}, 'Interpreter', 'none');

    % Reference signals (set from the first file in the set)
    ref_t    = [];
    ref_sigs = [];      % columns: ch3, ch5 of the first file

    axh = gobjects(6, 1);
    sum_aligned = [];   % running sum of aligned signals on ref_t grid (6 channels)
    n_aligned = 0;
    for ifile = 1:numel(files)
        [dat_table,dt_str,tag] = load_datalog(fullfile(data_path,files(ifile).name));

        dat = table2dat(dat_table);
        dat(dat(:,1)>50,:) = [];
        t = dat(:,1);

        v = dat(:,2:end) - dat(1,2:end); % sensor voltage, remove offset
        % v = flip(v,2);
        Fs = round(numel(t)/(t(end)-t(1)));
        nch = size(v,2);
        nsensors = nch/2;

        % --- Detrend + smooth all channels first, so we can xcorr before plotting
        ys_all = zeros(numel(t), nch);

        idx1 = t < t_thresholds(1) | t > t_thresholds(2);
        for ich = 1:nch
            yy = v(:,ich);
            p1 = polyfit(t(idx1), yy(idx1), 6);
            fitted_line1 = polyval(p1, t);
            yd = yy - fitted_line1;
            ys_all(:,ich) = smoothdata(yd,1,"movmean",round(Fs/5))*smoothing_gain;
        end

        % --- Compute alignment shift via cross-correlation on ch3 and ch5
        if ifile == 1
            ref_t = t;
            ref_sigs = ys_all(:, align_ch);
            t_shift = 0;
        else
            % Interpolate current signals onto the reference time grid
            cur_on_ref = interp1(t, ys_all(:, align_ch), ref_t, 'linear', 0);
            win = ref_t >= xc_window(1) & ref_t <= xc_window(2);
            dt_ref = mean(diff(ref_t));
            maxlag = round(max_shift / dt_ref);
            % Sum xcorr of the two alignment channels; peak gives best joint shift
            c_total = zeros(2*maxlag + 1, 1);
            for k = 1:numel(align_ch)
                [c, lags] = xcorr(cur_on_ref(win, k), ref_sigs(win, k), maxlag);
                c_total = c_total + c;
            end
            [~, imax] = max(c_total);
            lag_samples = lags(imax);
            % Positive lag => current is delayed vs reference; shift t left to align
            t_shift = lag_samples * dt_ref;
            fprintf('[%s] file %d: alignment shift = %+.3f s\n', ...
                labels{ipat}, ifile, t_shift);
        end

        % --- Plot all channels with the alignment shift applied
        for ich = 1:6
            axh(ich) = nexttile(ich);
            hold on;
            plot(t - t_shift, ys_all(:,ich),"LineWidth",3);
        end

        % --- Accumulate aligned signals on the reference time grid for averaging
        aligned_on_ref = interp1(t - t_shift, ys_all, ref_t, 'linear', 0);
        if isempty(sum_aligned)
            sum_aligned = zeros(size(aligned_on_ref));
        end
        sum_aligned = sum_aligned + aligned_on_ref;
        n_aligned = n_aligned + 1;
    end

    % --- Overlay the mean across all repeats as a thin black line
    mean_aligned = sum_aligned / n_aligned;
    for ich = 1:6
        axes(axh(ich));
        plot(ref_t, mean_aligned(:,ich), 'k', 'LineWidth', 2);
    end

    % --- Cosmetic axis formatting for this set's figure
    for i = 1:6
        axes(axh(i));
        grid on;
        xticks(1:10);
        title(title_txts{i});
        yline(axh(i),0);
        xlim([0 10])
        ylim([-0.009 0.009])
        if i==5 || i==6
            xlabel('Time (s)')
        else
            set(axh(i), 'XTickLabel', []);
        end

        if mod(i,2) == 1
            ylabel('Signal (mV)')
        else
            set(axh(i), 'YTickLabel', []);
        end
    end

    % Stash this set's mean for the comparison figure
    set_mean{ipat}  = mean_aligned;
    set_ref_t{ipat} = ref_t;
end

% --- Comparison figure: means from each set overlaid in the same layout
figure(Position=[1920+100+40*numel(patterns) 100+40*numel(patterns) 720 720]);
tiledlayout(3,2,"TileSpacing","tight");
sgtitle('Comparison of set means');
% --- xcorr-align each set's mean to set 1 (using ch3 and ch5), if enabled
set_t_shift = zeros(numel(patterns), 1);
if align_sets
    ref_t1  = set_ref_t{1};
    ref_m1  = set_mean{1}(:, align_ch);
    dt_ref1 = mean(diff(ref_t1));
    maxlag1 = round(max_shift / dt_ref1);
    win1    = ref_t1 >= xc_window(1) & ref_t1 <= xc_window(2);
    for ipat = 2:numel(patterns)
        cur_on_ref = interp1(set_ref_t{ipat}, set_mean{ipat}(:, align_ch), ...
                             ref_t1, 'linear', 0);
        c_total = zeros(2*maxlag1 + 1, 1);
        for k = 1:numel(align_ch)
            [c, lags1] = xcorr(cur_on_ref(win1, k), ref_m1(win1, k), maxlag1);
            c_total = c_total + c;
        end
        [~, imax] = max(c_total);
        set_t_shift(ipat) = lags1(imax) * dt_ref1;
        fprintf('comparison: set %d (%s) shift = %+.3f s\n', ...
                ipat, labels{ipat}, set_t_shift(ipat));
    end
end

cmp_axh = gobjects(6, 1);
cmp_lines = gobjects(numel(patterns), 1);  % handles captured from subplot 2 for the legend
for ich = 1:6
    cmp_axh(ich) = nexttile(ich);
    hold on;
    for ipat = 1:numel(patterns)
        h = plot(set_ref_t{ipat} - set_t_shift(ipat), set_mean{ipat}(:,ich), ...
             'LineWidth', 1.5, 'DisplayName', labels{ipat});
        if ich == 2
            cmp_lines(ipat) = h;
        end
    end
end

for i = 1:6
    axes(cmp_axh(i));
    grid on;
    xticks(1:10);
    title(title_txts{i});
    yline(cmp_axh(i),0);
    xlim([0 10])
    ylim([-0.009 0.009])
    if i==5 || i==6
        xlabel('Time (s)')
    else
        set(cmp_axh(i), 'XTickLabel', []);
    end
    if mod(i,2) == 1
        ylabel('Signal (mV)')
    else
        set(cmp_axh(i), 'YTickLabel', []);
    end
end
legend(cmp_axh(2), cmp_lines, 'Location', 'best', 'Interpreter', 'none');
