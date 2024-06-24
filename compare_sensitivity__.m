% compare multiple dipole sensitivity data

clear;
root = 'C:\Users\bigeme\Working\SealWhisker\sensor\prototype_tests';

flist = {fullfile(root,'prototype4_2024-06-07/ch2_sensitivity_10Hz.dat'),...
         fullfile(root,'prototype4_2024-06-07/ch2_x2_sensitivity_10Hz.dat'),...
         fullfile(root,'FatBoy 2ch 2024-05-08/sensitivity_10Hz.dat')
    };

mkr = {'-+','-o','-<'};
lgd ={'pt4 x3 ch2', 'pt4 x2 ch2', 'pt3 x3 ch1'};
ttl = '10 Hz';

set(0, 'DefaultAxesFontName', 'Arial')
set(0, 'DefaultAxesFontSize', 20)

figure; hold on

for i=1:numel(flist)
    s = load(flist{i});
    errorbar(s(:,2),s(:,3),s(:,4),'-+','LineWidth',2)

end

xlim([0 50]);
% ylim([0 0.01]);
xlabel('whisker tip flow velocity (mm/s)');
ylabel('Signal (mV)');
legend(lgd,'Location','best')
box on
title(ttl)