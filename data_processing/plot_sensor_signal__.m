
% plot a sensor signal to check
clear;clc;close all
path = 'sensor_data_samples';
file = 'st_2024-05-14_1210_19.75_pt3_rep50_045d.dat';
dat = load_datalog(fullfile(path,file));
dtime = datetime([datestr(dat{:,1}) datestr(dat{:,2},' HH:MM:SS.FFF')]);
ttime = dtime - dtime(1);
t = seconds(ttime); % time in seconds
y1 = dat{:,3};
y2 = dat{:,4};

y1 = y1-y1(1);
y2 = y2-y2(1);

h=figure(Units="inch",Position=[1,1,5.2,1.5]);hold on;box on;grid on
set(gca, 'FontName', 'Times', 'FontSize', 10);
plot(t,y1,'-+r','LineWidth',1.5);
plot(t,y2,'-o')
% xlim([14 15.5])
% ylim([-0.5 1.5])
xlabel('time (s)')
ylabel('signal (mV)')
legend('ch 1','ch 2')
title('sensor signal')
