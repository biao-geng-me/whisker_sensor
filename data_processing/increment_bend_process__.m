% bgeng 2024-04-29 process incremental load data

clear;clc;close all
datdir = 'H:\Shared drives\Biao Reseach\Work\prototype5_2024-06-07\bend-1';
savdir = 'C:\Users\bigeme\Working\SealWhisker\sensor\prototype_tests\prototype4_2024-06-07\bend1';


% constants
tag = 'pt5';
cmd = [902 5 7 3 5 10]; % cmd used for inc test
Fs = 88; % signal sample rate
ns_sstep = 45; % number of sampling points during servo step
rservo = 20; % servo arm radius, mm
angle0 = 315; % rotation table read at zero load orientation
Lt = 80.3; % from cup surface to actuation point, mm
Lr = 11.5; % flexible pdms part, mm
% end

%%%%%% find repeats %%%%%%%
% angle-specific inputs
n = 6; % angle index, 0 to 24, corresponding to 0° to 360°
use_channel_no = 2; % for peak detection
hmin = 1e-3; % min peak height for detecting end of loading steps
peak_sign = -1;

nlead = Fs*1.5; % number of sampling points to exclude at the beginning of each increment
t_avg = 1; % time used to takge average for each level.
% end

angle = angle0 + n*15;
if n*15~=360
    if angle > 360
        angle = angle -360;
    end
end


qstep = cmd(2); % servo step angle
nstep = cmd(3);
t_hold = cmd(4); % hold time for each increment
nrepeat = cmd(5);
t_rep = cmd(6); % hold time between repeats


var = zeros(nstep+1,2);
err = zeros(nstep+1,2);
xh  = rservo*tand(0:qstep:qstep*nstep)';

varr1 = zeros(nstep+1,nrepeat);
varr2 = zeros(nstep+1,nrepeat);

fname = sprintf("st_*_pt5_inc_%03dd.dat",angle); % change this to match data files

flist = dir(fullfile(datdir,fname));
fname = flist.name;

angle = angle - angle0;
if angle < 0
    angle = angle + 360;
end

tag = sprintf("%s_%03d",tag,angle);

dat = load_datalog(fullfile(datdir,fname));
y1 = dat{:,3};
y2 = dat{:,4};

ib_m = 1*Fs; % index for mean calculation
ind_m = ib_m:ib_m+3*Fs;

v01 = mean(y1(ind_m)); % initial values
v02 = mean(y2(ind_m));


figure; hold on
plot(y1-y1(1),'-r');
plot(ind_m,y1(ind_m)-y1(1),'ok');
plot(y2-y2(1),'-b')

if use_channel_no==1
    y = y1;
else
    y = y2;
end

dy = smoothdata(diff(y),'movmean',Fs);
figure; plot(dy);

% set markers
dmin = Fs*(t_hold*nstep+t_rep)+ns_sstep*nstep;
[pks,locs]=findpeaks(dy*peak_sign,"MinPeakHeight",hmin,"MinPeakDistance",dmin);


figure(2);hold on;plot(locs+1,pks*peak_sign,'or');

locs = locs -88; % distance frome true end due to servo acceleration effect
figure(1);plot(locs+1,y(locs+1)-y(1),'sk')

%% process inc data

for j = 1:nrepeat

    ie = locs(j);
    ib = ie - (Fs*t_hold+ns_sstep)*(nstep+1);

    yrep = y1(ib:ie);

    for i = 1:nstep + 1
        is_b = ib+(i-1)*(Fs*t_hold+ns_sstep) + nlead;
        is_e = is_b + Fs*t_avg;
        varr1(i,j) = mean(y1(is_b:is_e));
        varr2(i,j) = mean(y2(is_b:is_e));
        ind_s = is_b:is_e;
        figure(1);plot(is_b:is_e,y1(ind_s)-y1(1),'-^k');
    end
end 

%% plot bend sensitivity curve
for j = 1:nstep+1
    var(j,1) = mean(varr1(j,:)-v01);
    err(j,1) = std(varr1(j,:));

    var(j,2) = mean(varr2(j,:)-v02);
    err(j,2) = std(varr2(j,:));
end

set(0, 'DefaultAxesFontName', 'Arial')
set(0, 'DefaultAxesFontSize', 20)
figure;
hold on
errorbar(xh,var(:,1),err(:,1),'LineWidth',2,'Color','r');
errorbar(xh,var(:,2),err(:,2),'LineWidth',2,'Color','b');
xlabel("lateral displacement (mm)");
ylabel('signal (mV)')
legend({'ch1','ch2'},'Location','best');
title(sprintf('Orientation %d°',angle));
grid on
box on

%% save
fid = fopen(fullfile(savdir,sprintf('%s.dat',tag)),'w');
fprintf(fid,'%12.5e %12.5e %12.5e %12.5e %12.5e %12.5e\n',[xh var err bend_angle(Lt,Lr,xh)]');
fclose(fid);

%% calculate slope
v = var;
dq = bend_angle(Lt,Lr,xh(end))-bend_angle(Lt,Lr,xh(2));
slope = (v(end,:)-v(2,:))/dq;
fprintf('bend-signal slope:%5.3f, %5.3f\n',slope);


%% plot signal (optional)

figure; hold on
t = (1:numel(y1))/88;
plot(t,y1-y1(1),'-r','LineWidth',2);
plot(t,y2-y2(1),'-b','LineWidth',2);

xlabel('time (s)')
ylabel('signal (mV)')
grid on
box on
xlim([0 240]);
legend({'ch1','ch2'});
title(sprintf('Orientation %d°',angle));

figure(1)


