% step contour

clear; close all
datdir = 'H:\Shared drives\Biao Reseach\Work\prototype5_2024-06-07\bend_processed';
tag = 'pt4';
Amp = 0.5;

a = 0:15:360;

v1 = zeros(8,25);
v2 = v1;
err1 = v1;
err2 = v1;
for i=1:numel(a)
    fname = fullfile(datdir,sprintf('%s_%03d.dat',tag,a(i)));
    info = load(fname);
    xh = info(:,1);
    v1(:,i) = info(:,2);
    v2(:,i) = info(:,3);
    err1(:,i) = info(:,4);
    err2(:,i) = info(:,5);
end
xh = info(:,1);

%% contour
figure;
contourf(a,xh,v1);
colormap jet
c=colorbar;
xticks(a(1:6:end));
xlabel('load orientation (°)');
ylabel('load amplitude (mm)');
clim([-1 1]*Amp)
title('Ch1')

figure;
contourf(a,xh,v2);
colormap jet
c2=colorbar;
xticks(a(1:6:end));
xlabel('load orientation (°)');
ylabel('load amplitude (mm)');
clim([-1 1]*Amp)
title('Ch2');

%% line plots

figure;

errorbar(a,v1(end,:),err1(end,:),'Color','r','LineWidth',2);
hold on
errorbar(a,v2(end,:),err2(end,:),'Color','b','LineWidth',2);

xticks(a(1:3:end));
yticks(-0.3:0.3:0.3);
xlabel('load orientation (°)')
ylabel('signal (mV)')
grid on
box on
xlim([0 360]);
legend({'ch1','ch2'},'Location','best');
title(sprintf('load amplitude %3.1f (mm)',xh(end)));
ylim([-1 1]*Amp)


%% measured angle for selected loads
figure;
hold on

mkr = {'o','+','s'};

iload = [3 5 7];

for j=1:numel(iload)
i = iload(j);
ma = atan2(v2(i,:),v1(i,:))/pi*180;
ind = ma<0;

ma(ind) = ma(ind) + 360;
ma(1) = ma(1) - 360;

plot(a,ma,mkr{j},'LineWidth',2);

end


plot(a,a,'--k','LineWidth',2);

xticks(a(1:6:end));
yticks(a(1:6:end));
xlabel('load orientation (°)')
ylabel('measured orientation (°)')
grid on
box on
xlim([-15 360]);
lgd = cell(1,3);
for i = 1:numel(iload)
    lgd{i} = sprintf('%3.1fmm',xh(iload(i)));
end
legend(lgd);

% title(sprintf('load amplitude %3.1f (mm)',xh(end)));
ylim([-15 360])

pbaspect([1 1 1]);
daspect([1 1 1]);


