% step contour

clear; close all
datdir = '.';
a = 0:15:360;

v = zeros(7,25);
v2 = v;
err = v;
err2 = v;
for i=1:numel(a)
    fname = fullfile(datdir,sprintf('pt3_inc_%03d.dat',a(i)));
    info = load(fname);
    v(:,i) = info(:,2);
    v2(:,i) = info(:,3);
    err(:,i) = info(:,4);
    err2(:,i) = info(:,5);
end
xh = info(:,1);

%% contour
figure;
contourf(a,xh,v);
colormap jet
c=colorbar;
xticks(a(1:6:end));
xlabel('load orientation (°)');
ylabel('load amplitude (mm)');
clim([-0.3 0.3001])
title('Ch1')

figure;
contourf(a,xh,v2);
colormap jet
c2=colorbar;
xticks(a(1:6:end));
xlabel('load orientation (°)');
ylabel('load amplitude (mm)');
clim([-0.3 0.3001])
title('Ch2');

%% line plots

figure;

errorbar(a,v(end,:),err(end,:),'Color','r','LineWidth',2);
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
ylim([-0.3 0.3])


%% measured angle for selected loads
figure;
hold on

mkr = {'o','+','s'};

iload = [3 5 7];

for j=1:numel(iload)
i = iload(j);
ma = atan2(v2(i,:),v(i,:))/pi*180;
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
legend({'2.1 mm','4.3 mm','6.5 mm'});

% title(sprintf('load amplitude %3.1f (mm)',xh(end)));
ylim([-15 360])

pbaspect([1 1 1]);
daspect([1 1 1]);


