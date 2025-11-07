% generate a path xy data
clc; clear; close all
% domain size, mm
L = 3800;
H = 800;

% ----------------------------------
% Define path
% ----------------------------------
interp_method = 'linear';
nm = 4;
xm = linspace(0,L,1000);
% xm = [0.0 0.1 0.3 0.5 0.7 1.0]*L;
% ym = [0.1 0.1 0.9 0.9 0.1 0.1]*H;

% ym = -xm.*(xm-L)/L^2*2*H;
% ym = ym + 0.3*H*sin(2*pi*xm/750).*ym/max(ym)+100;

ym = 0.3*H*sin(2*pi*xm/900)+400;

fy = griddedInterpolant(xm,ym,interp_method,'spline');

x = 0:10:L;
y = fy(x);

figure;hold on;box on
ax = gca;
ax.XDir = "reverse";
ax.YDir = "reverse";
ax.XAxisLocation = "top";
ax.YAxisLocation = "right";
ax.XLabel.String = "X (mm)";
ax.YLabel.String = "Y (mm)";
scatter(xm,ym,30,"red")
plot(x,y,LineWidth=3,Color=[1,1,1]*0.75);
xlim([0 L]);
ylim([0 H]);
daspect([1 1 1 ])
xlabel('X (mm)');
ylabel('Y (mm)');
title('Cylinder trajectory')

% arc length
dx = [0 diff(x)];
dy = [0 diff(y)];
s = cumsum(vecnorm([dx; dy], 2, 1));

% plot(x,s);
xp = griddedInterpolant(s,x,'spline','nearest');
ym = griddedInterpolant(s,y,'spline','nearest');
plot(xp(1:10:s(end)),ym(1:10:s(end)),'r');

%% save 

fid = fopen(fullfile('paths','xy_path12.dat'),'w');
fprintf(fid,'%12.5f, %12.5f\n',[x;y]);
fclose(fid);