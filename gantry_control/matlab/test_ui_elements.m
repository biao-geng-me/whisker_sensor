%% Test CarriageTrack
clear;clc

fig1 = uifigure('Name','Figure 1');
ax = uiaxes(fig1);
ax.Position = [50,50,300,300];
ax.XLim = [-0.2 1.2];
ax.YLim = [-0.2 1.2];
ax.DataAspectRatio = [1 1 1];
ax.Box = 'on';
ct1 = CarriageTrack(ax);

i=0;
while (1)
    i = i + 1;
    n = mod(i,100);
    ct1.set_xy(n/100+sin(4*pi*n/100)/5,n/100+cos(2*pi*n/100)/10);
    ct1.redraw;
    pause(0.05);
end

%% Test GantryView
clear;clc
gv = GantryView();
i=0;
while (1)
    tic
    i = i + 1;
    x = mod(i,4500);
    y = mod(i,1500);
    gv.redraw(x+sin(2*pi*x/4500)*500,y,4500-x+sin(2*pi*x/4500)*500,1500-y);
    drawnow limitrate
    toc 
end
%%
% Create a plot
plot(rand(10, 1));
ax = gca;

% Disable all interactions by setting the property
ax.Interactions = [];