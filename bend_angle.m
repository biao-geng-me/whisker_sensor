function ba = bend_angle(Lt,Lr,xh)
% compute bend angle from bending configuration and deflection, in degree
% Lt - total length
% Lr - flexible length
% xh - deflection

% Lt = 80.3; % from cup surface to actuation point
% Lr = 11.5; % flexible pdms part, mm


phi = 1e-4:0.01:1.51;
x = Lt*tan(phi) + (1-1./cos(phi))*Lr./phi; % constant height loading


% figure; plot(x,phi/pi*180); xlim([0 50])

% interp1(phi,x,18/180*pi)

ind = xh ~= 0;
ba = zeros(size(xh));

ba(ind) = interp1(x,phi,xh(ind))/pi()*180;
