function [co,f] = makeWhiskerMesh(options)
    %MAKEWHISKERMESH generate surface mesh for given whisker shape parameters
    %   B. Geng 2022-12-19
    %

    %  2022-12-22
    %    the previous version changes aspect ratio when adding r-c phase
    %    now maintaining aspect ratio

    %  2024-01-30
    %  add wavenumber
    %  add caps
    %  change parameters to semi-axis

    % todo:
    % adaptive circumferential grid based on curvature 
    % see https://math.stackexchange.com/questions/527538/how-to-calculate-the-curvature-of-an-ellipse

    arguments
        % default whisker (mm) Harbor seal avg (Rinehart et al 2017)
        options.a = 0.525
        options.b = 0.178
        options.lambda = 1.724*2;
        options.k = 0.416
        options.l = 0.219
        options.alpha = 0
        options.wavenumber = 1
        options.nzl = 20
        options.neq1 = 5 % # of elements in narrow side
        options.neq2 = 5 % # of elements in wide side
        options.scale = 1
        options.start_phase = 0
    end

    A = mean([options.a options.k]); % mean major
    B = mean([options.b options.l]); % mean minor
    lambda = options.lambda;
    a = (options.a - options.k)/2; % major amplitude
    b = (options.l - options.b)/2; % minor amplitude

    alpha = options.alpha;
    nwave = options.wavenumber;

    phi0 = options.start_phase;

    % two tiers non-uniform circumferencial grid
    nq1 = options.neq1; nq2=options.neq2;
    dq1 = pi/3/nq1;
    dq2 = pi/3/nq2;
    q = [-pi/3:dq1:pi/3, (pi/3+dq2):dq2:2/3*pi, (2/3*pi+dq1):dq1:4/3*pi, (4/3*pi+dq2):dq2:(5/3*pi-dq2)];
    q = q';
    neq = numel(q);
    npq = neq; % # of points in circumferential (q)

    % offset angle
    offset=A*sind(alpha)*1.5;
    Zs = -offset;
    Ze = lambda*nwave + offset;

    dz = lambda/options.nzl;

    nez = round((lambda)/dz)*nwave; % # of elements in z
    npz = nez+1;
    dz = (Ze-Zs)/nez;
    co = zeros(npz*npq, 3);

    for j=1:npz
        z = Zs + (j-1)*dz;
        % Major and minor axis
        M = A + a*cos(z/lambda*2*pi+phi0);
        M = M/cosd(alpha); % to keep aspect ratio
        m = B + b*cos(z/lambda*2*pi++phi0+pi);

        x=zeros(npq,3);

        x(:,1) = M*cos(q);
        x(:,2) = m*sin(q);
        x(:,3) = z;

        x = rotate_mesh(x,[0 1 0],alpha,[0 0 z]);

        co((j-1)*npq+1:j*npq,:) = x;
    end 

    % 
    ne_tot = neq*nez*2 + neq*2;
    f = zeros(ne_tot,3);

    nc = 0;
    for i=1:nez
    for j=1:neq-1
        ip1 = (i-1)*neq+j;
        ip2 = ip1+1;
        ip3 = ip1+neq;
        ip4 = ip3+1;
        nc = nc + 1;
        f(nc,:)=[ip1,ip2,ip3];
        nc = nc+1;
        f(nc,:)=[ip2,ip4,ip3];
    end
    ip1=ip2;
    ip3=ip4;
    ip2 = (i-1)*neq+1;
    ip4 = ip2+neq;
    nc = nc + 1;
    f(nc,:)=[ip1,ip2,ip3];
    nc = nc+1;
    f(nc,:)=[ip2,ip4,ip3];
    end

    % caps at two ends
    % create two center points

    co = [co; 0 0 co(1,3); 0 0 co(end,3)] * options.scale;

    for i = 1:neq-1
        nc=nc+1;
        ip1 = size(co,1)-1;
        ip2 = i;
        ip3 = i+1;
        f(nc,:) = [ip1 ip3 ip2];
    end
    nc = nc + 1;
    f(nc,:) = [ip1 1 neq];

    %
    for i = 1:neq-1
        nc=nc+1;
        ip1 = size(co,1);
        ip2 = nez*neq+i;
        ip3 = nez*neq+i+1;
        f(nc,:) = [ip1 ip2 ip3];
    end
    nc = nc + 1;
    f(nc,:) = [ip1 nez*neq+neq nez*neq+1];

end