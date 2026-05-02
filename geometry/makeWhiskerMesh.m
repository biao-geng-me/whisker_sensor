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
        options.include_handle = false
        options.handle_diameter = 2.5
        options.handle_length = 10
        options.transition_length = 1.5
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
    co_whisker = zeros(npz*npq, 3);

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

        % x = rotate_mesh(x,[0 1 0],alpha,[0 0 z]);

        co_whisker((j-1)*npq+1:j*npq,:) = x;
    end 

    co_whisker = co_whisker * options.scale;

    % Optional handle uses two circular sections: base and transition ring.
    if options.include_handle
        r_handle = options.handle_diameter/2;
        z_whisker_start = co_whisker(1,3);
        z_handle_transition = z_whisker_start - options.transition_length;
        z_handle_base = z_handle_transition - options.handle_length;

        xh1 = zeros(npq,3);
        xh1(:,1) = r_handle*cos(q);
        xh1(:,2) = r_handle*sin(q);
        xh1(:,3) = z_handle_base;
        % xh1 = rotate_mesh(xh1,[0 1 0],alpha,[0 0 z_handle_base]);

        xh2 = zeros(npq,3);
        xh2(:,1) = r_handle*cos(q);
        xh2(:,2) = r_handle*sin(q);
        xh2(:,3) = z_handle_transition;
        % xh2 = rotate_mesh(xh2,[0 1 0],alpha,[0 0 z_handle_transition]);

        co = [xh1; xh2; co_whisker];
    else
        co = co_whisker;
    end

    nring = size(co,1)/neq;
    ne_tot = neq*(nring-1)*2 + neq*2;
    f = zeros(ne_tot,3);

    nc = 0;
    for i = 1:nring-1
    for j = 1:neq
        j2 = mod(j,neq)+1;
        ip1 = (i-1)*neq + j;
        ip2 = (i-1)*neq + j2;
        ip3 = i*neq + j;
        ip4 = i*neq + j2;
        nc = nc + 1;
        f(nc,:) = [ip1,ip2,ip3];
        nc = nc + 1;
        f(nc,:) = [ip2,ip4,ip3];
    end
    end

    % caps at two ends
    % create two center points

    co = [co; 0 0 co(1,3); 0 0 co(end,3)];

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
    last_ring_start = (nring-1)*neq + 1;
    for i = 1:neq-1
        nc=nc+1;
        ip1 = size(co,1);
        ip2 = last_ring_start + i - 1;
        ip3 = last_ring_start + i;
        f(nc,:) = [ip1 ip2 ip3];
    end
    nc = nc + 1;
    f(nc,:) = [ip1 last_ring_start+neq-1 last_ring_start];

end