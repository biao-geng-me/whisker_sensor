function radius = computeradiusGeometric(x, y)
    % computeradiusGeometric Approximate radius using geometric 3-point circle method
    % Inputs:
    %   x, y - vectors of coordinates along the curve (same length)
    % Output:
    %   radius - vector of radius values (NaN at endpoints)

    n = length(x);
    radius = NaN(n, 1);  % Initialize with NaNs

    for i = 2:n-1
        % Three consecutive points
        x1 = x(i-1); y1 = y(i-1);
        x2 = x(i);   y2 = y(i);
        x3 = x(i+1); y3 = y(i+1);

        % Side lengths
        a = sqrt((x2 - x3)^2 + (y2 - y3)^2);
        b = sqrt((x1 - x3)^2 + (y1 - y3)^2);
        c = sqrt((x1 - x2)^2 + (y1 - y2)^2);

        % Semi-perimeter
        s = (a + b + c) / 2;

        % Triangle area using Heron's formula
        A = sqrt(s * (s - a) * (s - b) * (s - c));

        % radius: kappa = 4A / (a*b*c)
        if A ~= 0 && a*b*c ~= 0
            radius(i) = (a * b * c) / (4 * A );
        else
            radius(i) = Inf;  % Degenerate case (points collinear)
        end
    end
end
