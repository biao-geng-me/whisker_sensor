classdef PathData
    % PathData - load XY path and compute arc length, curvature radius,
    % and provide interpolants for x(arc), y(arc), r(arc).
    properties
        x % vector (mm?) original units as provided
        y
        arc % cumulative arc length vector (same units as x,y spacing)
        r % radius of curvature at arc locations
        a % tangent angle (radians) at arc locations
        xp % griddedInterpolant for x(arc)
        yp % griddedInterpolant for y(arc)
        rp % griddedInterpolant for r(arc)
        ap % griddedInterpolant for a(arc)
        Ltot % total arc length
    end

    methods
        function obj = PathData(input)
            % input can be:
            %  - filename (string) pointing to a .mat or whitespace-separated xy file
            %  - Nx2 numeric array [x y]
            %  - two arguments (x,y) are not supported in this simple constructor
            if nargin<1 || isempty(input)
                error('PathData:NoInput','Provide a filename or Nx2 numeric matrix');
            end

            if ischar(input) || isstring(input)
                % try to load file
                data = load(input);
                if isstruct(data)
                    fn = fieldnames(data);
                    val = data.(fn{1});
                else
                    val = data;
                end
                if isnumeric(val) && size(val,2)>=2
                    obj.x = double(val(:,1));
                    obj.y = double(val(:,2));
                else
                    error('PathData:InvalidFile','File does not contain Nx2 numeric data');
                end
            elseif isnumeric(input) && size(input,2)>=2
                obj.x = double(input(:,1));
                obj.y = double(input(:,2));
            else
                error('PathData:InvalidInput','Input must be a filename or Nx2 numeric array');
            end

            % compute arc length
            dx = diff(obj.x);
            dy = diff(obj.y);
            dl = sqrt(dx.^2 + dy.^2);
            obj.a = atan(dy./dx); % tangent angle
            obj.a = [obj.a; obj.a(end)]; % pad to same length
            obj.arc = [0; cumsum(dl(:))];
            obj.Ltot = obj.arc(end);

            % compute radius of curvature using repository function if available
            try
                rp_vec = computeRadiusGeometric(obj.x, obj.y);
                % computeRadiusGeometric returns radius per sample; ensure same length as arc
                if numel(rp_vec) == numel(obj.x)
                    obj.r = rp_vec(:);
                else
                    obj.r = nan(size(obj.x));
                end
            catch
                % fallback: set to large radius (near-straight)
                obj.r = inf(size(obj.x));
            end

            % build interpolants - allow evaluating at arbitrary arc positions
            obj.xp = griddedInterpolant(obj.arc, obj.x, 'linear', 'nearest');
            obj.yp = griddedInterpolant(obj.arc, obj.y, 'linear', 'nearest');
            obj.rp = griddedInterpolant(obj.arc, obj.r, 'linear', 'nearest');
            obj.ap = griddedInterpolant(obj.arc, obj.a, 'linear', 'nearest');
        end

        function [xp,yp,rp,ap,L] = getInterpolants(obj)
            xp = obj.xp; yp = obj.yp; rp = obj.rp; ap = obj.ap; L = obj.Ltot;
        end
    end
end
