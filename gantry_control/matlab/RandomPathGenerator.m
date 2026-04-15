classdef RandomPathGenerator
    % RandomPathGenerator Generate bounded random spline paths for the gantry.
    % The generator creates 3-5 seed points with equally spaced x values,
    % bounded step-to-step y deviations, and evaluates a spline on a dense x grid.

    properties
        XRange = [0 3800]
        YRange = [0 850]
        MinSeedPoints = 3
        MaxSeedPoints = 5
        MaxSeedDeltaY = 350
        SampleSpacing = 10
        MaxAttempts = 100
        Seed = []
    end

    methods
        function obj = RandomPathGenerator(varargin)
            % Allow property overrides as name/value pairs.
            if mod(numel(varargin), 2) ~= 0
                error('RandomPathGenerator:InvalidArguments', ...
                    'Use name/value pairs when constructing RandomPathGenerator.');
            end

            for k = 1:2:numel(varargin)
                propName = varargin{k};
                if ~(ischar(propName) || isstring(propName))
                    error('RandomPathGenerator:InvalidPropertyName', ...
                        'Property names must be strings or character vectors.');
                end

                propName = char(propName);
                if ~isprop(obj, propName)
                    error('RandomPathGenerator:UnknownProperty', ...
                        'Unknown property "%s".', propName);
                end

                obj.(propName) = varargin{k + 1};
            end

            obj.validateConfiguration();
        end

        function [xy, seedPoints, meta] = generate(obj)
            % generate Create one random spline path as an Nx2 [x y] array.
            obj.validateConfiguration();
            rngCleanup = obj.configureRng(); %#ok<NASGU>

            xSamples = obj.buildSampleGrid();
            ySpan = diff(obj.YRange);

            for attempt = 1:obj.MaxAttempts
                numSeedPoints = randi([obj.MinSeedPoints obj.MaxSeedPoints]);
                xSeed = linspace(obj.XRange(1), obj.XRange(2), numSeedPoints);
                ySeed = zeros(1, numSeedPoints);

                edgeMargin = min(0.25 * ySpan, obj.MaxSeedDeltaY);
                yLow0 = obj.YRange(1) + edgeMargin;
                yHigh0 = obj.YRange(2) - edgeMargin;
                if yLow0 > yHigh0
                    yLow0 = obj.YRange(1);
                    yHigh0 = obj.YRange(2);
                end
                ySeed(1) = yLow0 + (yHigh0 - yLow0) * rand();

                for idx = 2:numSeedPoints
                    yLower = max(obj.YRange(1), ySeed(idx - 1) - obj.MaxSeedDeltaY);
                    yUpper = min(obj.YRange(2), ySeed(idx - 1) + obj.MaxSeedDeltaY);
                    ySeed(idx) = yLower + (yUpper - yLower) * rand();
                end

                ySamples = spline(xSeed, ySeed, xSamples);

                if all(ySamples >= obj.YRange(1)) && all(ySamples <= obj.YRange(2))
                    xy = [xSamples(:) ySamples(:)];
                    seedPoints = [xSeed(:) ySeed(:)];
                    meta = struct( ...
                        'attempts', attempt, ...
                        'numSeedPoints', numSeedPoints, ...
                        'seed', obj.Seed, ...
                        'xRange', obj.XRange, ...
                        'yRange', obj.YRange, ...
                        'sampleSpacing', obj.SampleSpacing, ...
                        'maxSeedDeltaY', obj.MaxSeedDeltaY);
                    return
                end
            end

            error('RandomPathGenerator:GenerationFailed', ...
                'Failed to generate a spline path within bounds after %d attempts.', ...
                obj.MaxAttempts);
        end

        function pd = generatePathData(obj)
            % generatePathData Return a PathData object for the generated path.
            xy = obj.generate();
            pd = PathData(xy);
        end

        function writeDat(obj, filename)
            % writeDat Save a generated path using the repository's .dat format.
            [xy, ~, ~] = obj.generate();
            fid = fopen(filename, 'w');
            if fid < 0
                error('RandomPathGenerator:FileOpenFailed', ...
                    'Could not open "%s" for writing.', filename);
            end

            fileCleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, '%12.5f, %12.5f\n', xy.');
        end
    end

    methods (Access = private)
        function validateConfiguration(obj)
            if ~isnumeric(obj.XRange) || numel(obj.XRange) ~= 2 || obj.XRange(1) >= obj.XRange(2)
                error('RandomPathGenerator:InvalidXRange', ...
                    'XRange must be a numeric [min max] vector with min < max.');
            end

            if ~isnumeric(obj.YRange) || numel(obj.YRange) ~= 2 || obj.YRange(1) >= obj.YRange(2)
                error('RandomPathGenerator:InvalidYRange', ...
                    'YRange must be a numeric [min max] vector with min < max.');
            end

            if obj.MinSeedPoints < 2 || obj.MaxSeedPoints < obj.MinSeedPoints
                error('RandomPathGenerator:InvalidSeedPointRange', ...
                    'Seed point bounds must satisfy 2 <= MinSeedPoints <= MaxSeedPoints.');
            end

            if obj.SampleSpacing <= 0
                error('RandomPathGenerator:InvalidSampleSpacing', ...
                    'SampleSpacing must be positive.');
            end

            if obj.MaxSeedDeltaY < 0
                error('RandomPathGenerator:InvalidMaxSeedDeltaY', ...
                    'MaxSeedDeltaY must be nonnegative.');
            end

            if obj.MaxAttempts < 1 || obj.MaxAttempts ~= floor(obj.MaxAttempts)
                error('RandomPathGenerator:InvalidMaxAttempts', ...
                    'MaxAttempts must be a positive integer.');
            end
        end

        function xSamples = buildSampleGrid(obj)
            xSamples = obj.XRange(1):obj.SampleSpacing:obj.XRange(2);
            if isempty(xSamples) || xSamples(end) < obj.XRange(2)
                xSamples = [xSamples obj.XRange(2)];
            end
        end

        function cleanupObj = configureRng(obj)
            cleanupObj = [];
            if isempty(obj.Seed)
                return
            end

            previousState = rng;
            rng(obj.Seed, 'twister');
            cleanupObj = onCleanup(@() rng(previousState));
        end
    end

    methods (Static)
        function tf = validateGeneratedPath(xy, seedPoints, xRange, yRange, maxSeedDeltaY)
            % validateGeneratedPath Return true when the generated path meets constraints.
            if nargin < 3 || isempty(xRange)
                xRange = [0 3800];
            end
            if nargin < 4 || isempty(yRange)
                yRange = [0 850];
            end
            if nargin < 5 || isempty(maxSeedDeltaY)
                maxSeedDeltaY = inf;
            end

            tf = size(xy, 2) >= 2 ...
                && all(diff(seedPoints(:,1)) > 0) ...
                && all(abs(diff(seedPoints(:,2))) <= maxSeedDeltaY + 1e-9) ...
                && all(xy(:,1) >= xRange(1) - 1e-9) ...
                && all(xy(:,1) <= xRange(2) + 1e-9) ...
                && all(xy(:,2) >= yRange(1) - 1e-9) ...
                && all(xy(:,2) <= yRange(2) + 1e-9);
        end
    end
end
