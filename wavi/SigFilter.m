classdef SigFilter < handle
    % SigFilter  Streaming IIR/FIR filter helper for multi-channel signals.

    properties
        filterType = 'lowpass-iir'
        fs = 80
        order = 3
        cutoffHz = 4
        highpassHz = 0.2
        gain = 1.0
        nChannels = 1
    end

    properties (SetAccess = private)
        b = 1
        a = 1
        zi = []
    end

    methods
        function obj = SigFilter(options)
            if nargin < 1
                options = struct();
            end

            if isfield(options, 'filterType'), obj.filterType = options.filterType; end
            if isfield(options, 'fs'),         obj.fs = options.fs; end
            if isfield(options, 'order'),      obj.order = options.order; end
            if isfield(options, 'cutoffHz'),   obj.cutoffHz = options.cutoffHz; end
            if isfield(options, 'highpassHz'), obj.highpassHz = options.highpassHz; end
            if isfield(options, 'gain'),       obj.gain = options.gain; end
            if isfield(options, 'nChannels'),  obj.nChannels = options.nChannels; end

            obj.configure(struct());
        end

        function configure(obj, options)
            if nargin < 2
                options = struct();
            end

            if isfield(options, 'filterType'), obj.filterType = options.filterType; end
            if isfield(options, 'fs'),         obj.fs = options.fs; end
            if isfield(options, 'order'),      obj.order = options.order; end
            if isfield(options, 'cutoffHz'),   obj.cutoffHz = options.cutoffHz; end
            if isfield(options, 'highpassHz'), obj.highpassHz = options.highpassHz; end
            if isfield(options, 'gain'),       obj.gain = options.gain; end
            if isfield(options, 'nChannels'),  obj.nChannels = options.nChannels; end

            obj.order = max(1, round(double(obj.order)));
            obj.fs = max(1, double(obj.fs));
            obj.cutoffHz = max(0.001, double(obj.cutoffHz));
            obj.highpassHz = max(0.001, double(obj.highpassHz));
            obj.nChannels = max(1, round(double(obj.nChannels)));
            obj.gain = double(obj.gain);

            [obj.b, obj.a] = SigFilter.designFilter( ...
                obj.filterType, obj.fs, obj.order, obj.cutoffHz, obj.highpassHz);
            obj.reset();
        end

        function setChannelCount(obj, nChannels)
            obj.nChannels = max(1, round(double(nChannels)));
            obj.reset();
        end

        function reset(obj)
            zLen = max(length(obj.a), length(obj.b)) - 1;
            obj.zi = zeros(zLen, obj.nChannels);
        end

        function y = apply(obj, x)
            if isempty(x)
                y = x;
                return;
            end

            x = double(x);
            if size(x, 2) ~= obj.nChannels
                error('SigFilter:ChannelMismatch', ...
                    'Input channels (%d) do not match filter channels (%d).', size(x, 2), obj.nChannels);
            end

            x(~isfinite(x)) = 0;
            y = zeros(size(x));
            for k = 1:obj.nChannels
                [yk, obj.zi(:,k)] = filter(obj.b, obj.a, x(:,k), obj.zi(:,k));
                y(:,k) = obj.gain * yk;
            end
        end

        function metrics = computeMetrics(obj, raw, filtered, blockSize)
            if nargin < 4 || isempty(blockSize)
                blockSize = 1;
            end
            raw = double(raw);
            filtered = double(filtered);
            if ~isequal(size(raw), size(filtered))
                error('SigFilter:SizeMismatch', 'Raw and filtered arrays must have the same size.');
            end

            e = raw - filtered;
            mse = mean(e(:).^2, 'omitnan');
            maxAbsErr = max(abs(e(:)), [], 'omitnan');
            rmsRaw = rms(raw(:), 'omitnan');
            rmsFiltered = rms(filtered(:), 'omitnan');
            snrDb = SigFilter.signalToNoiseDb(raw(:), e(:));

            if numel(obj.a) == 1
                gdSamples = (numel(obj.b) - 1) / 2;
            else
                gdSamples = max(numel(obj.a), numel(obj.b)) - 1;
            end
            latencyMs = 1000 * (gdSamples + (double(blockSize) - 1)) / obj.fs;

            metrics = struct( ...
                'mse', mse, ...
                'maxAbsErr', maxAbsErr, ...
                'rmsRaw', rmsRaw, ...
                'rmsFiltered', rmsFiltered, ...
                'snrDb', snrDb, ...
                'latencyMs', latencyMs);
        end
    end

    methods (Static, Access = private)
        function [b, a] = designFilter(filterType, fs, order, cutoffHz, highpassHz)
            nyq = fs / 2;
            switch lower(string(filterType))
                case "moving-average"
                    n = max(1, order);
                    b = ones(1, n) / n;
                    a = 1;
                case "lowpass-iir"
                    wn = min(max(cutoffHz / nyq, 0.001), 0.999);
                    n = max(1, order);
                    [b, a] = butter(n, wn, 'low');
                case "highpass-iir"
                    wn = min(max(highpassHz / nyq, 0.001), 0.999);
                    n = max(1, order);
                    [b, a] = butter(n, wn, 'high');
                case "biquad-lowpass"
                    q = 0.707;
                    w0 = 2*pi * min(max(cutoffHz, 0.5), nyq*0.95) / fs;
                    alph = sin(w0) / (2*q);
                    cw = cos(w0);
                    b0 =  (1 - cw) / 2;
                    b1 =   1 - cw;
                    b2 =  (1 - cw) / 2;
                    a0 =   1 + alph;
                    a1 =  -2 * cw;
                    a2 =   1 - alph;
                    b = [b0 b1 b2] / a0;
                    a = [1  a1/a0  a2/a0];
                otherwise
                    error('SigFilter:BadType', 'Unknown filter type: %s', string(filterType));
            end
        end

        function snrDb = signalToNoiseDb(signal, noise)
            pSig = mean(signal(:).^2, 'omitnan');
            pNoise = mean(noise(:).^2, 'omitnan');
            snrDb = 10*log10((pSig + eps) / (pNoise + eps));
        end
    end
end
