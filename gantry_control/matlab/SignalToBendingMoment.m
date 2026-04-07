classdef SignalToBendingMoment < handle
    % SignalToBendingMoment  Convert sensor samples to bending moments.
    %
    % Usage:
    %   conv = SignalToBendingMoment(calibrationFile, channelMapFile);
    %   bm = conv.convertSamples(samples);
    %   bmSim = conv.convertSamples(samples, reorderToSimulation=true);
    % example:
    %
    % conv = SignalToBendingMoment( ...
    % 'sensor_calibration/calibration_sim_v1.csv', ...
    % 'sensor_calibration/ch_map_sim_v1.csv');

    % [dat, dtStr, tag] = load_datalog('your_log.dat');
    % samples = dat{:,3:end};
    % bendingMoments = conv.convertSamples(samples, reorderToSimulation=true);

    properties
        calibrationDataFile = ''
        channelMapFile = ''
        coefficients = []
        polarity = []
        expToSimChannelMap = []
    end

    methods
        function obj = SignalToBendingMoment(calibrationDataFile, channelMapFile)
            if nargin >= 1 && ~isempty(calibrationDataFile)
                obj.calibrationDataFile = calibrationDataFile;
                obj.loadCalibrationData(calibrationDataFile);
            end
            if nargin >= 2 && ~isempty(channelMapFile)
                obj.channelMapFile = channelMapFile;
                obj.loadChannelMap(channelMapFile);
            end
        end

        function loadCalibrationData(obj, calibrationDataFile)
            if nargin >= 2 && ~isempty(calibrationDataFile)
                obj.calibrationDataFile = calibrationDataFile;
            end
            if isempty(obj.calibrationDataFile)
                error('SignalToBendingMoment:MissingCalibrationFile', 'Calibration data file is not set.');
            end

            try
                calibData = readtable(obj.calibrationDataFile);
                % calibration data should have columns 'Sensor_ID,Drag_Pos,Drag_Neg,Lift_Pos,Lift_Neg,Drag_Polarity,Lift_Polarity'
                % Polarity is used for sign consistency
                % no ID mapping is performed, assuming calibration file is ordered to match installed sensor layout
                dragPos = calibData.Drag_Pos;
                dragNeg = calibData.Drag_Neg;
                liftPos = calibData.Lift_Pos;
                liftNeg = calibData.Lift_Neg;

                coeffs = [liftPos dragPos; liftNeg dragNeg]';
                obj.coefficients = reshape(coeffs(:), [], 2);
                polarityVec = [calibData.Lift_Polarity, calibData.Drag_Polarity]';
                obj.polarity = polarityVec(:);
            catch me
                error('SignalToBendingMoment:CalibrationDataError', ...
                    'Failed to load calibration data from %s: %s', obj.calibrationDataFile, me.message);
            end
        end

        function loadChannelMap(obj, channelMapFile)
            if nargin >= 2 && ~isempty(channelMapFile)
                obj.channelMapFile = channelMapFile;
            end
            if isempty(obj.channelMapFile)
                error('SignalToBendingMoment:MissingChannelMapFile', 'Channel map file is not set.');
            end

            try
                mapTable = readtable(obj.channelMapFile);
                expChannels = mapTable.exp_ch;
                simChannels = mapTable.sim_ch;
                if numel(unique(expChannels)) ~= numel(expChannels) || numel(unique(simChannels)) ~= numel(simChannels)
                    error('Channel mapping file contains duplicate channels.');
                end
                obj.expToSimChannelMap = expChannels;
            catch me
                error('SignalToBendingMoment:ChannelMapError', ...
                    'Failed to load channel mapping from %s: %s', obj.channelMapFile, me.message);
            end
        end

        function bendingMoments = convertSamples(obj, samples, options)
            arguments
                obj
                samples (:,:) double
                options.reorderToSimulation (1,1) logical = false
            end

            if isempty(obj.coefficients) || isempty(obj.polarity)
                error('SignalToBendingMoment:NotConfigured', 'Calibration coefficients have not been loaded.');
            end

            nChannels = size(samples, 2);
            if nChannels ~= size(obj.coefficients, 1)
                error('SignalToBendingMoment:ChannelCountMismatch', ...
                    'Sample channel count (%d) does not match calibration channel count (%d).', ...
                    nChannels, size(obj.coefficients, 1));
            end

            posCoeff = obj.coefficients(:, 1)';
            negCoeff = obj.coefficients(:, 2)';
            polarityVec = obj.polarity';
            sampleSignMask = samples >= 0;
            bendingMoments = samples .* (sampleSignMask .* posCoeff + (~sampleSignMask) .* negCoeff) .* polarityVec;

            if options.reorderToSimulation
                if isempty(obj.expToSimChannelMap)
                    error('SignalToBendingMoment:MissingChannelMap', 'Channel map must be loaded before reordering to simulation order.');
                end
                bendingMoments = bendingMoments(:, obj.expToSimChannelMap);
            end
        end
    end
end
