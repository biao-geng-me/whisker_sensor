classdef SignalData
    % SignalData  Simple container for time-series sensor data
    %
    %   obj = SignalData(filepath) loads common file types (.mat, .csv, .txt)
    %   and tries to extract a time vector and signal matrix. It performs
    %   light cleaning: convert datetimes, remove NaNs, normalize shapes.

    properties
        filepath char
        filename char
        dtime   
        t        double   % time vector (seconds, column)
        sig      double   % signal matrix (samples x channels)
        Fs       double = NaN % sampling rate if available
        nChannels
        nSamples
        timeOffset double = 0 % global time offset (seconds)
        sigOffset  double = [] % per-channel signal offset (1 x nChannels)
        chNames  cell = {}    % channel names
        dt_str   char     % date time string of collection time
        meta     struct = struct()
    end

    methods
        function obj = SignalData(filepath,opt)
            arguments
                filepath
                opt.chNames = []
            end
            if nargin==0
                return
            end
            obj.filepath = filepath;
            obj.chNames = opt.chNames;

            [~,f,ext] = fileparts(filepath);
            obj.filename = [f ext];

            switch lower(ext)
                % load t and sig from file
                case {'.dat', '.txt'}
                    try
                        [dat,dt_str,tag] = load_datalog(filepath);
                    catch
                        error('SignalData:ReadFailed','Failed to read table from %s', filepath);
                    end
                    
                    % obj.dtime  = datetime([datestr(dat{:,1}) datestr(dat{:,2},' HH:MM:SS.FFF')]);
                    obj.dtime  = dat{:,1} + dat{:,2};
                    t = obj.dtime;
                    sig = dat{:,3:end};
                otherwise
                    error('SignalData:UnsupportedExt','Unsupported extension: %s', ext);
            end
            
            if isdatetime(t)
                % convert to seconds relative to first sample
                dt = seconds(t - t(1));
                t = double(dt);
            elseif isduration(t)
                t = seconds(t);
            end
            t = double(t(:));

            % ensure numeric
            sig = double(sig);
            mask = ~isnan(t) & ~any(isnan(sig),2);
            t = t(mask);
            sig = sig(mask,:);

            [obj.nSamples,obj.nChannels] = size(sig);
            obj.sig = sig;

            obj.t = t;
            obj.Fs = (numel(t)-1)/t(end);

            % initialize offsets: timeOffset default 0, sigOffset = mean of first 0.5s samples
            obj.timeOffset = 0;
            if ~isempty(obj.sig)
                % determine window length for 0.5 seconds
                if ~isnan(obj.Fs) && obj.Fs>0
                    n0 = max(1, floor(0.5 * obj.Fs));
                else
                    n0 = min( max(1,floor(0.5)), size(obj.sig,1) );
                end
                % prefer time-based indexing when t available
                if ~isempty(obj.t)
                    t0end = obj.t(1) + 0.5;
                    idx0 = find(obj.t <= t0end);
                    if isempty(idx0)
                        idx0 = 1:min(n0, size(obj.sig,1));
                    end
                else
                    idx0 = 1:min(n0, size(obj.sig,1));
                end
                obj.sigOffset = mean(obj.sig(idx0,:),1);
            else
                obj.sigOffset = [];
            end

            % default channel names
            if isempty(obj.chNames) && ~isempty(obj.sig)
                nc = size(obj.sig,2);
                for i=1:nc
                    isensor = ceil(i/2);
                    ich = mod(i,2);
                    if ich == 0
                        ich = 2;
                    end
                    % obj.chNames = arrayfun(@(k) sprintf('Ch%d',k), 1:nc, 'UniformOutput', false);
                    obj.chNames{i} = sprintf('S%d-Ch%d',isensor,ich);
                end
            end

            % store some meta
            obj.meta.source = filepath;
        end

        function tOut = getTimeWithOffset(obj)
            if isempty(obj.t)
                tOut = [];
            else
                tOut = obj.t + obj.timeOffset;
            end
        end

        function sigOut = getSigWithOffset(obj)
            if isempty(obj.sig)
                sigOut = [];
                return;
            end
            if isempty(obj.sigOffset)
                sigOut = obj.sig;
            else
                % ensure row vector for offsets
                so = obj.sigOffset;
                if iscolumn(so)
                    so = so';
                end
                sigOut = bsxfun(@plus, obj.sig, so);
            end
        end

        function setTimeOffset(obj, val)
            obj.timeOffset = double(val);
        end

        function setSigOffset(obj, chIdx, val)
            if isempty(obj.sigOffset)
                obj.sigOffset = zeros(1, size(obj.sig,2));
            end
            obj.sigOffset(chIdx) = double(val);
        end
    end
end
