classdef spectrogram_view < handle
    %SPECTROGRAM_VIEW 
    %   Detailed explanation goes here
    
    properties
        fh % figure handle
        mh % map handle
    end
    
    methods
        function obj = spectrogram_view(spec_data,Fs,nread)
            [obj.fh,obj.mh] = init_spec_plot(spec_data,Fs,nread);
        end
        
        function update(obj,spec_data)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.mh.CData = spec_data;
            set(0,'CurrentFigure',obj.fh);
            grid on;
        end
    end
end

