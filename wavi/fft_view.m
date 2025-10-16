classdef fft_view < handle
    %FFT_VIEW Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fh
        sh
        bh
        ax1
        ax2
    end
    
    methods
        function obj = fft_view(nch,nfreq,t_fft)
            %FFT_VIEW Construct an instance of this class
            %   Detailed explanation goes here
            
            [obj.fh,obj.sh,obj.bh,obj.ax1,obj.ax2] = init_fft_surf(nch,nfreq,t_fft);
        end
        
        function update(obj,fft_map)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            update_fft_surf(obj.fh,obj.sh,obj.bh,fft_map,obj.ax1,obj.ax2);
        end
    end
end

