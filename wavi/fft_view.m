classdef fft_view < handle
    %FFT_VIEW Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fh % figure handle
        sh % surface plot handle
        bh % bar plot handle
        ah % average line plot handle
        ax1 % surface plot axis
        ax2 % bar plot axis
        n_avg = 40; % number of spectrogram frames to average for bar plot
    end
    
    methods
        function obj = fft_view(nch,nfreq,t_fft)
            %FFT_VIEW Construct an instance of this class
            %   Detailed explanation goes here
            
            [obj.fh,obj.sh,obj.bh,obj.ax1,obj.ax2,obj.ah] = init_fft_surf(nch,nfreq,t_fft);
        end
        
        function update(obj,fft_map_3d)
            %METHOD1 Summary of this method goes here
            %   fft_map_3d contains all fft results with size nfreq*nch*ntime
            update_fft_surf(obj.fh,obj.sh,obj.bh,fft_map_3d,obj.ax1,obj.ax2,obj.ah,n_avg=obj.n_avg);
        end
    end
end

