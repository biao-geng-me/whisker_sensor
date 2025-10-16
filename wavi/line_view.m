classdef line_view < handle
    %LINE_VIEW Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fh
        ln_sig
        nsensor
    end
    
    methods
        function obj = line_view(nsensor,darr,sig)
            %LINE_VIEW Construct an instance of this class
            %   Detailed explanation goes here

            % time history window
            display_info=get(groot);
            pos1 = display_info.MonitorPositions(1,:);
            obj.fh = figure('OuterPosition',[0,61,pos1(3)*0.5,pos1(4)-80]);hold on;grid on;box on
            obj.ln_sig = cell(1,nsensor*2);
            for i=1:nsensor
                obj.ln_sig{i*2-1} = line(darr,sig(:,i*2-1),'Color','r','LineWidth',2,'Marker','+');
                obj.ln_sig{i*2}   = line(darr,sig(:,i*2),  'Color','b','LineWidth',2,'Marker','None');
            end
            obj.nsensor = nsensor;
        end
        
        function update(obj,darr,sig,V0,scale)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            for i=1:obj.nsensor
                for j=1:2
                    ich = (i-1)*2+j;
                    obj.ln_sig{ich}.XData = darr; 
                    obj.ln_sig{ich}.YData = (sig(:,ich)-V0(ich))*scale+i;
                end
            end
            set(0,'CurrentFigure',obj.fh);
            xlim([darr(1) darr(end)])
            ylim([0 obj.nsensor+1]);
        end
    end
end

