classdef Controller
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        joy
        fig
        hInputText
    end
    
    methods
        function obj = Controller()
            %CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            obj.joy = vrjoystick(1);
            obj.fig = uifigure('name','Joystick Test');
            obj.hInputText = uilabel(obj.fig);
            obj.hInputText.Position = [100 100 300 30];

            while true
              [axx, buttons, povs] = read(obj.joy);
              msg = "Press some buttons";
              if any(buttons)
                  msg = ['Pressed buttons: ' num2str(find(buttons))];
                fprintf('%s\n',msg);
                fprintf('axes:');fprintf('%g ',axx);fprintf('\n');
                fprintf('povs:');fprintf('%g ',povs);fprintf('\n');
                fprintf('btns:');fprintf('%g ',buttons);fprintf('\n');
              end
              obj.hInputText.Text = msg;
              pause(0.05);
            end
        end
        
        function run_test(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            while true
              [axx, buttons, povs] = read(obj.joy);
              if any(buttons)
                  msg = ['Pressed buttons: ' num2str(find(buttons))];
                obj.hInputText.Text = msg;
                disp(msg);
                disp(axx);
                disp(povs);
              end
              pause(0.05);
            end
        end
    end
end

