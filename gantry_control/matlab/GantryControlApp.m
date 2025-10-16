classdef GantryControlApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure          % Main UI figure
        SerialPanel1       % Panel to hold the SerialPortApp UI
        SerialPanel2
        ControlPanel      % Panel for motor control UI
        SerialApp1      % Handle to the SerialPortApp instance
        SerialApp2
        SpeedSlider       % Slider to control motor speed
        MotorStatusLabel  % Label to display motor status
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end
    
    % Other
    properties (Access = private)
        autoUpdate
    end

    % 
    % --- Methods ---
    %
    
    % Callbacks that handle component events
    methods (Access = private)
        
        % Callback for the motor speed slider
        function SpeedSliderValueChanged(app, event)
            if isvalid(app.SerialApp1.s)
                % Check if the serial port object is valid and connected.
                % This is how the main app gets access to the serial port.
                
                speedValue = round(app.SpeedSlider.Value);
                
                % Example: Send a command over the serial port.
                % You would define your own protocol here.
                % For example, sending a string like 'S123' to set speed.
                writeline(app.SerialApp1.s, ['S', num2str(speedValue)]);
                
                app.MotorStatusLabel.Text = ['Motor Running: ', num2str(speedValue), ' RPM'];
            else
                app.MotorStatusLabel.Text = 'Motor Disconnected';
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create and configure all UI components for the main app.
        function createComponents(app)
            % Create the main UI figure.
            app.UIFigure = uifigure;
            app.UIFigure.Name = 'Gantry Control';
            % app.UIFigure.Position = [100 100 400 300];
            app.UIFigure.WindowStyle = 'modal';
            
            % Use a grid layout for main UI.
            gl = uigridlayout(app.UIFigure);
            gl.ColumnWidth = {100, '1x'};
            gl.RowHeight = {180,180,'1x'};
            gl.Padding = 15;
            
            % Create a panel to hold the serial port UI.
            app.SerialPanel1 = uipanel(gl, 'Title', 'Front');
            app.SerialPanel2 = uipanel(gl, 'Title', 'Back');
            app.SerialPanel2.Layout.Row = 2;
            app.SerialPanel2.Layout.Column = 1;
          
            
            % Create a panel for motor control.
            app.ControlPanel = uipanel(gl, 'Title', 'Motor Control');
            
            % Add UI elements to the control panel using a grid layout.
            gl_control = uigridlayout(app.ControlPanel, [3 1]);
            gl_control.RowHeight = {'1x', '1x', '1x'};
            gl_control.Padding = 15;
            
            % Create the motor status label.
            app.MotorStatusLabel = uilabel(gl_control, 'Text', 'Motor Status:');
            app.MotorStatusLabel.FontWeight = 'bold';
            app.MotorStatusLabel.HorizontalAlignment = 'center';
            
            % Create the motor speed slider.
            app.SpeedSlider = uislider(gl_control, 'Limits', [0 100], 'Value', 0);
            app.SpeedSlider.ValueChangedFcn = @(src, event) app.SpeedSliderValueChanged(event);

            % Add a placeholder button for Stop.
            uibutton(gl_control, 'Text', 'Stop Motor', 'ButtonPushedFcn', @(src, event) app.stopMotor());
        end

        % App startup function
        function startupFcn(app)
            % Create instances of the SerialPortApp class in panels.
            app.SerialApp1 = SerialPortApp(app.SerialPanel1);
            app.SerialApp2 = SerialPortApp(app.SerialPanel2);
        end

    end

    % App creation and deletion
    methods (Access = public)

        % Construct myapp
        function app = GantryControlApp

            % Create UIFigure and components
            createComponents(app)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)
            
            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end