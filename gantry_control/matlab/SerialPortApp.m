% Save this code as "SerialPortApp.m" and run it in MATLAB.
% This app allows a user to select a serial port, connect, and
% then refresh the available port list upon disconnection.

classdef SerialPortApp < handle

    properties (Access = public)
        s                    % Serial port object handle
        portname = ''
        baudrate = 9600
    end
    properties (Access = private)
        UIFigure             % Main UI figure
        PortDropdown         % Dropdown list for serial ports
        ConnectButton        % Button to connect/disconnect
        StatusLabel          % Label to display connection status
    end

    events
        SerialConnected
        SerialDisconnected
    end

    methods (Access = private)

        % Connect/Disconnect button callback
        function ConnectButtonPushed(app, event)
            % Check the current state of the button text to determine action.
            if strcmp(app.ConnectButton.Text, 'Connect')
                % --- Attempt to Connect ---
                
                % Update status label to show "Connecting..."
                app.StatusLabel.Text = 'Connecting...';
                app.StatusLabel.FontColor = [0.1 0.5 0.8]; % Blue
                
                portName = app.PortDropdown.Value;
                app.portname = portName;
                try
                    % Use a try-catch block to handle connection errors robustly.
                    % Create the serial port object and store it in an app property.
                    app.s = serialport(portName, app.baudrate); 
                    
                    % Update UI for a successful connection.
                    app.StatusLabel.Text = ['Connected to ', portName];
                    app.StatusLabel.FontColor = [0 0.8 0]; % Green
                    app.ConnectButton.Text = 'Disconnect';
                    app.PortDropdown.Enable = 'off'; % Disable dropdown during connection
                    notify(app,'SerialConnected');
                catch ME
                    % Handle connection failure.
                    app.StatusLabel.Text = 'Connection failed';
                    app.StatusLabel.FontColor = [1 0 0]; % Red
                    disp(ME.message); % Display the MATLAB error message in the Command Window for debugging.
                end
                
            else
                % --- Disconnect ---
                
                % Check if the serial port object is valid before deleting.
                try
                    if ~isempty(app.s) && isvalid(app.s)
                        delete(app.s); % Explicitly delete the serial port object to release the port.
                    end
                catch
                end
                
                % Update UI for disconnection.
                app.StatusLabel.Text = 'Disconnected';
                app.StatusLabel.FontColor = [1 0 0]; % Red
                app.ConnectButton.Text = 'Connect';
                
                % Call the helper function to refresh the port list.
                % bgeng: automatic disconnect detection seems too
                % complicated, deferred
                app.refreshPortList();
                notify(app,'SerialDisconnected');
            end
        end

        % Helper function to find and update the list of available serial ports.
        function refreshPortList(app)
            ports = serialportlist("available");
            
            if isempty(ports)
                app.PortDropdown.Items = {'No Ports Found'};
                app.ConnectButton.Enable = 'off';
                app.StatusLabel.Text = 'No ports found';
                app.StatusLabel.FontColor = [0.5 0.5 0.5]; % Gray
            else
                app.PortDropdown.Items = ports;
                if ismember(app.portname,ports)
                    pname = app.portname;
                else
                    pname = ports{1};
                end
                app.PortDropdown.Value = pname;
                app.ConnectButton.Enable = 'on';
                app.PortDropdown.Enable = 'on';
                
                % Keep the status label as disconnected or failed
                if strcmp(app.StatusLabel.Text, 'No ports found') || ...
                   strcmp(app.StatusLabel.Text, 'Connection failed')
                    % Do nothing, keep the current status
                else
                    app.StatusLabel.Text = 'Disconnected';
                    app.StatusLabel.FontColor = [1 0 0];
                end
            end
        end

        % App startup function
        function startupFcn(app)
            % This helper function refreshes the port list.
            app.refreshPortList();
        end

        % Create and configure all UI components and their layout.
        function createComponents(app,parent)
            % Use a grid layout for a clean, responsive design.
            gl = uigridlayout(parent, [3 1]);
            gl.RowHeight = {'1x', '1x', '1x'};
            gl.ColumnWidth = {'1x'};
            gl.Padding = 15;
            
            % Create the dropdown label.
            % uilabel(gl, 'Text', 'Select Port:', 'HorizontalAlignment', 'center');
            
            % Create the port selection dropdown.
            app.PortDropdown = uidropdown(gl, ...
                'Items', {'Loading...'}, 'DropDownOpeningFcn', @(src,evt) app.refreshPortList());
            
            % Create the connect/disconnect button.
            app.ConnectButton = uibutton(gl, ...
                'Text', 'Connect', ...
                'ButtonPushedFcn', @(src, event) ConnectButtonPushed(app, event));
            
            % Create the status label.
            app.StatusLabel = uilabel(gl, ...
                'Text', 'Disconnected', ...
                'FontColor', [1 0 0], ...
                'HorizontalAlignment', 'center', ...
                'FontWeight', 'bold');
        end
    end
    
    methods (Access = public)        
        % The main function to run the app.
        function app = SerialPortApp(varargin)
            if nargin == 0
                % Create separate UI figure.
                app.UIFigure = uifigure;
                app.UIFigure.Name = 'Serial Port App';
                app.UIFigure.Position = [100 100 300 150];
                parent = app.UIFigure;
            elseif nargin == 1
                % create the UI in the parent container.
                parent = varargin{1};
            else
                error('Too many arguments.');
            end
            

            app.createComponents(parent);
            app.startupFcn();
            if nargout == 0
                clear app % this avoids returning the object to the workspace.
            end
        end
    end
end
