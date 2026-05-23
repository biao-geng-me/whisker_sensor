classdef TcpClientApp < handle
    % TCPCLIENTAPP TCP client UI shim that mirrors SerialPortApp's interface.
    %
    % wavi.m treats the underlying transport via three handles only:
    %   - obj.SerialApp.s             (a serialport/tcpclient with the same
    %                                  read/write API used by wavi.m)
    %   - the SerialConnected event   (fires once obj.s is ready)
    %   - the SerialDisconnected event
    %
    % MATLAB's tcpclient supports read(s,n,'single'), readline(s),
    % writeline(s,...), write(s,bytes,'uint8'), flush(s), NumBytesAvailable,
    % and isvalid(s) -- exactly the calls wavi.m makes against serialport --
    % so substituting this class for SerialPortApp requires no other change.

    properties (Access = public)
        s                       % tcpclient handle (empty when disconnected)
        host = '127.0.0.1'
        port = 5555
        baudrate = 0            % unused; kept for SerialPortApp API parity
    end

    properties (Access = private)
        UIFigure
        HostField
        PortField
        ConnectButton
        StatusLabel
    end

    events
        SerialConnected
        SerialDisconnected
    end

    methods (Access = public)
        function app = TcpClientApp(varargin)
            if nargin == 0
                app.UIFigure = uifigure;
                app.UIFigure.Name = 'TCP Client';
                app.UIFigure.Position = [100 100 320 150];
                parent = app.UIFigure;
            elseif nargin == 1
                parent = varargin{1};
            else
                error('TcpClientApp:TooManyArgs','Too many arguments.');
            end

            app.createComponents(parent);
            if nargout == 0
                clear app
            end
        end

        function setEndpoint(app, host, port)
            % Update host/port and the visible UI fields together.
            if ~isempty(host)
                app.host = char(host);
                if ~isempty(app.HostField) && isvalid(app.HostField)
                    app.HostField.Value = app.host;
                end
            end
            if ~isempty(port)
                app.port = double(port);
                if ~isempty(app.PortField) && isvalid(app.PortField)
                    app.PortField.Value = app.port;
                end
            end
        end
    end

    methods (Access = private)
        function createComponents(app, parent)
            gl = uigridlayout(parent, [3, 1]);
            gl.RowHeight = {'2x', '2x', '1x'};
            gl.ColumnWidth = {'1x'};
            gl.Padding = 5;

            % Row 1: host/port entry
            hpgl = uigridlayout(gl, [1, 4]);
            hpgl.RowHeight = {'1x'};
            hpgl.ColumnWidth = {'1x', '3x', 40, '1.5x'};
            hpgl.Padding = [0 0 0 0];
            hpgl.Layout.Row = 1;

            uilabel(hpgl, 'Text', 'Host:', 'HorizontalAlignment', 'right');
            app.HostField = uieditfield(hpgl, 'text');
            app.HostField.Value = app.host;
            app.HostField.ValueChangedFcn = @(src,~) app.onHostChanged(src);

            uilabel(hpgl, 'Text', 'Port:', 'HorizontalAlignment', 'right');
            app.PortField = uieditfield(hpgl, 'numeric');
            app.PortField.Value = app.port;
            app.PortField.Limits = [1 65535];
            app.PortField.RoundFractionalValues = 'on';
            app.PortField.ValueChangedFcn = @(src,~) app.onPortChanged(src);

            % Row 2: connect/disconnect button
            app.ConnectButton = uibutton(gl, ...
                'Text', 'Connect', ...
                'ButtonPushedFcn', @(src, evt) app.onConnectPressed(src, evt));

            % Row 3: status
            app.StatusLabel = uilabel(gl, ...
                'Text', 'Disconnected', ...
                'FontColor', [1 0 0], ...
                'HorizontalAlignment', 'center', ...
                'FontWeight', 'bold');
        end

        function onHostChanged(app, src)
            app.host = src.Value;
        end

        function onPortChanged(app, src)
            app.port = double(src.Value);
        end

        function onConnectPressed(app, ~, ~)
            if strcmp(app.ConnectButton.Text, 'Connect')
                app.host = app.HostField.Value;
                app.port = double(app.PortField.Value);

                app.StatusLabel.Text = 'Connecting...';
                app.StatusLabel.FontColor = [0.1 0.5 0.8];
                drawnow;

                try
                    app.s = tcpclient(app.host, app.port, "Timeout", 5);
                    % default ByteOrder is little-endian, matching Arduino
                    app.StatusLabel.Text = sprintf('Connected to %s:%d', app.host, app.port);
                    app.StatusLabel.FontColor = [0 0.8 0];
                    app.ConnectButton.Text = 'Disconnect';
                    app.HostField.Enable = 'off';
                    app.PortField.Enable = 'off';
                    notify(app, 'SerialConnected');
                catch ME
                    app.StatusLabel.Text = 'Connection failed';
                    app.StatusLabel.FontColor = [1 0 0];
                    app.s = [];
                    disp(ME.message);
                end
            else
                try
                    if ~isempty(app.s) && isvalid(app.s)
                        delete(app.s);
                    end
                catch
                end
                app.s = [];

                app.StatusLabel.Text = 'Disconnected';
                app.StatusLabel.FontColor = [1 0 0];
                app.ConnectButton.Text = 'Connect';
                app.HostField.Enable = 'on';
                app.PortField.Enable = 'on';
                notify(app, 'SerialDisconnected');
            end
        end
    end
end
