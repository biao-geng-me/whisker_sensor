classdef FigureToggler < handle
    % FIGURETOGGLER A class to create a UI button that toggles the visibility
    % of a target MATLAB figure. The button now uses the 'state' style to
    % clearly represent the plot's visibility state (On/Off).
    %
    % Instantiation Example:
    % toggler = FigureToggler(h_fig1, f_ui, 'Toggle Plot', [x y w h]);

    properties (Access = private)
        FigureHandle % Handle to the figure being controlled
        ButtonHandle % Handle to the UI button itself
    end

    methods
        function obj = FigureToggler(fig, container, buttonText, position)
            % FIGURETOGGLER Constructor
            %   fig: Handle of the target figure (e.g., h_fig1)
            %   container: Handle of the UI container (e.g., f_ui)
            %   buttonText: Text to display on the button (string)
            %   position: [x y width height] for the button

            if ~isvalid(fig)
                error('FigureToggler:InvalidFigureHandle', 'The provided figure handle is invalid.');
            end

            % Store the figure handle
            obj.FigureHandle = fig;

            % Define common button properties
            button_props = {'FontName', 'Arial', 'FontSize', 9, 'BackgroundColor', [0.5 0.8 0.5], 'FontWeight', 'bold'};

            % Create the button in the specified container
            obj.ButtonHandle = uibutton(container,'state', ...
                button_props{:}, ...
                'Text', buttonText, ...
                'Value', false, ...                    % Set initial state to OFF (figure starts hidden)
                'Position', position, ...
                'ValueChangedFcn', @obj.toggleVisibility); % CHANGED: Use ValueChangedFcn for state buttons
        end

        function toggleVisibility(obj, ~, ~)
            % TOGGLEVISIBILITY Callback function for the button.
            % The figure's visibility is set based on the button's Value property (true/false).

            hFig = obj.FigureHandle;

            % Check if the controlled figure is still valid
            if isvalid(hFig)
                % Read the button's current state (true = ON, false = OFF)
                is_on = obj.ButtonHandle.Value;

                if is_on
                    hFig.Visible = 'on';
                    % disp(['Figure "', hFig.Name, '" is now VISIBLE (Button State: ON).']);
                    % Bring the plot window to the front
                    % figure(hFig);
                else
                    hFig.Visible = 'off';
                    % disp(['Figure "', hFig.Name, '" is now HIDDEN (Button State: OFF).']);
                end
            else
                warning('FigureToggler:ClosedFigure', 'The controlled figure was closed manually. Button will no longer function.');
                % If figure is closed, reset button state and disable it
                obj.ButtonHandle.Value = false;
                obj.ButtonHandle.Enable = 'off';
            end
        end
    end
end
