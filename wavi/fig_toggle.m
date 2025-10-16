function fig_toggle()
% FIGURE_VISIBILITY_APP Creates a simple UI to toggle the visibility of three different plots.
%
% This version uses the 'FigureToggler' class to encapsulate the button logic.
% To run, ensure 'FigureToggler.m' is in the same directory.
% Type 'figure_visibility_app' in the MATLAB Command Window.

    % --- 1. Create the UI Figure (The main control window) ---
    f_ui = uifigure('Name', 'Plot Visibility Controller (Class-Based)', ...
                    'Position', [100 100 350 220], ...
                    'Resize', 'off'); % Prevent resizing for simplicity

    % --- 2. Create the three standard figures/plots (Initially Invisible) ---
    % We use 'figure' instead of 'uifigure' for the plots themselves to get 
    % standard, native figure windows.

    % Plot 1: Sine Wave
    h_fig1 = figure('Name', 'Plot 1: Sine Wave', 'Visible', 'off', ...
                    'NumberTitle', 'off', 'Color', [0.95 0.95 1]);
    plot(0:0.1:4*pi, sin(0:0.1:4*pi), 'LineWidth', 3, 'Color', [0 0.4470 0.7410]);
    title('Standard Sine Wave');
    grid on;

    % Plot 2: Cosine Wave
    h_fig2 = figure('Name', 'Plot 2: Cosine Wave', 'Visible', 'off', ...
                    'NumberTitle', 'off', 'Color', [1 0.95 0.95]);
    plot(0:0.1:4*pi, cos(0:0.1:4*pi), 'LineWidth', 3, 'Color', [0.8500 0.3250 0.0980]);
    title('Standard Cosine Wave');
    grid on;

    % Plot 3: Scatter Plot
    h_fig3 = figure('Name', 'Plot 3: Scatter Plot', 'Visible', 'off', ...
                    'NumberTitle', 'off', 'Color', [0.95 1 0.95]);
    scatter(randn(100, 1), randn(100, 1), 70, randn(100, 1), 'filled', 'MarkerEdgeColor', 'k');
    title('Random Scatter Data');
    colormap(h_fig3, 'parula'); % Set colormap for the figure
    colorbar;

    % --- 3. Instantiate the FigureToggler class for each plot ---
    % This replaces the previous explicit uibutton calls and the nested function.
    
    % Toggler 1 (Controls h_fig1)
    toggler1 = FigureToggler(h_fig1, f_ui, 'Toggle Sine Plot', [75 160 200 30]);

    % Toggler 2 (Controls h_fig2)
    toggler2 = FigureToggler(h_fig2, f_ui, 'Toggle Cosine Plot', [75 110 200 30]);

    % Toggler 3 (Controls h_fig3)
    toggler3 = FigureToggler(h_fig3, f_ui, 'Toggle Scatter Plot', [75 60 200 30]);
    
    % Store the togglers in the UI figure's UserData to ensure they persist 
    % while the UI is open (preventing premature garbage collection).
    f_ui.UserData.Togglers = [toggler1, toggler2, toggler3];


    % --- 4. Clean up figures when the main UI is closed ---
    % Note: The class cleanup is now slightly more robust.

    f_ui.CloseRequestFcn = @cleanup_figures;

    function cleanup_figures(~, ~)
        % Close all controlled figures if they are still open
        if isvalid(h_fig1); delete(h_fig1); end
        if isvalid(h_fig2); delete(h_fig2); end
        if isvalid(h_fig3); delete(h_fig3); end

        % Close the UI figure itself (and trigger the deletion of the Toggler objects)
        delete(f_ui);
    end

end
