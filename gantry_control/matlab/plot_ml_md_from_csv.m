function [figHandle, selectedIdx, data] = plot_ml_md_from_csv(csvFile, selectedIdx)
%PLOT_ML_MD_FROM_CSV Plot one ML/MD pair from trajectory CSV on one axis.
%   plot_ml_md_from_csv(csvFile) reads the CSV file, lets you select an
%   ML/MD index, and plots MLx and MDx versus time in a single plot.
%
%   plot_ml_md_from_csv(csvFile, selectedIdx) uses the provided index.
%
%   [figHandle, selectedIdx, data] = ... returns the figure handle,
%   selected index, and loaded table.

    if nargin < 1 || strlength(string(csvFile)) == 0
        error('Please provide a CSV file path.');
    end

    if nargin < 2
        selectedIdx = [];
    end

    data = readtable(csvFile);
    varNames = string(data.Properties.VariableNames);

    mlTokens = regexp(varNames, '^ML(\d+)$', 'tokens', 'once');
    mdTokens = regexp(varNames, '^MD(\d+)$', 'tokens', 'once');

    mlIdx = nan(size(varNames));
    mdIdx = nan(size(varNames));

    for i = 1:numel(varNames)
        if ~isempty(mlTokens{i})
            mlIdx(i) = str2double(mlTokens{i}{1});
        end
        if ~isempty(mdTokens{i})
            mdIdx(i) = str2double(mdTokens{i}{1});
        end
    end

    mlAvailable = mlIdx(~isnan(mlIdx));
    mdAvailable = mdIdx(~isnan(mdIdx));
    commonIdx = intersect(mlAvailable, mdAvailable);

    if isempty(commonIdx)
        error('No matching MLx/MDx pairs found in file: %s', csvFile);
    end

    commonIdx = sort(commonIdx);

    if isempty(selectedIdx)
        labels = compose('Index %d (ML%d / MD%d)', commonIdx, commonIdx, commonIdx);
        [selection, ok] = listdlg(...
            'PromptString', 'Select ML/MD index to plot:', ...
            'SelectionMode', 'single', ...
            'ListString', cellstr(labels), ...
            'ListSize', [260 180]);

        if ~ok || isempty(selection)
            error('No index selected.');
        end

        selectedIdx = commonIdx(selection);
    else
        selectedIdx = double(selectedIdx);
        if ~isscalar(selectedIdx) || ~ismember(selectedIdx, commonIdx)
            error('selectedIdx must be one of: %s', mat2str(commonIdx));
        end
    end

    mlName = sprintf('ML%d', selectedIdx);
    mdName = sprintf('MD%d', selectedIdx);

    if ismember('t', data.Properties.VariableNames)
        x = data.t;
        xLabelText = 't';
    else
        x = (1:height(data)).';
        xLabelText = 'Sample';
    end

    figHandle = figure('Color', 'w', 'Name', sprintf('ML/MD Index %d', selectedIdx));
    hold on;
    plot(x, data.(mlName), 'LineWidth', 1.5, 'DisplayName', mlName);
    plot(x, data.(mdName), 'LineWidth', 1.5, 'DisplayName', mdName);
    hold off;

    grid on;
    xlabel(xLabelText);
    ylabel('Value');
    title(sprintf('%s and %s from %s', mlName, mdName, csvFile), 'Interpreter', 'none');
    legend('Location', 'best');
end
