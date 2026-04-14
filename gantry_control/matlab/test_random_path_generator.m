clear
clc
close all

generator = RandomPathGenerator( ...
    'XRange', [0 4000], ...
    'YRange', [0 900], ...
    'MinSeedPoints', 3, ...
    'MaxSeedPoints', 5, ...
    'MaxSeedDeltaY', 250, ...
    'SampleSpacing', 10);

numTrials = 25;
previewCount = 10;
previewPaths = cell(previewCount, 1);
previewSeeds = cell(previewCount, 1);

for trialIdx = 1:numTrials
    % generator.Seed = trialIdx;
    [xy, seedPoints, meta] = generator.generate();

    assert(all(xy(:,1) >= generator.XRange(1) - eps), ...
        'Generated x values fell below the requested range.');
    assert(all(xy(:,1) <= generator.XRange(2) + eps), ...
        'Generated x values exceeded the requested range.');
    assert(all(xy(:,2) >= generator.YRange(1) - eps), ...
        'Generated y values fell below the requested range.');
    assert(all(xy(:,2) <= generator.YRange(2) + eps), ...
        'Generated y values exceeded the requested range.');

    dxSeed = diff(seedPoints(:,1));
    assert(all(abs(dxSeed - dxSeed(1)) < 1e-9), ...
        'Seed points are not equally spaced in x.');
    assert(all(abs(diff(seedPoints(:,2))) <= generator.MaxSeedDeltaY + 1e-9), ...
        'Consecutive seed-point y deviations exceeded MaxSeedDeltaY.');
    assert(meta.numSeedPoints >= generator.MinSeedPoints && ...
        meta.numSeedPoints <= generator.MaxSeedPoints, ...
        'Generator used an invalid number of seed points.');

    assert(RandomPathGenerator.validateGeneratedPath( ...
        xy, seedPoints, generator.XRange, generator.YRange, generator.MaxSeedDeltaY), ...
        'RandomPathGenerator.validateGeneratedPath rejected a generated path.');

    pd = PathData(xy);
    assert(pd.Ltot > 0, 'Generated PathData has zero arc length.');

    if trialIdx <= previewCount
        previewPaths{trialIdx} = xy;
        previewSeeds{trialIdx} = seedPoints;
    end
end

fprintf('RandomPathGenerator passed %d trials.\n', numTrials);

figure('Name', 'Random Path Generator Preview', 'Color', 'w');
hold on
grid on
box on

pathColors = lines(previewCount);
for idx = 1:previewCount
    xy = previewPaths{idx};
    seedPoints = previewSeeds{idx};
    plot(xy(:,1), xy(:,2), ...
        'LineWidth', 1.5, ...
        'Color', pathColors(idx,:), ...
        'DisplayName', sprintf('Path %d', idx));
    scatter(seedPoints(:,1), seedPoints(:,2), 45, pathColors(idx,:), ...
        'filled', 'HandleVisibility', 'off');
end

xlim(generator.XRange)
ylim(generator.YRange)
xlabel('X (mm)')
ylabel('Y (mm)')
title('Random spline paths and their seed points')
legend('Location', 'eastoutside')
daspect([1 1 1])
