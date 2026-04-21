function plot_exported_results(exportDir)
%PLOT_EXPORTED_RESULTS Generate report-ready figures from exported result bundles.
%   plot_exported_results() scans apps/osgearth_viewer/data/exports and batch-processes
%   every export bundle it finds. Each bundle keeps its own figures in that
%   bundle's figures_matlab subfolder.
%
%   plot_exported_results(exportDir) accepts either:
%   1. a specific export bundle folder containing analysis-summary.csv, or
%   2. the export root folder containing multiple export bundles.

if nargin < 1 || strlength(string(exportDir)) == 0
    exportDir = defaultExportRoot();
else
    exportDir = string(exportDir);
end

exportDir = string(exportDir);
if ~isfolder(exportDir)
    error('Export directory not found: %s', exportDir);
end

analysisPath = fullfile(exportDir, 'analysis-summary.csv');
if isfile(analysisPath)
    plotSingleExport(exportDir);
else
    plotBatchExports(exportDir);
end
end

function plotBatchExports(exportRoot)
items = dir(exportRoot);
items = items([items.isdir]);
items = items(~ismember({items.name}, {'.', '..'}));
if isempty(items)
    error('No export folders found under %s', exportRoot);
end

processed = strings(0, 1);
for i = 1:numel(items)
    exportDir = fullfile(exportRoot, items(i).name);
    if isfile(fullfile(exportDir, 'analysis-summary.csv'))
        plotSingleExport(exportDir);
        processed(end + 1, 1) = string(exportDir); %#ok<AGROW>
    end
end

logPath = fullfile(exportRoot, 'batch_plot_log.txt');
fid = fopen(logPath, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Processed bundles: %d\n', numel(processed));
for i = 1:numel(processed)
    fprintf(fid, '%s\n', processed(i));
end
fprintf('Batch processing complete. Log written to: %s\n', logPath);
end

function plotSingleExport(exportDir)
outDir = fullfile(exportDir, 'figures_matlab');
if ~isfolder(outDir)
    mkdir(outDir);
end

analysis = readtable(fullfile(exportDir, 'analysis-summary.csv'));
impact = readtable(fullfile(exportDir, 'impact-point.csv'));
optimal = readtable(fullfile(exportDir, 'optimal-trajectory.csv'));
profile = readtable(fullfile(exportDir, 'optimal-trajectory-profile.csv'));
speedCurve = readtable(fullfile(exportDir, 'speed-time.csv'));
boundary = readtable(fullfile(exportDir, 'attack-boundary.csv'));
effectiveBoundary = readOptionalTable(fullfile(exportDir, 'effective-attack-boundary.csv'));
interceptedBoundary = readOptionalTable(fullfile(exportDir, 'intercepted-attack-boundary.csv'));
comparison = readtable(fullfile(exportDir, 'method-comparison.csv'));
runParameters = readtable(fullfile(exportDir, 'run-parameters.csv'), 'TextType', 'string');

styleFigureDefaults();

plotOptimalProfile(profile, analysis, outDir);
plotSpeedTime(speedCurve, analysis, outDir);
plotGroundTrack(optimal, impact, analysis, outDir);
plotAttackBoundary(boundary, effectiveBoundary, interceptedBoundary, analysis, runParameters, outDir);
plotMethodComparison(comparison, outDir);
plotMethodProfiles(exportDir, outDir);

fprintf('MATLAB figures written to: %s\n', outDir);
end

function rootDir = defaultExportRoot()
rootDir = fullfile(pwd, 'apps', 'osgearth_viewer', 'data', 'exports');
if ~isfolder(rootDir)
    error('Export root not found: %s', rootDir);
end
end

function styleFigureDefaults()
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultLineLineWidth', 1.8);
end

function tbl = readOptionalTable(path)
if isfile(path)
    tbl = readtable(path);
else
    tbl = table();
end
end

function plotOptimalProfile(profile, analysis, outDir)
fig = figure('Color', 'w', 'Position', [100, 100, 920, 560]);
plot(profile.ground_range_km, profile.height_km, 'Color', [0.95, 0.42, 0.10], 'LineWidth', 2.8);
grid on;
xlabel('Ground Range (km)');
ylabel('Altitude (km)');
title('Optimal Trajectory Profile');
subtitle(sprintf('%s | Peak alpha %.1f deg | Range %.1f km', ...
    string(analysis.alpha_schedule(1)), analysis.peak_alpha_deg(1), analysis.optimal_range_m(1) / 1000.0));
saveFigure(fig, fullfile(outDir, '01_optimal_trajectory_profile.png'));
close(fig);
end

function plotSpeedTime(speedCurve, analysis, outDir)
fig = figure('Color', 'w', 'Position', [120, 120, 920, 560]);
plot(speedCurve.time_s, speedCurve.speed_mps, 'Color', [0.15, 0.55, 0.95], 'LineWidth', 2.8);
grid on;
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed-Time History of the Optimal Trajectory');
subtitle(sprintf('Flight time %.1f s | Peak altitude %.1f km', ...
    analysis.flight_time_s(1), analysis.peak_altitude_m(1) / 1000.0));
saveFigure(fig, fullfile(outDir, '02_speed_time_history.png'));
close(fig);
end

function plotGroundTrack(optimal, impact, analysis, outDir)
fig = figure('Color', 'w', 'Position', [140, 140, 980, 620]);
plot(optimal.longitude_deg, optimal.latitude_deg, 'Color', [0.94, 0.46, 0.12], 'LineWidth', 2.8);
hold on;
plot(optimal.longitude_deg(1), optimal.latitude_deg(1), 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.14, 0.74, 0.56], 'MarkerEdgeColor', 'k');
plot(impact.longitude_deg(1), impact.latitude_deg(1), 'p', 'MarkerSize', 11, ...
    'MarkerFaceColor', [0.95, 0.18, 0.12], 'MarkerEdgeColor', 'k');
text(optimal.longitude_deg(1), optimal.latitude_deg(1), '  Launch', 'FontSize', 11, ...
    'VerticalAlignment', 'bottom');
text(impact.longitude_deg(1), impact.latitude_deg(1), ...
    sprintf('  Impact (%.2f, %.2f)', impact.longitude_deg(1), impact.latitude_deg(1)), ...
    'FontSize', 11, 'VerticalAlignment', 'top');
grid on;
axis equal;
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('Ground Track of the Optimal Trajectory');
subtitle(sprintf('Optimal heading %.2f deg | Attack-zone spread %.1f km', ...
    analysis.optimal_psi_deg(1), analysis.attack_range_spread_m(1) / 1000.0));
saveFigure(fig, fullfile(outDir, '03_ground_track.png'));
close(fig);
end

function plotAttackBoundary(boundary, effectiveBoundary, interceptedBoundary, analysis, runParameters, outDir)
fig = figure('Color', 'w', 'Position', [160, 160, 1320, 620]);
t = tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(t, 'Attack-Zone Boundary with Defense Filtering');
blockedMask = findBlockedHeadings(boundary, effectiveBoundary, interceptedBoundary);

nexttile;
hold on;
fill(boundary.longitude_deg, boundary.latitude_deg, [0.18, 0.75, 0.95], ...
    'FaceAlpha', 0.10, 'EdgeColor', 'none');
plot(boundary.longitude_deg, boundary.latitude_deg, '-', 'Color', [0.10, 0.72, 0.92], ...
    'LineWidth', 2.2, 'DisplayName', 'Raw attack zone');

if ~isempty(effectiveBoundary)
    fill(effectiveBoundary.longitude_deg, effectiveBoundary.latitude_deg, [0.22, 0.78, 0.38], ...
        'FaceAlpha', 0.16, 'EdgeColor', 'none');
    plot(effectiveBoundary.longitude_deg, effectiveBoundary.latitude_deg, '-', ...
        'Color', [0.12, 0.65, 0.24], 'LineWidth', 2.2, 'DisplayName', 'Effective attack zone');
    drawBlockedSectors(boundary, effectiveBoundary, blockedMask);
end

if ~isempty(interceptedBoundary) && ismember('intercept_longitude_deg', interceptedBoundary.Properties.VariableNames)
    scatter(interceptedBoundary.intercept_longitude_deg, interceptedBoundary.intercept_latitude_deg, ...
        30, 'filled', 'MarkerFaceColor', [0.84, 0.18, 0.12], ...
        'MarkerEdgeColor', [0.35, 0.05, 0.02], 'DisplayName', 'Intercept points');
end

plotBlockedArc(boundary, blockedMask);

sites = extractDefenseSites(runParameters);
for i = 1:numel(sites)
    plot(sites(i).longitude_deg, sites(i).latitude_deg, '^', 'MarkerSize', 9, ...
        'MarkerFaceColor', [0.99, 0.82, 0.20], 'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Defense site');
    text(sites(i).longitude_deg, sites(i).latitude_deg, "  " + sites(i).name, ...
        'FontSize', 10, 'Color', [0.28, 0.24, 0.10], 'VerticalAlignment', 'bottom');
end

plot(analysis.launch_longitude_deg(1), analysis.launch_latitude_deg(1), 'o', ...
    'MarkerSize', 8, 'MarkerFaceColor', [0.12, 0.80, 0.58], 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Launch site');
grid on;
axis equal;
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
subtitle(sprintf('Map view | Intercepted headings %d / %d', ...
    analysis.intercepted_attack_heading_count(1), analysis.attack_heading_count(1)));
legend('Location', 'best');

nexttile;
[headings, rawRangeKm, effectiveRangeKm] = alignBoundaryRanges(boundary, effectiveBoundary);
removedRangeKm = max(rawRangeKm - effectiveRangeKm, 0.0);
removedRangeKm(~blockedMask) = 0.0;
yyaxis left;
plot(headings, rawRangeKm, '-o', 'Color', [0.10, 0.72, 0.92], ...
    'MarkerSize', 4.2, 'DisplayName', 'Raw boundary');
hold on;
plot(headings, effectiveRangeKm, '-o', 'Color', [0.12, 0.65, 0.24], ...
    'MarkerSize', 4.2, 'DisplayName', 'Effective boundary');
if any(blockedMask)
    scatter(headings(blockedMask), rawRangeKm(blockedMask), 38, 'filled', ...
        'MarkerFaceColor', [0.84, 0.18, 0.12], 'MarkerEdgeColor', [0.35, 0.05, 0.02], ...
        'DisplayName', 'Blocked headings');
end
ylabel('Boundary range (km)');

yyaxis right;
bar(headings, removedRangeKm, 0.72, 'FaceColor', [0.84, 0.18, 0.12], ...
    'FaceAlpha', 0.22, 'EdgeColor', 'none', 'DisplayName', 'Removed range');
ylabel('Removed by defense (km)');

grid on;
xlabel('Heading (deg)');
title('Boundary Range by Heading');
subtitle(sprintf('Max removed %.1f km | Effective reachable flag: %d', ...
    max(removedRangeKm), analysis.target_effective_reachable(1)));
legend('Location', 'northwest');

saveFigure(fig, fullfile(outDir, '04_attack_zone_boundary.png'));
close(fig);
end

function plotMethodComparison(comparison, outDir)
fig = figure('Color', 'w', 'Position', [180, 180, 1120, 640]);
t = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(t, 'Method Comparison');

nexttile;
bar(categorical(comparison.method), comparison.range_m / 1000.0, 'FaceColor', [0.20, 0.63, 0.95]);
ylabel('Range (km)');
xtickangle(15);
grid on;
title('Range');

nexttile;
bar(categorical(comparison.method), comparison.flight_time_s, 'FaceColor', [0.97, 0.64, 0.18]);
ylabel('Flight Time (s)');
xtickangle(15);
grid on;
title('Flight Time');

nexttile;
bar(categorical(comparison.method), comparison.steps, 'FaceColor', [0.35, 0.78, 0.46]);
ylabel('Accepted Steps');
xtickangle(15);
grid on;
title('Integration Cost');

nexttile;
bar(categorical(comparison.method), comparison.peak_altitude_m / 1000.0, ...
    'FaceColor', [0.86, 0.42, 0.88]);
ylabel('Peak Altitude (km)');
xtickangle(15);
grid on;
title('Peak Altitude');

saveFigure(fig, fullfile(outDir, '05_method_comparison.png'));
close(fig);
end

function plotMethodProfiles(exportDir, outDir)
methodsDir = fullfile(exportDir, 'methods');
if ~isfolder(methodsDir)
    return;
end

methodFolders = dir(methodsDir);
methodFolders = methodFolders([methodFolders.isdir]);
methodFolders = methodFolders(~ismember({methodFolders.name}, {'.', '..'}));
if isempty(methodFolders)
    return;
end

colors = lines(numel(methodFolders));
fig = figure('Color', 'w', 'Position', [200, 200, 980, 620]);
hold on;
legendItems = cell(numel(methodFolders), 1);
for i = 1:numel(methodFolders)
    folder = fullfile(methodsDir, methodFolders(i).name);
    summary = readtable(fullfile(folder, 'summary.csv'));
    profile = readtable(fullfile(folder, 'trajectory-profile.csv'));
    plot(profile.ground_range_km, profile.height_km, 'Color', colors(i, :), 'LineWidth', 2.0);
    legendItems{i} = string(summary.method(1));
end
grid on;
xlabel('Ground Range (km)');
ylabel('Altitude (km)');
title('Trajectory Profiles by Method');
legend(legendItems, 'Location', 'best', 'Interpreter', 'none');
saveFigure(fig, fullfile(outDir, '06_method_profiles.png'));
close(fig);
end

function saveFigure(fig, filename)
[folder, ~, ~] = fileparts(filename);
if ~isfolder(folder)
    mkdir(folder);
end
exportgraphics(fig, filename, 'Resolution', 220);
end

function drawBlockedSectors(boundary, effectiveBoundary, blockedMask)
if height(boundary) < 2 || isempty(effectiveBoundary) || ~any(blockedMask)
    return;
end

[headings, rawLon, rawLat, effLon, effLat] = alignBoundaryPolygons(boundary, effectiveBoundary);
pointCount = numel(headings);
for i = 1:pointCount
    j = mod(i, pointCount) + 1;
    if ~blockedMask(i) && ~blockedMask(j)
        continue;
    end

    patch([rawLon(i), rawLon(j), effLon(j), effLon(i)], ...
          [rawLat(i), rawLat(j), effLat(j), effLat(i)], ...
          [0.86, 0.20, 0.12], 'FaceAlpha', 0.28, 'EdgeColor', 'none', ...
          'HandleVisibility', 'off');
end

plot(nan, nan, '-', 'Color', [0.86, 0.20, 0.12], 'LineWidth', 6.0, ...
    'DisplayName', 'Blocked sector');
end

function [headings, rawRangeKm, effectiveRangeKm] = alignBoundaryRanges(boundary, effectiveBoundary)
headings = boundary.heading_deg(:);
rawRangeKm = boundary.range_m(:) / 1000.0;
effectiveRangeKm = zeros(size(rawRangeKm));

if isempty(effectiveBoundary)
    effectiveRangeKm(:) = NaN;
    return;
end

[isMatched, matchIndex] = ismember(headings, effectiveBoundary.heading_deg(:));
effectiveRangeKm(:) = NaN;
effectiveRangeKm(isMatched) = effectiveBoundary.range_m(matchIndex(isMatched)) / 1000.0;
end

function [headings, rawLon, rawLat, effLon, effLat] = alignBoundaryPolygons(boundary, effectiveBoundary)
headings = boundary.heading_deg(:);
rawLon = boundary.longitude_deg(:);
rawLat = boundary.latitude_deg(:);
effLon = rawLon;
effLat = rawLat;

if isempty(effectiveBoundary)
    return;
end

[isMatched, matchIndex] = ismember(headings, effectiveBoundary.heading_deg(:));
effLon(isMatched) = effectiveBoundary.longitude_deg(matchIndex(isMatched));
effLat(isMatched) = effectiveBoundary.latitude_deg(matchIndex(isMatched));
end

function blockedMask = findBlockedHeadings(boundary, effectiveBoundary, interceptedBoundary)
blockedMask = false(height(boundary), 1);
if ismember('intercepted', boundary.Properties.VariableNames)
    blockedMask = boundary.intercepted(:) ~= 0;
end

if ~any(blockedMask) && ~isempty(interceptedBoundary) && ismember('heading_deg', interceptedBoundary.Properties.VariableNames)
    blockedMask = ismember(boundary.heading_deg(:), interceptedBoundary.heading_deg(:));
end

if ~any(blockedMask) && ~isempty(effectiveBoundary)
    [~, rawRangeKm, effectiveRangeKm] = alignBoundaryRanges(boundary, effectiveBoundary);
    blockedMask = (rawRangeKm - effectiveRangeKm) > 5.0;
end
end

function plotBlockedArc(boundary, blockedMask)
if ~any(blockedMask)
    return;
end

rawLon = boundary.longitude_deg(:);
rawLat = boundary.latitude_deg(:);
rawLon(~blockedMask) = NaN;
rawLat(~blockedMask) = NaN;
plot(rawLon, rawLat, '-', 'Color', [0.70, 0.05, 0.02], 'LineWidth', 3.4, ...
    'DisplayName', 'Blocked boundary arc');
end

function sites = extractDefenseSites(runParameters)
sites = struct('name', {}, 'longitude_deg', {}, 'latitude_deg', {});
if isempty(runParameters)
    return;
end

siteCount = round(getRunParameterNumeric(runParameters, "defense_site_count", 0));
for i = 1:siteCount
    enabled = getRunParameterNumeric(runParameters, "defense_site_" + i + "_enabled", 0);
    if enabled < 0.5
        continue;
    end

    site.name = getRunParameterText(runParameters, "defense_site_" + i + "_name", "Defense site " + i);
    site.longitude_deg = getRunParameterNumeric(runParameters, "defense_site_" + i + "_longitude_deg", NaN);
    site.latitude_deg = getRunParameterNumeric(runParameters, "defense_site_" + i + "_latitude_deg", NaN);
    sites(end + 1) = site; %#ok<AGROW>
end
end

function value = getRunParameterNumeric(runParameters, parameterName, defaultValue)
value = defaultValue;
match = string(runParameters.parameter) == string(parameterName);
if ~any(match)
    return;
end

rawValue = string(runParameters.value(find(match, 1, 'first')));
parsedValue = str2double(rawValue);
if ~isnan(parsedValue)
    value = parsedValue;
end
end

function value = getRunParameterText(runParameters, parameterName, defaultValue)
value = string(defaultValue);
match = string(runParameters.parameter) == string(parameterName);
if ~any(match)
    return;
end

value = string(runParameters.value(find(match, 1, 'first')));
end
