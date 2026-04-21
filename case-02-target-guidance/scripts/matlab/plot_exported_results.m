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
boundary = readOptionalTable(fullfile(exportDir, 'attack-boundary.csv'));
comparison = readtable(fullfile(exportDir, 'method-comparison.csv'));

styleFigureDefaults();

plotOptimalProfile(profile, analysis, outDir);
plotSpeedTime(speedCurve, analysis, outDir);
plotGroundTrack(optimal, impact, analysis, outDir);
if ~isempty(boundary)
    plotAttackBoundary(boundary, analysis, outDir);
end
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

function tf = hasVar(tbl, name)
tf = ismember(name, tbl.Properties.VariableNames);
end

function value = firstValue(tbl, name, fallback)
if nargin < 3
    fallback = [];
end
if hasVar(tbl, name) && height(tbl) >= 1
    value = tbl.(name)(1);
else
    value = fallback;
end
end

function tbl = readOptionalTable(path)
if ~isfile(path)
    tbl = [];
    return;
end

tbl = readtable(path);
if isempty(tbl) || height(tbl) == 0
    tbl = [];
end
end

function plotOptimalProfile(profile, analysis, outDir)
fig = figure('Color', 'w', 'Position', [100, 100, 920, 560]);
plot(profile.ground_range_km, profile.height_km, 'Color', [0.95, 0.42, 0.10], 'LineWidth', 2.8);
grid on;
xlabel('Ground Range (km)');
ylabel('Altitude (km)');
title('Optimal Trajectory Profile');
subtitle(sprintf('Initial (theta0, psi_v0) = (%.1f deg, %.1f deg) | Range %.1f km', ...
    firstValue(analysis, 'initial_theta_deg', NaN), ...
    firstValue(analysis, 'initial_psi_deg', NaN), ...
    firstValue(analysis, 'optimal_range_m', NaN) / 1000.0));
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
if hasVar(analysis, 'attack_range_spread_m')
    subtitle(sprintf('Optimal heading %.2f deg | Attack-zone spread %.1f km', ...
        firstValue(analysis, 'optimal_psi_deg', NaN), ...
        firstValue(analysis, 'attack_range_spread_m', NaN) / 1000.0));
else
    subtitle(sprintf('Optimal heading %.2f deg', ...
        firstValue(analysis, 'optimal_psi_deg', NaN)));
end
saveFigure(fig, fullfile(outDir, '03_ground_track.png'));
close(fig);
end

function plotAttackBoundary(boundary, analysis, outDir)
if isempty(boundary) || ~hasVar(boundary, 'longitude_deg') || ~hasVar(boundary, 'latitude_deg')
    return;
end

fig = figure('Color', 'w', 'Position', [160, 160, 980, 620]);
plot(boundary.longitude_deg, boundary.latitude_deg, '-', 'Color', [0.10, 0.72, 0.92], ...
    'LineWidth', 2.2);
hold on;
fill(boundary.longitude_deg, boundary.latitude_deg, [0.18, 0.75, 0.95], ...
    'FaceAlpha', 0.10, 'EdgeColor', 'none');
plot(analysis.launch_longitude_deg(1), analysis.launch_latitude_deg(1), 'o', ...
    'MarkerSize', 8, 'MarkerFaceColor', [0.12, 0.80, 0.58], 'MarkerEdgeColor', 'k');
grid on;
axis equal;
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('Attack-Zone Boundary');
subtitle(sprintf('Min %.1f km | Max %.1f km | Spread %.1f km', ...
    firstValue(analysis, 'attack_range_min_m', NaN) / 1000.0, ...
    firstValue(analysis, 'attack_range_max_m', NaN) / 1000.0, ...
    firstValue(analysis, 'attack_range_spread_m', NaN) / 1000.0));
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
