function outputPath = plot_blocked_attack_zone(exportDir)
%PLOT_BLOCKED_ATTACK_ZONE Create a defense-aware attack-zone map from one export bundle.

if nargin < 1 || strlength(string(exportDir)) == 0
    error('An export directory is required.');
end

exportDir = string(exportDir);
if ~isfolder(exportDir)
    error('Export directory not found: %s', exportDir);
end

boundary = readtable(fullfile(exportDir, 'attack-boundary.csv'));
effective = readtable(fullfile(exportDir, 'effective-attack-boundary.csv'));
intercepted = readtable(fullfile(exportDir, 'intercepted-attack-boundary.csv'));
analysis = readtable(fullfile(exportDir, 'analysis-summary.csv'));
runParameters = readtable(fullfile(exportDir, 'run-parameters.csv'), 'TextType', 'string');

blockedMask = boundary.intercepted ~= 0;
if ~any(blockedMask) && ~isempty(intercepted)
    blockedMask = ismember(boundary.heading_deg, intercepted.heading_deg);
end

fig = figure('Color', 'w', 'Position', [120, 120, 1080, 860], 'Visible', 'off');
hold on;

fill(boundary.longitude_deg, boundary.latitude_deg, [0.63, 0.83, 0.96], ...
    'FaceAlpha', 0.28, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(boundary.longitude_deg, boundary.latitude_deg, '-', 'Color', [0.10, 0.66, 0.88], ...
    'LineWidth', 3.4, 'DisplayName', 'Raw attack boundary');

fill(effective.longitude_deg, effective.latitude_deg, [0.50, 0.83, 0.58], ...
    'FaceAlpha', 0.34, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(effective.longitude_deg, effective.latitude_deg, '-', 'Color', [0.10, 0.55, 0.18], ...
    'LineWidth', 3.0, 'DisplayName', 'Effective boundary after defense');

[rawLon, rawLat, effLon, effLat] = alignBoundaryPolygons(boundary, effective);
pointCount = numel(rawLon);
for i = 1:pointCount
    j = mod(i, pointCount) + 1;
    if ~blockedMask(i) && ~blockedMask(j)
        continue;
    end

    patch([rawLon(i), rawLon(j), effLon(j), effLon(i)], ...
          [rawLat(i), rawLat(j), effLat(j), effLat(i)], ...
          [0.85, 0.18, 0.12], 'FaceAlpha', 0.42, 'EdgeColor', 'none', ...
          'HandleVisibility', 'off');
end

blockedArcLon = boundary.longitude_deg;
blockedArcLat = boundary.latitude_deg;
blockedArcLon(~blockedMask) = NaN;
blockedArcLat(~blockedMask) = NaN;
plot(blockedArcLon, blockedArcLat, '-', 'Color', [0.72, 0.04, 0.02], ...
    'LineWidth', 4.2, 'DisplayName', 'Blocked boundary arc');

if ~isempty(intercepted)
    scatter(intercepted.intercept_longitude_deg, intercepted.intercept_latitude_deg, ...
        48, 'filled', 'MarkerFaceColor', [0.74, 0.05, 0.03], ...
        'MarkerEdgeColor', [0.25, 0.02, 0.01], 'DisplayName', 'Intercept points');
end

plot(analysis.launch_longitude_deg(1), analysis.launch_latitude_deg(1), 'o', ...
    'MarkerSize', 13, 'MarkerFaceColor', [0.15, 0.82, 0.72], 'MarkerEdgeColor', 'k', ...
    'LineWidth', 1.2, 'DisplayName', 'Launch site');

siteLon = getRunParameterNumeric(runParameters, "defense_site_1_longitude_deg", NaN);
siteLat = getRunParameterNumeric(runParameters, "defense_site_1_latitude_deg", NaN);
siteName = getRunParameterText(runParameters, "defense_site_1_name", "Defense site");
if ~isnan(siteLon) && ~isnan(siteLat)
    plot(siteLon, siteLat, '^', 'MarkerSize', 12, 'MarkerFaceColor', [1.00, 0.85, 0.20], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.2, 'DisplayName', 'Defense site');
    text(siteLon + 0.35, siteLat + 0.15, safeChar(siteName, 'Defense site'), 'FontSize', 12, ...
        'Color', [0.30, 0.20, 0.05], 'FontWeight', 'bold');
end

text(analysis.launch_longitude_deg(1) + 0.35, analysis.launch_latitude_deg(1) - 0.5, ...
    'Launch', 'FontSize', 12, 'Color', [0.05, 0.25, 0.22], 'FontWeight', 'bold');

blockedHeadings = boundary.heading_deg(blockedMask);
blockedLabel = formatBlockedHeadingLabel(blockedHeadings);
annotationText = sprintf(['Blocked headings: %s\n' ...
    'Blocked count: %d / %d\n' ...
    'Max removed range: %.1f km'], ...
    blockedLabel, nnz(blockedMask), height(boundary), ...
    max((boundary.range_m - effective.range_m) ./ 1000.0));
text(min(boundary.longitude_deg) + 0.8, max(boundary.latitude_deg) - 1.1, annotationText, ...
    'FontSize', 12, 'BackgroundColor', [1 1 1 0.82], 'Margin', 8, ...
    'VerticalAlignment', 'top', 'EdgeColor', [0.8 0.8 0.8]);

grid on;
axis equal;
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title({'Defense-Filtered Attack Zone'; ...
    sprintf('Raw boundary vs effective boundary for farthest-point export | Intercepted headings %d/%d', ...
    nnz(blockedMask), height(boundary))});
legend('Location', 'southoutside', 'NumColumns', 3);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14, 'LineWidth', 1.0);

outDir = fullfile(exportDir, 'figures_matlab');
if ~isfolder(outDir)
    mkdir(outDir);
end
outputPath = fullfile(outDir, '04_attack_zone_boundary_defense.png');
exportgraphics(fig, outputPath, 'Resolution', 240);
close(fig);
end

function [rawLon, rawLat, effLon, effLat] = alignBoundaryPolygons(boundary, effective)
rawLon = boundary.longitude_deg(:);
rawLat = boundary.latitude_deg(:);
effLon = rawLon;
effLat = rawLat;

[isMatched, matchIndex] = ismember(boundary.heading_deg(:), effective.heading_deg(:));
effLon(isMatched) = effective.longitude_deg(matchIndex(isMatched));
effLat(isMatched) = effective.latitude_deg(matchIndex(isMatched));
end

function value = getRunParameterNumeric(runParameters, parameterName, defaultValue)
value = defaultValue;
match = string(runParameters.parameter) == string(parameterName);
if ~any(match)
    return;
end

parsedValue = str2double(string(runParameters.value(find(match, 1, 'first'))));
if ~isnan(parsedValue)
    value = parsedValue;
end
end

function value = getRunParameterText(runParameters, parameterName, defaultValue)
value = string(defaultValue);
match = string(runParameters.parameter) == string(parameterName);
if any(match)
    value = string(runParameters.value(find(match, 1, 'first')));
end
end

function textValue = safeChar(value, defaultValue)
if ismissing(value) || strlength(string(value)) == 0
    textValue = char(string(defaultValue));
else
    textValue = char(string(value));
end
end

function label = formatBlockedHeadingLabel(blockedHeadings)
if isempty(blockedHeadings)
    label = 'none';
    return;
end

blockedHeadings = sort(blockedHeadings(:)');
segments = strings(0, 1);
startHeading = blockedHeadings(1);
prevHeading = blockedHeadings(1);
for i = 2:numel(blockedHeadings)
    if abs(blockedHeadings(i) - prevHeading - 10.0) > 1e-6
        segments(end + 1, 1) = formatSegment(startHeading, prevHeading); %#ok<AGROW>
        startHeading = blockedHeadings(i);
    end
    prevHeading = blockedHeadings(i);
end
segments(end + 1, 1) = formatSegment(startHeading, prevHeading); %#ok<AGROW>
label = strjoin(segments, ', ');
end

function textValue = formatSegment(startHeading, endHeading)
if abs(startHeading - endHeading) < 1e-6
    textValue = sprintf('%.0f°', startHeading);
else
    textValue = sprintf('%.0f° to %.0f°', startHeading, endHeading);
end
end
