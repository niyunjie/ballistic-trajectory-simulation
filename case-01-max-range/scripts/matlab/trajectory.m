clc; clear; close all;

%% ===== 1. Load Data =====
data = readtable('optimal-trajectory.csv');

lat = deg2rad(data.latitude_deg);
lon = deg2rad(data.longitude_deg);
h   = data.height_m;

R = 6371000; % Earth radius (m)

%% ===== 2. Convert to ECEF Coordinates =====
x = (R + h) .* cos(lat) .* cos(lon);
y = (R + h) .* cos(lat) .* sin(lon);
z = (R + h) .* sin(lat);

x = x(:); y = y(:); z = z(:);
P = [x y z];

%% ===== 3. Straightness Check =====
dP = diff(P);

angles = zeros(size(dP,1)-1,1);

for i = 1:length(angles)
    v1 = dP(i,:);
    v2 = dP(i+1,:);
    
    cos_val = dot(v1,v2)/(norm(v1)*norm(v2));
    cos_val = max(min(cos_val,1),-1);
    
    angles(i) = acos(cos_val);
end

angles_deg = rad2deg(angles);

fprintf('--- Straightness Analysis ---\n');
fprintf('Mean direction change: %.4f deg\n', mean(angles_deg));
fprintf('Max direction change: %.4f deg\n', max(angles_deg));

%% ===== 4. Great-circle (Coplanarity) Check =====
v1 = P(2,:) - P(1,:);
v2 = P(3,:) - P(1,:);

if norm(cross(v1,v2)) < 1e-6
    error('First three points are collinear.');
end

n = cross(v1, v2);
n = n / norm(n);

dist = abs(P * n');

fprintf('\n--- Great-circle Deviation ---\n');
fprintf('Mean deviation: %.2f m\n', mean(dist));
fprintf('Max deviation: %.2f m\n', max(dist));

%% ===== 5. 3D Trajectory =====
figure;
plot3(x, y, z, 'b-', 'LineWidth', 2); hold on;
plot3(x(1), y(1), z(1), 'go', 'MarkerSize',8,'LineWidth',2);
plot3(x(end), y(end), z(end), 'ro', 'MarkerSize',8,'LineWidth',2);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Ballistic Trajectory (ECEF)');
legend('Trajectory','Start','End','Location','best');
grid on; axis equal;

%% ===== 6. Direction Change Plot =====
figure;
plot(angles_deg, 'LineWidth',1.5);

xlabel('Segment Index');
ylabel('Direction Change (deg)');
title('Direction Change Between Consecutive Segments');
grid on;

%% ===== 7. Deviation from Great Circle =====
figure;
plot(dist/1000, 'LineWidth',1.5);

xlabel('Point Index');
ylabel('Deviation from Great Circle (km)');
title('Deviation from Great-circle Plane');
grid on;