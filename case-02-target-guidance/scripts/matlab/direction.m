clc; clear; close all;

% ===== user input =====
theta0_deg = 10;   % velocity elevation angle
psi_v0_deg = 270;    % heading angle: 0=N, 90=E, 180=S, 270=W

% ===== direction vector =====
theta = deg2rad(theta0_deg);
psi   = deg2rad(psi_v0_deg);

% local ENU frame:
% x -> East, y -> North, z -> Up
vx = cos(theta) * sin(psi);   % East
vy = cos(theta) * cos(psi);   % North
vz = sin(theta);              % Up

% ===== plot =====
figure('Color','w');
hold on; grid on; axis equal;

% axes
quiver3(0,0,0,1,0,0,0,'r','LineWidth',1.8,'MaxHeadSize',0.4); % East
quiver3(0,0,0,0,1,0,0,'g','LineWidth',1.8,'MaxHeadSize',0.4); % North
quiver3(0,0,0,0,0,1,0,'b','LineWidth',1.8,'MaxHeadSize',0.4); % Up

text(1.05,0,0,'East','Color','r','FontSize',11);
text(0,1.05,0,'North','Color','g','FontSize',11);
text(0,0,1.05,'Up','Color','b','FontSize',11);

% missile initial direction
quiver3(0,0,0,vx,vy,vz,0,'k','LineWidth',3,'MaxHeadSize',0.5);
text(vx*1.08, vy*1.08, vz*1.08, 'Initial velocity', 'FontSize', 11);

xlabel('East');
ylabel('North');
zlabel('Up');
title(sprintf('\\theta_0 = %.1f^\\circ, \\psi_{v0} = %.1f^\\circ', theta0_deg, psi_v0_deg));

xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
zlim([-0.2 1.2]);
view(45,25);

% ===== printed explanation =====
fprintf('theta0 = %.2f deg\n', theta0_deg);
fprintf('psi_v0 = %.2f deg\n', psi_v0_deg);
fprintf('Direction vector in ENU = [%.4f, %.4f, %.4f]\n', vx, vy, vz);

if psi_v0_deg < 0
    psi_show = mod(psi_v0_deg, 360);
else
    psi_show = mod(psi_v0_deg, 360);
end
fprintf('Ground heading: %.2f deg clockwise from North\n', psi_show);
