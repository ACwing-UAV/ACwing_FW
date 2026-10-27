function [offset, radius] = fitHardIron(logFile)
%FITHARDIRON  Magnetometer hard-iron offset (least-squares sphere fit).
%
%   [offset, radius] = fitHardIron('acwing_log_....mat')
%
%   Procedure: firmware with MAG_HARD_IRON = {0,0,0}, start acwingLiveViewer, then
%   slowly rotate the board in ALL directions for ~30 s (figure-eight on several
%   planes). The log holds the field already in the BODY frame (MAG_AXIS_MAP
%   applied), so the offset can be copied as-is into MAG_HARD_IRON (app_config.h).
%   calibMagAxes.m estimates axis map AND hard-iron together and is preferred.

S = load(logFile); m = S.L.mag;
m = m(:, all(isfinite(m), 1));
N = size(m, 2);
A = [2*m.', ones(N,1)];
b = sum(m.^2, 1).';
p = A \ b;
offset = p(1:3);
radius = sqrt(p(4) + sum(offset.^2));

res = vecnorm(m - offset) - radius;
span = max(m, [], 2) - min(m, [], 2);
fprintf('hard-iron offset [uT]: %.2f %.2f %.2f   |B| = %.1f uT   residual std %.2f uT\n', ...
        offset, radius, std(res));
if any(span < radius)
    warning(['Poor angular coverage (per-axis span %.0f/%.0f/%.0f uT < 2|B|): ' ...
             'rotate the board in more directions.'], span);
end
fprintf('\nIn app_config.h:\n#define MAG_HARD_IRON  { %.2ff, %.2ff, %.2ff }\n', offset);

figure('Name', 'Fit hard-iron', 'Color', 'w');
plot3(m(1,:), m(2,:), m(3,:), '.', 'MarkerSize', 4); hold on; grid on; axis equal;
[xs, ys, zs] = sphere(24);
surf(radius*xs + offset(1), radius*ys + offset(2), radius*zs + offset(3), ...
     'FaceAlpha', 0.08, 'EdgeAlpha', 0.2);
xlabel('x_b [uT]'); ylabel('y_b [uT]'); zlabel('z_b [uT]');
title('Magnetometer samples and fitted sphere');
end
