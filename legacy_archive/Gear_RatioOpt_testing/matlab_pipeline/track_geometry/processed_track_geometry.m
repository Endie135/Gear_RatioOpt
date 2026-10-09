clear; clc; close all;

%% 1. Load processed track coordinates
loaded = load('processed_track.csv');
track = loaded;

x = track(:,1);
y = track(:,2);
n = length(x);

fprintf('Loaded %d track points\n', n);

%% 2. Compute distance along the path

dx = diff(x);
dy = diff(y);
ds = sqrt(dx.^2 + dy.^2);
s  = [0; cumsum(ds)];       % cumulative distance travelled, s(1) = 0

fprintf('Total track distance: %.1f m\n', s(end));
fprintf('Min/mean/max point spacing: %.3f / %.3f / %.3f m\n', ...
    min(ds), mean(ds), max(ds));

%% 3. Compute curvature along the path

xp  = gradient(x, s);
yp  = gradient(y, s);
xpp = gradient(xp, s);
ypp = gradient(yp, s);

kappa = (xp .* ypp - yp .* xpp) ./ (xp.^2 + yp.^2).^1.5;

fprintf('Curvature stats: min = %.5f, max = %.5f, std = %.5f (1/m)\n', ...
    min(abs(kappa)), max(abs(kappa)), std(abs(kappa)));

%% 4. Plot: track shape colored by curvature
figure('Position', [100 100 750 700]);

% Plot the underlying track line in a light color
plot(x, y, 'Color', [0.8 0.85 0.9], 'LineWidth', 1.5); 
hold on;

% Plot scattered points colored by curvature magnitude
step = 2; 
scatter(x(1:step:end), y(1:step:end), 15, abs(kappa(1:step:end)), 'filled');

axis equal; grid on;
colormap(parula);
c = colorbar;
c.Label.String = 'Curvature [1/m]';

xlabel('X [m]'); 
ylabel('Y [m]');
title('Processed Telemetry Path');

%% 5. Plot: curvature vs distance
figure('Position', [150 150 900 400]);
plot(s, kappa, 'r-', 'LineWidth', 1.2);
yline(0, 'k--');
grid on;
xlabel('Distance travelled, s (m)');
ylabel('Curvature, \kappa (1/m)');
title('Curvature vs distance along the processed track data');