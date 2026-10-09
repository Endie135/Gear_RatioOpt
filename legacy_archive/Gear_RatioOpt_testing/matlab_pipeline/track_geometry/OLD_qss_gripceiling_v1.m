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
s  = [0; cumsum(ds)];

%% 3. Compute curvature along the path

xp  = gradient(x, s);
yp  = gradient(y, s);
xpp = gradient(xp, s);
ypp = gradient(yp, s);

kappa = (xp .* ypp - yp .* xpp) ./ (xp.^2 + yp.^2).^1.5;

fprintf('Curvature stats: min = %.5f, max = %.5f, std = %.5f (1/m)\n', ...
    min(kappa), max(kappa), std(kappa));

%% 4. Vehicle / grip parameters

g       = 9.81;       % gravitational acceleration (m/s^2)
mass    = 854;        % vehicle mass (kg)
mu      = 1.2;        % tyre-road coefficient of friction (-)
CdA_df  = 2;          % downforce area product, i.e. Cl*A lumped (m^2)
rho_air = 1.225;      % air density (kg/m^3)

k_aero = 0.5 * rho_air * CdA_df;   % F_downforce = k_aero * v^2  (N per (m/s)^2)

v_cap = (100);   % m/s (360km/h)

%% 5. v_max(s) using the curvature

R = 1 ./ abs(kappa);
denom = mass - mu .* R .* k_aero;

v_max = nan(n, 1);
grip_limited = denom > 0;
v_max(grip_limited)  = sqrt( (mu .* R(grip_limited) .* mass .* g) ./ denom(grip_limited) );
v_max(~grip_limited) = v_cap;
v_max = min(v_max, v_cap);

N_at_vmax = mass * g + k_aero .* v_max.^2;

fprintf('\nv_max stats: min = %.2f m/s, max = %.2f m/s, mean = %.2f m/s\n', ...
    min(v_max), max(v_max), mean(v_max));

%% 6. Identify corner apexes (local minima of v_max)

% anchor points for forwards and backwards integration

is_apex = islocalmin(v_max);

apex_idx   = find(is_apex);
apex_s     = s(apex_idx);
apex_x     = x(apex_idx);
apex_y     = y(apex_idx);
apex_vmax  = v_max(apex_idx);
apex_kappa = kappa(apex_idx);
apex_R     = R(apex_idx);
apex_N     = N_at_vmax(apex_idx);

apexes = table(apex_idx, apex_s, apex_x, apex_y, apex_kappa, apex_R, apex_vmax, apex_N, ...
    'VariableNames', {'idx', 's', 'x', 'y', 'kappa', 'R', 'v_max', 'N_at_vmax'});

fprintf('Found %d apex points\n', height(apexes));

%% 8. Plot

figure;
plot(s, v_max*3.6, 'b-'); hold on;
plot(apex_s, apex_vmax*3.6, 'ro', 'MarkerFaceColor', 'r');
xlabel('Distance along track, s (m)');
ylabel('Grip-limited v_{max} (km/h)');
title('Grip- and downforce-limited speed profile with apexes');
legend('v_{max}(s)', 'Apex points', 'Location', 'best');
grid on;

figure;
subplot(1, 2, 1);
plot(x, y, 'k-', 'LineWidth', 1.5); hold on;
plot((apex_x), (apex_y), 'r*', 'MarkerSize', 8);
grid on; axis equal;
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('2D Track Layout & Identified Apexes');
grid on;