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

%% 3a. RAW curvature (kept for comparison plots only)

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
mu      = 1.35;        % tyre-road coefficient of friction (-)
CdA_df  = 1.75;          % downforce area product, i.e. Cl*A lumped (m^2)
rho_air = 1.225;      % air density (kg/m^3)

k_aero = 0.5 * rho_air * CdA_df;   % F_downforce = k_aero * v^2  (N per (m/s)^2)

v_cap = (100);   % m/s (360km/h)

%% 5. v_max(s) using the curvature

R = 1 ./ abs(kappa);
denom = mass - mu .* R .* k_aero;

% initialize v_max as zeros where denom <= 0 (will be set to v_cap later),
% and NaN elsewhere to flag any unexpected untouched entries
v_max = nan(n,1);
grip_limited = denom > 0;
v_max(grip_limited)  = sqrt( (mu .* R(grip_limited) .* mass .* g) ./ denom(grip_limited) );
v_max(~grip_limited) = v_cap;
v_max = min(v_max, v_cap);

N_at_vmax = mass * g + k_aero .* v_max.^2;

fprintf('\nv_max stats: min = %.2f m/s, max = %.2f m/s, mean = %.2f m/s\n', ...
    min(v_max), max(v_max), mean(v_max));

%% 6. Apex detection with prominence + minimum-spacing filtering

min_prominence   = 45;    % m/s - max velocity to be identified as apex
min_apex_spacing_m = 50;  % m - min distance between apex

is_apex_raw = islocalmin(v_max, 'MinProminence', min_prominence);

apex_idx_candidates = find(is_apex_raw);

% Merge candidates that are too close together in s
keep = true(size(apex_idx_candidates));
for i = 1:length(apex_idx_candidates)
    if ~keep(i); continue; end
    si = s(apex_idx_candidates(i));
    for j = i+1:length(apex_idx_candidates)
        if ~keep(j); continue; end
        if abs(s(apex_idx_candidates(j)) - si) < min_apex_spacing_m
            if v_max(apex_idx_candidates(i)) <= v_max(apex_idx_candidates(j))
                keep(j) = false;
            else
                keep(i) = false;
                break;
            end
        end
    end
end

apex_idx = apex_idx_candidates(keep);

apex_s     = s(apex_idx);
apex_x     = x(apex_idx);
apex_y     = y(apex_idx);
apex_vmax  = v_max(apex_idx);
apex_kappa = kappa(apex_idx);
apex_R     = R(apex_idx);
apex_N     = N_at_vmax(apex_idx);

apexes = table(apex_idx, apex_s, apex_x, apex_y, apex_kappa, apex_R, apex_vmax, apex_N, ...
    'VariableNames', {'idx', 's', 'x', 'y', 'kappa', 'R', 'v_max', 'N_at_vmax'});

fprintf('Found %d apex points after prominence/spacing filtering\n', height(apexes));

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