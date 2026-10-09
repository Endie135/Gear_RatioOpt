clear; clc; close all;

%% 1. Vehicle Parameters
car.m   = 854;               % Static vehicle mass [kg]
g       = 9.81;
rho     = 1.225;

car.CdA = 0.8;               % Aero drag area [m^2]
car.ClA = 1.75;              % Aero downforce area [m^2]

car.mu_y  = 1.35;            % Lateral CoF
car.mu_xd = 1.65;            % Braking CoF
car.mu_xa = 1.4;             % Accel CoF
car.Crr   = 0.015;           % Rolling resistance

car.max_power  = 350000;     % Motor power limit [W]
car.max_rpm    = 19500;      % Redline RPM
car.max_torque = 350;        % Max motor torque [Nm]
car.r_eff      = 0.34;       % Tyre effective radius [m]
car.eta        = 0.95;       % Drivetrain efficiency

%% 2. Gear Rotational Inertia Model Baseline Parameters
car.I_wheels = 1.20;         % Combined wheel assembly inertia [kg*m^2]
car.I_motor  = 0.025;        % Motor rotor inertia [kg*m^2]

% Reference gear pair inertias at baseline ratio G_ref
car.G_ref        = 7.5;
car.I_pinion_ref = 0.0005;    % Pinion inertia at baseline [kg*m^2]
car.I_gear_ref   = 0.0080;    % Driven gear inertia at baseline [kg*m^2]

%% 3. Load Track Coordinates
script_dir = fileparts(mfilename('fullpath'));
data_file  = fullfile(script_dir, '..', 'data', 'track_recon.csv');

if exist(data_file, 'file') == 2
    track = load(data_file);
    fprintf('Loaded %d reconstructed track points from %s\n', length(track), data_file);
else
    error('File not found: %s\nEnsure the Python and/or MATLAB track reconstruction script ran first.', data_file);
end

%% 4. Gear Ratio Sweep Configuration
gear_ratios = (4.0 : 0.2 : 14.0);
num_ratios  = length(gear_ratios);

lap_times        = zeros(num_ratios, 1);
effective_masses = zeros(num_ratios, 1);
top_speeds       = zeros(num_ratios, 1);
all_results      = cell(num_ratios, 1);

fprintf('Running QSS optimisation sweep across %d gear ratios...\n', num_ratios);

for i = 1:num_ratios
    G = gear_ratios(i);
    car.gear_ratio = G;
    
    % Scale gear inertias parametrically relative to baseline ratio
    car.I_pinion = car.I_pinion_ref * (car.G_ref / G)^2;
    car.I_gear   = car.I_gear_ref   * (G / car.G_ref)^4;
    
    % Call QSS simulation with enable_plots set to false during sweep
    [lap_times(i), res] = qss_simulation(car, track, false);
    
    effective_masses(i) = res.m_eff;
    top_speeds(i)       = max(res.v_final) * 3.6;
    all_results{i}      = res;
end

%% 5. Optimisation Results Analysis
[min_lap_time, min_idx] = min(lap_times);
opt_gear_ratio          = gear_ratios(min_idx);

opt_duration = seconds(min_lap_time);
opt_duration.Format = 'mm:ss.SSS';

fprintf('\n======================================================\n');
fprintf('  OPTIMISATION COMPLETE\n');
fprintf('  Optimal Gear Ratio : %.2f\n', opt_gear_ratio);
fprintf('  Best Lap Time      : %s (%.3f s)\n', string(opt_duration), min_lap_time);
fprintf('  Effective Mass     : %.2f kg (+%.2f kg dynamic inertia)\n', ...
        effective_masses(min_idx), all_results{min_idx}.delta_m);
fprintf('======================================================\n');

%% 6. Macro Optimisation Dashboard Plot
figure('Name', 'Gear Ratio Sweep Dashboard', 'Position', [100, 100, 1100, 700]);

% Panel 1: Lap Time vs Gear Ratio
subplot(2, 2, 1);
plot(gear_ratios, lap_times, 'b-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b'); hold on;
plot(opt_gear_ratio, min_lap_time, 'r*', 'MarkerSize', 12, 'LineWidth', 2);
title('Lap Time vs Gear Ratio');
xlabel('Gear Ratio'); ylabel('Lap Time [s]');
grid on; legend('Sweep Data', sprintf('Optimal (G=%.2f)', opt_gear_ratio), 'Location', 'northeast');

% Panel 2: Effective Dynamic Mass Penalty
subplot(2, 2, 2);
plot(gear_ratios, effective_masses, 'm-s', 'LineWidth', 1.5, 'MarkerFaceColor', 'm');
title('Apparent Dynamic Mass (m + \Delta m)');
xlabel('Gear Ratio'); ylabel('Effective Mass [kg]');
grid on;

% Panel 3: Speed Limits vs Gear Ratio
subplot(2, 2, 3);
redlines = zeros(num_ratios, 1);
for i = 1:num_ratios, redlines(i) = all_results{i}.v_redline * 3.6; end
plot(gear_ratios, top_speeds, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Achieved Top Speed'); hold on;
plot(gear_ratios, redlines, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Motor Redline');
title('Speed Envelope vs Gear Ratio');
xlabel('Gear Ratio'); ylabel('Speed [km/h]');
legend('Location', 'northeast'); grid on;

% Panel 4: Velocity Traces for Min, Opt, and Max Gear Ratios
subplot(2, 2, 4);
idx_min = 1;
idx_opt = min_idx;
idx_max = num_ratios;
s_track = all_results{1}.s;

plot(s_track, all_results{idx_min}.v_final * 3.6, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2, 'DisplayName', sprintf('Min G=%.1f', gear_ratios(idx_min))); hold on;
plot(s_track, all_results{idx_opt}.v_final * 3.6, 'b-', 'LineWidth', 2.0, 'DisplayName', sprintf('Opt G=%.1f', gear_ratios(idx_opt)));
plot(s_track, all_results{idx_max}.v_final * 3.6, 'r--', 'LineWidth', 1.2, 'DisplayName', sprintf('Max G=%.1f', gear_ratios(idx_max)));
title('Velocity Trace Comparison');
xlabel('Track Distance s [m]'); ylabel('Speed [km/h]');
legend('Location', 'best'); grid on;

%% 7. Run Single Simulation for Optimal Ratio & Display Both Plots
car.gear_ratio = opt_gear_ratio;
car.I_pinion   = car.I_pinion_ref * (car.G_ref / opt_gear_ratio)^2;
car.I_gear     = car.I_gear_ref   * (opt_gear_ratio / car.G_ref)^4;

% Passing enable_plots = true generates both figures for the optimal ratio
qss_simulation(car, track, true);