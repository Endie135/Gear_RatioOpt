clear; clc; close all;

%% 1 Vehicle parameters

% Global 
m   = 854;               % mass        - kg
g   = 9.81;              % gravity     - ms^-2 
rho = 1.225;             % air density - kg/m^3

% Aerodynamics
car.CdA = 0.8;           % drag coefficient area product      - m^2
car.ClA = 1.75;          % downforce coefficient area product - m^2

% Tyres
car.mu_y  = 1.35;        % lateral CoF
car.mu_xd = 1.65;        % deceleration longitudinal CoF (bwd pass)
car.mu_xa = 1.4;         % acceleration longitudinal CoF (fwd pass)
car.Crr   = 0.015;       % rolling resistance

% Powertrain
car.max_power  = 350000; % motor power limit - W
car.max_rpm    = 19500;  % motor rpm redline - rpm
car.max_torque = 350;    % max motor torque  - Nm
car.r_eff      = 0.34;   % tyre radius       - m
car.eta        = 0.95;   % powertrain efficiency

%% 2. Load reconstructed track coordinates & select gear ratio range

loaded = load('track_recon4.csv');
track = loaded;

s = track(:,1);
kappa = track(:,2);
kappa = abs(kappa);

n = length(s);
fprintf('Loaded %d track points\n', n);

% Selecting gear ratio
car.gear_ratio = 7.5;
fprintf('Gear ratio selected: %.2f\n', car.gear_ratio);

%% 3. Pass 1: grip-limited ceiling v_grip(s) using curvature

% use aero limited top speed as limiting by gear ratio is unrealistic
v_lim = (car.max_power / (0.5 * rho * car.CdA))^(1/3);   % aero limited top speed of car
fprintf('Aero-power limited top speed [v_lim] = %.2f m/s (%.1f km/h)\n', v_lim, v_lim * 3.6);

v_grip = zeros(n, 1);
denom = (m * kappa) - (0.5 * rho * car.ClA * car.mu_y);

for i = 1:n
    if denom(i) <= 0
        % Aerodynamically uncapped / pure straightaway limit
        v_grip(i) = v_lim; 
    else
        % Take the smaller of tire cornering limit OR motor redline limit
        v_grip_raw = sqrt((car.mu_y * m * g) / denom(i));
        v_grip(i)  = min(v_grip_raw, v_lim);
    end
end

% Plot v_grip vs distance s
figure('Name', 'Pass 1 - Grip Limited Ceiling');
plot(s, v_grip * 3.6, 'r', 'LineWidth', 1.5);
xlabel('Track Distance s [m]');
ylabel('Grip-Limited Speed v_{grip} [km/h]');
title('Pass 1: Pure Grip Lateral Speed Ceiling');
ylim([0, (v_lim * 3.6) + 20]); % Caps y-axis at limit speed
grid on;

%% 4: Modeling the powertrain and computing F_drive(v) array

% Mechanical redline speed limit for selected gear ratio
v_redline = (car.max_rpm * (2 * pi / 60) * car.r_eff) / car.gear_ratio;

% Base speed from power & torque limits
w_base   = car.max_power / car.max_torque;              % [rad/s]
RPM_base = w_base * (60 / (2 * pi));                    % [rpm]
v_base   = (w_base * car.r_eff) / car.gear_ratio;       % [m/s]

fprintf('Base speed [v_base] = %.2f km/h | Redline speed [v_redline] = %.2f km/h\n', v_base * 3.6, v_redline * 3.6);

% Velocity grid from 0 to redline
v_grid   = (0.1 : 0.1 : v_redline);
num_v   = length(v_grid);
F_drive = zeros(num_v, 1);

% Compute F_drive array across velocity grid
for i = 1:num_v
    v_curr   = v_grid(i);
    F_torque = (car.max_torque * car.gear_ratio * car.eta) / car.r_eff;
    F_power  = (car.max_power * car.eta) / v_curr;

    F_drive(i) = min(F_torque, F_power);
end

% Plot F_drive vs Velocity
figure('Name', 'Powertrain Characteristic');
plot(v_grid * 3.6, F_drive / 1000, 'LineWidth', 1.5);
xlabel('Vehicle Speed (km/h)');
ylabel('Available Drive Force F_{drive} (kN)');
title('Powertrain Force-Speed Envelope');
xline(v_base * 3.6, '--r', 'Base Speed');
xline(v_redline * 3.6, '--k', 'Redline');
xlim([0, (v_redline * 3.6) + (0.15* v_redline)]);
ylim([(min(F_drive)/1000 - 0.2), (max(F_drive)/1000+0.5)]);
grid on;

%% 5. Pass 2: Forward Acceleration Pass (v_fwd) with Closed-Loop Continuity
v_fwd = zeros(n, 1);
ds = s(2) - s(1);

% Initial guess using grip ceiling at start line
v_fwd(1) = v_grip(1);

% Convergence settings for closed circuit boundary condition
max_iter = 10;
tol = 1e-3; % 0.001 m/s tolerance

for iter = 1:max_iter
    v_start_prev = v_fwd(1);
    
    for i = 1:n-1
        v = v_fwd(i);
        
        % Aerodynamic and rolling forces
        F_drag = 0.5 * rho * car.CdA * v^2;
        F_rr   = car.Crr * m * g;
        F_z    = m * g + 0.5 * rho * car.ClA * v^2;
        a_y    = v^2 * kappa(i);
        
        % Friction ellipse check (Acceleration CoF)
        g_eff      = F_z / m; 
        radicand   = 1 - (a_y / (car.mu_xa * g_eff))^2;
        a_x_grip   = car.mu_xa * g_eff * sqrt(max(0,radicand)); % max prevents imaginary 
        F_grip_max = m * a_x_grip;
        
        % Powertrain Drive Force Lookup
        if v >= v_redline
            F_motor = 0;
        else
            F_motor = interp1(v_grid, F_drive, v, 'linear', 'extrap');
            F_motor = max(0, F_motor);
        end

        % Tractive drive limit and net acceleration
        F_drive_actual = min(F_grip_max, F_motor);
        a_x = (F_drive_actual - F_drag - F_rr) / m;

        % Integration to next velocity step
        v_next = sqrt(max(0, v_fwd(i)^2 + 2 * a_x * ds));
        v_fwd(i+1) = min([v_next, v_grip(i+1), v_redline]);

    end
    
    % Pass the final exit velocity back to the start line for closed-circuit continuity
    v_fwd(1) = min(v_fwd(n), v_grip(1));
    
    % Check for convergence
    if abs(v_fwd(1) - v_start_prev) < tol
        fprintf('Forward pass converged in %d iteration(s). Start/finish line speed: %.2f km/h\n', iter, v_fwd(1) * 3.6);
        break;
    end
end

%% 6. Pass 3: Backward Braking Pass (v_bwd)
v_bwd    = zeros(n, 1);
v_bwd(n) = min(v_grip(n), v_redline);

for i = n:-1:2
    v = v_bwd(i);
    
    % Aerodynamic and rolling forces
    F_drag = 0.5 * rho * car.CdA * v^2;
    F_rr   = car.Crr * m * g;
    F_z    = m * g + 0.5 * rho * car.ClA * v^2;
    a_y    = v^2 * kappa(i);
    
    % Friction ellipse check (Deceleration CoF)
    g_eff        = F_z / m;
    radicand     = max(0, 1 - (a_y / (car.mu_xd * g_eff))^2);
    a_x_grip     = car.mu_xd * g_eff * sqrt(radicand);
    F_brake_tire = m * a_x_grip;
    
    % Total retarding force (brakes + drag + rolling resistance)
    F_retarding = F_brake_tire + F_drag + F_rr;
    a_x_brake   = F_retarding / m;
    
    % Backward kinematic integration
    v_prev = sqrt(max(0, v_bwd(i)^2 + 2 * a_x_brake * ds));
    v_bwd(i-1) = min([v_prev, v_grip(i-1), v_redline]);
end

% Plot v_fwd & v_bwd
figure('Name', 'Pass 2 & 3 - Forward Accel & Backward Brake');
plot(s, v_fwd * 3.6, 'b', 'LineWidth', 1.5); hold on;
plot(s, v_bwd * 3.6, 'g', 'LineWidth', 1.5);
xlabel('Track Distance s [m]');
ylabel('Velocity [km/h]');
title('Pass 2 & 3: Forward & Backward Speed Ceiling');
ylim([0, (v_lim * 3.6) + 20]);
legend('Pass 2: Forward Accel','Pass 3: Backward Brake','Location','best');
grid on;

%% 7. Velocity Profile Combination & Lap Time Calculation
% Final velocity profile is the element-wise minimum across all passes
v_final = min([v_grip, v_fwd, v_bwd], [], 2);

% Integrate lap time across track segments
v_avg = (v_final(1:end-1) + v_final(2:end)) / 2;
dt = ds ./ v_avg;
lap_time = seconds(sum(dt));
lap_time.Format = 'mm:ss.SSS';
lap_time_m = string(lap_time);

fprintf('\n========================================\n');
fprintf('  FINAL LAP TIME (Gear Ratio %.2f): %s \n', car.gear_ratio, lap_time_m);
fprintf('========================================\n');

%% 8. Track Sim Plot with Interactive Toggle

% Convert velocities to km/h
v_grip_kmh    = v_grip * 3.6;
v_fwd_kmh     = v_fwd * 3.6;
v_bwd_kmh     = v_bwd * 3.6;
v_final_kmh   = v_final * 3.6;
v_redline_kmh = v_redline * 3.6;

% Custom color palette for readability
c_grip  = [0.85, 0.35, 0.35]; % Muted Red
c_fwd   = [0.20, 0.50, 0.85]; % Soft Blue
c_bwd   = [0.20, 0.70, 0.30]; % Soft Green
c_final = [0.00, 0.00, 0.00]; % Bold Black
c_red   = [0.40, 0.40, 0.40]; % Slate Gray

%% Interactive Single-Overlay Plot with Legend Toggles
fig = figure('Name', 'QSS Lap Time Simulator', 'Position', [150, 150, 1100, 550]);
hold on;

h1 = plot(s, v_grip_kmh,  '--', 'Color', c_grip,  'LineWidth', 1.2, 'DisplayName', 'Pass 1: Grip Ceiling');
h2 = plot(s, v_fwd_kmh,   '-',  'Color', c_fwd,   'LineWidth', 1.2, 'DisplayName', 'Pass 2: Acceleration');
h3 = plot(s, v_bwd_kmh,   '-',  'Color', c_bwd,   'LineWidth', 1.2, 'DisplayName', 'Pass 3: Braking');
h4 = plot(s, v_final_kmh, '-',  'Color', c_final, 'LineWidth', 2.5, 'DisplayName', 'Velocity Profile');
h5 = yline(v_redline_kmh, '--', 'Color', c_red,   'LineWidth', 1.2, 'DisplayName', 'Motor Redline');

title(sprintf('QSS Velocity Profile (Gear Ratio: %.2f | Lap Time: %s)', car.gear_ratio, lap_time_m));
xlabel('Track Distance s [m]');
ylabel('Velocity [km/h]');
xlim([0, max(s)]);
ylim([0, max(v_lim * 3.6, v_redline_kmh) + 15]);
grid on;

% Enable Interactive Legend Toggling
lgd = legend([h1, h2, h3, h4, h5], 'Location', 'northeast');
lgd.ItemHitFcn = @hitcallback;

hold off;

% Callback Function for Interactive Legend Toggle

function hitcallback(~, event)
% Toggle visibility of line clicked in the legend
if strcmp(event.Peer.Visible, 'on')
    event.Peer.Visible = 'off';
else
    event.Peer.Visible = 'on';
end
end