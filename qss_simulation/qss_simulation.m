% QSS_SIMULATION 3D Quasi-Static Lap Time Solver with Rotational Inertia Model
%
% Inputs:
%   car          - Struct with vehicle, motor, tyre, and inertia parameters
%   track        - Nx4 matrix [s, kappa, theta, kappa_z]
%   enable_plots - Boolean (optional): Set to true to display diagnostic plots
%                  (Powertrain Envelope & Interactive Velocity Profile)
%
% Outputs:
%   lap_time_sec - Lap time in seconds
%   results      - Struct containing calculated profiles and properties

function [lap_time_sec, results] = qss_simulation(car, track, enable_plots)
if nargin < 3, enable_plots = false; end

%% 1. Vehicle & Track Setup
m   = car.m;             % Static vehicle mass [kg]
g   = 9.81;              % Gravity [m/s^2]
rho = 1.225;             % Air density [kg/m^3]

s       = track(:,1);
kappa   = abs(track(:,2));
theta   = track(:,3);
kappa_z = track(:,4);

n  = length(s);
ds = s(2) - s(1);

%% 2. Rotational Inertia Model & Effective Mass
% Inertia spinning at wheel-speed: wheels, (driven) gear
% Inertia spinning at motor-speed: motor rotor, pinion
I_wheel_speed = car.I_wheels + car.I_gear;
I_motor_speed = car.I_motor + car.I_pinion;

% Equivalent added mass due to rotational inertia
delta_m = ((car.gear_ratio^2 * I_motor_speed) + I_wheel_speed) / (car.r_eff^2);
m_eff   = m + delta_m;   % Apparent dynamic mass for acceleration/braking passes

%% 3. Pass 1: Grip-Limited Ceiling v_grip(s) using 3D geometry
v_lim = (car.max_power / (0.5 * rho * car.CdA))^(1/3); % Aero top speed ceiling

v_grip    = zeros(n, 1);
numerator = car.mu_y * m * g .* cos(theta);
denom     = (m * kappa) - (0.5 * rho * car.ClA * car.mu_y) - (m * car.mu_y * kappa_z);

for i = 1:n
    if denom(i) <= 0
        v_grip(i) = v_lim; 
    else
        v_grip_raw = sqrt(numerator(i) / denom(i));
        v_grip(i)  = min(v_grip_raw, v_lim);
    end
end

%% 4. Powertrain Drive Force Array F_drive(v)
v_redline = (car.max_rpm * (2 * pi / 60) * car.r_eff) / car.gear_ratio;
w_base    = car.max_power / car.max_torque;
v_base    = (w_base * car.r_eff) / car.gear_ratio;

v_grid   = (0.1 : 0.1 : v_redline);
num_v    = length(v_grid);
F_drive  = zeros(num_v, 1);

for i = 1:num_v
    v_curr   = v_grid(i);
    F_torque = (car.max_torque * car.gear_ratio * car.eta) / car.r_eff;
    F_power  = (car.max_power * car.eta) / v_curr;
    F_drive(i) = min(F_torque, F_power);
end

%% 5. Pass 2: Forward Acceleration Pass (v_fwd)
v_fwd    = zeros(n, 1);
v_fwd(1) = v_grip(1);

max_iter = 10;
tol      = 1e-3;

for iter = 1:max_iter
    v_start_prev = v_fwd(1);
    
    for i = 1:n-1
        v = v_fwd(i);
        
        F_z = m * g * cos(theta(i)) + 0.5 * rho * car.ClA * v^2 + m * v^2 * kappa_z(i);
        F_z = max(0, F_z); 

        F_drag  = 0.5 * rho * car.CdA * v^2;
        F_rr    = car.Crr * F_z; 
        F_slope = m * g * sin(theta(i));
        a_y     = v^2 * kappa(i);
        
        g_eff = F_z / m; 
        if g_eff <= 0
            F_grip_max = 0;
        else
            radicand   = 1 - (a_y / (car.mu_xa * g_eff))^2;
            a_x_grip   = car.mu_xa * g_eff * sqrt(max(0, radicand)); 
            F_grip_max = m * a_x_grip;
        end
        
        if v >= v_redline
            F_motor = 0;
        else
            F_motor = interp1(v_grid, F_drive, v, 'linear', 'extrap');
            F_motor = max(0, F_motor);
        end

        % Acceleration calculated using m_eff (rotational inertia included)
        F_drive_actual = min(F_grip_max, F_motor);
        a_x = (F_drive_actual - F_drag - F_rr - F_slope) / m_eff;

        v_next     = sqrt(max(0, v_fwd(i)^2 + 2 * a_x * ds));
        v_fwd(i+1) = min([v_next, v_grip(i+1), v_redline]);
    end
    
    v_fwd(1) = min(v_fwd(n), v_grip(1));
    if abs(v_fwd(1) - v_start_prev) < tol
        break;
    end
end

%% 6. Pass 3: Backward Braking Pass (v_bwd)
v_bwd    = zeros(n, 1);
v_bwd(n) = min(v_grip(n), v_redline);

for i = n:-1:2
    v = v_bwd(i);
    
    F_z = m * g * cos(theta(i)) + 0.5 * rho * car.ClA * v^2 + m * v^2 * kappa_z(i);
    F_z = max(0, F_z);

    F_drag  = 0.5 * rho * car.CdA * v^2;
    F_rr    = car.Crr * F_z;
    F_slope = m * g * sin(theta(i));
    a_y     = v^2 * kappa(i);
    
    g_eff = F_z / m;
    if g_eff <= 0
        F_brake_tire = 0;
    else
        radicand     = max(0, 1 - (a_y / (car.mu_xd * g_eff))^2);
        a_x_grip     = car.mu_xd * g_eff * sqrt(radicand);
        F_brake_tire = m * a_x_grip;
    end
    
    % Retarding force acts against m_eff
    F_braking = F_brake_tire + F_drag + F_rr + F_slope;
    a_x_brake = F_braking / m_eff;
    
    v_prev     = sqrt(max(0, v_bwd(i)^2 + 2 * a_x_brake * ds));
    v_bwd(i-1) = min([v_prev, v_grip(i-1), v_redline]);
end

%% 7. Combination & Lap Time Calculation
v_final      = min([v_grip, v_fwd, v_bwd], [], 2);
v_avg        = (v_final(1:end-1) + v_final(2:end)) / 2;
dt           = ds ./ v_avg;
lap_time_sec = sum(dt);

% Store output results
results.s         = s;
results.v_grip    = v_grip;
results.v_fwd     = v_fwd;
results.v_bwd     = v_bwd;
results.v_final   = v_final;
results.v_redline = v_redline;
results.v_base    = v_base;
results.m_eff     = m_eff;
results.delta_m   = delta_m;
results.v_grid    = v_grid;
results.F_drive   = F_drive;

%% 8. Optional Plots Controlled by 'enable_plots' Flag
if enable_plots
    % --- FIGURE 1: Powertrain Characteristic Envelope ---
    figure('Name', sprintf('Powertrain Characteristic (Ratio: %.2f)', car.gear_ratio), 'Position', [150, 150, 1100, 550]);
    hold on;
    plot(v_grid * 3.6, F_drive / 1000, 'LineWidth', 1.5);
    xlabel('Vehicle Speed (km/h)');
    ylabel('Available Drive Force F_{drive} (kN)');
    title(sprintf('Powertrain Force-Speed Envelope (Gear Ratio: %.2f)', car.gear_ratio));
    xline(v_base * 3.6, '--r', 'Base Speed');
    xline(v_redline * 3.6, '--k', 'Redline');
    xlim([0, (v_redline * 3.6) + (0.15 * v_redline)]);
    ylim([(min(F_drive)/1000 - 0.2), (max(F_drive)/1000 + 0.5)]);
    grid on;

    % --- FIGURE 2: Interactive Velocity Trace Profile ---
    lap_dur = seconds(lap_time_sec);
    lap_dur.Format = 'mm:ss.SSS';
    lap_time_m = string(lap_dur);

    figure('Name', 'QSS Lap Time Simulator', 'Position', [150, 150, 1100, 550]);
    hold on;

    h1 = plot(s, v_grip * 3.6,  '--', 'Color', [0.85, 0.35, 0.35], 'LineWidth', 1.2, 'DisplayName', 'Pass 1: Grip Ceiling');
    h2 = plot(s, v_fwd * 3.6,   '-',  'Color', [0.20, 0.50, 0.85], 'LineWidth', 1.2, 'DisplayName', 'Pass 2: Acceleration');
    h3 = plot(s, v_bwd * 3.6,   '-',  'Color', [0.20, 0.70, 0.30], 'LineWidth', 1.2, 'DisplayName', 'Pass 3: Braking');
    h4 = plot(s, v_final * 3.6, '-',  'Color', [0.00, 0.00, 0.00], 'LineWidth', 2.5, 'DisplayName', 'Velocity Profile');
    h5 = yline(v_redline * 3.6, '--', 'Color', [0.40, 0.40, 0.40], 'LineWidth', 1.2, 'DisplayName', 'Motor Redline');

    title(sprintf('QSS Velocity Profile (Gear Ratio: %.2f | Lap Time: %s)', car.gear_ratio, lap_time_m));
    xlabel('Track Distance s [m]');
    ylabel('Velocity [km/h]');
    xlim([0, max(s)]);
    ylim([0, max(v_lim * 3.6, v_redline * 3.6) + 15]);
    grid on;

    lgd = legend([h1, h2, h3, h4, h5], 'Location', 'northeast');
    lgd.ItemHitFcn = @hitcallback;
    hold off;
end

end

% Callback function for toggling legend entries
function hitcallback(~, event)
    if strcmp(event.Peer.Visible, 'on')
        event.Peer.Visible = 'off';
    else
        event.Peer.Visible = 'on';
    end
end