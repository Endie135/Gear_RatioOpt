clear; clc; close all;

%% 1. Base Vehicle & Global Parameters
car_base.m           = 854.0;    % Total vehicle static mass [kg] (KEPT CONSTANT VIA BALLASTING)

car_base.CdA         = 0.8;      % Aero drag area product [m^2]
car_base.ClA         = 1.75;     % Aero downforce area product [m^2]

car_base.mu_y        = 1.35;     % Lateral CoF
car_base.mu_xd       = 1.65;     % Braking CoF
car_base.mu_xa       = 1.4;      % Acceleration CoF
car_base.Crr         = 0.015;    % Rolling resistance

car_base.max_power    = 350000;   % Motor power limit [W]
car_base.max_rpm      = 19500;    % Redline RPM
car_base.max_torque   = 350;      % Max motor torque [Nm]
car_base.r_eff        = 0.34;     % Tyre effective rolling radius [m]
car_base.eta          = 0.95;     % Drivetrain mechanical efficiency

% Constant Non-Gearbox Inertias
car_base.I_wheels    = 1.20;     % Combined wheel assembly inertia [kg*m^2]
car_base.I_motor     = 0.025;    % Motor rotor inertia [kg*m^2]

% Overall 2-Stage Drivetrain Gear Ratios
car_base.G1          = 3.4;      % Stage 1 Reduction Ratio (Motor -> Intermediate)
car_base.G2          = 3.0;      % Stage 2 Reduction Ratio (Intermediate -> Wheel)
car_base.gear_ratio  = car_base.G1 * car_base.G2; % Total Gear Ratio (10.2:1)

%% 2. CAD Component Parameters (Inputs in g and g*mm^2, converted to kg and kg*m^2)

% --- SET A: UNOPTIMISED ASSEMBLY ---
unopt.m_input_assembly = 1271.551 / 1000;          % [g] -> [kg]
unopt.I_input_assembly = 4662000 * 1e-9;        % [g*mm^2] -> [kg*m^2]

unopt.m_counter_shaft  = 987.513 / 1000;           % [g] -> [kg]
unopt.I_counter_shaft  = 1978000 * 1e-9;        % [g*mm^2] -> [kg*m^2]

unopt.m_gear_n2        = 7520.253 / 1000;          % [g] -> [kg]
unopt.I_gear_n2        = 20500000 * 1e-9;       % [g*mm^2] -> [kg*m^2] (Counter shaft gear)

unopt.m_pinion_n3      = 2775.615 / 1000;           % [g] -> [kg]
unopt.I_pinion_n3      = 2487000 * 1e-9;        % [g*mm^2] -> [kg*m^2] (Counter shaft pinion)

unopt.m_output_shaft   = 1738.218 / 1000;          % [g] -> [kg]
unopt.I_output_shaft   = 7927000 * 1e-9;       % [g*mm^2] -> [kg*m^2]

unopt.m_gear_n4        = 27294.997 / 1000;          % [g] -> [kg]
unopt.I_gear_n4        = 161100000 * 1e-9;       % [g*mm^2] -> [kg*m^2] (Output shaft gear)

% --- SET B: FEA MASS-OPTIMISED ASSEMBLY ---
opt.m_input_assembly = 1271.551 / 1000;          % [g] -> [kg]
opt.I_input_assembly = 4662000 * 1e-9;        % [g*mm^2] -> [kg*m^2]

opt.m_counter_shaft  = 987.513 / 1000;           % [g] -> [kg]
opt.I_counter_shaft  = 1978000 * 1e-9;        % [g*mm^2] -> [kg*m^2]

opt.m_gear_n2        = 5283.682 / 1000;          % [g] -> [kg]
opt.I_gear_n2        = 13520000 * 1e-9;       % [g*mm^2] -> [kg*m^2] (Counter shaft gear)

opt.m_pinion_n3      = 2775.615 / 1000;           % [g] -> [kg]
opt.I_pinion_n3      = 2487000 * 1e-9;        % [g*mm^2] -> [kg*m^2] (Counter shaft pinion)

opt.m_output_shaft   = 1738.218 / 1000;          % [g] -> [kg]
opt.I_output_shaft   = 7927000 * 1e-9;       % [g*mm^2] -> [kg*m^2]

opt.m_gear_n4        = 16498.618 / 1000;          % [g] -> [kg]
opt.I_gear_n4        = 103500000 * 1e-9;       % [g*mm^2] -> [kg*m^2] (Output shaft gear)

%% 3. Combine Drivetrain Totals & Group Shaft Speed Inertias

% --- Total Drivetrain Component Masses [kg] ---
m_drivetrain_unopt = unopt.m_input_assembly + unopt.m_counter_shaft + ...
    unopt.m_gear_n2 + unopt.m_pinion_n3 + ...
    unopt.m_output_shaft + unopt.m_gear_n4;

m_drivetrain_opt   = opt.m_input_assembly + opt.m_counter_shaft + ...
    opt.m_gear_n2 + opt.m_pinion_n3 + ...
    opt.m_output_shaft + opt.m_gear_n4;

% --- Total Unoptimised Speed Group Inertias [kg*m^2] ---
I_input_shaft_total_unopt  = unopt.I_input_assembly;
I_counter_shaft_total_unopt= unopt.I_counter_shaft + unopt.I_gear_n2 + unopt.I_pinion_n3;
I_output_shaft_total_unopt = unopt.I_output_shaft + unopt.I_gear_n4;

I_drivetrain_unopt = I_input_shaft_total_unopt + ...
    I_counter_shaft_total_unopt + ...
    I_output_shaft_total_unopt;

% --- Total Optimised Speed Group Inertias [kg*m^2] ---
I_input_shaft_total_opt  = opt.I_input_assembly;
I_counter_shaft_total_opt= opt.I_counter_shaft + opt.I_gear_n2 + opt.I_pinion_n3;
I_output_shaft_total_opt = opt.I_output_shaft + opt.I_gear_n4;

I_drivetrain_opt   = I_input_shaft_total_opt + ...
    I_counter_shaft_total_opt + ...
    I_output_shaft_total_opt;

%% 4. Build Vehicle Parameter Structs for QSS Simulation

% Unoptimised Struct
car_unopt                        = car_base;
car_unopt.I_input_shaft_total    = I_input_shaft_total_unopt;
car_unopt.I_counter_shaft_total  = I_counter_shaft_total_unopt;
car_unopt.I_output_shaft_total   = I_output_shaft_total_unopt;

% Optimised Struct
car_opt                          = car_base;
car_opt.I_input_shaft_total      = I_input_shaft_total_opt;
car_opt.I_counter_shaft_total    = I_counter_shaft_total_opt;
car_opt.I_output_shaft_total     = I_output_shaft_total_opt;


%% 5. Load Track Data & Run QSS Simulations
script_dir = fileparts(mfilename('fullpath'));
data_file  = fullfile(script_dir, '..', 'data', 'track_recon.csv');

if exist(data_file, 'file') == 2
    track = load(data_file);
    fprintf('Loaded %d reconstructed track points from %s\n', length(track), data_file);
else
    error('File not found: %s\nEnsure the Python and/or MATLAB track reconstruction script ran first.', data_file);
end

% Execute QSS Simulation Solver (Internal QSS plots disabled)
[t_unopt, res_unopt] = qss_simulation_comparison(car_unopt, track, false);
[t_opt,   res_opt]   = qss_simulation_comparison(car_opt,   track, false);


%% 6. Format & Print Comparative Performance Results
% Component-Specific Calculations (N2 + N4 Gears)
m_gears_unopt = unopt.m_gear_n2 + unopt.m_gear_n4;
m_gears_opt   = opt.m_gear_n2   + opt.m_gear_n4;
delta_m_gears = m_gears_unopt - m_gears_opt; 

I_gears_unopt = unopt.I_gear_n2 + unopt.I_gear_n4;
I_gears_opt   = opt.I_gear_n2   + opt.I_gear_n4;

% Laptime formatting
dur_unopt = seconds(t_unopt); dur_unopt.Format = 'mm:ss.SSS';
dur_opt   = seconds(t_opt);   dur_opt.Format   = 'mm:ss.SSS';
delta_t   = t_unopt - t_opt;
pct_t_gain = (delta_t / t_unopt) * 100;

% Energy Calculations
delta_E_elec_J   = res_unopt.E_elec_joules - res_opt.E_elec_joules;  % Joules saved
delta_E_elec_kJ  = delta_E_elec_J / 1000;                            % kJ saved
pct_E_gain       = (delta_E_elec_J / res_unopt.E_elec_joules) * 100;  % % saved

fprintf('\n========================================================================\n');
fprintf('        CAD ROTATIONAL INERTIA OPTIMISATION COMPARISON RESULTS\n');
fprintf('   (Vehicle Static Mass Constant at %.1f kg via Ballasting)\n', car_base.m);
fprintf('========================================================================\n');
fprintf('Parameter                           Unoptimised     Optimised     Delta (%%)\n');
fprintf('------------------------------------------------------------------------\n');
fprintf('Dynamic Gearbox Mass (kg)           %10.3f    %10.3f     %+.2f %%\n', ...
    m_drivetrain_unopt, m_drivetrain_opt, ...
    ((m_drivetrain_opt - m_drivetrain_unopt)/m_drivetrain_unopt)*100);
fprintf('Dynamic Gearbox Inertia (kg*m^2)    %10.6f    %10.6f     %+.2f %%\n', ...
    I_drivetrain_unopt, I_drivetrain_opt, ...
    ((I_drivetrain_opt - I_drivetrain_unopt)/I_drivetrain_unopt)*100);
fprintf('N2 + N4 Gears Mass (kg)             %10.3f    %10.3f     %+.2f %%\n', ...
    m_gears_unopt, m_gears_opt, ...
    ((m_gears_opt - m_gears_unopt)/m_gears_unopt)*100);
fprintf('N2 + N4 Gear Inertia (kg*m^2)       %10.6f    %10.6f     %+.2f %%\n', ...
    I_gears_unopt, I_gears_opt, ...
    ((I_gears_opt - I_gears_unopt)/I_gears_unopt)*100);
fprintf('Dynamic Added Mass delta_m (kg)     %10.3f    %10.3f     %+.2f %%\n', ...
    res_unopt.delta_m, res_opt.delta_m, ...
    ((res_opt.delta_m - res_unopt.delta_m)/res_unopt.delta_m)*100);
fprintf('Total Apparent Mass m_eff (kg)      %10.3f    %10.3f     %+.2f %%\n', ...
    res_unopt.m_eff, res_opt.m_eff, ...
    ((res_opt.m_eff - res_unopt.m_eff)/res_unopt.m_eff)*100);
fprintf('------------------------------------------------------------------------\n');
fprintf('Unoptimised Lap Time                : %s (%.3fs)\n', string(dur_unopt), t_unopt);
fprintf('FEA Optimised Lap Time              : %s (%.3fs)\n', string(dur_opt), t_opt);
fprintf('Lap Time Difference (Time Saved)    : -%.3f s (-%.2f %%)\n', delta_t, pct_t_gain);
fprintf('Electrical Energy Consumed (kJ)     : %10.2f kJ  %10.2f kJ  [Saved: -%.2f kJ / -%.2f %%]\n', ...
    res_unopt.E_elec_joules/1000, res_opt.E_elec_joules/1000, delta_E_elec_kJ, pct_E_gain);
fprintf('Electrical Energy Consumed (kWh)    : %10.4f kWh %10.4f kWh\n', ...
    res_unopt.E_elec_kWh, res_opt.E_elec_kWh);
fprintf('========================================================================\n');

%% 7. FEA Gear Lightweighting Efficiency & Sensitivity Metrics

% Rotational Dynamic Mass Return Ratio: (kg dynamic mass saved / kg gear mass removed)
dyn_mass_return_ratio = (res_unopt.delta_m - res_opt.delta_m) / delta_m_gears;

% Specific Energy ROI: (kJ saved per lap / kg gear mass removed)
specific_energy_roi   = delta_E_elec_kJ / delta_m_gears;

% Specific Lap Time Sensitivity: (ms saved per lap / kg gear mass removed)
specific_time_sens    = (delta_t * 1000) / delta_m_gears;

fprintf('\n');
fprintf('      FEA LIGHTWEIGHTING EFFICIENCY & SENSITIVITY METRICS (N2 & N4)\n');
fprintf('Physical Gear Mass Saved (N2 + N4)  : %8.3f kg\n', delta_m_gears);
fprintf('Rotational Dynamic Mass Return Ratio: %8.3f kg dynamic saved / kg gear removed\n', dyn_mass_return_ratio);
fprintf('Specific Energy ROI (per lap)       : %8.3f kJ saved / kg gear removed\n', specific_energy_roi);
fprintf('Specific Lap Time Sensitivity       : %8.3f ms saved / kg gear removed\n', specific_time_sens);


%% 8. Diagnostic Plots & Detailed QSS Velocity Trace

% --- FIGURE 1: FEA Optimised CAD QSS Multi-Pass Velocity Profile ---
dur_opt_plot = seconds(t_opt); 
dur_opt_plot.Format = 'mm:ss.SSS';

figure('Name', 'Optimised CAD QSS Lap Trace', 'Position', [100, 100, 1100, 550]);
hold on;

h1 = plot(res_opt.s, res_opt.v_grip * 3.6,  '--', 'Color', [0.85, 0.35, 0.35], 'LineWidth', 1.2, 'DisplayName', 'Pass 1: Grip Ceiling');
h2 = plot(res_opt.s, res_opt.v_fwd * 3.6,   '-',  'Color', [0.20, 0.50, 0.85], 'LineWidth', 1.2, 'DisplayName', 'Pass 2: Acceleration');
h3 = plot(res_opt.s, res_opt.v_bwd * 3.6,   '-',  'Color', [0.20, 0.70, 0.30], 'LineWidth', 1.2, 'DisplayName', 'Pass 3: Braking');
h4 = plot(res_opt.s, res_opt.v_final * 3.6, '-',  'Color', [0.00, 0.00, 0.00], 'LineWidth', 2.2, 'DisplayName', 'Final Velocity Profile');
h5 = yline(res_opt.v_redline * 3.6,        '--', 'Color', [0.40, 0.40, 0.40], 'LineWidth', 1.2, 'DisplayName', 'Motor Redline');

title(sprintf('FEA Optimised CAD Velocity Profile (Ratio: %.2f:1 | Lap Time: %s)', ...
    car_base.gear_ratio, string(dur_opt_plot)));
xlabel('Track Distance s [m]');
ylabel('Velocity [km/h]');
xlim([0, max(res_opt.s)]);
ylim([0, max(max(res_opt.v_grip * 3.6), res_opt.v_redline * 3.6) + 15]);
grid on;

lgd = legend([h1, h2, h3, h4, h5], 'Location', 'northeast');
lgd.ItemHitFcn = @hitcallback;
hold off;


% --- FIGURE 2: Unoptimised vs. Optimised Comparison Dashboard ---
figure('Name', 'CAD Inertia Optimisation Performance Comparison', 'Position', [150, 150, 1100, 650]);

% Subplot 1: Velocity Profiles Overlay
subplot(2, 1, 1);
plot(res_unopt.s, res_unopt.v_final * 3.6, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Unoptimised CAD'); hold on;
plot(res_opt.s,   res_opt.v_final * 3.6,   'b-',  'LineWidth', 1.8, 'DisplayName', 'FEA Optimised CAD');
title(sprintf('Velocity Profile Overlay (Total Gear Ratio: %.2f:1)', car_base.gear_ratio));
xlabel('Track Distance s [m]'); ylabel('Velocity [km/h]');
legend('Location', 'best'); grid on;

% Subplot 2: Cumulative Time Saved [ms] along Track
subplot(2, 1, 2);
ds = res_unopt.s(2) - res_unopt.s(1);
dt_unopt = ds ./ ((res_unopt.v_final(1:end-1) + res_unopt.v_final(2:end))/2);
dt_opt   = ds ./ ((res_opt.v_final(1:end-1)   + res_opt.v_final(2:end))/2);

time_gain_ms_cum = [0; cumsum(dt_unopt - dt_opt)] * 1000; % Cumulative ms saved

plot(res_unopt.s, time_gain_ms_cum, 'g-', 'LineWidth', 1.8);
title('Cumulative Lap Time Saved along Track Position (Rotational Inertia Only)');
xlabel('Track Distance s [m]'); ylabel('Time Saved [ms]');
grid on;

%% 9. Final QSS Simulation Summary Output
dur_unopt_final = seconds(t_unopt); dur_unopt_final.Format = 'mm:ss.SSS';
dur_opt_final   = seconds(t_opt);   dur_opt_final.Format   = 'mm:ss.SSS';

top_speed_unopt = max(res_unopt.v_final) * 3.6; % [km/h]
top_speed_opt   = max(res_opt.v_final)   * 3.6; % [km/h]

% Laptime formatting
dur_unopt = seconds(t_unopt); dur_unopt.Format = 'mm:ss.SSS';
dur_opt   = seconds(t_opt);   dur_opt.Format   = 'mm:ss.SSS';
delta_t   = t_unopt - t_opt;
pct_t_gain = (delta_t / t_unopt) * 100;

fprintf('\n========================================================================\n');
fprintf('                   FINAL QSS SIMULATION OVERVIEW\n');
fprintf('Configuration        Lap Time (mm:ss.SSS)   Lap Time (s)   Top Speed (km/h)\n');
fprintf('------------------------------------------------------------------------\n');
fprintf('Unoptimised CAD         %15s    %10.3f s       %8.2f km/h\n', ...
    string(dur_unopt_final), t_unopt, top_speed_unopt);
fprintf('FEA Optimised CAD       %15s    %10.3f s       %8.2f km/h\n', ...
    string(dur_opt_final), t_opt, top_speed_opt);
fprintf('------------------------------------------------------------------------\n');
fprintf('Net Improvement         %15.3f s saved           %+0.2f km/h\n', ...
    (t_unopt - t_opt), (top_speed_opt - top_speed_unopt));
fprintf('========================================================================\n');


%%

% Callback function for toggling legend entries in diagnostic plots
function hitcallback(~, event)
if strcmp(event.Peer.Visible, 'on')
    event.Peer.Visible = 'off';
else
    event.Peer.Visible = 'on';
end
end