clear; clc; close all;

%% 1. Load track coordinates

loaded = load('processed_trackZ2.csv');
track = loaded;

x       = track(:,1);
y       = track(:,2);
z       = track(:,3);
s       = track(:,4);
kappa   = track(:,5);
theta   = track(:,6);
kappa_z = track(:,7);

n = length(x);
abs_kappa = abs(kappa);

fprintf('Loaded %d track points\n', n);

%% 2. Section Definitions

% Define track sections along s-coordinate.
% Edit/add section parameters to fine-tune specific regions of the track.
sections = struct(...
    's_start',   {0,    1040, 1430, 2000, 2250, 2410, 2725, 2812}, ... % start s distance (m)
    's_end',     {1040, 1430, 2000, 2250, 2410, 2725, 2812, s(end)}, ... % end s distance (m)
    'N_corners', {3,    4,    0,    3,    1,    4,    1,    3}, ...    % corner search target for section
    'corner_min',{0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10} ... % min curvature fraction threshold
);

N_sections = length(sections); 

%% 3. Section-by-Section Corner Extraction

corners = struct('apex_entry', {}, 'corner_apex', {}, 'apex_exit', {}, ...
                  'R', {}, 's_apex', {}, 'idx_entry', {}, 'idx_apex', {}, 'idx_exit', {}, ...
                  'direction', {}); 

total_corners_found = 0; 

for sec = 1:N_sections
    s_start_sec = sections(sec).s_start; 
    s_end_sec   = sections(sec).s_end; 
    N_corners_sec  = sections(sec).N_corners; 
    corner_min_sec = sections(sec).corner_min; 
    
    % Mask for points within this s section boundary
    sec_mask = (s >= s_start_sec) & (s <= s_end_sec); 
    available = sec_mask; 
    
    kappa_work = abs_kappa; 
    count_sec = 0; 
    
    while count_sec < N_corners_sec && any(available)
        kappa_work(~available) = -Inf; 
        [best_val, i_apex] = max(kappa_work); 
        if best_val <= 0
            break;
        end

        kappa_apex = abs_kappa(i_apex); 
        threshold  = corner_min_sec * kappa_apex; 

        % Find entry point within section bounds
        i_entry = i_apex; 
        while i_entry > 1 && abs_kappa(i_entry) > threshold && s(i_entry) >= s_start_sec
            i_entry = i_entry - 1;
        end

        % Find exit point within section bounds
        i_exit = i_apex; 
        while i_exit < n && abs_kappa(i_exit) > threshold && s(i_exit) <= s_end_sec
            i_exit = i_exit + 1;
        end

        total_corners_found = total_corners_found + 1; 
        count_sec = count_sec + 1; 

        corners(total_corners_found).idx_entry   = i_entry; 
        corners(total_corners_found).idx_apex    = i_apex; 
        corners(total_corners_found).idx_exit    = i_exit; 
        corners(total_corners_found).apex_entry  = [x(i_entry), y(i_entry)]; 
        corners(total_corners_found).corner_apex = [x(i_apex),  y(i_apex)]; 
        corners(total_corners_found).apex_exit   = [x(i_exit),  y(i_exit)]; 
        corners(total_corners_found).R           = 1 / kappa_apex; 

        if kappa(i_apex) > 0 
            corners(total_corners_found).direction = 'Left'; 
        else
            corners(total_corners_found).direction = 'Right'; 
        end

        corners(total_corners_found).s_apex = s(i_apex); 

        % Exclude entry-exit range from future iteration within section
        available(i_entry:i_exit) = false; 
    end

    if count_sec < N_corners_sec
        fprintf(2, 'Warning: Section %d (%.0fm to %.0fm) only found %d corners (requested %d).\n', ...
            sec, s_start_sec, s_end_sec, count_sec, N_corners_sec);
    end
end

N_found = total_corners_found; 

% Sort all discovered corners chronologically along the track
[~, sort_order] = sort([corners.s_apex]); 
corners = corners(sort_order); 

%% 3.1. Resolve Entry/Exit Overlaps Between Consecutive Corners

for k = 1:N_found-1
    if corners(k).idx_exit >= corners(k+1).idx_entry - 3
        % Find midpoint index between the two apexes or overlap region
        mid_idx = round((corners(k).idx_apex + corners(k+1).idx_apex) / 2);

        % Clamp midpoint so it doesn't cross the apexes themselves
        mid_idx = max(corners(k).idx_apex + 1, min(corners(k+1).idx_apex - 1, mid_idx));

        % Set previous corner's exit and next corner's entry to mid_idx
        corners(k).idx_exit   = mid_idx;
        corners(k+1).idx_entry = mid_idx;

        % Update coordinates
        corners(k).apex_exit   = [x(mid_idx), y(mid_idx)];
        corners(k+1).apex_entry = [x(mid_idx), y(mid_idx)];

        fprintf('Resolved overlap between Corner %d and Corner %d at index %d (s = %.1fm)\n', ...
            k, k+1, mid_idx, s(mid_idx));
    end
end

%% 4. Straights: track start -> first corner, between corners, last corner -> track end

straights = struct('straight_start', {}, 'straight_end', {}, 'straight_distance', {}); 

% First straight
straight_start = [x(1), y(1)]; 
straight_end   = corners(1).apex_entry; 
straights(1).straight_start    = straight_start; 
straights(1).straight_end      = straight_end; 
straights(1).straight_distance = hypot(straight_end(1)-straight_start(1), ...
                                        straight_end(2)-straight_start(2)); 

% Inter-corner straights
for k = 1:N_found-1
    straight_start = corners(k).apex_exit; 
    straight_end   = corners(k+1).apex_entry; 

    straights(k+1).straight_start    = straight_start; 
    straights(k+1).straight_end      = straight_end; 
    straights(k+1).straight_distance = hypot(straight_end(1)-straight_start(1), ...
                                               straight_end(2)-straight_start(2)); 
end

% Final straight
straight_start = corners(end).apex_exit; 
straight_end   = [x(end), y(end)]; 
straights(end+1).straight_start    = straight_start; 
straights(end+1).straight_end      = straight_end; 
straights(end+1).straight_distance = hypot(straight_end(1)-straight_start(1), ...
                                             straight_end(2)-straight_start(2)); 

%% 5. Reconstructed curvature profile (s_final, kappa_final)

s_recon     = s;              
kappa_recon = zeros(n, 1);    

for k = 1:N_found
    i_entry = corners(k).idx_entry; 
    i_apex  = corners(k).idx_apex; 
    i_exit  = corners(k).idx_exit; 

    R = corners(k).R; 
    if strcmp(corners(k).direction, 'Left') 
        kappa_target = 1 / R; 
    else
        kappa_target = -1 / R; 
    end

    d_in  = s(i_apex)  - s(i_entry);  
    d_out = s(i_exit)  - s(i_apex);   

    if d_in > 0
        rate_in = kappa_target / d_in; 
        idx_in  = i_entry:i_apex; 
        kappa_recon(idx_in) = rate_in .* (s(idx_in) - s(i_entry)); 
    else
        kappa_recon(i_apex) = kappa_target; 
    end

    if d_out > 0
        rate_out = -kappa_target / d_out; 
        idx_out  = i_apex:i_exit; 
        kappa_recon(idx_out) = kappa_target + rate_out .* (s(idx_out) - s(i_apex)); 
    else
        kappa_recon(i_apex) = kappa_target; 
    end
end

%% 6. Text summary of track sections, in chronological order

fprintf('\n===== Track section summary (%d corners found across %d sections) =====\n', N_found, N_sections); 
fprintf('Straight  1: length = %6.1f m (track start -> Corner 1 entry)\n', straights(1).straight_distance); 
for k = 1:N_found
    fprintf('Corner %2d (%-5s): apex at s = %7.1f m, R = %6.1f m\n', ...
        k, corners(k).direction, corners(k).s_apex, corners(k).R); 
    fprintf('Straight %2d: length = %6.1f m\n', k+1, straights(k+1).straight_distance); 
end
fprintf('(final straight above runs Corner %d exit -> track end)\n', N_found); 

%% 7. Plot: curvature profile, raw vs reconstructed, with apexes

apex_idx_sorted   = [corners.idx_apex]; 
kappa_apex_values = kappa(apex_idx_sorted); 

figure;
plot(s, kappa, 'b-', 'DisplayName', '\kappa(s) raw'); hold on; 
plot(s_recon, kappa_recon, 'g-', 'LineWidth', 1.5, 'DisplayName', '\kappa_{recon}(s) reconstructed'); 
plot(s(apex_idx_sorted), kappa_apex_values, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Corner apex'); 

% Add vertical visual markers for section boundaries
for sec = 1:N_sections-1
    xline(sections(sec).s_end, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
end

xlabel('Distance along track, s (m)'); 
ylabel('Curvature, \kappa (1/m)'); 
title(sprintf('Curvature profile: raw vs reconstructed (%d corners)', N_found)); 
legend('show', 'Location', 'best'); 
grid on; 

%% 8. Plot: 2D track map with entry / apex / exit points

apex_xy  = reshape([corners.corner_apex], 2, [])'; 
entry_xy = reshape([corners.apex_entry],  2, [])'; 
exit_xy  = reshape([corners.apex_exit],   2, [])'; 

figure;
plot(x, y, 'k-'); hold on; axis equal; 
plot(apex_xy(:,1),  apex_xy(:,2),  'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Corner apex'); 
plot(entry_xy(:,1), entry_xy(:,2), 'g^', 'MarkerFaceColor', 'g', 'DisplayName', 'Corner entry'); 
plot(exit_xy(:,1),  exit_xy(:,2),  'ms', 'MarkerFaceColor', 'm', 'DisplayName', 'Corner exit'); 
xlabel('x (m)'); ylabel('y (m)'); 
title('2D track map with corner entry / apex / exit'); 
legend('show', 'Location', 'best'); 
grid on; 

%% 9. Export reconstructed track as .csv

track_recon = [s_recon(:), kappa_recon(:), theta, kappa_z]; 
writematrix(track_recon, 'track_recon4.csv');