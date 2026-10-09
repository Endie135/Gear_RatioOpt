clear; clc; close all;

%% 1. Load track coordinates (already smoothed, resampled to ~1 m spacing)

loaded = load('Catalunya.csv');
track = loaded;

x = track(:,1);
y = track(:,2);
n = length(x);

fprintf('Loaded %d track points\n', n);

%% 2. Path coordinates: distance along path and curvature

dx = diff(x);
dy = diff(y);
ds = sqrt(dx.^2 + dy.^2);
s  = [0; cumsum(ds)];

xp  = gradient(x, s);
yp  = gradient(y, s);
xpp = gradient(xp, s);
ypp = gradient(yp, s);

kappa = (xp .* ypp - yp .* xpp) ./ (xp.^2 + yp.^2).^1.5;

fprintf('Curvature stats: min = %.5f, max = %.5f, std = %.5f (1/m)\n', ...
    min(kappa), max(kappa), std(kappa));

abs_kappa = abs(kappa);

%% 3. Corner-by-corner extraction

% Loop: take the highest remaining |kappa| point as the next apex, find
% its entry/exit as the points where |kappa| first drops to corner_min
% times the apex curvature, then exclude that whole entry:exit span from
% future searches so the same physical corner can never be picked twice.
% Once N_corners have been found (or no candidates remain), sort the
% result chronologically by s.


N_corners  = 16;    % target number of corners - change as needed
corner_min = 0.1;   % entry/exit defined where |kappa| falls to this
                     % fraction of the apex curvature

available  = true(n,1);  % points still eligible to be picked as an apex
kappa_work = abs_kappa;

corners = struct('apex_entry', {}, 'corner_apex', {}, 'apex_exit', {}, ...
                  'R', {}, 's_apex', {}, 'idx_entry', {}, 'idx_apex', {}, 'idx_exit', {}, ...
                  'direction', {});

count = 0;
while count < N_corners && any(available)
    kappa_work(~available) = -Inf;
    [best_val, i_apex] = max(kappa_work);
    if best_val <= 0
        break;
    end

    kappa_apex = abs_kappa(i_apex);
    threshold  = corner_min * kappa_apex;

    i_entry = i_apex;
    while i_entry > 1 && abs_kappa(i_entry) > threshold
        i_entry = i_entry - 1;
    end

    i_exit = i_apex;
    while i_exit < n && abs_kappa(i_exit) > threshold
        i_exit = i_exit + 1;
    end

    count = count + 1;
    corners(count).idx_entry   = i_entry;
    corners(count).idx_apex    = i_apex;
    corners(count).idx_exit    = i_exit;
    corners(count).apex_entry  = [x(i_entry), y(i_entry)];
    corners(count).corner_apex = [x(i_apex),  y(i_apex)];
    corners(count).apex_exit   = [x(i_exit),  y(i_exit)];
    corners(count).R           = 1 / kappa_apex;

    % + curvature = left-hand corner, - curvature = right-hand corner
    if kappa(i_apex) > 0
        corners(count).direction = 'Left';
    else
        corners(count).direction = 'Right';
    end

    corners(count).s_apex = s(i_apex);

    available(i_entry:i_exit) = false;
end

N_found = count;
if N_found < N_corners
    fprintf(2, 'Warning: only found %d corners (requested %d).\n', N_found, N_corners);
end


% sort chronologically by along-track distance now that all corners have
% been found (they were discovered in curvature-magnitude order, not track order)
[~, sort_order] = sort([corners.s_apex]);
corners = corners(sort_order);

%% 4. Straights: track start -> first corner, between corners, last corner -> track end

straights = struct('straight_start', {}, 'straight_end', {}, 'straight_distance', {});

% first straight: from the very first track point to the first corner's entry
straight_start = [x(1), y(1)];
straight_end   = corners(1).apex_entry;
straights(1).straight_start    = straight_start;
straights(1).straight_end      = straight_end;
straights(1).straight_distance = hypot(straight_end(1)-straight_start(1), ...
                                        straight_end(2)-straight_start(2));

% straights between consecutive corners
for k = 1:N_found-1
    straight_start = corners(k).apex_exit;
    straight_end   = corners(k+1).apex_entry;

    straights(k+1).straight_start    = straight_start;
    straights(k+1).straight_end      = straight_end;
    straights(k+1).straight_distance = hypot(straight_end(1)-straight_start(1), ...
                                               straight_end(2)-straight_start(2));
end

% final straight: from the last corner's exit to the very last track point
straight_start = corners(end).apex_exit;
straight_end   = [x(end), y(end)];
straights(end+1).straight_start    = straight_start;
straights(end+1).straight_end      = straight_end;
straights(end+1).straight_distance = hypot(straight_end(1)-straight_start(1), ...
                                             straight_end(2)-straight_start(2));

%% 5. Reconstructed curvature profile (s_final, kappa_final)

% Straights carry kappa = 0. Through each corner, curvature is ramped
% linearly (constant rate of change) from 0 at the entry up to the
% signed apex curvature 1/R at the apex, then linearly back down to 0 at
% the exit - using the entry->apex and apex->exit distances to set each
% ramp's rate. Sign follows the corner's direction (+ left, - right).

s_recon     = s;              % same distance axis as the raw data
kappa_recon = zeros(n, 1);    % default: 0 on straights

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

    d_in  = s(i_apex) - s(i_entry);   % entry -> apex distance
    d_out = s(i_exit)  - s(i_apex);   % apex -> exit distance

    if d_in > 0
        rate_in = kappa_target / d_in;                 % curvature rate of change, entry -> apex
        idx_in  = i_entry:i_apex;
        kappa_recon(idx_in) = rate_in .* (s(idx_in) - s(i_entry));
    else
        kappa_recon(i_apex) = kappa_target;
    end

    if d_out > 0
        rate_out = -kappa_target / d_out;               % curvature rate of change, apex -> exit
        idx_out  = i_apex:i_exit;
        kappa_recon(idx_out) = kappa_target + rate_out .* (s(idx_out) - s(i_apex));
    else
        kappa_recon(i_apex) = kappa_target;
    end
end

%% 6. Text summary of track sections, in chronological order

fprintf('\n===== Track section summary (%d corners) =====\n', N_found);
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
title('2D Catalunya track map with corner entry / apex / exit');
legend('show', 'Location', 'best');
grid on;

%% Export reconstructed track as .csv

% track_recon = [s_recon(:), kappa_recon(:)];
% writematrix(track_recon, 'track_recon.csv');