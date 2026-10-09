clear; clc; close all;

%% 1. Load track coordinates (already smoothed, resampled to ~1 m spacing)

loaded = load('processed_track.csv');
track = loaded;

x = track(:,1);
y = track(:,2);
n = length(x);

fprintf('Loaded %d track points\n', n);

%% 2. Path coordinates: distance along path and curvature

dx = diff(x);
dy = diff(y);
ds = sqrt(dx.^2 + dy.^2);
s  = [0; cumsum(round(ds))];

xp  = gradient(x, s);
yp  = gradient(y, s);
xpp = gradient(xp, s);
ypp = gradient(yp, s);

kappa = (xp .* ypp - yp .* xpp) ./ (xp.^2 + yp.^2).^1.5;

fprintf('Curvature stats: min = %.5f, max = %.5f, std = %.5f (1/m)\n', ...
    min(kappa), max(kappa), std(kappa));

%% 3. Corner apex selection: greatest-curvature points, spaced apart

N_corners        = 16;   % target number of corners - change as needed
min_apex_spacing = 40;   % m, minimum along-track separation between apexes
                          % (prevents one physical corner being picked twice)

abs_kappa = abs(kappa);

available   = true(n,1);
kappa_work  = abs_kappa;
apex_idx    = zeros(N_corners,1);
count       = 0;

while count < N_corners && any(available)
    kappa_work(~available) = -Inf;
    [best_val, idx] = max(kappa_work);
    if best_val == -Inf
        break;   % no candidates left
    end
    count = count + 1;
    apex_idx(count) = idx;

    % suppress every point within min_apex_spacing of this apex so it
    % can't be picked again as a "different" corner
    within_range = abs(s - s(idx)) < min_apex_spacing;
    available(within_range) = false;
end

apex_idx = sort(apex_idx(1:count));   % chronological order (increasing s)
N_found  = numel(apex_idx);

if N_found < N_corners
    fprintf(2, 'Warning: only found %d corners (requested %d). Consider reducing min_apex_spacing.\n', ...
        N_found, N_corners);
end

corner_apex = [x(apex_idx), y(apex_idx)];   % (x,y) of each apex, chronological order

%% 4. Corner entry / exit via curvature rate-of-change

min_corner_delta = 0.001;   % threshold on |d(kappa)/ds| (1/m per m) marking
                             % where a corner's curvature has flattened out
                             % into the entry/exit transition - tune to data

dkappa_ds = gradient(kappa, s);

corners = struct('apex_entry', {}, 'corner_apex', {}, 'apex_exit', {}, ...
                  'R', {}, 's_apex', {}, 'idx_entry', {}, 'idx_apex', {}, 'idx_exit', {});

for k = 1:N_found
    i_apex = apex_idx(k);

    % walk backward from the apex until curvature stops changing quickly
    i_entry = i_apex;
    while i_entry > 1 && abs(dkappa_ds(i_entry)) > min_corner_delta
        i_entry = i_entry - 1;
    end

    % walk forward from the apex until curvature stops changing quickly
    i_exit = i_apex;
    while i_exit < n && abs(dkappa_ds(i_exit)) > min_corner_delta
        i_exit = i_exit + 1;
    end

    corners(k).idx_entry   = i_entry;
    corners(k).idx_apex    = i_apex;
    corners(k).idx_exit    = i_exit;
    corners(k).apex_entry  = [x(i_entry), y(i_entry)];
    corners(k).corner_apex = [x(i_apex),  y(i_apex)];
    corners(k).apex_exit   = [x(i_exit),  y(i_exit)];
    corners(k).R           = 1 / abs(kappa(i_apex));
    corners(k).s_apex      = s(i_apex);
end

%% 5. Straights between consecutive corners (plus lap-closing straight)

straights = struct('straight_start', {}, 'straight_end', {}, 'straight_distance', {});

for k = 1:N_found-1
    straight_start = corners(k).apex_exit;       % exit of current corner
    straight_end   = corners(k+1).apex_entry;    % entry of next corner

    straight_distance = hypot(straight_end(1)-straight_start(1), ...
                               straight_end(2)-straight_start(2));

    straights(k).straight_start    = straight_start;
    straights(k).straight_end      = straight_end;
    straights(k).straight_distance = straight_distance;
end

% Closing straight: last corner's exit back to the first corner's entry
% (the track is a closed loop, so this completes the lap)
straight_start = corners(end).apex_exit;
straight_end   = corners(1).apex_entry;
straight_distance = hypot(straight_end(1)-straight_start(1), ...
                           straight_end(2)-straight_start(2));

straights(end+1).straight_start    = straight_start;
straights(end+1).straight_end      = straight_end;
straights(end+1).straight_distance = straight_distance;

%% 6. Text summary of track sections, in chronological order

fprintf('\n===== Track section summary (%d corners) =====\n', N_found);
for k = 1:N_found
    fprintf('Corner %2d: apex at s = %7.1f m, R = %6.1f m\n', ...
        k, corners(k).s_apex, corners(k).R);
    if k < N_found
        fprintf('Straight %2d: length = %6.1f m\n', k, straights(k).straight_distance);
    end
end
fprintf('Closing straight (Corner %d -> Corner 1): length = %6.1f m\n', ...
    N_found, straights(end).straight_distance);

%% 7. Plot: curvature profile with selected apexes

figure;
plot(s, kappa, 'b-'); hold on;
plot(s(apex_idx), kappa(apex_idx), 'ro', 'MarkerFaceColor', 'r');
xlabel('Distance along track, s (m)');
ylabel('Curvature, \kappa (1/m)');
title(sprintf('Curvature profile with %d selected corner apexes', N_found));
legend('\kappa(s)', 'Corner apex', 'Location', 'best');
grid on;

%% 8. Plot: 2D track map with entry / apex / exit points

apex_xy  = reshape([corners.corner_apex], 2, [])';
entry_xy = reshape([corners.apex_entry],  2, [])';
exit_xy  = reshape([corners.apex_exit],   2, [])';

figure;
plot(x, y, 'k-'); hold on; axis equal;
xlim([0 800]);
ylim([0,1000]);
plot(apex_xy(:,1),  apex_xy(:,2),  'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Corner apex');
plot(entry_xy(:,1), entry_xy(:,2), 'g^', 'MarkerFaceColor', 'g', 'DisplayName', 'Corner entry');
plot(exit_xy(:,1),  exit_xy(:,2),  'ms', 'MarkerFaceColor', 'm', 'DisplayName', 'Corner exit');
xlabel('x (m)'); ylabel('y (m)');
title('2D track map with corner entry/apex/exit');
legend('show', 'Location', 'best');
grid on;