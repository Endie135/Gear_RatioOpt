
import fastf1
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev

fastf1.Cache.enable_cache('fastf1_cache')

session = fastf1.get_session(2026, 'Monaco', 'Q')
session.load()

fastest_lap = session.laps.pick_fastest()
telemetry = fastest_lap.get_telemetry()

x = telemetry['X'].to_numpy()
y = telemetry['Y'].to_numpy()

## Convert from 1/10 metre units to metres 
x = x / 10.0
y = y / 10.0

## Shift from quadrant 3 into quadrant 1 (all values positive)
x = x - x.min()
y = y - y.min()

## Smoothing: Periodic parametric smoothing spline
s_factor = 5.0
tck, u = splprep([x, y], s=s_factor, per=True)

# evaluate the fitted spline on a dense grid
u_fine = np.linspace(0, 1, 20000, endpoint=False)
x_fine, y_fine = splev(u_fine, tck)

## Resample by arc length with 1m interval
dx = np.diff(x_fine)
dy = np.diff(y_fine)
ds = np.sqrt(dx**2 + dy**2)
s_fine = np.concatenate(([0.0], np.cumsum(ds)))
total_length = s_fine[-1]

s_uniform = np.arange(0, total_length, 1.0)

x_resampled = np.interp(s_uniform, s_fine, x_fine)
y_resampled = np.interp(s_uniform, s_fine, y_fine)

## Compile into a single 2D array and save as .csv file
processed_track = np.column_stack((x_resampled, y_resampled))
np.savetxt('processed_track.csv',processed_track,delimiter=',')

# Plot the processed track
plt.figure(figsize=(8, 8))
plt.plot(processed_track[:, 0], processed_track[:, 1], color='blue', label='Resampled path (1 m steps)')
plt.scatter(processed_track[::20, 0], processed_track[::20, 1], color='red', s=12, zorder=3,
            label='every 20th point (~20 m spacing')
plt.axis('equal')
plt.grid(True)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Processed Monaco track: shifted, closed, smoothed, resampled every 1 m')
plt.legend()
plt.show()

print(f"Number of points: {len(processed_track)}")
print(f"Approximate track length: {total_length:.1f} m")