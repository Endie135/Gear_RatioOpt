
import fastf1
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d

fastf1.Cache.enable_cache('fastf1_cache')

session = fastf1.get_session(2026, 'Monaco', 'Q')
session.load()

fastest_lap = session.laps.pick_fastest()
telemetry = fastest_lap.get_telemetry()

x = telemetry['X'].to_numpy()
y = telemetry['Y'].to_numpy()
z = telemetry['Z'].to_numpy()

## Convert from 1/10 metre units to metres 
x = x / 10.0
y = y / 10.0
z = z / 10.0

## Shift from quadrant 3 into quadrant 1 (all values positive)
x = x - x.min()
y = y - y.min()

## Smoothing: Periodic parametric smoothing spline
s_factor = 5.0
tck, u = splprep([x, y, z], s=s_factor, per=True)

# evaluate the fitted spline on a dense grid
u_fine = np.linspace(0, 1, 20000, endpoint=False)
x_fine, y_fine, z_fine = splev(u_fine, tck)

## Resample by arc length with 1m interval
dx = np.diff(x_fine)
dy = np.diff(y_fine)
ds = np.sqrt(dx**2 + dy**2)
s_fine = np.concatenate(([0.0], np.cumsum(ds)))
total_length = s_fine[-1]

s_uniform = np.arange(0, total_length, 1.0)

x_resampled = np.interp(s_uniform, s_fine, x_fine)
y_resampled = np.interp(s_uniform, s_fine, y_fine)
z_resampled = np.interp(s_uniform, s_fine, z_fine)

# Apply Gaussian smoothing to the resampled Z values
z_smoothed = gaussian_filter1d(z_resampled, sigma=30)

# Calculate vertical curvature to check noise
slope = np.gradient(z_smoothed, s_uniform)
kappa_z = np.gradient(slope, s_uniform)

# Compile into 3D array
processed_trackZ = np.column_stack((x_resampled, y_resampled, z_resampled))

## Plot the processed track with Z-axis as colormap
fig, ax = plt.subplots(figsize=(10, 8))

# Scatter plot of track with Z mapped to color
sc = ax.scatter(processed_trackZ[:, 0], processed_trackZ[:, 1], c=processed_trackZ[:, 2], 
                cmap='viridis', s=10, zorder=2, label='Resampled path (1 m steps)')

ax.axis('equal')
ax.grid(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Processed Monaco track: shifted, closed, smoothed, resampled every 1 m')

cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('Elevation Z [m]', rotation=270, labelpad=15, fontweight='bold')

ax.legend(loc='best')
plt.tight_layout()
plt.show()

print(f"Number of points: {len(processed_trackZ)}")
print(f"Approximate track length: {total_length:.1f} m")

## Plot Track Elevation and Vertical Curvature
fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

# Top Subplot: Elevation
ax_top.plot(s_uniform, z_smoothed, color='black', linewidth=1.5, label='Smoothed Z (Sigma=20)')
ax_top.set_ylabel('Elevation z [m]')
ax_top.set_title('Track Elevation Profile')
ax_top.grid(True)
ax_top.legend(loc='best')

# Bottom Subplot: Vertical Curvature
ax_bot.plot(s_uniform, kappa_z, color='magenta', linewidth=1.5, label=r'$\kappa_z(s)$ (Vertical Curvature)')
ax_bot.axhline(0, color='black', linestyle='--', alpha=0.7)  # Zero reference line
ax_bot.set_xlabel('Distance along track s [m]')
ax_bot.set_ylabel(r'Vertical Curvature $\kappa_z$ [1/m]')
ax_bot.set_title('Vertical Curvature Profile (Crests < 0, Dips > 0)')
ax_bot.grid(True)
ax_bot.legend(loc='best')

plt.tight_layout()
plt.show()