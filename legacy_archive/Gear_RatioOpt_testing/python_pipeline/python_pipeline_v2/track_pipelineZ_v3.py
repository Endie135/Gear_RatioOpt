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
z = telemetry['Z'].to_numpy()

## Convert from 1/10 metre units to metres 
x = x / 10.0
y = y / 10.0
z = z / 10.0

# Shift from quadrant 3 into quadrant 1 (all values positive)
x = x - x.min()
y = y - y.min()

## Refined Track Processing Pipeline with Differential Geometry Outputs
# 1. Fit Periodic Smoothing B-Spline (Degree k=5 for smooth 2nd derivatives)
s_factor = 5.0
tck, u = splprep([x, y, z], s=s_factor, per=True, k=5)

# 2. Dense grid to compute 2D horizontal arc length
u_dense = np.linspace(0, 1, 50000, endpoint=False)
dx_du, dy_du, dz_du = splev(u_dense, tck, der=1)

# Compute 2D differential step ds = sqrt(dx^2 + dy^2)
ds_du_dense = np.sqrt(dx_du**2 + dy_du**2)

# Integrate 2D arc length via cumulative trapezoidal sum
s_dense = np.concatenate(([0.0], np.cumsum((ds_du_dense[:-1] + ds_du_dense[1:]) / 2.0 * np.diff(u_dense))))
total_length = s_dense[-1]

# 3. Define uniform 1-meter arc length grid (s)
s = np.arange(0, total_length, 1.0)

# Map uniform 1m spatial steps back to the underlying spline parameter u
u_uniform = np.interp(s, s_dense, u_dense)

# 4. Evaluate analytical 1st and 2nd derivatives at exact 1-meter spacing
x_resampled, y_resampled, z_resampled = splev(u_uniform, tck, der=0)
dx, dy, dz = splev(u_uniform, tck, der=1)
d2x, d2y, d2z = splev(u_uniform, tck, der=2)

# 2D Speed metric along spline parameter: ds/du
ds_du = np.sqrt(dx**2 + dy**2)

# 5. Calculate Differential Geometry Outputs

# Slope Angle theta (in rads)
theta = np.arctan2(dz, ds_du)

# Vertical Curvature kappa_z (1/m)
d_theta_du = np.gradient(theta, u_uniform)
kappa_z = d_theta_du / ds_du

# 2D Path Curvature kappa (1/m)
kappa = (dx * d2y - dy * d2x) / (ds_du**3)

# Compile into array and export as .csv for QSS
# Columns: [x, y, z, s, kappa, theta, kappa_z]
processed_track = np.column_stack((x_resampled, y_resampled, z_resampled, s, kappa, theta, kappa_z))
np.savetxt('processed_trackZ1.csv', processed_track, delimiter=',')

print(f"Number of points: {len(processed_track)}")
print(f"Approximate track length: {total_length:.1f} m")

## Plot the processed track with Z-axis as colormap
fig, ax = plt.subplots(figsize=(10, 8))

# Scatter plot of track with Z mapped to color
sc = ax.scatter(processed_track[:, 0], processed_track[:, 1], c=processed_track[:, 2], 
                cmap='viridis', s=10, zorder=2, label='Resampled path (1 m steps)')

ax.axis('equal')
ax.grid(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Processed Monaco track: shifted, closed, smoothed, resampled every 1 m')

# Add Colorbar for Elevation (Z)
cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('Elevation Z [m]', rotation=270, labelpad=15, fontweight='bold')

ax.legend(loc='best')
plt.tight_layout()
plt.show()

## Plot Track Elevation and Vertical Curvature
fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

# Top Subplot: Elevation
ax_top.plot(s, z_resampled, color='black', linewidth=1.5, label='Resampled Z')
ax_top.set_ylabel('Elevation z [m]')
ax_top.set_title('Track Elevation Profile')
ax_top.grid(True)
ax_top.legend(loc='best')

# Bottom Subplot: Vertical Curvature
ax_bot.plot(s, kappa_z, color='magenta', linewidth=1.5, label=r'$\kappa_z(s)$ (Vertical Curvature)')
ax_bot.axhline(0, color='black', linestyle='--', alpha=0.7)  # Zero reference line
ax_bot.set_xlabel('Distance along track s [m]')
ax_bot.set_ylabel(r'Vertical Curvature $\kappa_z$ [1/m]')
ax_bot.set_title('Vertical Curvature Profile')
ax_bot.grid(True)
ax_bot.legend(loc='best')

plt.tight_layout()
plt.show()

## Slope and Curvature Plots (3 Subplots)
fig, (ax_top, ax_mid, ax_bot) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

# Top Subplot: Slope Angle
ax_top.plot(s, theta, color='blue', linewidth=1.5, label=r'Slope $\theta$ [rad]')
ax_top.axhline(0, color='black', linestyle='--', alpha=0.5)
ax_top.set_ylabel(r'Slope Angle $\theta$ [rad]')
ax_top.set_title(r'Track Slope Angle ($\theta$)')
ax_top.grid(True)
ax_top.legend(loc='best')

# Middle Subplot: 2D Path Curvature
ax_mid.plot(s, kappa, color='green', linewidth=1.2, label=r'$\kappa(s)$ (2D Path Curvature)')
ax_mid.axhline(0, color='black', linestyle='--', alpha=0.5)
ax_mid.set_ylabel(r'Path Curvature [1/m]')
ax_mid.set_title(r'2D Path Curvature ($\kappa$)')
ax_mid.grid(True)
ax_mid.legend(loc='best')

# Bottom Subplot: Vertical Curvature
ax_bot.plot(s, kappa_z, color='magenta', linewidth=1.2, label=r'$\kappa_z(s)$ (Vertical Curvature)')
ax_bot.axhline(0, color='black', linestyle='--', alpha=0.5)
ax_bot.set_xlabel('Path Distance s [m]')
ax_bot.set_ylabel(r'Vertical Curvature [1/m]')
ax_bot.set_title(r'Vertical Curvature ($\kappa_z$)')
ax_bot.grid(True)
ax_bot.legend(loc='best')

plt.tight_layout()
plt.show()

## 2D Track Map with Vertical Curvature (kappa_z) Colormap
fig, ax = plt.subplots(figsize=(10, 8))

# Symmetric color scale limits centered around 0 using 2nd and 98th percentiles
# to prevent extreme kerb spikes from clipping the colorbar range completely
vmax = np.percentile(np.abs(kappa_z), 98)

sc_kz = ax.scatter(x_resampled, y_resampled, c=kappa_z, 
                   cmap='coolwarm', vmin=-vmax, vmax=vmax, s=15, zorder=2)

ax.axis('equal')
ax.grid(True)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_title(r'2D Track Map: Vertical Curvature ($\kappa_z$) Distribution')

# Colorbar for Vertical Curvature
cbar = plt.colorbar(sc_kz, ax=ax)
cbar.set_label(r'Vertical Curvature $\kappa_z$ [1/m] (Red = Dip / Blue = Crest)', 
               rotation=270, labelpad=20, fontweight='bold')

plt.tight_layout()
plt.show()