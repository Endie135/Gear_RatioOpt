
import fastf1
import matplotlib.pyplot as plt
import numpy as np

fastf1.Cache.enable_cache('fastf1_cache')

## Load Monaco Session (2026 Qualifying)
session = fastf1.get_session(2026, 'Monaco', 'Q')
session.load()

## Get telemetry from the fastest lap
fastest_lap = session.laps.pick_fastest()
telemetry = fastest_lap.get_telemetry()

## Extract telemetry coordinates
x = telemetry['X']
y = telemetry['Y']
z = telemetry['Z']

# Calculate total track length (Distance is in meters)
total_length_m = telemetry['Distance'].iloc[-1]

# Compile into 3D array
raw_trackZ = np.column_stack((x, y, z))

# Plot raw telemetry coordinates with Z-axis as colormap
fig, ax = plt.subplots(figsize=(10, 8))

# Scatter plot of track with Z mapped to color
sc = ax.scatter(x, y, c=z, cmap='viridis', s=15, zorder=2)

# Optional line beneath scatter to visually connect any telemetry gaps
ax.plot(x, y, color='gray', linestyle='--', alpha=0.4, zorder=1)

ax.set_title('Monaco Telemetry: Track Layout & Elevation Profile (Z)', fontsize=12, fontweight='bold')
ax.axis('equal')
ax.grid(True)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')

# Add Colorbar for Elevation (Z)
cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('Elevation Z [dm]', rotation=270, labelpad=15, fontweight='bold')

## Display track length at the bottom center of the graph
plt.figtext(
    0.5, 0.02, 
    f"Total Track Length: {total_length_m:.1f} m", 
    ha='center', fontsize=11, fontweight='bold',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='gray', alpha=0.8)
)

plt.tight_layout(rect=[0, 0.05, 1, 1])

plt.show()