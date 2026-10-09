
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

# Calculate total track length (Distance is in meters)
total_length_m = telemetry['Distance'].iloc[-1]

# Export raw track coordinates as .csv
raw_track = np.column_stack((x, y))
np.savetxt('raw_track.csv',raw_track,delimiter=',')

# Plot raw telemetry coordinates with track length display
plt.figure(figsize=(8, 8))
plt.plot(x, y, color='red', label='raw path')
plt.scatter(x[::5], y[::5], color='blue', s=12, zorder=3,
            label='every 5th point (~1s spacing)')
plt.title('Monaco Telemetry')
plt.axis('equal')
plt.grid(True)
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()

## Display track length at the bottom center of the graph
plt.figtext(
    0.5, 0.02, 
    f"Total Track Length: {total_length_m:.1f} m", 
    ha='center', fontsize=11, fontweight='bold',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='gray', alpha=0.8)
)

# Adjust layout so the bottom label isn't clipped
plt.tight_layout(rect=[0, 0.05, 1, 1])

plt.show()