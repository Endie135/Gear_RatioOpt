
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
car_telemetry = fastest_lap.get_car_data()

## Extract telemetry coordinates
distance = telemetry['Distance']
speed = telemetry['Speed']

print(f"Fastest Lap Speed Data: {speed}")

# 6. Generate the Matplotlib graph
fig, ax = plt.subplots(figsize=(12, 5))

# Plot the distance against speed
ax.plot(distance, speed, label=fastest_lap['Driver'], color='black', linewidth=2)

# Set labels, titles, and grid formatting
ax.set_title(f"Speed Trace - {session.event['EventName']} {session.event['EventDate'].year}\nFastest Lap: {fastest_lap['Driver']}", fontsize=14, fontweight='bold')
ax.set_xlabel('Distance (meters)', fontsize=12)
ax.set_ylabel('Speed (km/h)', fontsize=12)
ax.grid(True, linestyle='--', alpha=0.5)
ax.legend(loc='upper right')

# Display the chart
plt.show()