"""
SINGLE-STAGE SPUR GEAR & SHAFT SIZING SCRIPT
Standards: Lewis Bending Stress (Gears), Static von Mises Criterion (Shafts)
Materials: 
  - Gears: AISI 9310 Steel (Carburized Core Properties)
  - Shafts: AISI 4340M
"""

import math


## CENTRALIZED USER INPUT BLOCK ##

# Drivetrain Operating Parameters
T_in = 350.0          # Input Torque [Nm]
i_target = 10.20      # Target Gear Reduction Ratio
N1 = 18               # Input Pinion Tooth Count (N1 >= 17 to prevent undercut)
SF = 1.5              # Design Safety Factor

# Gear Material Properties: AISI 9310 Steel (Carburized Core Properties)
Sy_gear = 850.0       # Tensile Yield Strength [MPa]
rho_gear = 7850.0     # Mass Density [kg/m^3]

# Shaft Material Properties: AISI 4340M / 300M Steel
Sy_shaft = 1200.0     # Tensile Yield Strength [MPa]
rho_shaft = 7850.0    # Mass Density [kg/m^3]

# Gear Design Constraints & Empirics
k_bm = 10.0           # Face Width Ratio factor (b = k_bm * m)
phi_deg = 20.0        # Pressure Angle [degrees]
Y1 = 0.308            # Lewis Form Factor for N1 = 18 at 20° pressure angle

# Shaft Design Constants
L_arm = 25.0          # Estimated distance from shaft bearing to gear face [mm]



## GOVERNING CALCULATIONS & LOGIC ##

## 1. Gear Teeth & Gear Sizing
N2 = int(round(N1 * i_target))
i_actual = N2 / N1
T_out = T_in * i_actual

## 2. Gear Tooth Sizing (Lewis Bending Stress)
sigma_allow_gear = Sy_gear / SF  # Allowable Gear Bending Stress [MPa]

m_calc = ((2.0 * T_in * 1000.0) / (N1 * k_bm * Y1 * sigma_allow_gear)) ** (1.0 / 3.0)

# Standard Metric Modules array [mm]
standard_modules = [1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 5.0]
m = min([mod for mod in standard_modules if mod >= m_calc], default=m_calc)

## 3. Gear Dimensions
b = k_bm * m                 # Face Width [mm]
d1 = m * N1                  # Pinion Pitch Diameter [mm]
d2 = m * N2                  # Driven Gear Pitch Diameter [mm]
d_a1 = m * (N1 + 2)          # Pinion Outer Diameter [mm]
d_a2 = m * (N2 + 2)          # Driven Gear Outer Diameter [mm]
a = (d1 + d2) / 2.0          # Center Distance [mm]
backlash = 0.04 * m          # Backlash [mm]
root_fillet = 0.35 * m       # Root Fillet Radius [mm]

## 4. Estimated Mass Metrics (using cylinder baseline)
# Conversion to meters for mass calculations:
d1_m = d1 / 1000.0
d2_m = d2 / 1000.0
b_m = b / 1000.0    

# Solid cylinder mass: m_mass = rho * pi * (r^2) * b
m1_mass = rho_gear * math.pi * ((d1_m / 2.0) ** 2) * b_m
m2_mass = rho_gear * math.pi * ((d2_m / 2.0) ** 2) * b_m


## 5. Shaft Sizing (von Mises)
sigma_allow_shaft = Sy_shaft / SF  # Allowable Shaft Bending Stress [MPa]

# Input Shaft Calculations
Ft1 = (2.0 * T_in * 1000.0) / d1
Fr1 = Ft1 * math.tan(math.radians(phi_deg))
Fres1 = math.sqrt(Ft1**2 + Fr1**2)
M1 = Fres1 * L_arm
M_eq_in = math.sqrt(4.0 * (M1**2) + 3.0 * ((T_in * 1000.0)**2))
d_shaft_in_calc = ((16.0 * M_eq_in) / (math.pi * sigma_allow_shaft)) ** (1.0 / 3.0)

# Output Shaft Calculations
Ft2 = (2.0 * T_out * 1000.0) / d2
Fr2 = Ft2 * math.tan(math.radians(phi_deg))
Fres2 = math.sqrt(Ft2**2 + Fr2**2)
M2 = Fres2 * L_arm
M_eq_out = math.sqrt(4.0 * (M2**2) + 3.0 * ((T_out * 1000.0)**2))
d_shaft_out_calc = ((16.0 * M_eq_out) / (math.pi * sigma_allow_shaft)) ** (1.0 / 3.0)

# Round shaft bores UP to standard bearing bore sizes [mm]
standard_bearing_bores = [12, 15, 17, 20, 25, 30, 35, 40, 45, 50, 60]
d_shaft_in = min([bore for bore in standard_bearing_bores if bore >= d_shaft_in_calc], default=d_shaft_in_calc)
d_shaft_out = min([bore for bore in standard_bearing_bores if bore >= d_shaft_out_calc], default=d_shaft_out_calc)


## STRUCTURED REPORT OUTPUT ##

print("=" * 78)
print("             SINGLE-STAGE SPUR GEAR REDUCTION SIZING")
print("=" * 78)

print("\n1. DESIGN INPUTS & MATERIAL SUMMARY")
print("-" * 50)
print(f"  • Input Torque (T_in)           : {T_in:.2f} Nm")
print(f"  • Target Ratio (i_target)       : {i_target:.2f}")
print(f"  • Pinion Teeth (N1)             : {N1}")
print(f"  • Design Safety Factor (SF)     : {SF:.2f}")
print("  • Gear Material (AISI 9310)     ")
print(f"      - Yield Strength (Sy_gear)  : {Sy_gear:.1f} MPa")
print(f"      - Density (rho_gear)        : {rho_gear:.1f} kg/m^3")
print("  • Shaft Material (AISI 4340M)   ")
print(f"      - Yield Strength (Sy_shaft) : {Sy_shaft:.1f} MPa")
print(f"      - Density (rho_shaft)       : {rho_shaft:.1f} kg/m^3")

print("\n2. CALCULATED RATIO & TORQUE")
print("-" * 50)
print(f"  • Pinion Teeth (N1)             : {N1}")
print(f"  • Driven Gear Teeth (N2)        : {N2}")
print(f"  • Actual Gear Ratio (i_actual)  : {i_actual:.4f}")
print(f"  • Output Torque (T_out)         : {T_out:.2f} Nm")

print("\n3. GEAR DIMENSIONS & SIZING")
print("-" * 50)
print("  Parameter values for  Fusion 360:")
print(f"  [x] Standard                   : Metric")
print(f"  [x] Pressure Angle             : {phi_deg:.1f} deg")
print(f"  [x] Module (m)                 : {m:.2f} mm  (Calculated minimum: {m_calc:.3f} mm)")
print(f"  [x] Pinion Teeth (N1)          : {N1}")
print(f"  [x] Driven Gear Teeth (N2)     : {N2}")
print(f"  [x] Gear Thickness (b)         : {b:.2f} mm")
print(f"  [x] Backlash                   : {backlash:.3f} mm")
print(f"  [x] Root Fillet Radius         : {root_fillet:.3f} mm")

print("\n4. KEY GEAR & SHAFT GEOMETRY")
print("-" * 50)
print(f"  • Pinion Pitch Diameter (d1)    : {d1:.2f} mm")
print(f"  • Pinion Outer Diameter (d_a1)  : {d_a1:.2f} mm")
print(f"  • Driven Gear Pitch Dia (d2)    : {d2:.2f} mm")
print(f"  • Driven Gear Outer Dia (d_a2)  : {d_a2:.2f} mm")
print(f"  • Center Distance (a)           : {a:.2f} mm")
print(f"  • Input Shaft Hole Dia          : {d_shaft_in:.2f} mm  (Calculated min: {d_shaft_in_calc:.2f} mm)")
print(f"  • Output Shaft Hole Dia         : {d_shaft_out:.2f} mm  (Calculated min: {d_shaft_out_calc:.2f} mm)")

print("\n5. ESTIMATED MASS METRICS (Cylinder Approximation)")
print("-" * 50)
print(f"  • Pinion Mass                   : {m1_mass:.3f} kg")
print(f"  • Driven Gear Mass              : {m2_mass:.3f} kg")
print(f"  • Total Solid Gear Mass         : {m1_mass + m2_mass:.3f} kg")
print("=" * 78)