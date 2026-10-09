"""
2-STAGE SPUR GEARBOX & SHAFT SIZING SCRIPT
Standards: Lewis Bending Stress (Gears), Static von Mises Yield Criterion (Shafts)
Materials: 
  - Gears: AISI 9310 Steel (Carburized Core Properties)
  - Shafts: AISI 4340M
"""

import math


## CENTRALIZED USER INPUT BLOCK ##

# Drivetrain Operating Parameters
T_in = 350.0          # Input Torque [Nm]
i_target = 10.20      # Target Overall Gear Reduction Ratio
N1 = 20               # Stage 1 Input Pinion Teeth (N1 >= 17 to prevent undercut)
N3 = 20               # Stage 2 Pinion Teeth (N3 >= 17 to prevent undercut)
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
Y1 = 0.320            # Lewis Form Factor for N1 = 18 at 20° pressure angle
Y3 = 0.320            # Lewis Form Factor for N3 = 18 at 20° pressure angle

# Shaft Design Constants
L_arm = 25.0          # Estimated distance from shaft bearing to gear face [mm]



## GOVERNING CALCULATIONS & LOGIC ##


## 1. Kinematics & Stage Ratio Splits
N2 = 68
N4 = 60

i_stage1 = N2 / N1
i_stage2 = N4 / N3
i_actual_total = i_stage1 * i_stage2

T_counter = T_in * i_stage1
T_out = T_in * i_actual_total


## 2. Gear Tooth Sizing (Lewis Bending Stress Criterion)
sigma_allow_gear = Sy_gear / SF  # Allowable Gear Bending Stress [MPa]
standard_modules = [1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 5.0]

# Stage 1 Module Sizing
m1_calc = ((2.0 * T_in * 1000.0) / (N1 * k_bm * Y1 * sigma_allow_gear)) ** (1.0 / 3.0)
m1 = min([mod for mod in standard_modules if mod >= m1_calc], default=m1_calc)

# Stage 2 Module Sizing
m2_calc = ((2.0 * T_counter * 1000.0) / (N3 * k_bm * Y3 * sigma_allow_gear)) ** (1.0 / 3.0)
m2 = min([mod for mod in standard_modules if mod >= m2_calc], default=m2_calc)


## 3. Gear Dimensions
# Stage 1 Geometry
b1 = k_bm * m1               # Face Width [mm]
d1 = m1 * N1                # Pinion 1 Pitch Diameter [mm]
d2 = m1 * N2                # Gear 2 Pitch Diameter [mm]
d_a1 = m1 * (N1 + 2)        # Pinion 1 Outer Diameter [mm]
d_a2 = m1 * (N2 + 2)        # Gear 2 Outer Diameter [mm]
a1 = (d1 + d2) / 2.0        # Stage 1 Center Distance [mm]
backlash1 = 0.04 * m1       # Backlash Stage 1 [mm]
root_fillet1 = 0.35 * m1    # Root Fillet Stage 1 [mm]

# Stage 2 Geometry
b2 = k_bm * m2               # Face Width [mm]
d3 = m2 * N3                # Pinion 3 Pitch Diameter [mm]
d4 = m2 * N4                # Gear 4 Pitch Diameter [mm]
d_a3 = m2 * (N3 + 2)        # Pinion 3 Outer Diameter [mm]
d_a4 = m2 * (N4 + 2)        # Gear 4 Outer Diameter [mm]
a2 = (d3 + d4) / 2.0        # Stage 2 Center Distance [mm]
backlash2 = 0.04 * m2       # Backlash Stage 2 [mm]
root_fillet2 = 0.35 * m2    # Root Fillet Stage 2 [mm]

##  4. Tooth Force Calculations
# Gear N1 Forces (Stage 1 Pinion)
Ft1 = (2.0 * T_in * 1000.0) / d1
Fr1 = Ft1 * math.tan(math.radians(phi_deg))
Fres1 = math.sqrt(Ft1**2 + Fr1**2)

# Gear N2 Forces (Stage 1 Driven Gear)
Ft2 = (2.0 * T_counter * 1000.0) / d2
Fr2 = Ft2 * math.tan(math.radians(phi_deg))
Fres2 = math.sqrt(Ft2**2 + Fr2**2)

# Gear N3 Forces (Stage 2 Pinion)
Ft3 = (2.0 * T_counter * 1000.0) / d3
Fr3 = Ft3 * math.tan(math.radians(phi_deg))
Fres_counter = math.sqrt((Ft2 + Ft3)**2 + (Fr2 + Fr3)**2)

# Gear N4 Forces (Stage 2 Driven Gear)
Ft4 = (2.0 * T_out * 1000.0) / d4
Fr4 = Ft4 * math.tan(math.radians(phi_deg))
Fres4 = math.sqrt(Ft4**2 + Fr4**2)


## 5. Shaft Sizing (Static von Mises Yield Criterion)
sigma_allow_shaft = Sy_shaft / SF  # Allowable Shaft Stress [MPa]

# Shaft 1: Input Shaft
M_in = Fres1 * L_arm
M_eq_in = math.sqrt(4.0 * (M_in**2) + 3.0 * ((T_in * 1000.0)**2))
d_shaft_in_calc = ((16.0 * M_eq_in) / (math.pi * sigma_allow_shaft)) ** (1.0 / 3.0)

# Shaft 2: Countershaft
M_counter = Fres_counter * L_arm
M_eq_counter = math.sqrt(4.0 * (M_counter**2) + 3.0 * ((T_counter * 1000.0)**2))
d_shaft_counter_calc = ((16.0 * M_eq_counter) / (math.pi * sigma_allow_shaft)) ** (1.0 / 3.0)

# Shaft 3: Output Shaft
M_out = Fres4 * L_arm
M_eq_out = math.sqrt(4.0 * (M_out**2) + 3.0 * ((T_out * 1000.0)**2))
d_shaft_out_calc = ((16.0 * M_eq_out) / (math.pi * sigma_allow_shaft)) ** (1.0 / 3.0)

# Input Shaft Extension
d_shaft_in_pure_torsion_calc = ((16.0 * math.sqrt(3.0) * (T_in * 1000.0)) / (math.pi * sigma_allow_shaft)) ** (1.0 / 3.0)

# Output Shaft Extension
d_shaft_out_pure_torsion_calc = ((16.0 * math.sqrt(3.0) * (T_out * 1000.0)) / (math.pi * sigma_allow_shaft)) ** (1.0 / 3.0)

# Standard Bearing Bores Lookup
standard_bearing_bores = [12, 15, 17, 20, 25, 30, 35, 40, 45, 50, 60]
d_shaft_in = min([bore for bore in standard_bearing_bores if bore >= d_shaft_in_calc], default=d_shaft_in_calc)
d_shaft_counter = min([bore for bore in standard_bearing_bores if bore >= d_shaft_counter_calc], default=d_shaft_counter_calc)
d_shaft_out = min([bore for bore in standard_bearing_bores if bore >= d_shaft_out_calc], default=d_shaft_out_calc)


## 6. Mass Metrics (Solid Cylinder Approximation)
m1_mass = rho_gear * math.pi * (((d1 / 1000.0) / 2.0) ** 2) * (b1 / 1000.0)
m2_mass = rho_gear * math.pi * (((d2 / 1000.0) / 2.0) ** 2) * (b1 / 1000.0)
m3_mass = rho_gear * math.pi * (((d3 / 1000.0) / 2.0) ** 2) * (b2 / 1000.0)
m4_mass = rho_gear * math.pi * (((d4 / 1000.0) / 2.0) ** 2) * (b2 / 1000.0)
total_gear_mass = m1_mass + m2_mass + m3_mass + m4_mass


## STRUCTURED REPORT OUTPUT ##

print("=" * 78)
print("             2-STAGE SPUR GEAR REDUCTION SIZING REPORT")
print("=" * 78)

print("\n1. DESIGN INPUTS & MATERIAL SUMMARY")
print("-" * 50)
print(f"  • Input Torque (T_in)             : {T_in:.2f} Nm")
print(f"  • Target Overall Ratio (i_target) : {i_target:.2f}")
print(f"  • Pinion Teeth (N1, N3)           : N1 = {N1}, N3 = {N3}")
print(f"  • Design Safety Factor (SF)       : {SF:.2f}")
print("  • Gear Material (AISI 9310)     ")
print(f"      - Yield Strength (Sy_gear)    : {Sy_gear:.1f} MPa")
print(f"      - Density (rho_gear)          : {rho_gear:.1f} kg/m^3")
print("  • Shaft Material (AISI 4340M)   ")
print(f"      - Yield Strength (Sy_shaft)   : {Sy_shaft:.1f} MPa")
print(f"      - Density (rho_shaft)         : {rho_shaft:.1f} kg/m^3")

print("\n2. CALCULATED RATIO & TORQUE")
print("-" * 50)
print(f"  • Stage 1 Teeth (N1 -> N2)      : {N1} -> {N2}")
print(f"  • Stage 1 Ratio (i_stage1)      : {i_stage1:.4f}")
print(f"  • Stage 2 Teeth (N3 -> N4)      : {N3} -> {N4}")
print(f"  • Stage 2 Ratio (i_stage2)      : {i_stage2:.4f}")
print(f"  • Total Actual Ratio (i_actual) : {i_actual_total:.4f}")
print(f"  • Shaft 1 Torque (Input)        : {T_in:.2f} Nm")
print(f"  • Shaft 2 Torque (Countershaft) : {T_counter:.2f} Nm")
print(f"  • Shaft 3 Torque (Output)       : {T_out:.2f} Nm")

print("\n3. FUSION 360 SPURGEAR DIRECT INPUT CHECKLIST")
print("-" * 50)
print("  STAGE 1 GEAR SET (Pinion N1 / Gear N2):")
print(f"  [x] Standard                   : Metric")
print(f"  [x] Pressure Angle             : {phi_deg:.1f} deg")
print(f"  [x] Module (m1)                : {m1:.2f} mm  (Calculated min: {m1_calc:.3f} mm)")
print(f"  [x] Pinion Teeth (N1)          : {N1}")
print(f"  [x] Driven Gear Teeth (N2)     : {N2}")
print(f"  [x] Gear Thickness (b1)        : {b1:.2f} mm")
print(f"  [x] Backlash                   : {backlash1:.3f} mm")
print(f"  [x] Root Fillet Radius         : {root_fillet1:.3f} mm")

print("\n  STAGE 2 GEAR SET (Pinion N3 / Gear N4):")
print(f"  [x] Standard                   : Metric")
print(f"  [x] Pressure Angle             : {phi_deg:.1f} deg")
print(f"  [x] Module (m2)                : {m2:.2f} mm  (Calculated min: {m2_calc:.3f} mm)")
print(f"  [x] Pinion Teeth (N3)          : {N3}")
print(f"  [x] Driven Gear Teeth (N4)     : {N4}")
print(f"  [x] Gear Thickness (b2)        : {b2:.2f} mm")
print(f"  [x] Backlash                   : {backlash2:.3f} mm")
print(f"  [x] Root Fillet Radius         : {root_fillet2:.3f} mm")

print("\n4. KEY GEAR & SHAFT GEOMETRY")
print("-" * 50)
print(f"  • Stage 1 Pitch Diameters       : d1 = {d1:.2f} mm, d2 = {d2:.2f} mm")
print(f"  • Stage 1 Outer Diameters       : d_a1 = {d_a1:.2f} mm, d_a2 = {d_a2:.2f} mm")
print(f"  • Stage 1 Center Distance (a1)  : {a1:.2f} mm")
print(f"  • Stage 2 Pitch Diameters       : d3 = {d3:.2f} mm, d4 = {d4:.2f} mm")
print(f"  • Stage 2 Outer Diameters       : d_a3 = {d_a3:.2f} mm, d_a4 = {d_a4:.2f} mm")
print(f"  • Stage 2 Center Distance (a2)  : {a2:.2f} mm")
print(f"  • Input Shaft Hole Dia (S1)     : {d_shaft_in:.2f} mm  (Calculated min: {d_shaft_in_calc:.2f} mm)")
print(f"      - Pure Torsion Min Dia      : {d_shaft_in_pure_torsion_calc:.2f} mm")
print(f"  • Countershaft Hole Dia (S2)    : {d_shaft_counter:.2f} mm  (Calculated min: {d_shaft_counter_calc:.2f} mm)")
print(f"  • Output Shaft Hole Dia (S3)    : {d_shaft_out:.2f} mm  (Calculated min: {d_shaft_out_calc:.2f} mm)")
print(f"      - Pure Torsion Min Dia      : {d_shaft_out_pure_torsion_calc:.2f} mm")

print("\n5. ESTIMATED MASS METRICS (Cylinder Approximation)")
print("-" * 50)
print(f"  • Pinion 1 Mass (Stage 1)       : {m1_mass:.3f} kg")
print(f"  • Gear 2 Mass (Stage 1)         : {m2_mass:.3f} kg")
print(f"  • Pinion 3 Mass (Stage 2)       : {m3_mass:.3f} kg")
print(f"  • Gear 4 Mass (Stage 2)         : {m4_mass:.3f} kg")
print(f"  • Total Solid Gear Mass         : {total_gear_mass:.3f} kg")
print("=" * 78)