# -*- coding: utf-8 -*-
"""Run shooter analysis with proper encoding."""
import sys
import io

# Force UTF-8 output
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

from shooter_simulation import *

print("\n" + "="*90)
print("  HIGH vs LOW ANGLE TRAJECTORY COMPARISON")
print("  Release height: 18.0 in | Muzzle velocity: 540 in/s (45.0 ft/s)")
print("="*90)

distances_ft = [3, 5, 7, 9, 11]
y0 = 18
v_shot = 45 * 12

print(f"{'Dist':>6} | {'--- LOW ANGLE ---':^35} | {'--- HIGH ANGLE ---':^35}")
print(f"{'(ft)':>6} | {'Angle':>7} {'T_flight':>9} {'Entry':>7} {'MaxHt':>8} | {'Angle':>7} {'T_flight':>9} {'Entry':>7} {'MaxHt':>8}")
print("-"*90)

for d_ft in distances_ft:
    x0 = d_ft * 12
    results = compare_high_low_angles(x0, y0, v_shot, 0, 0)

    low = results['low']
    high = results['high']

    low_str = (f"{low.angle_deg:>6.1f}deg {low.time_of_flight:>8.3f}s {low.entry_angle_deg:>6.1f}deg {low.max_height:>7.1f}in"
               if low else "      --- NO SOLUTION ---      ")
    high_str = (f"{high.angle_deg:>6.1f}deg {high.time_of_flight:>8.3f}s {high.entry_angle_deg:>6.1f}deg {high.max_height:>7.1f}in"
                if high else "      --- NO SOLUTION ---      ")

    print(f"{d_ft:>6.1f} | {low_str} | {high_str}")

print("="*90)
print("Entry angle: angle ball enters goal (positive = descending)")
print("MaxHt: maximum trajectory height")
print()

# Now with robot motion
print("\n" + "="*90)
print("  WITH ROBOT MOVING 10 ft/s TOWARD TARGET")
print("="*90)

vx_robot = 10 * 12  # 10 ft/s toward target

print(f"{'Dist':>6} | {'--- LOW ANGLE ---':^35} | {'--- HIGH ANGLE ---':^35}")
print(f"{'(ft)':>6} | {'Angle':>7} {'T_flight':>9} {'Entry':>7} {'MaxHt':>8} | {'Angle':>7} {'T_flight':>9} {'Entry':>7} {'MaxHt':>8}")
print("-"*90)

for d_ft in distances_ft:
    x0 = d_ft * 12
    results = compare_high_low_angles(x0, y0, v_shot, vx_robot, 0)

    low = results['low']
    high = results['high']

    low_str = (f"{low.angle_deg:>6.1f}deg {low.time_of_flight:>8.3f}s {low.entry_angle_deg:>6.1f}deg {low.max_height:>7.1f}in"
               if low else "      --- NO SOLUTION ---      ")
    high_str = (f"{high.angle_deg:>6.1f}deg {high.time_of_flight:>8.3f}s {high.entry_angle_deg:>6.1f}deg {high.max_height:>7.1f}in"
                if high else "      --- NO SOLUTION ---      ")

    print(f"{d_ft:>6.1f} | {low_str} | {high_str}")

print("="*90)

# Flywheel analysis
print("\n" + "="*70)
print("  FLYWHEEL INERTIA ANALYSIS FOR MULTI-BALL SHOOTER")
print("="*70)

v_shot_fps = 45
results = calculate_required_inertia(v_shot_fps * 12, 3, 5.0)

print(f"\n  REQUIREMENTS:")
print(f"  * Ball exit velocity: {v_shot_fps:.1f} ft/s ({v_shot_fps*12:.0f} in/s)")
print(f"  * Balls fired per burst: 3")
print(f"  * Maximum RPM drop: 5.0%")

print(f"\n  OPERATING POINT:")
op = results['operating_point']
print(f"  * Flywheel RPM: {op['rpm']:.0f}")
print(f"  * Angular velocity: {op['omega_rad_s']:.1f} rad/s")

print(f"\n  ENERGY ANALYSIS:")
ea = results['energy_analysis']
print(f"  * Energy per ball: {ea['energy_per_ball_J']:.2f} J")
print(f"  * Total energy for 3 balls: {ea['total_energy_needed_J']:.2f} J")
print(f"  * Required flywheel energy: {ea['flywheel_energy_required_J']:.1f} J")
print(f"  * Energy transfer ratio: {ea['energy_fraction_transferred']*100:.1f}%")

print(f"\n  INERTIA REQUIREMENTS:")
ir = results['inertia_requirements']
print(f"  * Total moment of inertia: {ir['total_I_kg_m2']*1000:.2f} g*m^2 = {ir['per_wheel_I_lb_in2']*2:.2f} lb*in^2")
print(f"  * Per wheel (assuming 2 wheels): {ir['per_wheel_I_lb_in2']:.3f} lb*in^2")

print(f"\n  DESIGN ESTIMATES (for 2\" radius solid disk):")
sd = results['solid_disk_estimates']
print(f"  * Required wheel mass: {sd['wheel_mass_lb']:.2f} lb each")
print(f"  * Steel disk thickness: {sd['steel_thickness_in']:.2f}\"")
print(f"  * Aluminum disk thickness: {sd['aluminum_thickness_in']:.2f}\"")

print("="*70)

# Wheel design comparison
print("\n" + "="*70)
print("  FLYWHEEL DESIGN COMPARISON")
print("  Target: 45 ft/s exit velocity, 3 balls, <5% RPM drop")
print("="*70)
print(f"{'Design':^20} | {'Radius':>8} | {'RPM':>7} | {'Mass/wheel':>10} | {'Spin-up*':>8}")
print("-"*70)

designs = [
    ("2\" Wheels", 2.0),
    ("3\" Wheels", 3.0),
    ("4\" Wheels", 4.0),
    ("6\" Wheels", 6.0),
]

for name, radius in designs:
    results = calculate_required_inertia(45*12, 3, 5.0, radius, 2)
    rpm = results['operating_point']['rpm']
    mass = results['solid_disk_estimates']['wheel_mass_lb']

    I = results['inertia_requirements']['total_I_kg_m2']
    omega = results['operating_point']['omega_rad_s']
    motor_torque = 2 * 2.6  # 2 NEOs
    spinup_time = I * omega / motor_torque

    print(f"{name:^20} | {radius:>6.1f}\" | {rpm:>7.0f} | {mass:>8.2f} lb | {spinup_time:>6.2f} s")

print("-"*70)
print("* Spin-up time estimated with 2 NEO motors")
print("="*70)

# Aiming lookup table
print("\n" + "="*70)
print("  DRIVER AIMING LOOKUP TABLE (High Angle)")
print("  Release height: 18\" | Muzzle velocity: 45 ft/s")
print("="*70)

distances = list(range(3, 13))
robot_vels = [-5, 0, 5, 10]

print(f"{'Distance':>10} |", end='')
for v in robot_vels:
    print(f" {v:>6} fps |", end='')
print()
print("-"*70)

for d_ft in distances:
    x0 = d_ft * 12
    print(f"{d_ft:>8} ft |", end='')
    for v_robot_fps in robot_vels:
        vx_robot = v_robot_fps * 12
        angle = find_launch_angle(x0, 18, TARGET_HEIGHT_IN, 45*12, vx_robot, 0, prefer_high=True)
        if angle is None:
            print(f"    --- |", end='')
        else:
            print(f" {np.degrees(angle):>6.1f}deg|", end='')
    print()

print("="*70)
print("Negative velocity = driving away from target")
print("Positive velocity = driving toward target")
