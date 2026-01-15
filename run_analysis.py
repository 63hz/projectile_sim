# -*- coding: utf-8 -*-
"""
Run lob shooter analysis with proper encoding.

This script demonstrates the lob trajectory calculations for the
2026 REBUILT game funnel target.
"""
import sys
import io

# Force UTF-8 output
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

from shooter_simulation import (
    find_lob_solution, analyze_full_range, print_range_analysis,
    print_aiming_table, print_flywheel_analysis, calculate_flywheel_requirements,
    RECOMMENDED_VELOCITY_FPS, TARGET_HEIGHT_IN, G
)
import numpy as np


def main():
    print("\n" + "="*80)
    print("  LOB SHOOTER TRAJECTORY ANALYSIS")
    print("  FRC 2026 REBUILT - Team 2491")
    print("="*80)

    # Full range analysis
    print_range_analysis(velocity_fps=30, release_height_in=18)

    # Aiming lookup table
    print_aiming_table(velocity_fps=30, release_height_in=18)

    # Flywheel requirements
    print_flywheel_analysis(velocity_fps=30)

    # Robot motion analysis
    print("\n" + "="*80)
    print("  ROBOT MOTION EFFECT ON LOB SHOTS")
    print("  Distance: 7 ft | Velocity: 30 fps")
    print("="*80)

    distance_in = 7 * 12
    velocity_fps = 30
    release_height = 18

    print(f"{'Robot vel':>12} | {'Launch':>10} | {'Entry':>10} | {'Max Ht':>10} | {'Quality'}")
    print("-"*70)

    for v_robot_fps in [-10, -5, 0, 5, 10]:
        vx_robot = v_robot_fps * 12
        sol = find_lob_solution(distance_in, velocity_fps, release_height,
                               TARGET_HEIGHT_IN, vx_robot)
        if sol:
            print(f"{v_robot_fps:>+10} fps | {sol.launch_angle_deg:>8.1f}° | "
                  f"{sol.entry_angle_deg:>8.1f}° | {sol.max_height_in:>8.0f}\" | "
                  f"{sol.quality}")
        else:
            print(f"{v_robot_fps:>+10} fps | {'--- NO SOLUTION ---':^50}")

    print("="*80)
    print("\nEntry angle = descent angle into funnel (higher = steeper = better)")

    # Wheel size comparison
    print("\n" + "="*70)
    print("  FLYWHEEL DESIGN COMPARISON")
    print("  Target: 30 ft/s exit velocity, 3 balls, <5% RPM drop")
    print("="*70)
    print(f"{'Diameter':>10} | {'RPM':>8} | {'Mass/wheel':>12} | {'Spin-up*':>10}")
    print("-"*70)

    for diameter in [4, 6, 8]:
        radius = diameter / 2
        r = calculate_flywheel_requirements(30, 3, 5.0, radius, 2)
        spinup_time = r['spinup_time_s']

        print(f"{diameter:>8}\" | {r['rpm']:>8.0f} | {r['wheel_mass_lb']:>10.2f} lb | "
              f"{spinup_time:>8.2f} s")

    print("-"*70)
    print("* Spin-up time estimated with 2 NEO motors")
    print("="*70)


if __name__ == "__main__":
    main()
