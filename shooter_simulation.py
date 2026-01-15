# -*- coding: utf-8 -*-
"""
FRC 2026 REBUILT - Lob Shooter Trajectory Simulation
=====================================================

This module provides kinematic analysis for a CONSTANT-SPEED LOB SHOOTER
designed for funnel targets (opening faces UP).

KEY INSIGHT: For funnel targets, the ball must DESCEND into the opening.
This requires HIGH-ANGLE lob shots (70-88 degrees) that arc UP above the
target, then fall DOWN through the opening.

RECOMMENDED VELOCITY: 34 ft/s
- Covers full range: 24" (2 ft) to 250" (21 ft)
- Entry angles: 67-88 degrees (steep descent)
- All shots are high lobs that come DOWN into the funnel

Physical Constants (2026 Game Manual):
- FUEL diameter: 5.91 inches
- FUEL mass: ~0.47 lbs (0.213 kg)
- Target height: 72 inches (opening faces UP)
- Target width: 41.7 inches
- Robot max height: 30 inches
- Alliance Zone: 0 to 250 inches from target (corner to corner)

Author: FRC Team 2491 Engineering
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict
import warnings

# =============================================================================
# PHYSICAL CONSTANTS
# =============================================================================

# Gravitational acceleration
G = 386.09  # in/s^2 (imperial)
G_METRIC = 9.81  # m/s^2

# FUEL (game piece) properties
FUEL_DIAMETER_IN = 5.91
FUEL_MASS_LB = 0.47
FUEL_MASS_KG = FUEL_MASS_LB * 0.4536  # 0.213 kg

# Target properties (funnel opening faces UP)
TARGET_HEIGHT_IN = 72
TARGET_WIDTH_IN = 41.7

# Robot constraints
ROBOT_MAX_HEIGHT_IN = 30
ROBOT_PRACTICAL_HEIGHT_IN = 22

# Shooting range
MIN_DISTANCE_IN = 24   # 2 ft minimum
MAX_DISTANCE_IN = 250  # 21 ft maximum (alliance zone corner)

# Recommended muzzle velocity
RECOMMENDED_VELOCITY_FPS = 34
RECOMMENDED_VELOCITY_INS = RECOMMENDED_VELOCITY_FPS * 12  # 408 in/s


# =============================================================================
# CORE TRAJECTORY FUNCTIONS
# =============================================================================

@dataclass
class LobSolution:
    """Solution for a lob trajectory that descends into the target."""
    launch_angle_deg: float
    entry_angle_deg: float  # Descent angle into funnel
    max_height_in: float
    flight_time_s: float
    distance_in: float
    velocity_fps: float

    @property
    def quality(self) -> str:
        """Rate the quality of entry angle for funnel scoring."""
        if self.entry_angle_deg >= 70:
            return "IDEAL"
        elif self.entry_angle_deg >= 55:
            return "GOOD"
        else:
            return "OK"


def find_lob_solution(distance_in: float,
                      velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                      release_height_in: float = 18,
                      target_height_in: float = TARGET_HEIGHT_IN,
                      vx_robot: float = 0,
                      vy_robot: float = 0) -> Optional[LobSolution]:
    """
    Find the optimal lob trajectory to hit the target while DESCENDING.

    The ball must be going DOWN (vy < 0) when it passes through the target
    opening. This is essential for funnel targets that face UP.

    Args:
        distance_in: Horizontal distance from target (inches)
        velocity_fps: Muzzle velocity (ft/s)
        release_height_in: Height of ball release (inches)
        target_height_in: Height of target opening (inches)
        vx_robot: Robot velocity toward target (in/s)
        vy_robot: Robot vertical velocity (in/s)

    Returns:
        LobSolution with best (steepest) entry angle, or None if impossible
    """
    v_shot = velocity_fps * 12  # Convert to in/s
    best = None

    # Search angles from medium to very steep
    for theta_deg in np.linspace(30, 89.95, 3000):
        theta = np.radians(theta_deg)

        # Combined velocity (robot + shot)
        vx0 = v_shot * np.cos(theta) + vx_robot
        vy0 = v_shot * np.sin(theta) + vy_robot

        if vx0 <= 0:
            continue

        # Time to reach target x-position
        t_hit = distance_in / vx0

        # Height and velocity at target
        y_at_target = release_height_in + vy0 * t_hit - 0.5 * G * t_hit**2
        vy_at_target = vy0 - G * t_hit

        # CRITICAL: Ball must be at target height AND descending
        if abs(y_at_target - target_height_in) < 2.5 and vy_at_target < 0:
            # Entry angle (degrees below horizontal)
            entry_angle = np.degrees(np.arctan2(abs(vy_at_target), vx0))

            # Maximum height of trajectory
            t_apex = vy0 / G
            max_height = release_height_in + vy0 * t_apex - 0.5 * G * t_apex**2

            # Keep solution with steepest entry
            if best is None or entry_angle > best.entry_angle_deg:
                best = LobSolution(
                    launch_angle_deg=theta_deg,
                    entry_angle_deg=entry_angle,
                    max_height_in=max_height,
                    flight_time_s=t_hit,
                    distance_in=distance_in,
                    velocity_fps=velocity_fps
                )

    return best


def calculate_trajectory(distance_in: float,
                        launch_angle_deg: float,
                        velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                        release_height_in: float = 18,
                        vx_robot: float = 0,
                        vy_robot: float = 0,
                        dt: float = 0.005) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Calculate the full trajectory for plotting.

    Returns:
        t, x, y: Arrays of time, x-position, y-position
    """
    v_shot = velocity_fps * 12
    theta = np.radians(launch_angle_deg)

    vx0 = v_shot * np.cos(theta) + vx_robot
    vy0 = v_shot * np.sin(theta) + vy_robot

    # Simulate trajectory
    t_max = 3.0  # seconds
    t = np.arange(0, t_max, dt)

    x = distance_in - vx0 * t
    y = release_height_in + vy0 * t - 0.5 * G * t**2

    # Keep valid portion (above ground, not too far past target)
    valid = (y >= 0) & (x >= -30)

    return t[valid], x[valid], y[valid]


# =============================================================================
# ANALYSIS FUNCTIONS
# =============================================================================

def analyze_full_range(velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                      release_height_in: float = 18,
                      vx_robot: float = 0) -> List[Dict]:
    """
    Analyze trajectories across the full shooting range.

    Args:
        velocity_fps: Muzzle velocity
        release_height_in: Release height
        vx_robot: Robot velocity toward target (in/s)

    Returns:
        List of analysis results for each distance
    """
    distances_in = [24, 36, 48, 60, 72, 84, 96, 120, 144, 180, 200, 220, 250]
    results = []

    for dist in distances_in:
        solution = find_lob_solution(dist, velocity_fps, release_height_in,
                                     TARGET_HEIGHT_IN, vx_robot)
        if solution:
            results.append({
                'distance_in': dist,
                'distance_ft': dist / 12,
                'launch_deg': solution.launch_angle_deg,
                'entry_deg': solution.entry_angle_deg,
                'max_height_in': solution.max_height_in,
                'flight_time_s': solution.flight_time_s,
                'quality': solution.quality
            })
        else:
            results.append({
                'distance_in': dist,
                'distance_ft': dist / 12,
                'launch_deg': None,
                'entry_deg': None,
                'max_height_in': None,
                'flight_time_s': None,
                'quality': 'NO SOLUTION'
            })

    return results


def print_range_analysis(velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                        release_height_in: float = 18,
                        vx_robot_fps: float = 0):
    """Print a formatted analysis table."""
    vx_robot = vx_robot_fps * 12
    results = analyze_full_range(velocity_fps, release_height_in, vx_robot)

    robot_str = f", Robot: {vx_robot_fps:+.0f} fps" if vx_robot_fps != 0 else ""

    print("\n" + "="*75)
    print(f"  LOB SHOOTER ANALYSIS")
    print(f"  Velocity: {velocity_fps} fps | Release: {release_height_in}\" | Target: {TARGET_HEIGHT_IN}\"{robot_str}")
    print("="*75)
    print(f"{'Distance':>12} | {'Launch':>10} | {'Entry':>10} | {'Max Ht':>10} | {'Flight':>10} | {'Quality':>8}")
    print("-"*75)

    for r in results:
        if r['launch_deg'] is not None:
            print(f"{r['distance_in']:>6}\" ({r['distance_ft']:>4.1f}ft) | "
                  f"{r['launch_deg']:>8.1f} | {r['entry_deg']:>8.1f} | "
                  f"{r['max_height_in']:>8.0f}\" | {r['flight_time_s']:>8.2f}s | {r['quality']:>8}")
        else:
            print(f"{r['distance_in']:>6}\" ({r['distance_ft']:>4.1f}ft) | {'--- NO SOLUTION ---':^50}")

    print("="*75)
    print("Entry angle = descent angle into funnel (higher = steeper = better)")
    print()


def generate_aiming_table(velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                         release_height_in: float = 18,
                         robot_velocities_fps: List[float] = None) -> np.ndarray:
    """
    Generate a driver aiming lookup table.

    Returns launch angles indexed by [distance, robot_velocity].
    """
    if robot_velocities_fps is None:
        robot_velocities_fps = [-10, -5, 0, 5, 10]

    distances_in = [24, 36, 48, 60, 72, 84, 96, 120, 144, 180, 220, 250]

    table = np.zeros((len(distances_in), len(robot_velocities_fps)))

    for i, dist in enumerate(distances_in):
        for j, v_robot_fps in enumerate(robot_velocities_fps):
            vx_robot = v_robot_fps * 12
            solution = find_lob_solution(dist, velocity_fps, release_height_in,
                                        TARGET_HEIGHT_IN, vx_robot)
            table[i, j] = solution.launch_angle_deg if solution else np.nan

    return distances_in, robot_velocities_fps, table


def print_aiming_table(velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                      release_height_in: float = 18):
    """Print a formatted aiming lookup table for drivers."""
    robot_vels = [-10, -5, 0, 5, 10]
    distances, _, table = generate_aiming_table(velocity_fps, release_height_in, robot_vels)

    print("\n" + "="*70)
    print(f"  DRIVER AIMING TABLE - Launch Angles (degrees)")
    print(f"  Velocity: {velocity_fps} fps | Release: {release_height_in}\"")
    print("="*70)

    header = f"{'Distance':>12} |"
    for v in robot_vels:
        header += f" {v:>+5} fps |"
    print(header)
    print("-"*70)

    for i, dist in enumerate(distances):
        row = f"{dist:>6}\" ({dist/12:>4.1f}ft) |"
        for j in range(len(robot_vels)):
            val = table[i, j]
            if np.isnan(val):
                row += "   --- |"
            else:
                row += f" {val:>6.1f} |"
        print(row)

    print("="*70)
    print("Negative velocity = driving away from target")
    print("Positive velocity = driving toward target")
    print()


# =============================================================================
# FLYWHEEL CALCULATIONS
# =============================================================================

def calculate_flywheel_requirements(velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                                   num_balls: int = 3,
                                   max_rpm_drop_pct: float = 5.0,
                                   wheel_radius_in: float = 2.0,
                                   num_wheels: int = 2) -> Dict:
    """
    Calculate flywheel requirements for multi-ball firing.

    For a lob shooter at 34 fps, the energy requirements are much lower
    than a flat shooter at 45+ fps, making the flywheel design easier.
    """
    # Convert units
    v_shot_m_s = velocity_fps * 12 * 0.0254
    wheel_radius_m = wheel_radius_in * 0.0254

    # Energy per ball
    E_ball = 0.5 * FUEL_MASS_KG * v_shot_m_s**2
    E_total = num_balls * E_ball

    # Angular velocity
    omega = v_shot_m_s / wheel_radius_m
    rpm = omega * 60 / (2 * np.pi)

    # For specified RPM drop
    drop_fraction = max_rpm_drop_pct / 100
    energy_fraction = 1 - (1 - drop_fraction)**2

    # Required stored energy
    E_required = E_total / energy_fraction

    # Required inertia
    I_total = 2 * E_required / omega**2
    I_per_wheel = I_total / num_wheels

    # Wheel mass estimate (solid disk: I = 0.5 * m * r^2)
    wheel_mass_kg = 2 * I_per_wheel / wheel_radius_m**2
    wheel_mass_lb = wheel_mass_kg / 0.4536

    # Spin-up time (2 NEO motors, ~2.6 N-m average)
    motor_torque = num_wheels * 2.6
    spinup_time = I_total * omega / motor_torque

    return {
        'velocity_fps': velocity_fps,
        'rpm': rpm,
        'energy_per_ball_J': E_ball,
        'total_energy_J': E_total,
        'stored_energy_J': E_required,
        'I_total_kg_m2': I_total,
        'I_per_wheel_lb_in2': I_per_wheel / (0.0254**2 * 0.4536),
        'wheel_mass_lb': wheel_mass_lb,
        'spinup_time_s': spinup_time
    }


def print_flywheel_analysis(velocity_fps: float = RECOMMENDED_VELOCITY_FPS):
    """Print flywheel requirements."""
    r = calculate_flywheel_requirements(velocity_fps)

    print("\n" + "="*60)
    print(f"  FLYWHEEL REQUIREMENTS FOR LOB SHOOTER")
    print(f"  Velocity: {velocity_fps} fps | 3 balls | <5% RPM drop")
    print("="*60)
    print(f"\n  Operating Point:")
    print(f"    Flywheel RPM (4\" wheel): {r['rpm']:.0f}")
    print(f"\n  Energy Analysis:")
    print(f"    Energy per ball: {r['energy_per_ball_J']:.2f} J")
    print(f"    Energy for 3 balls: {r['total_energy_J']:.2f} J")
    print(f"    Required stored energy: {r['stored_energy_J']:.1f} J")
    print(f"\n  Wheel Design (4\" diameter):")
    print(f"    Inertia needed: {r['I_per_wheel_lb_in2']:.2f} lb-in² per wheel")
    print(f"    Mass estimate: {r['wheel_mass_lb']:.2f} lb per wheel")
    print(f"    Spin-up time (2 NEOs): {r['spinup_time_s']:.2f} s")
    print("="*60)


# =============================================================================
# VISUALIZATION FUNCTIONS
# =============================================================================

def plot_trajectory_sweep(velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                         release_height_in: float = 18,
                         distances_ft: List[float] = None,
                         save_path: str = None):
    """
    Plot trajectories for multiple distances showing the lob arcs.
    """
    if distances_ft is None:
        distances_ft = [2, 4, 7, 10, 15, 21]

    fig, ax = plt.subplots(figsize=(14, 10))

    colors = plt.cm.viridis(np.linspace(0.2, 0.9, len(distances_ft)))

    for d_ft, color in zip(distances_ft, colors):
        dist_in = d_ft * 12
        solution = find_lob_solution(dist_in, velocity_fps, release_height_in)

        if solution:
            t, x, y = calculate_trajectory(dist_in, solution.launch_angle_deg,
                                          velocity_fps, release_height_in)
            ax.plot(x, y, color=color, linewidth=2.5,
                   label=f'{d_ft} ft: {solution.launch_angle_deg:.0f}° launch → '
                         f'{solution.entry_angle_deg:.0f}° entry')

    # Draw funnel target
    funnel_w = TARGET_WIDTH_IN / 2
    ax.plot([0, -10], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN-25], 'k-', linewidth=4)
    ax.plot([0, 10], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN-25], 'k-', linewidth=4)
    ax.plot([-funnel_w, funnel_w], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN], 'g-', linewidth=6)
    ax.fill_between([-funnel_w, funnel_w], TARGET_HEIGHT_IN-3, TARGET_HEIGHT_IN+3,
                    color='green', alpha=0.3)
    ax.annotate('FUNNEL TARGET\n(opening faces UP)', (0, TARGET_HEIGHT_IN+15),
                ha='center', fontsize=11, fontweight='bold')

    # Ground
    ax.axhline(y=0, color='saddlebrown', linewidth=4)
    ax.fill_between([-30, 280], -10, 0, color='saddlebrown', alpha=0.3)

    ax.set_xlim(-30, 280)
    ax.set_ylim(-15, 260)
    ax.set_xlabel('Distance from target (inches)', fontsize=12)
    ax.set_ylabel('Height (inches)', fontsize=12)
    ax.set_title(f'Lob Trajectories: {velocity_fps} fps Muzzle Velocity\n'
                f'Ball arcs UP then DESCENDS into funnel', fontsize=14)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


def plot_entry_angle_chart(release_height_in: float = 18,
                          save_path: str = None):
    """
    Plot entry angle vs distance for different velocities.
    """
    fig, ax = plt.subplots(figsize=(12, 8))

    # Entry angle zones
    ax.axhspan(70, 90, color='green', alpha=0.2, label='IDEAL (>70°)')
    ax.axhspan(55, 70, color='yellow', alpha=0.2, label='GOOD (55-70°)')
    ax.axhspan(0, 55, color='red', alpha=0.1, label='OK (<55°)')

    distances = np.linspace(24, 250, 50)

    for v_fps in [30, 34, 38, 42]:
        entry_angles = []
        valid_dists = []

        for dist in distances:
            solution = find_lob_solution(dist, v_fps, release_height_in)
            if solution:
                entry_angles.append(solution.entry_angle_deg)
                valid_dists.append(dist / 12)  # Convert to feet

        if entry_angles:
            ax.plot(valid_dists, entry_angles, 'o-', linewidth=2, markersize=4,
                   label=f'{v_fps} fps')

    ax.axvline(x=RECOMMENDED_VELOCITY_FPS, color='blue', linestyle=':', alpha=0.5)

    ax.set_xlabel('Distance from target (feet)', fontsize=12)
    ax.set_ylabel('Entry Angle (degrees descent)', fontsize=12)
    ax.set_title('Entry Angle vs Distance\n'
                'Higher entry angle = steeper descent = better for funnel', fontsize=14)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 22)
    ax.set_ylim(50, 90)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


def plot_robot_velocity_effect(distance_ft: float = 10,
                              velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                              release_height_in: float = 18,
                              save_path: str = None):
    """
    Show how robot motion affects the required launch angle.
    """
    dist_in = distance_ft * 12

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

    # Left: Trajectories at different robot velocities
    robot_vels_fps = [-10, -5, 0, 5, 10]
    colors = plt.cm.coolwarm(np.linspace(0, 1, len(robot_vels_fps)))

    for v_robot_fps, color in zip(robot_vels_fps, colors):
        vx_robot = v_robot_fps * 12
        solution = find_lob_solution(dist_in, velocity_fps, release_height_in,
                                    TARGET_HEIGHT_IN, vx_robot)

        if solution:
            t, x, y = calculate_trajectory(dist_in, solution.launch_angle_deg,
                                          velocity_fps, release_height_in, vx_robot)
            direction = "toward" if v_robot_fps > 0 else "away" if v_robot_fps < 0 else "stationary"
            ax1.plot(x, y, color=color, linewidth=2.5,
                    label=f'{abs(v_robot_fps)} fps {direction}: {solution.launch_angle_deg:.0f}°')

    # Target
    ax1.axvline(x=0, color='green', linewidth=4, alpha=0.7)
    ax1.add_patch(Rectangle((-5, TARGET_HEIGHT_IN-5), 10, 10, color='green', alpha=0.4))
    ax1.axhline(y=0, color='saddlebrown', linewidth=4)

    ax1.set_xlim(-20, dist_in + 30)
    ax1.set_ylim(-10, 260)
    ax1.set_xlabel('Distance from target (inches)', fontsize=12)
    ax1.set_ylabel('Height (inches)', fontsize=12)
    ax1.set_title(f'Robot Motion Effect: {distance_ft} ft shot, {velocity_fps} fps', fontsize=12)
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)

    # Right: Angle compensation chart
    robot_vels_range = np.linspace(-15, 15, 50)
    angles = []

    for v_robot_fps in robot_vels_range:
        vx_robot = v_robot_fps * 12
        solution = find_lob_solution(dist_in, velocity_fps, release_height_in,
                                    TARGET_HEIGHT_IN, vx_robot)
        angles.append(solution.launch_angle_deg if solution else np.nan)

    ax2.plot(robot_vels_range, angles, 'b-', linewidth=3)
    ax2.axvline(x=0, color='gray', linestyle=':', alpha=0.5)

    ax2.annotate('Driving TOWARD target\n→ higher angle needed',
                xy=(8, max(angles)-5), fontsize=11, ha='center', color='red')
    ax2.annotate('Driving AWAY\n→ lower angle',
                xy=(-8, min([a for a in angles if not np.isnan(a)])+5),
                fontsize=11, ha='center', color='blue')

    ax2.set_xlabel('Robot velocity toward target (ft/s)', fontsize=12)
    ax2.set_ylabel('Required launch angle (degrees)', fontsize=12)
    ax2.set_title('Angle Compensation Chart\nUse for driver practice!', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(-15, 15)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


def plot_aiming_heatmap(velocity_fps: float = RECOMMENDED_VELOCITY_FPS,
                       release_height_in: float = 18,
                       save_path: str = None):
    """
    Create a heatmap lookup table for driver practice.
    """
    distances, robot_vels, table = generate_aiming_table(velocity_fps, release_height_in)

    fig, ax = plt.subplots(figsize=(12, 10))

    im = ax.imshow(table, aspect='auto', cmap='viridis',
                   extent=[robot_vels[0]-2.5, robot_vels[-1]+2.5,
                          len(distances)-0.5, -0.5])

    # Add text annotations
    for i in range(len(distances)):
        for j in range(len(robot_vels)):
            val = table[i, j]
            if not np.isnan(val):
                ax.text(robot_vels[j], i, f'{val:.0f}°', ha='center', va='center',
                       fontsize=9, color='white', fontweight='bold')

    # Y-axis labels
    ax.set_yticks(range(len(distances)))
    ax.set_yticklabels([f'{d}" ({d/12:.1f}ft)' for d in distances])

    ax.set_xlabel('Robot velocity toward target (ft/s)', fontsize=12)
    ax.set_ylabel('Distance from target', fontsize=12)
    ax.set_title(f'Launch Angle Lookup Table\n{velocity_fps} fps muzzle velocity', fontsize=14)

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Launch Angle (degrees)', fontsize=11)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


def create_all_plots(output_dir: str = "."):
    """Generate all visualization plots."""
    import os

    print("Generating lob shooter visualizations...")

    plot_trajectory_sweep(save_path=os.path.join(output_dir, "lob_trajectories.png"))
    plot_entry_angle_chart(save_path=os.path.join(output_dir, "entry_angles.png"))
    plot_robot_velocity_effect(save_path=os.path.join(output_dir, "robot_velocity_effect.png"))
    plot_aiming_heatmap(save_path=os.path.join(output_dir, "aiming_table.png"))

    print("\nAll plots saved!")


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    print("\n" + "="*60)
    print("  FRC 2026 REBUILT - Lob Shooter Analysis")
    print("  Team 2491 Engineering Toolkit")
    print("="*60)

    print(f"\n  RECOMMENDED VELOCITY: {RECOMMENDED_VELOCITY_FPS} fps")
    print(f"  Coverage: {MIN_DISTANCE_IN}\" to {MAX_DISTANCE_IN}\" (2 ft to 21 ft)")

    # Print analysis
    print_range_analysis()

    # Print aiming table
    print_aiming_table()

    # Print flywheel requirements
    print_flywheel_analysis()

    print("\nTo generate plots, run: python shooter_simulation.py --plots")
    print("Or import as module and call create_all_plots()")
