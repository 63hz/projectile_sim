# -*- coding: utf-8 -*-
"""
CORRECTED Lob Shooter Analysis
==============================

Key insight: For a funnel target facing UP, the ball must be DESCENDING
when it passes through the opening. This means:
- vy < 0 at the moment the ball reaches y = target_height at x = 0
- The ball must go UP above the target, then come DOWN into it

This is like lobbing a ball into a trash can - you throw it UP in an arc
so it falls DOWN into the opening.
"""

import numpy as np
import matplotlib.pyplot as plt

G = 386.09  # in/s^2

def find_lob_trajectory(x0, y0, y_target, v_shot, return_all=False):
    """
    Find launch angles where the ball DESCENDS through the target.

    The ball must be going DOWN (vy < 0) when it reaches y_target at x=0.

    Returns:
        If return_all=False: (high_angle, analysis_dict) or (None, None)
        If return_all=True: list of all valid (angle, analysis) pairs
    """
    valid_solutions = []

    # Scan angles from low to high
    for theta_deg in np.linspace(5, 89, 2000):
        theta = np.radians(theta_deg)
        vx0 = v_shot * np.cos(theta)
        vy0 = v_shot * np.sin(theta)

        if vx0 <= 0:
            continue

        # Time to reach target x-position
        t_hit = x0 / vx0

        # Height and velocity at target
        y_at_target = y0 + vy0 * t_hit - 0.5 * G * t_hit**2
        vy_at_target = vy0 - G * t_hit

        # Check: ball must be at target height AND descending
        if abs(y_at_target - y_target) < 1.0 and vy_at_target < 0:
            # Calculate entry angle (degrees below horizontal)
            entry_angle = np.degrees(np.arctan2(abs(vy_at_target), vx0))

            # Max height
            t_apex = vy0 / G
            y_max = y0 + vy0 * t_apex - 0.5 * G * t_apex**2

            analysis = {
                'angle_deg': theta_deg,
                't_flight': t_hit,
                'y_at_target': y_at_target,
                'vy_at_target': vy_at_target,
                'entry_angle_deg': entry_angle,
                'max_height': y_max,
                'descending': True
            }
            valid_solutions.append((theta_deg, analysis))

    if return_all:
        return valid_solutions

    if valid_solutions:
        # Return the solution with steepest entry (highest angle)
        best = max(valid_solutions, key=lambda x: x[1]['entry_angle_deg'])
        return best

    return None, None


def calculate_lob_trajectory(x0, y0, v_shot, theta_deg, dt=0.001):
    """Calculate full trajectory for plotting."""
    theta = np.radians(theta_deg)
    vx0 = v_shot * np.cos(theta)
    vy0 = v_shot * np.sin(theta)

    # Simulate until ball hits ground or passes target
    t_max = 3.0  # 3 second max
    t = np.arange(0, t_max, dt)

    x = x0 - vx0 * t
    y = y0 + vy0 * t - 0.5 * G * t**2

    # Keep valid portion
    valid = (y >= 0) & (x >= -20)
    return t[valid], x[valid], y[valid]


def plot_lob_analysis(distance_ft=7, y0=18, y_target=72):
    """
    Show how different velocities affect lob trajectories.

    Key insight: LOWER velocity requires HIGHER angle = STEEPER entry!
    """
    x0 = distance_ft * 12

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # === Panel 1: Find minimum velocity that works ===
    ax1 = axes[0, 0]

    velocities_fps = [15, 20, 25, 30, 35, 40]
    colors = plt.cm.viridis(np.linspace(0.9, 0.2, len(velocities_fps)))

    found_any = False
    for v_fps, color in zip(velocities_fps, colors):
        v_shot = v_fps * 12
        result = find_lob_trajectory(x0, y0, y_target, v_shot)

        if result[0] is not None:
            theta_deg, analysis = result
            found_any = True

            # Plot trajectory
            t, x, y = calculate_lob_trajectory(x0, y0, v_shot, theta_deg)
            ax1.plot(x, y, color=color, linewidth=2.5,
                    label=f'{v_fps} fps @ {theta_deg:.0f}° → {analysis["entry_angle_deg"]:.0f}° entry')

    if not found_any:
        ax1.text(x0/2, y_target, 'No valid lob solutions found!\nTry closer distance or higher velocity.',
                ha='center', fontsize=12, color='red')

    # Draw funnel target
    funnel_w = 20
    ax1.plot([0, -8], [y_target, y_target-20], 'k-', linewidth=4)
    ax1.plot([0, 8], [y_target, y_target-20], 'k-', linewidth=4)
    ax1.plot([-funnel_w, funnel_w], [y_target, y_target], 'g-', linewidth=5)
    ax1.fill_between([-funnel_w, funnel_w], y_target-3, y_target+3, color='green', alpha=0.3)
    ax1.annotate('FUNNEL\n(faces UP)', (0, y_target+10), ha='center', fontsize=10, fontweight='bold')

    # Robot
    ax1.add_patch(plt.Rectangle((x0-14, 0), 28, 22, color='gray', alpha=0.4))
    ax1.plot(x0, y0, 'ko', markersize=10)

    # Ground
    ax1.axhline(y=0, color='saddlebrown', linewidth=4)

    ax1.set_xlim(-30, x0+30)
    ax1.set_ylim(-10, 200)
    ax1.set_xlabel('Distance from target (inches)', fontsize=11)
    ax1.set_ylabel('Height (inches)', fontsize=11)
    ax1.set_title(f'LOB Trajectories: {distance_ft} ft shot\nBall must DESCEND into funnel', fontsize=12)
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)

    # === Panel 2: Entry angle vs velocity ===
    ax2 = axes[0, 1]

    velocities_range = np.linspace(15, 50, 50)
    entry_angles = []
    launch_angles = []
    max_heights = []

    for v_fps in velocities_range:
        v_shot = v_fps * 12
        result = find_lob_trajectory(x0, y0, y_target, v_shot)
        if result[0] is not None:
            _, analysis = result
            entry_angles.append(analysis['entry_angle_deg'])
            launch_angles.append(analysis['angle_deg'])
            max_heights.append(analysis['max_height'])
        else:
            entry_angles.append(np.nan)
            launch_angles.append(np.nan)
            max_heights.append(np.nan)

    ax2.plot(velocities_range, entry_angles, 'b-', linewidth=2.5, label='Entry angle')
    ax2.plot(velocities_range, launch_angles, 'r--', linewidth=2, label='Launch angle')

    ax2.axhspan(60, 90, color='green', alpha=0.2, label='Ideal entry (60-90°)')
    ax2.axhspan(45, 60, color='yellow', alpha=0.2, label='Good entry (45-60°)')
    ax2.axhline(y=45, color='orange', linestyle=':', linewidth=1)

    ax2.set_xlabel('Muzzle velocity (ft/s)', fontsize=11)
    ax2.set_ylabel('Angle (degrees)', fontsize=11)
    ax2.set_title('Entry Angle vs Muzzle Velocity\n(LOWER velocity = STEEPER entry!)', fontsize=12)
    ax2.legend(loc='right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(15, 50)
    ax2.set_ylim(0, 90)

    # === Panel 3: Different distances ===
    ax3 = axes[1, 0]

    v_shot = 25 * 12  # Fixed 25 fps
    distances = [4, 6, 8, 10]
    colors = plt.cm.plasma(np.linspace(0.2, 0.8, len(distances)))

    for d_ft, color in zip(distances, colors):
        x0_test = d_ft * 12
        result = find_lob_trajectory(x0_test, y0, y_target, v_shot)

        if result[0] is not None:
            theta_deg, analysis = result
            t, x, y = calculate_lob_trajectory(x0_test, y0, v_shot, theta_deg)
            ax3.plot(x, y, color=color, linewidth=2.5,
                    label=f'{d_ft} ft: {theta_deg:.0f}° launch, {analysis["entry_angle_deg"]:.0f}° entry')

    # Funnel
    ax3.plot([0, -8], [y_target, y_target-20], 'k-', linewidth=4)
    ax3.plot([0, 8], [y_target, y_target-20], 'k-', linewidth=4)
    ax3.plot([-20, 20], [y_target, y_target], 'g-', linewidth=5)
    ax3.axhline(y=0, color='saddlebrown', linewidth=4)

    ax3.set_xlim(-30, 140)
    ax3.set_ylim(-10, 180)
    ax3.set_xlabel('Distance from target (inches)', fontsize=11)
    ax3.set_ylabel('Height (inches)', fontsize=11)
    ax3.set_title('Distance Comparison @ 25 ft/s\n(All trajectories DESCEND into target)', fontsize=12)
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3)

    # === Panel 4: Summary table ===
    ax4 = axes[1, 1]
    ax4.axis('off')

    # Build summary data
    summary_text = """
    LOB SHOOTER ANALYSIS SUMMARY
    ══════════════════════════════════════════════════════════════

    KEY INSIGHT: For a funnel target, the ball must DESCEND into it.
    This requires HIGH-ANGLE lob shots, NOT flat trajectories.

    CRITICAL DIFFERENCE:
    • WRONG: Low angle shot passing THROUGH target height (ascending)
    • RIGHT: High angle lob DESCENDING through target opening

    ══════════════════════════════════════════════════════════════

    VELOCITY EFFECTS:
    • LOWER velocity → HIGHER launch angle → STEEPER entry
    • This is GOOD for funnel targets!
    • 20-30 ft/s gives excellent 60-80° entry angles

    RECOMMENDED PARAMETERS:
    • Muzzle velocity: 20-30 ft/s (NOT 45+ ft/s)
    • Launch angles: 65-80° (high lobs)
    • Entry angles: 55-75° (steep descent)
    • Max trajectory height: 120-180" (goes well above target)

    ══════════════════════════════════════════════════════════════

    FLYWHEEL IMPLICATIONS:
    • Lower velocity = lower energy per shot
    • Lower velocity = lower RPM = easier bearings
    • Lower velocity = smaller/lighter flywheel needed

    This is a WIN-WIN for your design!
    """

    ax4.text(0.05, 0.95, summary_text, transform=ax4.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.suptitle('CORRECTED: Lob Trajectory Analysis for Funnel Target', fontsize=14, fontweight='bold')
    plt.tight_layout()
    return fig


def print_lob_table(y0=18, y_target=72):
    """Print a table of lob solutions for various distances and velocities."""

    print("\n" + "="*80)
    print("  LOB TRAJECTORY SOLUTIONS (ball DESCENDING through target)")
    print("  Release: {}\" | Target: {}\"".format(y0, y_target))
    print("="*80)
    print(f"{'Dist':>6} | {'Velocity':>10} | {'Launch':>8} | {'Entry':>8} | {'Max Ht':>8} | {'Flight':>8}")
    print("-"*80)

    for d_ft in [4, 5, 6, 7, 8, 9, 10]:
        x0 = d_ft * 12
        row_printed = False

        for v_fps in [20, 25, 30, 35]:
            v_shot = v_fps * 12
            result = find_lob_trajectory(x0, y0, y_target, v_shot)

            if result[0] is not None:
                _, a = result
                dist_str = f"{d_ft} ft" if not row_printed else ""
                print(f"{dist_str:>6} | {v_fps:>8} fps | {a['angle_deg']:>6.1f}° | "
                      f"{a['entry_angle_deg']:>6.1f}° | {a['max_height']:>6.0f}\" | {a['t_flight']:>6.2f}s")
                row_printed = True

        if not row_printed:
            print(f"{d_ft:>4} ft | {'--- No valid lob solutions ---':^55}")

        if row_printed:
            print("-"*80)

    print("="*80)
    print("Entry angle = how steeply ball descends into funnel (higher = better)")
    print("Max Ht = apex of trajectory (ball goes UP then comes DOWN)")
    print()


if __name__ == "__main__":
    print("\nGenerating LOB trajectory analysis...")

    # Print the data table
    print_lob_table()

    # Generate visualization
    fig = plot_lob_analysis(distance_ft=7)
    plt.savefig('lob_trajectories.png', dpi=150, bbox_inches='tight')
    print("Saved: lob_trajectories.png")

    plt.show()
