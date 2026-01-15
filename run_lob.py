# -*- coding: utf-8 -*-
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

G = 386.09  # in/s^2

def find_lob_trajectory(x0, y0, y_target, v_shot):
    """Find launch angle where ball DESCENDS through target."""
    valid_solutions = []

    for theta_deg in np.linspace(5, 89, 2000):
        theta = np.radians(theta_deg)
        vx0 = v_shot * np.cos(theta)
        vy0 = v_shot * np.sin(theta)

        if vx0 <= 0:
            continue

        t_hit = x0 / vx0
        y_at_target = y0 + vy0 * t_hit - 0.5 * G * t_hit**2
        vy_at_target = vy0 - G * t_hit

        # Ball must be at target height AND descending (vy < 0)
        if abs(y_at_target - y_target) < 1.0 and vy_at_target < 0:
            entry_angle = np.degrees(np.arctan2(abs(vy_at_target), vx0))
            t_apex = vy0 / G
            y_max = y0 + vy0 * t_apex - 0.5 * G * t_apex**2

            valid_solutions.append({
                'angle_deg': theta_deg,
                't_flight': t_hit,
                'entry_angle_deg': entry_angle,
                'max_height': y_max,
            })

    if valid_solutions:
        return max(valid_solutions, key=lambda x: x['entry_angle_deg'])
    return None


def calculate_trajectory(x0, y0, v_shot, theta_deg):
    theta = np.radians(theta_deg)
    vx0 = v_shot * np.cos(theta)
    vy0 = v_shot * np.sin(theta)

    t = np.linspace(0, 2.0, 500)
    x = x0 - vx0 * t
    y = y0 + vy0 * t - 0.5 * G * t**2

    valid = (y >= 0) & (x >= -20)
    return x[valid], y[valid]


# Print table
print("\n" + "="*80)
print("  LOB TRAJECTORY SOLUTIONS (ball DESCENDING through target)")
print("  Release: 18\" | Target: 72\"")
print("="*80)
print(f"{'Dist':>6} | {'Velocity':>10} | {'Launch':>8} | {'Entry':>8} | {'Max Ht':>8} | {'Flight':>8}")
print("-"*80)

y0 = 18
y_target = 72

for d_ft in [4, 5, 6, 7, 8, 9, 10]:
    x0 = d_ft * 12
    row_printed = False

    for v_fps in [20, 25, 30, 35, 40]:
        v_shot = v_fps * 12
        result = find_lob_trajectory(x0, y0, y_target, v_shot)

        if result is not None:
            a = result
            dist_str = f"{d_ft} ft" if not row_printed else ""
            print(f"{dist_str:>6} | {v_fps:>8} fps | {a['angle_deg']:>6.1f} deg | "
                  f"{a['entry_angle_deg']:>6.1f} deg | {a['max_height']:>6.0f}\" | {a['t_flight']:>6.2f}s")
            row_printed = True

    if not row_printed:
        print(f"{d_ft:>4} ft | {'--- No valid lob solutions ---':^55}")

print("="*80)
print("Entry angle = how steeply ball descends (higher = better for funnel)")
print()

# Create visualization
fig, axes = plt.subplots(1, 2, figsize=(16, 8))

# Left: Trajectories at different velocities
ax1 = axes[0]
x0 = 7 * 12  # 7 ft

velocities = [20, 25, 30, 35, 40]
colors = plt.cm.viridis(np.linspace(0.9, 0.2, len(velocities)))

for v_fps, color in zip(velocities, colors):
    v_shot = v_fps * 12
    result = find_lob_trajectory(x0, y0, y_target, v_shot)

    if result is not None:
        x, y = calculate_trajectory(x0, y0, v_shot, result['angle_deg'])
        ax1.plot(x, y, color=color, linewidth=2.5,
                label=f"{v_fps} fps: {result['angle_deg']:.0f} deg launch, {result['entry_angle_deg']:.0f} deg entry")

# Funnel target
ax1.plot([0, -8], [y_target, y_target-20], 'k-', linewidth=4)
ax1.plot([0, 8], [y_target, y_target-20], 'k-', linewidth=4)
ax1.plot([-20, 20], [y_target, y_target], 'g-', linewidth=5)
ax1.fill_between([-20, 20], y_target-3, y_target+3, color='green', alpha=0.3)
ax1.annotate('FUNNEL TARGET\n(opening faces UP)', (0, y_target+12), ha='center', fontsize=11, fontweight='bold')

# Robot
ax1.add_patch(plt.Rectangle((x0-14, 0), 28, 22, color='gray', alpha=0.4))
ax1.axhline(y=0, color='saddlebrown', linewidth=4)

ax1.set_xlim(-30, 110)
ax1.set_ylim(-10, 200)
ax1.set_xlabel('Distance from target (inches)', fontsize=12)
ax1.set_ylabel('Height (inches)', fontsize=12)
ax1.set_title('LOB Trajectories: 7 ft Shot\nBall arcs UP then DESCENDS into funnel', fontsize=12)
ax1.legend(loc='upper right', fontsize=9)
ax1.grid(True, alpha=0.3)

# Right: Entry angle vs velocity
ax2 = axes[1]

velocities_range = np.linspace(18, 50, 50)
entry_angles = []
launch_angles = []

for v_fps in velocities_range:
    v_shot = v_fps * 12
    result = find_lob_trajectory(x0, y0, y_target, v_shot)
    if result is not None:
        entry_angles.append(result['entry_angle_deg'])
        launch_angles.append(result['angle_deg'])
    else:
        entry_angles.append(np.nan)
        launch_angles.append(np.nan)

ax2.plot(velocities_range, entry_angles, 'b-', linewidth=3, label='Entry angle (descent)')
ax2.plot(velocities_range, launch_angles, 'r--', linewidth=2, label='Launch angle')

ax2.axhspan(60, 90, color='green', alpha=0.2)
ax2.axhspan(45, 60, color='yellow', alpha=0.2)
ax2.axhspan(0, 45, color='red', alpha=0.1)

ax2.annotate('IDEAL\n(60-90 deg)', (22, 75), fontsize=11, ha='center', color='darkgreen')
ax2.annotate('GOOD\n(45-60 deg)', (22, 52), fontsize=10, ha='center', color='darkorange')
ax2.annotate('RISKY\n(<45 deg)', (22, 30), fontsize=10, ha='center', color='red')

ax2.annotate('LOWER velocity\n= STEEPER entry\n= BETTER!', (25, 40), fontsize=12,
            ha='center', color='blue', fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='lightyellow'))

ax2.set_xlabel('Muzzle velocity (ft/s)', fontsize=12)
ax2.set_ylabel('Angle (degrees)', fontsize=12)
ax2.set_title('Entry Angle vs Muzzle Velocity\nLower velocity gives steeper descent!', fontsize=12)
ax2.legend(loc='right', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim(18, 50)
ax2.set_ylim(0, 90)

plt.suptitle('CORRECTED: Lob Shooter Analysis for Funnel Target', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('lob_trajectories.png', dpi=150, bbox_inches='tight')
print("\nSaved: lob_trajectories.png")
