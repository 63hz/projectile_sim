# -*- coding: utf-8 -*-
"""
Create comprehensive summary visualization for LOB SHOOTER.

All trajectories show balls DESCENDING into funnel target.
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from shooter_simulation import (
    find_lob_solution, calculate_trajectory, TARGET_HEIGHT_IN,
    RECOMMENDED_VELOCITY_FPS, G
)

fig = plt.figure(figsize=(20, 12))

# Layout: 2 rows, 3 columns
ax1 = fig.add_subplot(2, 3, 1)  # Trajectory sweep
ax2 = fig.add_subplot(2, 3, 2)  # Robot velocity effect
ax3 = fig.add_subplot(2, 3, 3)  # Entry angle zones
ax4 = fig.add_subplot(2, 3, 4)  # Funnel cross-section
ax5 = fig.add_subplot(2, 3, 5)  # Aiming table heatmap
ax6 = fig.add_subplot(2, 3, 6)  # Extension envelope

y0 = 18
y_target = TARGET_HEIGHT_IN
v_fps = 30  # Lob shooter velocity

# === Plot 1: LOB Trajectory sweep ===
distances_ft = [3, 5, 7, 10, 15, 18]
colors = plt.cm.viridis(np.linspace(0.15, 0.9, len(distances_ft)))

for i, d_ft in enumerate(distances_ft):
    dist_in = d_ft * 12
    sol = find_lob_solution(dist_in, v_fps, y0)

    if sol:
        t, x, y = calculate_trajectory(dist_in, sol.launch_angle_deg, v_fps, y0)
        ax1.plot(x, y, color=colors[i], linewidth=2.5,
                label=f'{d_ft} ft: {sol.launch_angle_deg:.0f}° → {sol.entry_angle_deg:.0f}° entry')

# Funnel target
funnel_w = 21
ax1.plot([0, -8], [y_target, y_target-20], 'k-', linewidth=4)
ax1.plot([0, 8], [y_target, y_target-20], 'k-', linewidth=4)
ax1.plot([-funnel_w, funnel_w], [y_target, y_target], 'g-', linewidth=5)
ax1.fill_between([-funnel_w, funnel_w], y_target-2, y_target+2, color='green', alpha=0.3)
ax1.annotate('FUNNEL\n(opens UP)', (0, y_target+10), ha='center', fontsize=9, fontweight='bold')

ax1.axhline(y=30, color='red', linestyle=':', linewidth=2, label='30" height limit')
ax1.axhline(y=0, color='saddlebrown', linewidth=3)
ax1.set_xlim(-30, 230)
ax1.set_ylim(-10, 220)
ax1.set_xlabel('Distance from target (in)')
ax1.set_ylabel('Height (in)')
ax1.set_title(f'LOB Trajectories @ {v_fps} fps\nBall arcs UP then DESCENDS', fontsize=10)
ax1.legend(fontsize=7, loc='upper right')
ax1.grid(True, alpha=0.3)

# === Plot 2: Robot velocity effect ===
dist_in = 7 * 12
robot_vels = [-10, -5, 0, 5, 10]
colors2 = plt.cm.coolwarm(np.linspace(0, 1, len(robot_vels)))

for v_robot_fps, color in zip(robot_vels, colors2):
    vx_robot = v_robot_fps * 12
    sol = find_lob_solution(dist_in, v_fps, y0, y_target, vx_robot)

    if sol:
        t, x, y = calculate_trajectory(dist_in, sol.launch_angle_deg, v_fps, y0, vx_robot)
        direction = "toward" if v_robot_fps > 0 else "away" if v_robot_fps < 0 else "stat"
        ax2.plot(x, y, color=color, linewidth=2,
                label=f'{v_robot_fps:+d} fps {direction}: {sol.launch_angle_deg:.0f}°')

ax2.axvline(x=0, color='green', linewidth=3, alpha=0.7)
ax2.axhline(y=0, color='saddlebrown', linewidth=3)
ax2.set_xlim(-15, 100)
ax2.set_ylim(-10, 200)
ax2.set_xlabel('Distance (in)')
ax2.set_ylabel('Height (in)')
ax2.set_title(f'Robot Motion Effect\n7 ft shot, {v_fps} fps', fontsize=10)
ax2.legend(fontsize=7)
ax2.grid(True, alpha=0.3)

# === Plot 3: Entry angle zones ===
distances_range = np.linspace(36, 216, 20)  # 3-18 ft

ax3.axhspan(70, 90, color='green', alpha=0.2, label='IDEAL (>70°)')
ax3.axhspan(55, 70, color='yellow', alpha=0.2, label='GOOD (55-70°)')
ax3.axhspan(0, 55, color='red', alpha=0.1, label='OK (<55°)')

for v_test in [28, 30, 34]:
    entry_angles = []
    dists = []
    for dist_in in distances_range:
        sol = find_lob_solution(dist_in, v_test, y0)
        if sol:
            entry_angles.append(sol.entry_angle_deg)
            dists.append(dist_in / 12)
    if entry_angles:
        ax3.plot(dists, entry_angles, 'o-', linewidth=2, markersize=4, label=f'{v_test} fps')

ax3.set_xlabel('Distance (ft)')
ax3.set_ylabel('Entry Angle (deg descent)')
ax3.set_title('Entry Angle Zones\n(LOB into Funnel)', fontsize=10)
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)
ax3.set_xlim(2, 19)
ax3.set_ylim(50, 90)

# === Plot 4: Funnel detail ===
sol = find_lob_solution(7*12, v_fps, y0)
if sol:
    t, x, y = calculate_trajectory(7*12, sol.launch_angle_deg, v_fps, y0)
    ax4.plot(x, y, 'b-', linewidth=3)

# Funnel shape
ax4.plot([0, -10], [y_target, y_target-25], 'k-', linewidth=5)
ax4.plot([0, 10], [y_target, y_target-25], 'k-', linewidth=5)
ax4.plot([-25, 25], [y_target, y_target], 'g-', linewidth=6)
ax4.fill_between([-25, 25], y_target-3, y_target+3, color='green', alpha=0.3)

# Entry arrow
ax4.annotate('', xy=(0, y_target), xytext=(15, y_target+25),
            arrowprops=dict(arrowstyle='->', color='red', lw=3))
if sol:
    ax4.annotate(f'{sol.entry_angle_deg:.0f}° descent', (18, y_target+15),
                fontsize=10, color='red', fontweight='bold')

ax4.set_xlim(-40, 100)
ax4.set_ylim(40, 200)
ax4.set_xlabel('Distance (in)')
ax4.set_ylabel('Height (in)')
ax4.set_title('Funnel Target Detail\n7 ft LOB shot', fontsize=10)
ax4.grid(True, alpha=0.3)

# === Plot 5: Aiming lookup heatmap ===
distances = np.arange(3, 19)
robot_vels_range = np.arange(-10, 11, 5)

angles_grid = np.zeros((len(distances), len(robot_vels_range)))

for i, d_ft in enumerate(distances):
    for j, v_robot_fps in enumerate(robot_vels_range):
        dist_in = d_ft * 12
        vx_robot = v_robot_fps * 12
        sol = find_lob_solution(dist_in, v_fps, y0, y_target, vx_robot)
        if sol:
            angles_grid[i, j] = sol.launch_angle_deg
        else:
            angles_grid[i, j] = np.nan

im = ax5.imshow(angles_grid, aspect='auto', cmap='viridis',
               extent=[robot_vels_range[0]-2.5, robot_vels_range[-1]+2.5,
                      distances[-1]+0.5, distances[0]-0.5])
ax5.set_xlabel('Robot velocity (fps)')
ax5.set_ylabel('Distance (ft)')
ax5.set_title(f'LOB Angle Lookup\n{v_fps} fps', fontsize=10)
cbar = plt.colorbar(im, ax=ax5)
cbar.set_label('Launch Angle (deg)')

# Add text annotations
for i, d_ft in enumerate(distances):
    for j, v_robot in enumerate(robot_vels_range):
        val = angles_grid[i, j]
        if not np.isnan(val):
            ax5.text(v_robot, d_ft, f'{val:.0f}', ha='center', va='center',
                    fontsize=6, color='white')

# === Plot 6: Extension envelope ===
barrel_length = 0  # Zero muzzle length as specified
pivot_y = 18

ax6.add_patch(plt.Rectangle((-14, 0), 28, 20, color='gray', alpha=0.3, label='Robot frame'))
ax6.plot(0, pivot_y, 'ko', markersize=10, label='Pivot')

# Draw arc showing pivot range
angles_sweep = np.linspace(0, np.pi, 37)
for theta in angles_sweep:
    # With zero muzzle length, just show direction
    dx = 8 * np.cos(theta - np.pi/2)
    dy = 8 * np.sin(theta - np.pi/2)
    ax6.arrow(0, pivot_y, dx, dy, head_width=1, head_length=1,
             fc='blue', ec='blue', alpha=0.3)

# 12" and 30" limits
ax6.axvline(x=14+12, color='red', linestyle='--', linewidth=2, label='12" extension')
ax6.axvline(x=-14-12, color='red', linestyle='--', linewidth=2)
ax6.axhline(y=30, color='red', linestyle=':', linewidth=2, label='30" height')

ax6.set_xlim(-30, 30)
ax6.set_ylim(-5, 35)
ax6.set_xlabel('X (in)')
ax6.set_ylabel('Y (in)')
ax6.set_title('R106 Compliance\nZero muzzle length', fontsize=10)
ax6.legend(fontsize=7, loc='upper left')
ax6.set_aspect('equal')
ax6.grid(True, alpha=0.3)

plt.suptitle('FRC 2026 REBUILT - LOB Shooter Design Summary\nTeam 2491 | Ball DESCENDS into funnel',
             fontsize=16, fontweight='bold')
plt.tight_layout()
plt.savefig('shooter_summary.png', dpi=150, bbox_inches='tight')
print('Saved shooter_summary.png')
