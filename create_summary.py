# -*- coding: utf-8 -*-
"""Create comprehensive summary visualization."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

G = 386.09  # in/s^2

fig = plt.figure(figsize=(20, 12))

# Layout: 2 rows, 3 columns
ax1 = fig.add_subplot(2, 3, 1)  # Trajectory sweep
ax2 = fig.add_subplot(2, 3, 2)  # Robot velocity effect
ax3 = fig.add_subplot(2, 3, 3)  # Descent angle zones
ax4 = fig.add_subplot(2, 3, 4)  # Funnel cross-section
ax5 = fig.add_subplot(2, 3, 5)  # Aiming table heatmap
ax6 = fig.add_subplot(2, 3, 6)  # Extension envelope

y0 = 18
y_target = 72
v_shot = 45 * 12

# === Plot 1: Trajectory sweep ===
distances_ft = [4, 6, 8, 10]
colors = plt.cm.viridis([0.2, 0.4, 0.6, 0.8])

for i, d_ft in enumerate(distances_ft):
    x0 = d_ft * 12
    for theta_deg in np.linspace(5, 89, 500):
        theta = np.radians(theta_deg)
        vx0 = v_shot * np.cos(theta)
        vy0 = v_shot * np.sin(theta)
        t_hit = x0 / vx0
        y_at_target = y0 + vy0 * t_hit - 0.5 * G * t_hit**2
        if abs(y_at_target - y_target) < 1:
            t = np.linspace(0, t_hit, 100)
            x = x0 - vx0 * t
            y = y0 + vy0 * t - 0.5 * G * t**2
            vy_end = vy0 - G * t_hit
            descent = np.degrees(np.arctan2(abs(vy_end), vx0))
            ax1.plot(x, y, color=colors[i], linewidth=2.5,
                    label=f'{d_ft} ft: {theta_deg:.0f} deg ({descent:.0f} deg entry)')
            break

ax1.axvline(x=0, color='green', linewidth=3, alpha=0.7)
ax1.add_patch(plt.Rectangle((-4, 69), 8, 6, color='green', alpha=0.4))
ax1.axhline(y=30, color='red', linestyle=':', linewidth=2)
ax1.axhline(y=0, color='saddlebrown', linewidth=3)
ax1.set_xlim(-15, 130)
ax1.set_ylim(-5, 140)
ax1.set_xlabel('Distance (in)')
ax1.set_ylabel('Height (in)')
ax1.set_title('Trajectory Sweep\n45 ft/s, 18" release', fontsize=10)
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.3)

# === Plot 2: Robot velocity effect ===
x0 = 7 * 12
robot_vels = [-5, 0, 5, 10]
colors2 = ['blue', 'black', 'green', 'red']

for v_robot_fps, color in zip(robot_vels, colors2):
    vx_robot = v_robot_fps * 12

    for theta_deg in np.linspace(5, 89, 500):
        theta = np.radians(theta_deg)
        vx0 = v_shot * np.cos(theta) + vx_robot
        vy0 = v_shot * np.sin(theta)
        if vx0 <= 0:
            continue
        t_hit = x0 / vx0
        y_at_target = y0 + vy0 * t_hit - 0.5 * G * t_hit**2
        if abs(y_at_target - y_target) < 1:
            t = np.linspace(0, t_hit, 100)
            x = x0 - vx0 * t
            y = y0 + vy0 * t - 0.5 * G * t**2
            label = f'{v_robot_fps:+d} fps robot: {theta_deg:.0f} deg'
            ax2.plot(x, y, color=color, linewidth=2, label=label)
            break

ax2.axvline(x=0, color='green', linewidth=3, alpha=0.7)
ax2.axhline(y=0, color='saddlebrown', linewidth=3)
ax2.set_xlim(-15, 100)
ax2.set_ylim(-5, 130)
ax2.set_xlabel('Distance (in)')
ax2.set_ylabel('Height (in)')
ax2.set_title('Robot Motion Effect\n7 ft shot, 45 ft/s', fontsize=10)
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

# === Plot 3: Descent angle zones ===
distances_ft_range = np.linspace(3, 12, 30)
velocities = [(35, 'purple'), (45, 'blue'), (55, 'green'), (65, 'orange')]

ax3.axhspan(50, 70, color='green', alpha=0.2, label='IDEAL')
ax3.axhspan(30, 50, color='yellow', alpha=0.2, label='OK')
ax3.axhspan(0, 30, color='red', alpha=0.2, label='RISKY')

for v_fps, color in velocities:
    v = v_fps * 12
    descents = []
    dists = []
    for d_ft in distances_ft_range:
        x0 = d_ft * 12
        for theta_deg in np.linspace(5, 89, 500):
            theta = np.radians(theta_deg)
            vx0 = v * np.cos(theta)
            vy0 = v * np.sin(theta)
            t_hit = x0 / vx0
            y_at_target = y0 + vy0 * t_hit - 0.5 * G * t_hit**2
            if abs(y_at_target - y_target) < 1:
                vy_end = vy0 - G * t_hit
                descents.append(np.degrees(np.arctan2(abs(vy_end), vx0)))
                dists.append(d_ft)
                break
    ax3.plot(dists, descents, color=color, marker='o', linewidth=2, markersize=4, label=f'{v_fps} fps')

ax3.set_xlabel('Distance (ft)')
ax3.set_ylabel('Descent Angle (deg)')
ax3.set_title('Entry Angle Zones\n(Funnel Target)', fontsize=10)
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)
ax3.set_xlim(2, 13)
ax3.set_ylim(0, 70)

# === Plot 4: Funnel cross-section ===
theta = np.radians(36)  # Typical launch
vx0 = v_shot * np.cos(theta)
vy0 = v_shot * np.sin(theta)
x0 = 7 * 12
t_hit = x0 / vx0

t = np.linspace(0, t_hit * 1.1, 100)
x = x0 - vx0 * t
y = y0 + vy0 * t - 0.5 * G * t**2
ax4.plot(x, y, 'b-', linewidth=3)

# Funnel shape
funnel_w = 21
ax4.plot([0, -7], [y_target, y_target - 18], 'k-', linewidth=5)
ax4.plot([0, 7], [y_target, y_target - 18], 'k-', linewidth=5)
ax4.plot([-funnel_w, funnel_w], [y_target, y_target], 'g-', linewidth=6)
ax4.fill_between([-funnel_w, funnel_w], y_target-2, y_target+2, color='green', alpha=0.3)

# Entry arrow
vy_end = vy0 - G * t_hit
ax4.annotate('', xy=(0, y_target), xytext=(10, y_target + 15),
            arrowprops=dict(arrowstyle='->', color='red', lw=3))
descent = np.degrees(np.arctan2(abs(vy_end), vx0))
ax4.annotate(f'{descent:.0f} deg\ndescent', (12, y_target + 8), fontsize=10, color='red')

ax4.set_xlim(-30, 100)
ax4.set_ylim(30, 110)
ax4.set_xlabel('Distance (in)')
ax4.set_ylabel('Height (in)')
ax4.set_title('Funnel Target Detail\n7 ft shot', fontsize=10)
ax4.set_aspect('equal')
ax4.grid(True, alpha=0.3)

# === Plot 5: Aiming lookup heatmap ===
distances = np.arange(3, 13)
robot_vels_range = np.arange(-10, 16, 5)

angles_grid = np.zeros((len(distances), len(robot_vels_range)))

for i, d_ft in enumerate(distances):
    for j, v_robot_fps in enumerate(robot_vels_range):
        x0 = d_ft * 12
        vx_robot = v_robot_fps * 12
        for theta_deg in np.linspace(5, 89, 500):
            theta = np.radians(theta_deg)
            vx0 = v_shot * np.cos(theta) + vx_robot
            vy0 = v_shot * np.sin(theta)
            if vx0 <= 0:
                continue
            t_hit = x0 / vx0
            y_at_target = y0 + vy0 * t_hit - 0.5 * G * t_hit**2
            if abs(y_at_target - y_target) < 1:
                angles_grid[i, j] = theta_deg
                break

im = ax5.imshow(angles_grid, aspect='auto', cmap='viridis',
               extent=[robot_vels_range[0]-2.5, robot_vels_range[-1]+2.5,
                      distances[-1]+0.5, distances[0]-0.5])
ax5.set_xlabel('Robot velocity (fps)')
ax5.set_ylabel('Distance (ft)')
ax5.set_title('Launch Angle Lookup\n(degrees)', fontsize=10)
cbar = plt.colorbar(im, ax=ax5)
cbar.set_label('Angle (deg)')

# Add text annotations
for i, d_ft in enumerate(distances):
    for j, v_robot in enumerate(robot_vels_range):
        val = angles_grid[i, j]
        if val > 0:
            ax5.text(v_robot, d_ft, f'{val:.0f}', ha='center', va='center', fontsize=7, color='white')

# === Plot 6: Extension envelope ===
barrel_length = 10
pivot_y = 15

angles_sweep = np.linspace(0, np.pi, 37)
tip_x = []
tip_y = []

for theta in angles_sweep:
    bx = barrel_length * np.cos(theta - np.pi/2)
    by = pivot_y + barrel_length * np.sin(theta - np.pi/2)
    tip_x.append(bx)
    tip_y.append(by)
    ax6.plot([0, bx], [pivot_y, by], 'b-', linewidth=1.5, alpha=0.4)

ax6.plot(tip_x, tip_y, 'g-', linewidth=3, label='Barrel sweep')
ax6.add_patch(plt.Rectangle((-14, 0), 28, 20, color='gray', alpha=0.3, label='Robot frame'))
ax6.plot(0, pivot_y, 'ko', markersize=10, label='Pivot')

# 12" limit
ax6.axvline(x=14+12, color='red', linestyle='--', linewidth=2)
ax6.axvline(x=-14-12, color='red', linestyle='--', linewidth=2)
ax6.axhline(y=30, color='red', linestyle=':', linewidth=2, label='30" height')

ax6.set_xlim(-30, 30)
ax6.set_ylim(-5, 35)
ax6.set_xlabel('X (in)')
ax6.set_ylabel('Y (in)')
ax6.set_title('R106 Extension Envelope\n10" barrel', fontsize=10)
ax6.legend(fontsize=7, loc='upper left')
ax6.set_aspect('equal')
ax6.grid(True, alpha=0.3)

plt.suptitle('FRC 2026 REBUILT - Multi-Barrel Shooter Design Summary\nTeam 2491', fontsize=16, fontweight='bold')
plt.tight_layout()
plt.savefig('shooter_summary.png', dpi=150, bbox_inches='tight')
print('Saved shooter_summary.png')
