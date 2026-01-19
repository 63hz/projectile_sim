# =============================================================================
# FIELD ELEVATION VIEW - All Three Regimes in Absolute Coordinates
# =============================================================================
# This visualization shows the entire field from an elevation (side) view,
# with all three shooting regimes depicted at their actual field positions.
# =============================================================================

def plot_field_elevation_view(scoring_v=30, passing_v=40, stealing_v=55,
                              release_height=18, show_trajectories=True):
    """
    Create a comprehensive field elevation view showing all three regimes.

    Field Layout (x = distance from friendly alliance far wall):
        x = 0: Friendly alliance far wall
        x = 0-183": Friendly alliance zone (scoring robots here, landing zone for passes)
        x = 183": HUB (72" high) + Wall (48" high)
        x = 183-470": Neutral zone (passing robots here)
        x = 470-650": Opposing alliance zone (stealing robots here)

    Args:
        scoring_v: Velocity for scoring shots (ft/s)
        passing_v: Velocity for passing shots (ft/s)
        stealing_v: Velocity for stealing shots (ft/s)
        release_height: Ball release height (inches)
        show_trajectories: Whether to draw trajectory arcs
    """
    fig, ax = plt.subplots(figsize=(20, 10))

    # Field zones (colored backgrounds)
    # Friendly alliance zone (light blue)
    ax.fill_between([0, FIELD_HUB_X], 0, 300, color='lightblue', alpha=0.3, label='Friendly Alliance')
    # Neutral zone (light gray)
    ax.fill_between([FIELD_HUB_X, FIELD_NEUTRAL_END_X], 0, 300, color='lightgray', alpha=0.3, label='Neutral Zone')
    # Opposing alliance zone (light red)
    ax.fill_between([FIELD_NEUTRAL_END_X, FIELD_OPPOSING_WALL_X], 0, 300, color='lightsalmon', alpha=0.3, label='Opposing Alliance')

    # Ground
    ax.axhline(y=0, color='saddlebrown', linewidth=4)
    ax.fill_between([0, FIELD_OPPOSING_WALL_X], -10, 0, color='saddlebrown', alpha=0.5)

    # HUB at x=183 (the scoring target)
    hub_width = 40  # Visual width of HUB
    ax.fill_between([FIELD_HUB_X - hub_width/2, FIELD_HUB_X + hub_width/2],
                    FIELD_HUB_HEIGHT - 10, FIELD_HUB_HEIGHT + 5,
                    color='green', alpha=0.8)
    ax.plot([FIELD_HUB_X - hub_width/2, FIELD_HUB_X - hub_width/2 - 15],
            [FIELD_HUB_HEIGHT, FIELD_HUB_HEIGHT - 30], 'g-', linewidth=3)
    ax.plot([FIELD_HUB_X + hub_width/2, FIELD_HUB_X + hub_width/2 + 15],
            [FIELD_HUB_HEIGHT, FIELD_HUB_HEIGHT - 30], 'g-', linewidth=3)
    ax.text(FIELD_HUB_X, FIELD_HUB_HEIGHT + 15, 'HUB\n(72")', ha='center', fontsize=10, fontweight='bold')

    # Wall at x=183 (48" tall)
    ax.fill_between([FIELD_HUB_X - 5, FIELD_HUB_X + 5], 0, FIELD_WALL_HEIGHT,
                    color='red', alpha=0.6)
    ax.text(FIELD_HUB_X, FIELD_WALL_HEIGHT + 8, f'Wall\n({FIELD_WALL_HEIGHT}")',
            ha='center', fontsize=9, color='red')

    # Wall at x=470 (if present - using same height for consistency)
    ax.fill_between([FIELD_NEUTRAL_END_X - 5, FIELD_NEUTRAL_END_X + 5], 0, FIELD_WALL_HEIGHT,
                    color='red', alpha=0.4)
    ax.text(FIELD_NEUTRAL_END_X, FIELD_WALL_HEIGHT + 8, f'Wall\n({FIELD_WALL_HEIGHT}")',
            ha='center', fontsize=9, color='red', alpha=0.7)

    # Zone labels
    ax.text(FIELD_HUB_X/2, 280, 'FRIENDLY\nALLIANCE', ha='center', va='top',
            fontsize=14, fontweight='bold', color='navy')
    ax.text((FIELD_HUB_X + FIELD_NEUTRAL_END_X)/2, 280, 'NEUTRAL\nZONE', ha='center', va='top',
            fontsize=14, fontweight='bold', color='gray')
    ax.text((FIELD_NEUTRAL_END_X + FIELD_OPPOSING_WALL_X)/2, 280, 'OPPOSING\nALLIANCE', ha='center', va='top',
            fontsize=14, fontweight='bold', color='darkred')

    # Position markers
    ax.axvline(x=0, color='blue', linewidth=2, linestyle='--', alpha=0.5)
    ax.axvline(x=FIELD_HUB_X, color='green', linewidth=2, linestyle='-', alpha=0.5)
    ax.axvline(x=FIELD_NEUTRAL_END_X, color='orange', linewidth=2, linestyle='--', alpha=0.5)
    ax.axvline(x=FIELD_OPPOSING_WALL_X, color='red', linewidth=2, linestyle='--', alpha=0.5)

    # X-axis labels
    ax.text(0, -25, 'x=0"', ha='center', fontsize=9)
    ax.text(FIELD_HUB_X, -25, 'x=183"', ha='center', fontsize=9)
    ax.text(FIELD_NEUTRAL_END_X, -25, 'x=470"', ha='center', fontsize=9)
    ax.text(FIELD_OPPOSING_WALL_X, -25, 'x=650"', ha='center', fontsize=9)

    if show_trajectories:
        # =========== SCORING TRAJECTORIES (Left to Right) ===========
        # Robot positions in friendly zone, shooting AT the HUB
        scoring_positions = [50, 100, 150]  # x positions of scoring robots
        scoring_colors = plt.cm.Blues(np.linspace(0.4, 0.9, len(scoring_positions)))

        for robot_x, color in zip(scoring_positions, scoring_colors):
            distance = FIELD_HUB_X - robot_x  # Distance to HUB
            if 'find_lob_solution' in dir():
                traj = find_lob_solution(distance, scoring_v, release_height)
            else:
                # Simplified lob solution
                traj = None
                v_shot = scoring_v * 12
                for theta_deg in np.linspace(50, 85, 500):
                    theta = np.radians(theta_deg)
                    vx0 = v_shot * np.cos(theta)
                    vy0 = v_shot * np.sin(theta)
                    if vx0 <= 0:
                        continue
                    t_hit = distance / vx0
                    y_at_target = release_height + vy0 * t_hit - 0.5 * G_IN_S2 * t_hit**2
                    vy_at_target = vy0 - G_IN_S2 * t_hit
                    if abs(y_at_target - FIELD_HUB_HEIGHT) < 5 and vy_at_target < 0:
                        traj = {'angle_deg': theta_deg, 'vx0': vx0, 'vy0': vy0}
                        break

            if traj:
                v_shot = scoring_v * 12
                theta = np.radians(traj['angle_deg'])
                vx0 = v_shot * np.cos(theta)
                vy0 = v_shot * np.sin(theta)

                t = np.linspace(0, 2, 300)
                # Scoring: shooting from robot_x toward larger x (the HUB)
                x = robot_x + vx0 * t
                y = release_height + vy0 * t - 0.5 * G_IN_S2 * t**2

                valid = (x <= FIELD_HUB_X + 20) & (y >= -5) & (y <= 250)
                ax.plot(x[valid], y[valid], color=color, linewidth=2.5, alpha=0.8)

                # Robot marker
                ax.plot(robot_x, release_height, 's', color=color, markersize=10)

        ax.plot([], [], 'b-', linewidth=2.5, label=f'SCORING @ {scoring_v} ft/s')

        # =========== PASSING TRAJECTORIES (Right to Left) ===========
        # Robot positions in neutral zone, shooting toward friendly zone
        passing_positions = [250, 350, 450]  # x positions of passing robots
        passing_colors = plt.cm.Purples(np.linspace(0.4, 0.9, len(passing_positions)))

        for robot_x, color in zip(passing_positions, passing_colors):
            solution = find_wall_clearing_trajectory_absolute(
                robot_x, passing_v, FIELD_HUB_X, FIELD_WALL_HEIGHT,
                (0, FIELD_LANDING_ZONE_DEPTH), release_height
            )

            if solution:
                theta = np.radians(solution['optimal_angle'])
                v_shot = passing_v * 12
                vx0 = v_shot * np.cos(theta)
                vy0 = v_shot * np.sin(theta)

                t = np.linspace(0, 3, 400)
                # Passing: shooting from robot_x toward smaller x
                x = robot_x - vx0 * t
                y = release_height + vy0 * t - 0.5 * G_IN_S2 * t**2

                valid = (x >= -20) & (y >= -5) & (y <= 250)
                ax.plot(x[valid], y[valid], color=color, linewidth=2.5, alpha=0.8)

                # Robot marker
                ax.plot(robot_x, release_height, 'o', color=color, markersize=10)

        ax.plot([], [], 'purple', linewidth=2.5, label=f'PASSING @ {passing_v} ft/s')

        # =========== STEALING TRAJECTORIES (Right to Left) ===========
        # Robot positions in opposing zone, shooting toward friendly zone
        stealing_positions = [500, 550, 600]  # x positions of stealing robots
        stealing_colors = plt.cm.Oranges(np.linspace(0.4, 0.9, len(stealing_positions)))

        for robot_x, color in zip(stealing_positions, stealing_colors):
            solution = find_wall_clearing_trajectory_absolute(
                robot_x, stealing_v, FIELD_HUB_X, FIELD_WALL_HEIGHT,
                (0, FIELD_LANDING_ZONE_DEPTH), release_height
            )

            if solution:
                theta = np.radians(solution['optimal_angle'])
                v_shot = stealing_v * 12
                vx0 = v_shot * np.cos(theta)
                vy0 = v_shot * np.sin(theta)

                t = np.linspace(0, 4, 500)
                # Stealing: shooting from robot_x toward smaller x
                x = robot_x - vx0 * t
                y = release_height + vy0 * t - 0.5 * G_IN_S2 * t**2

                valid = (x >= -20) & (y >= -5) & (y <= 300)
                ax.plot(x[valid], y[valid], color=color, linewidth=2.5, alpha=0.8)

                # Robot marker
                ax.plot(robot_x, release_height, '^', color=color, markersize=10)

        ax.plot([], [], 'orange', linewidth=2.5, label=f'STEALING @ {stealing_v} ft/s')

    # Formatting
    ax.set_xlim(-30, FIELD_OPPOSING_WALL_X + 30)
    ax.set_ylim(-40, 320)
    ax.set_xlabel('Field Position (inches from friendly alliance wall)', fontsize=12)
    ax.set_ylabel('Height (inches)', fontsize=12)
    ax.set_title('FRC 2026 REBUILT - Field Elevation View\nAll Three Shooting Regimes',
                 fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Add info box
    info_text = f"""REGIME SUMMARY
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SCORING: {scoring_v} ft/s
  • Robot: x < 183" (friendly zone)
  • Target: HUB at 72"
  • Constraint: Ball must DESCEND

PASSING: {passing_v} ft/s
  • Robot: 183" < x < 470" (neutral)
  • Target: x < 183" (friendly zone)
  • Constraint: Clear 48" wall

STEALING: {stealing_v} ft/s
  • Robot: x > 470" (opposing zone)
  • Target: x < 183" (friendly zone)
  • Constraint: Clear 48" wall
"""
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

    plt.tight_layout()
    plt.show()

    return fig


# Run the visualization
print("="*70)
print("  FIELD ELEVATION VIEW - All Shooting Regimes")
print("="*70)
print("\nGenerating field elevation view with example trajectories...")
print("  • SCORING shots: Blue (left to right, into HUB)")
print("  • PASSING shots: Purple (right to left, from neutral zone)")
print("  • STEALING shots: Orange (right to left, from opposing zone)")
print()

plot_field_elevation_view(scoring_v=30, passing_v=40, stealing_v=55, release_height=18)
