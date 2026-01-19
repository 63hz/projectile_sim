# =============================================================================
# PASSING REGIME ANALYSIS (with Absolute Field Coordinates)
# =============================================================================
# Check that required functions are defined (run cell 39 first!)
if 'analyze_regime' not in dir() or 'find_wall_clearing_trajectory' not in dir():
    raise RuntimeError("Please run the Section 13 setup cell (cell 39) first!")
if 'find_wall_clearing_trajectory_absolute' not in dir():
    raise RuntimeError("Please run the updated Section 13 setup cell first!")
if 'G_IN_S2' not in dir():
    G_IN_S2 = 386.09  # Fallback definition

def analyze_passing_with_field_view(velocity_fps=40, release_height=18):
    """
    Comprehensive passing regime analysis with field elevation view.

    PASSING shots:
    - Robot in NEUTRAL ZONE (183" < x < 470")
    - Target: FRIENDLY ALLIANCE ZONE (0 < x < 180")
    - Must clear 48" wall at x = 183"
    """

    print("="*75)
    print("  PASSING REGIME - Field Coordinate Analysis")
    print("="*75)
    print(f"\n  Robot Zone: Neutral (183\" < x < 470\")")
    print(f"  Target Zone: Friendly Alliance (0\" < x < 180\")")
    print(f"  Wall: x = 183\", height = 48\"")
    print(f"  Velocity: {velocity_fps} ft/s")

    # Analyze using absolute coordinates
    results = analyze_regime_absolute(
        "PASSING",
        robot_x_range=(220, 450),  # Robot positions in neutral zone
        velocity_fps=velocity_fps,
        wall_x=FIELD_HUB_X,
        wall_height=FIELD_WALL_HEIGHT,
        landing_x_range=(0, FIELD_LANDING_ZONE_DEPTH),
        release_height=release_height,
        num_points=12
    )

    # Visualization
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))

    # Plot 1: Field elevation view with trajectories
    ax1 = axes[0]

    # Draw field zones
    ax1.fill_between([0, FIELD_HUB_X], 0, 200, color='lightblue', alpha=0.3)
    ax1.fill_between([FIELD_HUB_X, FIELD_NEUTRAL_END_X], 0, 200, color='lightgray', alpha=0.3)

    # Ground
    ax1.axhline(y=0, color='saddlebrown', linewidth=3)

    # Wall at x=183
    ax1.fill_between([FIELD_HUB_X-3, FIELD_HUB_X+3], 0, FIELD_WALL_HEIGHT, color='red', alpha=0.6)
    ax1.text(FIELD_HUB_X, FIELD_WALL_HEIGHT+5, f'Wall ({FIELD_WALL_HEIGHT}\")',
             ha='center', fontsize=9, color='red')

    # Landing zone highlight
    ax1.fill_between([0, FIELD_LANDING_ZONE_DEPTH], 0, 15, color='green', alpha=0.2)
    ax1.text(90, 8, 'Landing Zone', ha='center', fontsize=9, color='darkgreen')

    # Plot trajectories for several robot positions
    robot_positions = [220, 300, 380, 450]
    colors = plt.cm.plasma(np.linspace(0.2, 0.9, len(robot_positions)))

    for robot_x, color in zip(robot_positions, colors):
        solution = find_wall_clearing_trajectory_absolute(
            robot_x, velocity_fps, FIELD_HUB_X, FIELD_WALL_HEIGHT,
            (0, FIELD_LANDING_ZONE_DEPTH), release_height
        )

        if solution:
            theta = np.radians(solution['optimal_angle'])
            v_shot = velocity_fps * 12
            vx0 = v_shot * np.cos(theta)
            vy0 = v_shot * np.sin(theta)

            t = np.linspace(0, 3, 400)
            x = robot_x - vx0 * t
            y = release_height + vy0 * t - 0.5 * G_IN_S2 * t**2

            valid = (x >= -10) & (y >= -5)
            ax1.plot(x[valid], y[valid], color=color, linewidth=2.5,
                    label=f'x={robot_x}\": {solution[\"optimal_angle\"]:.0f}° ({solution[\"optimal_flight_time\"]:.2f}s)')

            # Robot marker
            ax1.plot(robot_x, release_height, 'o', color=color, markersize=12)

    ax1.set_xlim(-20, 500)
    ax1.set_ylim(-15, 180)
    ax1.set_xlabel('Field Position (inches)', fontsize=11)
    ax1.set_ylabel('Height (inches)', fontsize=11)
    ax1.set_title(f'PASSING Trajectories @ {velocity_fps} ft/s\n(Absolute Field Coordinates)', fontsize=12)
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Zone labels
    ax1.text(90, 170, 'FRIENDLY', ha='center', fontsize=10, fontweight='bold', color='navy')
    ax1.text(320, 170, 'NEUTRAL ZONE', ha='center', fontsize=10, fontweight='bold', color='gray')

    # Plot 2: Analysis summary
    ax2 = axes[1]
    ax2.axis('off')

    if results:
        angles = [r['angle'] for r in results]
        clearances = [r['clearance'] for r in results]
        flight_times = [r['flight_time'] for r in results]
        landing_xs = [r['landing_x'] for r in results]

        summary = f"""
PASSING REGIME ANALYSIS SUMMARY
════════════════════════════════════════════════════

Configuration:
  • Ball velocity: {velocity_fps} ft/s ({velocity_fps*12:.0f} in/s)
  • Release height: {release_height}"
  • Wall: x = {FIELD_HUB_X}", height = {FIELD_WALL_HEIGHT}"

Robot Coverage:
  • Positions tested: x = 220" to 450"
  • Coverage: {len(results)}/12 positions successful

Trajectory Parameters:
  • Launch angles: {min(angles):.1f}° to {max(angles):.1f}°
  • Wall clearance: {min(clearances):.1f}" to {max(clearances):.1f}"
  • Flight times: {min(flight_times):.2f}s to {max(flight_times):.2f}s
  • Landing positions: x = {min(landing_xs):.0f}" to {max(landing_xs):.0f}"

Physics Notes:
  • Higher angles = more clearance but longer flight
  • Ball clears wall, then descends into landing zone
  • Air resistance may reduce range by ~10% at this velocity

════════════════════════════════════════════════════
        """
    else:
        summary = f"""
PASSING REGIME ANALYSIS SUMMARY
════════════════════════════════════════════════════

⚠️  NO VALID TRAJECTORIES FOUND at {velocity_fps} ft/s

Try:
  • Increasing velocity (need higher speeds for longer range)
  • Checking wall clearance constraints
  • Verifying robot position ranges

════════════════════════════════════════════════════
        """

    ax2.text(0.05, 0.95, summary, transform=ax2.transAxes, fontsize=11,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

    plt.tight_layout()
    plt.show()

    return results


# Run the analysis
analyze_passing_with_field_view(velocity_fps=40, release_height=18)
