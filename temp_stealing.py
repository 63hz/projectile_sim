# =============================================================================
# STEALING REGIME ANALYSIS (with Absolute Field Coordinates)
# =============================================================================
# Check that required functions are defined
if 'analyze_regime' not in dir() or 'find_wall_clearing_trajectory' not in dir():
    raise RuntimeError("Please run the Section 13 setup cell first!")
if 'find_wall_clearing_trajectory_absolute' not in dir():
    raise RuntimeError("Please run the updated Section 13 setup cell first!")
if 'G_IN_S2' not in dir():
    G_IN_S2 = 386.09

def analyze_stealing_with_field_view(velocity_fps=55, release_height=18):
    """
    Comprehensive stealing regime analysis with field elevation view.

    STEALING shots:
    - Robot in OPPOSING ALLIANCE ZONE (470" < x < 650")
    - Target: FRIENDLY ALLIANCE ZONE (0 < x < 180")
    - Must clear 48" wall at x = 183"
    - Longest range shots in the game!
    """

    print("="*75)
    print("  STEALING REGIME - Field Coordinate Analysis")
    print("="*75)
    print(f"\n  Robot Zone: Opposing Alliance (470\" < x < 650\")")
    print(f"  Target Zone: Friendly Alliance (0\" < x < 180\")")
    print(f"  Wall: x = 183\", height = 48\"")
    print(f"  Velocity: {velocity_fps} ft/s")
    print(f"\n  ⚠️  These are LONG shots (350-600+ inches)")
    print(f"      Air resistance may reduce range by 10-15%!")

    # Analyze using absolute coordinates
    results = analyze_regime_absolute(
        "STEALING",
        robot_x_range=(490, 630),  # Robot positions in opposing zone
        velocity_fps=velocity_fps,
        wall_x=FIELD_HUB_X,
        wall_height=FIELD_WALL_HEIGHT,
        landing_x_range=(0, FIELD_LANDING_ZONE_DEPTH),
        release_height=release_height,
        num_points=12
    )

    # Visualization
    fig, axes = plt.subplots(1, 2, figsize=(20, 8))

    # Plot 1: Full field elevation view with trajectories
    ax1 = axes[0]

    # Draw field zones
    ax1.fill_between([0, FIELD_HUB_X], 0, 350, color='lightblue', alpha=0.3)
    ax1.fill_between([FIELD_HUB_X, FIELD_NEUTRAL_END_X], 0, 350, color='lightgray', alpha=0.3)
    ax1.fill_between([FIELD_NEUTRAL_END_X, FIELD_OPPOSING_WALL_X], 0, 350, color='lightsalmon', alpha=0.3)

    # Ground
    ax1.axhline(y=0, color='saddlebrown', linewidth=3)

    # Wall at x=183
    ax1.fill_between([FIELD_HUB_X-3, FIELD_HUB_X+3], 0, FIELD_WALL_HEIGHT, color='red', alpha=0.6)
    ax1.text(FIELD_HUB_X, FIELD_WALL_HEIGHT+10, f'Wall\n({FIELD_WALL_HEIGHT}\")',
             ha='center', fontsize=9, color='red')

    # Wall at x=470 (may need to clear this too from some angles)
    ax1.fill_between([FIELD_NEUTRAL_END_X-3, FIELD_NEUTRAL_END_X+3], 0, FIELD_WALL_HEIGHT,
                     color='red', alpha=0.4)

    # Landing zone highlight
    ax1.fill_between([0, FIELD_LANDING_ZONE_DEPTH], 0, 15, color='green', alpha=0.2)
    ax1.text(90, 8, 'Landing Zone', ha='center', fontsize=9, color='darkgreen')

    # Plot trajectories for several robot positions
    robot_positions = [490, 530, 570, 610]
    colors = plt.cm.magma(np.linspace(0.2, 0.8, len(robot_positions)))

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

            t = np.linspace(0, 4, 500)
            x = robot_x - vx0 * t
            y = release_height + vy0 * t - 0.5 * G_IN_S2 * t**2

            valid = (x >= -10) & (y >= -5)
            ax1.plot(x[valid], y[valid], color=color, linewidth=2.5,
                    label=f'x={robot_x}\": {solution[\"optimal_angle\"]:.0f}° ({solution[\"optimal_flight_time\"]:.2f}s)')

            # Robot marker
            ax1.plot(robot_x, release_height, '^', color=color, markersize=12)

    ax1.set_xlim(-30, 680)
    ax1.set_ylim(-20, 320)
    ax1.set_xlabel('Field Position (inches)', fontsize=11)
    ax1.set_ylabel('Height (inches)', fontsize=11)
    ax1.set_title(f'STEALING Trajectories @ {velocity_fps} ft/s\n(Absolute Field Coordinates)', fontsize=12)
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Zone labels
    ax1.text(90, 300, 'FRIENDLY', ha='center', fontsize=10, fontweight='bold', color='navy')
    ax1.text(320, 300, 'NEUTRAL', ha='center', fontsize=10, fontweight='bold', color='gray')
    ax1.text(560, 300, 'OPPOSING', ha='center', fontsize=10, fontweight='bold', color='darkred')

    # Plot 2: Analysis summary
    ax2 = axes[1]
    ax2.axis('off')

    if results:
        angles = [r['angle'] for r in results]
        clearances = [r['clearance'] for r in results]
        flight_times = [r['flight_time'] for r in results]
        landing_xs = [r['landing_x'] for r in results]
        max_heights = [r['max_height'] for r in results]

        summary = f"""
STEALING REGIME ANALYSIS SUMMARY
════════════════════════════════════════════════════

Configuration:
  • Ball velocity: {velocity_fps} ft/s ({velocity_fps*12:.0f} in/s)
  • Release height: {release_height}"
  • Wall to clear: x = {FIELD_HUB_X}", height = {FIELD_WALL_HEIGHT}"

Robot Coverage:
  • Positions tested: x = 490" to 630"
  • Coverage: {len(results)}/12 positions successful

Trajectory Parameters:
  • Launch angles: {min(angles):.1f}° to {max(angles):.1f}°
  • Wall clearance: {min(clearances):.1f}" to {max(clearances):.1f}"
  • Flight times: {min(flight_times):.2f}s to {max(flight_times):.2f}s
  • Max heights: {min(max_heights):.0f}" to {max(max_heights):.0f}"
  • Landing positions: x = {min(landing_xs):.0f}" to {max(landing_xs):.0f}"

⚠️  AIR RESISTANCE WARNING:
  At {velocity_fps} ft/s over {int(max(r['robot_x'] - r['landing_x'] for r in results))}+ inches,
  air resistance will reduce actual range by ~10-15%.
  Add {int(velocity_fps * 0.15):.0f} ft/s margin to velocity estimate.

════════════════════════════════════════════════════
        """
    else:
        summary = f"""
STEALING REGIME ANALYSIS SUMMARY
════════════════════════════════════════════════════

⚠️  NO VALID TRAJECTORIES FOUND at {velocity_fps} ft/s

This is expected for very long shots. Try:
  • Increasing velocity significantly (60+ ft/s needed)
  • These shots require high-powered flywheels
  • Consider if stealing is mechanically feasible

════════════════════════════════════════════════════
        """

    ax2.text(0.05, 0.95, summary, transform=ax2.transAxes, fontsize=11,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='mistyrose', alpha=0.9))

    plt.tight_layout()
    plt.show()

    return results


# Run the analysis
analyze_stealing_with_field_view(velocity_fps=55, release_height=18)
