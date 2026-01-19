# =============================================================================
# UNIFIED VELOCITY REGIME FINDER (with Absolute Field Coordinates)
# =============================================================================
# This cell finds the MINIMUM velocity needed for each regime to cover its
# full range, and visualizes all three on a single field elevation view.
# =============================================================================

# Check that required functions are defined
if 'find_wall_clearing_trajectory_absolute' not in dir():
    raise RuntimeError("Please run the Section 13 setup cell (cell 39) first!")
if 'G_IN_S2' not in dir():
    G_IN_S2 = 386.09
if 'TARGET_HEIGHT_IN' not in dir():
    TARGET_HEIGHT_IN = 72

# Check for find_lob_solution (needed for SCORING regime)
if 'find_lob_solution' not in dir():
    print("NOTE: find_lob_solution not available. Defining simplified version.")
    def find_lob_solution(distance_in, velocity_fps, release_height_in):
        """Simplified lob solution for scoring (descend into 72" funnel)."""
        v_shot = velocity_fps * 12
        best = None
        best_entry = 0
        for theta_deg in np.linspace(50, 89, 800):
            theta = np.radians(theta_deg)
            vx0 = v_shot * np.cos(theta)
            vy0 = v_shot * np.sin(theta)
            if vx0 <= 0:
                continue
            t_hit = distance_in / vx0
            y_at_target = release_height_in + vy0 * t_hit - 0.5 * G_IN_S2 * t_hit**2
            vy_at_target = vy0 - G_IN_S2 * t_hit
            # Must hit target height AND be descending
            if abs(y_at_target - TARGET_HEIGHT_IN) < 3 and vy_at_target < 0:
                entry_angle = np.degrees(np.arctan2(abs(vy_at_target), vx0))
                if entry_angle > best_entry:
                    best_entry = entry_angle
                    best = {'angle_deg': theta_deg, 'entry_angle_deg': entry_angle, 'flight_time_s': t_hit}
        return best


def find_minimum_velocity_for_regime_absolute(name, robot_x_range, release_height=18,
                                               velocity_range=(20, 80)):
    """
    Find minimum velocity for a regime using ABSOLUTE field coordinates.

    SCORING: Robot in friendly zone, shooting INTO the HUB (lob shot)
    PASSING: Robot in neutral zone, shooting OVER wall into friendly zone
    STEALING: Robot in opposing zone, shooting OVER wall into friendly zone

    Returns: (min_velocity, coverage_data) or (None, reason)
    """

    for v in range(velocity_range[0], velocity_range[1] + 1):
        all_covered = True
        solutions = []

        # Test 10 positions across the robot's operating range
        for robot_x in np.linspace(robot_x_range[0], robot_x_range[1], 10):

            if name == "SCORING":
                # SCORING: Robot at robot_x, shooting at HUB at x=183
                # Distance to HUB
                distance = FIELD_HUB_X - robot_x
                if distance <= 0:
                    continue  # Robot is past the HUB

                solution = find_lob_solution(distance, v, release_height)
                if solution:
                    solutions.append({
                        'robot_x': robot_x,
                        'target_x': FIELD_HUB_X,
                        'angle': solution['angle_deg'],
                        'flight_time': solution['flight_time_s']
                    })
                else:
                    all_covered = False
                    break

            else:
                # PASSING/STEALING: Must clear wall at x=183, land in friendly zone
                solution = find_wall_clearing_trajectory_absolute(
                    robot_x, v, FIELD_HUB_X, FIELD_WALL_HEIGHT,
                    (0, FIELD_LANDING_ZONE_DEPTH), release_height
                )
                if solution:
                    solutions.append({
                        'robot_x': robot_x,
                        'landing_x': solution['optimal_landing_x'],
                        'angle': solution['optimal_angle'],
                        'clearance': solution['optimal_clearance'],
                        'flight_time': solution['optimal_flight_time']
                    })
                else:
                    all_covered = False
                    break

        if all_covered and solutions:
            angles = [s['angle'] for s in solutions]
            flight_times = [s['flight_time'] for s in solutions]
            return v, {
                'min_angle': min(angles),
                'max_angle': max(angles),
                'min_flight_time': min(flight_times),
                'max_flight_time': max(flight_times),
                'solutions': solutions
            }

    return None, f"No solution found in velocity range {velocity_range[0]}-{velocity_range[1]} ft/s"


def unified_regime_analysis(release_height=18):
    """
    Find minimum velocities for all three regimes and display results.
    """

    print("\n" + "="*80)
    print("  UNIFIED VELOCITY REGIME ANALYSIS (Absolute Field Coordinates)")
    print("="*80)

    # Define regime robot position ranges
    regimes = {
        'SCORING': {
            'robot_x_range': (24, 170),  # In friendly zone, shooting at HUB
            'description': 'Friendly zone → HUB (lob shot)',
            'velocity_range': (20, 45)
        },
        'PASSING': {
            'robot_x_range': (220, 450),  # In neutral zone
            'description': 'Neutral zone → Friendly zone (clear wall)',
            'velocity_range': (25, 60)
        },
        'STEALING': {
            'robot_x_range': (490, 630),  # In opposing zone
            'description': 'Opposing zone → Friendly zone (clear wall)',
            'velocity_range': (40, 80)
        }
    }

    results = {}

    print(f"\n  Release height: {release_height}\"")
    print(f"  Wall position: x = {FIELD_HUB_X}\" (height {FIELD_WALL_HEIGHT}\")")
    print(f"  Landing zone: x = 0\" to {FIELD_LANDING_ZONE_DEPTH}\"")

    print(f"\n  {'Regime':<12} | {'Robot Range':>18} | {'Min Velocity':>14} | {'Angles':>18} | {'Flight Times':>18}")
    print("  " + "-"*90)

    for regime_name, config in regimes.items():
        min_v, data = find_minimum_velocity_for_regime_absolute(
            regime_name,
            config['robot_x_range'],
            release_height,
            config['velocity_range']
        )

        if min_v:
            results[regime_name] = {
                'velocity': min_v,
                'data': data,
                'robot_x_range': config['robot_x_range']
            }
            print(f"  {regime_name:<12} | x = {config['robot_x_range'][0]:>3}\" - {config['robot_x_range'][1]:>3}\" | "
                  f"{min_v:>10} ft/s | {data['min_angle']:>6.1f}° - {data['max_angle']:>5.1f}° | "
                  f"{data['min_flight_time']:>5.2f}s - {data['max_flight_time']:>5.2f}s")
        else:
            results[regime_name] = None
            print(f"  {regime_name:<12} | x = {config['robot_x_range'][0]:>3}\" - {config['robot_x_range'][1]:>3}\" | "
                  f"{'NO SOLUTION':^14} | {data}")

    # Visualization: Field elevation with all regimes
    print("\n\n  Generating field elevation view...")

    fig, ax = plt.subplots(figsize=(22, 10))

    # Field zones
    ax.fill_between([0, FIELD_HUB_X], 0, 350, color='lightblue', alpha=0.3)
    ax.fill_between([FIELD_HUB_X, FIELD_NEUTRAL_END_X], 0, 350, color='lightgray', alpha=0.3)
    ax.fill_between([FIELD_NEUTRAL_END_X, FIELD_OPPOSING_WALL_X], 0, 350, color='lightsalmon', alpha=0.3)

    # Ground
    ax.axhline(y=0, color='saddlebrown', linewidth=4)

    # HUB at x=183
    hub_w = 30
    ax.fill_between([FIELD_HUB_X - hub_w/2, FIELD_HUB_X + hub_w/2],
                    FIELD_HUB_HEIGHT - 8, FIELD_HUB_HEIGHT + 3,
                    color='green', alpha=0.8)
    ax.plot([FIELD_HUB_X - hub_w/2, FIELD_HUB_X - hub_w/2 - 12],
            [FIELD_HUB_HEIGHT, FIELD_HUB_HEIGHT - 25], 'g-', linewidth=2)
    ax.plot([FIELD_HUB_X + hub_w/2, FIELD_HUB_X + hub_w/2 + 12],
            [FIELD_HUB_HEIGHT, FIELD_HUB_HEIGHT - 25], 'g-', linewidth=2)
    ax.text(FIELD_HUB_X, FIELD_HUB_HEIGHT + 12, 'HUB (72\")', ha='center', fontsize=9, fontweight='bold')

    # Walls
    ax.fill_between([FIELD_HUB_X - 4, FIELD_HUB_X + 4], 0, FIELD_WALL_HEIGHT,
                    color='red', alpha=0.6)
    ax.fill_between([FIELD_NEUTRAL_END_X - 4, FIELD_NEUTRAL_END_X + 4], 0, FIELD_WALL_HEIGHT,
                    color='red', alpha=0.4)

    # Landing zone
    ax.fill_between([0, FIELD_LANDING_ZONE_DEPTH], 0, 12, color='green', alpha=0.2)
    ax.text(90, 6, 'Landing Zone', ha='center', fontsize=8, color='darkgreen')

    # Draw trajectories for each regime
    regime_colors = {'SCORING': 'blue', 'PASSING': 'purple', 'STEALING': 'orange'}

    for regime_name, result in results.items():
        if result is None:
            continue

        v = result['velocity']
        color = regime_colors[regime_name]

        # Sample 3 robot positions
        robot_xs = np.linspace(result['robot_x_range'][0], result['robot_x_range'][1], 3)

        for i, robot_x in enumerate(robot_xs):
            alpha = 0.5 + 0.2 * i

            if regime_name == "SCORING":
                # Scoring: left to right
                distance = FIELD_HUB_X - robot_x
                solution = find_lob_solution(distance, v, release_height)
                if solution:
                    theta = np.radians(solution['angle_deg'])
                    v_shot = v * 12
                    vx0 = v_shot * np.cos(theta)
                    vy0 = v_shot * np.sin(theta)

                    t = np.linspace(0, 2, 300)
                    x = robot_x + vx0 * t  # Left to right
                    y = release_height + vy0 * t - 0.5 * G_IN_S2 * t**2

                    valid = (x <= FIELD_HUB_X + 15) & (y >= -5)
                    ax.plot(x[valid], y[valid], color=color, linewidth=2, alpha=alpha)
                    ax.plot(robot_x, release_height, 's', color=color, markersize=8)
            else:
                # Passing/Stealing: right to left
                solution = find_wall_clearing_trajectory_absolute(
                    robot_x, v, FIELD_HUB_X, FIELD_WALL_HEIGHT,
                    (0, FIELD_LANDING_ZONE_DEPTH), release_height
                )
                if solution:
                    theta = np.radians(solution['optimal_angle'])
                    v_shot = v * 12
                    vx0 = v_shot * np.cos(theta)
                    vy0 = v_shot * np.sin(theta)

                    t = np.linspace(0, 4, 500)
                    x = robot_x - vx0 * t  # Right to left
                    y = release_height + vy0 * t - 0.5 * G_IN_S2 * t**2

                    valid = (x >= -20) & (y >= -5)
                    ax.plot(x[valid], y[valid], color=color, linewidth=2, alpha=alpha)
                    marker = 'o' if regime_name == 'PASSING' else '^'
                    ax.plot(robot_x, release_height, marker, color=color, markersize=8)

    # Legend entries
    for regime_name, result in results.items():
        if result:
            color = regime_colors[regime_name]
            marker = 's' if regime_name == 'SCORING' else ('o' if regime_name == 'PASSING' else '^')
            ax.plot([], [], color=color, linewidth=2, marker=marker, markersize=8,
                   label=f'{regime_name}: {result["velocity"]} ft/s')

    # Zone labels
    ax.text(90, 320, 'FRIENDLY\nALLIANCE', ha='center', fontsize=12, fontweight='bold', color='navy')
    ax.text(320, 320, 'NEUTRAL\nZONE', ha='center', fontsize=12, fontweight='bold', color='gray')
    ax.text(560, 320, 'OPPOSING\nALLIANCE', ha='center', fontsize=12, fontweight='bold', color='darkred')

    # Axis labels with field coordinate markers
    ax.set_xlim(-30, FIELD_OPPOSING_WALL_X + 30)
    ax.set_ylim(-30, 360)
    ax.set_xlabel('Field Position (inches from friendly alliance wall)', fontsize=11)
    ax.set_ylabel('Height (inches)', fontsize=11)
    ax.set_title('UNIFIED REGIME ANALYSIS - Minimum Velocities\n(All Regimes at Calculated Minimum Speeds)',
                 fontsize=13, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)

    # Info box
    if all(results.values()):
        info = f"""MINIMUM VELOCITIES FOUND:
SCORING:  {results['SCORING']['velocity']:>2} ft/s (lob into HUB)
PASSING:  {results['PASSING']['velocity']:>2} ft/s (clear 48\" wall)
STEALING: {results['STEALING']['velocity']:>2} ft/s (clear 48\" wall)

Release height: {release_height}\""""
    else:
        info = "Some regimes have no valid solution.\nIncrease velocity range or check constraints."

    ax.text(0.02, 0.98, info, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

    plt.tight_layout()
    plt.show()

    return results


# Run the unified analysis
unified_results = unified_regime_analysis(release_height=18)
