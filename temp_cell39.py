# =============================================================================
# FIELD GEOMETRY & TRAJECTORY FUNCTIONS (Absolute Coordinates)
# =============================================================================
#
# COORDINATE SYSTEM:
#   x = 0        : Far wall of friendly alliance zone
#   x = 183"     : Wall/HUB boundary (wall 48" tall, HUB rim 72" high)
#   x = 470"     : Neutral/Opposing alliance boundary
#   x = 650"     : Far wall of opposing alliance zone
#
# LANDING ZONE: 0 < x < 183" (about 180" usable before the wall)
# =============================================================================

import numpy as np
import matplotlib.pyplot as plt

# Field geometry constants (in inches)
FIELD_FRIENDLY_WALL_X = 0          # Far end of friendly alliance
FIELD_HUB_X = 183                  # HUB and wall position
FIELD_NEUTRAL_END_X = 470          # End of neutral zone
FIELD_OPPOSING_WALL_X = 650        # Far end of opposing alliance
FIELD_WALL_HEIGHT = 48             # Wall height (safe clearance)
FIELD_HUB_HEIGHT = 72              # HUB funnel rim height
FIELD_LANDING_ZONE_DEPTH = 180     # Usable landing depth in alliance zone

# Physics constants
if 'G_IN_S2' not in dir():
    G_IN_S2 = 386.09  # gravity in in/s^2
if 'TARGET_HEIGHT_IN' not in dir():
    TARGET_HEIGHT_IN = 72  # funnel height for scoring

def find_wall_clearing_trajectory_absolute(robot_x, velocity_fps,
                                           wall_x=FIELD_HUB_X, wall_height=FIELD_WALL_HEIGHT,
                                           landing_x_range=(0, 180), release_height=18):
    """
    Find launch angles for a shot in ABSOLUTE field coordinates.

    Args:
        robot_x: Robot's x position on field (inches)
        velocity_fps: Ball exit velocity (ft/s)
        wall_x: X position of wall to clear (default: 183" = HUB boundary)
        wall_height: Height of wall (default: 48")
        landing_x_range: (min_x, max_x) tuple for valid landing zone
        release_height: Height of ball release above floor

    Returns:
        dict with optimal_angle, wall_clearance, flight_time, landing_x, etc.
        or None if no valid trajectory exists
    """
    v_shot = velocity_fps * 12  # Convert to in/s

    # Validate geometry: robot should be on the "far" side of the wall
    # (i.e., ball travels from robot toward wall, then past it)
    if robot_x <= wall_x:
        return None  # Robot is already past the wall, can't "clear" it

    distance_to_wall = robot_x - wall_x

    valid_solutions = []

    # Scan launch angles
    for theta_deg in np.linspace(10, 80, 1400):
        theta = np.radians(theta_deg)
        vx0 = v_shot * np.cos(theta)  # Always positive (shooting toward smaller x)
        vy0 = v_shot * np.sin(theta)

        if vx0 <= 0:
            continue

        # Time to reach wall (x = wall_x when robot_x - vx0*t = wall_x)
        t_wall = distance_to_wall / vx0

        # Height at wall
        y_at_wall = release_height + vy0 * t_wall - 0.5 * G_IN_S2 * t_wall**2
        wall_clearance = y_at_wall - wall_height

        # Must clear wall with some margin
        if wall_clearance < 2:  # 2" safety margin
            continue

        # Find landing point (where y = 0)
        # Solve: release_height + vy0*t - 0.5*g*t^2 = 0
        # Using quadratic formula: t = (vy0 + sqrt(vy0^2 + 2*g*h0)) / g
        discriminant = vy0**2 + 2 * G_IN_S2 * release_height
        if discriminant < 0:
            continue
        t_land = (vy0 + np.sqrt(discriminant)) / G_IN_S2

        # Landing position
        landing_x = robot_x - vx0 * t_land

        # Check if landing is within valid zone
        if landing_x < landing_x_range[0] or landing_x > landing_x_range[1]:
            continue

        # Max height of trajectory
        t_apex = vy0 / G_IN_S2
        max_height = release_height + vy0 * t_apex - 0.5 * G_IN_S2 * t_apex**2

        valid_solutions.append({
            'angle_deg': theta_deg,
            'wall_clearance': wall_clearance,
            'max_height': max_height,
            'flight_time': t_land,
            'landing_x': landing_x,
            'landing_distance': robot_x - landing_x,
            't_wall': t_wall,
            'y_at_wall': y_at_wall,
            'vx0': vx0,
            'vy0': vy0
        })

    if not valid_solutions:
        return None

    # Select optimal: maximize wall clearance (safest trajectory)
    optimal = max(valid_solutions, key=lambda s: s['wall_clearance'])

    return {
        'optimal_angle': optimal['angle_deg'],
        'optimal_clearance': optimal['wall_clearance'],
        'optimal_max_height': optimal['max_height'],
        'optimal_flight_time': optimal['flight_time'],
        'optimal_landing_x': optimal['landing_x'],
        'min_angle': min(s['angle_deg'] for s in valid_solutions),
        'max_angle': max(s['angle_deg'] for s in valid_solutions),
        'all_solutions': valid_solutions
    }


def find_wall_clearing_trajectory(distance_in, velocity_fps, wall_height_in=48,
                                   wall_distance_in=None, release_height_in=18):
    """
    Legacy interface - finds trajectory to clear wall at specified distance.

    This is the RELATIVE coordinate version (distance from shooter).
    Wall is assumed to be between shooter and landing zone.

    Args:
        distance_in: Total distance from shooter to landing point
        velocity_fps: Ball exit velocity (ft/s)
        wall_height_in: Height of wall to clear (default 48")
        wall_distance_in: Distance from shooter to wall (default: 0.75 * distance)
        release_height_in: Height of ball release

    Returns:
        dict with angle info or None
    """
    v_shot = velocity_fps * 12  # Convert to in/s

    # If wall_distance not specified, assume wall is at 75% of total distance
    # (i.e., landing zone is 25% of distance past the wall)
    if wall_distance_in is None:
        wall_distance_in = distance_in * 0.75

    valid_angles = []

    for theta_deg in np.linspace(10, 80, 1400):
        theta = np.radians(theta_deg)
        vx0 = v_shot * np.cos(theta)
        vy0 = v_shot * np.sin(theta)

        if vx0 <= 0:
            continue

        # Time to reach wall
        t_wall = wall_distance_in / vx0
        y_at_wall = release_height_in + vy0 * t_wall - 0.5 * G_IN_S2 * t_wall**2

        # Must clear wall
        wall_clearance = y_at_wall - wall_height_in
        if wall_clearance < 2:
            continue

        # Time to reach target distance
        t_target = distance_in / vx0
        y_at_target = release_height_in + vy0 * t_target - 0.5 * G_IN_S2 * t_target**2

        # Ball should be descending toward ground at landing
        # Accept if ball is above ground or within reasonable landing window
        if y_at_target < -10:  # Too far past ground
            continue
        if y_at_target > 60:  # Still way too high (hasn't descended)
            continue

        t_apex = vy0 / G_IN_S2
        max_height = release_height_in + vy0 * t_apex - 0.5 * G_IN_S2 * t_apex**2

        valid_angles.append({
            'angle_deg': theta_deg,
            'wall_clearance': wall_clearance,
            'max_height': max_height,
            'flight_time': t_target,
            'y_at_target': y_at_target,
            'vx0': vx0,
            'vy0': vy0
        })

    if not valid_angles:
        return None

    # Select optimal: maximize wall clearance
    optimal = max(valid_angles, key=lambda s: s['wall_clearance'])

    return {
        'optimal_angle': optimal['angle_deg'],
        'optimal_clearance': optimal['wall_clearance'],
        'optimal_max_height': optimal['max_height'],
        'optimal_flight_time': optimal['flight_time'],
        'min_angle': min(s['angle_deg'] for s in valid_angles),
        'max_angle': max(s['angle_deg'] for s in valid_angles),
        'all_solutions': valid_angles
    }


def analyze_regime(name, min_dist_in, max_dist_in, velocity_fps,
                   wall_height=48, release_height=18, num_points=10):
    """
    Analyze a shooting regime across its distance range.

    Uses RELATIVE coordinates (distances from shooter position).
    """
    print(f"\n{'='*70}")
    print(f"  {name} REGIME ANALYSIS")
    print(f"  Velocity: {velocity_fps} ft/s | Wall: {wall_height}\" | Release: {release_height}\"")
    print(f"  Distance range: {min_dist_in}\" to {max_dist_in}\" ({min_dist_in/12:.1f} to {max_dist_in/12:.1f} ft)")
    print(f"{'='*70}")

    distances = np.linspace(min_dist_in, max_dist_in, num_points)
    results = []

    print(f"\n  {'Distance':>10} | {'Optimal':>8} | {'Clearance':>10} | {'Max Ht':>8} | {'Flight':>8}")
    print("  " + "-"*60)

    for d in distances:
        solution = find_wall_clearing_trajectory(d, velocity_fps, wall_height,
                                                  release_height_in=release_height)
        if solution:
            results.append({
                'distance': d,
                'angle': solution['optimal_angle'],
                'clearance': solution['optimal_clearance'],
                'max_height': solution['optimal_max_height'],
                'flight_time': solution['optimal_flight_time']
            })
            print(f"  {d:>8.0f}\" | {solution['optimal_angle']:>6.1f} deg | "
                  f"{solution['optimal_clearance']:>7.1f}\" | {solution['optimal_max_height']:>6.0f}\" | "
                  f"{solution['optimal_flight_time']:>6.2f} s")
        else:
            print(f"  {d:>8.0f}\" | {'NO SOLUTION':^48}")

    if results:
        print(f"\n  Summary: {len(results)}/{num_points} distances covered")
        print(f"  Angle range: {min(r['angle'] for r in results):.1f} deg to {max(r['angle'] for r in results):.1f} deg")
        print(f"  Flight times: {min(r['flight_time'] for r in results):.2f}s to {max(r['flight_time'] for r in results):.2f}s")
    else:
        print(f"\n  WARNING: No valid trajectories found at {velocity_fps} ft/s!")

    return results


def analyze_regime_absolute(name, robot_x_range, velocity_fps,
                            wall_x=FIELD_HUB_X, wall_height=FIELD_WALL_HEIGHT,
                            landing_x_range=(0, 180), release_height=18, num_points=10):
    """
    Analyze a shooting regime using ABSOLUTE field coordinates.

    Args:
        name: Regime name for display
        robot_x_range: (min_x, max_x) tuple of robot positions on field
        velocity_fps: Ball exit velocity
        wall_x: X position of wall to clear
        wall_height: Wall height
        landing_x_range: Valid landing zone
        release_height: Ball release height
        num_points: Number of test points

    Returns:
        List of solution dicts
    """
    print(f"\n{'='*75}")
    print(f"  {name} REGIME ANALYSIS (Absolute Coordinates)")
    print(f"  Velocity: {velocity_fps} ft/s | Wall at x={wall_x}\" ({wall_height}\" tall)")
    print(f"  Robot positions: x = {robot_x_range[0]}\" to {robot_x_range[1]}\"")
    print(f"  Landing zone: x = {landing_x_range[0]}\" to {landing_x_range[1]}\"")
    print(f"{'='*75}")

    robot_positions = np.linspace(robot_x_range[0], robot_x_range[1], num_points)
    results = []

    print(f"\n  {'Robot X':>8} | {'Optimal':>8} | {'Clearance':>10} | {'Land X':>8} | {'Flight':>8}")
    print("  " + "-"*65)

    for robot_x in robot_positions:
        solution = find_wall_clearing_trajectory_absolute(
            robot_x, velocity_fps, wall_x, wall_height, landing_x_range, release_height
        )
        if solution:
            results.append({
                'robot_x': robot_x,
                'angle': solution['optimal_angle'],
                'clearance': solution['optimal_clearance'],
                'landing_x': solution['optimal_landing_x'],
                'flight_time': solution['optimal_flight_time'],
                'max_height': solution['optimal_max_height']
            })
            print(f"  {robot_x:>7.0f}\" | {solution['optimal_angle']:>6.1f} deg | "
                  f"{solution['optimal_clearance']:>7.1f}\" | {solution['optimal_landing_x']:>6.0f}\" | "
                  f"{solution['optimal_flight_time']:>6.2f} s")
        else:
            print(f"  {robot_x:>7.0f}\" | {'NO SOLUTION':^50}")

    if results:
        print(f"\n  Summary: {len(results)}/{num_points} positions covered")
    else:
        print(f"\n  WARNING: No valid trajectories found at {velocity_fps} ft/s!")

    return results


print("Field geometry and trajectory functions loaded.")
print(f"  Wall position: x = {FIELD_HUB_X}\" (height {FIELD_WALL_HEIGHT}\")")
print(f"  Landing zone: x = 0\" to {FIELD_LANDING_ZONE_DEPTH}\"")
print(f"  Neutral zone: x = {FIELD_HUB_X}\" to {FIELD_NEUTRAL_END_X}\"")
print(f"  Opposing zone: x = {FIELD_NEUTRAL_END_X}\" to {FIELD_OPPOSING_WALL_X}\"")
