"You are an expert FRC Engineering Mentor. I am coaching a team of students on a shooter design for the 2026 game, REBUILT.
The Goal: Design a multi-barrel shooter capable of 180-degree elevation (horizon-to-horizon). The Strategy: Use a constant-speed roller system (high inertia) to fire multiple FUEL pieces simultaneously. Aiming is handled strictly by adjusting the elevation angle, not roller speed. We prefer high-angle trajectories for better entry into the goal and to maintain high roller RPM for rate of fire.
Physical Constraints from the Manual:
1. The Projectile: 5.91" diameter foam ball weighing ~0.5 lbs.
2. The Target: 72" high opening, 41.7" wide.
3. The Robot: Maximum total height is 30" including all extensions.
4. Expansion: Only 12" horizontal extension beyond the perimeter, and only in one direction at a time.
5. Field Distances: Robots must shoot from the Alliance Zone (0 to 250" from the target).
Your Task:
1. Help the students derive the kinematic equations for trajectories where the release point is below 22" and the target is 72".
2. Analyze the High angle trajectories (i.e. lob shots not straight shots) for distances between 2 and 12 feet. - The target is like a bucket, not a hole in the wall. scoring must happen from above, i.e. the projectile has downward velocity.
3. Calculate the required rotational inertia for the rollers to fire three balls simultaneously with less than a 5% drop in RPM.
4. Help us create Python/Matplotlib code to visualize these trajectories within the 30" height cap.
~~5. Provide feedback on the mechanical risk of a 'one direction at a time' extension rule (R106) for a 180-degree pivoting shooter."~~ (shooter should have zero muzzle length)  