# -*- coding: utf-8 -*-
"""
FRC Shooter System Simulator - Integrated with Trajectory Analysis
===================================================================

Models a wide roller shooter with closed-loop velocity control:
- Single roller that is N game-pieces wide
- Closed-loop motor control maintains target RPM
- Recovery time is the key metric (how fast to shoot again)
- Integrated with lob trajectory calculations

For FRC 2026 REBUILT - Team 2491

Key Physics:
------------
1. Wide roller inertia scales with width: I = I_per_unit_width * N
2. Energy per burst: E = N * (0.5 * m_ball * v^2) / efficiency
3. Closed-loop control: Motors maintain target RPM, wait for recovery
4. Recovery time: How long to spin back up after N-ball burst
5. Ball velocity = surface_speed * sqrt(energy_efficiency) * velocity_transfer_factor
   - energy_efficiency: compression/slip losses during contact (~0.80)
   - velocity_transfer_factor: dual-wheel geometry averaging (~0.75 for hood roller)

Design Philosophy:
------------------
- Gear for motor sweet spot (70-80% of free speed) for fast recovery
- Heavier flywheel = less RPM drop = faster recovery to target
- Width trades off: more balls/burst vs more energy drain
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from enum import Enum


# =============================================================================
# CONSTANTS
# =============================================================================

# FUEL properties (2026 game piece)
FUEL_DIAMETER_IN = 5.91
FUEL_RADIUS_IN = FUEL_DIAMETER_IN / 2
FUEL_MASS_LB = 0.47
FUEL_MASS_KG = FUEL_MASS_LB * 0.4536  # 0.213 kg

# Target properties
TARGET_HEIGHT_IN = 72

# Robot constraints
ROBOT_MAX_HEIGHT_IN = 30
PRACTICAL_RELEASE_HEIGHT_IN = 22  # Max Z extent for shooter

# Gravity
G_IN_S2 = 386.09  # in/s^2

# Unit conversions
IN_TO_M = 0.0254
LB_TO_KG = 0.4536
RPM_TO_RAD_S = 2 * np.pi / 60
RAD_S_TO_RPM = 60 / (2 * np.pi)
FT_S_TO_IN_S = 12
IN_S_TO_FT_S = 1/12


# =============================================================================
# MOTOR DATABASE
# =============================================================================

@dataclass
class MotorSpec:
    """FRC motor specifications."""
    name: str
    free_speed_rpm: float
    stall_torque_nm: float
    stall_current_a: float
    free_current_a: float
    weight_lb: float

    @property
    def free_speed_rad_s(self) -> float:
        return self.free_speed_rpm * RPM_TO_RAD_S

    def torque_at_speed(self, omega_rad_s: float, voltage: float = 12.0) -> float:
        """Calculate torque at given angular velocity (linear model)."""
        omega_free = self.free_speed_rad_s * (voltage / 12.0)
        tau_stall = self.stall_torque_nm * (voltage / 12.0)

        if omega_rad_s >= omega_free:
            return 0.0
        if omega_rad_s <= 0:
            return tau_stall

        return tau_stall * (1 - omega_rad_s / omega_free)

    def current_at_speed(self, omega_rad_s: float, voltage: float = 12.0) -> float:
        """Calculate current draw at given speed."""
        omega_free = self.free_speed_rad_s * (voltage / 12.0)
        if omega_rad_s >= omega_free:
            return self.free_current_a
        ratio = omega_rad_s / omega_free
        return self.stall_current_a - (self.stall_current_a - self.free_current_a) * ratio

    def power_out_at_speed(self, omega_rad_s: float, voltage: float = 12.0) -> float:
        """Mechanical power output (watts)."""
        return self.torque_at_speed(omega_rad_s, voltage) * omega_rad_s


# Motor database
MOTORS = {
    'NEO': MotorSpec(
        name='REV NEO',
        free_speed_rpm=5676,
        stall_torque_nm=2.6,
        stall_current_a=105,
        free_current_a=1.8,
        weight_lb=0.94
    ),
    'NEO550': MotorSpec(
        name='REV NEO 550',
        free_speed_rpm=11000,
        stall_torque_nm=0.97,
        stall_current_a=100,
        free_current_a=1.4,
        weight_lb=0.31
    ),
    'FALCON500': MotorSpec(
        name='VEX Falcon 500',
        free_speed_rpm=6380,
        stall_torque_nm=4.69,
        stall_current_a=257,
        free_current_a=1.5,
        weight_lb=1.1
    ),
    'KRAKENX60': MotorSpec(
        name='WCP Kraken X60',
        free_speed_rpm=6000,
        stall_torque_nm=7.09,
        stall_current_a=366,
        free_current_a=2.0,
        weight_lb=1.23
    ),
    'KRAKENX44': MotorSpec(
        name='WCP Kraken X44',
        free_speed_rpm=5800,
        stall_torque_nm=1.36,
        stall_current_a=63,
        free_current_a=1.5,
        weight_lb=0.39
    ),
}


# =============================================================================
# WIDE ROLLER SHOOTER SYSTEM
# =============================================================================

@dataclass
class WideRollerShooter:
    """
    A shooter with a single wide roller that handles N game pieces across.

    The roller is a continuous cylinder - when you fire N balls simultaneously,
    all N balls see the same surface speed and the roller loses N times the energy.

    With closed-loop control, motors maintain target RPM. After a burst,
    the key question is: how long to recover back to target RPM?
    """
    # Motor configuration
    motor_type: str = 'KRAKENX60'
    num_motors: int = 2
    gear_ratio: float = 1.5  # Motor:roller (>1 = step down to roller)
    gearbox_efficiency: float = 0.95

    # Roller configuration
    roller_diameter_in: float = 4.0
    roller_width_game_pieces: int = 1  # How many balls wide
    roller_mass_per_inch_lb: float = 0.3  # Mass per inch of roller length
    added_flywheel_inertia_lb_in2: float = 0.0  # Additional flywheel

    # Energy transfer
    energy_transfer_efficiency: float = 0.80  # Ball gets 80% of theoretical energy
    velocity_transfer_factor: float = 0.75  # Ball velocity / main wheel surface speed (dual-wheel geometry)
    compression_in: float = 0.5  # Ball compression

    def __post_init__(self):
        if self.motor_type not in MOTORS:
            raise ValueError(f"Unknown motor: {self.motor_type}")
        self._motor = MOTORS[self.motor_type]

    @property
    def motor(self) -> MotorSpec:
        return self._motor

    @property
    def roller_radius_in(self) -> float:
        return self.roller_diameter_in / 2

    @property
    def roller_radius_m(self) -> float:
        return self.roller_radius_in * IN_TO_M

    @property
    def effective_radius_in(self) -> float:
        """Contact radius accounting for compression."""
        return self.roller_radius_in - self.compression_in / 2

    @property
    def effective_radius_m(self) -> float:
        return self.effective_radius_in * IN_TO_M

    @property
    def roller_length_in(self) -> float:
        """Total roller length based on game pieces it handles."""
        # Each game piece needs ~6" of roller width + margin
        return self.roller_width_game_pieces * (FUEL_DIAMETER_IN + 0.5)

    @property
    def roller_mass_lb(self) -> float:
        """Total roller mass."""
        return self.roller_length_in * self.roller_mass_per_inch_lb

    @property
    def roller_inertia_kg_m2(self) -> float:
        """Moment of inertia of the roller (solid cylinder)."""
        mass_kg = self.roller_mass_lb * LB_TO_KG
        r_m = self.roller_radius_m
        # Solid cylinder: I = 0.5 * m * r^2
        return 0.5 * mass_kg * r_m**2

    @property
    def total_inertia_kg_m2(self) -> float:
        """Total rotational inertia (both rollers + any added flywheel)."""
        # Top and bottom rollers (geared together)
        roller_inertia = self.roller_inertia_kg_m2 * 2
        # Added flywheel
        added_inertia = self.added_flywheel_inertia_lb_in2 * (LB_TO_KG * IN_TO_M**2)
        return roller_inertia + added_inertia

    @property
    def total_inertia_lb_in2(self) -> float:
        return self.total_inertia_kg_m2 / (LB_TO_KG * IN_TO_M**2)

    # =========================================================================
    # RPM and Velocity Calculations
    # =========================================================================

    def surface_velocity_fps(self, roller_rpm: float) -> float:
        """Surface velocity of roller at given RPM."""
        omega = roller_rpm * RPM_TO_RAD_S
        v_m_s = omega * self.effective_radius_m
        return v_m_s / IN_TO_M * IN_S_TO_FT_S

    def ball_exit_velocity_fps(self, roller_rpm: float) -> float:
        """
        Ball exit velocity accounting for energy efficiency and dual-wheel geometry.

        For dual-wheel shooters (main wheel + hood roller at different speeds):
        - velocity_transfer_factor accounts for ball velocity being the average
          of both wheel surface speeds (e.g., 0.75 for hood at 2x RPM, half radius)
        - energy_transfer_efficiency accounts for compression/slip losses
        """
        return (self.surface_velocity_fps(roller_rpm) *
                np.sqrt(self.energy_transfer_efficiency) *
                self.velocity_transfer_factor)

    def rpm_for_ball_velocity(self, v_ball_fps: float) -> float:
        """
        Required roller RPM to achieve desired ball exit velocity.

        Inverse of ball_exit_velocity_fps: accounts for both energy efficiency
        and velocity transfer factor from dual-wheel geometry.
        """
        # v_ball = v_surface * sqrt(eff) * transfer_factor
        # v_surface = v_ball / (sqrt(eff) * transfer_factor)
        v_surface_fps = v_ball_fps / (np.sqrt(self.energy_transfer_efficiency) *
                                       self.velocity_transfer_factor)
        v_surface_m_s = v_surface_fps * FT_S_TO_IN_S * IN_TO_M
        omega = v_surface_m_s / self.effective_radius_m
        return omega * RAD_S_TO_RPM

    def motor_rpm_for_roller_rpm(self, roller_rpm: float) -> float:
        """Motor RPM required for given roller RPM."""
        return roller_rpm * self.gear_ratio

    def max_roller_rpm(self, voltage: float = 12.0) -> float:
        """Maximum achievable roller RPM."""
        motor_free_rpm = self._motor.free_speed_rpm * (voltage / 12.0)
        return motor_free_rpm / self.gear_ratio

    def max_ball_velocity_fps(self, voltage: float = 12.0) -> float:
        """Maximum ball exit velocity."""
        return self.ball_exit_velocity_fps(self.max_roller_rpm(voltage))

    # =========================================================================
    # Energy Calculations
    # =========================================================================

    def stored_energy_j(self, roller_rpm: float) -> float:
        """Kinetic energy stored in flywheel at given RPM."""
        omega = roller_rpm * RPM_TO_RAD_S
        return 0.5 * self.total_inertia_kg_m2 * omega**2

    def energy_per_ball_j(self, v_ball_fps: float) -> float:
        """Kinetic energy of one ball at given velocity."""
        v_m_s = v_ball_fps * FT_S_TO_IN_S * IN_TO_M
        return 0.5 * FUEL_MASS_KG * v_m_s**2

    def energy_drawn_per_ball_j(self, v_ball_fps: float) -> float:
        """Energy drawn from flywheel per ball (includes efficiency loss)."""
        return self.energy_per_ball_j(v_ball_fps) / self.energy_transfer_efficiency

    def energy_for_burst_j(self, num_balls: int, v_ball_fps: float) -> float:
        """Total energy drawn for N-ball burst."""
        return num_balls * self.energy_drawn_per_ball_j(v_ball_fps)

    # =========================================================================
    # Burst and Recovery Simulation (Closed-Loop Control)
    # =========================================================================

    def rpm_after_burst(self, initial_rpm: float, num_balls: int) -> Tuple[float, float]:
        """
        Calculate RPM after firing N balls (all balls exit at same velocity).

        With closed-loop control, all balls fire at the initial RPM's velocity.
        The flywheel slows down but control maintains the commanded velocity
        until we run out of headroom.

        Returns: (final_rpm, ball_exit_velocity_fps)
        """
        v_ball = self.ball_exit_velocity_fps(initial_rpm)
        e_drawn = self.energy_for_burst_j(num_balls, v_ball)
        e_before = self.stored_energy_j(initial_rpm)
        e_after = max(0, e_before - e_drawn)

        if e_after > 0:
            omega_after = np.sqrt(2 * e_after / self.total_inertia_kg_m2)
            rpm_after = omega_after * RAD_S_TO_RPM
        else:
            rpm_after = 0.0

        return rpm_after, v_ball

    def torque_at_roller(self, roller_rpm: float, voltage: float = 12.0) -> float:
        """Net torque at roller shaft from all motors."""
        motor_rpm = self.motor_rpm_for_roller_rpm(roller_rpm)
        motor_omega = motor_rpm * RPM_TO_RAD_S

        motor_torque = self._motor.torque_at_speed(motor_omega, voltage)
        total_torque = motor_torque * self.num_motors * self.gear_ratio * self.gearbox_efficiency

        return total_torque

    def recovery_time_s(self, start_rpm: float, target_rpm: float,
                        voltage: float = 12.0, dt: float = 0.0005) -> float:
        """
        Time to recover from start_rpm to target_rpm.

        This is THE key metric for closed-loop shooters:
        How long until we can fire again at full velocity?
        """
        if start_rpm >= target_rpm:
            return 0.0

        rpm = start_rpm
        t = 0.0
        max_rpm = self.max_roller_rpm(voltage)

        while rpm < target_rpm and t < 5.0:
            omega = rpm * RPM_TO_RAD_S
            torque = self.torque_at_roller(rpm, voltage)
            alpha = torque / self.total_inertia_kg_m2
            omega_new = omega + alpha * dt
            rpm = min(omega_new * RAD_S_TO_RPM, max_rpm)
            t += dt

        return t

    def spinup_time_s(self, target_rpm: float, voltage: float = 12.0) -> float:
        """Time to spin up from rest to target RPM."""
        return self.recovery_time_s(0, target_rpm, voltage)

    def simulate_burst_cycle(self, target_velocity_fps: float,
                             num_balls: int = None,
                             voltage: float = 12.0) -> Dict:
        """
        Simulate a complete burst cycle with closed-loop control.

        Cycle: Spin up -> Fire N balls -> Recover -> Ready for next burst

        If num_balls is None, fires roller_width_game_pieces balls.
        """
        if num_balls is None:
            num_balls = self.roller_width_game_pieces

        target_rpm = self.rpm_for_ball_velocity(target_velocity_fps)
        max_rpm = self.max_roller_rpm(voltage)

        # Check feasibility
        if target_rpm > max_rpm:
            return {
                'feasible': False,
                'reason': f'Target {target_rpm:.0f} RPM > max {max_rpm:.0f} RPM',
                'target_rpm': target_rpm,
                'max_rpm': max_rpm,
            }

        # Spinup
        spinup_time = self.spinup_time_s(target_rpm, voltage)

        # Fire burst
        rpm_after, v_exit = self.rpm_after_burst(target_rpm, num_balls)
        rpm_drop_pct = (target_rpm - rpm_after) / target_rpm * 100

        # Recovery
        recovery_time = self.recovery_time_s(rpm_after, target_rpm, voltage)

        # Cycle time (fire + recover)
        cycle_time = recovery_time  # Firing is ~instantaneous

        # Theoretical max rate
        if cycle_time > 0:
            bursts_per_second = 1.0 / cycle_time
            balls_per_second = num_balls / cycle_time
        else:
            bursts_per_second = float('inf')
            balls_per_second = float('inf')

        return {
            'feasible': True,
            'target_velocity_fps': target_velocity_fps,
            'target_rpm': target_rpm,
            'max_rpm': max_rpm,
            'headroom_pct': (max_rpm - target_rpm) / target_rpm * 100,
            'num_balls': num_balls,
            'exit_velocity_fps': v_exit,
            'spinup_time_s': spinup_time,
            'rpm_after_burst': rpm_after,
            'rpm_drop_pct': rpm_drop_pct,
            'recovery_time_s': recovery_time,
            'cycle_time_s': cycle_time,
            'bursts_per_second': bursts_per_second,
            'balls_per_second': balls_per_second,
        }

    def analyze_system(self, target_velocity_fps: float = 30.0) -> Dict:
        """Comprehensive system analysis."""
        results = self.simulate_burst_cycle(target_velocity_fps)

        if not results['feasible']:
            return results

        # Add configuration info
        results['config'] = {
            'motor': self.motor.name,
            'num_motors': self.num_motors,
            'gear_ratio': self.gear_ratio,
            'roller_diameter_in': self.roller_diameter_in,
            'roller_width': self.roller_width_game_pieces,
            'roller_mass_lb': self.roller_mass_lb,
            'total_inertia_lb_in2': self.total_inertia_lb_in2,
            'added_flywheel_lb_in2': self.added_flywheel_inertia_lb_in2,
        }

        # Energy analysis
        target_rpm = results['target_rpm']
        results['energy'] = {
            'stored_at_target_j': self.stored_energy_j(target_rpm),
            'per_ball_j': self.energy_drawn_per_ball_j(target_velocity_fps),
            'per_burst_j': self.energy_for_burst_j(results['num_balls'], target_velocity_fps),
        }

        return results


# =============================================================================
# TRAJECTORY INTEGRATION
# =============================================================================

def find_lob_solution(distance_in: float, velocity_fps: float = 30.0,
                      release_height_in: float = 18.0,
                      target_height_in: float = TARGET_HEIGHT_IN,
                      vx_robot: float = 0.0) -> Optional[Dict]:
    """
    Find launch angle for lob trajectory where ball DESCENDS into target.

    Returns dict with angle, entry_angle, max_height, etc. or None.
    """
    v_shot = velocity_fps * FT_S_TO_IN_S
    best = None
    best_entry = 0

    for theta_deg in np.linspace(5, 89.5, 2000):
        theta = np.radians(theta_deg)
        vx0 = v_shot * np.cos(theta) + vx_robot
        vy0 = v_shot * np.sin(theta)

        if vx0 <= 0:
            continue

        t_hit = distance_in / vx0
        y_at_target = release_height_in + vy0 * t_hit - 0.5 * G_IN_S2 * t_hit**2
        vy_at_target = vy0 - G_IN_S2 * t_hit

        # Ball must be at target height AND descending
        if abs(y_at_target - target_height_in) < 2.5 and vy_at_target < 0:
            entry_angle = np.degrees(np.arctan2(abs(vy_at_target), vx0))

            if entry_angle > best_entry:
                best_entry = entry_angle
                t_apex = vy0 / G_IN_S2
                max_height = release_height_in + vy0 * t_apex - 0.5 * G_IN_S2 * t_apex**2

                best = {
                    'angle_deg': theta_deg,
                    'entry_angle_deg': entry_angle,
                    'max_height_in': max_height,
                    'flight_time_s': t_hit,
                    'vx0': vx0,
                    'vy0': vy0,
                }

    return best


def calculate_trajectory_points(distance_in: float, theta_deg: float,
                                velocity_fps: float, release_height_in: float = 18.0,
                                vx_robot: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """Calculate trajectory points for plotting."""
    v_shot = velocity_fps * FT_S_TO_IN_S
    theta = np.radians(theta_deg)

    vx0 = v_shot * np.cos(theta) + vx_robot
    vy0 = v_shot * np.sin(theta)

    t = np.linspace(0, 3.0, 500)
    x = distance_in - vx0 * t
    y = release_height_in + vy0 * t - 0.5 * G_IN_S2 * t**2

    valid = (x >= -20) & (y >= -5)
    return x[valid], y[valid]


# =============================================================================
# INTEGRATED ANALYSIS
# =============================================================================

@dataclass
class IntegratedShooterAnalysis:
    """
    Combines shooter system and trajectory analysis.

    Given a shooter configuration, analyzes:
    - What distances can be reached
    - Launch angles required
    - Entry angles achieved
    - Shot timing and rate of fire
    """
    shooter: WideRollerShooter
    release_height_in: float = 18.0
    target_height_in: float = TARGET_HEIGHT_IN

    def analyze_shot(self, distance_in: float, robot_velocity_fps: float = 0.0) -> Dict:
        """
        Analyze a single shot at given distance.

        Returns shooter performance + trajectory data.
        """
        vx_robot = robot_velocity_fps * FT_S_TO_IN_S

        # Get shooter performance
        shooter_data = self.shooter.simulate_burst_cycle(
            target_velocity_fps=30.0,  # We'll adjust based on trajectory needs
        )

        if not shooter_data['feasible']:
            return {'feasible': False, 'reason': shooter_data['reason']}

        ball_velocity = shooter_data['exit_velocity_fps']

        # Find trajectory solution
        traj = find_lob_solution(
            distance_in, ball_velocity,
            self.release_height_in, self.target_height_in,
            vx_robot
        )

        if traj is None:
            return {
                'feasible': False,
                'reason': f'No trajectory solution at {distance_in/12:.1f} ft with {ball_velocity:.1f} fps'
            }

        # Quality assessment
        if traj['entry_angle_deg'] >= 60:
            quality = 'IDEAL'
        elif traj['entry_angle_deg'] >= 45:
            quality = 'GOOD'
        else:
            quality = 'MARGINAL'

        return {
            'feasible': True,
            'distance_in': distance_in,
            'distance_ft': distance_in / 12,
            'robot_velocity_fps': robot_velocity_fps,
            'ball_velocity_fps': ball_velocity,
            'launch_angle_deg': traj['angle_deg'],
            'entry_angle_deg': traj['entry_angle_deg'],
            'max_height_in': traj['max_height_in'],
            'flight_time_s': traj['flight_time_s'],
            'quality': quality,
            'shooter': shooter_data,
        }

    def analyze_range(self, distances_ft: List[float] = None,
                      robot_velocity_fps: float = 0.0) -> List[Dict]:
        """Analyze multiple distances."""
        if distances_ft is None:
            distances_ft = [3, 5, 7, 10, 12, 15, 18, 21]

        results = []
        for d_ft in distances_ft:
            result = self.analyze_shot(d_ft * 12, robot_velocity_fps)
            results.append(result)

        return results

    def print_analysis(self, target_velocity_fps: float = 30.0):
        """Print comprehensive analysis."""
        shooter_data = self.shooter.analyze_system(target_velocity_fps)

        print("\n" + "="*80)
        print("  INTEGRATED SHOOTER SYSTEM ANALYSIS")
        print("="*80)

        if not shooter_data['feasible']:
            print(f"\n  NOT FEASIBLE: {shooter_data['reason']}")
            return

        cfg = shooter_data['config']
        print(f"\n  CONFIGURATION:")
        print(f"    Motor: {cfg['motor']} x{cfg['num_motors']}")
        print(f"    Gear ratio: {cfg['gear_ratio']:.2f}:1")
        print(f"    Roller: {cfg['roller_diameter_in']:.1f}\" dia x {cfg['roller_width']} balls wide")
        print(f"    Roller mass: {cfg['roller_mass_lb']:.2f} lb")
        print(f"    Total inertia: {cfg['total_inertia_lb_in2']:.2f} lb-in²")

        print(f"\n  SHOOTER PERFORMANCE:")
        print(f"    Target ball velocity: {shooter_data['target_velocity_fps']:.1f} ft/s")
        print(f"    Required RPM: {shooter_data['target_rpm']:.0f}")
        print(f"    Max RPM: {shooter_data['max_rpm']:.0f} ({shooter_data['headroom_pct']:.0f}% headroom)")
        print(f"    Spinup time: {shooter_data['spinup_time_s']:.3f} s")

        print(f"\n  BURST PERFORMANCE ({shooter_data['num_balls']} balls):")
        print(f"    RPM drop: {shooter_data['rpm_drop_pct']:.1f}%")
        print(f"    Recovery time: {shooter_data['recovery_time_s']*1000:.0f} ms")
        print(f"    Theoretical rate: {shooter_data['balls_per_second']:.1f} balls/sec")

        print(f"\n  TRAJECTORY COVERAGE:")
        print(f"    {'Distance':>10} | {'Launch':>8} | {'Entry':>8} | {'Max Ht':>8} | {'Quality':>10}")
        print("    " + "-"*60)

        for d_ft in [3, 5, 7, 10, 12, 15, 18, 21]:
            result = self.analyze_shot(d_ft * 12)
            if result['feasible']:
                print(f"    {d_ft:>8} ft | {result['launch_angle_deg']:>6.1f}° | "
                      f"{result['entry_angle_deg']:>6.1f}° | {result['max_height_in']:>6.0f}\" | "
                      f"{result['quality']:>10}")
            else:
                print(f"    {d_ft:>8} ft | {'--- NO SOLUTION ---':^45}")

        print("="*80)


# =============================================================================
# DESIGN HELPERS
# =============================================================================

def calculate_optimal_gear_ratio(motor_type: str, target_velocity_fps: float,
                                 roller_diameter_in: float,
                                 energy_efficiency: float = 0.80,
                                 velocity_transfer_factor: float = 0.75) -> Dict:
    """
    Find gear ratio that puts motor in optimal range.

    For fast recovery with stable control, run motor at 70-80% of free speed.

    Args:
        energy_efficiency: Compression/slip losses (~0.80)
        velocity_transfer_factor: Dual-wheel geometry factor (~0.75)
    """
    motor = MOTORS[motor_type]

    # Required roller RPM
    # v_ball = v_surface * sqrt(energy_eff) * velocity_transfer
    # v_surface = v_ball / (sqrt(energy_eff) * velocity_transfer)
    r_m = (roller_diameter_in / 2 - 0.25) * IN_TO_M  # Effective radius
    v_surface = target_velocity_fps / (np.sqrt(energy_efficiency) * velocity_transfer_factor) * FT_S_TO_IN_S * IN_TO_M
    omega_roller = v_surface / r_m
    roller_rpm = omega_roller * RAD_S_TO_RPM

    motor_free_rpm = motor.free_speed_rpm

    # Optimal: motor at 75% of free speed for good torque headroom
    optimal_motor_rpm = motor_free_rpm * 0.75
    optimal_ratio = optimal_motor_rpm / roller_rpm

    # Min ratio (motor at 95% - barely achievable)
    min_ratio = (motor_free_rpm * 0.95) / roller_rpm

    # Max ratio for decent torque (motor at 50%)
    max_ratio = (motor_free_rpm * 0.50) / roller_rpm

    return {
        'target_roller_rpm': roller_rpm,
        'motor_free_rpm': motor_free_rpm,
        'min_ratio': min_ratio,
        'optimal_ratio': optimal_ratio,
        'max_ratio': max_ratio,
    }


def calculate_flywheel_for_recovery(target_velocity_fps: float,
                                     num_balls: int,
                                     target_recovery_ms: float,
                                     motor_type: str = 'KRAKENX60',
                                     num_motors: int = 2,
                                     roller_diameter_in: float = 4.0) -> Dict:
    """
    Calculate flywheel inertia needed to achieve target recovery time.

    This is an iterative calculation since recovery time depends on
    both the inertia and the RPM drop (which depends on inertia).
    """
    motor = MOTORS[motor_type]
    target_recovery_s = target_recovery_ms / 1000

    # Start with a guess
    best_inertia = None
    best_error = float('inf')

    for added_inertia in np.linspace(0, 50, 100):  # 0 to 50 lb-in²
        shooter = WideRollerShooter(
            motor_type=motor_type,
            num_motors=num_motors,
            roller_diameter_in=roller_diameter_in,
            roller_width_game_pieces=num_balls,
            added_flywheel_inertia_lb_in2=added_inertia,
        )

        result = shooter.simulate_burst_cycle(target_velocity_fps, num_balls)

        if result['feasible']:
            recovery = result['recovery_time_s']
            error = abs(recovery - target_recovery_s)

            if error < best_error:
                best_error = error
                best_inertia = added_inertia
                best_result = result

    if best_inertia is None:
        return {'feasible': False, 'reason': 'Could not find solution'}

    return {
        'feasible': True,
        'added_flywheel_inertia_lb_in2': best_inertia,
        'actual_recovery_ms': best_result['recovery_time_s'] * 1000,
        'rpm_drop_pct': best_result['rpm_drop_pct'],
        'target_rpm': best_result['target_rpm'],
    }


def compare_configurations(target_velocity_fps: float = 30.0,
                           num_balls: int = 1) -> None:
    """Compare different shooter configurations."""
    print("\n" + "="*95)
    print(f"  SHOOTER CONFIGURATION COMPARISON")
    print(f"  Target: {target_velocity_fps} ft/s | {num_balls} ball(s) per burst")
    print("="*95)

    configs = [
        ('NEO', 2, 1.0, 4.0, 0),
        ('FALCON500', 2, 1.5, 4.0, 0),
        ('KRAKENX60', 2, 1.5, 4.0, 0),
        ('KRAKENX60', 2, 1.5, 4.0, 10),
        ('KRAKENX60', 2, 2.0, 6.0, 0),
    ]

    print(f"{'Motor':^12}|{'#M':^4}|{'Ratio':^6}|{'Wheel':^6}|{'+Fly':^6}|"
          f"{'RPM':^6}|{'Spinup':^8}|{'Drop%':^6}|{'Recov':^8}|{'Rate':^8}")
    print("-"*95)

    for motor, n_mot, ratio, wheel_d, added_fly in configs:
        shooter = WideRollerShooter(
            motor_type=motor,
            num_motors=n_mot,
            gear_ratio=ratio,
            roller_diameter_in=wheel_d,
            roller_width_game_pieces=num_balls,
            added_flywheel_inertia_lb_in2=added_fly,
        )

        result = shooter.simulate_burst_cycle(target_velocity_fps)

        if result['feasible']:
            print(f"{motor:^12}|{n_mot:^4}|{ratio:^6.1f}|{wheel_d:^6.1f}|{added_fly:^6.0f}|"
                  f"{result['target_rpm']:^6.0f}|{result['spinup_time_s']*1000:^7.0f}ms|"
                  f"{result['rpm_drop_pct']:^6.1f}|{result['recovery_time_s']*1000:^7.0f}ms|"
                  f"{result['balls_per_second']:^7.1f}/s")
        else:
            print(f"{motor:^12}|{n_mot:^4}|{ratio:^6.1f}| NOT FEASIBLE")

    print("="*95)


# =============================================================================
# VISUALIZATION
# =============================================================================

def plot_shooter_performance(shooter: WideRollerShooter,
                             target_velocity_fps: float = 30.0,
                             save_path: str = None):
    """Visualize shooter performance characteristics."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    result = shooter.simulate_burst_cycle(target_velocity_fps)

    if not result['feasible']:
        fig.text(0.5, 0.5, f"NOT FEASIBLE\n{result['reason']}",
                ha='center', va='center', fontsize=16, color='red')
        return fig

    target_rpm = result['target_rpm']

    # Plot 1: Spinup curve
    ax1 = axes[0, 0]
    times = []
    rpms = []
    rpm = 0
    t = 0
    dt = 0.001

    while rpm < target_rpm * 0.99 and t < 2:
        times.append(t * 1000)
        rpms.append(rpm)
        omega = rpm * RPM_TO_RAD_S
        torque = shooter.torque_at_roller(rpm)
        alpha = torque / shooter.total_inertia_kg_m2
        omega += alpha * dt
        rpm = omega * RAD_S_TO_RPM
        t += dt

    ax1.plot(times, rpms, 'b-', linewidth=2)
    ax1.axhline(y=target_rpm, color='g', linestyle='--', label=f'Target: {target_rpm:.0f} RPM')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('Roller RPM')
    ax1.set_title('Spinup from Rest')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Recovery after burst
    ax2 = axes[0, 1]
    rpm_after = result['rpm_after_burst']

    times = []
    rpms = []
    rpm = rpm_after
    t = 0

    while rpm < target_rpm * 0.99 and t < 0.5:
        times.append(t * 1000)
        rpms.append(rpm)
        omega = rpm * RPM_TO_RAD_S
        torque = shooter.torque_at_roller(rpm)
        alpha = torque / shooter.total_inertia_kg_m2
        omega += alpha * dt
        rpm = min(omega * RAD_S_TO_RPM, shooter.max_roller_rpm())
        t += dt

    ax2.plot(times, rpms, 'b-', linewidth=2)
    ax2.axhline(y=target_rpm, color='g', linestyle='--', label=f'Target: {target_rpm:.0f} RPM')
    ax2.axhline(y=rpm_after, color='r', linestyle=':', label=f'After burst: {rpm_after:.0f} RPM')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Roller RPM')
    ax2.set_title(f'Recovery After {result["num_balls"]}-Ball Burst')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot 3: Rate vs width
    ax3 = axes[1, 0]
    widths = [1, 2, 3, 4]
    recovery_times = []
    rates = []

    for w in widths:
        test_shooter = WideRollerShooter(
            motor_type=shooter.motor_type,
            num_motors=shooter.num_motors,
            gear_ratio=shooter.gear_ratio,
            roller_diameter_in=shooter.roller_diameter_in,
            roller_width_game_pieces=w,
            added_flywheel_inertia_lb_in2=shooter.added_flywheel_inertia_lb_in2,
        )
        r = test_shooter.simulate_burst_cycle(target_velocity_fps)
        if r['feasible']:
            recovery_times.append(r['recovery_time_s'] * 1000)
            rates.append(r['balls_per_second'])
        else:
            recovery_times.append(np.nan)
            rates.append(np.nan)

    ax3.bar([str(w) for w in widths], rates, color='steelblue', alpha=0.7)
    ax3.set_xlabel('Roller Width (game pieces)')
    ax3.set_ylabel('Balls per Second')
    ax3.set_title('Rate of Fire vs Roller Width')
    ax3.grid(True, alpha=0.3, axis='y')

    # Plot 4: Info text
    ax4 = axes[1, 1]
    ax4.axis('off')

    info = f"""
    SHOOTER SYSTEM SUMMARY
    ══════════════════════════════════════════

    Configuration:
      Motor: {shooter.motor.name} x{shooter.num_motors}
      Gear ratio: {shooter.gear_ratio:.2f}:1
      Roller: {shooter.roller_diameter_in:.1f}" x {shooter.roller_width_game_pieces} wide
      Inertia: {shooter.total_inertia_lb_in2:.2f} lb-in²

    Performance @ {target_velocity_fps} ft/s:
      Target RPM: {target_rpm:.0f}
      Spinup: {result['spinup_time_s']*1000:.0f} ms
      RPM drop per burst: {result['rpm_drop_pct']:.1f}%
      Recovery time: {result['recovery_time_s']*1000:.0f} ms

    Rate of Fire:
      {result['balls_per_second']:.1f} balls/sec
      {result['bursts_per_second']:.1f} bursts/sec
    """

    ax4.text(0.05, 0.95, info, transform=ax4.transAxes, fontsize=11,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.suptitle(f'Wide Roller Shooter Analysis - {target_velocity_fps} ft/s Target',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


def plot_integrated_analysis(shooter: WideRollerShooter,
                             release_height_in: float = 18.0,
                             save_path: str = None):
    """Plot integrated trajectory + shooter analysis."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    # Get shooter performance
    target_v = 30.0
    result = shooter.simulate_burst_cycle(target_v)

    if not result['feasible']:
        fig.text(0.5, 0.5, f"NOT FEASIBLE", ha='center', va='center', fontsize=16, color='red')
        return fig

    ball_v = result['exit_velocity_fps']

    # Plot 1: Trajectories
    ax1 = axes[0]
    distances_ft = [3, 5, 7, 10, 12, 15, 18]
    colors = plt.cm.viridis(np.linspace(0.15, 0.9, len(distances_ft)))

    for d_ft, color in zip(distances_ft, colors):
        d_in = d_ft * 12
        traj = find_lob_solution(d_in, ball_v, release_height_in)

        if traj:
            x, y = calculate_trajectory_points(d_in, traj['angle_deg'], ball_v, release_height_in)
            ax1.plot(x, y, color=color, linewidth=2,
                    label=f"{d_ft} ft: {traj['angle_deg']:.0f}° → {traj['entry_angle_deg']:.0f}° entry")

    # Funnel target
    ax1.plot([0, -10], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN-25], 'k-', linewidth=4)
    ax1.plot([0, 10], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN-25], 'k-', linewidth=4)
    ax1.plot([-25, 25], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN], 'g-', linewidth=5)
    ax1.fill_between([-25, 25], TARGET_HEIGHT_IN-3, TARGET_HEIGHT_IN+3, color='green', alpha=0.3)
    ax1.axhline(y=0, color='saddlebrown', linewidth=4)

    ax1.set_xlim(-40, 230)
    ax1.set_ylim(-10, 220)
    ax1.set_xlabel('Distance from target (inches)')
    ax1.set_ylabel('Height (inches)')
    ax1.set_title(f'Lob Trajectories @ {ball_v:.1f} ft/s Ball Velocity')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Plot 2: Performance summary
    ax2 = axes[1]
    ax2.axis('off')

    info = f"""
    INTEGRATED SHOOTER + TRAJECTORY ANALYSIS
    ════════════════════════════════════════════════════════

    SHOOTER CONFIGURATION
    ─────────────────────
    Motor: {shooter.motor.name} x{shooter.num_motors}
    Gear ratio: {shooter.gear_ratio:.2f}:1
    Roller: {shooter.roller_diameter_in:.1f}" dia × {shooter.roller_width_game_pieces} balls wide
    Total inertia: {shooter.total_inertia_lb_in2:.2f} lb-in²

    SHOOTER PERFORMANCE
    ─────────────────────
    Ball exit velocity: {ball_v:.1f} ft/s
    Operating RPM: {result['target_rpm']:.0f}
    Spinup time: {result['spinup_time_s']*1000:.0f} ms
    Recovery after burst: {result['recovery_time_s']*1000:.0f} ms
    Rate of fire: {result['balls_per_second']:.1f} balls/sec

    TRAJECTORY COVERAGE
    ─────────────────────
    Release height: {release_height_in}" from floor
    Target height: {TARGET_HEIGHT_IN}" (funnel facing UP)

    Distance coverage: 3 ft to 18+ ft
    All shots descend into funnel (lob trajectories)

    ════════════════════════════════════════════════════════
    """

    ax2.text(0.05, 0.95, info, transform=ax2.transAxes, fontsize=11,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.8))

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    print("\n" + "="*75)
    print("  FRC WIDE ROLLER SHOOTER SIMULATOR")
    print("  Team 2491 - 2026 REBUILT")
    print("="*75)

    # Example: 1-ball-wide shooter
    shooter = WideRollerShooter(
        motor_type='KRAKENX60',
        num_motors=2,
        gear_ratio=1.5,
        roller_diameter_in=4.0,
        roller_width_game_pieces=1,
        added_flywheel_inertia_lb_in2=0,
    )

    # Analyze
    analysis = IntegratedShooterAnalysis(shooter, release_height_in=18.0)
    analysis.print_analysis(target_velocity_fps=30.0)

    # Compare configurations
    print("\n\nComparing different configurations...")
    compare_configurations(target_velocity_fps=30.0, num_balls=1)

    print("\n\n4-ball wide roller:")
    compare_configurations(target_velocity_fps=30.0, num_balls=4)

    print("\n\nTo generate plots:")
    print("  plot_shooter_performance(shooter, save_path='shooter_perf.png')")
    print("  plot_integrated_analysis(shooter, save_path='integrated.png')")
