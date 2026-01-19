# -*- coding: utf-8 -*-
"""Refactor remaining notebook cells to fix duplicate output issue."""
import json

with open('shooter_analysis.ipynb', encoding='utf-8') as f:
    nb = json.load(f)

# Cell 30: analyze_shooter_interactive
cell30 = '''def analyze_shooter_interactive(motor_type='KRAKENX60', num_motors=2, gear_ratio=1.5,
                                  roller_diameter=4.0, roller_width=1,
                                  added_flywheel=0, target_velocity=30,
                                  release_height=18):
    """Interactive shooter analysis with trajectory integration. Returns figure."""

    shooter = WideRollerShooter(
        motor_type=motor_type, num_motors=num_motors, gear_ratio=gear_ratio,
        roller_diameter_in=roller_diameter, roller_width_game_pieces=roller_width,
        added_flywheel_inertia_lb_in2=added_flywheel, energy_transfer_efficiency=0.80,
    )

    result = shooter.simulate_burst_cycle(target_velocity, roller_width)
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    if not result['feasible']:
        fig.text(0.5, 0.5, f"NOT FEASIBLE\\n{result['reason']}",
                ha='center', va='center', fontsize=20, color='red')
        plt.tight_layout()
        return fig

    # Plot 1: Trajectories
    ax1 = axes[0, 0]
    ball_v = result['exit_velocity_fps']
    distances_ft = [3, 5, 7, 10, 12, 15, 18]
    colors = plt.cm.viridis(np.linspace(0.15, 0.9, len(distances_ft)))

    for d_ft, color in zip(distances_ft, colors):
        d_in = d_ft * 12
        traj = find_lob_solution(d_in, ball_v, release_height)
        if traj:
            x, y = calculate_trajectory_points(d_in, traj['angle_deg'], ball_v, release_height)
            ax1.plot(x, y, color=color, linewidth=2,
                    label=f"{d_ft}ft: {traj['angle_deg']:.0f}deg")

    ax1.plot([0, -10], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN-25], 'k-', linewidth=3)
    ax1.plot([0, 10], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN-25], 'k-', linewidth=3)
    ax1.plot([-25, 25], [TARGET_HEIGHT_IN, TARGET_HEIGHT_IN], 'g-', linewidth=4)
    ax1.axhline(y=0, color='saddlebrown', linewidth=3)
    ax1.set_xlim(-40, 230)
    ax1.set_ylim(-10, 220)
    ax1.set_xlabel('Distance from target (inches)')
    ax1.set_ylabel('Height (inches)')
    ax1.set_title(f'Lob Trajectories @ {ball_v:.1f} ft/s')
    ax1.legend(loc='upper right', fontsize=7)
    ax1.grid(True, alpha=0.3)

    # Plot 2: Recovery curve
    ax2 = axes[0, 1]
    target_rpm = result['target_rpm']
    rpm_after = result['rpm_after_burst']
    times, rpms = [], []
    rpm, t, dt = rpm_after, 0, 0.0005
    while rpm < target_rpm * 0.99 and t < 0.2:
        times.append(t * 1000)
        rpms.append(rpm)
        omega = rpm * (2 * np.pi / 60)
        torque = shooter.torque_at_roller(rpm)
        alpha = torque / shooter.total_inertia_kg_m2
        omega += alpha * dt
        rpm = min(omega * 60 / (2 * np.pi), shooter.max_roller_rpm())
        t += dt
    ax2.plot(times, rpms, 'b-', linewidth=2.5)
    ax2.axhline(y=target_rpm, color='g', linestyle='--', label=f'Target: {target_rpm:.0f}')
    ax2.axhline(y=rpm_after, color='r', linestyle=':', label=f'After: {rpm_after:.0f}')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Roller RPM')
    ax2.set_title(f'Recovery After {roller_width}-Ball Burst')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # Plot 3: Rate vs width
    ax3 = axes[1, 0]
    widths = [1, 2, 3, 4]
    rates = []
    for w in widths:
        ts = WideRollerShooter(motor_type=motor_type, num_motors=num_motors, gear_ratio=gear_ratio,
            roller_diameter_in=roller_diameter, roller_width_game_pieces=w, added_flywheel_inertia_lb_in2=added_flywheel)
        r = ts.simulate_burst_cycle(target_velocity, w)
        rates.append(r['balls_per_second'] if r['feasible'] else 0)
    ax3.bar([str(w) for w in widths], rates, color='steelblue', alpha=0.7)
    ax3.set_xlabel('Roller Width (balls)')
    ax3.set_ylabel('Rate of Fire (balls/sec)')
    ax3.set_title('Rate vs Width')
    ax3.grid(True, alpha=0.3, axis='y')

    # Plot 4: Summary
    ax4 = axes[1, 1]
    ax4.axis('off')
    summary = f"""CONFIGURATION
Motor: {MOTORS[motor_type].name} x {num_motors}
Gear: {gear_ratio:.2f}:1
Roller: {roller_diameter:.1f}" x {roller_width} wide
Flywheel: {shooter.total_inertia_lb_in2:.2f} lb-in2

PERFORMANCE @ {target_velocity} fps
RPM: {target_rpm:.0f} (max {result['max_rpm']:.0f})
Spinup: {result['spinup_time_s']*1000:.0f} ms
Recovery: {result['recovery_time_s']*1000:.0f} ms
Drop: {result['rpm_drop_pct']:.1f}%

RATE: {result['balls_per_second']:.1f} balls/sec"""
    ax4.text(0.05, 0.95, summary, transform=ax4.transAxes, fontsize=11, verticalalignment='top',
             fontfamily='monospace', bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
    plt.tight_layout()
    return fig

# Widgets
w_sh_motor = Dropdown(options=['NEO', 'FALCON500', 'KRAKENX60'], value='KRAKENX60', description='Motor')
w_sh_nmotors = IntSlider(min=1, max=4, value=2, description='# Motors', continuous_update=False)
w_sh_gear = FloatSlider(min=0.5, max=3.0, step=0.1, value=1.5, description='Gear Ratio', continuous_update=False)
w_sh_roller = FloatSlider(min=3.0, max=6.0, step=0.5, value=4.0, description='Roller (in)', continuous_update=False)
w_sh_width = IntSlider(min=1, max=4, value=1, description='Width', continuous_update=False)
w_sh_fly = FloatSlider(min=0, max=30, step=2, value=0, description='+Flywheel', continuous_update=False)
w_sh_vel = IntSlider(min=25, max=40, value=30, description='Target fps', continuous_update=False)
w_sh_rel = IntSlider(min=14, max=22, value=18, description='Release in', continuous_update=False)
out_sh = widgets.Output()

def update_sh(change=None):
    with out_sh:
        clear_output(wait=True)
        fig = analyze_shooter_interactive(w_sh_motor.value, w_sh_nmotors.value, w_sh_gear.value,
            w_sh_roller.value, w_sh_width.value, w_sh_fly.value, w_sh_vel.value, w_sh_rel.value)
        display(fig)
        plt.close(fig)

widgets.interactive_output(update_sh, {'m': w_sh_motor, 'n': w_sh_nmotors, 'g': w_sh_gear,
    'r': w_sh_roller, 'w': w_sh_width, 'f': w_sh_fly, 'v': w_sh_vel, 'h': w_sh_rel})
display(VBox([HBox([w_sh_motor, w_sh_nmotors, w_sh_gear, w_sh_roller]),
              HBox([w_sh_width, w_sh_fly, w_sh_vel, w_sh_rel]), out_sh]))
update_sh()'''

nb['cells'][30]['source'] = cell30
print("Refactored cell 30")

# Cell 32: compare_shooters_interactive
cell32 = '''def compare_shooters_interactive(target_velocity=30, roller_width=1):
    """Compare all motor configurations. Returns figure."""
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    motors = ['NEO', 'FALCON500', 'KRAKENX60']
    colors = ['#2ecc71', '#3498db', '#e74c3c']
    spinup_times, recovery_times, rates = [], [], []

    for motor in motors:
        shooter = WideRollerShooter(motor_type=motor, num_motors=2, gear_ratio=1.5,
            roller_diameter_in=4.0, roller_width_game_pieces=roller_width)
        r = shooter.simulate_burst_cycle(target_velocity, roller_width)
        if r['feasible']:
            spinup_times.append(r['spinup_time_s'] * 1000)
            recovery_times.append(r['recovery_time_s'] * 1000)
            rates.append(r['balls_per_second'])
        else:
            spinup_times.append(0)
            recovery_times.append(0)
            rates.append(0)

    axes[0].bar(motors, spinup_times, color=colors)
    axes[0].set_ylabel('Time (ms)')
    axes[0].set_title('Spinup Time')
    axes[0].grid(True, alpha=0.3, axis='y')

    axes[1].bar(motors, recovery_times, color=colors)
    axes[1].set_ylabel('Time (ms)')
    axes[1].set_title('Recovery Time')
    axes[1].grid(True, alpha=0.3, axis='y')

    axes[2].bar(motors, rates, color=colors)
    axes[2].set_ylabel('Balls/second')
    axes[2].set_title('Rate of Fire')
    axes[2].grid(True, alpha=0.3, axis='y')

    plt.suptitle(f'Motor Comparison: {roller_width}-Wide @ {target_velocity} ft/s', fontsize=12, fontweight='bold')
    plt.tight_layout()
    return fig

# Widgets
w_cmp_vel = IntSlider(min=25, max=40, value=30, description='Target (fps)', continuous_update=False)
w_cmp_width = IntSlider(min=1, max=4, value=1, description='Width (balls)', continuous_update=False)
out_cmp = widgets.Output()

def update_cmp(change=None):
    with out_cmp:
        clear_output(wait=True)
        fig = compare_shooters_interactive(w_cmp_vel.value, w_cmp_width.value)
        display(fig)
        plt.close(fig)

widgets.interactive_output(update_cmp, {'v': w_cmp_vel, 'w': w_cmp_width})
display(VBox([HBox([w_cmp_vel, w_cmp_width]), out_cmp]))
update_cmp()'''

nb['cells'][32]['source'] = cell32
print("Refactored cell 32")

# Cell 34: optimize_gear_ratio
cell34 = '''def optimize_gear_ratio(motor_type='KRAKENX60', num_motors=2, roller_diameter=4.0,
                        roller_width=1, target_velocity=30, headroom_target=20):
    """Find optimal gear ratio for maximum rate of fire. Returns figure."""
    gear_ratios = np.linspace(0.5, 4.0, 50)
    rates, recovery_times, headrooms, feasible = [], [], [], []

    for gr in gear_ratios:
        shooter = WideRollerShooter(motor_type=motor_type, num_motors=num_motors, gear_ratio=gr,
            roller_diameter_in=roller_diameter, roller_width_game_pieces=roller_width)
        r = shooter.simulate_burst_cycle(target_velocity, roller_width)
        if r['feasible']:
            rates.append(r['balls_per_second'])
            recovery_times.append(r['recovery_time_s'] * 1000)
            headrooms.append(r['headroom_pct'])
            feasible.append(True)
        else:
            rates.append(0)
            recovery_times.append(np.nan)
            headrooms.append(0)
            feasible.append(False)

    valid_rates = [(gr, rate) for gr, rate, f in zip(gear_ratios, rates, feasible) if f]
    optimal_gr, optimal_rate = max(valid_rates, key=lambda x: x[1]) if valid_rates else (None, 0)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    axes[0].plot(gear_ratios, rates, 'b-', linewidth=2.5)
    if optimal_gr:
        axes[0].axvline(x=optimal_gr, color='green', linestyle='--', label=f'Optimal: {optimal_gr:.2f}')
        axes[0].plot(optimal_gr, optimal_rate, 'go', markersize=12)
    axes[0].set_xlabel('Gear Ratio')
    axes[0].set_ylabel('Rate (balls/s)')
    axes[0].set_title('Rate vs Gear Ratio')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(gear_ratios, recovery_times, 'r-', linewidth=2.5)
    if optimal_gr: axes[1].axvline(x=optimal_gr, color='green', linestyle='--')
    axes[1].set_xlabel('Gear Ratio')
    axes[1].set_ylabel('Recovery (ms)')
    axes[1].set_title('Recovery Time')
    axes[1].grid(True, alpha=0.3)

    axes[2].fill_between(gear_ratios, 0, headrooms, alpha=0.3, color='blue')
    axes[2].plot(gear_ratios, headrooms, 'b-', linewidth=2.5)
    axes[2].axhline(y=headroom_target, color='orange', linestyle='--', label=f'Target: {headroom_target}%')
    if optimal_gr: axes[2].axvline(x=optimal_gr, color='green', linestyle='--')
    axes[2].set_xlabel('Gear Ratio')
    axes[2].set_ylabel('Headroom (%)')
    axes[2].set_title('RPM Headroom')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.suptitle(f'{motor_type} x {num_motors} | {roller_diameter}" roller | {target_velocity} fps', fontsize=12, fontweight='bold')
    plt.tight_layout()
    return fig

# Widgets
w_opt_motor = Dropdown(options=['NEO', 'FALCON500', 'KRAKENX60'], value='KRAKENX60', description='Motor')
w_opt_nmotors = IntSlider(min=1, max=4, value=2, description='# Motors', continuous_update=False)
w_opt_roller = FloatSlider(min=3.0, max=6.0, step=0.5, value=4.0, description='Roller (in)', continuous_update=False)
w_opt_width = IntSlider(min=1, max=4, value=1, description='Width', continuous_update=False)
w_opt_vel = IntSlider(min=25, max=40, value=30, description='Target fps', continuous_update=False)
w_opt_head = IntSlider(min=10, max=40, value=20, description='Headroom %', continuous_update=False)
out_opt = widgets.Output()

def update_opt(change=None):
    with out_opt:
        clear_output(wait=True)
        fig = optimize_gear_ratio(w_opt_motor.value, w_opt_nmotors.value, w_opt_roller.value,
            w_opt_width.value, w_opt_vel.value, w_opt_head.value)
        display(fig)
        plt.close(fig)

widgets.interactive_output(update_opt, {'m': w_opt_motor, 'n': w_opt_nmotors, 'r': w_opt_roller,
    'w': w_opt_width, 'v': w_opt_vel, 'h': w_opt_head})
display(VBox([HBox([w_opt_motor, w_opt_nmotors, w_opt_roller]),
              HBox([w_opt_width, w_opt_vel, w_opt_head]), out_opt]))
update_opt()'''

nb['cells'][34]['source'] = cell34
print("Refactored cell 34")

# Cell 36: flywheel_sizing_helper
cell36 = '''def flywheel_sizing_helper(motor_type='KRAKENX60', num_motors=2, gear_ratio=1.5,
                           roller_diameter=4.0, roller_width=2, target_velocity=30,
                           max_rpm_drop=5.0, max_recovery_ms=100):
    """Find required flywheel inertia for target performance. Returns figure."""
    flywheel_range = np.linspace(0, 50, 100)
    rpm_drops, recovery_times, rates = [], [], []

    for fw in flywheel_range:
        shooter = WideRollerShooter(motor_type=motor_type, num_motors=num_motors, gear_ratio=gear_ratio,
            roller_diameter_in=roller_diameter, roller_width_game_pieces=roller_width,
            added_flywheel_inertia_lb_in2=fw)
        r = shooter.simulate_burst_cycle(target_velocity, roller_width)
        if r['feasible']:
            rpm_drops.append(r['rpm_drop_pct'])
            recovery_times.append(r['recovery_time_s'] * 1000)
            rates.append(r['balls_per_second'])
        else:
            rpm_drops.append(np.nan)
            recovery_times.append(np.nan)
            rates.append(0)

    min_fw_for_drop = next((fw for fw, drop in zip(flywheel_range, rpm_drops)
                           if not np.isnan(drop) and drop <= max_rpm_drop), None)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    axes[0].plot(flywheel_range, rpm_drops, 'b-', linewidth=2.5)
    axes[0].axhline(y=max_rpm_drop, color='red', linestyle='--', label=f'Target: {max_rpm_drop}%')
    if min_fw_for_drop:
        axes[0].axvline(x=min_fw_for_drop, color='green', linestyle='--', label=f'Min: {min_fw_for_drop:.1f}')
    axes[0].set_xlabel('Added Flywheel (lb-in2)')
    axes[0].set_ylabel('RPM Drop (%)')
    axes[0].set_title(f'RPM Drop per {roller_width}-Ball Burst')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(flywheel_range, recovery_times, 'r-', linewidth=2.5)
    axes[1].axhline(y=max_recovery_ms, color='orange', linestyle='--', label=f'Target: {max_recovery_ms} ms')
    axes[1].set_xlabel('Added Flywheel (lb-in2)')
    axes[1].set_ylabel('Recovery (ms)')
    axes[1].set_title('Recovery Time')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(flywheel_range, rates, 'g-', linewidth=2.5)
    axes[2].set_xlabel('Added Flywheel (lb-in2)')
    axes[2].set_ylabel('Rate (balls/s)')
    axes[2].set_title('Rate of Fire')
    axes[2].grid(True, alpha=0.3)

    plt.suptitle(f'Flywheel Sizing: {motor_type} x {num_motors} | {roller_width}-wide @ {target_velocity} fps',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    return fig

# Widgets
w_fs_motor = Dropdown(options=['NEO', 'FALCON500', 'KRAKENX60'], value='KRAKENX60', description='Motor')
w_fs_nmotors = IntSlider(min=1, max=4, value=2, description='# Motors', continuous_update=False)
w_fs_gear = FloatSlider(min=0.5, max=3.0, step=0.1, value=1.5, description='Gear Ratio', continuous_update=False)
w_fs_roller = FloatSlider(min=3.0, max=6.0, step=0.5, value=4.0, description='Roller (in)', continuous_update=False)
w_fs_width = IntSlider(min=1, max=4, value=2, description='Width', continuous_update=False)
w_fs_vel = IntSlider(min=25, max=40, value=30, description='Target fps', continuous_update=False)
w_fs_drop = FloatSlider(min=1, max=15, step=0.5, value=5.0, description='Max drop %', continuous_update=False)
w_fs_rec = IntSlider(min=50, max=200, step=10, value=100, description='Max rec ms', continuous_update=False)
out_fs = widgets.Output()

def update_fs(change=None):
    with out_fs:
        clear_output(wait=True)
        fig = flywheel_sizing_helper(w_fs_motor.value, w_fs_nmotors.value, w_fs_gear.value,
            w_fs_roller.value, w_fs_width.value, w_fs_vel.value, w_fs_drop.value, w_fs_rec.value)
        display(fig)
        plt.close(fig)

widgets.interactive_output(update_fs, {'m': w_fs_motor, 'n': w_fs_nmotors, 'g': w_fs_gear,
    'r': w_fs_roller, 'w': w_fs_width, 'v': w_fs_vel, 'd': w_fs_drop, 'c': w_fs_rec})
display(VBox([HBox([w_fs_motor, w_fs_nmotors, w_fs_gear, w_fs_roller]),
              HBox([w_fs_width, w_fs_vel, w_fs_drop, w_fs_rec]), out_fs]))
update_fs()'''

nb['cells'][36]['source'] = cell36
print("Refactored cell 36")

# Cell 38: generate_design_summary (text output, no plot)
cell38 = '''def generate_design_summary(motor_type='KRAKENX60', num_motors=2, gear_ratio=1.5,
                            roller_diameter=4.0, roller_width=2, added_flywheel=0,
                            target_velocity=30, release_height=18):
    """Generate a complete design summary for the chosen configuration."""
    shooter = WideRollerShooter(motor_type=motor_type, num_motors=num_motors, gear_ratio=gear_ratio,
        roller_diameter_in=roller_diameter, roller_width_game_pieces=roller_width,
        added_flywheel_inertia_lb_in2=added_flywheel, energy_transfer_efficiency=0.80)

    result = shooter.simulate_burst_cycle(target_velocity, roller_width)

    distances_ft = [3, 5, 7, 10, 12, 15, 18]
    trajectory_coverage = []
    if result['feasible']:
        for d_ft in distances_ft:
            sol = find_lob_solution(d_ft * 12, result['exit_velocity_fps'], release_height)
            if sol:
                trajectory_coverage.append((d_ft, sol['angle_deg'], sol['entry_angle_deg']))

    motor = MOTORS[motor_type]

    print("=" + "="*70 + "=")
    print(" SHOOTER DESIGN SUMMARY ".center(72))
    print("=" + "="*70 + "=")

    print("\\n MECHANICAL CONFIGURATION")
    print("-"*72)
    print(f"  Motor: {motor.name} x {num_motors}")
    print(f"  Gear ratio: {gear_ratio:.2f}:1 (motor:roller)")
    print(f"  Roller: {roller_diameter:.1f}\\" dia x {roller_width} balls wide")
    print(f"  Added flywheel: {added_flywheel:.1f} lb-in2")
    print(f"  Total inertia: {shooter.total_inertia_lb_in2:.2f} lb-in2")

    print(f"\\n PERFORMANCE @ {target_velocity} ft/s")
    print("-"*72)

    if result['feasible']:
        print(f"  Required RPM: {result['target_rpm']:.0f}")
        print(f"  Max RPM: {result['max_rpm']:.0f} ({result['headroom_pct']:.0f}% headroom)")
        print(f"  Exit velocity: {result['exit_velocity_fps']:.1f} ft/s")
        print(f"  Spinup: {result['spinup_time_s']*1000:.0f} ms")
        print(f"  RPM drop: {result['rpm_drop_pct']:.1f}%")
        print(f"  Recovery: {result['recovery_time_s']*1000:.0f} ms")
        print(f"\\n  >>> RATE OF FIRE: {result['balls_per_second']:.1f} balls/second <<<")
    else:
        print(f"  NOT FEASIBLE: {result['reason']}")

    print(f"\\n TRAJECTORY COVERAGE (release {release_height}\\")")
    print("-"*72)
    if trajectory_coverage:
        print(f"  {'Distance':>8} | {'Launch':>10} | {'Entry':>10} | {'Quality'}")
        print("  " + "-"*50)
        for d_ft, launch, entry in trajectory_coverage:
            quality = "IDEAL" if entry >= 60 else "GOOD" if entry >= 45 else "RISK"
            print(f"  {d_ft:>6} ft | {launch:>8.0f} deg | {entry:>8.0f} deg | {quality}")
    else:
        print("  No valid trajectories")
    print("=" + "="*70 + "=")

# Widgets
w_ds_motor = Dropdown(options=['NEO', 'FALCON500', 'KRAKENX60'], value='KRAKENX60', description='Motor')
w_ds_nmotors = IntSlider(min=1, max=4, value=2, description='# Motors', continuous_update=False)
w_ds_gear = FloatSlider(min=0.5, max=3.0, step=0.1, value=1.5, description='Gear Ratio', continuous_update=False)
w_ds_roller = FloatSlider(min=3.0, max=6.0, step=0.5, value=4.0, description='Roller (in)', continuous_update=False)
w_ds_width = IntSlider(min=1, max=4, value=2, description='Width', continuous_update=False)
w_ds_fly = FloatSlider(min=0, max=30, step=2, value=0, description='+Flywheel', continuous_update=False)
w_ds_vel = IntSlider(min=25, max=40, value=30, description='Target fps', continuous_update=False)
w_ds_rel = IntSlider(min=14, max=22, value=18, description='Release in', continuous_update=False)
out_ds = widgets.Output()

def update_ds(change=None):
    with out_ds:
        clear_output(wait=True)
        generate_design_summary(w_ds_motor.value, w_ds_nmotors.value, w_ds_gear.value,
            w_ds_roller.value, w_ds_width.value, w_ds_fly.value, w_ds_vel.value, w_ds_rel.value)

widgets.interactive_output(update_ds, {'m': w_ds_motor, 'n': w_ds_nmotors, 'g': w_ds_gear,
    'r': w_ds_roller, 'w': w_ds_width, 'f': w_ds_fly, 'v': w_ds_vel, 'h': w_ds_rel})
display(VBox([HBox([w_ds_motor, w_ds_nmotors, w_ds_gear, w_ds_roller]),
              HBox([w_ds_width, w_ds_fly, w_ds_vel, w_ds_rel]), out_ds]))
update_ds()'''

nb['cells'][38]['source'] = cell38
print("Refactored cell 38")

# Save
with open('shooter_analysis.ipynb', 'w', encoding='utf-8') as f:
    json.dump(nb, f, indent=1, ensure_ascii=False)

print("\nAll Section 12 cells refactored!")
