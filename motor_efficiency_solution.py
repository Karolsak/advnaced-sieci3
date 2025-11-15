"""
Solution to Example 29.42: Shunt Motor Efficiency Analysis
"""

import numpy as np
import matplotlib.pyplot as plt

def solve_shunt_motor_efficiency():
    """
    Solve the shunt motor efficiency problem

    Given:
    - Rated power output: 7.46 kW
    - Voltage: 250 V
    - No-load line current: 5 A
    - Armature resistance: 0.5 Ω
    - Field resistance: 250 Ω
    """

    print("=" * 70)
    print("SHUNT MOTOR EFFICIENCY ANALYSIS - Example 29.42")
    print("=" * 70)

    # Given parameters
    P_rated = 7460  # W
    V = 250  # V
    I_L0 = 5  # A (no-load line current)
    Ra = 0.5  # Ω
    Rsh = 250  # Ω

    print("\nGiven Parameters:")
    print(f"  Rated Output Power: {P_rated/1000} kW")
    print(f"  Supply Voltage: {V} V")
    print(f"  No-load Line Current: {I_L0} A")
    print(f"  Armature Resistance: {Ra} Ω")
    print(f"  Field Resistance: {Rsh} Ω")

    # Calculate field current (constant)
    I_sh = V / Rsh
    print(f"\n1. Field Current (constant): I_sh = V/Rsh = {V}/{Rsh} = {I_sh:.2f} A")

    # No-load armature current
    I_a0 = I_L0 - I_sh
    print(f"2. No-load Armature Current: I_a0 = I_L0 - I_sh = {I_L0} - {I_sh} = {I_a0:.2f} A")

    # No-load losses (constant losses)
    P_input_no_load = V * I_L0
    P_cu_field = V * I_sh  # Field copper loss
    P_cu_armature_no_load = I_a0**2 * Ra
    P_constant = P_input_no_load - P_cu_field - P_cu_armature_no_load

    print(f"\n3. No-load Analysis:")
    print(f"   Input Power at no-load: {P_input_no_load} W")
    print(f"   Field Copper Loss: {P_cu_field} W")
    print(f"   Armature Copper Loss (no-load): {P_cu_armature_no_load} W")
    print(f"   Constant Losses (rotational + iron): {P_constant} W")

    # Full load analysis
    # Output = (V - Ia*Ra) * Ia
    # Solving: P_rated = V*Ia - Ra*Ia^2
    # Ra*Ia^2 - V*Ia + P_rated = 0

    a = Ra
    b = -V
    c = P_rated

    discriminant = b**2 - 4*a*c
    I_a1 = (-b - np.sqrt(discriminant)) / (2*a)
    I_a2 = (-b + np.sqrt(discriminant)) / (2*a)

    # Take the practical value (smaller current)
    I_a_full = I_a1

    print(f"\n4. Full Load Analysis:")
    print(f"   Solving: {Ra}*Ia² - {V}*Ia + {P_rated} = 0")
    print(f"   Solutions: Ia = {I_a1:.2f} A or {I_a2:.2f} A")
    print(f"   Practical value: Ia = {I_a_full:.2f} A")

    # Full load line current
    I_L_full = I_a_full + I_sh
    print(f"   Line Current at full load: I_L = Ia + I_sh = {I_a_full:.2f} + {I_sh:.2f} = {I_L_full:.2f} A")

    # Back EMF at full load
    Eb_full = V - I_a_full * Ra
    print(f"   Back EMF: Eb = V - Ia*Ra = {V} - {I_a_full:.2f}*{Ra} = {Eb_full:.2f} V")

    # Verify output power
    P_output_verify = Eb_full * I_a_full
    print(f"   Output Power (verify): Eb*Ia = {P_output_verify:.2f} W")

    # Calculate losses at full load
    P_cu_armature_full = I_a_full**2 * Ra
    P_cu_field_full = P_cu_field  # Same as before
    P_total_losses = P_constant + P_cu_armature_full + P_cu_field_full

    print(f"\n5. Losses at Full Load:")
    print(f"   Armature Copper Loss: Ia²*Ra = {I_a_full:.2f}²*{Ra} = {P_cu_armature_full:.2f} W")
    print(f"   Field Copper Loss: {P_cu_field_full:.2f} W")
    print(f"   Constant Losses: {P_constant:.2f} W")
    print(f"   Total Losses: {P_total_losses:.2f} W")

    # Input power and efficiency
    P_input_full = P_rated + P_total_losses
    efficiency_full = (P_rated / P_input_full) * 100

    print(f"\n6. Efficiency at Full Load:")
    print(f"   Input Power: {P_input_full:.2f} W")
    print(f"   Output Power: {P_rated} W")
    print(f"   Efficiency: η = (Output/Input) × 100 = {efficiency_full:.2f}%")

    # Maximum efficiency condition
    # Max efficiency when variable losses = constant losses
    # Variable losses = Ia²*Ra
    # Constant losses = P_constant + P_cu_field

    P_const_total = P_constant + P_cu_field
    I_a_max_eff = np.sqrt(P_const_total / Ra)

    print(f"\n7. Maximum Efficiency Analysis:")
    print(f"   Condition: Variable Losses = Constant Losses")
    print(f"   Constant Losses Total: {P_const_total:.2f} W")
    print(f"   Ia²*Ra = {P_const_total:.2f}")
    print(f"   Armature Current for max efficiency: Ia = {I_a_max_eff:.2f} A")

    # Output power at maximum efficiency
    Eb_max_eff = V - I_a_max_eff * Ra
    P_output_max_eff = Eb_max_eff * I_a_max_eff

    print(f"   Back EMF at max efficiency: {Eb_max_eff:.2f} V")
    print(f"   Output Power at max efficiency: {P_output_max_eff:.2f} W = {P_output_max_eff/1000:.2f} kW")

    # Check if this output is achievable
    if P_output_max_eff > P_rated:
        overload_percent = ((P_output_max_eff - P_rated) / P_rated) * 100
        print(f"\n   ⚠ This output ({P_output_max_eff/1000:.2f} kW) exceeds rated output ({P_rated/1000} kW)")
        print(f"   Overload: {overload_percent:.1f}%")
        print(f"   It would require OVERLOADING the machine - NOT recommended for continuous operation")
    else:
        print(f"\n   ✓ This output is within the rated capacity of the machine")

    # Calculate efficiency at maximum efficiency point
    P_losses_max_eff = 2 * P_const_total  # Variable = Constant at max efficiency
    P_input_max_eff = P_output_max_eff + P_losses_max_eff
    efficiency_max = (P_output_max_eff / P_input_max_eff) * 100
    print(f"   Maximum Efficiency: {efficiency_max:.2f}%")

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Efficiency at full load ({P_rated/1000} kW): {efficiency_full:.2f}%")
    print(f"Maximum efficiency occurs at: {P_output_max_eff/1000:.2f} kW")
    print(f"Maximum efficiency value: {efficiency_max:.2f}%")
    print(f"Is max efficiency output achievable? {'NO - Requires overloading' if P_output_max_eff > P_rated else 'YES'}")
    print("=" * 70)

    # Plot efficiency vs output power
    plot_efficiency_curve(V, Ra, Rsh, P_constant, P_rated, P_output_max_eff, I_a_full, I_a_max_eff)

    return {
        'efficiency_full_load': efficiency_full,
        'output_max_efficiency': P_output_max_eff,
        'max_efficiency': efficiency_max,
        'can_achieve': P_output_max_eff <= P_rated
    }

def plot_efficiency_curve(V, Ra, Rsh, P_constant, P_rated, P_max_eff, I_a_full, I_a_max_eff):
    """Plot efficiency vs output power curve"""

    # Range of armature currents
    I_a = np.linspace(5, 80, 200)
    I_sh = V / Rsh

    # Calculate output power and efficiency for each current
    Eb = V - I_a * Ra
    P_output = Eb * I_a

    P_cu_armature = I_a**2 * Ra
    P_cu_field = V * I_sh
    P_total_losses = P_constant + P_cu_armature + P_cu_field
    P_input = P_output + P_total_losses

    efficiency = (P_output / P_input) * 100

    # Create figure with subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    # Plot 1: Efficiency vs Output Power
    ax1.plot(P_output/1000, efficiency, 'b-', linewidth=2, label='Efficiency Curve')
    ax1.axvline(P_rated/1000, color='g', linestyle='--', linewidth=1.5, label=f'Rated Output ({P_rated/1000} kW)')
    ax1.axvline(P_max_eff/1000, color='r', linestyle='--', linewidth=1.5, label=f'Max Eff. Point ({P_max_eff/1000:.2f} kW)')

    # Mark specific points
    idx_full = np.argmin(np.abs(I_a - I_a_full))
    idx_max_eff = np.argmin(np.abs(I_a - I_a_max_eff))

    ax1.plot(P_output[idx_full]/1000, efficiency[idx_full], 'go', markersize=10, label=f'Full Load ({efficiency[idx_full]:.2f}%)')
    ax1.plot(P_output[idx_max_eff]/1000, efficiency[idx_max_eff], 'ro', markersize=10, label=f'Max Eff. ({efficiency[idx_max_eff]:.2f}%)')

    ax1.set_xlabel('Output Power (kW)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Efficiency (%)', fontsize=12, fontweight='bold')
    ax1.set_title('Shunt Motor Efficiency vs Output Power', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best')
    ax1.set_xlim(0, max(P_output)/1000 * 1.1)
    ax1.set_ylim(0, 100)

    # Plot 2: Losses breakdown
    ax2.plot(P_output/1000, P_cu_armature, 'r-', linewidth=2, label='Armature Copper Loss')
    ax2.plot(P_output/1000, np.ones_like(P_output)*P_cu_field, 'b-', linewidth=2, label='Field Copper Loss')
    ax2.plot(P_output/1000, np.ones_like(P_output)*P_constant, 'g-', linewidth=2, label='Constant Losses')
    ax2.plot(P_output/1000, P_total_losses, 'k-', linewidth=2, label='Total Losses')

    ax2.axvline(P_rated/1000, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax2.axvline(P_max_eff/1000, color='orange', linestyle='--', linewidth=1, alpha=0.5)

    ax2.set_xlabel('Output Power (kW)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Losses (W)', fontsize=12, fontweight='bold')
    ax2.set_title('Power Losses vs Output Power', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best')
    ax2.set_xlim(0, max(P_output)/1000 * 1.1)

    plt.tight_layout()
    plt.savefig('/home/user/advnaced-sieci3/motor_efficiency_analysis.png', dpi=300, bbox_inches='tight')
    print(f"\nEfficiency curve saved to: motor_efficiency_analysis.png")
    plt.show()

if __name__ == "__main__":
    results = solve_shunt_motor_efficiency()
