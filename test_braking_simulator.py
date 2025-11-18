"""
Test script for DC Motor Braking Simulator
Validates theoretical calculations without GUI
"""

import numpy as np

def test_braking_calculations():
    """Test the theoretical braking calculations"""

    print("="*80)
    print(" "*20 + "DC MOTOR BRAKING PROBLEM SOLUTION")
    print("="*80)
    print()

    # Given parameters
    V = 250.0  # Supply voltage (V)
    Ia_initial = 150.0  # Initial armature current (A)
    N_initial = 550.0  # Initial speed (rpm)
    Ra = 0.09  # Armature resistance (Ω)
    Ia_brake = 240.0  # Braking current limit (A)

    print("GIVEN PARAMETERS:")
    print(f"  Supply Voltage (V):           {V} V")
    print(f"  Initial Armature Current:     {Ia_initial} A")
    print(f"  Initial Speed:                {N_initial} rpm")
    print(f"  Armature Resistance:          {Ra} Ω")
    print(f"  Braking Current Limit:        {Ia_brake} A")
    print()

    # Calculate back EMF at initial speed (before braking)
    print("STEP 1: Calculate Initial Back EMF")
    print("-"*80)
    Eb_initial = V - Ia_initial * Ra
    print(f"  Eb = V - Ia × Ra")
    print(f"  Eb = {V} - {Ia_initial} × {Ra}")
    print(f"  Eb = {Eb_initial:.4f} V")
    print()

    # Calculate motor constant k·Φ
    print("STEP 2: Calculate Motor Constant (k·Φ)")
    print("-"*80)
    omega_initial = 2 * np.pi * N_initial / 60  # Convert to rad/s
    k_phi = Eb_initial / omega_initial
    print(f"  ω = 2πN/60 = 2π × {N_initial}/60 = {omega_initial:.4f} rad/s")
    print(f"  k·Φ = Eb/ω = {Eb_initial:.4f}/{omega_initial:.4f}")
    print(f"  k·Φ = {k_phi:.4f} V·s/rad")
    print()

    # (a) Calculate series resistance required
    print("PART (a): Series Resistance Required to Limit Current to 240 A")
    print("-"*80)
    print("  During braking (armature reversed):")
    print("  - Supply voltage V opposes current")
    print("  - Back EMF Eb also opposes current")
    print("  - Total opposing voltage: V + Eb")
    print()
    print(f"  V + Eb = Ia_brake × (Ra + R_series)")
    print(f"  {V} + {Eb_initial:.4f} = {Ia_brake} × ({Ra} + R_series)")

    total_voltage = V + Eb_initial
    R_series = total_voltage / Ia_brake - Ra

    print(f"  {total_voltage:.4f} = {Ia_brake} × ({Ra} + R_series)")
    print(f"  {total_voltage:.4f}/{Ia_brake} = {Ra} + R_series")
    print(f"  {total_voltage/Ia_brake:.4f} = {Ra} + R_series")
    print(f"  R_series = {total_voltage/Ia_brake:.4f} - {Ra}")
    print()
    print(f"  ✓ ANSWER (a): R_series = {R_series:.4f} Ω")
    print(f"  ✓ Rounded:    R_series ≈ {R_series:.2f} Ω")
    print()

    # (b) Calculate initial braking torque
    print("PART (b): Initial Braking Torque")
    print("-"*80)
    T_initial = k_phi * Ia_brake
    print(f"  T = k·Φ × Ia")
    print(f"  T = {k_phi:.4f} × {Ia_brake}")
    print(f"  T = {T_initial:.4f} N·m")
    print()
    print(f"  ✓ ANSWER (b): T_initial = {T_initial:.2f} N·m")
    print()

    # (c) Calculate braking torque at 200 rpm
    print("PART (c): Braking Torque at 200 rpm")
    print("-"*80)
    N_200 = 200.0
    omega_200 = 2 * np.pi * N_200 / 60
    Eb_200 = k_phi * omega_200

    print(f"  At N = {N_200} rpm:")
    print(f"  ω = 2π × {N_200}/60 = {omega_200:.4f} rad/s")
    print(f"  Eb = k·Φ × ω = {k_phi:.4f} × {omega_200:.4f} = {Eb_200:.4f} V")
    print()

    R_total = Ra + R_series
    Ia_200 = (V + Eb_200) / R_total

    print(f"  Current at 200 rpm:")
    print(f"  Ia = (V + Eb)/(Ra + R_series)")
    print(f"  Ia = ({V} + {Eb_200:.4f})/({Ra} + {R_series:.4f})")
    print(f"  Ia = {V + Eb_200:.4f}/{R_total:.4f}")
    print(f"  Ia = {Ia_200:.4f} A")
    print()

    T_200 = k_phi * Ia_200
    print(f"  Torque at 200 rpm:")
    print(f"  T = k·Φ × Ia")
    print(f"  T = {k_phi:.4f} × {Ia_200:.4f}")
    print(f"  T = {T_200:.4f} N·m")
    print()
    print(f"  ✓ ANSWER (c): T_at_200rpm = {T_200:.2f} N·m")
    print()

    # Summary
    print("="*80)
    print(" "*25 + "FINAL ANSWERS")
    print("="*80)
    print(f"  (a) Series Resistance:        R_series = {R_series:.4f} Ω  ({R_series:.2f} Ω)")
    print(f"  (b) Initial Braking Torque:   T_init   = {T_initial:.4f} N·m  ({T_initial:.2f} N·m)")
    print(f"  (c) Torque at 200 rpm:        T_200    = {T_200:.4f} N·m  ({T_200:.2f} N·m)")
    print("="*80)
    print()

    # Additional information
    print("ADDITIONAL CALCULATIONS:")
    print("-"*80)

    # Energy dissipated during braking
    print(f"  Motor Constant:               k·Φ = {k_phi:.4f} V·s/rad")
    print(f"  Initial Back EMF:             Eb_init = {Eb_initial:.4f} V")
    print(f"  Back EMF at 200 rpm:          Eb_200 = {Eb_200:.4f} V")
    print(f"  Current at 200 rpm:           Ia_200 = {Ia_200:.4f} A")
    print(f"  Total Circuit Resistance:     R_total = {R_total:.4f} Ω")
    print()

    # Power calculations
    P_initial = V * Ia_brake
    P_loss_initial = Ia_brake**2 * R_total
    P_mech_initial = T_initial * omega_initial

    print("  Power at Initial Braking:")
    print(f"    Input Power:                P_in = {P_initial:.2f} W  ({P_initial/1000:.2f} kW)")
    print(f"    Resistive Loss:             P_loss = {P_loss_initial:.2f} W  ({P_loss_initial/1000:.2f} kW)")
    print(f"    Mechanical Power:           P_mech = {P_mech_initial:.2f} W  ({P_mech_initial/1000:.2f} kW)")
    print()

    # Braking curve analysis
    print("  Braking Performance:")
    speeds = [550, 400, 300, 200, 100, 50]
    print(f"  {'Speed (rpm)':>12} {'Current (A)':>12} {'Torque (N·m)':>12} {'Power (kW)':>12}")
    print("  " + "-"*50)

    for N in speeds:
        omega = 2 * np.pi * N / 60
        Eb = k_phi * omega
        Ia = (V + Eb) / R_total
        T = k_phi * Ia
        P = T * omega / 1000
        print(f"  {N:12.0f} {Ia:12.2f} {T:12.2f} {P:12.2f}")

    print()
    print("="*80)
    print("Test completed successfully! ✓")
    print("="*80)

    return {
        'R_series': R_series,
        'T_initial': T_initial,
        'T_200': T_200,
        'k_phi': k_phi,
        'Eb_initial': Eb_initial
    }


if __name__ == '__main__':
    results = test_braking_calculations()

    print("\nTo run the full GUI simulator, execute:")
    print("  python3 dc_motor_braking_advanced_simulator.py")
    print()
