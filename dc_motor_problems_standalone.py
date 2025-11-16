#!/usr/bin/env python3
"""
DC Shunt Motor Problem Solver - Standalone Version
No dependencies, pure Python calculations
"""


class DCShuntMotorProblems:
    """Analytical solutions for DC shunt motor problems"""

    @staticmethod
    def problem_1():
        """
        Problem 1: 220-V, 10-kW, 2500 r.p.m. shunt motor
        Given:
        - V = 220V, P = 10kW, N = 2500 rpm
        - I_line = 41A at rated conditions
        - Ra = 0.2Ω, Rc = 0.05Ω, Ri = 0.1Ω, Rf = 110Ω

        Find: Ia and N when:
        - Flux reduced by 25% (φ2 = 0.75φ1)
        - 1Ω series resistance added to armature
        - Load torque reduced by 50%
        """
        print("\n" + "="*80)
        print("PROBLEM 1 SOLUTION")
        print("="*80)

        # Given parameters
        V = 220  # V
        P_rated = 10000  # W
        N1 = 2500  # rpm
        I_line_rated = 41  # A
        Ra = 0.2  # Ω
        Rc = 0.05  # Ω
        Ri = 0.1  # Ω
        Rf = 110  # Ω

        # Total armature circuit resistance
        Ra_total = Ra + Rc + Ri  # 0.35Ω

        print(f"\nGiven Parameters:")
        print(f"  Supply Voltage: {V} V")
        print(f"  Rated Power: {P_rated/1000} kW")
        print(f"  Rated Speed: {N1} rpm")
        print(f"  Line Current (rated): {I_line_rated} A")
        print(f"  Ra = {Ra}Ω, Rc = {Rc}Ω, Ri = {Ri}Ω, Rf = {Rf}Ω")
        print(f"  Total armature resistance: {Ra_total}Ω")

        # Rated condition calculations
        If1 = V / Rf  # Field current
        Ia1 = I_line_rated - If1  # Armature current at rated
        Eb1 = V - Ia1 * Ra_total  # Back EMF at rated

        print(f"\nRated Condition Analysis:")
        print(f"  Field current If = V/Rf = {V}/{Rf} = {If1:.3f} A")
        print(f"  Armature current Ia1 = {I_line_rated} - {If1:.3f} = {Ia1:.3f} A")
        print(f"  Back EMF Eb1 = {V} - {Ia1:.3f}×{Ra_total} = {Eb1:.3f} V")

        # New conditions
        phi2_phi1 = 0.75  # Flux reduced by 25%
        R_series = 1.0  # Series resistance added
        Ra_total_new = Ra_total + R_series  # New total armature resistance
        T2_T1 = 0.5  # Load torque reduced by 50%

        print(f"\nNew Operating Conditions:")
        print(f"  Flux ratio φ2/φ1 = {phi2_phi1}")
        print(f"  Series resistance added: {R_series}Ω")
        print(f"  New total armature resistance: {Ra_total_new}Ω")
        print(f"  Torque ratio T2/T1 = {T2_T1}")

        # Field current remains same (shunt field connected across supply)
        If2 = If1

        # For torque: T ∝ φ × Ia
        # T2/T1 = (φ2/φ1) × (Ia2/Ia1)
        # 0.5 = 0.75 × (Ia2/Ia1)
        # Ia2 = 0.5/0.75 × Ia1
        Ia2 = (T2_T1 / phi2_phi1) * Ia1

        print(f"\nArmature Current Calculation:")
        print(f"  Since T ∝ φ×Ia:")
        print(f"  T2/T1 = (φ2/φ1) × (Ia2/Ia1)")
        print(f"  {T2_T1} = {phi2_phi1} × (Ia2/{Ia1:.3f})")
        print(f"  Ia2 = {T2_T1}/{phi2_phi1} × {Ia1:.3f} = {Ia2:.3f} A")

        # Back EMF in new condition
        Eb2 = V - Ia2 * Ra_total_new

        print(f"  Back EMF Eb2 = {V} - {Ia2:.3f}×{Ra_total_new} = {Eb2:.3f} V")

        # Speed calculation
        # Eb ∝ φ × N
        # Eb2/Eb1 = (φ2/φ1) × (N2/N1)
        # N2 = (Eb2/Eb1) × (φ1/φ2) × N1
        N2 = (Eb2 / Eb1) * (1 / phi2_phi1) * N1

        print(f"\nSpeed Calculation:")
        print(f"  Since Eb ∝ φ×N:")
        print(f"  Eb2/Eb1 = (φ2/φ1) × (N2/N1)")
        print(f"  {Eb2:.3f}/{Eb1:.3f} = {phi2_phi1} × (N2/{N1})")
        print(f"  N2 = ({Eb2:.3f}/{Eb1:.3f}) × (1/{phi2_phi1}) × {N1}")
        print(f"  N2 = {N2:.2f} rpm")

        # Total line current
        I_line2 = Ia2 + If2

        print(f"\n" + "="*80)
        print("FINAL RESULTS - PROBLEM 1:")
        print("="*80)
        print(f"  Armature Current (Ia2) = {Ia2:.3f} A")
        print(f"  Motor Speed (N2) = {N2:.2f} rpm")
        print(f"  Line Current = {I_line2:.3f} A")
        print(f"  Power Output = {(Eb2 * Ia2):.2f} W")
        print("="*80)

        return {
            'Ia2': Ia2,
            'N2': N2,
            'Eb2': Eb2,
            'If2': If2,
            'I_line2': I_line2,
            'power_output': Eb2 * Ia2
        }

    @staticmethod
    def problem_2():
        """
        Problem 2: DC shunt motor speed control
        Given:
        - V = 220V, Ia = 20A, Ra = 0.5Ω
        - Reduce speed by 50%

        Find resistance for:
        (a) Constant load torque
        (b) Load torque ∝ speed²
        """
        print("\n" + "="*80)
        print("PROBLEM 2 SOLUTION")
        print("="*80)

        # Given parameters
        V = 220  # V
        Ia1 = 20  # A
        Ra = 0.5  # Ω
        N2_N1 = 0.5  # Speed reduced by 50%

        print(f"\nGiven Parameters:")
        print(f"  Supply Voltage: {V} V")
        print(f"  Armature Current (initial): {Ia1} A")
        print(f"  Armature Resistance: {Ra} Ω")
        print(f"  Speed Ratio N2/N1: {N2_N1}")

        # Initial back EMF
        Eb1 = V - Ia1 * Ra

        print(f"\nInitial Condition:")
        print(f"  Back EMF Eb1 = {V} - {Ia1}×{Ra} = {Eb1} V")

        # (a) Constant load torque
        print(f"\n" + "-"*80)
        print("(a) CONSTANT LOAD TORQUE")
        print("-"*80)

        # For shunt motor with constant flux: T ∝ Ia
        # Constant torque means constant Ia
        Ia2_a = Ia1

        print(f"  For constant torque: Ia2 = Ia1 = {Ia2_a} A")

        # Eb ∝ N for constant flux
        # Eb2/Eb1 = N2/N1
        Eb2_a = Eb1 * N2_N1

        print(f"  Since Eb ∝ N: Eb2 = Eb1 × (N2/N1) = {Eb1} × {N2_N1} = {Eb2_a} V")

        # Voltage equation: V = Eb2 + Ia2 × (Ra + R_series)
        # R_series = (V - Eb2)/Ia2 - Ra
        R_series_a = (V - Eb2_a) / Ia2_a - Ra

        print(f"  From V = Eb2 + Ia2×(Ra + Rs):")
        print(f"  Rs = (V - Eb2)/Ia2 - Ra")
        print(f"  Rs = ({V} - {Eb2_a})/{Ia2_a} - {Ra}")
        print(f"  Rs = {R_series_a:.3f} Ω")

        # (b) Load torque proportional to speed²
        print(f"\n" + "-"*80)
        print("(b) LOAD TORQUE ∝ SPEED²")
        print("-"*80)

        # T ∝ N²
        # Also T ∝ Ia for constant flux
        # Therefore: Ia ∝ N²
        # Ia2/Ia1 = (N2/N1)²
        Ia2_b = Ia1 * (N2_N1 ** 2)

        print(f"  Since T ∝ N² and T ∝ Ia:")
        print(f"  Ia2/Ia1 = (N2/N1)²")
        print(f"  Ia2 = Ia1 × ({N2_N1})² = {Ia1} × {N2_N1**2} = {Ia2_b} A")

        # Back EMF
        Eb2_b = Eb1 * N2_N1

        print(f"  Eb2 = Eb1 × (N2/N1) = {Eb1} × {N2_N1} = {Eb2_b} V")

        # Series resistance
        R_series_b = (V - Eb2_b) / Ia2_b - Ra

        print(f"  Rs = (V - Eb2)/Ia2 - Ra")
        print(f"  Rs = ({V} - {Eb2_b})/{Ia2_b} - {Ra}")
        print(f"  Rs = {R_series_b:.3f} Ω")

        print(f"\n" + "="*80)
        print("FINAL RESULTS - PROBLEM 2:")
        print("="*80)
        print(f"(a) Constant Load Torque:")
        print(f"    Series Resistance Required = {R_series_a:.3f} Ω")
        print(f"    Armature Current = {Ia2_a:.2f} A")
        print(f"\n(b) Load Torque ∝ Speed²:")
        print(f"    Series Resistance Required = {R_series_b:.3f} Ω")
        print(f"    Armature Current = {Ia2_b:.2f} A")
        print("="*80)

        return {
            'case_a': {
                'R_series': R_series_a,
                'Ia2': Ia2_a,
                'Eb2': Eb2_a
            },
            'case_b': {
                'R_series': R_series_b,
                'Ia2': Ia2_b,
                'Eb2': Eb2_b
            }
        }


def main():
    """Main entry point"""
    print("\n")
    print("="*80)
    print(" DC SHUNT MOTOR ANALYTICAL PROBLEM SOLUTIONS")
    print("="*80)

    # Solve problems
    result1 = DCShuntMotorProblems.problem_1()
    result2 = DCShuntMotorProblems.problem_2()

    print("\n\n")
    print("="*80)
    print("SUMMARY OF RESULTS")
    print("="*80)

    print("\nPROBLEM 1 - Final Answers:")
    print(f"  • Armature Current: {result1['Ia2']:.3f} A")
    print(f"  • Motor Speed: {result1['N2']:.2f} rpm")
    print(f"  • Back EMF: {result1['Eb2']:.3f} V")
    print(f"  • Power Output: {result1['power_output']:.2f} W")

    print("\nPROBLEM 2 - Final Answers:")
    print(f"  (a) Constant Load Torque:")
    print(f"      • Series Resistance: {result2['case_a']['R_series']:.3f} Ω")
    print(f"      • Armature Current: {result2['case_a']['Ia2']:.2f} A")
    print(f"  (b) Load Torque ∝ Speed²:")
    print(f"      • Series Resistance: {result2['case_b']['R_series']:.3f} Ω")
    print(f"      • Armature Current: {result2['case_b']['Ia2']:.2f} A")

    print("\n" + "="*80)
    print("ALL CALCULATIONS COMPLETED SUCCESSFULLY!")
    print("="*80 + "\n")


if __name__ == "__main__":
    main()
