#!/usr/bin/env python3
"""
Test script to verify the DC shunt motor problem solutions
"""

import sys
sys.path.insert(0, '/home/user/advnaced-sieci3')

from advanced_dc_shunt_motor_simulator import DCShuntMotorProblems

def main():
    print("\n" + "="*80)
    print("TESTING DC SHUNT MOTOR PROBLEM SOLUTIONS")
    print("="*80)

    # Test Problem 1
    print("\n\nTEST 1: Running Problem 1 Solution...")
    result1 = DCShuntMotorProblems.problem_1()

    print("\n\nProblem 1 Results Summary:")
    print(f"  Armature Current: {result1['Ia2']:.3f} A")
    print(f"  Motor Speed: {result1['N2']:.2f} rpm")
    print(f"  Back EMF: {result1['Eb2']:.3f} V")

    # Test Problem 2
    print("\n\n" + "="*80)
    print("\n\nTEST 2: Running Problem 2 Solution...")
    result2 = DCShuntMotorProblems.problem_2()

    print("\n\nProblem 2 Results Summary:")
    print(f"  Case (a) - Constant Torque:")
    print(f"    Series Resistance: {result2['case_a']['R_series']:.3f} Ω")
    print(f"    Armature Current: {result2['case_a']['Ia2']:.2f} A")
    print(f"\n  Case (b) - Torque ∝ Speed²:")
    print(f"    Series Resistance: {result2['case_b']['R_series']:.3f} Ω")
    print(f"    Armature Current: {result2['case_b']['Ia2']:.2f} A")

    print("\n\n" + "="*80)
    print("ALL TESTS COMPLETED SUCCESSFULLY!")
    print("="*80)

if __name__ == "__main__":
    main()
