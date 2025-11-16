#!/usr/bin/env python3
"""
Advanced DC Shunt Motor Multi-Physics Simulator
Comprehensive electrical engineering analysis tool with GUI
Includes electromagnetic-thermal-mechanical coupling, economic analysis, and real-time simulation
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp, odeint
import json
from datetime import datetime
import math


# ============================================================================
# PROBLEM SOLVERS - Analytical Solutions
# ============================================================================

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

        # Torque at rated conditions
        # T ∝ φ × Ia, and Eb ∝ φ × N
        # At rated: T1 ∝ φ1 × Ia1

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


# ============================================================================
# MULTI-PHYSICS SIMULATION ENGINE
# ============================================================================

class MultiPhysicsMotorModel:
    """
    Advanced multi-physics DC shunt motor model
    Couples electromagnetic, thermal, and mechanical systems
    """

    def __init__(self):
        # Electrical parameters
        self.V_supply = 220.0  # Supply voltage (V)
        self.Ra = 0.35  # Armature resistance (Ω)
        self.Rf = 110.0  # Field resistance (Ω)
        self.La = 0.05  # Armature inductance (H)
        self.Lf = 5.0  # Field inductance (H)

        # Mechanical parameters
        self.J = 0.5  # Moment of inertia (kg·m²)
        self.B = 0.01  # Viscous friction coefficient (N·m·s/rad)
        self.Kt = 1.2  # Torque constant (N·m/A)
        self.Ke = 1.2  # Back EMF constant (V·s/rad)

        # Thermal parameters
        self.thermal_resistance_a = 2.0  # Armature thermal resistance (°C/W)
        self.thermal_resistance_f = 3.0  # Field thermal resistance (°C/W)
        self.thermal_capacitance_a = 500.0  # Armature thermal capacitance (J/°C)
        self.thermal_capacitance_f = 800.0  # Field thermal capacitance (J/°C)
        self.ambient_temp = 25.0  # Ambient temperature (°C)
        self.temp_coeff_resistance = 0.00393  # Copper temperature coefficient

        # Iron loss parameters
        self.Kh = 0.002  # Hysteresis loss coefficient
        self.Ke_eddy = 0.0001  # Eddy current loss coefficient

        # Mechanical loss parameters
        self.friction_constant = 0.5  # Friction loss constant
        self.windage_constant = 0.0001  # Windage loss constant

        # State variables
        self.state = {
            'Ia': 0.0,  # Armature current
            'If': 0.0,  # Field current
            'omega': 0.0,  # Angular velocity (rad/s)
            'theta': 0.0,  # Angular position (rad)
            'temp_a': 25.0,  # Armature temperature
            'temp_f': 25.0,  # Field temperature
        }

        # Load torque
        self.T_load = 0.0

        # Control parameters
        self.R_series = 0.0  # Series resistance for speed control
        self.flux_weakening = 1.0  # Flux weakening factor (1.0 = full flux)

    def update_parameters(self, params):
        """Update motor parameters from GUI"""
        for key, value in params.items():
            if hasattr(self, key):
                setattr(self, key, value)

    def temperature_corrected_resistance(self, R_base, temp, temp_base=25.0):
        """Calculate temperature-corrected resistance"""
        return R_base * (1 + self.temp_coeff_resistance * (temp - temp_base))

    def iron_loss(self, omega, flux):
        """Calculate iron losses (hysteresis + eddy current)"""
        f = omega / (2 * np.pi)  # Frequency
        P_hysteresis = self.Kh * f * flux**2
        P_eddy = self.Ke_eddy * f**2 * flux**2
        return P_hysteresis + P_eddy

    def mechanical_loss(self, omega):
        """Calculate mechanical losses (friction + windage)"""
        P_friction = self.friction_constant * omega
        P_windage = self.windage_constant * omega**3
        return P_friction + P_windage

    def differential_equations(self, t, y):
        """
        System of differential equations for multi-physics simulation
        State vector: y = [Ia, If, omega, theta, temp_a, temp_f]
        """
        Ia, If, omega, theta, temp_a, temp_f = y

        # Temperature-corrected resistances
        Ra_corrected = self.temperature_corrected_resistance(self.Ra + self.R_series, temp_a)
        Rf_corrected = self.temperature_corrected_resistance(self.Rf, temp_f)

        # Back EMF
        Eb = self.Ke * If * self.flux_weakening * omega

        # Electrical equations
        # Armature circuit: V = Eb + Ia*Ra + La*dIa/dt
        dIa_dt = (self.V_supply - Eb - Ia * Ra_corrected) / self.La

        # Field circuit: V = If*Rf + Lf*dIf/dt
        dIf_dt = (self.V_supply - If * Rf_corrected) / self.Lf

        # Electromagnetic torque
        T_em = self.Kt * If * self.flux_weakening * Ia

        # Mechanical equation
        # J*dω/dt = T_em - T_load - B*ω
        domega_dt = (T_em - self.T_load - self.B * omega) / self.J

        # Angular position
        dtheta_dt = omega

        # Thermal equations
        # Armature: Copper losses + portion of iron losses
        P_copper_a = Ia**2 * Ra_corrected
        P_iron = self.iron_loss(omega, If * self.flux_weakening)
        P_loss_a = P_copper_a + 0.7 * P_iron  # 70% of iron loss in armature

        # Heat transfer: C*dT/dt = P_loss - (T - T_amb)/R_th
        dtemp_a_dt = (P_loss_a - (temp_a - self.ambient_temp) / self.thermal_resistance_a) / self.thermal_capacitance_a

        # Field: Copper losses
        P_copper_f = If**2 * Rf_corrected
        P_loss_f = P_copper_f + 0.3 * P_iron  # 30% of iron loss in field

        dtemp_f_dt = (P_loss_f - (temp_f - self.ambient_temp) / self.thermal_resistance_f) / self.thermal_capacitance_f

        return [dIa_dt, dIf_dt, domega_dt, dtheta_dt, dtemp_a_dt, dtemp_f_dt]

    def simulate_transient(self, t_span, method='RK45', initial_state=None):
        """
        Simulate motor transient response

        Args:
            t_span: (t_start, t_end) tuple
            method: 'RK45' or 'Euler'
            initial_state: Initial state vector (if None, use current state)

        Returns:
            Solution object with time and state history
        """
        if initial_state is None:
            y0 = [
                self.state['Ia'],
                self.state['If'],
                self.state['omega'],
                self.state['theta'],
                self.state['temp_a'],
                self.state['temp_f']
            ]
        else:
            y0 = initial_state

        if method == 'RK45':
            # Use scipy's adaptive Runge-Kutta method
            sol = solve_ivp(
                self.differential_equations,
                t_span,
                y0,
                method='RK45',
                dense_output=True,
                max_step=0.001
            )
            return sol

        elif method == 'Euler':
            # Implement simple Euler method
            dt = 0.0001  # Time step
            t = np.arange(t_span[0], t_span[1], dt)
            n_steps = len(t)
            y = np.zeros((6, n_steps))
            y[:, 0] = y0

            for i in range(1, n_steps):
                dy = self.differential_equations(t[i-1], y[:, i-1])
                y[:, i] = y[:, i-1] + np.array(dy) * dt

            # Create solution object compatible with solve_ivp
            class EulerSolution:
                def __init__(self, t, y):
                    self.t = t
                    self.y = y
                    self.success = True

            return EulerSolution(t, y)

        else:
            raise ValueError(f"Unknown method: {method}")

    def calculate_losses(self, Ia, If, omega):
        """Calculate detailed loss breakdown"""
        # Temperature-corrected resistances
        Ra_corrected = self.temperature_corrected_resistance(
            self.Ra + self.R_series,
            self.state['temp_a']
        )
        Rf_corrected = self.temperature_corrected_resistance(
            self.Rf,
            self.state['temp_f']
        )

        # Copper losses
        P_copper_armature = Ia**2 * Ra_corrected
        P_copper_field = If**2 * Rf_corrected

        # Iron losses
        P_iron = self.iron_loss(omega, If * self.flux_weakening)

        # Mechanical losses
        P_mechanical = self.mechanical_loss(omega)

        # Stray load losses (approximately 1% of output power)
        Eb = self.Ke * If * self.flux_weakening * omega
        P_output = Eb * Ia
        P_stray = 0.01 * abs(P_output)

        return {
            'copper_armature': P_copper_armature,
            'copper_field': P_copper_field,
            'iron': P_iron,
            'mechanical': P_mechanical,
            'stray': P_stray,
            'total': P_copper_armature + P_copper_field + P_iron + P_mechanical + P_stray
        }

    def calculate_efficiency(self, Ia, If, omega):
        """Calculate motor efficiency"""
        Eb = self.Ke * If * self.flux_weakening * omega
        P_output = Eb * Ia
        losses = self.calculate_losses(Ia, If, omega)
        P_input = P_output + losses['total']

        if P_input > 0:
            efficiency = (P_output / P_input) * 100
        else:
            efficiency = 0

        return efficiency, P_input, P_output

    def calculate_derating_factor(self):
        """Calculate derating factor based on temperature"""
        max_temp = max(self.state['temp_a'], self.state['temp_f'])
        rated_temp = 75.0  # °C
        max_allowable_temp = 130.0  # °C

        if max_temp <= rated_temp:
            return 1.0
        elif max_temp >= max_allowable_temp:
            return 0.0
        else:
            # Linear derating between rated and max temperature
            return 1.0 - (max_temp - rated_temp) / (max_allowable_temp - rated_temp)


# ============================================================================
# ADVANCED GUI APPLICATION
# ============================================================================

class AdvancedDCMotorSimulator:
    """
    Main GUI application for DC shunt motor simulation
    """

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Shunt Motor Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Initialize motor model
        self.motor = MultiPhysicsMotorModel()

        # Simulation state
        self.is_running = False
        self.simulation_time = 0.0
        self.time_history = []
        self.state_history = {
            'Ia': [], 'If': [], 'omega': [], 'theta': [],
            'temp_a': [], 'temp_f': [], 'torque': [], 'power': [],
            'efficiency': [], 'losses': []
        }

        # Setup GUI
        self.setup_menu()
        self.setup_main_interface()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

        # Display problem solutions on startup
        self.display_problem_solutions()

    def setup_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Results", command=self.save_results)
        file_menu.add_command(label="Load Parameters", command=self.load_parameters)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Simulation menu
        sim_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Simulation", menu=sim_menu)
        sim_menu.add_command(label="Run Analysis", command=self.run_simulation)
        sim_menu.add_command(label="Reset", command=self.reset_simulation)

        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        tools_menu.add_command(label="Problem 1 Solution", command=lambda: DCShuntMotorProblems.problem_1())
        tools_menu.add_command(label="Problem 2 Solution", command=lambda: DCShuntMotorProblems.problem_2())

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def setup_main_interface(self):
        """Setup main tabbed interface"""
        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.tab_control = ttk.Frame(self.notebook)
        self.tab_simulation = ttk.Frame(self.notebook)
        self.tab_analysis = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)
        self.tab_thermal = ttk.Frame(self.notebook)
        self.tab_problems = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_control, text='⚡ Control & Parameters')
        self.notebook.add(self.tab_simulation, text='📊 Dynamic Simulation')
        self.notebook.add(self.tab_analysis, text='📈 Loss Analysis')
        self.notebook.add(self.tab_economic, text='💰 Economic Analysis')
        self.notebook.add(self.tab_thermal, text='🌡️ Thermal Analysis')
        self.notebook.add(self.tab_problems, text='📝 Problem Solutions')

        # Setup each tab
        self.setup_control_tab()
        self.setup_simulation_tab()
        self.setup_analysis_tab()
        self.setup_economic_tab()
        self.setup_thermal_tab()
        self.setup_problems_tab()

    def setup_control_tab(self):
        """Setup control and parameters tab"""
        # Left panel - Parameters
        left_frame = ttk.LabelFrame(self.tab_control, text="Motor Parameters", padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create scrollable frame
        canvas = tk.Canvas(left_frame, width=400)
        scrollbar = ttk.Scrollbar(left_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Electrical parameters
        ttk.Label(scrollable_frame, text="Electrical Parameters", font=('Arial', 10, 'bold')).grid(row=0, column=0, columnspan=3, pady=5)

        self.params = {}
        row = 1

        param_definitions = [
            ('V_supply', 'Supply Voltage (V)', 220, 0, 500),
            ('Ra', 'Armature Resistance (Ω)', 0.35, 0.01, 5),
            ('Rf', 'Field Resistance (Ω)', 110, 10, 500),
            ('La', 'Armature Inductance (H)', 0.05, 0.001, 1),
            ('Lf', 'Field Inductance (H)', 5.0, 0.1, 50),
        ]

        for param, label, default, min_val, max_val in param_definitions:
            ttk.Label(scrollable_frame, text=label).grid(row=row, column=0, sticky='w', pady=2)
            var = tk.DoubleVar(value=default)
            self.params[param] = var

            scale = ttk.Scale(scrollable_frame, from_=min_val, to=max_val,
                            orient='horizontal', variable=var, length=200)
            scale.grid(row=row, column=1, padx=5)

            entry = ttk.Entry(scrollable_frame, textvariable=var, width=10)
            entry.grid(row=row, column=2, padx=5)
            row += 1

        # Mechanical parameters
        ttk.Label(scrollable_frame, text="Mechanical Parameters", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=3, pady=5)
        row += 1

        mech_params = [
            ('J', 'Moment of Inertia (kg·m²)', 0.5, 0.01, 5),
            ('B', 'Friction Coefficient (N·m·s/rad)', 0.01, 0.001, 0.1),
            ('Kt', 'Torque Constant (N·m/A)', 1.2, 0.1, 5),
            ('Ke', 'Back EMF Constant (V·s/rad)', 1.2, 0.1, 5),
        ]

        for param, label, default, min_val, max_val in mech_params:
            ttk.Label(scrollable_frame, text=label).grid(row=row, column=0, sticky='w', pady=2)
            var = tk.DoubleVar(value=default)
            self.params[param] = var

            scale = ttk.Scale(scrollable_frame, from_=min_val, to=max_val,
                            orient='horizontal', variable=var, length=200)
            scale.grid(row=row, column=1, padx=5)

            entry = ttk.Entry(scrollable_frame, textvariable=var, width=10)
            entry.grid(row=row, column=2, padx=5)
            row += 1

        # Control parameters
        ttk.Label(scrollable_frame, text="Control Parameters", font=('Arial', 10, 'bold')).grid(row=row, column=0, columnspan=3, pady=5)
        row += 1

        control_params = [
            ('T_load', 'Load Torque (N·m)', 0, 0, 100),
            ('R_series', 'Series Resistance (Ω)', 0, 0, 10),
            ('flux_weakening', 'Flux Weakening Factor', 1.0, 0.5, 1.0),
        ]

        for param, label, default, min_val, max_val in control_params:
            ttk.Label(scrollable_frame, text=label).grid(row=row, column=0, sticky='w', pady=2)
            var = tk.DoubleVar(value=default)
            self.params[param] = var

            scale = ttk.Scale(scrollable_frame, from_=min_val, to=max_val,
                            orient='horizontal', variable=var, length=200)
            scale.grid(row=row, column=1, padx=5)

            entry = ttk.Entry(scrollable_frame, textvariable=var, width=10)
            entry.grid(row=row, column=2, padx=5)
            row += 1

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Right panel - Control buttons and status
        right_frame = ttk.LabelFrame(self.tab_control, text="Control Panel", padding=10)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        # Control buttons
        btn_frame = ttk.Frame(right_frame)
        btn_frame.pack(pady=10)

        self.btn_start = ttk.Button(btn_frame, text="▶ Start", command=self.start_simulation, width=15)
        self.btn_start.grid(row=0, column=0, padx=5, pady=5)

        self.btn_stop = ttk.Button(btn_frame, text="⏸ Stop", command=self.stop_simulation, width=15, state='disabled')
        self.btn_stop.grid(row=0, column=1, padx=5, pady=5)

        self.btn_reset = ttk.Button(btn_frame, text="🔄 Reset", command=self.reset_simulation, width=15)
        self.btn_reset.grid(row=1, column=0, padx=5, pady=5)

        self.btn_analyze = ttk.Button(btn_frame, text="📊 Analyze", command=self.run_simulation, width=15)
        self.btn_analyze.grid(row=1, column=1, padx=5, pady=5)

        # Solver selection
        solver_frame = ttk.LabelFrame(right_frame, text="ODE Solver", padding=5)
        solver_frame.pack(pady=10, fill='x')

        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(solver_frame, text="RK45 (Adaptive)", variable=self.solver_var,
                       value='RK45').pack(anchor='w')
        ttk.Radiobutton(solver_frame, text="Euler (Fixed Step)", variable=self.solver_var,
                       value='Euler').pack(anchor='w')

        # Status display
        status_frame = ttk.LabelFrame(right_frame, text="Real-Time Status", padding=10)
        status_frame.pack(pady=10, fill='both', expand=True)

        self.status_labels = {}
        status_params = [
            ('Ia', 'Armature Current (A)', '0.00'),
            ('If', 'Field Current (A)', '0.00'),
            ('Speed', 'Speed (rpm)', '0.00'),
            ('Torque', 'Torque (N·m)', '0.00'),
            ('Power', 'Power (W)', '0.00'),
            ('Efficiency', 'Efficiency (%)', '0.00'),
            ('Temp_a', 'Armature Temp (°C)', '25.0'),
            ('Temp_f', 'Field Temp (°C)', '25.0'),
        ]

        for i, (key, label, default) in enumerate(status_params):
            ttk.Label(status_frame, text=label + ':').grid(row=i, column=0, sticky='w', pady=2)
            lbl = ttk.Label(status_frame, text=default, font=('Arial', 10, 'bold'))
            lbl.grid(row=i, column=1, sticky='e', pady=2)
            self.status_labels[key] = lbl

        # Configure grid weights
        self.tab_control.columnconfigure(0, weight=1)
        self.tab_control.columnconfigure(1, weight=1)
        self.tab_control.rowconfigure(0, weight=1)

    def setup_simulation_tab(self):
        """Setup dynamic simulation tab with real-time graphs"""
        # Create figure with subplots
        self.fig_sim = Figure(figsize=(12, 8), dpi=100)

        # Create subplots
        self.ax_current = self.fig_sim.add_subplot(3, 2, 1)
        self.ax_speed = self.fig_sim.add_subplot(3, 2, 2)
        self.ax_torque = self.fig_sim.add_subplot(3, 2, 3)
        self.ax_power = self.fig_sim.add_subplot(3, 2, 4)
        self.ax_temp = self.fig_sim.add_subplot(3, 2, 5)
        self.ax_efficiency = self.fig_sim.add_subplot(3, 2, 6)

        # Set labels
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Armature & Field Current')
        self.ax_current.grid(True, alpha=0.3)
        self.ax_current.legend(['Ia', 'If'])

        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.set_title('Motor Speed')
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.grid(True, alpha=0.3)

        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.set_title('Output Power')
        self.ax_power.grid(True, alpha=0.3)

        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Temperature Rise')
        self.ax_temp.grid(True, alpha=0.3)
        self.ax_temp.legend(['Armature', 'Field'])

        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('Motor Efficiency')
        self.ax_efficiency.grid(True, alpha=0.3)

        self.fig_sim.tight_layout()

        # Create canvas
        self.canvas_sim = FigureCanvasTkAgg(self.fig_sim, self.tab_simulation)
        self.canvas_sim.draw()
        self.canvas_sim.get_tk_widget().pack(fill='both', expand=True)

        # Add toolbar
        toolbar = NavigationToolbar2Tk(self.canvas_sim, self.tab_simulation)
        toolbar.update()

    def setup_analysis_tab(self):
        """Setup loss analysis tab"""
        # Create figure
        self.fig_analysis = Figure(figsize=(12, 8), dpi=100)

        # Loss breakdown pie chart
        self.ax_pie = self.fig_analysis.add_subplot(2, 2, 1)
        self.ax_pie.set_title('Loss Distribution')

        # Loss vs time
        self.ax_losses = self.fig_analysis.add_subplot(2, 2, 2)
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Losses (W)')
        self.ax_losses.set_title('Losses Over Time')
        self.ax_losses.grid(True, alpha=0.3)

        # Efficiency map
        self.ax_eff_map = self.fig_analysis.add_subplot(2, 2, 3)
        self.ax_eff_map.set_xlabel('Speed (rpm)')
        self.ax_eff_map.set_ylabel('Torque (N·m)')
        self.ax_eff_map.set_title('Efficiency Map')

        # Power flow diagram (bar chart)
        self.ax_power_flow = self.fig_analysis.add_subplot(2, 2, 4)
        self.ax_power_flow.set_ylabel('Power (W)')
        self.ax_power_flow.set_title('Power Flow')

        self.fig_analysis.tight_layout()

        # Create canvas
        self.canvas_analysis = FigureCanvasTkAgg(self.fig_analysis, self.tab_analysis)
        self.canvas_analysis.draw()
        self.canvas_analysis.get_tk_widget().pack(fill='both', expand=True)

        # Add toolbar
        toolbar = NavigationToolbar2Tk(self.canvas_analysis, self.tab_analysis)
        toolbar.update()

    def setup_economic_tab(self):
        """Setup economic analysis tab"""
        # Left panel - Input parameters
        left_frame = ttk.LabelFrame(self.tab_economic, text="Economic Parameters", padding=10)
        left_frame.pack(side='left', fill='both', expand=False, padx=5, pady=5)

        self.economic_params = {}

        params = [
            ('electricity_cost', 'Electricity Cost ($/kWh)', 0.12),
            ('operating_hours', 'Operating Hours (h/day)', 8),
            ('operating_days', 'Operating Days (days/year)', 250),
            ('motor_cost', 'Motor Initial Cost ($)', 5000),
            ('maintenance_cost', 'Annual Maintenance ($)', 500),
            ('lifetime', 'Expected Lifetime (years)', 15),
            ('discount_rate', 'Discount Rate (%)', 5),
        ]

        for i, (key, label, default) in enumerate(params):
            ttk.Label(left_frame, text=label).grid(row=i, column=0, sticky='w', pady=5)
            var = tk.DoubleVar(value=default)
            entry = ttk.Entry(left_frame, textvariable=var, width=15)
            entry.grid(row=i, column=1, padx=5, pady=5)
            self.economic_params[key] = var

        ttk.Button(left_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=len(params), column=0,
                                                         columnspan=2, pady=10)

        # Right panel - Results
        right_frame = ttk.LabelFrame(self.tab_economic, text="Economic Analysis Results", padding=10)
        right_frame.pack(side='right', fill='both', expand=True, padx=5, pady=5)

        self.economic_text = scrolledtext.ScrolledText(right_frame, height=20, width=60)
        self.economic_text.pack(fill='both', expand=True)

        # Initial calculation
        self.calculate_economics()

    def setup_thermal_tab(self):
        """Setup thermal analysis tab"""
        # Create figure
        self.fig_thermal = Figure(figsize=(12, 8), dpi=100)

        # Temperature vs time
        self.ax_temp_time = self.fig_thermal.add_subplot(2, 2, 1)
        self.ax_temp_time.set_xlabel('Time (s)')
        self.ax_temp_time.set_ylabel('Temperature (°C)')
        self.ax_temp_time.set_title('Temperature Transient')
        self.ax_temp_time.grid(True, alpha=0.3)

        # Temperature distribution
        self.ax_temp_dist = self.fig_thermal.add_subplot(2, 2, 2)
        self.ax_temp_dist.set_title('Temperature Distribution')

        # Thermal resistance network
        self.ax_thermal_network = self.fig_thermal.add_subplot(2, 2, 3)
        self.ax_thermal_network.set_title('Thermal Resistance Network')
        self.ax_thermal_network.axis('off')

        # Derating curve
        self.ax_derating = self.fig_thermal.add_subplot(2, 2, 4)
        self.ax_derating.set_xlabel('Temperature (°C)')
        self.ax_derating.set_ylabel('Derating Factor')
        self.ax_derating.set_title('Thermal Derating Curve')
        self.ax_derating.grid(True, alpha=0.3)

        # Plot derating curve
        temp_range = np.linspace(25, 150, 100)
        derating = np.zeros_like(temp_range)
        for i, t in enumerate(temp_range):
            if t <= 75:
                derating[i] = 1.0
            elif t >= 130:
                derating[i] = 0.0
            else:
                derating[i] = 1.0 - (t - 75) / (130 - 75)

        self.ax_derating.plot(temp_range, derating, 'r-', linewidth=2)
        self.ax_derating.axhline(y=1.0, color='g', linestyle='--', label='Full Load')
        self.ax_derating.axvline(x=75, color='b', linestyle='--', label='Rated Temp')
        self.ax_derating.axvline(x=130, color='r', linestyle='--', label='Max Temp')
        self.ax_derating.legend()

        self.fig_thermal.tight_layout()

        # Create canvas
        self.canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, self.tab_thermal)
        self.canvas_thermal.draw()
        self.canvas_thermal.get_tk_widget().pack(fill='both', expand=True)

        # Add toolbar
        toolbar = NavigationToolbar2Tk(self.canvas_thermal, self.tab_thermal)
        toolbar.update()

    def setup_problems_tab(self):
        """Setup problem solutions tab"""
        # Create text widget for solutions
        text_frame = ttk.Frame(self.tab_problems)
        text_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.problem_text = scrolledtext.ScrolledText(text_frame, font=('Courier', 10))
        self.problem_text.pack(fill='both', expand=True)

        # Button frame
        btn_frame = ttk.Frame(self.tab_problems)
        btn_frame.pack(fill='x', padx=5, pady=5)

        ttk.Button(btn_frame, text="Solve Problem 1",
                  command=self.solve_problem_1).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Solve Problem 2",
                  command=self.solve_problem_2).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Solve Both Problems",
                  command=self.display_problem_solutions).pack(side='left', padx=5)

    def display_problem_solutions(self):
        """Display solutions to both problems"""
        import sys
        from io import StringIO

        # Capture stdout
        old_stdout = sys.stdout
        sys.stdout = StringIO()

        try:
            # Solve both problems
            DCShuntMotorProblems.problem_1()
            DCShuntMotorProblems.problem_2()

            # Get output
            output = sys.stdout.getvalue()
        finally:
            # Restore stdout
            sys.stdout = old_stdout

        # Display in text widget
        self.problem_text.delete(1.0, tk.END)
        self.problem_text.insert(tk.END, output)

    def solve_problem_1(self):
        """Solve and display problem 1"""
        import sys
        from io import StringIO

        old_stdout = sys.stdout
        sys.stdout = StringIO()

        try:
            DCShuntMotorProblems.problem_1()
            output = sys.stdout.getvalue()
        finally:
            sys.stdout = old_stdout

        self.problem_text.delete(1.0, tk.END)
        self.problem_text.insert(tk.END, output)

    def solve_problem_2(self):
        """Solve and display problem 2"""
        import sys
        from io import StringIO

        old_stdout = sys.stdout
        sys.stdout = StringIO()

        try:
            DCShuntMotorProblems.problem_2()
            output = sys.stdout.getvalue()
        finally:
            sys.stdout = old_stdout

        self.problem_text.delete(1.0, tk.END)
        self.problem_text.insert(tk.END, output)

    def update_motor_parameters(self):
        """Update motor model with parameters from GUI"""
        params = {key: var.get() for key, var in self.params.items()}
        self.motor.update_parameters(params)

    def start_simulation(self):
        """Start real-time simulation"""
        self.is_running = True
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')

        # Update motor parameters
        self.update_motor_parameters()

        # Start animation
        self.animate_simulation()

    def stop_simulation(self):
        """Stop simulation"""
        self.is_running = False
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.stop_simulation()

        # Reset motor state
        self.motor.state = {
            'Ia': 0.0,
            'If': 0.0,
            'omega': 0.0,
            'theta': 0.0,
            'temp_a': 25.0,
            'temp_f': 25.0,
        }

        # Clear history
        self.simulation_time = 0.0
        self.time_history = []
        for key in self.state_history:
            self.state_history[key] = []

        # Clear plots
        self.clear_plots()

        # Update status
        self.update_status_display()

    def animate_simulation(self):
        """Animate real-time simulation"""
        if not self.is_running:
            return

        # Time step
        dt = 0.01  # 10ms

        # Update motor parameters
        self.update_motor_parameters()

        # Simulate one step
        t_span = (self.simulation_time, self.simulation_time + dt)
        y0 = [
            self.motor.state['Ia'],
            self.motor.state['If'],
            self.motor.state['omega'],
            self.motor.state['theta'],
            self.motor.state['temp_a'],
            self.motor.state['temp_f']
        ]

        sol = self.motor.simulate_transient(t_span, method=self.solver_var.get(), initial_state=y0)

        # Update state
        if sol.success:
            self.motor.state['Ia'] = sol.y[0, -1]
            self.motor.state['If'] = sol.y[1, -1]
            self.motor.state['omega'] = sol.y[2, -1]
            self.motor.state['theta'] = sol.y[3, -1]
            self.motor.state['temp_a'] = sol.y[4, -1]
            self.motor.state['temp_f'] = sol.y[5, -1]

            # Calculate derived quantities
            Ia = self.motor.state['Ia']
            If = self.motor.state['If']
            omega = self.motor.state['omega']

            torque = self.motor.Kt * If * self.motor.flux_weakening * Ia
            Eb = self.motor.Ke * If * self.motor.flux_weakening * omega
            power = Eb * Ia
            efficiency, _, _ = self.motor.calculate_efficiency(Ia, If, omega)
            losses = self.motor.calculate_losses(Ia, If, omega)

            # Store history
            self.time_history.append(self.simulation_time)
            self.state_history['Ia'].append(Ia)
            self.state_history['If'].append(If)
            self.state_history['omega'].append(omega * 60 / (2 * np.pi))  # Convert to rpm
            self.state_history['torque'].append(torque)
            self.state_history['power'].append(power)
            self.state_history['efficiency'].append(efficiency)
            self.state_history['temp_a'].append(self.motor.state['temp_a'])
            self.state_history['temp_f'].append(self.motor.state['temp_f'])
            self.state_history['losses'].append(losses['total'])

            # Update plots (every 10 steps to reduce overhead)
            if len(self.time_history) % 10 == 0:
                self.update_plots()

            # Update status display
            self.update_status_display()

            # Increment time
            self.simulation_time += dt

        # Schedule next update
        self.root.after(10, self.animate_simulation)

    def run_simulation(self):
        """Run complete simulation and analysis"""
        self.update_motor_parameters()

        # Run transient simulation
        t_span = (0, 5.0)  # 5 seconds
        sol = self.motor.simulate_transient(t_span, method=self.solver_var.get())

        if sol.success:
            # Store results
            self.time_history = sol.t.tolist()
            self.state_history['Ia'] = sol.y[0, :].tolist()
            self.state_history['If'] = sol.y[1, :].tolist()
            omega_history = sol.y[2, :]
            self.state_history['omega'] = (omega_history * 60 / (2 * np.pi)).tolist()
            self.state_history['temp_a'] = sol.y[4, :].tolist()
            self.state_history['temp_f'] = sol.y[5, :].tolist()

            # Calculate derived quantities
            torque_history = []
            power_history = []
            efficiency_history = []
            losses_history = []

            for i in range(len(sol.t)):
                Ia = sol.y[0, i]
                If = sol.y[1, i]
                omega = sol.y[2, i]

                torque = self.motor.Kt * If * self.motor.flux_weakening * Ia
                Eb = self.motor.Ke * If * self.motor.flux_weakening * omega
                power = Eb * Ia
                efficiency, _, _ = self.motor.calculate_efficiency(Ia, If, omega)
                losses = self.motor.calculate_losses(Ia, If, omega)

                torque_history.append(torque)
                power_history.append(power)
                efficiency_history.append(efficiency)
                losses_history.append(losses['total'])

            self.state_history['torque'] = torque_history
            self.state_history['power'] = power_history
            self.state_history['efficiency'] = efficiency_history
            self.state_history['losses'] = losses_history

            # Update all plots
            self.update_plots()
            self.update_analysis_plots()
            self.update_thermal_plots()

            # Update status with final values
            self.motor.state['Ia'] = sol.y[0, -1]
            self.motor.state['If'] = sol.y[1, -1]
            self.motor.state['omega'] = sol.y[2, -1]
            self.motor.state['temp_a'] = sol.y[4, -1]
            self.motor.state['temp_f'] = sol.y[5, -1]
            self.update_status_display()

            messagebox.showinfo("Simulation Complete",
                              f"Simulation completed successfully!\n"
                              f"Final Speed: {self.state_history['omega'][-1]:.2f} rpm\n"
                              f"Final Efficiency: {self.state_history['efficiency'][-1]:.2f}%")

    def update_plots(self):
        """Update simulation plots"""
        if not self.time_history:
            return

        t = self.time_history

        # Clear axes
        self.ax_current.clear()
        self.ax_speed.clear()
        self.ax_torque.clear()
        self.ax_power.clear()
        self.ax_temp.clear()
        self.ax_efficiency.clear()

        # Plot current
        self.ax_current.plot(t, self.state_history['Ia'], 'b-', label='Ia', linewidth=2)
        self.ax_current.plot(t, self.state_history['If'], 'r-', label='If', linewidth=2)
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Armature & Field Current')
        self.ax_current.legend()
        self.ax_current.grid(True, alpha=0.3)

        # Plot speed
        self.ax_speed.plot(t, self.state_history['omega'], 'g-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.set_title('Motor Speed')
        self.ax_speed.grid(True, alpha=0.3)

        # Plot torque
        self.ax_torque.plot(t, self.state_history['torque'], 'm-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.grid(True, alpha=0.3)

        # Plot power
        self.ax_power.plot(t, self.state_history['power'], 'c-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.set_title('Output Power')
        self.ax_power.grid(True, alpha=0.3)

        # Plot temperature
        self.ax_temp.plot(t, self.state_history['temp_a'], 'r-', label='Armature', linewidth=2)
        self.ax_temp.plot(t, self.state_history['temp_f'], 'b-', label='Field', linewidth=2)
        self.ax_temp.axhline(y=75, color='orange', linestyle='--', label='Rated')
        self.ax_temp.axhline(y=130, color='red', linestyle='--', label='Max')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Temperature Rise')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        # Plot efficiency
        self.ax_efficiency.plot(t, self.state_history['efficiency'], 'k-', linewidth=2)
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('Motor Efficiency')
        self.ax_efficiency.grid(True, alpha=0.3)

        self.fig_sim.tight_layout()
        self.canvas_sim.draw()

    def update_analysis_plots(self):
        """Update loss analysis plots"""
        if not self.time_history:
            return

        # Get final state for steady-state analysis
        Ia = self.state_history['Ia'][-1]
        If = self.state_history['If'][-1]
        omega = self.state_history['omega'][-1] * 2 * np.pi / 60  # Convert to rad/s

        losses = self.motor.calculate_losses(Ia, If, omega)

        # Clear axes
        self.ax_pie.clear()
        self.ax_losses.clear()
        self.ax_eff_map.clear()
        self.ax_power_flow.clear()

        # Loss distribution pie chart
        loss_labels = ['Copper (Armature)', 'Copper (Field)', 'Iron', 'Mechanical', 'Stray']
        loss_values = [
            losses['copper_armature'],
            losses['copper_field'],
            losses['iron'],
            losses['mechanical'],
            losses['stray']
        ]
        colors = ['#ff9999', '#ff6666', '#66b3ff', '#99ff99', '#ffcc99']

        self.ax_pie.pie(loss_values, labels=loss_labels, autopct='%1.1f%%',
                       colors=colors, startangle=90)
        self.ax_pie.set_title('Loss Distribution')

        # Losses over time
        t = self.time_history
        self.ax_losses.plot(t, self.state_history['losses'], 'r-', linewidth=2)
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Total Losses (W)')
        self.ax_losses.set_title('Losses Over Time')
        self.ax_losses.grid(True, alpha=0.3)

        # Efficiency map (speed vs torque)
        speed_range = np.linspace(100, 3000, 20)
        torque_range = np.linspace(1, 50, 20)
        eff_map = np.zeros((len(torque_range), len(speed_range)))

        for i, T in enumerate(torque_range):
            for j, N in enumerate(speed_range):
                omega_map = N * 2 * np.pi / 60
                Ia_map = T / (self.motor.Kt * If * self.motor.flux_weakening)
                eff, _, _ = self.motor.calculate_efficiency(Ia_map, If, omega_map)
                eff_map[i, j] = eff

        contour = self.ax_eff_map.contourf(speed_range, torque_range, eff_map,
                                           levels=15, cmap='RdYlGn')
        self.ax_eff_map.set_xlabel('Speed (rpm)')
        self.ax_eff_map.set_ylabel('Torque (N·m)')
        self.ax_eff_map.set_title('Efficiency Map (%)')
        self.fig_analysis.colorbar(contour, ax=self.ax_eff_map)

        # Power flow
        Eb = self.motor.Ke * If * self.motor.flux_weakening * omega
        P_input = self.motor.V_supply * (Ia + If)
        P_output = Eb * Ia

        categories = ['Input', 'Output', 'Losses']
        values = [P_input, P_output, losses['total']]
        colors_bar = ['green', 'blue', 'red']

        bars = self.ax_power_flow.bar(categories, values, color=colors_bar, alpha=0.7)
        self.ax_power_flow.set_ylabel('Power (W)')
        self.ax_power_flow.set_title('Power Flow')

        # Add value labels on bars
        for bar, val in zip(bars, values):
            height = bar.get_height()
            self.ax_power_flow.text(bar.get_x() + bar.get_width()/2., height,
                                   f'{val:.1f}W',
                                   ha='center', va='bottom')

        self.fig_analysis.tight_layout()
        self.canvas_analysis.draw()

    def update_thermal_plots(self):
        """Update thermal analysis plots"""
        if not self.time_history:
            return

        t = self.time_history

        # Clear axes (except derating curve which is static)
        self.ax_temp_time.clear()
        self.ax_temp_dist.clear()
        self.ax_thermal_network.clear()

        # Temperature vs time
        self.ax_temp_time.plot(t, self.state_history['temp_a'], 'r-', label='Armature', linewidth=2)
        self.ax_temp_time.plot(t, self.state_history['temp_f'], 'b-', label='Field', linewidth=2)
        self.ax_temp_time.axhline(y=75, color='orange', linestyle='--', label='Rated', alpha=0.5)
        self.ax_temp_time.axhline(y=130, color='red', linestyle='--', label='Max', alpha=0.5)
        self.ax_temp_time.set_xlabel('Time (s)')
        self.ax_temp_time.set_ylabel('Temperature (°C)')
        self.ax_temp_time.set_title('Temperature Transient')
        self.ax_temp_time.legend()
        self.ax_temp_time.grid(True, alpha=0.3)

        # Temperature distribution (bar chart)
        components = ['Armature', 'Field', 'Ambient']
        temps = [
            self.state_history['temp_a'][-1],
            self.state_history['temp_f'][-1],
            self.motor.ambient_temp
        ]
        colors_temp = ['red', 'blue', 'green']

        bars = self.ax_temp_dist.bar(components, temps, color=colors_temp, alpha=0.7)
        self.ax_temp_dist.set_ylabel('Temperature (°C)')
        self.ax_temp_dist.set_title('Temperature Distribution')
        self.ax_temp_dist.axhline(y=75, color='orange', linestyle='--', alpha=0.5)
        self.ax_temp_dist.axhline(y=130, color='red', linestyle='--', alpha=0.5)

        # Add value labels
        for bar, temp in zip(bars, temps):
            height = bar.get_height()
            self.ax_temp_dist.text(bar.get_x() + bar.get_width()/2., height,
                                  f'{temp:.1f}°C',
                                  ha='center', va='bottom')

        # Thermal network diagram (simple text representation)
        self.ax_thermal_network.text(0.5, 0.9, 'Thermal Resistance Network',
                                    ha='center', fontsize=12, fontweight='bold')

        network_text = f"""
        Armature:
          Thermal Resistance: {self.motor.thermal_resistance_a:.2f} °C/W
          Thermal Capacitance: {self.motor.thermal_capacitance_a:.1f} J/°C

        Field:
          Thermal Resistance: {self.motor.thermal_resistance_f:.2f} °C/W
          Thermal Capacitance: {self.motor.thermal_capacitance_f:.1f} J/°C

        Ambient Temperature: {self.motor.ambient_temp:.1f} °C

        Derating Factor: {self.motor.calculate_derating_factor():.2f}
        """

        self.ax_thermal_network.text(0.1, 0.5, network_text,
                                    ha='left', va='center', fontsize=10,
                                    family='monospace')
        self.ax_thermal_network.axis('off')

        self.fig_thermal.tight_layout()
        self.canvas_thermal.draw()

    def update_status_display(self):
        """Update real-time status display"""
        Ia = self.motor.state['Ia']
        If = self.motor.state['If']
        omega = self.motor.state['omega']

        # Calculate derived quantities
        speed_rpm = omega * 60 / (2 * np.pi)
        torque = self.motor.Kt * If * self.motor.flux_weakening * Ia
        Eb = self.motor.Ke * If * self.motor.flux_weakening * omega
        power = Eb * Ia
        efficiency, _, _ = self.motor.calculate_efficiency(Ia, If, omega)

        # Update labels
        self.status_labels['Ia'].config(text=f"{Ia:.2f}")
        self.status_labels['If'].config(text=f"{If:.3f}")
        self.status_labels['Speed'].config(text=f"{speed_rpm:.2f}")
        self.status_labels['Torque'].config(text=f"{torque:.2f}")
        self.status_labels['Power'].config(text=f"{power:.2f}")
        self.status_labels['Efficiency'].config(text=f"{efficiency:.2f}")
        self.status_labels['Temp_a'].config(text=f"{self.motor.state['temp_a']:.1f}")
        self.status_labels['Temp_f'].config(text=f"{self.motor.state['temp_f']:.1f}")

    def clear_plots(self):
        """Clear all plots"""
        # Clear simulation plots
        for ax in [self.ax_current, self.ax_speed, self.ax_torque,
                  self.ax_power, self.ax_temp, self.ax_efficiency]:
            ax.clear()
        self.canvas_sim.draw()

        # Clear analysis plots
        self.ax_pie.clear()
        self.ax_losses.clear()
        self.ax_eff_map.clear()
        self.ax_power_flow.clear()
        self.canvas_analysis.draw()

    def calculate_economics(self):
        """Calculate economic analysis"""
        # Get parameters
        elec_cost = self.economic_params['electricity_cost'].get()
        op_hours = self.economic_params['operating_hours'].get()
        op_days = self.economic_params['operating_days'].get()
        motor_cost = self.economic_params['motor_cost'].get()
        maint_cost = self.economic_params['maintenance_cost'].get()
        lifetime = self.economic_params['lifetime'].get()
        discount_rate = self.economic_params['discount_rate'].get() / 100

        # Assume average power consumption (or use from simulation)
        if self.state_history['power']:
            avg_power = np.mean(self.state_history['power'])
        else:
            avg_power = 5000  # Default 5kW

        # Annual energy consumption (kWh)
        annual_energy = (avg_power / 1000) * op_hours * op_days

        # Annual electricity cost
        annual_elec_cost = annual_energy * elec_cost

        # Total annual cost
        annual_total_cost = annual_elec_cost + maint_cost

        # Present value of operating costs
        pv_operating = 0
        for year in range(1, int(lifetime) + 1):
            pv_operating += annual_total_cost / ((1 + discount_rate) ** year)

        # Life cycle cost
        life_cycle_cost = motor_cost + pv_operating

        # Cost per operating hour
        total_operating_hours = op_hours * op_days * lifetime
        cost_per_hour = life_cycle_cost / total_operating_hours

        # Format output
        output = f"""
{'='*70}
ECONOMIC ANALYSIS REPORT
{'='*70}

OPERATING PARAMETERS:
  Operating Hours per Day:     {op_hours:.1f} hours
  Operating Days per Year:      {op_days:.0f} days
  Expected Lifetime:            {lifetime:.0f} years
  Total Operating Hours:        {total_operating_hours:.0f} hours

ENERGY CONSUMPTION:
  Average Power Consumption:    {avg_power:.2f} W ({avg_power/1000:.2f} kW)
  Annual Energy Consumption:    {annual_energy:.2f} kWh
  Lifetime Energy Consumption:  {annual_energy * lifetime:.2f} kWh

COST ANALYSIS:
  Electricity Cost:             ${elec_cost:.4f} per kWh
  Annual Electricity Cost:      ${annual_elec_cost:.2f}
  Annual Maintenance Cost:      ${maint_cost:.2f}
  Total Annual Operating Cost:  ${annual_total_cost:.2f}

LIFE CYCLE COST (Discount Rate: {discount_rate*100:.1f}%):
  Initial Motor Cost:           ${motor_cost:.2f}
  PV of Operating Costs:        ${pv_operating:.2f}
  Total Life Cycle Cost:        ${life_cycle_cost:.2f}

UNIT COSTS:
  Cost per Operating Hour:      ${cost_per_hour:.4f}
  Cost per kWh Delivered:       ${life_cycle_cost / (annual_energy * lifetime):.4f}

EFFICIENCY IMPACT:
  Current Efficiency:           {self.state_history['efficiency'][-1] if self.state_history['efficiency'] else 85:.2f}%
  If Efficiency Improved by 5%:
    Energy Savings (annual):    {annual_energy * 0.05:.2f} kWh
    Cost Savings (annual):      ${annual_energy * 0.05 * elec_cost:.2f}
    Cost Savings (lifetime):    ${annual_energy * 0.05 * elec_cost * lifetime:.2f}

PAYBACK ANALYSIS:
  For efficiency improvement costing $1000:
    Simple Payback Period:      {1000 / (annual_energy * 0.05 * elec_cost) if annual_energy > 0 else 0:.2f} years

{'='*70}
"""

        self.economic_text.delete(1.0, tk.END)
        self.economic_text.insert(tk.END, output)

    def save_results(self):
        """Save simulation results to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motor_simulation_{timestamp}.json"

        results = {
            'timestamp': timestamp,
            'parameters': {key: var.get() for key, var in self.params.items()},
            'time': self.time_history,
            'state_history': self.state_history
        }

        try:
            with open(filename, 'w') as f:
                json.dump(results, f, indent=2)
            messagebox.showinfo("Success", f"Results saved to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save results: {str(e)}")

    def load_parameters(self):
        """Load parameters from file"""
        # This would open a file dialog and load parameters
        messagebox.showinfo("Info", "Load parameters feature - to be implemented")

    def show_about(self):
        """Show about dialog"""
        about_text = """
Advanced DC Shunt Motor Multi-Physics Simulator
Version 1.0

Features:
• Multi-physics coupling (electromagnetic-thermal-mechanical)
• Real-time ODE simulation (RK45, Euler)
• Comprehensive loss analysis
• Economic analysis
• Thermal derating
• Problem solving capabilities

Developed for electrical engineering education and analysis.
"""
        messagebox.showinfo("About", about_text)

    def on_resize(self, event):
        """Handle window resize event"""
        # The matplotlib canvases automatically handle resize
        # This method can be extended for custom resize behavior
        pass


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    """Main entry point"""
    # First, solve the problems in console
    print("\n")
    print("="*80)
    print(" DC SHUNT MOTOR ANALYTICAL PROBLEM SOLUTIONS")
    print("="*80)

    # Solve problems
    DCShuntMotorProblems.problem_1()
    DCShuntMotorProblems.problem_2()

    print("\n")
    print("="*80)
    print(" LAUNCHING ADVANCED GUI SIMULATOR")
    print("="*80)
    print("\n")

    # Create and run GUI
    root = tk.Tk()
    app = AdvancedDCMotorSimulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
