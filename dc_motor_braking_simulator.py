"""
Advanced DC Motor Braking Multi-Physics Simulator
Comprehensive simulation with electromagnetic-thermal-mechanical coupling
Author: Advanced Electrical Engineering Simulator
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from scipy.integrate import odeint, solve_ivp
from datetime import datetime
import threading
import time

# ============================================================================
# CONSTANTS AND PHYSICAL PARAMETERS
# ============================================================================

class PhysicalConstants:
    """Physical constants for multi-physics simulation"""
    # Thermal constants
    STEFAN_BOLTZMANN = 5.67e-8  # W/(m²·K⁴)
    AIR_THERMAL_CONDUCTIVITY = 0.026  # W/(m·K)
    COPPER_THERMAL_CONDUCTIVITY = 385  # W/(m·K)
    IRON_THERMAL_CONDUCTIVITY = 80  # W/(m·K)
    COPPER_SPECIFIC_HEAT = 385  # J/(kg·K)
    IRON_SPECIFIC_HEAT = 450  # J/(kg·K)
    AMBIENT_TEMP = 298.15  # K (25°C)

    # Mechanical constants
    STEEL_DENSITY = 7850  # kg/m³
    COPPER_DENSITY = 8960  # kg/m³
    STEEL_YOUNGS_MODULUS = 200e9  # Pa
    POISSON_RATIO = 0.3

    # Electrical constants
    COPPER_RESISTIVITY_20C = 1.68e-8  # Ω·m
    TEMP_COEFF_COPPER = 0.00393  # per °C

# ============================================================================
# MATHEMATICAL MODELS - MULTI-PHYSICS EQUATIONS
# ============================================================================

class DCMotorMultiPhysicsModel:
    """
    Comprehensive multi-physics model for DC motor
    Couples electromagnetic, thermal, and mechanical equations
    """

    def __init__(self, params):
        self.params = params
        self.update_parameters(params)

    def update_parameters(self, params):
        """Update motor parameters"""
        self.V = params.get('voltage', 220)  # RMS voltage
        self.Ra = params.get('Ra', 0.1)  # Armature resistance
        self.La = params.get('La', 0.05)  # Armature inductance
        self.Rf = params.get('Rf', 100)  # Field resistance
        self.Lf = params.get('Lf', 10)  # Field inductance
        self.J = params.get('J', 0.5)  # Moment of inertia
        self.B = params.get('B', 0.01)  # Friction coefficient
        self.Kt = params.get('Kt', 2.0)  # Torque constant
        self.Ke = params.get('Ke', 2.0)  # Back EMF constant
        self.R_ext = params.get('R_ext', 3.21)  # External resistance for braking

        # Thermal parameters
        self.mass_armature = params.get('mass_armature', 20)  # kg
        self.mass_field = params.get('mass_field', 15)  # kg
        self.surface_area = params.get('surface_area', 1.5)  # m²
        self.convection_coeff = params.get('convection_coeff', 25)  # W/(m²·K)

        # Mechanical parameters
        self.shaft_diameter = params.get('shaft_diameter', 0.05)  # m
        self.shaft_length = params.get('shaft_length', 0.3)  # m

        # Loss parameters
        self.k_hysteresis = params.get('k_hysteresis', 0.001)
        self.k_eddy = params.get('k_eddy', 0.0005)
        self.k_friction = params.get('k_friction', 0.01)
        self.k_windage = params.get('k_windage', 0.0001)

        # Control parameters
        self.is_plugging = params.get('is_plugging', False)
        self.is_dynamic_braking = params.get('is_dynamic_braking', False)
        self.braking_resistance = params.get('braking_resistance', 10)

    def electrical_equations_rk45(self, t, y):
        """
        State equations for RK45 solver
        y = [Ia, If, omega, theta, T_armature, T_field]
        Using RMS values for voltage and current
        """
        Ia, If, omega, theta, T_arm, T_field = y

        # Update temperature-dependent resistance
        Ra_temp = self.Ra * (1 + PhysicalConstants.TEMP_COEFF_COPPER * (T_arm - 20))
        Rf_temp = self.Rf * (1 + PhysicalConstants.TEMP_COEFF_COPPER * (T_field - 20))

        # Back EMF (RMS value)
        Eb = self.Ke * If * omega

        # Voltage equation considering braking mode
        if self.is_plugging:
            # Plugging: reverse armature connections
            V_applied = -self.V
            R_total = Ra_temp + self.R_ext
        elif self.is_dynamic_braking:
            # Dynamic braking: disconnect supply, short through resistance
            V_applied = 0
            R_total = Ra_temp + self.braking_resistance
        else:
            # Normal operation
            V_applied = self.V
            R_total = Ra_temp

        # Armature circuit equation (RMS)
        dIa_dt = (V_applied - Eb - Ia * R_total) / self.La

        # Field circuit equation (RMS)
        dIf_dt = (self.V - If * Rf_temp) / self.Lf

        # Electromagnetic torque
        T_em = self.Kt * If * Ia

        # Load torque (can be modified for different loads)
        T_load = 0  # No load during braking

        # Detailed loss calculations
        losses = self.calculate_losses(Ia, If, omega, T_arm, T_field)

        # Mechanical equation
        domega_dt = (T_em - T_load - self.B * omega -
                     losses['friction_torque'] - losses['windage_torque']) / self.J

        # Angular position
        dtheta_dt = omega

        # Thermal equations (coupled)
        # Armature temperature
        P_copper_arm = Ia**2 * Ra_temp  # Copper losses (using RMS current)
        P_iron_arm = losses['hysteresis'] + losses['eddy_current']
        Q_conv_arm = self.convection_coeff * self.surface_area * 0.6 * (T_arm - PhysicalConstants.AMBIENT_TEMP)
        Q_rad_arm = (PhysicalConstants.STEFAN_BOLTZMANN * self.surface_area * 0.6 *
                     (T_arm**4 - PhysicalConstants.AMBIENT_TEMP**4))

        dT_arm_dt = (P_copper_arm + P_iron_arm - Q_conv_arm - Q_rad_arm) / (
            self.mass_armature * PhysicalConstants.COPPER_SPECIFIC_HEAT)

        # Field temperature
        P_copper_field = If**2 * Rf_temp  # Field copper losses (using RMS current)
        Q_conv_field = self.convection_coeff * self.surface_area * 0.4 * (T_field - PhysicalConstants.AMBIENT_TEMP)
        Q_rad_field = (PhysicalConstants.STEFAN_BOLTZMANN * self.surface_area * 0.4 *
                       (T_field**4 - PhysicalConstants.AMBIENT_TEMP**4))

        dT_field_dt = (P_copper_field - Q_conv_field - Q_rad_field) / (
            self.mass_field * PhysicalConstants.IRON_SPECIFIC_HEAT)

        return [dIa_dt, dIf_dt, domega_dt, dtheta_dt, dT_arm_dt, dT_field_dt]

    def electrical_equations_euler(self, y, t, dt):
        """
        Euler method implementation
        y = [Ia, If, omega, theta, T_armature, T_field]
        """
        derivatives = self.electrical_equations_rk45(t, y)
        y_new = y + np.array(derivatives) * dt
        return y_new

    def calculate_losses(self, Ia, If, omega, T_arm, T_field):
        """
        Detailed loss breakdown
        Returns dictionary with all loss components
        """
        losses = {}

        # Copper losses (I²R) - using RMS values
        Ra_temp = self.Ra * (1 + PhysicalConstants.TEMP_COEFF_COPPER * (T_arm - 20))
        Rf_temp = self.Rf * (1 + PhysicalConstants.TEMP_COEFF_COPPER * (T_field - 20))

        losses['copper_armature'] = Ia**2 * Ra_temp
        losses['copper_field'] = If**2 * Rf_temp
        losses['copper_total'] = losses['copper_armature'] + losses['copper_field']

        # Iron losses (core losses)
        f = abs(omega) / (2 * np.pi)  # Frequency in Hz
        B = If * 0.1  # Simplified flux density

        # Hysteresis loss: P_h = k_h * f * B^1.6
        losses['hysteresis'] = self.k_hysteresis * f * (B**1.6) if f > 0 else 0

        # Eddy current loss: P_e = k_e * f^2 * B^2
        losses['eddy_current'] = self.k_eddy * (f**2) * (B**2)

        losses['iron_total'] = losses['hysteresis'] + losses['eddy_current']

        # Mechanical losses
        # Friction loss: proportional to speed
        losses['friction'] = self.k_friction * abs(omega)
        losses['friction_torque'] = self.k_friction

        # Windage loss: proportional to speed^3
        losses['windage'] = self.k_windage * (omega**3) if omega > 0 else self.k_windage * ((-omega)**3)
        losses['windage_torque'] = self.k_windage * (omega**2) if omega > 0 else 0

        losses['mechanical_total'] = losses['friction'] + abs(losses['windage'])

        # Stray load losses (approximately 1% of output power)
        P_out = self.Kt * If * Ia * omega
        losses['stray_load'] = 0.01 * abs(P_out)

        # Total losses
        losses['total'] = (losses['copper_total'] + losses['iron_total'] +
                          losses['mechanical_total'] + losses['stray_load'])

        return losses

    def calculate_mechanical_stress(self, omega, T_em):
        """
        Calculate mechanical stress on shaft
        Returns shaft torque, shear stress, and bearing loads
        """
        stress = {}

        # Shaft torque
        stress['torque'] = T_em

        # Shear stress: τ = (16 * T) / (π * d³)
        d = self.shaft_diameter
        stress['shear_stress'] = (16 * abs(T_em)) / (np.pi * d**3)

        # Angular acceleration (approximate)
        alpha = T_em / self.J
        stress['angular_acceleration'] = alpha

        # Centrifugal force (simplified for rotating mass)
        # F_c = m * r * ω²
        r_avg = d / 2
        stress['centrifugal_force'] = self.mass_armature * r_avg * (omega**2)

        # Bearing load (simplified - radial load from weight + dynamic load)
        weight = self.mass_armature * 9.81
        dynamic_factor = 1 + 0.1 * abs(omega) / (2 * np.pi)  # increases with speed
        stress['bearing_load'] = weight * dynamic_factor

        # Shaft deflection (simplified beam equation)
        # δ = (F * L³) / (3 * E * I)
        I = np.pi * (d**4) / 64  # Second moment of area
        E = PhysicalConstants.STEEL_YOUNGS_MODULUS
        L = self.shaft_length
        F = stress['bearing_load']
        stress['shaft_deflection'] = (F * L**3) / (3 * E * I)

        return stress

    def calculate_efficiency(self, Ia, If, omega, losses):
        """Calculate motor efficiency"""
        P_in = self.V * Ia  # Input power (RMS values)
        P_out = self.Kt * If * Ia * omega  # Output power
        P_loss = losses['total']

        if P_in > 0.01:
            efficiency = (P_out / P_in) * 100
        else:
            efficiency = 0

        return max(0, min(100, efficiency))

    def calculate_derating_factor(self, T_arm, T_field, altitude=0):
        """
        Calculate derating factor based on temperature and altitude
        """
        # Temperature derating
        T_rated = 40  # °C rated ambient temperature
        T_max = 120  # °C maximum winding temperature
        T_avg = (T_arm + T_field) / 2

        if T_avg < T_rated:
            temp_derating = 1.0
        elif T_avg < T_max:
            temp_derating = 1.0 - 0.01 * (T_avg - T_rated)
        else:
            temp_derating = 0.5  # Severe derating at overtemperature

        # Altitude derating (reduced cooling at high altitude)
        # 1% reduction per 100m above 1000m
        if altitude > 1000:
            altitude_derating = 1.0 - 0.01 * ((altitude - 1000) / 100)
        else:
            altitude_derating = 1.0

        total_derating = temp_derating * altitude_derating
        return max(0.5, min(1.0, total_derating))

# ============================================================================
# SIMULATION ENGINE
# ============================================================================

class SimulationEngine:
    """Handles simulation execution with different ODE solvers"""

    def __init__(self, model):
        self.model = model
        self.results = None
        self.is_running = False
        self.solver_type = 'RK45'

    def run_simulation(self, t_span, y0, solver='RK45', max_step=0.001):
        """
        Run simulation with specified solver

        Parameters:
        - t_span: (t_start, t_end)
        - y0: initial conditions [Ia, If, omega, theta, T_arm, T_field]
        - solver: 'RK45' or 'Euler'
        - max_step: maximum time step
        """
        self.solver_type = solver
        t_start, t_end = t_span

        if solver == 'RK45':
            # Use scipy's RK45 solver
            sol = solve_ivp(
                self.model.electrical_equations_rk45,
                t_span,
                y0,
                method='RK45',
                max_step=max_step,
                dense_output=True,
                vectorized=False
            )

            # Create uniform time array for plotting
            t = np.linspace(t_start, t_end, 1000)
            y = sol.sol(t).T

        elif solver == 'Euler':
            # Manual Euler integration
            dt = max_step
            t = np.arange(t_start, t_end, dt)
            y = np.zeros((len(t), len(y0)))
            y[0] = y0

            for i in range(1, len(t)):
                y[i] = self.model.electrical_equations_euler(y[i-1], t[i-1], dt)

        else:
            raise ValueError(f"Unknown solver: {solver}")

        # Calculate additional results
        self.process_results(t, y)
        return t, y

    def process_results(self, t, y):
        """Process simulation results and calculate derived quantities"""
        self.results = {
            'time': t,
            'Ia': y[:, 0],  # RMS armature current
            'If': y[:, 1],  # RMS field current
            'omega': y[:, 2],  # Angular velocity
            'theta': y[:, 3],  # Angular position
            'T_armature': y[:, 4],  # Armature temperature
            'T_field': y[:, 5],  # Field temperature
        }

        # Calculate derived quantities
        n = len(t)
        self.results['speed_rpm'] = y[:, 2] * 60 / (2 * np.pi)
        self.results['Eb'] = self.model.Ke * y[:, 1] * y[:, 2]
        self.results['torque'] = self.model.Kt * y[:, 1] * y[:, 0]

        # Power calculations (using RMS values)
        self.results['P_input'] = self.model.V * y[:, 0]
        self.results['P_output'] = self.results['torque'] * y[:, 2]

        # Loss calculations for each time step
        losses_array = {
            'copper_total': np.zeros(n),
            'iron_total': np.zeros(n),
            'mechanical_total': np.zeros(n),
            'stray_load': np.zeros(n),
            'total': np.zeros(n),
            'copper_armature': np.zeros(n),
            'copper_field': np.zeros(n),
            'hysteresis': np.zeros(n),
            'eddy_current': np.zeros(n),
            'friction': np.zeros(n),
            'windage': np.zeros(n),
        }

        efficiency = np.zeros(n)
        shear_stress = np.zeros(n)
        bearing_load = np.zeros(n)
        derating_factor = np.zeros(n)

        for i in range(n):
            losses = self.model.calculate_losses(
                y[i, 0], y[i, 1], y[i, 2], y[i, 4], y[i, 5]
            )
            for key in losses_array:
                losses_array[key][i] = losses[key]

            efficiency[i] = self.model.calculate_efficiency(
                y[i, 0], y[i, 1], y[i, 2], losses
            )

            stress = self.model.calculate_mechanical_stress(y[i, 2], self.results['torque'][i])
            shear_stress[i] = stress['shear_stress']
            bearing_load[i] = stress['bearing_load']

            derating_factor[i] = self.model.calculate_derating_factor(y[i, 4], y[i, 5])

        self.results['losses'] = losses_array
        self.results['efficiency'] = efficiency
        self.results['shear_stress'] = shear_stress
        self.results['bearing_load'] = bearing_load
        self.results['derating_factor'] = derating_factor

# ============================================================================
# THEORETICAL PROBLEM SOLVER
# ============================================================================

class TheoreticalSolver:
    """Solves the specific braking problem"""

    @staticmethod
    def solve_braking_problem():
        """
        Solve: 18.65 kW, 220V DC shunt motor at 600 rpm
        Ra = 0.1 Ω, Ia_fl = 95 A
        Find: R for plugging with I_limit = 130 A
        Find: Initial braking torque and torque at half speed
        """
        results = {}

        # Given parameters
        P_rated = 18650  # W
        V = 220  # V (RMS)
        N_fl = 600  # rpm
        Ra = 0.1  # Ω
        Ia_fl = 95  # A (RMS)
        I_limit = 130  # A (RMS)

        # Calculate back EMF at full load
        Eb_fl = V - Ia_fl * Ra
        omega_fl = 2 * np.pi * N_fl / 60

        results['Eb_full_load'] = Eb_fl
        results['omega_full_load'] = omega_fl

        # Calculate EMF constant
        # Assuming field current at rated voltage
        If = V / 100  # Assuming Rf = 100 Ω
        Ke = Eb_fl / (If * omega_fl)

        # During plugging: V + Eb = Ia * (Ra + R)
        # At initial instant (speed still at full load)
        R = (V + Eb_fl) / I_limit - Ra
        results['external_resistance'] = R

        # Initial braking torque
        # T = Eb * Ia / omega (power balance)
        T_initial = Eb_fl * I_limit / omega_fl
        results['torque_initial'] = T_initial

        # At half speed
        Eb_half = Eb_fl / 2
        omega_half = omega_fl / 2
        Ia_half = (V + Eb_half) / (Ra + R)
        T_half = Eb_half * Ia_half / omega_half

        results['speed_half'] = N_fl / 2
        results['Eb_half'] = Eb_half
        results['Ia_half'] = Ia_half
        results['torque_half'] = T_half

        # Energy dissipated during braking
        # Approximate using average torque
        T_avg = (T_initial + T_half) / 2
        results['avg_torque'] = T_avg

        return results

# ============================================================================
# GUI APPLICATION
# ============================================================================

class DCMotorSimulatorGUI:
    """Main GUI application with all features"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Braking Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Simulation state
        self.is_simulating = False
        self.simulation_thread = None
        self.model = None
        self.engine = None

        # Default parameters
        self.params = {
            'voltage': 220,
            'Ra': 0.1,
            'La': 0.05,
            'Rf': 100,
            'Lf': 10,
            'J': 0.5,
            'B': 0.01,
            'Kt': 2.0,
            'Ke': 2.0,
            'R_ext': 3.21,
            'mass_armature': 20,
            'mass_field': 15,
            'surface_area': 1.5,
            'convection_coeff': 25,
            'shaft_diameter': 0.05,
            'shaft_length': 0.3,
            'k_hysteresis': 0.001,
            'k_eddy': 0.0005,
            'k_friction': 0.01,
            'k_windage': 0.0001,
            'is_plugging': True,
            'is_dynamic_braking': False,
            'braking_resistance': 10,
            'initial_speed': 600,  # rpm
            'initial_current': 95,  # A
            'sim_time': 5.0,
            'solver': 'RK45',
            'altitude': 0,
        }

        # Initialize model
        self.model = DCMotorMultiPhysicsModel(self.params)
        self.engine = SimulationEngine(self.model)

        # Create GUI
        self.create_widgets()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def create_widgets(self):
        """Create all GUI widgets"""

        # Main container with tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create tabs
        self.create_main_tab()
        self.create_parameters_tab()
        self.create_control_tab()
        self.create_visualization_tab()
        self.create_economic_tab()
        self.create_theoretical_tab()

        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def create_main_tab(self):
        """Main tab with overview and controls"""
        main_frame = ttk.Frame(self.notebook)
        self.notebook.add(main_frame, text="Main Menu")

        # Title
        title = ttk.Label(main_frame, text="DC Motor Braking Multi-Physics Simulator",
                         font=('Arial', 16, 'bold'))
        title.pack(pady=10)

        # Description
        desc_frame = ttk.LabelFrame(main_frame, text="Description")
        desc_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        desc_text = scrolledtext.ScrolledText(desc_frame, height=8, wrap=tk.WORD)
        desc_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        desc_text.insert(tk.END, """
Advanced DC Motor Braking Simulator with Multi-Physics Coupling

Features:
• Electromagnetic-Thermal-Mechanical Coupled Simulation
• Real-time ODE Solvers (RK45, Euler)
• Detailed Loss Breakdown (Copper, Iron, Mechanical, Stray)
• Thermal Analysis with Convection and Radiation
• Mechanical Stress Analysis (Shaft Torque, Bearing Loads)
• Advanced Control Methods (Plugging, Dynamic Braking)
• Economic Analysis and Derating Calculations
• Dynamic Visualization with Auto-scaling

Braking Methods:
1. Plugging: Reverse armature connections, add external resistance
2. Dynamic Braking: Disconnect supply, dissipate energy in resistance
        """)
        desc_text.config(state=tk.DISABLED)

        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(pady=10)

        self.start_btn = ttk.Button(button_frame, text="START SIMULATION",
                                    command=self.start_simulation, width=20)
        self.start_btn.grid(row=0, column=0, padx=5)

        self.stop_btn = ttk.Button(button_frame, text="STOP",
                                   command=self.stop_simulation, width=20, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=1, padx=5)

        self.reset_btn = ttk.Button(button_frame, text="RESET",
                                    command=self.reset_simulation, width=20)
        self.reset_btn.grid(row=0, column=2, padx=5)

        # Quick results display
        results_frame = ttk.LabelFrame(main_frame, text="Quick Results")
        results_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.results_text = scrolledtext.ScrolledText(results_frame, height=10)
        self.results_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_parameters_tab(self):
        """Parameters tab with all input parameters"""
        params_frame = ttk.Frame(self.notebook)
        self.notebook.add(params_frame, text="Parameters")

        # Create scrollable canvas
        canvas = tk.Canvas(params_frame)
        scrollbar = ttk.Scrollbar(params_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Electrical parameters
        elec_frame = ttk.LabelFrame(scrollable_frame, text="Electrical Parameters")
        elec_frame.grid(row=0, column=0, padx=10, pady=5, sticky='ew')

        self.param_widgets = {}

        elec_params = [
            ('voltage', 'Supply Voltage (V)', 50, 500, 220),
            ('Ra', 'Armature Resistance (Ω)', 0.01, 2, 0.1),
            ('La', 'Armature Inductance (H)', 0.001, 0.5, 0.05),
            ('Rf', 'Field Resistance (Ω)', 10, 500, 100),
            ('Lf', 'Field Inductance (H)', 0.1, 50, 10),
            ('R_ext', 'External Resistance (Ω)', 0, 20, 3.21),
        ]

        for i, (key, label, min_val, max_val, default) in enumerate(elec_params):
            self.create_parameter_row(elec_frame, i, key, label, min_val, max_val, default)

        # Mechanical parameters
        mech_frame = ttk.LabelFrame(scrollable_frame, text="Mechanical Parameters")
        mech_frame.grid(row=1, column=0, padx=10, pady=5, sticky='ew')

        mech_params = [
            ('J', 'Moment of Inertia (kg·m²)', 0.1, 5, 0.5),
            ('B', 'Friction Coefficient (N·m·s)', 0.001, 0.1, 0.01),
            ('Kt', 'Torque Constant (N·m/A)', 0.5, 10, 2.0),
            ('Ke', 'Back EMF Constant (V·s)', 0.5, 10, 2.0),
            ('shaft_diameter', 'Shaft Diameter (m)', 0.01, 0.2, 0.05),
            ('shaft_length', 'Shaft Length (m)', 0.1, 1.0, 0.3),
        ]

        for i, (key, label, min_val, max_val, default) in enumerate(mech_params):
            self.create_parameter_row(mech_frame, i, key, label, min_val, max_val, default)

        # Thermal parameters
        thermal_frame = ttk.LabelFrame(scrollable_frame, text="Thermal Parameters")
        thermal_frame.grid(row=2, column=0, padx=10, pady=5, sticky='ew')

        thermal_params = [
            ('mass_armature', 'Armature Mass (kg)', 5, 100, 20),
            ('mass_field', 'Field Mass (kg)', 5, 50, 15),
            ('surface_area', 'Surface Area (m²)', 0.5, 5, 1.5),
            ('convection_coeff', 'Convection Coeff (W/m²·K)', 5, 100, 25),
        ]

        for i, (key, label, min_val, max_val, default) in enumerate(thermal_params):
            self.create_parameter_row(thermal_frame, i, key, label, min_val, max_val, default)

        # Loss parameters
        loss_frame = ttk.LabelFrame(scrollable_frame, text="Loss Parameters")
        loss_frame.grid(row=3, column=0, padx=10, pady=5, sticky='ew')

        loss_params = [
            ('k_hysteresis', 'Hysteresis Coeff', 0.0001, 0.01, 0.001),
            ('k_eddy', 'Eddy Current Coeff', 0.0001, 0.01, 0.0005),
            ('k_friction', 'Friction Coeff', 0.001, 0.1, 0.01),
            ('k_windage', 'Windage Coeff', 0.00001, 0.001, 0.0001),
        ]

        for i, (key, label, min_val, max_val, default) in enumerate(loss_params):
            self.create_parameter_row(loss_frame, i, key, label, min_val, max_val, default)

        # Initial conditions
        ic_frame = ttk.LabelFrame(scrollable_frame, text="Initial Conditions")
        ic_frame.grid(row=4, column=0, padx=10, pady=5, sticky='ew')

        ic_params = [
            ('initial_speed', 'Initial Speed (rpm)', 0, 3000, 600),
            ('initial_current', 'Initial Current (A)', 0, 200, 95),
        ]

        for i, (key, label, min_val, max_val, default) in enumerate(ic_params):
            self.create_parameter_row(ic_frame, i, key, label, min_val, max_val, default)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_parameter_row(self, parent, row, key, label, min_val, max_val, default):
        """Create a parameter input row with label, slider, and entry"""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', padx=5, pady=2)

        var = tk.DoubleVar(value=default)

        slider = ttk.Scale(parent, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                          variable=var, length=200)
        slider.grid(row=row, column=1, padx=5, pady=2)

        entry = ttk.Entry(parent, textvariable=var, width=10)
        entry.grid(row=row, column=2, padx=5, pady=2)

        # Update params dict when value changes
        def update_param(*args):
            self.params[key] = var.get()

        var.trace('w', update_param)

        self.param_widgets[key] = var

    def create_control_tab(self):
        """Control tab with advanced control options"""
        control_frame = ttk.Frame(self.notebook)
        self.notebook.add(control_frame, text="Control")

        # Braking method selection
        method_frame = ttk.LabelFrame(control_frame, text="Braking Method")
        method_frame.pack(fill=tk.X, padx=10, pady=5)

        self.braking_method = tk.StringVar(value='plugging')

        ttk.Radiobutton(method_frame, text="Plugging (Reverse Connection)",
                       variable=self.braking_method, value='plugging',
                       command=self.update_braking_method).pack(anchor='w', padx=10, pady=2)

        ttk.Radiobutton(method_frame, text="Dynamic Braking (Resistive)",
                       variable=self.braking_method, value='dynamic',
                       command=self.update_braking_method).pack(anchor='w', padx=10, pady=2)

        ttk.Radiobutton(method_frame, text="Regenerative Braking",
                       variable=self.braking_method, value='regenerative',
                       command=self.update_braking_method).pack(anchor='w', padx=10, pady=2)

        # Solver selection
        solver_frame = ttk.LabelFrame(control_frame, text="ODE Solver")
        solver_frame.pack(fill=tk.X, padx=10, pady=5)

        self.solver_var = tk.StringVar(value='RK45')

        ttk.Radiobutton(solver_frame, text="RK45 (Runge-Kutta 4th/5th order)",
                       variable=self.solver_var, value='RK45').pack(anchor='w', padx=10, pady=2)

        ttk.Radiobutton(solver_frame, text="Euler (First order)",
                       variable=self.solver_var, value='Euler').pack(anchor='w', padx=10, pady=2)

        # Simulation settings
        sim_frame = ttk.LabelFrame(control_frame, text="Simulation Settings")
        sim_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(sim_frame, text="Simulation Time (s):").grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.sim_time_var = tk.DoubleVar(value=5.0)
        ttk.Entry(sim_frame, textvariable=self.sim_time_var, width=10).grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(sim_frame, text="Max Time Step (s):").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.max_step_var = tk.DoubleVar(value=0.001)
        ttk.Entry(sim_frame, textvariable=self.max_step_var, width=10).grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(sim_frame, text="Altitude (m):").grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.altitude_var = tk.DoubleVar(value=0)
        ttk.Entry(sim_frame, textvariable=self.altitude_var, width=10).grid(row=2, column=1, padx=5, pady=2)

        # Advanced features
        adv_frame = ttk.LabelFrame(control_frame, text="Advanced Features")
        adv_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.thermal_enable = tk.BooleanVar(value=True)
        ttk.Checkbutton(adv_frame, text="Enable Thermal Simulation",
                       variable=self.thermal_enable).pack(anchor='w', padx=10, pady=2)

        self.mechanical_enable = tk.BooleanVar(value=True)
        ttk.Checkbutton(adv_frame, text="Enable Mechanical Stress Analysis",
                       variable=self.mechanical_enable).pack(anchor='w', padx=10, pady=2)

        self.derating_enable = tk.BooleanVar(value=True)
        ttk.Checkbutton(adv_frame, text="Apply Derating Factors",
                       variable=self.derating_enable).pack(anchor='w', padx=10, pady=2)

    def create_visualization_tab(self):
        """Visualization tab with dynamic graphs"""
        viz_frame = ttk.Frame(self.notebook)
        self.notebook.add(viz_frame, text="Visualization")

        # Create matplotlib figure with subplots
        self.fig = Figure(figsize=(12, 10), dpi=100)

        # Create subplots (4x2 grid)
        self.ax1 = self.fig.add_subplot(4, 2, 1)
        self.ax2 = self.fig.add_subplot(4, 2, 2)
        self.ax3 = self.fig.add_subplot(4, 2, 3)
        self.ax4 = self.fig.add_subplot(4, 2, 4)
        self.ax5 = self.fig.add_subplot(4, 2, 5)
        self.ax6 = self.fig.add_subplot(4, 2, 6)
        self.ax7 = self.fig.add_subplot(4, 2, 7)
        self.ax8 = self.fig.add_subplot(4, 2, 8)

        self.fig.tight_layout(pad=3.0)

        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Initialize plots
        self.initialize_plots()

    def create_economic_tab(self):
        """Economic analysis tab"""
        econ_frame = ttk.Frame(self.notebook)
        self.notebook.add(econ_frame, text="Economic Analysis")

        # Cost parameters
        cost_frame = ttk.LabelFrame(econ_frame, text="Cost Parameters")
        cost_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(cost_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(cost_frame, textvariable=self.elec_cost_var, width=10).grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Motor Cost ($):").grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.motor_cost_var = tk.DoubleVar(value=5000)
        ttk.Entry(cost_frame, textvariable=self.motor_cost_var, width=10).grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Maintenance Cost ($/year):").grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.maint_cost_var = tk.DoubleVar(value=500)
        ttk.Entry(cost_frame, textvariable=self.maint_cost_var, width=10).grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Operating Hours (h/year):").grid(row=3, column=0, sticky='w', padx=5, pady=2)
        self.op_hours_var = tk.DoubleVar(value=4000)
        ttk.Entry(cost_frame, textvariable=self.op_hours_var, width=10).grid(row=3, column=1, padx=5, pady=2)

        # Calculate button
        calc_btn = ttk.Button(cost_frame, text="Calculate Economics", command=self.calculate_economics)
        calc_btn.grid(row=4, column=0, columnspan=2, pady=10)

        # Results display
        results_frame = ttk.LabelFrame(econ_frame, text="Economic Analysis Results")
        results_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.econ_text = scrolledtext.ScrolledText(results_frame, height=20)
        self.econ_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_theoretical_tab(self):
        """Tab for theoretical problem solution"""
        theory_frame = ttk.Frame(self.notebook)
        self.notebook.add(theory_frame, text="Theoretical Solution")

        # Problem statement
        prob_frame = ttk.LabelFrame(theory_frame, text="Problem Statement")
        prob_frame.pack(fill=tk.X, padx=10, pady=5)

        prob_text = scrolledtext.ScrolledText(prob_frame, height=6, wrap=tk.WORD)
        prob_text.pack(fill=tk.X, padx=5, pady=5)
        prob_text.insert(tk.END, """
A 18.65 kW, 220-V D.C. shunt motor with a full-load speed of 600 r.p.m. is to be braked by plugging.

Given:
• Armature resistance: Ra = 0.1 Ω
• Full-load armature current: Ia = 95 A
• Braking current limit: 130 A

Find:
1. Value of external resistance to limit current to 130 A during plugging
2. Initial value of electric braking torque
3. Braking torque when speed has fallen to half of its full-load value
        """)
        prob_text.config(state=tk.DISABLED)

        # Solve button
        solve_btn = ttk.Button(theory_frame, text="SOLVE THEORETICAL PROBLEM",
                              command=self.solve_theoretical_problem, width=30)
        solve_btn.pack(pady=10)

        # Solution display
        sol_frame = ttk.LabelFrame(theory_frame, text="Solution")
        sol_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.theory_text = scrolledtext.ScrolledText(sol_frame, height=20, wrap=tk.WORD, font=('Courier', 10))
        self.theory_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def update_braking_method(self):
        """Update braking method in parameters"""
        method = self.braking_method.get()
        self.params['is_plugging'] = (method == 'plugging')
        self.params['is_dynamic_braking'] = (method == 'dynamic')

    def initialize_plots(self):
        """Initialize all plot axes"""
        self.ax1.set_title('Speed vs Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Speed (rpm)')
        self.ax1.grid(True)

        self.ax2.set_title('Armature Current vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A RMS)')
        self.ax2.grid(True)

        self.ax3.set_title('Torque vs Time')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Torque (N·m)')
        self.ax3.grid(True)

        self.ax4.set_title('Temperature vs Time')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Temperature (°C)')
        self.ax4.grid(True)

        self.ax5.set_title('Power vs Time')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Power (W)')
        self.ax5.grid(True)

        self.ax6.set_title('Loss Breakdown')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Losses (W)')
        self.ax6.grid(True)

        self.ax7.set_title('Efficiency vs Time')
        self.ax7.set_xlabel('Time (s)')
        self.ax7.set_ylabel('Efficiency (%)')
        self.ax7.grid(True)

        self.ax8.set_title('Mechanical Stress')
        self.ax8.set_xlabel('Time (s)')
        self.ax8.set_ylabel('Stress (MPa)')
        self.ax8.grid(True)

    def update_plots(self, results):
        """Update all plots with simulation results"""
        t = results['time']

        # Clear all axes
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6, self.ax7, self.ax8]:
            ax.clear()

        # Plot 1: Speed
        self.ax1.plot(t, results['speed_rpm'], 'b-', linewidth=2)
        self.ax1.set_title('Speed vs Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Speed (rpm)')
        self.ax1.grid(True, alpha=0.3)

        # Plot 2: Currents
        self.ax2.plot(t, results['Ia'], 'r-', linewidth=2, label='Armature (RMS)')
        self.ax2.plot(t, results['If'], 'g-', linewidth=2, label='Field (RMS)')
        self.ax2.set_title('Currents vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A RMS)')
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.3)

        # Plot 3: Torque
        self.ax3.plot(t, results['torque'], 'purple', linewidth=2)
        self.ax3.set_title('Electromagnetic Torque vs Time')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Torque (N·m)')
        self.ax3.grid(True, alpha=0.3)

        # Plot 4: Temperatures
        self.ax4.plot(t, results['T_armature'], 'r-', linewidth=2, label='Armature')
        self.ax4.plot(t, results['T_field'], 'b-', linewidth=2, label='Field')
        self.ax4.axhline(y=120, color='k', linestyle='--', label='Max Limit')
        self.ax4.set_title('Temperature vs Time')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Temperature (°C)')
        self.ax4.legend()
        self.ax4.grid(True, alpha=0.3)

        # Plot 5: Power
        self.ax5.plot(t, results['P_input']/1000, 'g-', linewidth=2, label='Input')
        self.ax5.plot(t, results['P_output']/1000, 'b-', linewidth=2, label='Output')
        self.ax5.set_title('Power vs Time')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Power (kW)')
        self.ax5.legend()
        self.ax5.grid(True, alpha=0.3)

        # Plot 6: Loss breakdown
        losses = results['losses']
        self.ax6.plot(t, losses['copper_total'], label='Copper', linewidth=2)
        self.ax6.plot(t, losses['iron_total'], label='Iron', linewidth=2)
        self.ax6.plot(t, losses['mechanical_total'], label='Mechanical', linewidth=2)
        self.ax6.plot(t, losses['stray_load'], label='Stray Load', linewidth=2)
        self.ax6.set_title('Loss Breakdown')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Losses (W)')
        self.ax6.legend(fontsize=8)
        self.ax6.grid(True, alpha=0.3)

        # Plot 7: Efficiency
        self.ax7.plot(t, results['efficiency'], 'orange', linewidth=2)
        self.ax7.set_title('Efficiency vs Time')
        self.ax7.set_xlabel('Time (s)')
        self.ax7.set_ylabel('Efficiency (%)')
        self.ax7.set_ylim([0, 100])
        self.ax7.grid(True, alpha=0.3)

        # Plot 8: Mechanical stress
        self.ax8.plot(t, results['shear_stress']/1e6, 'r-', linewidth=2, label='Shear Stress')
        self.ax8.plot(t, results['bearing_load'], 'b-', linewidth=2, label='Bearing Load (N)')
        self.ax8.set_title('Mechanical Stress')
        self.ax8.set_xlabel('Time (s)')
        self.ax8.set_ylabel('Stress (MPa) / Load (N)')
        self.ax8.legend()
        self.ax8.grid(True, alpha=0.3)

        self.fig.tight_layout()
        self.canvas.draw()

    def start_simulation(self):
        """Start the simulation"""
        if self.is_simulating:
            messagebox.showwarning("Warning", "Simulation already running!")
            return

        # Update parameters from widgets
        self.params['solver'] = self.solver_var.get()
        self.params['sim_time'] = self.sim_time_var.get()
        self.params['altitude'] = self.altitude_var.get()

        # Update model
        self.model.update_parameters(self.params)

        # Set initial conditions
        omega_0 = self.params['initial_speed'] * 2 * np.pi / 60  # Convert rpm to rad/s
        Ia_0 = self.params['initial_current']
        If_0 = self.params['voltage'] / self.params['Rf']
        y0 = [Ia_0, If_0, omega_0, 0, 25, 25]  # [Ia, If, omega, theta, T_arm, T_field]

        # Disable start button, enable stop button
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.is_simulating = True

        # Run simulation in separate thread
        self.simulation_thread = threading.Thread(target=self.run_simulation_thread, args=(y0,))
        self.simulation_thread.start()

    def run_simulation_thread(self, y0):
        """Run simulation in separate thread"""
        try:
            self.status_var.set(f"Simulating with {self.params['solver']} solver...")

            t_span = (0, self.params['sim_time'])
            max_step = self.max_step_var.get()

            t, y = self.engine.run_simulation(t_span, y0, solver=self.params['solver'], max_step=max_step)

            # Update plots on main thread
            self.root.after(0, self.update_plots, self.engine.results)
            self.root.after(0, self.display_results, self.engine.results)

            self.status_var.set(f"Simulation complete! ({self.params['solver']} solver)")

        except Exception as e:
            self.status_var.set(f"Simulation error: {str(e)}")
            messagebox.showerror("Simulation Error", str(e))

        finally:
            self.is_simulating = False
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)

    def stop_simulation(self):
        """Stop the simulation"""
        self.is_simulating = False
        self.status_var.set("Simulation stopped by user")

    def reset_simulation(self):
        """Reset simulation to default state"""
        # Reset parameters to default
        for key, widget in self.param_widgets.items():
            if key in self.params:
                widget.set(self.params[key])

        # Clear plots
        self.initialize_plots()
        self.canvas.draw()

        # Clear results
        self.results_text.delete(1.0, tk.END)

        self.status_var.set("Reset complete")

    def display_results(self, results):
        """Display simulation results in text form"""
        self.results_text.delete(1.0, tk.END)

        # Get final values
        idx_final = -1

        text = "=== SIMULATION RESULTS ===\n\n"
        text += f"Solver Used: {self.params['solver']}\n"
        text += f"Simulation Time: {self.params['sim_time']} s\n\n"

        text += "--- Final Values ---\n"
        text += f"Speed: {results['speed_rpm'][idx_final]:.2f} rpm\n"
        text += f"Armature Current (RMS): {results['Ia'][idx_final]:.2f} A\n"
        text += f"Field Current (RMS): {results['If'][idx_final]:.2f} A\n"
        text += f"Torque: {results['torque'][idx_final]:.2f} N·m\n"
        text += f"Armature Temperature: {results['T_armature'][idx_final]:.2f} °C\n"
        text += f"Field Temperature: {results['T_field'][idx_final]:.2f} °C\n\n"

        text += "--- Power and Efficiency ---\n"
        text += f"Input Power: {results['P_input'][idx_final]/1000:.2f} kW\n"
        text += f"Output Power: {results['P_output'][idx_final]/1000:.2f} kW\n"
        text += f"Efficiency: {results['efficiency'][idx_final]:.2f} %\n\n"

        text += "--- Losses Breakdown ---\n"
        losses = results['losses']
        text += f"Copper Losses (Armature): {losses['copper_armature'][idx_final]:.2f} W\n"
        text += f"Copper Losses (Field): {losses['copper_field'][idx_final]:.2f} W\n"
        text += f"Iron Losses (Hysteresis): {losses['hysteresis'][idx_final]:.2f} W\n"
        text += f"Iron Losses (Eddy Current): {losses['eddy_current'][idx_final]:.2f} W\n"
        text += f"Friction Losses: {losses['friction'][idx_final]:.2f} W\n"
        text += f"Windage Losses: {losses['windage'][idx_final]:.2f} W\n"
        text += f"Stray Load Losses: {losses['stray_load'][idx_final]:.2f} W\n"
        text += f"Total Losses: {losses['total'][idx_final]:.2f} W\n\n"

        text += "--- Mechanical Stress ---\n"
        text += f"Shear Stress: {results['shear_stress'][idx_final]/1e6:.2f} MPa\n"
        text += f"Bearing Load: {results['bearing_load'][idx_final]:.2f} N\n\n"

        text += "--- Derating Factor ---\n"
        text += f"Derating: {results['derating_factor'][idx_final]:.3f}\n"

        # Calculate energy dissipated during braking
        P_loss_avg = np.mean(losses['total'])
        energy_dissipated = P_loss_avg * self.params['sim_time'] / 3600  # kWh
        text += f"\nTotal Energy Dissipated: {energy_dissipated:.4f} kWh\n"

        self.results_text.insert(tk.END, text)

    def solve_theoretical_problem(self):
        """Solve the theoretical braking problem"""
        solver = TheoreticalSolver()
        results = solver.solve_braking_problem()

        self.theory_text.delete(1.0, tk.END)

        text = "=== THEORETICAL SOLUTION ===\n\n"
        text += "Given:\n"
        text += "• Power: 18.65 kW\n"
        text += "• Voltage: 220 V (RMS)\n"
        text += "• Full-load speed: 600 rpm\n"
        text += "• Armature resistance: Ra = 0.1 Ω\n"
        text += "• Full-load armature current: 95 A (RMS)\n"
        text += "• Current limit during plugging: 130 A (RMS)\n\n"

        text += "Step 1: Calculate Back EMF at Full Load\n"
        text += "---------------------------------------\n"
        text += "V = Eb + Ia × Ra\n"
        text += f"Eb = V - Ia × Ra\n"
        text += f"Eb = 220 - 95 × 0.1\n"
        text += f"Eb = {results['Eb_full_load']:.2f} V\n\n"

        text += f"Angular velocity: ω = 2π × N / 60\n"
        text += f"ω = 2π × 600 / 60 = {results['omega_full_load']:.2f} rad/s\n\n"

        text += "Step 2: Calculate External Resistance for Plugging\n"
        text += "---------------------------------------------------\n"
        text += "During plugging, armature is reversed:\n"
        text += "V + Eb = Ia × (Ra + R)\n"
        text += "R = (V + Eb) / Ia - Ra\n"
        text += f"R = (220 + {results['Eb_full_load']:.2f}) / 130 - 0.1\n"
        text += f"R = {results['external_resistance']:.3f} Ω\n\n"

        text += "Step 3: Calculate Initial Braking Torque\n"
        text += "----------------------------------------\n"
        text += "At the instant of plugging (speed = 600 rpm):\n"
        text += "T = (Eb × Ia) / ω\n"
        text += f"T = ({results['Eb_full_load']:.2f} × 130) / {results['omega_full_load']:.2f}\n"
        text += f"T = {results['torque_initial']:.2f} N·m\n\n"

        text += "Step 4: Calculate Torque at Half Speed\n"
        text += "---------------------------------------\n"
        text += f"When speed = {results['speed_half']:.0f} rpm:\n"
        text += f"Eb (half) = {results['Eb_half']:.2f} V\n"
        text += f"Ia (half) = (V + Eb_half) / (Ra + R)\n"
        text += f"Ia (half) = (220 + {results['Eb_half']:.2f}) / {0.1 + results['external_resistance']:.2f}\n"
        text += f"Ia (half) = {results['Ia_half']:.2f} A\n"
        text += f"T (half) = {results['torque_half']:.2f} N·m\n\n"

        text += "=== FINAL ANSWERS ===\n\n"
        text += f"1. External Resistance Required: R = {results['external_resistance']:.3f} Ω\n\n"
        text += f"2. Initial Braking Torque: T₀ = {results['torque_initial']:.2f} N·m\n\n"
        text += f"3. Braking Torque at Half Speed: T₁/₂ = {results['torque_half']:.2f} N·m\n\n"

        text += "Note: All current values are RMS values as specified.\n"

        self.theory_text.insert(tk.END, text)

        # Also update parameters for simulation
        self.param_widgets['R_ext'].set(results['external_resistance'])
        self.param_widgets['initial_speed'].set(600)
        self.param_widgets['initial_current'].set(95)

        messagebox.showinfo("Success", "Theoretical problem solved! Parameters updated for simulation.")

    def calculate_economics(self):
        """Calculate economic analysis"""
        if self.engine.results is None:
            messagebox.showwarning("Warning", "Please run a simulation first!")
            return

        results = self.engine.results

        # Get cost parameters
        elec_cost = self.elec_cost_var.get()
        motor_cost = self.motor_cost_var.get()
        maint_cost = self.maint_cost_var.get()
        op_hours = self.op_hours_var.get()

        # Calculate average values
        P_avg = np.mean(np.abs(results['P_input'])) / 1000  # kW
        losses_avg = np.mean(results['losses']['total']) / 1000  # kW
        efficiency_avg = np.mean(results['efficiency'])

        # Annual energy consumption
        energy_annual = P_avg * op_hours  # kWh/year

        # Annual energy cost
        energy_cost_annual = energy_annual * elec_cost

        # Annual loss cost
        loss_energy_annual = losses_avg * op_hours
        loss_cost_annual = loss_energy_annual * elec_cost

        # Total annual operating cost
        total_annual_cost = energy_cost_annual + maint_cost

        # Lifecycle cost (10 years)
        lifecycle_years = 10
        lifecycle_cost = motor_cost + total_annual_cost * lifecycle_years

        # Cost per operating hour
        cost_per_hour = total_annual_cost / op_hours

        # Payback period for high-efficiency motor (assuming 5% efficiency improvement)
        he_motor_cost = motor_cost * 1.3  # 30% more expensive
        efficiency_improved = efficiency_avg * 1.05
        loss_reduction = (1 - efficiency_avg/100) - (1 - efficiency_improved/100)
        savings_annual = loss_reduction * P_avg * op_hours * elec_cost
        if savings_annual > 0:
            payback_period = (he_motor_cost - motor_cost) / savings_annual
        else:
            payback_period = float('inf')

        # Display results
        self.econ_text.delete(1.0, tk.END)

        text = "=== ECONOMIC ANALYSIS ===\n\n"
        text += "--- Operating Parameters ---\n"
        text += f"Average Power: {P_avg:.2f} kW\n"
        text += f"Average Efficiency: {efficiency_avg:.2f} %\n"
        text += f"Average Losses: {losses_avg:.2f} kW\n"
        text += f"Operating Hours: {op_hours:.0f} h/year\n\n"

        text += "--- Energy Consumption ---\n"
        text += f"Annual Energy Consumption: {energy_annual:.2f} kWh/year\n"
        text += f"Annual Energy Losses: {loss_energy_annual:.2f} kWh/year\n"
        text += f"Electricity Cost: ${elec_cost:.3f}/kWh\n\n"

        text += "--- Annual Costs ---\n"
        text += f"Energy Cost: ${energy_cost_annual:,.2f}/year\n"
        text += f"Loss Cost: ${loss_cost_annual:,.2f}/year\n"
        text += f"Maintenance Cost: ${maint_cost:,.2f}/year\n"
        text += f"Total Annual Operating Cost: ${total_annual_cost:,.2f}/year\n"
        text += f"Cost per Operating Hour: ${cost_per_hour:.2f}/h\n\n"

        text += "--- Lifecycle Analysis (10 years) ---\n"
        text += f"Initial Motor Cost: ${motor_cost:,.2f}\n"
        text += f"Total Operating Cost (10 years): ${total_annual_cost * lifecycle_years:,.2f}\n"
        text += f"Total Lifecycle Cost: ${lifecycle_cost:,.2f}\n\n"

        text += "--- High-Efficiency Motor Comparison ---\n"
        text += f"High-Efficiency Motor Cost: ${he_motor_cost:,.2f}\n"
        text += f"Improved Efficiency: {efficiency_improved:.2f} %\n"
        text += f"Annual Savings: ${savings_annual:,.2f}/year\n"
        if payback_period < 100:
            text += f"Payback Period: {payback_period:.2f} years\n"
        else:
            text += f"Payback Period: Not economically viable\n"
        text += "\n"

        text += "--- Braking Energy Analysis ---\n"
        if self.params['is_plugging']:
            text += "Braking Method: Plugging (energy dissipated as heat)\n"
        elif self.params['is_dynamic_braking']:
            text += "Braking Method: Dynamic Braking (energy dissipated in resistor)\n"
        else:
            text += "Braking Method: Regenerative (energy recovered to supply)\n"

        # Energy dissipated during braking
        sim_time = self.params['sim_time']
        total_loss_energy = np.trapz(results['losses']['total'], results['time']) / 3600000  # kWh
        text += f"Energy Dissipated in Braking: {total_loss_energy:.6f} kWh per cycle\n"

        # If braking happens frequently
        braking_cycles_per_day = 100
        braking_days_per_year = 250
        total_braking_cycles = braking_cycles_per_day * braking_days_per_year
        annual_braking_energy = total_loss_energy * total_braking_cycles
        annual_braking_cost = annual_braking_energy * elec_cost

        text += f"\nAssuming {braking_cycles_per_day} braking cycles/day, {braking_days_per_year} days/year:\n"
        text += f"Annual Braking Energy: {annual_braking_energy:.2f} kWh/year\n"
        text += f"Annual Braking Cost: ${annual_braking_cost:,.2f}/year\n\n"

        text += "--- Recommendations ---\n"
        if efficiency_avg < 85:
            text += "• Consider upgrading to a higher efficiency motor\n"
        if payback_period < 3:
            text += "• High-efficiency motor upgrade recommended (short payback)\n"
        if annual_braking_cost > 1000:
            text += "• Consider regenerative braking to recover energy\n"
        if losses_avg / P_avg > 0.2:
            text += "• Losses are significant - review operating conditions\n"

        self.econ_text.insert(tk.END, text)

    def on_window_resize(self, event):
        """Handle window resize event for auto-scaling"""
        # Only resize the figure, matplotlib will handle the rest
        if hasattr(self, 'canvas'):
            self.canvas.draw()

# ============================================================================
# MAIN APPLICATION
# ============================================================================

def main():
    """Main entry point"""
    root = tk.Tk()
    app = DCMotorSimulatorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
