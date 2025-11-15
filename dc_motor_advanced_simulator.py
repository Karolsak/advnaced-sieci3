#!/usr/bin/env python3
"""
Advanced DC Motor Multi-Physics Simulator
Comprehensive simulation with electromagnetic, thermal, mechanical, and acoustic modeling
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp, odeint
from scipy.interpolate import interp1d
import threading
import time
from dataclasses import dataclass
from typing import Tuple, List, Dict
import json

# Constants
PI = np.pi
MU_0 = 4 * PI * 1e-7  # Permeability of free space

@dataclass
class MotorParameters:
    """DC Motor parameters"""
    V_supply: float = 250.0  # Supply voltage (V)
    Ra: float = 0.25  # Armature resistance (Ω)
    Rf: float = 250.0  # Field resistance (Ω)
    Rf_added: float = 0.0  # Additional field resistance (Ω)
    La: float = 0.05  # Armature inductance (H)
    Lf: float = 1.0  # Field inductance (H)
    J: float = 0.5  # Moment of inertia (kg·m²)
    B: float = 0.01  # Friction coefficient (N·m·s)
    Kt: float = 1.0  # Torque constant (N·m/A)
    Ke: float = 1.0  # Back EMF constant (V·s/rad)
    pole_pairs: int = 2  # Number of pole pairs

    # Thermal parameters
    thermal_resistance: float = 2.0  # °C/W
    thermal_capacitance: float = 500.0  # J/°C
    ambient_temp: float = 25.0  # °C
    max_temp: float = 155.0  # °C (Class F insulation)

    # Mechanical parameters
    rated_power: float = 5000.0  # W
    rated_speed: float = 1500.0  # RPM
    max_torque: float = 50.0  # N·m

    # Economic parameters
    electricity_cost: float = 0.12  # $/kWh
    maintenance_cost: float = 0.02  # $/hour


class DCMotorPhysics:
    """Multi-physics DC motor model"""

    def __init__(self, params: MotorParameters):
        self.params = params
        self.reset()

    def reset(self):
        """Reset simulation state"""
        self.state = np.array([0.0, 0.0, 0.0, self.params.ambient_temp])  # [Ia, If, omega, temp]
        self.time = 0.0
        self.history = {
            't': [], 'Ia': [], 'If': [], 'omega': [], 'rpm': [],
            'torque': [], 'power_in': [], 'power_out': [], 'efficiency': [],
            'temp': [], 'copper_loss': [], 'iron_loss': [], 'mech_loss': [],
            'stray_loss': [], 'total_loss': [], 'sound_level': []
        }

    def magnetization_curve(self, If: float) -> float:
        """Magnetization curve - can be linear or non-linear"""
        # Linear approximation
        return self.params.Kt * If

    def flux_derivative(self, If: float) -> float:
        """Derivative of flux with respect to field current"""
        return self.params.Kt

    def back_emf(self, omega: float, If: float) -> float:
        """Calculate back EMF"""
        flux = self.magnetization_curve(If)
        return self.params.Ke * omega * flux

    def electromagnetic_torque(self, Ia: float, If: float) -> float:
        """Calculate electromagnetic torque"""
        flux = self.magnetization_curve(If)
        return flux * Ia

    def copper_losses(self, Ia: float, If: float) -> float:
        """Calculate copper losses (I²R)"""
        Rf_total = self.params.Rf + self.params.Rf_added
        return Ia**2 * self.params.Ra + If**2 * Rf_total

    def iron_losses(self, omega: float) -> float:
        """Calculate iron losses (hysteresis + eddy current)"""
        # Simplified model: P_iron = k_h*f + k_e*f²
        freq = abs(omega) * self.params.pole_pairs / (2 * PI)
        k_h = 0.5  # Hysteresis coefficient
        k_e = 0.02  # Eddy current coefficient
        return k_h * freq + k_e * freq**2

    def mechanical_losses(self, omega: float) -> float:
        """Calculate mechanical friction and windage losses"""
        # P_mech = B*ω² + P_bearing
        P_bearing = 10.0  # Constant bearing loss
        return self.params.B * omega**2 + P_bearing

    def stray_load_losses(self, Ia: float) -> float:
        """Calculate stray load losses"""
        # Approximately 1% of output power
        return 0.01 * abs(Ia) * self.params.V_supply

    def total_losses(self, Ia: float, If: float, omega: float) -> float:
        """Calculate total losses"""
        copper = self.copper_losses(Ia, If)
        iron = self.iron_losses(omega)
        mech = self.mechanical_losses(omega)
        stray = self.stray_load_losses(Ia)
        return copper + iron + mech + stray

    def acoustic_noise(self, omega: float, Ia: float) -> float:
        """Estimate acoustic noise level in dB"""
        # Simplified model based on speed and current
        rpm = abs(omega) * 60 / (2 * PI)
        L_speed = 20 * np.log10(rpm + 1)  # Speed contribution
        L_current = 15 * np.log10(abs(Ia) + 1)  # Electromagnetic noise
        L_base = 40  # Base noise level
        return L_base + L_speed + L_current

    def thermal_model(self, temp: float, losses: float) -> float:
        """Thermal model: dT/dt"""
        heat_dissipation = (temp - self.params.ambient_temp) / self.params.thermal_resistance
        return (losses - heat_dissipation) / self.params.thermal_capacitance

    def ode_system(self, t: float, state: np.ndarray, T_load: float, V_supply: float) -> np.ndarray:
        """
        System of differential equations
        state = [Ia, If, omega, temp]
        """
        Ia, If, omega, temp = state

        # Voltage equations
        Rf_total = self.params.Rf + self.params.Rf_added
        Eb = self.back_emf(omega, If)

        # Armature circuit: La * dIa/dt = V - Ia*Ra - Eb
        dIa_dt = (V_supply - Ia * self.params.Ra - Eb) / self.params.La

        # Field circuit: Lf * dIf/dt = V - If*Rf
        dIf_dt = (V_supply - If * Rf_total) / self.params.Lf

        # Mechanical equation: J * dω/dt = Te - Tload - B*ω
        Te = self.electromagnetic_torque(Ia, If)
        domega_dt = (Te - T_load - self.params.B * omega) / self.params.J

        # Thermal equation
        losses = self.total_losses(Ia, If, omega)
        dtemp_dt = self.thermal_model(temp, losses)

        # Temperature derating
        if temp > self.params.max_temp:
            dIa_dt *= 0.5  # Reduce current if overheating

        return np.array([dIa_dt, dIf_dt, domega_dt, dtemp_dt])

    def simulate_step_euler(self, dt: float, T_load: float, V_supply: float):
        """Euler method integration step"""
        derivatives = self.ode_system(self.time, self.state, T_load, V_supply)
        self.state += derivatives * dt
        self.time += dt

    def simulate_step_rk45(self, dt: float, T_load: float, V_supply: float):
        """RK45 integration step"""
        t_span = [self.time, self.time + dt]
        sol = solve_ivp(
            lambda t, y: self.ode_system(t, y, T_load, V_supply),
            t_span, self.state, method='RK45', dense_output=True
        )
        self.state = sol.y[:, -1]
        self.time = sol.t[-1]

    def update_history(self):
        """Update history with current state"""
        Ia, If, omega, temp = self.state
        rpm = omega * 60 / (2 * PI)

        Te = self.electromagnetic_torque(Ia, If)
        P_in = self.params.V_supply * (Ia + If)
        P_out = Te * omega
        efficiency = (P_out / P_in * 100) if P_in > 0 else 0

        copper = self.copper_losses(Ia, If)
        iron = self.iron_losses(omega)
        mech = self.mechanical_losses(omega)
        stray = self.stray_load_losses(Ia)
        total_loss = copper + iron + mech + stray

        sound = self.acoustic_noise(omega, Ia)

        self.history['t'].append(self.time)
        self.history['Ia'].append(Ia)
        self.history['If'].append(If)
        self.history['omega'].append(omega)
        self.history['rpm'].append(rpm)
        self.history['torque'].append(Te)
        self.history['power_in'].append(P_in)
        self.history['power_out'].append(P_out)
        self.history['efficiency'].append(efficiency)
        self.history['temp'].append(temp)
        self.history['copper_loss'].append(copper)
        self.history['iron_loss'].append(iron)
        self.history['mech_loss'].append(mech)
        self.history['stray_loss'].append(stray)
        self.history['total_loss'].append(total_loss)
        self.history['sound_level'].append(sound)

    def solve_steady_state(self, V_supply: float, T_load: float) -> Tuple[float, float, float]:
        """
        Solve for steady-state conditions analytically
        Returns: (Ia, If, omega_rpm)
        """
        Rf_total = self.params.Rf + self.params.Rf_added
        If = V_supply / Rf_total

        # For steady state: Te = T_load + B*omega
        # Te = Kt*If*Ia
        # Eb = Ke*omega*If
        # V = Ia*Ra + Eb

        # Simplified solution (neglecting B for initial estimate)
        flux = self.magnetization_curve(If)
        Ia = T_load / flux if flux > 0 else 0
        Eb = V_supply - Ia * self.params.Ra
        omega = Eb / (self.params.Ke * flux) if flux > 0 else 0

        # Iterative refinement
        for _ in range(10):
            Te = flux * Ia
            omega = (Te - T_load) / self.params.B if self.params.B > 0 else omega
            Eb = self.params.Ke * omega * flux
            Ia = (V_supply - Eb) / self.params.Ra if self.params.Ra > 0 else 0

        omega_rpm = omega * 60 / (2 * PI)
        return Ia, If, omega_rpm


class AdvancedDCMotorGUI:
    """Advanced GUI for DC Motor Simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Simulation parameters
        self.params = MotorParameters()
        self.motor = DCMotorPhysics(self.params)

        # Simulation control
        self.is_running = False
        self.simulation_thread = None
        self.dt = 0.001  # Time step
        self.solver_method = 'RK45'

        # Create UI
        self.create_menu()
        self.create_main_layout()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Export Data", command=self.export_data)
        file_menu.add_command(label="Load Parameters", command=self.load_parameters)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Solver menu
        solver_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Solver", menu=solver_menu)
        solver_menu.add_command(label="Euler", command=lambda: self.set_solver('Euler'))
        solver_menu.add_command(label="RK45", command=lambda: self.set_solver('RK45'))

    def set_solver(self, method: str):
        """Set ODE solver method"""
        self.solver_method = method
        messagebox.showinfo("Solver", f"Solver set to {method}")

    def create_main_layout(self):
        """Create main application layout"""
        # Main container with auto-resize
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Configure grid weights for auto-resize
        main_frame.columnconfigure(0, weight=3)
        main_frame.columnconfigure(1, weight=7)
        main_frame.rowconfigure(0, weight=1)

        # Left panel - Controls
        self.create_control_panel(main_frame)

        # Right panel - Visualization
        self.create_visualization_panel(main_frame)

    def create_control_panel(self, parent):
        """Create control panel with tabs"""
        control_frame = ttk.Frame(parent)
        control_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 5))

        # Notebook for tabs
        notebook = ttk.Notebook(control_frame)
        notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Motor Parameters
        self.create_motor_params_tab(notebook)

        # Tab 2: Control Settings
        self.create_control_tab(notebook)

        # Tab 3: Economic Analysis
        self.create_economic_tab(notebook)

        # Tab 4: Advanced Settings
        self.create_advanced_tab(notebook)

        # Control buttons at bottom
        self.create_control_buttons(control_frame)

    def create_motor_params_tab(self, notebook):
        """Create motor parameters tab"""
        tab = ttk.Frame(notebook)
        notebook.add(tab, text="Motor Parameters")

        # Scrollable frame
        canvas = tk.Canvas(tab)
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Parameters
        self.param_vars = {}
        params_config = [
            ("Supply Voltage (V)", "V_supply", 0, 500, 250),
            ("Armature Resistance (Ω)", "Ra", 0.01, 2, 0.25),
            ("Field Resistance (Ω)", "Rf", 10, 500, 250),
            ("Additional Field R (Ω)", "Rf_added", 0, 500, 0),
            ("Armature Inductance (H)", "La", 0.01, 1, 0.05),
            ("Field Inductance (H)", "Lf", 0.1, 5, 1.0),
            ("Inertia (kg·m²)", "J", 0.01, 5, 0.5),
            ("Friction Coeff (N·m·s)", "B", 0.001, 0.1, 0.01),
            ("Torque Constant", "Kt", 0.1, 5, 1.0),
            ("Back EMF Constant", "Ke", 0.1, 5, 1.0),
        ]

        for i, (label, attr, min_val, max_val, default) in enumerate(params_config):
            ttk.Label(scrollable_frame, text=label).grid(row=i, column=0, sticky='w', padx=5, pady=2)

            var = tk.DoubleVar(value=default)
            self.param_vars[attr] = var

            scale = ttk.Scale(scrollable_frame, from_=min_val, to=max_val,
                            variable=var, orient='horizontal', command=lambda v, a=attr: self.update_param(a))
            scale.grid(row=i, column=1, sticky='ew', padx=5, pady=2)

            entry = ttk.Entry(scrollable_frame, textvariable=var, width=8)
            entry.grid(row=i, column=2, padx=5, pady=2)

        scrollable_frame.columnconfigure(1, weight=1)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_control_tab(self, notebook):
        """Create control settings tab"""
        tab = ttk.Frame(notebook)
        notebook.add(tab, text="Control")

        # Load torque control
        ttk.Label(tab, text="Load Torque (N·m)").pack(pady=5)
        self.torque_var = tk.DoubleVar(value=10.0)
        torque_scale = ttk.Scale(tab, from_=0, to=50, variable=self.torque_var, orient='horizontal')
        torque_scale.pack(fill='x', padx=10, pady=5)

        torque_entry = ttk.Entry(tab, textvariable=self.torque_var, width=10)
        torque_entry.pack(pady=5)

        # Supply voltage control
        ttk.Label(tab, text="Supply Voltage (V)").pack(pady=5)
        self.voltage_var = tk.DoubleVar(value=250.0)
        voltage_scale = ttk.Scale(tab, from_=0, to=500, variable=self.voltage_var, orient='horizontal')
        voltage_scale.pack(fill='x', padx=10, pady=5)

        voltage_entry = ttk.Entry(tab, textvariable=self.voltage_var, width=10)
        voltage_entry.pack(pady=5)

        # Field resistance control
        ttk.Label(tab, text="Additional Field R (Ω)").pack(pady=5)
        self.field_r_var = tk.DoubleVar(value=0.0)
        field_scale = ttk.Scale(tab, from_=0, to=500, variable=self.field_r_var,
                               orient='horizontal', command=self.update_field_resistance)
        field_scale.pack(fill='x', padx=10, pady=5)

        field_entry = ttk.Entry(tab, textvariable=self.field_r_var, width=10)
        field_entry.pack(pady=5)

        # Speed reference
        ttk.Label(tab, text="Speed Reference (RPM)").pack(pady=5)
        self.speed_ref_var = tk.DoubleVar(value=1500.0)
        speed_scale = ttk.Scale(tab, from_=0, to=3000, variable=self.speed_ref_var, orient='horizontal')
        speed_scale.pack(fill='x', padx=10, pady=5)

        # Thermal protection
        ttk.Label(tab, text="Thermal Protection").pack(pady=10)
        self.thermal_protection_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(tab, text="Enable Thermal Derating",
                       variable=self.thermal_protection_var).pack()

    def create_economic_tab(self, notebook):
        """Create economic analysis tab"""
        tab = ttk.Frame(notebook)
        notebook.add(tab, text="Economics")

        ttk.Label(tab, text="Economic Analysis", font=('Arial', 12, 'bold')).pack(pady=10)

        # Electricity cost
        ttk.Label(tab, text="Electricity Cost ($/kWh)").pack(pady=5)
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(tab, textvariable=self.elec_cost_var, width=10).pack()

        # Operating hours
        ttk.Label(tab, text="Operating Hours").pack(pady=5)
        self.operating_hours_var = tk.DoubleVar(value=8760)  # 1 year
        ttk.Entry(tab, textvariable=self.operating_hours_var, width=10).pack()

        # Results display
        self.economic_results = tk.Text(tab, height=15, width=35)
        self.economic_results.pack(pady=10, padx=5, fill='both', expand=True)

        ttk.Button(tab, text="Calculate Economics",
                  command=self.calculate_economics).pack(pady=5)

    def create_advanced_tab(self, notebook):
        """Create advanced settings tab"""
        tab = ttk.Frame(notebook)
        notebook.add(tab, text="Advanced")

        ttk.Label(tab, text="Simulation Settings", font=('Arial', 12, 'bold')).pack(pady=10)

        # Time step
        ttk.Label(tab, text="Time Step (s)").pack(pady=5)
        self.dt_var = tk.DoubleVar(value=0.001)
        ttk.Entry(tab, textvariable=self.dt_var, width=10).pack()

        # Solver selection
        ttk.Label(tab, text="ODE Solver").pack(pady=5)
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(tab, text="Euler", variable=self.solver_var,
                       value='Euler').pack(anchor='w', padx=20)
        ttk.Radiobutton(tab, text="RK45 (Adaptive)", variable=self.solver_var,
                       value='RK45').pack(anchor='w', padx=20)

        # Current state display
        ttk.Label(tab, text="\nCurrent State", font=('Arial', 10, 'bold')).pack(pady=5)
        self.state_display = tk.Text(tab, height=10, width=35)
        self.state_display.pack(pady=5, padx=5, fill='both', expand=True)

    def create_control_buttons(self, parent):
        """Create control buttons"""
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill='x', pady=10)

        ttk.Button(button_frame, text="START", command=self.start_simulation,
                  style='Accent.TButton').pack(side='left', padx=5, expand=True, fill='x')
        ttk.Button(button_frame, text="STOP", command=self.stop_simulation).pack(
            side='left', padx=5, expand=True, fill='x')
        ttk.Button(button_frame, text="RESET", command=self.reset_simulation).pack(
            side='left', padx=5, expand=True, fill='x')

        # Solve button for analytical solution
        ttk.Button(button_frame, text="SOLVE", command=self.solve_problem,
                  style='Accent.TButton').pack(pady=5, fill='x')

    def create_visualization_panel(self, parent):
        """Create visualization panel with multiple plots"""
        viz_frame = ttk.Frame(parent)
        viz_frame.grid(row=0, column=1, sticky='nsew')

        # Configure grid weights
        viz_frame.rowconfigure(0, weight=1)
        viz_frame.columnconfigure(0, weight=1)

        # Notebook for different views
        self.viz_notebook = ttk.Notebook(viz_frame)
        self.viz_notebook.pack(fill=tk.BOTH, expand=True)

        # Create different visualization tabs
        self.create_main_plots_tab()
        self.create_loss_analysis_tab()
        self.create_efficiency_map_tab()
        self.create_thermal_tab()

    def create_main_plots_tab(self):
        """Create main plots tab"""
        tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(tab, text="Dynamic Response")

        self.fig1 = Figure(figsize=(10, 8), dpi=100)
        self.canvas1 = FigureCanvasTkAgg(self.fig1, tab)
        self.canvas1.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create subplots
        self.ax1 = self.fig1.add_subplot(3, 2, 1)
        self.ax2 = self.fig1.add_subplot(3, 2, 2)
        self.ax3 = self.fig1.add_subplot(3, 2, 3)
        self.ax4 = self.fig1.add_subplot(3, 2, 4)
        self.ax5 = self.fig1.add_subplot(3, 2, 5)
        self.ax6 = self.fig1.add_subplot(3, 2, 6)

        self.fig1.tight_layout(pad=2.0)

    def create_loss_analysis_tab(self):
        """Create loss analysis tab"""
        tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(tab, text="Loss Analysis")

        self.fig2 = Figure(figsize=(10, 6), dpi=100)
        self.canvas2 = FigureCanvasTkAgg(self.fig2, tab)
        self.canvas2.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax_loss1 = self.fig2.add_subplot(2, 2, 1)
        self.ax_loss2 = self.fig2.add_subplot(2, 2, 2)
        self.ax_loss3 = self.fig2.add_subplot(2, 2, 3)
        self.ax_loss4 = self.fig2.add_subplot(2, 2, 4)

        self.fig2.tight_layout(pad=2.0)

    def create_efficiency_map_tab(self):
        """Create efficiency map tab"""
        tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(tab, text="Efficiency Map")

        self.fig3 = Figure(figsize=(10, 6), dpi=100)
        self.canvas3 = FigureCanvasTkAgg(self.fig3, tab)
        self.canvas3.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax_eff = self.fig3.add_subplot(1, 1, 1)

        self.fig3.tight_layout(pad=2.0)

    def create_thermal_tab(self):
        """Create thermal analysis tab"""
        tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(tab, text="Thermal & Acoustic")

        self.fig4 = Figure(figsize=(10, 6), dpi=100)
        self.canvas4 = FigureCanvasTkAgg(self.fig4, tab)
        self.canvas4.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax_thermal = self.fig4.add_subplot(2, 1, 1)
        self.ax_acoustic = self.fig4.add_subplot(2, 1, 2)

        self.fig4.tight_layout(pad=2.0)

    def update_param(self, param_name):
        """Update motor parameter"""
        value = self.param_vars[param_name].get()
        setattr(self.params, param_name, value)
        self.motor = DCMotorPhysics(self.params)

    def update_field_resistance(self, value):
        """Update field resistance dynamically"""
        self.params.Rf_added = float(value)

    def start_simulation(self):
        """Start real-time simulation"""
        if not self.is_running:
            self.is_running = True
            self.dt = self.dt_var.get()
            self.solver_method = self.solver_var.get()

            self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
            self.simulation_thread.start()

    def stop_simulation(self):
        """Stop simulation"""
        self.is_running = False

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        time.sleep(0.1)
        self.motor.reset()
        self.update_plots()

    def run_simulation(self):
        """Main simulation loop"""
        while self.is_running:
            T_load = self.torque_var.get()
            V_supply = self.voltage_var.get()
            self.params.Rf_added = self.field_r_var.get()

            # Perform integration step
            if self.solver_method == 'Euler':
                self.motor.simulate_step_euler(self.dt, T_load, V_supply)
            else:  # RK45
                self.motor.simulate_step_rk45(self.dt, T_load, V_supply)

            self.motor.update_history()

            # Update GUI every 50ms
            if len(self.motor.history['t']) % 10 == 0:
                self.root.after(0, self.update_plots)
                self.root.after(0, self.update_state_display)

            time.sleep(self.dt)

    def solve_problem(self):
        """Solve the specific problem: 250V motor with field resistance change"""
        # Problem parameters
        V = 250.0
        Ra = 0.25
        Rf = 250.0
        N1 = 1500.0  # RPM
        Ia1 = 20.0   # A
        Rf_added = 250.0  # Additional resistance

        # Initial conditions
        If1 = V / Rf
        Eb1 = V - Ia1 * Ra

        # With additional resistance
        Rf_total = Rf + Rf_added
        If2 = V / Rf_total

        # Flux ratio (linear magnetization)
        flux_ratio = If2 / If1

        # Constant torque: Ia2 = Ia1 / flux_ratio
        Ia2 = Ia1 / flux_ratio

        # New back EMF
        Eb2 = V - Ia2 * Ra

        # New speed: N2 = N1 * (Eb2/Eb1) * (If1/If2)
        N2 = N1 * (Eb2 / Eb1) * (If1 / If2)

        # Display results
        result_text = f"""
╔══════════════════════════════════════════════════════════════╗
║         DC SHUNT MOTOR PROBLEM SOLUTION                      ║
╠══════════════════════════════════════════════════════════════╣
║ GIVEN:                                                       ║
║   Supply Voltage (V)          = {V:.1f} V                    ║
║   Armature Resistance (Ra)    = {Ra:.2f} Ω                   ║
║   Field Resistance (Rf)       = {Rf:.1f} Ω                   ║
║   Initial Speed (N1)          = {N1:.0f} RPM                 ║
║   Initial Armature Current    = {Ia1:.1f} A                  ║
║   Additional Field Resistance = {Rf_added:.1f} Ω             ║
║   Load Torque                 = Constant                     ║
║   Magnetization Curve         = Linear                       ║
╠══════════════════════════════════════════════════════════════╣
║ INITIAL CONDITIONS:                                          ║
║   Field Current (If1)         = {If1:.3f} A                  ║
║   Back EMF (Eb1)              = {Eb1:.2f} V                  ║
║   Flux (Φ1)                   = Φ₁ (reference)               ║
╠══════════════════════════════════════════════════════════════╣
║ WITH ADDITIONAL FIELD RESISTANCE:                            ║
║   Total Field Resistance      = {Rf_total:.1f} Ω             ║
║   Field Current (If2)         = {If2:.3f} A                  ║
║   Flux Ratio (Φ2/Φ1)          = {flux_ratio:.3f}             ║
║                                                              ║
║   Since Torque is constant:                                  ║
║   T = Φ₁·Ia₁ = Φ₂·Ia₂                                        ║
║   Ia₂ = Ia₁·(Φ₁/Φ₂)          = {Ia2:.2f} A                   ║
║                                                              ║
║   Back EMF (Eb2)              = {Eb2:.2f} V                  ║
║                                                              ║
║   Speed: N ∝ Eb/Φ                                            ║
║   N₂ = N₁·(Eb₂/Eb₁)·(Φ₁/Φ₂)   = {N2:.2f} RPM                ║
╠══════════════════════════════════════════════════════════════╣
║ FINAL ANSWER:                                                ║
║   New Armature Current (Ia2)  = {Ia2:.2f} A                  ║
║   New Speed (N2)              = {N2:.2f} RPM                 ║
╠══════════════════════════════════════════════════════════════╣
║ ANALYSIS:                                                    ║
║   • Field current decreased by {(1-flux_ratio)*100:.1f}%     ║
║   • Armature current increased by {(Ia2/Ia1-1)*100:.1f}%     ║
║   • Speed increased by {(N2/N1-1)*100:.1f}%                  ║
║   • Power consumption: {V*Ia2:.1f} W (armature)              ║
║   • Copper losses increased significantly                    ║
║   • Risk of overheating due to higher armature current       ║
╚══════════════════════════════════════════════════════════════╝
"""

        messagebox.showinfo("Problem Solution", result_text)

        # Also print to console
        print(result_text)

    def update_plots(self):
        """Update all plots with current data"""
        if len(self.motor.history['t']) < 2:
            return

        # Main plots
        self.update_main_plots()
        self.update_loss_plots()
        self.update_efficiency_map()
        self.update_thermal_plots()

    def update_main_plots(self):
        """Update main dynamic response plots"""
        t = np.array(self.motor.history['t'])

        # Speed
        self.ax1.clear()
        self.ax1.plot(t, self.motor.history['rpm'], 'b-', linewidth=2)
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Speed (RPM)')
        self.ax1.set_title('Motor Speed')
        self.ax1.grid(True, alpha=0.3)

        # Currents
        self.ax2.clear()
        self.ax2.plot(t, self.motor.history['Ia'], 'r-', label='Ia', linewidth=2)
        self.ax2.plot(t, self.motor.history['If'], 'g-', label='If', linewidth=2)
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Current (A)')
        self.ax2.set_title('Armature & Field Currents')
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.3)

        # Torque
        self.ax3.clear()
        self.ax3.plot(t, self.motor.history['torque'], 'purple', linewidth=2)
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Torque (N·m)')
        self.ax3.set_title('Electromagnetic Torque')
        self.ax3.grid(True, alpha=0.3)

        # Power
        self.ax4.clear()
        self.ax4.plot(t, np.array(self.motor.history['power_in'])/1000, 'r-',
                     label='Input', linewidth=2)
        self.ax4.plot(t, np.array(self.motor.history['power_out'])/1000, 'g-',
                     label='Output', linewidth=2)
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Power (kW)')
        self.ax4.set_title('Power Flow')
        self.ax4.legend()
        self.ax4.grid(True, alpha=0.3)

        # Efficiency
        self.ax5.clear()
        self.ax5.plot(t, self.motor.history['efficiency'], 'orange', linewidth=2)
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Efficiency (%)')
        self.ax5.set_title('Motor Efficiency')
        self.ax5.grid(True, alpha=0.3)

        # Temperature
        self.ax6.clear()
        self.ax6.plot(t, self.motor.history['temp'], 'darkred', linewidth=2)
        self.ax6.axhline(y=self.params.max_temp, color='r', linestyle='--',
                        label='Max Temp')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Temperature (°C)')
        self.ax6.set_title('Winding Temperature')
        self.ax6.legend()
        self.ax6.grid(True, alpha=0.3)

        self.fig1.tight_layout()
        self.canvas1.draw()

    def update_loss_plots(self):
        """Update loss analysis plots"""
        if len(self.motor.history['t']) < 2:
            return

        t = np.array(self.motor.history['t'])

        # Loss breakdown over time
        self.ax_loss1.clear()
        self.ax_loss1.plot(t, self.motor.history['copper_loss'], label='Copper')
        self.ax_loss1.plot(t, self.motor.history['iron_loss'], label='Iron')
        self.ax_loss1.plot(t, self.motor.history['mech_loss'], label='Mechanical')
        self.ax_loss1.plot(t, self.motor.history['stray_loss'], label='Stray')
        self.ax_loss1.set_xlabel('Time (s)')
        self.ax_loss1.set_ylabel('Loss (W)')
        self.ax_loss1.set_title('Loss Breakdown')
        self.ax_loss1.legend()
        self.ax_loss1.grid(True, alpha=0.3)

        # Pie chart of losses (latest values)
        if len(self.motor.history['copper_loss']) > 0:
            self.ax_loss2.clear()
            losses = [
                self.motor.history['copper_loss'][-1],
                self.motor.history['iron_loss'][-1],
                self.motor.history['mech_loss'][-1],
                self.motor.history['stray_loss'][-1]
            ]
            labels = ['Copper', 'Iron', 'Mechanical', 'Stray']
            colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']
            self.ax_loss2.pie(losses, labels=labels, colors=colors, autopct='%1.1f%%')
            self.ax_loss2.set_title('Current Loss Distribution')

        # Total loss
        self.ax_loss3.clear()
        self.ax_loss3.plot(t, self.motor.history['total_loss'], 'r-', linewidth=2)
        self.ax_loss3.set_xlabel('Time (s)')
        self.ax_loss3.set_ylabel('Total Loss (W)')
        self.ax_loss3.set_title('Total Losses')
        self.ax_loss3.grid(True, alpha=0.3)

        # Efficiency vs Load
        if len(self.motor.history['torque']) > 0:
            self.ax_loss4.clear()
            self.ax_loss4.scatter(self.motor.history['torque'],
                                 self.motor.history['efficiency'],
                                 c=t, cmap='viridis', alpha=0.6)
            self.ax_loss4.set_xlabel('Torque (N·m)')
            self.ax_loss4.set_ylabel('Efficiency (%)')
            self.ax_loss4.set_title('Efficiency vs Load')
            self.ax_loss4.grid(True, alpha=0.3)

        self.fig2.tight_layout()
        self.canvas2.draw()

    def update_efficiency_map(self):
        """Generate efficiency contour map"""
        # Generate efficiency map (torque vs speed)
        torque_range = np.linspace(0.1, 50, 30)
        speed_range = np.linspace(100, 3000, 30)

        T_grid, N_grid = np.meshgrid(torque_range, speed_range)
        eff_grid = np.zeros_like(T_grid)

        for i in range(len(speed_range)):
            for j in range(len(torque_range)):
                omega = speed_range[i] * 2 * PI / 60
                T = torque_range[j]

                # Estimate currents for this operating point
                If = self.params.V_supply / (self.params.Rf + self.params.Rf_added)
                flux = self.motor.magnetization_curve(If)
                Ia = T / flux if flux > 0 else 0

                # Calculate efficiency
                P_in = self.params.V_supply * (Ia + If)
                P_out = T * omega
                eff = (P_out / P_in * 100) if P_in > 0 else 0
                eff_grid[i, j] = min(eff, 100)

        self.ax_eff.clear()
        contour = self.ax_eff.contourf(T_grid, N_grid, eff_grid, levels=20, cmap='RdYlGn')
        self.ax_eff.set_xlabel('Torque (N·m)')
        self.ax_eff.set_ylabel('Speed (RPM)')
        self.ax_eff.set_title('Efficiency Map (%)')
        self.fig3.colorbar(contour, ax=self.ax_eff)

        # Mark current operating point
        if len(self.motor.history['torque']) > 0:
            current_torque = self.motor.history['torque'][-1]
            current_rpm = self.motor.history['rpm'][-1]
            self.ax_eff.plot(current_torque, current_rpm, 'r*', markersize=15,
                           label='Current')
            self.ax_eff.legend()

        self.fig3.tight_layout()
        self.canvas3.draw()

    def update_thermal_plots(self):
        """Update thermal and acoustic plots"""
        if len(self.motor.history['t']) < 2:
            return

        t = np.array(self.motor.history['t'])

        # Thermal plot
        self.ax_thermal.clear()
        self.ax_thermal.plot(t, self.motor.history['temp'], 'darkred', linewidth=2)
        self.ax_thermal.axhline(y=self.params.max_temp, color='r', linestyle='--',
                              label=f'Max ({self.params.max_temp}°C)')
        self.ax_thermal.axhline(y=self.params.ambient_temp, color='b', linestyle='--',
                              label=f'Ambient ({self.params.ambient_temp}°C)')
        self.ax_thermal.set_xlabel('Time (s)')
        self.ax_thermal.set_ylabel('Temperature (°C)')
        self.ax_thermal.set_title('Thermal Response')
        self.ax_thermal.legend()
        self.ax_thermal.grid(True, alpha=0.3)

        # Acoustic noise
        self.ax_acoustic.clear()
        self.ax_acoustic.plot(t, self.motor.history['sound_level'], 'purple', linewidth=2)
        self.ax_acoustic.axhline(y=85, color='orange', linestyle='--',
                                label='Warning (85 dB)')
        self.ax_acoustic.set_xlabel('Time (s)')
        self.ax_acoustic.set_ylabel('Sound Level (dB)')
        self.ax_acoustic.set_title('Acoustic Noise Estimation')
        self.ax_acoustic.legend()
        self.ax_acoustic.grid(True, alpha=0.3)

        self.fig4.tight_layout()
        self.canvas4.draw()

    def update_state_display(self):
        """Update state display in advanced tab"""
        if len(self.motor.history['t']) == 0:
            return

        Ia = self.motor.state[0]
        If = self.motor.state[1]
        omega = self.motor.state[2]
        temp = self.motor.state[3]
        rpm = omega * 60 / (2 * PI)

        torque = self.motor.electromagnetic_torque(Ia, If)
        power_out = torque * omega

        state_text = f"""
Time: {self.motor.time:.3f} s

Armature Current:  {Ia:.3f} A
Field Current:     {If:.3f} A
Angular Velocity:  {omega:.3f} rad/s
Speed:             {rpm:.1f} RPM
Torque:            {torque:.3f} N·m
Output Power:      {power_out:.1f} W
Temperature:       {temp:.1f} °C

Solver: {self.solver_method}
Time Step: {self.dt:.4f} s
"""

        self.state_display.delete('1.0', tk.END)
        self.state_display.insert('1.0', state_text)

    def calculate_economics(self):
        """Calculate economic analysis"""
        if len(self.motor.history['power_in']) == 0:
            messagebox.showwarning("Warning", "Run simulation first!")
            return

        # Average power consumption
        avg_power_kw = np.mean(self.motor.history['power_in']) / 1000

        # Operating hours
        hours = self.operating_hours_var.get()

        # Energy consumption
        energy_kwh = avg_power_kw * hours

        # Cost
        cost_per_kwh = self.elec_cost_var.get()
        total_cost = energy_kwh * cost_per_kwh

        # Efficiency impact
        avg_eff = np.mean(self.motor.history['efficiency'])
        losses_kwh = energy_kwh * (100 - avg_eff) / 100
        wasted_cost = losses_kwh * cost_per_kwh

        # CO2 emissions (approx 0.5 kg CO2 per kWh)
        co2_kg = energy_kwh * 0.5

        result_text = f"""
ECONOMIC ANALYSIS
{'='*40}

Operating Conditions:
  Average Power:      {avg_power_kw:.2f} kW
  Operating Hours:    {hours:.0f} h/year
  Average Efficiency: {avg_eff:.1f} %

Energy Consumption:
  Total Energy:       {energy_kwh:.1f} kWh/year
  Energy in Losses:   {losses_kwh:.1f} kWh/year

Financial Analysis:
  Electricity Cost:   ${cost_per_kwh:.3f}/kWh
  Total Annual Cost:  ${total_cost:.2f}
  Cost from Losses:   ${wasted_cost:.2f}

Potential Savings:
  If efficiency improved to 95%:
  Saved Energy:       {losses_kwh - energy_kwh*0.05:.1f} kWh
  Saved Cost:         ${wasted_cost - energy_kwh*0.05*cost_per_kwh:.2f}

Environmental Impact:
  CO₂ Emissions:      {co2_kg:.1f} kg/year

Recommendations:
  • Monitor temperature to prevent derating
  • Optimize field resistance for efficiency
  • Consider VFD for variable loads
  • Schedule maintenance every {hours/8760*12:.0f} months
"""

        self.economic_results.delete('1.0', tk.END)
        self.economic_results.insert('1.0', result_text)

    def export_data(self):
        """Export simulation data to JSON"""
        if len(self.motor.history['t']) == 0:
            messagebox.showwarning("Warning", "No data to export!")
            return

        # Convert numpy arrays to lists for JSON
        export_data = {}
        for key, value in self.motor.history.items():
            export_data[key] = [float(v) for v in value]

        filename = f"motor_data_{int(time.time())}.json"
        with open(filename, 'w') as f:
            json.dump(export_data, f, indent=2)

        messagebox.showinfo("Success", f"Data exported to {filename}")

    def load_parameters(self):
        """Load motor parameters from file"""
        messagebox.showinfo("Info", "Parameter loading not yet implemented")

    def on_resize(self, event):
        """Handle window resize"""
        # Plots automatically resize due to pack(fill=BOTH, expand=True)
        pass


def main():
    """Main application entry point"""
    root = tk.Tk()

    # Set style
    style = ttk.Style()
    style.theme_use('clam')

    # Create application
    app = AdvancedDCMotorGUI(root)

    # Start GUI
    root.mainloop()


if __name__ == "__main__":
    main()
