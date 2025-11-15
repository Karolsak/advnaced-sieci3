"""
Advanced DC Shunt Motor Simulator with Multi-Physics Analysis
Features:
- Dynamic simulation with RK45 and Euler ODE solvers
- Thermal modeling and cooling analysis
- Economic analysis
- Multi-physics coupling (electromagnetic-thermal-mechanical)
- Real-time visualization
- Advanced control strategies
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import time
from datetime import datetime

class DCShuntMotorSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Shunt Motor Simulator - Multi-Physics Analysis")
        self.root.geometry("1400x900")

        # Simulation state
        self.is_running = False
        self.simulation_time = 0
        self.time_data = []
        self.current_data = []
        self.speed_data = []
        self.torque_data = []
        self.efficiency_data = []
        self.temperature_data = []
        self.power_data = []

        # Motor parameters (default values from the problem)
        self.params = {
            'V': 250.0,          # Supply voltage (V)
            'Ra': 0.5,           # Armature resistance (Ω)
            'Rsh': 250.0,        # Field resistance (Ω)
            'La': 0.05,          # Armature inductance (H)
            'J': 0.5,            # Moment of inertia (kg·m²)
            'B': 0.01,           # Viscous damping (N·m·s)
            'Kv': 1.0,           # Voltage constant (V·s/rad)
            'Kt': 1.0,           # Torque constant (N·m/A)
            'P_rated': 7460,     # Rated power (W)
            'T_ambient': 25.0,   # Ambient temperature (°C)
            'thermal_R': 2.0,    # Thermal resistance (°C/W)
            'thermal_C': 500.0,  # Thermal capacitance (J/°C)
            'efficiency': 0.85,  # Nominal efficiency
            'cost_per_kwh': 0.12 # Electricity cost ($/kWh)
        }

        # State variables: [Ia, omega, theta, T_motor]
        self.state = np.array([0.0, 0.0, 0.0, 25.0])

        # Load torque
        self.T_load = 0.0

        # Solver selection
        self.solver_method = 'RK45'

        # Create GUI
        self.create_gui()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def create_gui(self):
        """Create the main GUI with tabs"""

        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_main_tab()
        self.create_analysis_tab()
        self.create_thermal_tab()
        self.create_economic_tab()
        self.create_multiphysics_tab()
        self.create_control_tab()

    def create_main_tab(self):
        """Main simulation and control tab"""
        main_frame = ttk.Frame(self.notebook)
        self.notebook.add(main_frame, text="Main Simulation")

        # Left panel - Controls
        left_panel = ttk.Frame(main_frame, width=350)
        left_panel.pack(side='left', fill='y', padx=5, pady=5)
        left_panel.pack_propagate(False)

        # Motor Parameters
        param_frame = ttk.LabelFrame(left_panel, text="Motor Parameters", padding=10)
        param_frame.pack(fill='x', pady=5)

        self.param_vars = {}
        param_list = [
            ('Supply Voltage (V)', 'V', 0, 500),
            ('Armature Resistance (Ω)', 'Ra', 0.1, 5),
            ('Field Resistance (Ω)', 'Rsh', 50, 500),
            ('Armature Inductance (H)', 'La', 0.01, 0.5),
            ('Moment of Inertia (kg·m²)', 'J', 0.1, 5),
            ('Damping Coefficient', 'B', 0.001, 0.1),
        ]

        for i, (label, key, min_val, max_val) in enumerate(param_list):
            ttk.Label(param_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)
            var = tk.DoubleVar(value=self.params[key])
            self.param_vars[key] = var

            slider = ttk.Scale(param_frame, from_=min_val, to=max_val,
                             variable=var, orient='horizontal', length=200)
            slider.grid(row=i, column=1, pady=2, padx=5)

            entry = ttk.Entry(param_frame, textvariable=var, width=8)
            entry.grid(row=i, column=2, pady=2)

        # Load Control
        load_frame = ttk.LabelFrame(left_panel, text="Load Control", padding=10)
        load_frame.pack(fill='x', pady=5)

        ttk.Label(load_frame, text="Load Torque (N·m)").grid(row=0, column=0, sticky='w')
        self.load_var = tk.DoubleVar(value=0)
        load_slider = ttk.Scale(load_frame, from_=0, to=100,
                               variable=self.load_var, orient='horizontal', length=200,
                               command=self.update_load)
        load_slider.grid(row=0, column=1, padx=5)
        ttk.Entry(load_frame, textvariable=self.load_var, width=8).grid(row=0, column=2)

        # Solver Selection
        solver_frame = ttk.LabelFrame(left_panel, text="Solver Settings", padding=10)
        solver_frame.pack(fill='x', pady=5)

        ttk.Label(solver_frame, text="ODE Solver:").grid(row=0, column=0, sticky='w')
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(solver_frame, textvariable=self.solver_var,
                                   values=['RK45', 'Euler', 'RK23', 'DOP853'],
                                   state='readonly', width=15)
        solver_combo.grid(row=0, column=1, pady=2)

        ttk.Label(solver_frame, text="Time Step (s):").grid(row=1, column=0, sticky='w')
        self.dt_var = tk.DoubleVar(value=0.01)
        ttk.Entry(solver_frame, textvariable=self.dt_var, width=15).grid(row=1, column=1, pady=2)

        # Control Buttons
        button_frame = ttk.Frame(left_panel)
        button_frame.pack(fill='x', pady=10)

        self.start_btn = ttk.Button(button_frame, text="START", command=self.start_simulation,
                                   style='Success.TButton')
        self.start_btn.pack(side='left', padx=2, expand=True, fill='x')

        self.stop_btn = ttk.Button(button_frame, text="STOP", command=self.stop_simulation,
                                  state='disabled')
        self.stop_btn.pack(side='left', padx=2, expand=True, fill='x')

        self.reset_btn = ttk.Button(button_frame, text="RESET", command=self.reset_simulation)
        self.reset_btn.pack(side='left', padx=2, expand=True, fill='x')

        # Status Display
        status_frame = ttk.LabelFrame(left_panel, text="Current Status", padding=10)
        status_frame.pack(fill='both', expand=True, pady=5)

        self.status_labels = {}
        status_items = [
            ('Armature Current (A)', 'Ia'),
            ('Speed (RPM)', 'speed'),
            ('Torque (N·m)', 'torque'),
            ('Power (kW)', 'power'),
            ('Efficiency (%)', 'efficiency'),
            ('Temperature (°C)', 'temp'),
            ('Simulation Time (s)', 'time')
        ]

        for i, (label, key) in enumerate(status_items):
            ttk.Label(status_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)
            value_label = ttk.Label(status_frame, text="0.00", font=('Arial', 10, 'bold'))
            value_label.grid(row=i, column=1, sticky='e', pady=2)
            self.status_labels[key] = value_label

        # Right panel - Plots
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side='right', fill='both', expand=True, padx=5, pady=5)

        # Create matplotlib figure
        self.main_fig = Figure(figsize=(10, 8), dpi=100)

        # Create subplots
        self.ax1 = self.main_fig.add_subplot(3, 2, 1)
        self.ax2 = self.main_fig.add_subplot(3, 2, 2)
        self.ax3 = self.main_fig.add_subplot(3, 2, 3)
        self.ax4 = self.main_fig.add_subplot(3, 2, 4)
        self.ax5 = self.main_fig.add_subplot(3, 2, 5)
        self.ax6 = self.main_fig.add_subplot(3, 2, 6)

        self.setup_main_plots()

        self.main_canvas = FigureCanvasTkAgg(self.main_fig, right_panel)
        self.main_canvas.draw()
        self.main_canvas.get_tk_widget().pack(fill='both', expand=True)

    def create_analysis_tab(self):
        """Performance analysis tab"""
        analysis_frame = ttk.Frame(self.notebook)
        self.notebook.add(analysis_frame, text="Performance Analysis")

        # Create figure for analysis
        self.analysis_fig = Figure(figsize=(12, 8), dpi=100)

        # Efficiency curve
        ax1 = self.analysis_fig.add_subplot(2, 2, 1)
        ax1.set_title('Efficiency vs Load', fontweight='bold')
        ax1.set_xlabel('Load (%)')
        ax1.set_ylabel('Efficiency (%)')
        ax1.grid(True, alpha=0.3)

        # Torque-Speed characteristic
        ax2 = self.analysis_fig.add_subplot(2, 2, 2)
        ax2.set_title('Torque-Speed Characteristic', fontweight='bold')
        ax2.set_xlabel('Speed (RPM)')
        ax2.set_ylabel('Torque (N·m)')
        ax2.grid(True, alpha=0.3)

        # Power breakdown
        ax3 = self.analysis_fig.add_subplot(2, 2, 3)
        ax3.set_title('Power Loss Breakdown', fontweight='bold')
        ax3.set_ylabel('Power (W)')
        ax3.grid(True, alpha=0.3)

        # Current characteristics
        ax4 = self.analysis_fig.add_subplot(2, 2, 4)
        ax4.set_title('Current vs Load', fontweight='bold')
        ax4.set_xlabel('Load (%)')
        ax4.set_ylabel('Current (A)')
        ax4.grid(True, alpha=0.3)

        self.analysis_fig.tight_layout()

        canvas = FigureCanvasTkAgg(self.analysis_fig, analysis_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True)

        # Update button
        update_btn = ttk.Button(analysis_frame, text="Update Analysis",
                               command=self.update_analysis_plots)
        update_btn.pack(pady=5)

    def create_thermal_tab(self):
        """Thermal analysis and derating tab"""
        thermal_frame = ttk.Frame(self.notebook)
        self.notebook.add(thermal_frame, text="Thermal & Derating")

        # Control panel
        control_panel = ttk.Frame(thermal_frame)
        control_panel.pack(side='left', fill='y', padx=10, pady=10)

        # Thermal parameters
        thermal_param_frame = ttk.LabelFrame(control_panel, text="Thermal Parameters", padding=10)
        thermal_param_frame.pack(fill='x', pady=5)

        ttk.Label(thermal_param_frame, text="Ambient Temp (°C):").grid(row=0, column=0, sticky='w')
        self.ambient_temp_var = tk.DoubleVar(value=25)
        ttk.Entry(thermal_param_frame, textvariable=self.ambient_temp_var, width=10).grid(row=0, column=1)

        ttk.Label(thermal_param_frame, text="Thermal Resistance (°C/W):").grid(row=1, column=0, sticky='w')
        self.thermal_r_var = tk.DoubleVar(value=2.0)
        ttk.Entry(thermal_param_frame, textvariable=self.thermal_r_var, width=10).grid(row=1, column=1)

        ttk.Label(thermal_param_frame, text="Thermal Capacitance (J/°C):").grid(row=2, column=0, sticky='w')
        self.thermal_c_var = tk.DoubleVar(value=500.0)
        ttk.Entry(thermal_param_frame, textvariable=self.thermal_c_var, width=10).grid(row=2, column=1)

        ttk.Label(thermal_param_frame, text="Max Temperature (°C):").grid(row=3, column=0, sticky='w')
        self.max_temp_var = tk.DoubleVar(value=120.0)
        ttk.Entry(thermal_param_frame, textvariable=self.max_temp_var, width=10).grid(row=3, column=1)

        # Cooling method
        cooling_frame = ttk.LabelFrame(control_panel, text="Cooling Method", padding=10)
        cooling_frame.pack(fill='x', pady=5)

        self.cooling_var = tk.StringVar(value='Natural')
        cooling_methods = ['Natural', 'Forced Air', 'Liquid Cooling']
        for method in cooling_methods:
            ttk.Radiobutton(cooling_frame, text=method, variable=self.cooling_var,
                          value=method).pack(anchor='w')

        # Plot panel
        plot_panel = ttk.Frame(thermal_frame)
        plot_panel.pack(side='right', fill='both', expand=True, padx=10, pady=10)

        self.thermal_fig = Figure(figsize=(10, 8), dpi=100)

        ax1 = self.thermal_fig.add_subplot(2, 2, 1)
        ax1.set_title('Temperature Rise Profile', fontweight='bold')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Temperature (°C)')
        ax1.grid(True, alpha=0.3)

        ax2 = self.thermal_fig.add_subplot(2, 2, 2)
        ax2.set_title('Derating Curve', fontweight='bold')
        ax2.set_xlabel('Ambient Temperature (°C)')
        ax2.set_ylabel('Max Continuous Power (%)')
        ax2.grid(True, alpha=0.3)

        ax3 = self.thermal_fig.add_subplot(2, 2, 3)
        ax3.set_title('Heat Distribution', fontweight='bold')
        ax3.set_ylabel('Power (W)')
        ax3.grid(True, alpha=0.3)

        ax4 = self.thermal_fig.add_subplot(2, 2, 4)
        ax4.set_title('Thermal Time Constant', fontweight='bold')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Normalized Temperature')
        ax4.grid(True, alpha=0.3)

        self.thermal_fig.tight_layout()

        canvas = FigureCanvasTkAgg(self.thermal_fig, plot_panel)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True)

    def create_economic_tab(self):
        """Economic analysis tab"""
        economic_frame = ttk.Frame(self.notebook)
        self.notebook.add(economic_frame, text="Economic Analysis")

        # Input panel
        input_panel = ttk.Frame(economic_frame)
        input_panel.pack(side='left', fill='y', padx=10, pady=10)

        # Economic parameters
        econ_param_frame = ttk.LabelFrame(input_panel, text="Economic Parameters", padding=10)
        econ_param_frame.pack(fill='x', pady=5)

        ttk.Label(econ_param_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w', pady=2)
        self.cost_kwh_var = tk.DoubleVar(value=0.12)
        ttk.Entry(econ_param_frame, textvariable=self.cost_kwh_var, width=10).grid(row=0, column=1, pady=2)

        ttk.Label(econ_param_frame, text="Operating Hours/Day:").grid(row=1, column=0, sticky='w', pady=2)
        self.hours_day_var = tk.DoubleVar(value=8)
        ttk.Entry(econ_param_frame, textvariable=self.hours_day_var, width=10).grid(row=1, column=1, pady=2)

        ttk.Label(econ_param_frame, text="Operating Days/Year:").grid(row=2, column=0, sticky='w', pady=2)
        self.days_year_var = tk.DoubleVar(value=250)
        ttk.Entry(econ_param_frame, textvariable=self.days_year_var, width=10).grid(row=2, column=1, pady=2)

        ttk.Label(econ_param_frame, text="Average Load (%):").grid(row=3, column=0, sticky='w', pady=2)
        self.avg_load_var = tk.DoubleVar(value=75)
        ttk.Entry(econ_param_frame, textvariable=self.avg_load_var, width=10).grid(row=3, column=1, pady=2)

        ttk.Label(econ_param_frame, text="Motor Cost ($):").grid(row=4, column=0, sticky='w', pady=2)
        self.motor_cost_var = tk.DoubleVar(value=5000)
        ttk.Entry(econ_param_frame, textvariable=self.motor_cost_var, width=10).grid(row=4, column=1, pady=2)

        ttk.Label(econ_param_frame, text="Maintenance Cost ($/year):").grid(row=5, column=0, sticky='w', pady=2)
        self.maint_cost_var = tk.DoubleVar(value=500)
        ttk.Entry(econ_param_frame, textvariable=self.maint_cost_var, width=10).grid(row=5, column=1, pady=2)

        # Calculate button
        calc_btn = ttk.Button(input_panel, text="Calculate Economics",
                            command=self.calculate_economics)
        calc_btn.pack(pady=10, fill='x')

        # Results frame
        results_frame = ttk.LabelFrame(input_panel, text="Annual Cost Analysis", padding=10)
        results_frame.pack(fill='both', expand=True, pady=5)

        self.econ_results = {}
        result_items = [
            ('Annual Energy (kWh)', 'energy'),
            ('Annual Energy Cost ($)', 'energy_cost'),
            ('Annual Maintenance ($)', 'maintenance'),
            ('Total Annual Cost ($)', 'total_annual'),
            ('Cost per Hour ($)', 'cost_hour'),
            ('Payback Period (years)', 'payback')
        ]

        for i, (label, key) in enumerate(result_items):
            ttk.Label(results_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)
            value_label = ttk.Label(results_frame, text="---", font=('Arial', 9, 'bold'))
            value_label.grid(row=i, column=1, sticky='e', pady=2)
            self.econ_results[key] = value_label

        # Plot panel
        plot_panel = ttk.Frame(economic_frame)
        plot_panel.pack(side='right', fill='both', expand=True, padx=10, pady=10)

        self.econ_fig = Figure(figsize=(10, 8), dpi=100)

        ax1 = self.econ_fig.add_subplot(2, 2, 1)
        ax1.set_title('Annual Operating Cost Breakdown', fontweight='bold')

        ax2 = self.econ_fig.add_subplot(2, 2, 2)
        ax2.set_title('Energy Cost vs Efficiency', fontweight='bold')
        ax2.set_xlabel('Efficiency (%)')
        ax2.set_ylabel('Annual Energy Cost ($)')
        ax2.grid(True, alpha=0.3)

        ax3 = self.econ_fig.add_subplot(2, 2, 3)
        ax3.set_title('Cumulative Cost Over Time', fontweight='bold')
        ax3.set_xlabel('Years')
        ax3.set_ylabel('Cumulative Cost ($)')
        ax3.grid(True, alpha=0.3)

        ax4 = self.econ_fig.add_subplot(2, 2, 4)
        ax4.set_title('Cost per Load Level', fontweight='bold')
        ax4.set_xlabel('Load (%)')
        ax4.set_ylabel('Hourly Cost ($)')
        ax4.grid(True, alpha=0.3)

        self.econ_fig.tight_layout()

        self.econ_canvas = FigureCanvasTkAgg(self.econ_fig, plot_panel)
        self.econ_canvas.draw()
        self.econ_canvas.get_tk_widget().pack(fill='both', expand=True)

    def create_multiphysics_tab(self):
        """Multi-physics coupling analysis tab"""
        multi_frame = ttk.Frame(self.notebook)
        self.notebook.add(multi_frame, text="Multi-Physics Analysis")

        # Create figure
        self.multi_fig = Figure(figsize=(12, 10), dpi=100)

        # Electromagnetic field plot
        ax1 = self.multi_fig.add_subplot(3, 2, 1)
        ax1.set_title('Electromagnetic Field Distribution', fontweight='bold')
        ax1.set_xlabel('Position')
        ax1.set_ylabel('Flux Density (T)')
        ax1.grid(True, alpha=0.3)

        # Thermal distribution
        ax2 = self.multi_fig.add_subplot(3, 2, 2)
        ax2.set_title('Temperature Distribution', fontweight='bold')
        ax2.set_xlabel('Position')
        ax2.set_ylabel('Temperature (°C)')
        ax2.grid(True, alpha=0.3)

        # Mechanical stress
        ax3 = self.multi_fig.add_subplot(3, 2, 3)
        ax3.set_title('Shaft Torque Transients', fontweight='bold')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Torque (N·m)')
        ax3.grid(True, alpha=0.3)

        # Loss breakdown
        ax4 = self.multi_fig.add_subplot(3, 2, 4)
        ax4.set_title('Detailed Loss Breakdown', fontweight='bold')

        # Bearing loads
        ax5 = self.multi_fig.add_subplot(3, 2, 5)
        ax5.set_title('Bearing Load Analysis', fontweight='bold')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Load (N)')
        ax5.grid(True, alpha=0.3)

        # Coupled dynamics
        ax6 = self.multi_fig.add_subplot(3, 2, 6)
        ax6.set_title('Coupled Electromagnetic-Thermal Dynamics', fontweight='bold')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Normalized Values')
        ax6.grid(True, alpha=0.3)

        self.multi_fig.tight_layout()

        canvas = FigureCanvasTkAgg(self.multi_fig, multi_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True)

        # Update button
        update_btn = ttk.Button(multi_frame, text="Run Multi-Physics Simulation",
                               command=self.run_multiphysics_simulation)
        update_btn.pack(pady=5)

    def create_control_tab(self):
        """Advanced control strategies tab"""
        control_frame = ttk.Frame(self.notebook)
        self.notebook.add(control_frame, text="Advanced Controls")

        # Control panel
        ctrl_panel = ttk.Frame(control_frame)
        ctrl_panel.pack(side='left', fill='y', padx=10, pady=10)

        # Control method selection
        method_frame = ttk.LabelFrame(ctrl_panel, text="Control Method", padding=10)
        method_frame.pack(fill='x', pady=5)

        self.control_method_var = tk.StringVar(value='Open Loop')
        control_methods = ['Open Loop', 'PI Speed Control', 'Fuzzy Logic', 'Adaptive Control']
        for method in control_methods:
            ttk.Radiobutton(method_frame, text=method, variable=self.control_method_var,
                          value=method).pack(anchor='w')

        # PI Controller parameters
        pi_frame = ttk.LabelFrame(ctrl_panel, text="PI Controller Parameters", padding=10)
        pi_frame.pack(fill='x', pady=5)

        ttk.Label(pi_frame, text="Proportional Gain (Kp):").grid(row=0, column=0, sticky='w')
        self.kp_var = tk.DoubleVar(value=1.0)
        ttk.Entry(pi_frame, textvariable=self.kp_var, width=10).grid(row=0, column=1)

        ttk.Label(pi_frame, text="Integral Gain (Ki):").grid(row=1, column=0, sticky='w')
        self.ki_var = tk.DoubleVar(value=0.5)
        ttk.Entry(pi_frame, textvariable=self.ki_var, width=10).grid(row=1, column=1)

        ttk.Label(pi_frame, text="Speed Setpoint (RPM):").grid(row=2, column=0, sticky='w')
        self.speed_setpoint_var = tk.DoubleVar(value=1500)
        ttk.Entry(pi_frame, textvariable=self.speed_setpoint_var, width=10).grid(row=2, column=1)

        # Plot panel
        plot_panel = ttk.Frame(control_frame)
        plot_panel.pack(side='right', fill='both', expand=True, padx=10, pady=10)

        self.control_fig = Figure(figsize=(10, 8), dpi=100)

        ax1 = self.control_fig.add_subplot(2, 2, 1)
        ax1.set_title('Speed Response', fontweight='bold')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Speed (RPM)')
        ax1.grid(True, alpha=0.3)

        ax2 = self.control_fig.add_subplot(2, 2, 2)
        ax2.set_title('Control Signal', fontweight='bold')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Control Voltage (V)')
        ax2.grid(True, alpha=0.3)

        ax3 = self.control_fig.add_subplot(2, 2, 3)
        ax3.set_title('Speed Error', fontweight='bold')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Error (RPM)')
        ax3.grid(True, alpha=0.3)

        ax4 = self.control_fig.add_subplot(2, 2, 4)
        ax4.set_title('Phase Plane', fontweight='bold')
        ax4.set_xlabel('Speed (RPM)')
        ax4.set_ylabel('Acceleration (RPM/s)')
        ax4.grid(True, alpha=0.3)

        self.control_fig.tight_layout()

        canvas = FigureCanvasTkAgg(self.control_fig, plot_panel)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True)

    def setup_main_plots(self):
        """Setup main plotting area"""
        plots = [
            (self.ax1, 'Armature Current (A)', 'Current (A)'),
            (self.ax2, 'Motor Speed (RPM)', 'Speed (RPM)'),
            (self.ax3, 'Torque (N·m)', 'Torque (N·m)'),
            (self.ax4, 'Efficiency (%)', 'Efficiency (%)'),
            (self.ax5, 'Temperature (°C)', 'Temperature (°C)'),
            (self.ax6, 'Power (kW)', 'Power (kW)')
        ]

        for ax, title, ylabel in plots:
            ax.set_title(title, fontweight='bold', fontsize=9)
            ax.set_xlabel('Time (s)', fontsize=8)
            ax.set_ylabel(ylabel, fontsize=8)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=7)

        self.main_fig.tight_layout()

    def motor_dynamics(self, t, state, V, Ra, La, Rsh, J, B, Kv, Kt, T_load, T_ambient, R_th, C_th):
        """
        Differential equations for DC shunt motor with thermal coupling

        State vector: [Ia, omega, theta, T_motor]
        Ia: Armature current
        omega: Angular velocity
        theta: Angular position
        T_motor: Motor temperature
        """
        Ia, omega, theta, T_motor = state

        # Field current (constant for shunt motor)
        I_sh = V / Rsh

        # Back EMF
        Eb = Kv * omega

        # Electrical equation: V = Eb + Ia*Ra + La*dIa/dt
        dIa_dt = (V - Eb - Ia * Ra) / La

        # Electromagnetic torque
        T_em = Kt * Ia

        # Mechanical equation: T_em - T_load - B*omega = J*domega/dt
        domega_dt = (T_em - T_load - B * omega) / J

        # Angular position
        dtheta_dt = omega

        # Power losses (heat generation)
        P_copper_armature = Ia**2 * Ra
        P_copper_field = I_sh**2 * Rsh
        P_friction = B * omega**2
        P_iron = 0.01 * omega**2  # Simplified iron loss
        P_total_loss = P_copper_armature + P_copper_field + P_friction + P_iron

        # Thermal equation: C_th * dT/dt = P_loss - (T_motor - T_ambient)/R_th
        dT_dt = (P_total_loss - (T_motor - T_ambient) / R_th) / C_th

        return [dIa_dt, domega_dt, dtheta_dt, dT_dt]

    def update_parameters(self):
        """Update motor parameters from GUI"""
        for key, var in self.param_vars.items():
            self.params[key] = var.get()

        self.params['T_ambient'] = self.ambient_temp_var.get()
        self.params['thermal_R'] = self.thermal_r_var.get()
        self.params['thermal_C'] = self.thermal_c_var.get()

    def update_load(self, val=None):
        """Update load torque"""
        self.T_load = self.load_var.get()

    def start_simulation(self):
        """Start the simulation"""
        self.is_running = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.simulation_time = 0
        self.update_parameters()
        self.run_simulation_step()

    def stop_simulation(self):
        """Stop the simulation"""
        self.is_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation to initial conditions"""
        self.is_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        self.simulation_time = 0
        self.state = np.array([0.0, 0.0, 0.0, 25.0])

        # Clear data
        self.time_data = []
        self.current_data = []
        self.speed_data = []
        self.torque_data = []
        self.efficiency_data = []
        self.temperature_data = []
        self.power_data = []

        # Clear plots
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            ax.clear()

        self.setup_main_plots()
        self.main_canvas.draw()

        # Reset status labels
        for label in self.status_labels.values():
            label.config(text="0.00")

    def run_simulation_step(self):
        """Run one step of the simulation"""
        if not self.is_running:
            return

        dt = self.dt_var.get()
        solver_method = self.solver_var.get()

        # Update parameters
        self.update_parameters()

        # Prepare parameters
        params = (
            self.params['V'], self.params['Ra'], self.params['La'],
            self.params['Rsh'], self.params['J'], self.params['B'],
            self.params['Kv'], self.params['Kt'], self.T_load,
            self.params['T_ambient'], self.params['thermal_R'],
            self.params['thermal_C']
        )

        # Solve ODE
        if solver_method == 'Euler':
            # Simple Euler method
            derivatives = self.motor_dynamics(self.simulation_time, self.state, *params)
            self.state = self.state + np.array(derivatives) * dt
        else:
            # Use scipy's ODE solver
            sol = solve_ivp(
                lambda t, y: self.motor_dynamics(t, y, *params),
                [self.simulation_time, self.simulation_time + dt],
                self.state,
                method=solver_method,
                dense_output=True
            )
            self.state = sol.y[:, -1]

        self.simulation_time += dt

        # Extract state variables
        Ia, omega, theta, T_motor = self.state

        # Calculate derived quantities
        speed_rpm = omega * 60 / (2 * np.pi)  # Convert to RPM
        T_em = self.params['Kt'] * Ia

        # Power calculations
        P_out = T_em * omega
        P_in = self.params['V'] * (Ia + self.params['V']/self.params['Rsh'])

        if P_in > 0:
            efficiency = (P_out / P_in) * 100
        else:
            efficiency = 0

        # Store data
        self.time_data.append(self.simulation_time)
        self.current_data.append(Ia)
        self.speed_data.append(speed_rpm)
        self.torque_data.append(T_em)
        self.efficiency_data.append(efficiency)
        self.temperature_data.append(T_motor)
        self.power_data.append(P_out / 1000)  # kW

        # Limit data size
        max_points = 500
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            self.current_data = self.current_data[-max_points:]
            self.speed_data = self.speed_data[-max_points:]
            self.torque_data = self.torque_data[-max_points:]
            self.efficiency_data = self.efficiency_data[-max_points:]
            self.temperature_data = self.temperature_data[-max_points:]
            self.power_data = self.power_data[-max_points:]

        # Update plots
        self.update_main_plots()

        # Update status labels
        self.status_labels['Ia'].config(text=f"{Ia:.2f}")
        self.status_labels['speed'].config(text=f"{speed_rpm:.2f}")
        self.status_labels['torque'].config(text=f"{T_em:.2f}")
        self.status_labels['power'].config(text=f"{P_out/1000:.3f}")
        self.status_labels['efficiency'].config(text=f"{efficiency:.2f}")
        self.status_labels['temp'].config(text=f"{T_motor:.2f}")
        self.status_labels['time'].config(text=f"{self.simulation_time:.2f}")

        # Schedule next step
        self.root.after(10, self.run_simulation_step)

    def update_main_plots(self):
        """Update the main plots with current data"""
        if len(self.time_data) < 2:
            return

        plots_data = [
            (self.ax1, self.time_data, self.current_data, 'b-'),
            (self.ax2, self.time_data, self.speed_data, 'g-'),
            (self.ax3, self.time_data, self.torque_data, 'r-'),
            (self.ax4, self.time_data, self.efficiency_data, 'm-'),
            (self.ax5, self.time_data, self.temperature_data, 'orange'),
            (self.ax6, self.time_data, self.power_data, 'c-')
        ]

        for ax, x_data, y_data, color in plots_data:
            ax.clear()
            ax.plot(x_data, y_data, color, linewidth=1.5)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=7)

        # Restore titles and labels
        titles = [
            'Armature Current (A)', 'Motor Speed (RPM)', 'Torque (N·m)',
            'Efficiency (%)', 'Temperature (°C)', 'Power (kW)'
        ]

        for ax, title in zip([self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6], titles):
            ax.set_title(title, fontweight='bold', fontsize=9)
            ax.set_xlabel('Time (s)', fontsize=8)

        self.main_fig.tight_layout()
        self.main_canvas.draw()

    def update_analysis_plots(self):
        """Update performance analysis plots"""
        # Generate efficiency vs load curve
        loads = np.linspace(0, 100, 50)
        efficiencies = []
        currents = []
        speeds = []
        torques = []

        for load_pct in loads:
            # Calculate operating point
            T_load = (load_pct / 100) * (self.params['P_rated'] / 100)  # Simplified

            # Steady-state solution (simplified)
            I_a = max(0, T_load / self.params['Kt'])
            omega = (self.params['V'] - I_a * self.params['Ra']) / self.params['Kv']

            P_out = T_load * omega
            I_sh = self.params['V'] / self.params['Rsh']
            P_in = self.params['V'] * (I_a + I_sh)

            if P_in > 0:
                eff = (P_out / P_in) * 100
            else:
                eff = 0

            efficiencies.append(eff)
            currents.append(I_a)
            speeds.append(omega * 60 / (2 * np.pi))
            torques.append(T_load)

        # Clear and update plots
        self.analysis_fig.clear()

        ax1 = self.analysis_fig.add_subplot(2, 2, 1)
        ax1.plot(loads, efficiencies, 'b-', linewidth=2)
        ax1.set_title('Efficiency vs Load', fontweight='bold')
        ax1.set_xlabel('Load (%)')
        ax1.set_ylabel('Efficiency (%)')
        ax1.grid(True, alpha=0.3)

        ax2 = self.analysis_fig.add_subplot(2, 2, 2)
        ax2.plot(speeds, torques, 'r-', linewidth=2)
        ax2.set_title('Torque-Speed Characteristic', fontweight='bold')
        ax2.set_xlabel('Speed (RPM)')
        ax2.set_ylabel('Torque (N·m)')
        ax2.grid(True, alpha=0.3)

        ax3 = self.analysis_fig.add_subplot(2, 2, 3)
        loss_categories = ['Copper\n(Armature)', 'Copper\n(Field)', 'Iron', 'Friction', 'Stray']
        loss_values = [500, 250, 300, 150, 100]  # Example values
        ax3.bar(loss_categories, loss_values, color=['red', 'orange', 'yellow', 'green', 'blue'])
        ax3.set_title('Power Loss Breakdown', fontweight='bold')
        ax3.set_ylabel('Power (W)')
        ax3.grid(True, alpha=0.3, axis='y')

        ax4 = self.analysis_fig.add_subplot(2, 2, 4)
        ax4.plot(loads, currents, 'g-', linewidth=2)
        ax4.set_title('Current vs Load', fontweight='bold')
        ax4.set_xlabel('Load (%)')
        ax4.set_ylabel('Current (A)')
        ax4.grid(True, alpha=0.3)

        self.analysis_fig.tight_layout()
        self.analysis_fig.canvas.draw()

    def calculate_economics(self):
        """Calculate economic analysis"""
        cost_kwh = self.cost_kwh_var.get()
        hours_day = self.hours_day_var.get()
        days_year = self.days_year_var.get()
        avg_load = self.avg_load_var.get() / 100
        motor_cost = self.motor_cost_var.get()
        maint_cost = self.maint_cost_var.get()

        # Calculate annual energy consumption
        avg_power = self.params['P_rated'] / 1000 * avg_load  # kW
        annual_hours = hours_day * days_year
        annual_energy = avg_power * annual_hours  # kWh

        # Calculate costs
        annual_energy_cost = annual_energy * cost_kwh
        total_annual_cost = annual_energy_cost + maint_cost
        cost_per_hour = total_annual_cost / annual_hours

        # Simple payback (assume comparing to less efficient motor)
        savings = annual_energy_cost * 0.1  # Assume 10% energy savings
        if savings > 0:
            payback = motor_cost / savings
        else:
            payback = 999

        # Update results
        self.econ_results['energy'].config(text=f"{annual_energy:.0f}")
        self.econ_results['energy_cost'].config(text=f"{annual_energy_cost:.2f}")
        self.econ_results['maintenance'].config(text=f"{maint_cost:.2f}")
        self.econ_results['total_annual'].config(text=f"{total_annual_cost:.2f}")
        self.econ_results['cost_hour'].config(text=f"{cost_per_hour:.2f}")
        self.econ_results['payback'].config(text=f"{payback:.1f}")

        # Update plots
        self.update_economic_plots(annual_energy_cost, maint_cost, cost_kwh)

    def update_economic_plots(self, energy_cost, maint_cost, cost_kwh):
        """Update economic analysis plots"""
        self.econ_fig.clear()

        # Cost breakdown pie chart
        ax1 = self.econ_fig.add_subplot(2, 2, 1)
        costs = [energy_cost, maint_cost]
        labels = ['Energy Cost', 'Maintenance']
        colors = ['#ff9999', '#66b3ff']
        ax1.pie(costs, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax1.set_title('Annual Operating Cost Breakdown', fontweight='bold')

        # Energy cost vs efficiency
        ax2 = self.econ_fig.add_subplot(2, 2, 2)
        efficiencies = np.linspace(70, 95, 20)
        costs_by_eff = energy_cost / (efficiencies / 85)  # Normalized to current efficiency
        ax2.plot(efficiencies, costs_by_eff, 'b-', linewidth=2)
        ax2.set_title('Energy Cost vs Efficiency', fontweight='bold')
        ax2.set_xlabel('Efficiency (%)')
        ax2.set_ylabel('Annual Energy Cost ($)')
        ax2.grid(True, alpha=0.3)

        # Cumulative cost over time
        ax3 = self.econ_fig.add_subplot(2, 2, 3)
        years = np.arange(0, 11)
        cumulative = years * (energy_cost + maint_cost)
        ax3.plot(years, cumulative, 'g-', linewidth=2, marker='o')
        ax3.set_title('Cumulative Cost Over Time', fontweight='bold')
        ax3.set_xlabel('Years')
        ax3.set_ylabel('Cumulative Cost ($)')
        ax3.grid(True, alpha=0.3)

        # Cost per load level
        ax4 = self.econ_fig.add_subplot(2, 2, 4)
        load_levels = np.linspace(25, 100, 20)
        hourly_costs = (load_levels / 100) * (self.params['P_rated'] / 1000) * cost_kwh
        ax4.plot(load_levels, hourly_costs, 'r-', linewidth=2)
        ax4.set_title('Cost per Load Level', fontweight='bold')
        ax4.set_xlabel('Load (%)')
        ax4.set_ylabel('Hourly Cost ($)')
        ax4.grid(True, alpha=0.3)

        self.econ_fig.tight_layout()
        self.econ_canvas.draw()

    def run_multiphysics_simulation(self):
        """Run comprehensive multi-physics simulation"""
        # This would normally involve FEA/FEM simulations
        # Here we'll create representative visualizations

        self.multi_fig.clear()

        # Electromagnetic field distribution
        ax1 = self.multi_fig.add_subplot(3, 2, 1)
        theta = np.linspace(0, 2*np.pi, 100)
        flux_density = 1.2 * np.sin(2*theta) + 0.8  # Simplified B-field
        ax1.plot(theta, flux_density, 'b-', linewidth=2)
        ax1.fill_between(theta, 0, flux_density, alpha=0.3)
        ax1.set_title('Electromagnetic Field Distribution', fontweight='bold')
        ax1.set_xlabel('Angular Position (rad)')
        ax1.set_ylabel('Flux Density (T)')
        ax1.grid(True, alpha=0.3)

        # Temperature distribution
        ax2 = self.multi_fig.add_subplot(3, 2, 2)
        position = np.linspace(0, 1, 50)
        temp_dist = 100 * np.exp(-position**2 / 0.1) + 25  # Gaussian temperature distribution
        ax2.plot(position, temp_dist, 'r-', linewidth=2)
        ax2.fill_between(position, 25, temp_dist, alpha=0.3, color='red')
        ax2.set_title('Temperature Distribution', fontweight='bold')
        ax2.set_xlabel('Normalized Position')
        ax2.set_ylabel('Temperature (°C)')
        ax2.grid(True, alpha=0.3)

        # Shaft torque transients
        ax3 = self.multi_fig.add_subplot(3, 2, 3)
        if len(self.time_data) > 0:
            ax3.plot(self.time_data, self.torque_data, 'g-', linewidth=1.5)
        ax3.set_title('Shaft Torque Transients', fontweight='bold')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Torque (N·m)')
        ax3.grid(True, alpha=0.3)

        # Loss breakdown pie chart
        ax4 = self.multi_fig.add_subplot(3, 2, 4)
        losses = {
            'Copper (Armature)': 507,
            'Copper (Field)': 250,
            'Iron Core': 300,
            'Mechanical Friction': 150,
            'Stray Load': 100
        }
        colors_pie = ['#ff6b6b', '#ff8c42', '#ffd93d', '#6bcf7f', '#4d96ff']
        ax4.pie(losses.values(), labels=losses.keys(), autopct='%1.1f%%',
               colors=colors_pie, startangle=90)
        ax4.set_title('Detailed Loss Breakdown', fontweight='bold')

        # Bearing loads
        ax5 = self.multi_fig.add_subplot(3, 2, 5)
        if len(self.time_data) > 0:
            bearing_load = np.array(self.torque_data) * 10 + 100  # Simplified bearing load
            ax5.plot(self.time_data, bearing_load, 'purple', linewidth=1.5)
        ax5.set_title('Bearing Load Analysis', fontweight='bold')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Load (N)')
        ax5.grid(True, alpha=0.3)

        # Coupled dynamics
        ax6 = self.multi_fig.add_subplot(3, 2, 6)
        if len(self.time_data) > 0:
            # Normalize data for comparison
            norm_current = np.array(self.current_data) / max(self.current_data) if max(self.current_data) > 0 else [0]
            norm_temp = (np.array(self.temperature_data) - 25) / 75  # Normalize temperature
            ax6.plot(self.time_data, norm_current, 'b-', label='Normalized Current', linewidth=1.5)
            ax6.plot(self.time_data, norm_temp, 'r-', label='Normalized Temperature', linewidth=1.5)
            ax6.legend()
        ax6.set_title('Coupled Electromagnetic-Thermal Dynamics', fontweight='bold')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Normalized Values')
        ax6.grid(True, alpha=0.3)

        self.multi_fig.tight_layout()
        self.multi_fig.canvas.draw()

    def on_window_resize(self, event=None):
        """Handle window resize for auto-scaling"""
        # This is called on window resize
        # The tight_layout() in matplotlib handles most of the auto-scaling
        pass

def main():
    root = tk.Tk()

    # Configure style
    style = ttk.Style()
    style.theme_use('clam')

    # Custom colors
    style.configure('TFrame', background='#f0f0f0')
    style.configure('TLabel', background='#f0f0f0', font=('Arial', 9))
    style.configure('TLabelframe', background='#f0f0f0', font=('Arial', 9, 'bold'))
    style.configure('TLabelframe.Label', background='#f0f0f0', font=('Arial', 9, 'bold'))

    app = DCShuntMotorSimulator(root)

    root.mainloop()

if __name__ == "__main__":
    main()
