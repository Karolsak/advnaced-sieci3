"""
Advanced DC Motor Simulator with Multi-Physics Analysis
Includes: Electromagnetic, Thermal, Mechanical, Acoustic, and Economic Analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import scipy.integrate as integrate
from datetime import datetime
import math

class DCMotorSimulator:
    """
    Comprehensive DC Motor Simulator with Multi-Physics Modeling
    """

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Initialize simulation parameters
        self.init_parameters()

        # Create GUI
        self.create_gui()

        # Bind window resize event
        self.root.bind("<Configure>", self.on_window_resize)

        # Simulation state
        self.is_running = False
        self.simulation_time = 0
        self.time_data = []
        self.speed_data = []
        self.torque_data = []
        self.current_data = []
        self.temp_data = []
        self.efficiency_data = []

    def init_parameters(self):
        """Initialize motor parameters and simulation settings"""
        # Motor parameters
        self.motor_type = "Shunt"
        self.poles = 6
        self.voltage = 500.0
        self.armature_conductors = 1200
        self.flux_per_pole = 0.02  # Wb
        self.Ra = 0.5  # Armature resistance (Ω)
        self.Rf = 250.0  # Field resistance (Ω)
        self.supply_current = 20.0  # A
        self.winding_type = "Wave"  # Wave or Lap

        # Additional parameters for dynamic simulation
        self.J = 0.5  # Moment of inertia (kg·m²)
        self.B = 0.01  # Friction coefficient (N·m·s)
        self.La = 0.05  # Armature inductance (H)
        self.Lf = 10.0  # Field inductance (H)

        # Thermal parameters
        self.thermal_resistance = 2.0  # K/W
        self.thermal_capacitance = 500.0  # J/K
        self.ambient_temp = 25.0  # °C
        self.max_temp = 155.0  # °C (Class F insulation)

        # Economic parameters
        self.electricity_cost = 0.12  # $/kWh
        self.maintenance_cost = 100.0  # $/year
        self.initial_cost = 5000.0  # $

        # Simulation parameters
        self.dt = 0.001  # Time step (s)
        self.solver_type = "RK45"  # RK45 or Euler

        # Load torque
        self.load_torque = 100.0  # N·m

    def create_gui(self):
        """Create the main GUI with tabs"""
        # Create main container with grid that expands
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Configure grid weights for responsiveness
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Create notebook for tabs
        self.notebook = ttk.Notebook(main_container)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Create tabs
        self.tab_main = ttk.Frame(self.notebook)
        self.tab_simulation = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)
        self.tab_advanced = ttk.Frame(self.notebook)
        self.tab_multiphysics = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_main, text="Main Control")
        self.notebook.add(self.tab_simulation, text="Dynamic Simulation")
        self.notebook.add(self.tab_economic, text="Economic Analysis")
        self.notebook.add(self.tab_advanced, text="Advanced Controls")
        self.notebook.add(self.tab_multiphysics, text="Multi-Physics")

        # Build each tab
        self.build_main_tab()
        self.build_simulation_tab()
        self.build_economic_tab()
        self.build_advanced_tab()
        self.build_multiphysics_tab()

    def build_main_tab(self):
        """Build the main control tab"""
        # Configure grid
        self.tab_main.grid_rowconfigure(1, weight=1)
        self.tab_main.grid_columnconfigure(0, weight=1)
        self.tab_main.grid_columnconfigure(1, weight=2)

        # Left panel - Parameters
        left_frame = ttk.LabelFrame(self.tab_main, text="Motor Parameters")
        left_frame.grid(row=0, column=0, rowspan=2, padx=5, pady=5, sticky="nsew")

        # Motor type selection
        ttk.Label(left_frame, text="Motor Type:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.motor_type_var = tk.StringVar(value="Shunt")
        motor_combo = ttk.Combobox(left_frame, textvariable=self.motor_type_var,
                                   values=["Shunt", "Series", "Compound"], state="readonly")
        motor_combo.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        motor_combo.bind("<<ComboboxSelected>>", self.on_motor_type_change)

        # Winding type
        ttk.Label(left_frame, text="Winding Type:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.winding_var = tk.StringVar(value="Wave")
        winding_combo = ttk.Combobox(left_frame, textvariable=self.winding_var,
                                     values=["Wave", "Lap"], state="readonly")
        winding_combo.grid(row=1, column=1, padx=5, pady=2, sticky="ew")

        # Input parameters with sliders
        params = [
            ("Poles:", "poles", 2, 12, 2),
            ("Voltage (V):", "voltage", 100, 1000, 100),
            ("Armature Conductors:", "conductors", 100, 2000, 100),
            ("Flux/Pole (mWb):", "flux", 5, 50, 5),
            ("Armature Resistance (Ω):", "Ra", 0.1, 5.0, 0.1),
            ("Field Resistance (Ω):", "Rf", 50, 500, 50),
            ("Supply Current (A):", "current", 5, 100, 5),
            ("Load Torque (N·m):", "load", 0, 500, 50),
        ]

        self.param_vars = {}
        self.param_scales = {}

        for i, (label, key, min_val, max_val, step) in enumerate(params, start=2):
            ttk.Label(left_frame, text=label).grid(row=i, column=0, sticky="w", padx=5, pady=2)

            var = tk.DoubleVar(value=getattr(self, key.replace("flux", "flux_per_pole").replace("current", "supply_current").replace("conductors", "armature_conductors").replace("load", "load_torque")) if key in ["flux", "current", "conductors", "load"] else getattr(self, key))
            if key == "flux":
                var.set(self.flux_per_pole * 1000)  # Convert to mWb

            scale = ttk.Scale(left_frame, from_=min_val, to=max_val,
                            variable=var, orient=tk.HORIZONTAL)
            scale.grid(row=i, column=1, padx=5, pady=2, sticky="ew")

            value_label = ttk.Label(left_frame, textvariable=var)
            value_label.grid(row=i, column=2, padx=5, pady=2)

            self.param_vars[key] = var
            self.param_scales[key] = scale

        # Configure column weights
        left_frame.grid_columnconfigure(1, weight=1)

        # Control buttons
        button_frame = ttk.Frame(left_frame)
        button_frame.grid(row=len(params)+2, column=0, columnspan=3, pady=10)

        self.start_btn = ttk.Button(button_frame, text="Start", command=self.start_simulation)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(button_frame, text="Stop", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.reset_btn = ttk.Button(button_frame, text="Reset", command=self.reset_simulation)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        self.calculate_btn = ttk.Button(button_frame, text="Calculate Steady-State",
                                       command=self.calculate_steady_state)
        self.calculate_btn.pack(side=tk.LEFT, padx=5)

        # Right panel - Results
        right_frame = ttk.LabelFrame(self.tab_main, text="Calculation Results")
        right_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        self.results_text = tk.Text(right_frame, height=15, width=50)
        self.results_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        scrollbar = ttk.Scrollbar(right_frame, command=self.results_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Graphs panel
        graph_frame = ttk.LabelFrame(self.tab_main, text="Real-Time Performance")
        graph_frame.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")

        # Create figure for main graphs
        self.fig_main = Figure(figsize=(8, 4), dpi=80)
        self.ax_speed = self.fig_main.add_subplot(121)
        self.ax_torque = self.fig_main.add_subplot(122)

        self.canvas_main = FigureCanvasTkAgg(self.fig_main, graph_frame)
        self.canvas_main.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.setup_main_graphs()

    def build_simulation_tab(self):
        """Build the dynamic simulation tab"""
        self.tab_simulation.grid_rowconfigure(1, weight=1)
        self.tab_simulation.grid_columnconfigure(0, weight=1)

        # Simulation settings
        settings_frame = ttk.LabelFrame(self.tab_simulation, text="Simulation Settings")
        settings_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        ttk.Label(settings_frame, text="ODE Solver:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.solver_var = tk.StringVar(value="RK45")
        solver_combo = ttk.Combobox(settings_frame, textvariable=self.solver_var,
                                    values=["RK45", "Euler", "RK23"], state="readonly")
        solver_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(settings_frame, text="Time Step (s):").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.dt_var = tk.DoubleVar(value=0.001)
        dt_entry = ttk.Entry(settings_frame, textvariable=self.dt_var, width=10)
        dt_entry.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(settings_frame, text="Simulation Duration (s):").grid(row=0, column=4, padx=5, pady=5, sticky="w")
        self.duration_var = tk.DoubleVar(value=5.0)
        duration_entry = ttk.Entry(settings_frame, textvariable=self.duration_var, width=10)
        duration_entry.grid(row=0, column=5, padx=5, pady=5)

        # Graphs
        graph_frame = ttk.LabelFrame(self.tab_simulation, text="Dynamic Response")
        graph_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        self.fig_sim = Figure(figsize=(12, 8), dpi=80)
        self.ax_sim_speed = self.fig_sim.add_subplot(231)
        self.ax_sim_current = self.fig_sim.add_subplot(232)
        self.ax_sim_torque = self.fig_sim.add_subplot(233)
        self.ax_sim_power = self.fig_sim.add_subplot(234)
        self.ax_sim_efficiency = self.fig_sim.add_subplot(235)
        self.ax_sim_temp = self.fig_sim.add_subplot(236)

        self.canvas_sim = FigureCanvasTkAgg(self.fig_sim, graph_frame)
        self.canvas_sim.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.setup_simulation_graphs()

    def build_economic_tab(self):
        """Build the economic analysis tab"""
        self.tab_economic.grid_rowconfigure(1, weight=1)
        self.tab_economic.grid_columnconfigure(0, weight=1)

        # Economic parameters
        params_frame = ttk.LabelFrame(self.tab_economic, text="Economic Parameters")
        params_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        ttk.Label(params_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(params_frame, textvariable=self.elec_cost_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(params_frame, text="Operating Hours/Year:").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.op_hours_var = tk.DoubleVar(value=4000)
        ttk.Entry(params_frame, textvariable=self.op_hours_var, width=15).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(params_frame, text="Maintenance Cost ($/year):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.maint_cost_var = tk.DoubleVar(value=100)
        ttk.Entry(params_frame, textvariable=self.maint_cost_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(params_frame, text="Initial Cost ($):").grid(row=1, column=2, padx=5, pady=5, sticky="w")
        self.init_cost_var = tk.DoubleVar(value=5000)
        ttk.Entry(params_frame, textvariable=self.init_cost_var, width=15).grid(row=1, column=3, padx=5, pady=5)

        ttk.Button(params_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=2, column=0, columnspan=4, pady=10)

        # Results
        results_frame = ttk.LabelFrame(self.tab_economic, text="Economic Analysis Results")
        results_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        self.economic_text = tk.Text(results_frame, height=20, width=80)
        self.economic_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Graphs
        self.fig_econ = Figure(figsize=(12, 4), dpi=80)
        self.ax_cost = self.fig_econ.add_subplot(121)
        self.ax_payback = self.fig_econ.add_subplot(122)

        canvas_econ = FigureCanvasTkAgg(self.fig_econ, results_frame)
        canvas_econ.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def build_advanced_tab(self):
        """Build the advanced controls tab"""
        self.tab_advanced.grid_rowconfigure(2, weight=1)
        self.tab_advanced.grid_columnconfigure(0, weight=1)

        # Thermal management
        thermal_frame = ttk.LabelFrame(self.tab_advanced, text="Thermal Management & Derating")
        thermal_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        ttk.Label(thermal_frame, text="Thermal Resistance (K/W):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.therm_res_var = tk.DoubleVar(value=2.0)
        ttk.Entry(thermal_frame, textvariable=self.therm_res_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(thermal_frame, text="Max Temperature (°C):").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.max_temp_var = tk.DoubleVar(value=155)
        ttk.Entry(thermal_frame, textvariable=self.max_temp_var, width=15).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(thermal_frame, text="Ambient Temp (°C):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.amb_temp_var = tk.DoubleVar(value=25)
        ttk.Entry(thermal_frame, textvariable=self.amb_temp_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        self.derating_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(thermal_frame, text="Enable Auto Derating",
                       variable=self.derating_var).grid(row=1, column=2, columnspan=2, padx=5, pady=5)

        # Control methods
        control_frame = ttk.LabelFrame(self.tab_advanced, text="Advanced Control Methods")
        control_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        self.control_method_var = tk.StringVar(value="Constant Voltage")
        controls = ["Constant Voltage", "PWM Control", "Field Weakening", "Armature Voltage Control"]

        for i, method in enumerate(controls):
            ttk.Radiobutton(control_frame, text=method, variable=self.control_method_var,
                           value=method).grid(row=0, column=i, padx=10, pady=5)

        ttk.Label(control_frame, text="PWM Duty Cycle (%):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.pwm_duty_var = tk.DoubleVar(value=100)
        pwm_scale = ttk.Scale(control_frame, from_=0, to=100, variable=self.pwm_duty_var, orient=tk.HORIZONTAL)
        pwm_scale.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        ttk.Label(control_frame, textvariable=self.pwm_duty_var).grid(row=1, column=3, padx=5, pady=5)

        # Power consumption monitoring
        power_frame = ttk.LabelFrame(self.tab_advanced, text="Power Consumption Monitoring")
        power_frame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")

        self.power_text = tk.Text(power_frame, height=15, width=80)
        self.power_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        ttk.Button(power_frame, text="Analyze Power Consumption",
                  command=self.analyze_power).pack(pady=5)

    def build_multiphysics_tab(self):
        """Build the multi-physics simulation tab"""
        self.tab_multiphysics.grid_rowconfigure(1, weight=1)
        self.tab_multiphysics.grid_columnconfigure(0, weight=1)

        # Analysis options
        options_frame = ttk.LabelFrame(self.tab_multiphysics, text="Multi-Physics Analysis Options")
        options_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        self.em_thermal_var = tk.BooleanVar(value=True)
        self.mechanical_var = tk.BooleanVar(value=True)
        self.acoustic_var = tk.BooleanVar(value=True)
        self.efficiency_map_var = tk.BooleanVar(value=True)
        self.loss_analysis_var = tk.BooleanVar(value=True)

        ttk.Checkbutton(options_frame, text="Electromagnetic-Thermal Coupling",
                       variable=self.em_thermal_var).grid(row=0, column=0, padx=10, pady=5, sticky="w")
        ttk.Checkbutton(options_frame, text="Mechanical Stress Analysis",
                       variable=self.mechanical_var).grid(row=0, column=1, padx=10, pady=5, sticky="w")
        ttk.Checkbutton(options_frame, text="Acoustic Noise Prediction",
                       variable=self.acoustic_var).grid(row=0, column=2, padx=10, pady=5, sticky="w")
        ttk.Checkbutton(options_frame, text="Efficiency Mapping",
                       variable=self.efficiency_map_var).grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ttk.Checkbutton(options_frame, text="Detailed Loss Breakdown",
                       variable=self.loss_analysis_var).grid(row=1, column=1, padx=10, pady=5, sticky="w")

        ttk.Button(options_frame, text="Run Multi-Physics Analysis",
                  command=self.run_multiphysics_analysis).grid(row=2, column=0, columnspan=3, pady=10)

        # Results
        results_frame = ttk.LabelFrame(self.tab_multiphysics, text="Multi-Physics Results")
        results_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        # Create notebook for different analysis results
        self.multiphysics_notebook = ttk.Notebook(results_frame)
        self.multiphysics_notebook.pack(fill=tk.BOTH, expand=True)

        # Thermal tab
        thermal_tab = ttk.Frame(self.multiphysics_notebook)
        self.multiphysics_notebook.add(thermal_tab, text="Thermal")

        self.fig_thermal = Figure(figsize=(10, 6), dpi=80)
        self.ax_temp_dist = self.fig_thermal.add_subplot(121)
        self.ax_heat_flow = self.fig_thermal.add_subplot(122)
        canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, thermal_tab)
        canvas_thermal.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Mechanical tab
        mechanical_tab = ttk.Frame(self.multiphysics_notebook)
        self.multiphysics_notebook.add(mechanical_tab, text="Mechanical")

        self.fig_mechanical = Figure(figsize=(10, 6), dpi=80)
        self.ax_stress = self.fig_mechanical.add_subplot(121)
        self.ax_bearing = self.fig_mechanical.add_subplot(122)
        canvas_mechanical = FigureCanvasTkAgg(self.fig_mechanical, mechanical_tab)
        canvas_mechanical.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Acoustic tab
        acoustic_tab = ttk.Frame(self.multiphysics_notebook)
        self.multiphysics_notebook.add(acoustic_tab, text="Acoustic")

        self.fig_acoustic = Figure(figsize=(10, 6), dpi=80)
        self.ax_noise = self.fig_acoustic.add_subplot(121)
        self.ax_vibration = self.fig_acoustic.add_subplot(122)
        canvas_acoustic = FigureCanvasTkAgg(self.fig_acoustic, acoustic_tab)
        canvas_acoustic.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Efficiency map tab
        efficiency_tab = ttk.Frame(self.multiphysics_notebook)
        self.multiphysics_notebook.add(efficiency_tab, text="Efficiency Map")

        self.fig_efficiency = Figure(figsize=(10, 6), dpi=80)
        self.ax_eff_map = self.fig_efficiency.add_subplot(111)
        canvas_efficiency = FigureCanvasTkAgg(self.fig_efficiency, efficiency_tab)
        canvas_efficiency.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Loss breakdown tab
        loss_tab = ttk.Frame(self.multiphysics_notebook)
        self.multiphysics_notebook.add(loss_tab, text="Loss Analysis")

        self.fig_loss = Figure(figsize=(10, 6), dpi=80)
        self.ax_loss_pie = self.fig_loss.add_subplot(121)
        self.ax_loss_bar = self.fig_loss.add_subplot(122)
        canvas_loss = FigureCanvasTkAgg(self.fig_loss, loss_tab)
        canvas_loss.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_main_graphs(self):
        """Setup the main graphs"""
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.set_title('Speed vs Time')
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Torque vs Time')
        self.ax_torque.grid(True, alpha=0.3)

        self.fig_main.tight_layout()

    def setup_simulation_graphs(self):
        """Setup the simulation graphs"""
        titles = ['Speed (rpm)', 'Current (A)', 'Torque (N·m)',
                 'Power (W)', 'Efficiency (%)', 'Temperature (°C)']
        axes = [self.ax_sim_speed, self.ax_sim_current, self.ax_sim_torque,
               self.ax_sim_power, self.ax_sim_efficiency, self.ax_sim_temp]

        for ax, title in zip(axes, titles):
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(title)
            ax.set_title(title + ' vs Time')
            ax.grid(True, alpha=0.3)

        self.fig_sim.tight_layout()

    def on_motor_type_change(self, event=None):
        """Handle motor type change"""
        motor_type = self.motor_type_var.get()
        if motor_type == "Series":
            self.param_scales['Rf'].config(state=tk.DISABLED)
        else:
            self.param_scales['Rf'].config(state=tk.NORMAL)

    def on_window_resize(self, event=None):
        """Handle window resize event for auto-scaling"""
        if event and event.widget == self.root:
            # Redraw canvases
            try:
                self.canvas_main.draw_idle()
                self.canvas_sim.draw_idle()
            except:
                pass

    def update_parameters_from_gui(self):
        """Update internal parameters from GUI values"""
        self.poles = int(self.param_vars['poles'].get())
        self.voltage = self.param_vars['voltage'].get()
        self.armature_conductors = int(self.param_vars['conductors'].get())
        self.flux_per_pole = self.param_vars['flux'].get() / 1000.0  # Convert from mWb to Wb
        self.Ra = self.param_vars['Ra'].get()
        self.Rf = self.param_vars['Rf'].get()
        self.supply_current = self.param_vars['current'].get()
        self.load_torque = self.param_vars['load'].get()
        self.motor_type = self.motor_type_var.get()
        self.winding_type = self.winding_var.get()

    def calculate_steady_state(self):
        """Calculate steady-state motor performance"""
        self.update_parameters_from_gui()

        # Number of parallel paths
        A = 2 if self.winding_type == "Wave" else self.poles

        # Calculations based on motor type
        if self.motor_type == "Shunt":
            If = self.voltage / self.Rf
            Ia = self.supply_current - If
        elif self.motor_type == "Series":
            Ia = self.supply_current
            If = Ia
        else:  # Compound
            If = self.voltage / self.Rf + self.supply_current
            Ia = self.supply_current

        # Back EMF
        Eb = self.voltage - Ia * self.Ra

        # Speed (RPM)
        if self.flux_per_pole > 0:
            N = (Eb * A * 60) / (self.poles * self.armature_conductors * self.flux_per_pole)
        else:
            N = 0

        # Torque (N·m)
        T = (self.poles * self.armature_conductors * self.flux_per_pole * Ia) / (2 * np.pi * A)

        # Power calculations
        P_input = self.voltage * self.supply_current
        P_output = (2 * np.pi * N / 60) * T

        # Losses
        copper_loss_armature = Ia**2 * self.Ra
        copper_loss_field = If**2 * self.Rf if self.motor_type != "Series" else 0
        total_copper_loss = copper_loss_armature + copper_loss_field

        # Estimate iron losses (hysteresis + eddy current)
        # Ph ∝ f * B^1.6 (hysteresis), Pe ∝ f^2 * B^2 (eddy current)
        f = (N * self.poles) / 120  # Frequency in Hz
        B = self.flux_per_pole / 0.05  # Approximate flux density (assuming 0.05 m² pole area)
        iron_loss = 0.01 * (f * B**1.6 + 0.001 * f**2 * B**2)  # Empirical formula

        # Mechanical losses (friction and windage)
        mechanical_loss = 0.01 * P_output  # Assume 1% of output power

        # Stray load losses
        stray_loss = 0.01 * P_output  # Assume 1% of output power

        total_loss = total_copper_loss + iron_loss + mechanical_loss + stray_loss

        # Efficiency
        efficiency = (P_output / P_input * 100) if P_input > 0 else 0

        # Thermal calculations
        temp_rise = total_loss * self.thermal_resistance
        operating_temp = self.ambient_temp + temp_rise

        # Display results
        results = f"""
{'='*60}
STEADY-STATE DC MOTOR ANALYSIS
{'='*60}
Motor Type: {self.motor_type} Motor ({self.winding_type} Winding)
Poles: {self.poles}
Supply Voltage: {self.voltage:.2f} V
Supply Current: {self.supply_current:.2f} A
{'='*60}

ELECTRICAL PARAMETERS:
{'─'*60}
Armature Conductors (Z): {self.armature_conductors}
Parallel Paths (A): {A}
Flux per Pole: {self.flux_per_pole*1000:.2f} mWb ({self.flux_per_pole:.4f} Wb)
Armature Resistance (Ra): {self.Ra:.2f} Ω
Field Resistance (Rf): {self.Rf:.2f} Ω

OPERATING POINT:
{'─'*60}
Field Current (If): {If:.3f} A
Armature Current (Ia): {Ia:.3f} A
Back EMF (Eb): {Eb:.2f} V

PERFORMANCE METRICS:
{'─'*60}
Speed: {N:.2f} rpm ({N/60:.2f} rps)
Angular Velocity: {2*np.pi*N/60:.2f} rad/s
Developed Torque: {T:.2f} N·m
Load Torque: {self.load_torque:.2f} N·m

POWER ANALYSIS:
{'─'*60}
Input Power: {P_input:.2f} W ({P_input/1000:.3f} kW)
Output Power: {P_output:.2f} W ({P_output/1000:.3f} kW)
Efficiency: {efficiency:.2f} %

DETAILED LOSS BREAKDOWN:
{'─'*60}
Copper Losses:
  - Armature: {copper_loss_armature:.2f} W
  - Field: {copper_loss_field:.2f} W
  - Total Copper Loss: {total_copper_loss:.2f} W

Iron Losses:
  - Core Frequency: {f:.2f} Hz
  - Flux Density: {B:.3f} T
  - Total Iron Loss: {iron_loss:.2f} W

Mechanical Losses: {mechanical_loss:.2f} W
Stray Load Losses: {stray_loss:.2f} W

TOTAL LOSSES: {total_loss:.2f} W ({total_loss/1000:.3f} kW)

THERMAL ANALYSIS:
{'─'*60}
Ambient Temperature: {self.ambient_temp:.1f} °C
Temperature Rise: {temp_rise:.2f} K
Operating Temperature: {operating_temp:.2f} °C
Maximum Rated Temp: {self.max_temp:.1f} °C
Thermal Margin: {self.max_temp - operating_temp:.2f} K
Status: {'⚠ OVERHEATING!' if operating_temp > self.max_temp else '✓ SAFE'}

TORQUE-SPEED CHARACTERISTICS:
{'─'*60}
Torque Constant (Kt): {T/Ia if Ia > 0 else 0:.4f} N·m/A
Speed Constant (Kv): {N/Eb if Eb > 0 else 0:.4f} rpm/V
No-Load Speed (approx): {self.voltage*A*60/(self.poles*self.armature_conductors*self.flux_per_pole):.2f} rpm

{'='*60}
Analysis completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*60}
"""

        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(1.0, results)

        # Store results for later use
        self.last_results = {
            'speed': N,
            'torque': T,
            'current': Ia,
            'power_in': P_input,
            'power_out': P_output,
            'efficiency': efficiency,
            'temperature': operating_temp,
            'losses': {
                'copper_armature': copper_loss_armature,
                'copper_field': copper_loss_field,
                'iron': iron_loss,
                'mechanical': mechanical_loss,
                'stray': stray_loss
            }
        }

    def motor_dynamics(self, t, y, motor_type, V, Ra, Rf, La, Lf, poles, Z, A, J, B, TL):
        """
        Dynamic model of DC motor
        State vector: y = [Ia, If, omega, theta, T_motor]
        """
        Ia, If, omega, theta, T_motor = y

        # Flux
        phi = 0.001 * If if If > 0 else self.flux_per_pole  # Simplified flux model

        # Back EMF
        Eb = (poles * Z * phi * omega) / (2 * np.pi * A)

        # Torque
        T = (poles * Z * phi * Ia) / (2 * np.pi * A)

        # Apply PWM if selected
        duty_cycle = self.pwm_duty_var.get() / 100.0
        V_effective = V * duty_cycle if self.control_method_var.get() == "PWM Control" else V

        # Differential equations
        if motor_type == "Shunt":
            dIa_dt = (V_effective - Eb - Ia * Ra) / La
            dIf_dt = (V - If * Rf) / Lf
        elif motor_type == "Series":
            dIa_dt = (V_effective - Eb - Ia * (Ra + Rf)) / (La + Lf)
            dIf_dt = dIa_dt  # Series connection
        else:  # Compound
            dIa_dt = (V_effective - Eb - Ia * Ra) / La
            dIf_dt = (V - If * Rf) / Lf + dIa_dt * 0.5  # Simplified compound model

        domega_dt = (T - B * omega - TL) / J
        dtheta_dt = omega
        dT_dt = (T - T_motor) / 0.01  # First-order torque dynamics

        return [dIa_dt, dIf_dt, domega_dt, dtheta_dt, dT_dt]

    def start_simulation(self):
        """Start dynamic simulation"""
        self.is_running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

        self.update_parameters_from_gui()
        self.solver_type = self.solver_var.get()
        self.dt = self.dt_var.get()
        duration = self.duration_var.get()

        # Initial conditions: [Ia, If, omega, theta, T_motor]
        y0 = [0.1, 0.1, 0, 0, 0]

        # Number of parallel paths
        A = 2 if self.winding_type == "Wave" else self.poles

        # Time span
        t_span = (0, duration)
        t_eval = np.arange(0, duration, self.dt)

        # Solve ODE
        if self.solver_type == "Euler":
            # Euler method
            sol = self.euler_solve(t_eval, y0, A)
        else:
            # Use scipy integrator
            sol = integrate.solve_ivp(
                lambda t, y: self.motor_dynamics(t, y, self.motor_type, self.voltage,
                                                 self.Ra, self.Rf, self.La, self.Lf,
                                                 self.poles, self.armature_conductors, A,
                                                 self.J, self.B, self.load_torque),
                t_span, y0, method=self.solver_type, t_eval=t_eval,
                max_step=self.dt
            )

        # Extract results
        if self.solver_type == "Euler":
            t = sol['t']
            Ia = sol['y'][0]
            If = sol['y'][1]
            omega = sol['y'][2]
            T_motor = sol['y'][4]
        else:
            t = sol.t
            Ia = sol.y[0]
            If = sol.y[1]
            omega = sol.y[2]
            T_motor = sol.y[4]

        # Convert to engineering units
        N = omega * 60 / (2 * np.pi)  # rpm

        # Calculate power and efficiency
        P_in = self.voltage * (Ia + If)
        P_out = T_motor * omega
        efficiency = np.where(P_in > 0, (P_out / P_in) * 100, 0)

        # Thermal simulation
        temperature = self.simulate_thermal(t, P_in, P_out)

        # Update graphs
        self.update_simulation_graphs(t, N, Ia, T_motor, P_in, efficiency, temperature)

        self.is_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

        messagebox.showinfo("Simulation Complete",
                           f"Dynamic simulation completed using {self.solver_type} solver")

    def euler_solve(self, t_eval, y0, A):
        """Custom Euler solver"""
        y = np.zeros((len(y0), len(t_eval)))
        y[:, 0] = y0

        for i in range(1, len(t_eval)):
            dt = t_eval[i] - t_eval[i-1]
            dydt = self.motor_dynamics(t_eval[i-1], y[:, i-1], self.motor_type,
                                      self.voltage, self.Ra, self.Rf, self.La, self.Lf,
                                      self.poles, self.armature_conductors, A,
                                      self.J, self.B, self.load_torque)
            y[:, i] = y[:, i-1] + np.array(dydt) * dt

        return {'t': t_eval, 'y': y}

    def simulate_thermal(self, t, P_in, P_out):
        """Simulate thermal behavior"""
        losses = P_in - P_out
        temperature = np.zeros_like(t)
        temperature[0] = self.ambient_temp

        for i in range(1, len(t)):
            dt = t[i] - t[i-1]
            # Heat transfer equation: C*dT/dt = P_loss - (T-T_amb)/R
            dT_dt = (losses[i] - (temperature[i-1] - self.ambient_temp) / self.thermal_resistance) / self.thermal_capacitance
            temperature[i] = temperature[i-1] + dT_dt * dt

            # Apply derating if enabled
            if self.derating_var.get() and temperature[i] > self.max_temp_var.get():
                temperature[i] = self.max_temp_var.get()

        return temperature

    def update_simulation_graphs(self, t, speed, current, torque, power, efficiency, temperature):
        """Update simulation graphs"""
        # Clear all axes
        for ax in [self.ax_sim_speed, self.ax_sim_current, self.ax_sim_torque,
                  self.ax_sim_power, self.ax_sim_efficiency, self.ax_sim_temp]:
            ax.clear()

        # Plot data
        self.ax_sim_speed.plot(t, speed, 'b-', linewidth=2)
        self.ax_sim_speed.set_ylabel('Speed (rpm)')
        self.ax_sim_speed.set_xlabel('Time (s)')
        self.ax_sim_speed.set_title('Speed Response')
        self.ax_sim_speed.grid(True, alpha=0.3)

        self.ax_sim_current.plot(t, current, 'r-', linewidth=2)
        self.ax_sim_current.set_ylabel('Current (A)')
        self.ax_sim_current.set_xlabel('Time (s)')
        self.ax_sim_current.set_title('Armature Current')
        self.ax_sim_current.grid(True, alpha=0.3)

        self.ax_sim_torque.plot(t, torque, 'g-', linewidth=2)
        self.ax_sim_torque.set_ylabel('Torque (N·m)')
        self.ax_sim_torque.set_xlabel('Time (s)')
        self.ax_sim_torque.set_title('Motor Torque')
        self.ax_sim_torque.grid(True, alpha=0.3)

        self.ax_sim_power.plot(t, power, 'm-', linewidth=2)
        self.ax_sim_power.set_ylabel('Power (W)')
        self.ax_sim_power.set_xlabel('Time (s)')
        self.ax_sim_power.set_title('Input Power')
        self.ax_sim_power.grid(True, alpha=0.3)

        self.ax_sim_efficiency.plot(t, efficiency, 'c-', linewidth=2)
        self.ax_sim_efficiency.set_ylabel('Efficiency (%)')
        self.ax_sim_efficiency.set_xlabel('Time (s)')
        self.ax_sim_efficiency.set_title('Efficiency')
        self.ax_sim_efficiency.grid(True, alpha=0.3)

        self.ax_sim_temp.plot(t, temperature, 'orange', linewidth=2)
        self.ax_sim_temp.axhline(y=self.max_temp_var.get(), color='r', linestyle='--', label='Max Temp')
        self.ax_sim_temp.set_ylabel('Temperature (°C)')
        self.ax_sim_temp.set_xlabel('Time (s)')
        self.ax_sim_temp.set_title('Temperature Rise')
        self.ax_sim_temp.legend()
        self.ax_sim_temp.grid(True, alpha=0.3)

        self.fig_sim.tight_layout()
        self.canvas_sim.draw()

    def stop_simulation(self):
        """Stop simulation"""
        self.is_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def reset_simulation(self):
        """Reset simulation"""
        self.is_running = False
        self.simulation_time = 0
        self.time_data = []
        self.speed_data = []
        self.torque_data = []
        self.current_data = []
        self.temp_data = []
        self.efficiency_data = []

        # Clear graphs
        for ax in [self.ax_speed, self.ax_torque]:
            ax.clear()
        self.setup_main_graphs()
        self.canvas_main.draw()

        self.results_text.delete(1.0, tk.END)
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def calculate_economics(self):
        """Calculate economic analysis"""
        if not hasattr(self, 'last_results'):
            messagebox.showwarning("Warning", "Please calculate steady-state performance first")
            return

        # Get economic parameters
        elec_cost = self.elec_cost_var.get()
        op_hours = self.op_hours_var.get()
        maint_cost = self.maint_cost_var.get()
        init_cost = self.init_cost_var.get()

        # Get performance data
        P_input_kW = self.last_results['power_in'] / 1000
        P_output_kW = self.last_results['power_out'] / 1000
        efficiency = self.last_results['efficiency']

        # Annual energy consumption
        annual_energy = P_input_kW * op_hours  # kWh

        # Annual operating cost
        annual_elec_cost = annual_energy * elec_cost
        annual_total_cost = annual_elec_cost + maint_cost

        # Lifetime analysis (20 years)
        years = 20
        lifetime_cost = init_cost + annual_total_cost * years

        # Energy savings with improved efficiency (compare with 80% efficient motor)
        baseline_efficiency = 80.0
        if efficiency > baseline_efficiency:
            energy_saved = P_output_kW * op_hours * (1/baseline_efficiency*100 - 1/efficiency*100)
            cost_saved = energy_saved * elec_cost
        else:
            energy_saved = 0
            cost_saved = 0

        # CO2 emissions (assuming 0.5 kg CO2/kWh)
        co2_emissions = annual_energy * 0.5  # kg

        # Payback period (if more efficient than baseline)
        if cost_saved > 0:
            payback = init_cost / cost_saved
        else:
            payback = float('inf')

        # Display results
        results = f"""
{'='*70}
ECONOMIC ANALYSIS
{'='*70}

OPERATING PARAMETERS:
{'─'*70}
Operating Hours per Year: {op_hours:.0f} hours
Electricity Cost: ${elec_cost:.4f}/kWh
Annual Maintenance Cost: ${maint_cost:.2f}
Initial Investment: ${init_cost:.2f}

POWER CONSUMPTION:
{'─'*70}
Input Power: {P_input_kW:.3f} kW
Output Power: {P_output_kW:.3f} kW
Efficiency: {efficiency:.2f}%
Annual Energy Consumption: {annual_energy:.2f} kWh

ANNUAL COSTS:
{'─'*70}
Electricity Cost: ${annual_elec_cost:.2f}/year
Maintenance Cost: ${maint_cost:.2f}/year
Total Annual Operating Cost: ${annual_total_cost:.2f}/year

LIFETIME ANALYSIS ({years} years):
{'─'*70}
Total Lifetime Cost: ${lifetime_cost:.2f}
Average Cost per Year: ${lifetime_cost/years:.2f}

EFFICIENCY COMPARISON:
{'─'*70}
Baseline Efficiency: {baseline_efficiency:.1f}%
This Motor Efficiency: {efficiency:.2f}%
Annual Energy Saved: {energy_saved:.2f} kWh
Annual Cost Saved: ${cost_saved:.2f}
{'Payback Period: ' + f'{payback:.2f} years' if payback != float('inf') else 'No payback (less efficient than baseline)'}

ENVIRONMENTAL IMPACT:
{'─'*70}
Annual CO₂ Emissions: {co2_emissions:.2f} kg
Lifetime CO₂ Emissions: {co2_emissions * years:.2f} kg

COST BREAKDOWN (20 years):
{'─'*70}
Initial Investment: ${init_cost:.2f} ({init_cost/lifetime_cost*100:.1f}%)
Electricity Costs: ${annual_elec_cost*years:.2f} ({annual_elec_cost*years/lifetime_cost*100:.1f}%)
Maintenance Costs: ${maint_cost*years:.2f} ({maint_cost*years/lifetime_cost*100:.1f}%)

{'='*70}
Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*70}
"""

        self.economic_text.delete(1.0, tk.END)
        self.economic_text.insert(1.0, results)

        # Plot cost breakdown
        self.plot_economic_analysis(years, annual_total_cost, init_cost, annual_elec_cost,
                                    maint_cost, payback)

    def plot_economic_analysis(self, years, annual_cost, init_cost, elec_cost, maint_cost, payback):
        """Plot economic analysis graphs"""
        # Clear axes
        self.ax_cost.clear()
        self.ax_payback.clear()

        # Cumulative cost over time
        year_range = np.arange(0, years+1)
        cumulative_cost = init_cost + annual_cost * year_range

        self.ax_cost.plot(year_range, cumulative_cost, 'b-', linewidth=2, marker='o')
        self.ax_cost.set_xlabel('Years')
        self.ax_cost.set_ylabel('Cumulative Cost ($)')
        self.ax_cost.set_title('Total Cost of Ownership')
        self.ax_cost.grid(True, alpha=0.3)

        # Cost breakdown pie chart
        costs = [init_cost, elec_cost*years, maint_cost*years]
        labels = ['Initial Investment', 'Electricity', 'Maintenance']
        colors = ['#ff9999', '#66b3ff', '#99ff99']

        self.ax_payback.pie(costs, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        self.ax_payback.set_title('Cost Breakdown (20 years)')

        self.fig_econ.tight_layout()
        self.canvas_main.draw()

    def analyze_power(self):
        """Analyze power consumption"""
        if not hasattr(self, 'last_results'):
            messagebox.showwarning("Warning", "Please calculate steady-state performance first")
            return

        losses = self.last_results['losses']
        P_in = self.last_results['power_in']
        P_out = self.last_results['power_out']

        total_loss = sum(losses.values())

        results = f"""
{'='*70}
DETAILED POWER CONSUMPTION ANALYSIS
{'='*70}

POWER FLOW:
{'─'*70}
Input Power: {P_in:.2f} W ({P_in/1000:.3f} kW)
Output Power: {P_out:.2f} W ({P_out/1000:.3f} kW)
Total Losses: {total_loss:.2f} W ({total_loss/1000:.3f} kW)

LOSS BREAKDOWN:
{'─'*70}
Copper Losses (Armature): {losses['copper_armature']:.2f} W ({losses['copper_armature']/total_loss*100:.1f}%)
Copper Losses (Field): {losses['copper_field']:.2f} W ({losses['copper_field']/total_loss*100:.1f}%)
Iron Losses: {losses['iron']:.2f} W ({losses['iron']/total_loss*100:.1f}%)
Mechanical Losses: {losses['mechanical']:.2f} W ({losses['mechanical']/total_loss*100:.1f}%)
Stray Load Losses: {losses['stray']:.2f} W ({losses['stray']/total_loss*100:.1f}%)

EFFICIENCY ANALYSIS:
{'─'*70}
Overall Efficiency: {self.last_results['efficiency']:.2f}%
Electrical Efficiency: {(P_in-losses['copper_armature']-losses['copper_field'])/P_in*100:.2f}%
Mechanical Efficiency: {(P_out+losses['mechanical'])/P_out*100 if P_out > 0 else 0:.2f}%

POWER QUALITY:
{'─'*70}
Power Factor: {P_out/P_in if P_in > 0 else 0:.3f}
Loss Factor: {total_loss/P_in*100:.2f}%

THERMAL POWER:
{'─'*70}
Heat Dissipation Required: {total_loss:.2f} W
Temperature Rise: {total_loss*self.thermal_resistance:.2f} K
Cooling Required: {'Yes' if total_loss*self.thermal_resistance > 50 else 'Natural convection sufficient'}

{'='*70}
"""

        self.power_text.delete(1.0, tk.END)
        self.power_text.insert(1.0, results)

    def run_multiphysics_analysis(self):
        """Run comprehensive multi-physics analysis"""
        if not hasattr(self, 'last_results'):
            messagebox.showwarning("Warning", "Please calculate steady-state performance first")
            return

        # Run selected analyses
        if self.em_thermal_var.get():
            self.analyze_electromagnetic_thermal()

        if self.mechanical_var.get():
            self.analyze_mechanical_stress()

        if self.acoustic_var.get():
            self.analyze_acoustic_noise()

        if self.efficiency_map_var.get():
            self.create_efficiency_map()

        if self.loss_analysis_var.get():
            self.detailed_loss_analysis()

        messagebox.showinfo("Analysis Complete", "Multi-physics analysis completed successfully")

    def analyze_electromagnetic_thermal(self):
        """Electromagnetic-thermal coupled analysis"""
        # Temperature distribution simulation
        r = np.linspace(0, 0.2, 50)  # Radial position (m)
        theta_angle = np.linspace(0, 2*np.pi, 100)  # Angular position

        # Create mesh
        R, THETA = np.meshgrid(r, theta_angle)

        # Temperature distribution (simplified model)
        # Higher temperature at center, decreasing radially
        losses = self.last_results['losses']
        total_loss = sum(losses.values())
        T_center = self.ambient_temp + total_loss * self.thermal_resistance

        T = T_center * (1 - (R/0.2)**2) + self.ambient_temp

        # Convert to Cartesian
        X = R * np.cos(THETA)
        Y = R * np.sin(THETA)

        # Plot temperature distribution
        self.ax_temp_dist.clear()
        contour = self.ax_temp_dist.contourf(X, Y, T, levels=20, cmap='hot')
        self.ax_temp_dist.set_xlabel('X (m)')
        self.ax_temp_dist.set_ylabel('Y (m)')
        self.ax_temp_dist.set_title('Temperature Distribution')
        self.ax_temp_dist.axis('equal')
        plt.colorbar(contour, ax=self.ax_temp_dist, label='Temperature (°C)')

        # Heat flow analysis
        self.ax_heat_flow.clear()
        time = np.linspace(0, 3600, 100)  # 1 hour
        heat_gen = np.ones_like(time) * total_loss
        heat_dissipated = (T_center - self.ambient_temp) / self.thermal_resistance * (1 - np.exp(-time/self.thermal_capacitance))
        temp_transient = self.ambient_temp + (T_center - self.ambient_temp) * (1 - np.exp(-time/(self.thermal_capacitance*self.thermal_resistance)))

        self.ax_heat_flow.plot(time/60, heat_gen, 'r-', label='Heat Generated', linewidth=2)
        self.ax_heat_flow.plot(time/60, heat_dissipated, 'b-', label='Heat Dissipated', linewidth=2)
        self.ax_heat_flow.set_xlabel('Time (min)')
        self.ax_heat_flow.set_ylabel('Power (W)')
        self.ax_heat_flow.set_title('Thermal Transient Response')
        self.ax_heat_flow.legend()
        self.ax_heat_flow.grid(True, alpha=0.3)

        # Add secondary axis for temperature
        ax2 = self.ax_heat_flow.twinx()
        ax2.plot(time/60, temp_transient, 'g--', label='Temperature', linewidth=2)
        ax2.set_ylabel('Temperature (°C)', color='g')
        ax2.tick_params(axis='y', labelcolor='g')

        self.fig_thermal.tight_layout()
        self.canvas_sim.draw()

    def analyze_mechanical_stress(self):
        """Mechanical stress and bearing load analysis"""
        # Shaft stress analysis
        torque = self.last_results['torque']
        shaft_diameter = 0.05  # m (assumed)
        shaft_length = 0.3  # m (assumed)

        # Torsional stress: τ = (16*T)/(π*d³)
        tau = (16 * torque) / (np.pi * shaft_diameter**3)

        # Bending stress (assuming bearing reaction forces)
        bearing_load = torque / (shaft_length/2)
        bending_moment = bearing_load * shaft_length / 4
        sigma_bending = (32 * bending_moment) / (np.pi * shaft_diameter**3)

        # Combined stress
        sigma_combined = np.sqrt(sigma_bending**2 + 3*tau**2)

        # Plot stress distribution along shaft
        self.ax_stress.clear()
        x = np.linspace(0, shaft_length, 100)
        torsional_stress = np.ones_like(x) * tau / 1e6  # MPa
        bending_stress = (bearing_load * x * (shaft_length - x) * 32) / (np.pi * shaft_diameter**3 * shaft_length) / 1e6  # MPa

        self.ax_stress.plot(x*1000, torsional_stress, 'r-', label='Torsional Stress', linewidth=2)
        self.ax_stress.plot(x*1000, bending_stress, 'b-', label='Bending Stress', linewidth=2)
        self.ax_stress.fill_between(x*1000, 0, torsional_stress, alpha=0.3, color='r')
        self.ax_stress.fill_between(x*1000, 0, bending_stress, alpha=0.3, color='b')
        self.ax_stress.set_xlabel('Position along shaft (mm)')
        self.ax_stress.set_ylabel('Stress (MPa)')
        self.ax_stress.set_title('Shaft Stress Distribution')
        self.ax_stress.legend()
        self.ax_stress.grid(True, alpha=0.3)

        # Bearing load analysis
        self.ax_bearing.clear()
        time = np.linspace(0, 1, 1000)  # One revolution
        angle = 2 * np.pi * time

        # Radial load variation (simplified)
        radial_load = bearing_load * (1 + 0.1 * np.sin(self.poles * angle))
        axial_load = bearing_load * 0.2 * np.ones_like(time)  # Assumed constant

        self.ax_bearing.plot(angle, radial_load, 'r-', label='Radial Load', linewidth=2)
        self.ax_bearing.plot(angle, axial_load, 'b--', label='Axial Load', linewidth=2)
        self.ax_bearing.set_xlabel('Shaft Angle (rad)')
        self.ax_bearing.set_ylabel('Bearing Load (N)')
        self.ax_bearing.set_title('Bearing Load Variation')
        self.ax_bearing.legend()
        self.ax_bearing.grid(True, alpha=0.3)

        self.fig_mechanical.tight_layout()
        self.canvas_sim.draw()

    def analyze_acoustic_noise(self):
        """Acoustic noise prediction"""
        # Calculate electromagnetic force frequencies
        speed_rps = self.last_results['speed'] / 60
        f_mech = speed_rps  # Mechanical frequency
        f_elec = self.poles * speed_rps / 2  # Electrical frequency

        # Harmonic frequencies
        harmonics = np.arange(1, 11)
        freq_mechanical = harmonics * f_mech
        freq_electromagnetic = harmonics * f_elec

        # Sound pressure levels (empirical model)
        # SPL ∝ (electromagnetic force)² ∝ (current × flux)²
        Ia = self.last_results['current']
        base_spl = 20 * np.log10(Ia * self.flux_per_pole * 1000) + 40  # dB

        spl_mech = base_spl - 3 * harmonics  # Mechanical noise decreases with harmonics
        spl_em = base_spl - 2 * harmonics  # Electromagnetic noise

        # Plot noise spectrum
        self.ax_noise.clear()
        self.ax_noise.stem(freq_mechanical, spl_mech, linefmt='b-', markerfmt='bo',
                          basefmt='k-', label='Mechanical')
        self.ax_noise.stem(freq_electromagnetic, spl_em, linefmt='r-', markerfmt='ro',
                          basefmt='k-', label='Electromagnetic')
        self.ax_noise.set_xlabel('Frequency (Hz)')
        self.ax_noise.set_ylabel('Sound Pressure Level (dB)')
        self.ax_noise.set_title('Acoustic Noise Spectrum')
        self.ax_noise.legend()
        self.ax_noise.grid(True, alpha=0.3)
        self.ax_noise.set_xlim([0, max(freq_electromagnetic) * 1.1])

        # Vibration analysis
        self.ax_vibration.clear()
        time = np.linspace(0, 0.1, 1000)

        # Combine vibrations from different sources
        vibration = np.zeros_like(time)
        for i, (f_m, f_e) in enumerate(zip(freq_mechanical[:5], freq_electromagnetic[:5])):
            vibration += (spl_mech[i]/100) * np.sin(2*np.pi*f_m*time) + \
                        (spl_em[i]/100) * np.sin(2*np.pi*f_e*time)

        self.ax_vibration.plot(time*1000, vibration, 'g-', linewidth=1.5)
        self.ax_vibration.set_xlabel('Time (ms)')
        self.ax_vibration.set_ylabel('Vibration Amplitude (mm/s)')
        self.ax_vibration.set_title('Vibration Time History')
        self.ax_vibration.grid(True, alpha=0.3)

        self.fig_acoustic.tight_layout()
        self.canvas_sim.draw()

    def create_efficiency_map(self):
        """Create efficiency map across torque-speed envelope"""
        # Create torque and speed ranges
        torque_range = np.linspace(10, 300, 30)
        speed_range = np.linspace(100, 1500, 30)

        T_grid, N_grid = np.meshgrid(torque_range, speed_range)
        efficiency_grid = np.zeros_like(T_grid)

        # Calculate efficiency for each operating point
        for i in range(len(speed_range)):
            for j in range(len(torque_range)):
                T = T_grid[i, j]
                N = N_grid[i, j]
                omega = 2 * np.pi * N / 60

                # Estimate current for this torque
                A = 2 if self.winding_type == "Wave" else self.poles
                Ia = (T * 2 * np.pi * A) / (self.poles * self.armature_conductors * self.flux_per_pole)

                # Estimate back EMF for this speed
                Eb = (self.poles * self.armature_conductors * self.flux_per_pole * omega) / (2 * np.pi * A)

                # Estimate voltage required
                V_req = Eb + Ia * self.Ra

                # Power
                P_out = T * omega
                If = V_req / self.Rf if self.motor_type != "Series" else Ia
                P_in = V_req * (Ia + If)

                # Efficiency
                efficiency_grid[i, j] = (P_out / P_in * 100) if P_in > 0 else 0

        # Plot efficiency map
        self.ax_eff_map.clear()
        contour = self.ax_eff_map.contourf(T_grid, N_grid, efficiency_grid, levels=20, cmap='RdYlGn')
        self.ax_eff_map.contour(T_grid, N_grid, efficiency_grid, levels=[80, 85, 90, 95],
                                colors='black', linewidths=1.5)
        self.ax_eff_map.set_xlabel('Torque (N·m)')
        self.ax_eff_map.set_ylabel('Speed (rpm)')
        self.ax_eff_map.set_title('Efficiency Map (%)')

        # Add colorbar
        cbar = plt.colorbar(contour, ax=self.ax_eff_map)
        cbar.set_label('Efficiency (%)')

        # Mark optimal operating point
        max_eff_idx = np.unravel_index(np.argmax(efficiency_grid), efficiency_grid.shape)
        self.ax_eff_map.plot(T_grid[max_eff_idx], N_grid[max_eff_idx], 'w*',
                            markersize=20, label=f'Optimal: {efficiency_grid[max_eff_idx]:.1f}%')
        self.ax_eff_map.legend()

        self.fig_efficiency.tight_layout()
        self.canvas_sim.draw()

    def detailed_loss_analysis(self):
        """Detailed loss breakdown analysis"""
        if not hasattr(self, 'last_results'):
            return

        losses = self.last_results['losses']

        # Pie chart
        self.ax_loss_pie.clear()
        loss_values = list(losses.values())
        loss_labels = ['Copper (Armature)', 'Copper (Field)', 'Iron', 'Mechanical', 'Stray']
        colors = ['#ff6b6b', '#ee5a6f', '#4ecdc4', '#45b7d1', '#96ceb4']

        self.ax_loss_pie.pie(loss_values, labels=loss_labels, colors=colors,
                            autopct='%1.1f%%', startangle=90)
        self.ax_loss_pie.set_title('Loss Distribution')

        # Bar chart
        self.ax_loss_bar.clear()
        x_pos = np.arange(len(loss_labels))
        bars = self.ax_loss_bar.bar(x_pos, loss_values, color=colors, edgecolor='black', linewidth=1.5)

        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            self.ax_loss_bar.text(bar.get_x() + bar.get_width()/2., height,
                                 f'{height:.1f} W',
                                 ha='center', va='bottom', fontweight='bold')

        self.ax_loss_bar.set_xlabel('Loss Type')
        self.ax_loss_bar.set_ylabel('Power Loss (W)')
        self.ax_loss_bar.set_title('Detailed Loss Breakdown')
        self.ax_loss_bar.set_xticks(x_pos)
        self.ax_loss_bar.set_xticklabels(loss_labels, rotation=45, ha='right')
        self.ax_loss_bar.grid(True, alpha=0.3, axis='y')

        self.fig_loss.tight_layout()
        self.canvas_sim.draw()


def main():
    """Main entry point"""
    root = tk.Tk()
    app = DCMotorSimulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
