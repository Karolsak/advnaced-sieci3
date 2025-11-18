"""
Advanced DC Shunt Motor Braking Multi-Physics Simulator
Features:
- Multi-tab Tkinter GUI with auto-scaling
- Real-time ODE solvers (RK45, Euler)
- Multi-physics modeling (Electromagnetic, Thermal, Mechanical)
- Dynamic visualization with live graphs
- Economic analysis and loss breakdown
- Advanced controls with thermal derating
- Professional UI with sliders and controls
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import threading
import time
from datetime import datetime


class DCMotorBrakingSimulator:
    """Advanced DC Motor Braking Simulator with Multi-Physics Analysis"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Braking Multi-Physics Simulator")
        self.root.geometry("1400x900")

        # Configure grid weight for auto-scaling
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Simulation control
        self.simulation_running = False
        self.simulation_paused = False
        self.simulation_thread = None

        # Motor parameters (default values from the problem)
        self.params = {
            'V': tk.DoubleVar(value=250.0),  # Supply voltage (V)
            'Ia_initial': tk.DoubleVar(value=150.0),  # Initial armature current (A)
            'N_initial': tk.DoubleVar(value=550.0),  # Initial speed (rpm)
            'Ra': tk.DoubleVar(value=0.09),  # Armature resistance (Ω)
            'Ia_brake_limit': tk.DoubleVar(value=240.0),  # Braking current limit (A)
            'J': tk.DoubleVar(value=2.5),  # Moment of inertia (kg·m²)
            'B': tk.DoubleVar(value=0.05),  # Friction coefficient (N·m·s)
            'Rf': tk.DoubleVar(value=125.0),  # Field resistance (Ω)
            'La': tk.DoubleVar(value=0.02),  # Armature inductance (H)
            'thermal_resistance': tk.DoubleVar(value=0.8),  # Thermal resistance (°C/W)
            'thermal_capacitance': tk.DoubleVar(value=1200.0),  # Thermal capacitance (J/°C)
            'ambient_temp': tk.DoubleVar(value=25.0),  # Ambient temperature (°C)
            'max_temp': tk.DoubleVar(value=155.0),  # Maximum allowed temperature (°C)
            'iron_loss_coeff': tk.DoubleVar(value=50.0),  # Iron loss coefficient (W)
            'stray_loss_coeff': tk.DoubleVar(value=1.0),  # Stray loss coefficient (%)
            'efficiency': tk.DoubleVar(value=0.90),  # Motor efficiency
            'cost_per_kwh': tk.DoubleVar(value=0.12),  # Electricity cost ($/kWh)
        }

        # Solver selection
        self.solver_type = tk.StringVar(value='RK45')

        # Results storage
        self.results = {
            'time': [],
            'speed': [],
            'current': [],
            'torque': [],
            'voltage': [],
            'power': [],
            'temperature': [],
            'copper_loss': [],
            'iron_loss': [],
            'friction_loss': [],
            'stray_loss': [],
            'total_loss': [],
            'efficiency': [],
            'shaft_stress': [],
            'bearing_load': [],
        }

        # Calculated values
        self.calculated = {
            'R_series': 0.0,
            'T_initial': 0.0,
            'T_at_200rpm': 0.0,
            'Eb_initial': 0.0,
            'k_phi': 0.0,
        }

        # Create UI
        self.create_ui()

        # Calculate initial values
        self.calculate_static_values()

    def create_ui(self):
        """Create the main user interface with tabs"""
        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.tab_main = ttk.Frame(self.notebook)
        self.tab_graphs = ttk.Frame(self.notebook)
        self.tab_thermal = ttk.Frame(self.notebook)
        self.tab_losses = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)
        self.tab_results = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_main, text='Main Control')
        self.notebook.add(self.tab_graphs, text='Dynamic Graphs')
        self.notebook.add(self.tab_thermal, text='Thermal Analysis')
        self.notebook.add(self.tab_losses, text='Loss Breakdown')
        self.notebook.add(self.tab_economic, text='Economic Analysis')
        self.notebook.add(self.tab_results, text='Results & Data')

        # Configure tab weights for auto-scaling
        for tab in [self.tab_main, self.tab_graphs, self.tab_thermal,
                    self.tab_losses, self.tab_economic, self.tab_results]:
            tab.grid_rowconfigure(0, weight=1)
            tab.grid_columnconfigure(0, weight=1)

        # Build each tab
        self.build_main_tab()
        self.build_graphs_tab()
        self.build_thermal_tab()
        self.build_losses_tab()
        self.build_economic_tab()
        self.build_results_tab()

    def build_main_tab(self):
        """Build the main control tab"""
        # Configure grid
        self.tab_main.grid_columnconfigure(0, weight=1)
        self.tab_main.grid_columnconfigure(1, weight=2)
        self.tab_main.grid_rowconfigure(2, weight=1)

        # Left panel - Parameters
        left_frame = ttk.LabelFrame(self.tab_main, text='Motor Parameters', padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5, rowspan=3)

        # Scrollable frame for parameters
        canvas = tk.Canvas(left_frame)
        scrollbar = ttk.Scrollbar(left_frame, orient='vertical', command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            '<Configure>',
            lambda e: canvas.configure(scrollregion=canvas.bbox('all'))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        # Parameter entries
        param_labels = {
            'V': 'Supply Voltage (V)',
            'Ia_initial': 'Initial Armature Current (A)',
            'N_initial': 'Initial Speed (rpm)',
            'Ra': 'Armature Resistance (Ω)',
            'Ia_brake_limit': 'Braking Current Limit (A)',
            'J': 'Moment of Inertia (kg·m²)',
            'B': 'Friction Coefficient (N·m·s)',
            'Rf': 'Field Resistance (Ω)',
            'La': 'Armature Inductance (H)',
            'thermal_resistance': 'Thermal Resistance (°C/W)',
            'thermal_capacitance': 'Thermal Capacitance (J/°C)',
            'ambient_temp': 'Ambient Temperature (°C)',
            'max_temp': 'Max Temperature (°C)',
            'iron_loss_coeff': 'Iron Loss Coefficient (W)',
            'stray_loss_coeff': 'Stray Loss Coefficient (%)',
            'efficiency': 'Motor Efficiency',
            'cost_per_kwh': 'Electricity Cost ($/kWh)',
        }

        row = 0
        self.sliders = {}
        for key, label in param_labels.items():
            ttk.Label(scrollable_frame, text=label).grid(row=row, column=0, sticky='w', pady=2)

            frame = ttk.Frame(scrollable_frame)
            frame.grid(row=row, column=1, sticky='ew', pady=2, padx=5)

            entry = ttk.Entry(frame, textvariable=self.params[key], width=10)
            entry.pack(side='left', padx=2)

            # Create slider for important parameters
            if key in ['V', 'Ia_brake_limit', 'N_initial', 'J', 'B']:
                current_val = self.params[key].get()
                min_val = max(0, current_val * 0.5)
                max_val = current_val * 2.0

                slider = ttk.Scale(frame, from_=min_val, to=max_val,
                                 variable=self.params[key], orient='horizontal')
                slider.pack(side='left', fill='x', expand=True, padx=2)
                self.sliders[key] = slider

            row += 1

        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        # Top right - Calculated values
        calc_frame = ttk.LabelFrame(self.tab_main, text='Calculated Values', padding=10)
        calc_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.calc_labels = {}
        calc_items = [
            ('R_series', 'Series Resistance Required (Ω)'),
            ('Eb_initial', 'Initial Back EMF (V)'),
            ('k_phi', 'Motor Constant k·Φ (V·s/rad)'),
            ('T_initial', 'Initial Braking Torque (N·m)'),
            ('T_at_200rpm', 'Braking Torque at 200 rpm (N·m)'),
        ]

        for i, (key, label) in enumerate(calc_items):
            ttk.Label(calc_frame, text=label).grid(row=i, column=0, sticky='w', pady=5)
            lbl = ttk.Label(calc_frame, text='0.000', font=('Arial', 10, 'bold'))
            lbl.grid(row=i, column=1, sticky='e', pady=5, padx=10)
            self.calc_labels[key] = lbl

        # Middle right - Controls
        control_frame = ttk.LabelFrame(self.tab_main, text='Simulation Controls', padding=10)
        control_frame.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)

        # Solver selection
        ttk.Label(control_frame, text='ODE Solver:').grid(row=0, column=0, sticky='w', pady=5)
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_type,
                                    values=['RK45', 'Euler'], state='readonly', width=15)
        solver_combo.grid(row=0, column=1, sticky='w', pady=5, padx=5)

        # Control buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=1, column=0, columnspan=2, pady=10)

        self.btn_start = ttk.Button(btn_frame, text='Start Simulation',
                                    command=self.start_simulation, width=15)
        self.btn_start.grid(row=0, column=0, padx=5, pady=5)

        self.btn_stop = ttk.Button(btn_frame, text='Stop', command=self.stop_simulation,
                                   width=15, state='disabled')
        self.btn_stop.grid(row=0, column=1, padx=5, pady=5)

        self.btn_reset = ttk.Button(btn_frame, text='Reset', command=self.reset_simulation,
                                    width=15)
        self.btn_reset.grid(row=0, column=2, padx=5, pady=5)

        ttk.Button(btn_frame, text='Calculate', command=self.calculate_static_values,
                  width=15).grid(row=1, column=0, padx=5, pady=5)

        # Status display
        status_frame = ttk.LabelFrame(control_frame, text='Simulation Status', padding=5)
        status_frame.grid(row=2, column=0, columnspan=2, sticky='ew', pady=5)

        self.status_label = ttk.Label(status_frame, text='Ready',
                                      font=('Arial', 10), foreground='green')
        self.status_label.pack()

        # Progress bar
        self.progress = ttk.Progressbar(control_frame, mode='indeterminate')
        self.progress.grid(row=3, column=0, columnspan=2, sticky='ew', pady=5)

        # Bottom right - Quick visualization
        viz_frame = ttk.LabelFrame(self.tab_main, text='Quick Preview', padding=10)
        viz_frame.grid(row=2, column=1, sticky='nsew', padx=5, pady=5)
        viz_frame.grid_rowconfigure(0, weight=1)
        viz_frame.grid_columnconfigure(0, weight=1)

        # Create quick preview plot
        self.fig_preview = Figure(figsize=(6, 3), dpi=80)
        self.ax_preview = self.fig_preview.add_subplot(111)
        self.canvas_preview = FigureCanvasTkAgg(self.fig_preview, viz_frame)
        self.canvas_preview.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def build_graphs_tab(self):
        """Build dynamic graphs tab"""
        self.tab_graphs.grid_rowconfigure(0, weight=1)
        self.tab_graphs.grid_rowconfigure(1, weight=1)
        self.tab_graphs.grid_columnconfigure(0, weight=1)
        self.tab_graphs.grid_columnconfigure(1, weight=1)

        # Create multiple subplots
        self.fig_main = Figure(figsize=(12, 8), dpi=100)

        # 2x2 grid of plots
        self.ax_speed = self.fig_main.add_subplot(221)
        self.ax_current = self.fig_main.add_subplot(222)
        self.ax_torque = self.fig_main.add_subplot(223)
        self.ax_power = self.fig_main.add_subplot(224)

        self.fig_main.tight_layout(pad=3.0)

        self.canvas_main = FigureCanvasTkAgg(self.fig_main, self.tab_graphs)
        self.canvas_main.get_tk_widget().grid(row=0, column=0, columnspan=2, sticky='nsew')

    def build_thermal_tab(self):
        """Build thermal analysis tab"""
        self.tab_thermal.grid_rowconfigure(0, weight=2)
        self.tab_thermal.grid_rowconfigure(1, weight=1)
        self.tab_thermal.grid_columnconfigure(0, weight=1)

        # Thermal graph
        graph_frame = ttk.LabelFrame(self.tab_thermal, text='Temperature Profile', padding=10)
        graph_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        graph_frame.grid_rowconfigure(0, weight=1)
        graph_frame.grid_columnconfigure(0, weight=1)

        self.fig_thermal = Figure(figsize=(10, 4), dpi=100)
        self.ax_temp = self.fig_thermal.add_subplot(121)
        self.ax_derating = self.fig_thermal.add_subplot(122)
        self.fig_thermal.tight_layout()

        self.canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, graph_frame)
        self.canvas_thermal.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Thermal info
        info_frame = ttk.LabelFrame(self.tab_thermal, text='Thermal Information', padding=10)
        info_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        self.thermal_text = scrolledtext.ScrolledText(info_frame, height=8, wrap=tk.WORD)
        self.thermal_text.pack(fill='both', expand=True)

    def build_losses_tab(self):
        """Build loss breakdown tab"""
        self.tab_losses.grid_rowconfigure(0, weight=1)
        self.tab_losses.grid_columnconfigure(0, weight=1)

        # Loss breakdown graph
        self.fig_losses = Figure(figsize=(10, 6), dpi=100)
        self.ax_loss_pie = self.fig_losses.add_subplot(121)
        self.ax_loss_time = self.fig_losses.add_subplot(122)
        self.fig_losses.tight_layout()

        self.canvas_losses = FigureCanvasTkAgg(self.fig_losses, self.tab_losses)
        self.canvas_losses.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def build_economic_tab(self):
        """Build economic analysis tab"""
        self.tab_economic.grid_rowconfigure(1, weight=1)
        self.tab_economic.grid_columnconfigure(0, weight=1)

        # Economic parameters
        param_frame = ttk.LabelFrame(self.tab_economic, text='Economic Parameters', padding=10)
        param_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        ttk.Label(param_frame, text='Electricity Cost ($/kWh):').grid(row=0, column=0, sticky='w')
        ttk.Entry(param_frame, textvariable=self.params['cost_per_kwh'], width=15).grid(
            row=0, column=1, padx=5)

        # Economic results
        results_frame = ttk.LabelFrame(self.tab_economic, text='Economic Analysis', padding=10)
        results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        results_frame.grid_rowconfigure(0, weight=1)
        results_frame.grid_columnconfigure(0, weight=1)

        self.economic_text = scrolledtext.ScrolledText(results_frame, height=15, wrap=tk.WORD,
                                                       font=('Courier', 10))
        self.economic_text.grid(row=0, column=0, sticky='nsew')

    def build_results_tab(self):
        """Build results and data tab"""
        self.tab_results.grid_rowconfigure(0, weight=1)
        self.tab_results.grid_columnconfigure(0, weight=1)

        # Results display
        self.results_text = scrolledtext.ScrolledText(self.tab_results, wrap=tk.WORD,
                                                      font=('Courier', 9))
        self.results_text.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Export button
        btn_frame = ttk.Frame(self.tab_results)
        btn_frame.grid(row=1, column=0, sticky='ew', padx=5, pady=5)

        ttk.Button(btn_frame, text='Export to CSV',
                  command=self.export_results).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='Copy to Clipboard',
                  command=self.copy_results).pack(side='left', padx=5)

    def calculate_static_values(self):
        """Calculate static braking values from the problem"""
        try:
            V = self.params['V'].get()
            Ia_initial = self.params['Ia_initial'].get()
            N_initial = self.params['N_initial'].get()
            Ra = self.params['Ra'].get()
            Ia_brake = self.params['Ia_brake_limit'].get()

            # Calculate back EMF at initial speed
            Eb_initial = V - Ia_initial * Ra
            self.calculated['Eb_initial'] = Eb_initial

            # Calculate motor constant k·Φ
            omega_initial = 2 * np.pi * N_initial / 60  # rad/s
            k_phi = Eb_initial / omega_initial
            self.calculated['k_phi'] = k_phi

            # (a) Calculate series resistance required
            # During braking: V + Eb = Ia_brake * (Ra + R_series)
            R_series = (V + Eb_initial) / Ia_brake - Ra
            self.calculated['R_series'] = R_series

            # (b) Calculate initial braking torque
            T_initial = k_phi * Ia_brake
            self.calculated['T_initial'] = T_initial

            # (c) Calculate braking torque at 200 rpm
            omega_200 = 2 * np.pi * 200 / 60
            Eb_200 = k_phi * omega_200
            Ia_200 = (V + Eb_200) / (Ra + R_series)
            T_200 = k_phi * Ia_200
            self.calculated['T_at_200rpm'] = T_200

            # Update labels
            self.calc_labels['R_series'].config(text=f'{R_series:.4f}')
            self.calc_labels['Eb_initial'].config(text=f'{Eb_initial:.4f}')
            self.calc_labels['k_phi'].config(text=f'{k_phi:.4f}')
            self.calc_labels['T_initial'].config(text=f'{T_initial:.4f}')
            self.calc_labels['T_at_200rpm'].config(text=f'{T_200:.4f}')

            # Update preview plot
            self.update_preview_plot()

            self.status_label.config(text='Calculations completed', foreground='green')

        except Exception as e:
            messagebox.showerror('Calculation Error', f'Error in calculations: {str(e)}')

    def update_preview_plot(self):
        """Update quick preview plot with static calculations"""
        try:
            self.ax_preview.clear()

            # Generate braking curve
            speeds = np.linspace(self.params['N_initial'].get(), 0, 100)
            k_phi = self.calculated['k_phi']
            V = self.params['V'].get()
            Ra = self.params['Ra'].get()
            R_series = self.calculated['R_series']
            R_total = Ra + R_series

            currents = []
            torques = []

            for N in speeds:
                omega = 2 * np.pi * N / 60
                Eb = k_phi * omega
                Ia = (V + Eb) / R_total
                T = k_phi * Ia
                currents.append(Ia)
                torques.append(T)

            self.ax_preview.plot(speeds, torques, 'b-', linewidth=2, label='Braking Torque')
            self.ax_preview.axhline(y=self.calculated['T_initial'], color='r',
                                   linestyle='--', label=f'Initial: {self.calculated["T_initial"]:.1f} N·m')
            self.ax_preview.axvline(x=200, color='g', linestyle='--', alpha=0.5,
                                   label='200 rpm')

            self.ax_preview.set_xlabel('Speed (rpm)')
            self.ax_preview.set_ylabel('Braking Torque (N·m)')
            self.ax_preview.set_title('Braking Torque vs Speed')
            self.ax_preview.legend()
            self.ax_preview.grid(True, alpha=0.3)

            self.canvas_preview.draw()

        except Exception as e:
            print(f"Preview plot error: {e}")

    def motor_braking_ode(self, t, y):
        """
        ODE system for motor braking with multi-physics coupling
        y = [omega, Ia, theta] where:
        - omega: angular velocity (rad/s)
        - Ia: armature current (A)
        - theta: temperature rise above ambient (°C)
        """
        omega, Ia, theta = y

        # Get parameters
        V = self.params['V'].get()
        Ra = self.params['Ra'].get()
        R_series = self.calculated['R_series']
        La = self.params['La'].get()
        J = self.params['J'].get()
        B = self.params['B'].get()
        k_phi = self.calculated['k_phi']
        Rth = self.params['thermal_resistance'].get()
        Cth = self.params['thermal_capacitance'].get()

        # Temperature-dependent resistance (copper has positive temp coefficient)
        alpha = 0.00393  # Temperature coefficient for copper (per °C)
        Ra_temp = Ra * (1 + alpha * theta)
        R_total = Ra_temp + R_series

        # Back EMF
        Eb = k_phi * omega

        # Electrical equation (armature circuit)
        # V + Eb = Ia * R_total + La * dIa/dt
        dIa_dt = (V + Eb - Ia * R_total) / La

        # Mechanical equation
        # T_brake - T_friction - J*domega/dt = 0
        T_brake = k_phi * Ia
        T_friction = B * omega
        domega_dt = -(T_brake + T_friction) / J

        # Thermal equation
        # Heat generation - Heat dissipation = Cth * dtheta/dt
        P_copper = Ia**2 * Ra_temp
        P_iron = self.params['iron_loss_coeff'].get() * (omega / (2*np.pi))**1.5
        P_friction = B * omega**2
        P_total = P_copper + P_iron + P_friction

        dtheta_dt = (P_total - theta / Rth) / Cth

        # Thermal derating (reduce torque if overheating)
        T_ambient = self.params['ambient_temp'].get()
        T_current = T_ambient + theta
        T_max = self.params['max_temp'].get()

        if T_current > T_max * 0.9:  # Start derating at 90% of max temp
            derating_factor = max(0, (T_max - T_current) / (T_max * 0.1))
            domega_dt *= derating_factor

        return [domega_dt, dIa_dt, dtheta_dt]

    def motor_braking_euler(self, t_span, y0, dt=0.001):
        """Euler method solver for motor braking ODE"""
        t_start, t_end = t_span
        t = np.arange(t_start, t_end, dt)
        n = len(t)

        # Initialize solution arrays
        y = np.zeros((n, len(y0)))
        y[0] = y0

        # Euler integration
        for i in range(1, n):
            if not self.simulation_running:
                break

            dy = self.motor_braking_ode(t[i-1], y[i-1])
            y[i] = y[i-1] + np.array(dy) * dt

            # Prevent negative values
            y[i, 0] = max(0, y[i, 0])  # omega >= 0
            y[i, 1] = max(0, y[i, 1])  # Ia >= 0
            y[i, 2] = max(0, y[i, 2])  # theta >= 0

            # Stop if motor has stopped
            if y[i, 0] < 0.1:  # Nearly stopped
                y = y[:i+1]
                t = t[:i+1]
                break

        return t, y

    def run_simulation(self):
        """Run the dynamic simulation"""
        try:
            self.status_label.config(text='Running simulation...', foreground='orange')
            self.progress.start()

            # Clear previous results
            for key in self.results:
                self.results[key] = []

            # Initial conditions
            omega0 = 2 * np.pi * self.params['N_initial'].get() / 60
            Ia0 = self.params['Ia_brake_limit'].get()
            theta0 = 0.0  # Initial temperature rise

            y0 = [omega0, Ia0, theta0]

            # Time span
            t_span = (0, 10.0)  # 10 seconds max

            # Solve based on selected method
            if self.solver_type.get() == 'RK45':
                sol = solve_ivp(
                    self.motor_braking_ode,
                    t_span,
                    y0,
                    method='RK45',
                    dense_output=True,
                    max_step=0.01,
                    events=lambda t, y: y[0] - 0.1  # Stop when speed ~ 0
                )
                t = sol.t
                y = sol.y.T
            else:  # Euler
                t, y = self.motor_braking_euler(t_span, y0, dt=0.001)

            # Extract results
            omega = y[:, 0]
            Ia = y[:, 1]
            theta = y[:, 2]

            # Calculate derived quantities
            N = omega * 60 / (2 * np.pi)  # rpm
            k_phi = self.calculated['k_phi']
            T = k_phi * Ia  # Torque
            Eb = k_phi * omega  # Back EMF
            V_supply = self.params['V'].get()
            P_mech = T * omega  # Mechanical power

            # Calculate losses
            Ra = self.params['Ra'].get()
            alpha = 0.00393
            P_copper = Ia**2 * Ra * (1 + alpha * theta)
            P_iron = self.params['iron_loss_coeff'].get() * (omega / (2*np.pi))**1.5
            B = self.params['B'].get()
            P_friction = B * omega**2
            stray_coeff = self.params['stray_loss_coeff'].get() / 100
            P_stray = stray_coeff * P_mech
            P_total_loss = P_copper + P_iron + P_friction + P_stray

            # Calculate mechanical stress
            J = self.params['J'].get()
            shaft_stress = np.abs(np.gradient(omega, t)) * J  # Proportional to angular acceleration
            bearing_load = np.sqrt(T**2 + shaft_stress**2)  # Combined load

            # Efficiency
            P_input = V_supply * Ia
            efficiency = np.where(P_input > 0, (P_input - P_total_loss) / P_input * 100, 0)

            # Store results
            self.results['time'] = t.tolist()
            self.results['speed'] = N.tolist()
            self.results['current'] = Ia.tolist()
            self.results['torque'] = T.tolist()
            self.results['voltage'] = Eb.tolist()
            self.results['power'] = P_mech.tolist()
            self.results['temperature'] = (self.params['ambient_temp'].get() + theta).tolist()
            self.results['copper_loss'] = P_copper.tolist()
            self.results['iron_loss'] = P_iron.tolist()
            self.results['friction_loss'] = P_friction.tolist()
            self.results['stray_loss'] = P_stray.tolist()
            self.results['total_loss'] = P_total_loss.tolist()
            self.results['efficiency'] = efficiency.tolist()
            self.results['shaft_stress'] = shaft_stress.tolist()
            self.results['bearing_load'] = bearing_load.tolist()

            # Update all visualizations
            self.update_all_graphs()
            self.update_thermal_analysis()
            self.update_loss_analysis()
            self.update_economic_analysis()
            self.update_results_display()

            self.status_label.config(text='Simulation completed successfully', foreground='green')
            self.progress.stop()

        except Exception as e:
            self.status_label.config(text=f'Simulation error: {str(e)}', foreground='red')
            self.progress.stop()
            messagebox.showerror('Simulation Error', f'Error during simulation: {str(e)}')
        finally:
            self.simulation_running = False
            self.btn_start.config(state='normal')
            self.btn_stop.config(state='disabled')

    def update_all_graphs(self):
        """Update all dynamic graphs"""
        if not self.results['time']:
            return

        t = np.array(self.results['time'])

        # Speed vs time
        self.ax_speed.clear()
        self.ax_speed.plot(t, self.results['speed'], 'b-', linewidth=2)
        self.ax_speed.axhline(y=200, color='r', linestyle='--', alpha=0.5, label='200 rpm')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.set_title('Motor Speed vs Time')
        self.ax_speed.grid(True, alpha=0.3)
        self.ax_speed.legend()

        # Current vs time
        self.ax_current.clear()
        self.ax_current.plot(t, self.results['current'], 'r-', linewidth=2)
        self.ax_current.axhline(y=self.params['Ia_brake_limit'].get(),
                               color='orange', linestyle='--', alpha=0.5,
                               label='Current Limit')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Armature Current (A)')
        self.ax_current.set_title('Armature Current vs Time')
        self.ax_current.grid(True, alpha=0.3)
        self.ax_current.legend()

        # Torque vs time
        self.ax_torque.clear()
        self.ax_torque.plot(t, self.results['torque'], 'g-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Braking Torque (N·m)')
        self.ax_torque.set_title('Braking Torque vs Time')
        self.ax_torque.grid(True, alpha=0.3)

        # Power vs time
        self.ax_power.clear()
        self.ax_power.plot(t, np.array(self.results['power'])/1000, 'm-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Mechanical Power (kW)')
        self.ax_power.set_title('Power vs Time')
        self.ax_power.grid(True, alpha=0.3)

        self.fig_main.tight_layout()
        self.canvas_main.draw()

    def update_thermal_analysis(self):
        """Update thermal analysis visualization"""
        if not self.results['time']:
            return

        t = np.array(self.results['time'])
        T = np.array(self.results['temperature'])
        T_max = self.params['max_temp'].get()

        # Temperature vs time
        self.ax_temp.clear()
        self.ax_temp.plot(t, T, 'r-', linewidth=2, label='Motor Temperature')
        self.ax_temp.axhline(y=T_max, color='orange', linestyle='--',
                            label=f'Max Temp ({T_max}°C)')
        self.ax_temp.axhline(y=T_max*0.9, color='yellow', linestyle=':',
                            alpha=0.7, label='Derating Threshold')
        self.ax_temp.fill_between(t, T, self.params['ambient_temp'].get(),
                                 alpha=0.3, color='red')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Temperature Profile')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        # Derating curve
        self.ax_derating.clear()
        derating = np.ones_like(T)
        mask = T > T_max * 0.9
        derating[mask] = np.maximum(0, (T_max - T[mask]) / (T_max * 0.1))

        self.ax_derating.plot(t, derating * 100, 'b-', linewidth=2)
        self.ax_derating.fill_between(t, derating * 100, 100, alpha=0.3, color='blue')
        self.ax_derating.set_xlabel('Time (s)')
        self.ax_derating.set_ylabel('Derating Factor (%)')
        self.ax_derating.set_title('Thermal Derating')
        self.ax_derating.set_ylim([0, 105])
        self.ax_derating.grid(True, alpha=0.3)

        self.fig_thermal.tight_layout()
        self.canvas_thermal.draw()

        # Update text info
        self.thermal_text.delete('1.0', tk.END)
        T_max_reached = np.max(T)
        T_avg = np.mean(T)
        T_final = T[-1]

        thermal_info = f"""
THERMAL ANALYSIS REPORT
{'='*50}

Maximum Temperature: {T_max_reached:.2f} °C
Average Temperature: {T_avg:.2f} °C
Final Temperature: {T_final:.2f} °C
Temperature Limit: {T_max:.2f} °C

Temperature Rise: {T_max_reached - self.params['ambient_temp'].get():.2f} °C
Safety Margin: {T_max - T_max_reached:.2f} °C

Thermal Time Constant: {self.params['thermal_resistance'].get() * self.params['thermal_capacitance'].get():.2f} s

Status: {'WARNING - Approaching limit!' if T_max_reached > T_max * 0.9 else 'SAFE - Within limits'}

Thermal Stress Analysis:
- Peak thermal stress occurs at t = {t[np.argmax(T)]:.3f} s
- Thermal cycling: {np.ptp(T):.2f} °C range
- Recommended cooling: {'Active cooling required' if T_max_reached > T_max * 0.8 else 'Natural convection sufficient'}
"""
        self.thermal_text.insert('1.0', thermal_info)

    def update_loss_analysis(self):
        """Update loss breakdown visualization"""
        if not self.results['time']:
            return

        t = np.array(self.results['time'])

        # Calculate average losses
        avg_copper = np.mean(self.results['copper_loss'])
        avg_iron = np.mean(self.results['iron_loss'])
        avg_friction = np.mean(self.results['friction_loss'])
        avg_stray = np.mean(self.results['stray_loss'])

        # Pie chart
        self.ax_loss_pie.clear()
        losses = [avg_copper, avg_iron, avg_friction, avg_stray]
        labels = ['Copper Loss', 'Iron Loss', 'Friction Loss', 'Stray Loss']
        colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']

        wedges, texts, autotexts = self.ax_loss_pie.pie(
            losses, labels=labels, colors=colors, autopct='%1.1f%%',
            startangle=90, textprops={'fontsize': 9}
        )
        self.ax_loss_pie.set_title('Average Loss Breakdown')

        # Time series
        self.ax_loss_time.clear()
        self.ax_loss_time.plot(t, self.results['copper_loss'], label='Copper', color=colors[0])
        self.ax_loss_time.plot(t, self.results['iron_loss'], label='Iron', color=colors[1])
        self.ax_loss_time.plot(t, self.results['friction_loss'], label='Friction', color=colors[2])
        self.ax_loss_time.plot(t, self.results['stray_loss'], label='Stray', color=colors[3])
        self.ax_loss_time.plot(t, self.results['total_loss'], 'k--', linewidth=2, label='Total')
        self.ax_loss_time.set_xlabel('Time (s)')
        self.ax_loss_time.set_ylabel('Power Loss (W)')
        self.ax_loss_time.set_title('Losses vs Time')
        self.ax_loss_time.legend(fontsize=8)
        self.ax_loss_time.grid(True, alpha=0.3)

        self.fig_losses.tight_layout()
        self.canvas_losses.draw()

    def update_economic_analysis(self):
        """Update economic analysis"""
        if not self.results['time']:
            return

        t = np.array(self.results['time'])
        dt = np.diff(t, prepend=0)

        # Calculate energy consumption
        power_kw = np.array(self.results['power']) / 1000
        energy_kwh = np.sum(power_kw * dt) / 3600

        # Calculate losses energy
        loss_kw = np.array(self.results['total_loss']) / 1000
        loss_energy_kwh = np.sum(loss_kw * dt) / 3600

        # Cost calculation
        cost_per_kwh = self.params['cost_per_kwh'].get()
        operating_cost = energy_kwh * cost_per_kwh
        loss_cost = loss_energy_kwh * cost_per_kwh

        # Efficiency
        avg_efficiency = np.mean(self.results['efficiency'])

        # Generate report
        report = f"""
{'='*60}
           ECONOMIC ANALYSIS REPORT
{'='*60}

ENERGY CONSUMPTION:
  Total Energy Consumed:        {energy_kwh:.4f} kWh
  Energy Lost as Heat:          {loss_energy_kwh:.4f} kWh
  Useful Energy:                {energy_kwh - loss_energy_kwh:.4f} kWh

COST ANALYSIS:
  Electricity Rate:             ${cost_per_kwh:.4f} per kWh
  Total Operating Cost:         ${operating_cost:.4f}
  Cost of Losses:               ${loss_cost:.4f}
  Cost Efficiency:              {(1 - loss_cost/operating_cost)*100:.2f}%

EFFICIENCY METRICS:
  Average Efficiency:           {avg_efficiency:.2f}%
  Peak Efficiency:              {np.max(self.results['efficiency']):.2f}%
  Minimum Efficiency:           {np.min(self.results['efficiency']):.2f}%

BRAKING PERFORMANCE:
  Braking Duration:             {t[-1]:.3f} seconds
  Initial Speed:                {self.results['speed'][0]:.1f} rpm
  Final Speed:                  {self.results['speed'][-1]:.1f} rpm
  Speed Reduction:              {self.results['speed'][0] - self.results['speed'][-1]:.1f} rpm

  Initial Torque:               {self.results['torque'][0]:.2f} N·m
  Average Torque:               {np.mean(self.results['torque']):.2f} N·m
  Peak Torque:                  {np.max(self.results['torque']):.2f} N·m

  Total Energy Dissipated:      {np.sum(np.array(self.results['total_loss']) * dt)/1000:.2f} kJ

MECHANICAL STRESS:
  Peak Shaft Stress:            {np.max(self.results['shaft_stress']):.2f} N·m/s
  Average Bearing Load:         {np.mean(self.results['bearing_load']):.2f} N·m
  Peak Bearing Load:            {np.max(self.results['bearing_load']):.2f} N·m

ANNUAL PROJECTION (if operated daily):
  Daily Operations:             1
  Annual Energy:                {energy_kwh * 365:.2f} kWh
  Annual Cost:                  ${operating_cost * 365:.2f}
  Annual Loss Cost:             ${loss_cost * 365:.2f}
  Potential Savings (10% eff.): ${loss_cost * 365 * 0.1:.2f}

RECOMMENDATIONS:
"""

        # Add recommendations
        if avg_efficiency < 85:
            report += "  ⚠ Low efficiency detected - consider motor upgrade\n"
        if loss_cost > operating_cost * 0.2:
            report += "  ⚠ High losses - optimize resistance values\n"
        if np.max(self.results['temperature']) > self.params['max_temp'].get() * 0.9:
            report += "  ⚠ High temperature - improve cooling system\n"
        if np.max(self.results['shaft_stress']) > 100:
            report += "  ⚠ High mechanical stress - check shaft design\n"

        if all([
            avg_efficiency >= 85,
            loss_cost <= operating_cost * 0.2,
            np.max(self.results['temperature']) <= self.params['max_temp'].get() * 0.8
        ]):
            report += "  ✓ System operating within optimal parameters\n"

        report += f"\n{'='*60}\n"
        report += f"Report generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
        report += f"{'='*60}\n"

        self.economic_text.delete('1.0', tk.END)
        self.economic_text.insert('1.0', report)

    def update_results_display(self):
        """Update detailed results display"""
        if not self.results['time']:
            return

        output = "="*80 + "\n"
        output += " "*20 + "SIMULATION RESULTS DATA\n"
        output += "="*80 + "\n\n"

        output += "PROBLEM SOLUTION:\n"
        output += "-"*80 + "\n"
        output += f"(a) Series Resistance Required:        {self.calculated['R_series']:.4f} Ω\n"
        output += f"(b) Initial Braking Torque:            {self.calculated['T_initial']:.4f} N·m\n"
        output += f"(c) Braking Torque at 200 rpm:         {self.calculated['T_at_200rpm']:.4f} N·m\n"
        output += f"    Initial Back EMF:                  {self.calculated['Eb_initial']:.4f} V\n"
        output += f"    Motor Constant (k·Φ):              {self.calculated['k_phi']:.4f} V·s/rad\n"
        output += "\n"

        output += "DYNAMIC SIMULATION DATA:\n"
        output += "-"*80 + "\n"
        output += f"{'Time':>8} {'Speed':>10} {'Current':>10} {'Torque':>10} {'Power':>10} {'Temp':>10} {'Eff':>10}\n"
        output += f"{'(s)':>8} {'(rpm)':>10} {'(A)':>10} {'(N·m)':>10} {'(W)':>10} {'(°C)':>10} {'(%)':>10}\n"
        output += "-"*80 + "\n"

        # Sample every 10th point for display
        for i in range(0, len(self.results['time']), max(1, len(self.results['time'])//50)):
            output += f"{self.results['time'][i]:8.3f} "
            output += f"{self.results['speed'][i]:10.2f} "
            output += f"{self.results['current'][i]:10.2f} "
            output += f"{self.results['torque'][i]:10.2f} "
            output += f"{self.results['power'][i]:10.2f} "
            output += f"{self.results['temperature'][i]:10.2f} "
            output += f"{self.results['efficiency'][i]:10.2f}\n"

        output += "\n" + "="*80 + "\n"

        self.results_text.delete('1.0', tk.END)
        self.results_text.insert('1.0', output)

    def start_simulation(self):
        """Start the simulation in a separate thread"""
        if self.simulation_running:
            return

        # Calculate static values first
        self.calculate_static_values()

        # Check if calculations are valid
        if self.calculated['R_series'] <= 0:
            messagebox.showerror('Error', 'Invalid parameters - please check input values')
            return

        self.simulation_running = True
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')

        # Run simulation in separate thread
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

    def stop_simulation(self):
        """Stop the running simulation"""
        self.simulation_running = False
        self.status_label.config(text='Stopping simulation...', foreground='orange')

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.simulation_running = False

        # Clear results
        for key in self.results:
            self.results[key] = []

        # Clear all plots
        for ax in [self.ax_speed, self.ax_current, self.ax_torque, self.ax_power]:
            ax.clear()
        self.canvas_main.draw()

        for ax in [self.ax_temp, self.ax_derating]:
            ax.clear()
        self.canvas_thermal.draw()

        for ax in [self.ax_loss_pie, self.ax_loss_time]:
            ax.clear()
        self.canvas_losses.draw()

        # Clear text displays
        self.thermal_text.delete('1.0', tk.END)
        self.economic_text.delete('1.0', tk.END)
        self.results_text.delete('1.0', tk.END)

        self.status_label.config(text='Reset complete - Ready', foreground='green')
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')

    def export_results(self):
        """Export results to CSV file"""
        if not self.results['time']:
            messagebox.showwarning('No Data', 'No simulation data to export')
            return

        try:
            filename = f"braking_simulation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

            with open(filename, 'w') as f:
                # Header
                f.write("Time(s),Speed(rpm),Current(A),Torque(Nm),Power(W),")
                f.write("Temp(C),CopperLoss(W),IronLoss(W),FrictionLoss(W),")
                f.write("StrayLoss(W),TotalLoss(W),Efficiency(%),ShaftStress,BearingLoad\n")

                # Data
                for i in range(len(self.results['time'])):
                    f.write(f"{self.results['time'][i]:.6f},")
                    f.write(f"{self.results['speed'][i]:.4f},")
                    f.write(f"{self.results['current'][i]:.4f},")
                    f.write(f"{self.results['torque'][i]:.4f},")
                    f.write(f"{self.results['power'][i]:.4f},")
                    f.write(f"{self.results['temperature'][i]:.4f},")
                    f.write(f"{self.results['copper_loss'][i]:.4f},")
                    f.write(f"{self.results['iron_loss'][i]:.4f},")
                    f.write(f"{self.results['friction_loss'][i]:.4f},")
                    f.write(f"{self.results['stray_loss'][i]:.4f},")
                    f.write(f"{self.results['total_loss'][i]:.4f},")
                    f.write(f"{self.results['efficiency'][i]:.4f},")
                    f.write(f"{self.results['shaft_stress'][i]:.4f},")
                    f.write(f"{self.results['bearing_load'][i]:.4f}\n")

            messagebox.showinfo('Export Success', f'Data exported to {filename}')

        except Exception as e:
            messagebox.showerror('Export Error', f'Failed to export: {str(e)}')

    def copy_results(self):
        """Copy results to clipboard"""
        if not self.results['time']:
            messagebox.showwarning('No Data', 'No simulation data to copy')
            return

        try:
            self.root.clipboard_clear()
            text = self.results_text.get('1.0', tk.END)
            self.root.clipboard_append(text)
            messagebox.showinfo('Success', 'Results copied to clipboard')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to copy: {str(e)}')


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = DCMotorBrakingSimulator(root)
    root.mainloop()


if __name__ == '__main__':
    main()
