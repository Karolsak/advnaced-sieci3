"""
DC Motor Braking Simulator - Non-GUI Demo Version
This version runs complete simulations without requiring a GUI display.
Perfect for servers, headless systems, or automated testing.
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from datetime import datetime
import os


class DCMotorBrakingEngine:
    """Core simulation engine without GUI dependencies"""

    def __init__(self):
        # Motor parameters (from problem)
        self.V = 250.0  # Supply voltage (V)
        self.Ia_initial = 150.0  # Initial armature current (A)
        self.N_initial = 550.0  # Initial speed (rpm)
        self.Ra = 0.09  # Armature resistance (Ω)
        self.Ia_brake_limit = 240.0  # Braking current limit (A)
        self.J = 2.5  # Moment of inertia (kg·m²)
        self.B = 0.05  # Friction coefficient (N·m·s)
        self.Rf = 125.0  # Field resistance (Ω)
        self.La = 0.02  # Armature inductance (H)
        self.thermal_resistance = 0.8  # Thermal resistance (°C/W)
        self.thermal_capacitance = 1200.0  # Thermal capacitance (J/°C)
        self.ambient_temp = 25.0  # Ambient temperature (°C)
        self.max_temp = 155.0  # Maximum allowed temperature (°C)
        self.iron_loss_coeff = 50.0  # Iron loss coefficient (W)
        self.stray_loss_coeff = 0.01  # Stray loss coefficient (1%)
        self.cost_per_kwh = 0.12  # Electricity cost ($/kWh)

        # Calculated values
        self.R_series = 0.0
        self.k_phi = 0.0
        self.Eb_initial = 0.0

    def calculate_static_values(self):
        """Calculate theoretical braking values"""
        print("\n" + "="*80)
        print(" "*20 + "THEORETICAL CALCULATIONS")
        print("="*80 + "\n")

        # Calculate back EMF
        self.Eb_initial = self.V - self.Ia_initial * self.Ra
        print(f"Initial Back EMF: Eb = V - Ia·Ra = {self.V} - {self.Ia_initial}×{self.Ra}")
        print(f"                  Eb = {self.Eb_initial:.4f} V\n")

        # Calculate motor constant
        omega_initial = 2 * np.pi * self.N_initial / 60
        self.k_phi = self.Eb_initial / omega_initial
        print(f"Motor Constant: k·Φ = Eb/ω = {self.Eb_initial:.4f}/{omega_initial:.4f}")
        print(f"                k·Φ = {self.k_phi:.4f} V·s/rad\n")

        # (a) Series resistance
        self.R_series = (self.V + self.Eb_initial) / self.Ia_brake_limit - self.Ra
        print(f"(a) Series Resistance Required:")
        print(f"    R_series = (V + Eb)/Ia_brake - Ra")
        print(f"    R_series = ({self.V} + {self.Eb_initial:.4f})/{self.Ia_brake_limit} - {self.Ra}")
        print(f"    R_series = {self.R_series:.4f} Ω\n")

        # (b) Initial braking torque
        T_initial = self.k_phi * self.Ia_brake_limit
        print(f"(b) Initial Braking Torque:")
        print(f"    T = k·Φ × Ia = {self.k_phi:.4f} × {self.Ia_brake_limit}")
        print(f"    T = {T_initial:.4f} N·m\n")

        # (c) Torque at 200 rpm
        omega_200 = 2 * np.pi * 200 / 60
        Eb_200 = self.k_phi * omega_200
        Ia_200 = (self.V + Eb_200) / (self.Ra + self.R_series)
        T_200 = self.k_phi * Ia_200
        print(f"(c) Braking Torque at 200 rpm:")
        print(f"    Eb_200 = k·Φ × ω_200 = {self.k_phi:.4f} × {omega_200:.4f} = {Eb_200:.4f} V")
        print(f"    Ia_200 = (V + Eb_200)/(Ra + R_series) = {Ia_200:.4f} A")
        print(f"    T_200 = k·Φ × Ia_200 = {T_200:.4f} N·m\n")

        print("="*80 + "\n")

        return {
            'R_series': self.R_series,
            'T_initial': T_initial,
            'T_200': T_200,
            'k_phi': self.k_phi,
            'Eb_initial': self.Eb_initial
        }

    def motor_braking_ode(self, t, y):
        """ODE system for motor braking dynamics"""
        omega, Ia, theta = y

        # Temperature-dependent resistance
        alpha = 0.00393
        Ra_temp = self.Ra * (1 + alpha * theta)
        R_total = Ra_temp + self.R_series

        # Back EMF
        Eb = self.k_phi * omega

        # Electrical equation
        dIa_dt = (self.V + Eb - Ia * R_total) / self.La

        # Mechanical equation
        T_brake = self.k_phi * Ia
        T_friction = self.B * omega
        domega_dt = -(T_brake + T_friction) / self.J

        # Thermal equation
        P_copper = Ia**2 * Ra_temp
        P_iron = self.iron_loss_coeff * (omega / (2*np.pi))**1.5
        P_friction = self.B * omega**2
        P_total = P_copper + P_iron + P_friction

        dtheta_dt = (P_total - theta / self.thermal_resistance) / self.thermal_capacitance

        # Thermal derating
        T_current = self.ambient_temp + theta
        if T_current > self.max_temp * 0.9:
            derating = max(0, (self.max_temp - T_current) / (self.max_temp * 0.1))
            domega_dt *= derating

        return [domega_dt, dIa_dt, dtheta_dt]

    def run_simulation(self, solver='RK45'):
        """Run dynamic simulation"""
        print("="*80)
        print(" "*20 + "DYNAMIC SIMULATION")
        print("="*80 + "\n")

        print(f"Solver: {solver}")
        print(f"Simulating motor braking from {self.N_initial} rpm to stop...\n")

        # Initial conditions
        omega0 = 2 * np.pi * self.N_initial / 60
        Ia0 = self.Ia_brake_limit
        theta0 = 0.0

        y0 = [omega0, Ia0, theta0]
        t_span = (0, 10.0)

        # Solve ODE
        print("Solving coupled differential equations...")
        if solver == 'RK45':
            sol = solve_ivp(
                self.motor_braking_ode,
                t_span,
                y0,
                method='RK45',
                dense_output=True,
                max_step=0.01,
                events=lambda t, y: y[0] - 0.1
            )
            t = sol.t
            y = sol.y.T
        else:  # Euler
            t, y = self.euler_solve(t_span, y0)

        print(f"✓ Simulation completed: {len(t)} time points\n")

        # Extract results
        omega = y[:, 0]
        Ia = y[:, 1]
        theta = y[:, 2]

        # Calculate derived quantities
        N = omega * 60 / (2 * np.pi)
        T = self.k_phi * Ia
        Eb = self.k_phi * omega
        P_mech = T * omega

        # Losses
        alpha = 0.00393
        P_copper = Ia**2 * self.Ra * (1 + alpha * theta)
        P_iron = self.iron_loss_coeff * (omega / (2*np.pi))**1.5
        P_friction = self.B * omega**2
        P_stray = self.stray_loss_coeff * P_mech
        P_total_loss = P_copper + P_iron + P_friction + P_stray

        # Temperature
        temp = self.ambient_temp + theta

        # Mechanical stress
        shaft_stress = np.abs(np.gradient(omega, t)) * self.J
        bearing_load = np.sqrt(T**2 + shaft_stress**2)

        # Efficiency
        P_input = self.V * Ia
        efficiency = np.where(P_input > 0, (P_input - P_total_loss) / P_input * 100, 0)

        results = {
            'time': t,
            'speed': N,
            'current': Ia,
            'torque': T,
            'voltage': Eb,
            'power': P_mech,
            'temperature': temp,
            'copper_loss': P_copper,
            'iron_loss': P_iron,
            'friction_loss': P_friction,
            'stray_loss': P_stray,
            'total_loss': P_total_loss,
            'efficiency': efficiency,
            'shaft_stress': shaft_stress,
            'bearing_load': bearing_load,
        }

        return results

    def euler_solve(self, t_span, y0, dt=0.001):
        """Euler method solver"""
        t_start, t_end = t_span
        t = np.arange(t_start, t_end, dt)
        n = len(t)

        y = np.zeros((n, len(y0)))
        y[0] = y0

        for i in range(1, n):
            dy = self.motor_braking_ode(t[i-1], y[i-1])
            y[i] = y[i-1] + np.array(dy) * dt
            y[i, 0] = max(0, y[i, 0])
            y[i, 1] = max(0, y[i, 1])
            y[i, 2] = max(0, y[i, 2])

            if y[i, 0] < 0.1:
                y = y[:i+1]
                t = t[:i+1]
                break

        return t, y

    def print_summary(self, results):
        """Print simulation summary"""
        print("="*80)
        print(" "*25 + "SIMULATION SUMMARY")
        print("="*80 + "\n")

        t = results['time']
        dt = np.diff(t, prepend=0)

        # Energy calculations
        energy_kwh = np.sum(results['power'] * dt) / 3600 / 1000
        loss_kwh = np.sum(results['total_loss'] * dt) / 3600 / 1000
        cost = energy_kwh * self.cost_per_kwh

        print(f"Braking Duration:             {t[-1]:.3f} seconds")
        print(f"Initial Speed:                {results['speed'][0]:.1f} rpm")
        print(f"Final Speed:                  {results['speed'][-1]:.1f} rpm")
        print(f"Speed Reduction:              {results['speed'][0] - results['speed'][-1]:.1f} rpm\n")

        print(f"Initial Torque:               {results['torque'][0]:.2f} N·m")
        print(f"Average Torque:               {np.mean(results['torque']):.2f} N·m")
        print(f"Peak Torque:                  {np.max(results['torque']):.2f} N·m\n")

        print(f"Peak Temperature:             {np.max(results['temperature']):.2f} °C")
        print(f"Temperature Limit:            {self.max_temp:.2f} °C")
        print(f"Safety Margin:                {self.max_temp - np.max(results['temperature']):.2f} °C\n")

        print(f"Average Efficiency:           {np.mean(results['efficiency']):.2f} %")
        print(f"Peak Efficiency:              {np.max(results['efficiency']):.2f} %\n")

        print(f"Energy Consumed:              {energy_kwh:.6f} kWh")
        print(f"Energy Lost:                  {loss_kwh:.6f} kWh")
        print(f"Operating Cost:               ${cost:.6f}\n")

        print(f"Peak Shaft Stress:            {np.max(results['shaft_stress']):.2f} N·m/s")
        print(f"Peak Bearing Load:            {np.max(results['bearing_load']):.2f} N·m\n")

        print(f"Average Copper Loss:          {np.mean(results['copper_loss']):.2f} W")
        print(f"Average Iron Loss:            {np.mean(results['iron_loss']):.2f} W")
        print(f"Average Friction Loss:        {np.mean(results['friction_loss']):.2f} W")
        print(f"Average Total Loss:           {np.mean(results['total_loss']):.2f} W\n")

        print("="*80 + "\n")

    def generate_plots(self, results, output_dir='output'):
        """Generate and save plots"""
        print("="*80)
        print(" "*25 + "GENERATING PLOTS")
        print("="*80 + "\n")

        os.makedirs(output_dir, exist_ok=True)

        t = results['time']

        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))

        # Speed
        ax1 = plt.subplot(3, 3, 1)
        ax1.plot(t, results['speed'], 'b-', linewidth=2)
        ax1.axhline(y=200, color='r', linestyle='--', alpha=0.5, label='200 rpm')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Speed (rpm)')
        ax1.set_title('Motor Speed vs Time')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Current
        ax2 = plt.subplot(3, 3, 2)
        ax2.plot(t, results['current'], 'r-', linewidth=2)
        ax2.axhline(y=self.Ia_brake_limit, color='orange', linestyle='--',
                   alpha=0.5, label=f'Limit: {self.Ia_brake_limit}A')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Armature Current (A)')
        ax2.set_title('Current vs Time')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Torque
        ax3 = plt.subplot(3, 3, 3)
        ax3.plot(t, results['torque'], 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Braking Torque (N·m)')
        ax3.set_title('Braking Torque vs Time')
        ax3.grid(True, alpha=0.3)

        # Power
        ax4 = plt.subplot(3, 3, 4)
        ax4.plot(t, np.array(results['power'])/1000, 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Mechanical Power (kW)')
        ax4.set_title('Power vs Time')
        ax4.grid(True, alpha=0.3)

        # Temperature
        ax5 = plt.subplot(3, 3, 5)
        ax5.plot(t, results['temperature'], 'r-', linewidth=2)
        ax5.axhline(y=self.max_temp, color='orange', linestyle='--',
                   label=f'Max: {self.max_temp}°C')
        ax5.fill_between(t, results['temperature'], self.ambient_temp, alpha=0.3)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Temperature (°C)')
        ax5.set_title('Temperature Profile')
        ax5.grid(True, alpha=0.3)
        ax5.legend()

        # Efficiency
        ax6 = plt.subplot(3, 3, 6)
        ax6.plot(t, results['efficiency'], 'c-', linewidth=2)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Efficiency (%)')
        ax6.set_title('Efficiency vs Time')
        ax6.grid(True, alpha=0.3)

        # Loss breakdown (pie chart)
        ax7 = plt.subplot(3, 3, 7)
        avg_losses = [
            np.mean(results['copper_loss']),
            np.mean(results['iron_loss']),
            np.mean(results['friction_loss']),
            np.mean(results['stray_loss'])
        ]
        labels = ['Copper', 'Iron', 'Friction', 'Stray']
        colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']
        ax7.pie(avg_losses, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax7.set_title('Average Loss Breakdown')

        # Losses vs time
        ax8 = plt.subplot(3, 3, 8)
        ax8.plot(t, results['copper_loss'], label='Copper', color=colors[0])
        ax8.plot(t, results['iron_loss'], label='Iron', color=colors[1])
        ax8.plot(t, results['friction_loss'], label='Friction', color=colors[2])
        ax8.plot(t, results['stray_loss'], label='Stray', color=colors[3])
        ax8.plot(t, results['total_loss'], 'k--', linewidth=2, label='Total')
        ax8.set_xlabel('Time (s)')
        ax8.set_ylabel('Power Loss (W)')
        ax8.set_title('Losses vs Time')
        ax8.legend(fontsize=8)
        ax8.grid(True, alpha=0.3)

        # Mechanical stress
        ax9 = plt.subplot(3, 3, 9)
        ax9.plot(t, results['shaft_stress'], label='Shaft Stress', color='purple')
        ax9.plot(t, results['bearing_load'], label='Bearing Load', color='brown')
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('Stress/Load (N·m)')
        ax9.set_title('Mechanical Stress Analysis')
        ax9.legend(fontsize=8)
        ax9.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save plot
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{output_dir}/braking_simulation_{timestamp}.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Plot saved: {filename}\n")

        plt.close()

    def export_csv(self, results, output_dir='output'):
        """Export results to CSV"""
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{output_dir}/braking_data_{timestamp}.csv'

        with open(filename, 'w') as f:
            # Header
            f.write("Time(s),Speed(rpm),Current(A),Torque(Nm),Power(W),")
            f.write("Temp(C),CopperLoss(W),IronLoss(W),FrictionLoss(W),")
            f.write("StrayLoss(W),TotalLoss(W),Efficiency(%),ShaftStress,BearingLoad\n")

            # Data
            for i in range(len(results['time'])):
                f.write(f"{results['time'][i]:.6f},")
                f.write(f"{results['speed'][i]:.4f},")
                f.write(f"{results['current'][i]:.4f},")
                f.write(f"{results['torque'][i]:.4f},")
                f.write(f"{results['power'][i]:.4f},")
                f.write(f"{results['temperature'][i]:.4f},")
                f.write(f"{results['copper_loss'][i]:.4f},")
                f.write(f"{results['iron_loss'][i]:.4f},")
                f.write(f"{results['friction_loss'][i]:.4f},")
                f.write(f"{results['stray_loss'][i]:.4f},")
                f.write(f"{results['total_loss'][i]:.4f},")
                f.write(f"{results['efficiency'][i]:.4f},")
                f.write(f"{results['shaft_stress'][i]:.4f},")
                f.write(f"{results['bearing_load'][i]:.4f}\n")

        print(f"✓ Data exported: {filename}\n")
        print("="*80 + "\n")


def main():
    """Main execution function"""
    print("\n")
    print("╔" + "="*78 + "╗")
    print("║" + " "*15 + "DC MOTOR BRAKING MULTI-PHYSICS SIMULATOR" + " "*22 + "║")
    print("║" + " "*22 + "Non-GUI Demo Version" + " "*37 + "║")
    print("╚" + "="*78 + "╝")

    # Create engine
    engine = DCMotorBrakingEngine()

    # Calculate static values
    static_results = engine.calculate_static_values()

    # Run dynamic simulation
    dynamic_results = engine.run_simulation(solver='RK45')

    # Print summary
    engine.print_summary(dynamic_results)

    # Generate plots
    engine.generate_plots(dynamic_results)

    # Export data
    engine.export_csv(dynamic_results)

    print("╔" + "="*78 + "╗")
    print("║" + " "*20 + "SIMULATION COMPLETED SUCCESSFULLY!" + " "*24 + "║")
    print("╚" + "="*78 + "╝")
    print("\nCheck the 'output' directory for plots and data files.")
    print("\nTo run the full GUI version, execute:")
    print("  python3 dc_motor_braking_advanced_simulator.py\n")


if __name__ == '__main__':
    main()
