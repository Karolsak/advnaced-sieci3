# DC Motor Braking Simulator - Quick Start Guide

## Problem Statement

**A 250-V d.c. shunt motor, taking an armature current of 150 A and running at 550 r.p.m. is braked by reversing the connections to the armature and inserting additional resistance in series with it.**

### Find:
- **(a)** The value of series resistance required to limit the initial current to 240 A
- **(b)** The initial value of braking torque
- **(c)** The value of braking torque when the speed has fallen to 200 r.p.m

### Answers:
- **(a)** R_series = **1.94 Ω**
- **(b)** T_initial = **985.49 N·m**
- **(c)** T_at_200rpm = **680.62 N·m**

---

## Installation

### 1. Prerequisites
- Python 3.7 or higher
- pip (Python package manager)

### 2. Install Dependencies

```bash
pip3 install -r requirements.txt
```

This will install:
- numpy (numerical computing)
- matplotlib (plotting)
- scipy (scientific computing)

**Note:** tkinter usually comes with Python. If missing:
- **Ubuntu/Debian:** `sudo apt-get install python3-tk`
- **Fedora:** `sudo dnf install python3-tkinter`
- **macOS:** Included with Python from python.org
- **Windows:** Included with standard Python

---

## Running the Simulator

### Option 1: Full GUI Version (Recommended)

#### Linux/macOS:
```bash
./run_simulator.sh
```

#### Windows:
```batch
run_simulator.bat
```

#### Direct Python:
```bash
python3 dc_motor_braking_advanced_simulator.py
```

### Option 2: Quick Test (Console Only)

```bash
python3 test_braking_simulator.py
```

This runs theoretical calculations without GUI.

### Option 3: Non-GUI Demo with Plots

```bash
python3 demo_simulation.py
```

This runs full simulation and generates plots in `output/` directory (no GUI required).

---

## Using the GUI Simulator

### Step 1: Launch Application
Run the simulator using one of the methods above. The GUI will open with 6 tabs.

### Step 2: Verify Parameters (Main Control Tab)
- Default values are pre-loaded from the problem
- Adjust parameters using:
  - **Text Entry:** Type exact values
  - **Sliders:** Adjust key parameters interactively

### Step 3: Calculate Static Values
Click **"Calculate"** button to compute:
- Series resistance required
- Initial back EMF
- Motor constant (k·Φ)
- Initial braking torque
- Torque at 200 rpm

Results appear in the "Calculated Values" panel.

### Step 4: Run Dynamic Simulation
1. Select ODE Solver:
   - **RK45** (recommended): Adaptive, accurate
   - **Euler**: Fixed step, educational

2. Click **"Start Simulation"**

3. Watch progress:
   - Status indicator shows "Running simulation..."
   - Progress bar animates
   - Simulation stops automatically when motor stops

### Step 5: Analyze Results

#### Dynamic Graphs Tab
- **Speed vs Time:** Motor deceleration curve
- **Current vs Time:** Current variation with limit line
- **Torque vs Time:** Braking torque profile
- **Power vs Time:** Mechanical power dissipation

#### Thermal Analysis Tab
- **Temperature Profile:** Real-time temperature tracking
- **Thermal Derating:** Automatic protection curve
- **Thermal Information:** Safety margins and recommendations

#### Loss Breakdown Tab
- **Pie Chart:** Average loss distribution
- **Time Series:** Individual losses over time
  - Copper losses (I²R)
  - Iron losses (eddy + hysteresis)
  - Friction losses
  - Stray losses

#### Economic Analysis Tab
- Energy consumption and costs
- Efficiency metrics
- Annual projections
- Optimization recommendations

#### Results & Data Tab
- Detailed data table
- **Export to CSV:** Save for Excel/MATLAB
- **Copy to Clipboard:** Quick sharing

### Step 6: Export Results (Optional)

1. Go to **Results & Data** tab
2. Click **"Export to CSV"**
3. File saved with timestamp in current directory
4. Use in Excel, MATLAB, Python, etc.

---

## Quick Reference

### Control Buttons

| Button | Function |
|--------|----------|
| **Calculate** | Compute static theoretical values |
| **Start Simulation** | Run dynamic simulation |
| **Stop** | Stop running simulation |
| **Reset** | Clear all results and plots |
| **Export to CSV** | Save data to file |
| **Copy to Clipboard** | Copy results text |

### Key Parameters

| Parameter | Symbol | Default | Unit |
|-----------|--------|---------|------|
| Supply Voltage | V | 250 | V |
| Armature Resistance | Ra | 0.09 | Ω |
| Braking Current Limit | Ia_brake | 240 | A |
| Initial Speed | N | 550 | rpm |
| Moment of Inertia | J | 2.5 | kg·m² |
| Friction Coefficient | B | 0.05 | N·m·s |

### ODE Solvers

| Solver | Pros | Cons | Use When |
|--------|------|------|----------|
| **RK45** | Adaptive step, accurate, event detection | Slightly slower | Production, accurate results |
| **Euler** | Simple, fast, educational | Fixed step, less accurate | Learning, quick tests |

---

## Output Files

### CSV Data File Format
Columns:
- Time (s)
- Speed (rpm)
- Current (A)
- Torque (N·m)
- Power (W)
- Temperature (°C)
- Copper Loss (W)
- Iron Loss (W)
- Friction Loss (W)
- Stray Loss (W)
- Total Loss (W)
- Efficiency (%)
- Shaft Stress (N·m/s)
- Bearing Load (N·m)

### Plot Files
- PNG format, 150 DPI
- 9-panel comprehensive visualization
- Timestamp in filename

---

## Troubleshooting

### Problem: "ModuleNotFoundError: No module named 'numpy'"
**Solution:**
```bash
pip3 install numpy matplotlib scipy
```

### Problem: "ModuleNotFoundError: No module named 'tkinter'"
**Solution:**
- **Ubuntu/Debian:** `sudo apt-get install python3-tk`
- **Fedora:** `sudo dnf install python3-tkinter`

### Problem: GUI doesn't open
**Solution:**
1. Check if running in headless environment (server without display)
2. Use `demo_simulation.py` instead
3. Verify X11/display server is running

### Problem: "Invalid parameters" error
**Solution:**
1. Ensure all parameters are positive
2. Check series resistance is calculated (click "Calculate")
3. Verify realistic values (e.g., J = 1-10 kg·m²)

### Problem: Simulation too slow
**Solution:**
1. Use Euler solver instead of RK45
2. Reduce simulation time
3. Close other applications

### Problem: Temperature too high
**Solution:**
1. Increase thermal resistance (better cooling)
2. Reduce braking current limit
3. Add cooling fan (reduce thermal resistance)

---

## File Structure

```
advnaced-sieci3/
├── dc_motor_braking_advanced_simulator.py  # Main GUI application
├── test_braking_simulator.py               # Quick theoretical test
├── demo_simulation.py                      # Non-GUI demo with plots
├── run_simulator.sh                        # Linux/Mac launcher
├── run_simulator.bat                       # Windows launcher
├── requirements.txt                        # Python dependencies
├── DC_MOTOR_BRAKING_README.md             # Full documentation
├── QUICKSTART.md                          # This file
└── output/                                # Generated plots and data
    ├── braking_simulation_*.png
    └── braking_data_*.csv
```

---

## Common Workflows

### Workflow 1: Solve the Problem (Fastest)
```bash
python3 test_braking_simulator.py
```
Get answers in seconds.

### Workflow 2: Full Analysis (Recommended)
```bash
python3 dc_motor_braking_advanced_simulator.py
```
1. Click "Calculate"
2. Click "Start Simulation"
3. Review all tabs
4. Export data if needed

### Workflow 3: Batch Processing (No GUI)
```bash
python3 demo_simulation.py
```
Check `output/` directory for results.

### Workflow 4: Parameter Study
1. Launch GUI
2. Adjust parameter (e.g., inertia J)
3. Run simulation
4. Note results
5. Adjust parameter again
6. Compare results

### Workflow 5: Export for Report
1. Run simulation
2. Go to "Results & Data" tab
3. Click "Export to CSV"
4. Open in Excel/MATLAB
5. Create custom plots/tables

---

## Physics Behind the Simulation

### Electrical Model
```
V + Eb = Ia × (Ra + R_series) + La × dIa/dt
Eb = k·Φ × ω
```

### Mechanical Model
```
T_brake - T_friction = J × dω/dt
T_brake = k·Φ × Ia
T_friction = B × ω
```

### Thermal Model
```
Cth × dT/dt = P_total - T/Rth
P_total = P_copper + P_iron + P_friction + P_stray
```

### Temperature Effects
```
Ra(T) = Ra₀ × (1 + α × ΔT)
α = 0.00393/°C (copper)
```

---

## Tips for Best Results

1. **Always click "Calculate" before "Start Simulation"**
2. **Use RK45 solver for accurate results**
3. **Check thermal limits** - ensure temperature stays below max
4. **Export data** for detailed analysis in other tools
5. **Compare solvers** - run both RK45 and Euler to see differences
6. **Save plots** before closing application
7. **Adjust inertia** to match real motor (typical: 1-5 kg·m²)
8. **Monitor efficiency** - should be 85-95% for good design

---

## Educational Value

This simulator demonstrates:
- **Multi-physics coupling** (electrical-thermal-mechanical)
- **Numerical ODE solving** (RK45, Euler methods)
- **Dynamic motor behavior** during braking
- **Thermal management** and derating
- **Loss analysis** and efficiency
- **Economic optimization**
- **Professional GUI design** with Python/Tkinter

Perfect for:
- Electrical engineering students
- Control systems courses
- Motor design projects
- Industrial training
- Research work

---

## Next Steps

1. **Run the simulator** with default parameters
2. **Explore all tabs** to understand features
3. **Experiment with parameters** to see effects
4. **Export and analyze data** in your preferred tool
5. **Read full documentation** in DC_MOTOR_BRAKING_README.md
6. **Modify code** for your specific application

---

## Support

For issues or questions:
- Check troubleshooting section above
- Read full documentation: `DC_MOTOR_BRAKING_README.md`
- Review code comments in Python files
- Test with `test_braking_simulator.py` first

---

## Quick Command Reference

```bash
# Install dependencies
pip3 install -r requirements.txt

# Run GUI (Linux/Mac)
./run_simulator.sh

# Run GUI (Windows)
run_simulator.bat

# Run GUI (direct)
python3 dc_motor_braking_advanced_simulator.py

# Quick test
python3 test_braking_simulator.py

# Non-GUI demo
python3 demo_simulation.py

# Check Python version
python3 --version

# Verify packages
python3 -c "import numpy, matplotlib, scipy, tkinter; print('All packages OK')"
```

---

**Enjoy exploring DC motor braking dynamics!** 🚀
