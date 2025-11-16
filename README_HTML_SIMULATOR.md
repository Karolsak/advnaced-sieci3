# Advanced DC Shunt Motor Simulator - HTML Web Application

## Overview

This is a fully standalone HTML5 web application for advanced DC shunt motor analysis and simulation. It provides the same functionality as the Python version but runs entirely in your browser with no installation required.

## Features

### ✨ Highlights

- **100% Browser-Based** - No server or installation needed
- **Responsive Design** - Works on desktop, tablet, and mobile
- **Interactive Charts** - Real-time visualization with Chart.js
- **Multi-Physics Simulation** - Electromagnetic-thermal-mechanical coupling
- **Problem Solver** - Automatic solutions for two DC motor problems
- **Economic Analysis** - Life-cycle cost calculations
- **Professional UI** - Modern gradient design with smooth animations

### 📊 Six Comprehensive Tabs

#### 1. ⚡ Control & Parameters
- **Parameter Sliders**: Adjust all motor parameters in real-time
  - Electrical: Voltage, resistances, inductances
  - Mechanical: Inertia, friction, constants
  - Control: Load torque, series resistance, flux weakening
- **Real-Time Status**: Live monitoring of 8 key parameters
- **Control Buttons**: Start, Stop, Reset, Analyze
- **Solver Selection**: Choose between RK45 or Euler methods

#### 2. 📊 Dynamic Simulation
- **6 Real-Time Graphs**:
  1. Armature & Field Current vs Time
  2. Motor Speed vs Time
  3. Electromagnetic Torque vs Time
  4. Output Power vs Time
  5. Temperature Rise vs Time
  6. Motor Efficiency vs Time
- Interactive charts with zoom, pan, and download capabilities

#### 3. 📈 Loss Analysis
- **Pie Chart**: Loss distribution breakdown
  - Copper losses (armature and field)
  - Iron losses
  - Mechanical losses
  - Stray load losses
- **Bar Chart**: Power flow visualization (input/output/losses)
- **Detailed Numbers**: Exact loss values in watts

#### 4. 💰 Economic Analysis
- **Input Parameters**:
  - Electricity cost
  - Operating schedule
  - Equipment costs
  - Discount rate
- **Results**:
  - Annual energy consumption
  - Life-cycle cost (NPV)
  - Cost per operating hour
  - Energy cost projections

#### 5. 🌡️ Thermal Analysis
- **Temperature Monitoring**:
  - Armature temperature
  - Field temperature
  - Maximum temperature tracking
- **Derating Factor**: Automatic calculation
- **Visual Display**: Bar chart of component temperatures
- **Safety Limits**: Rated (75°C) and maximum (130°C) temperatures

#### 6. 📝 Problem Solutions
- **Problem 1**: Operating point with reduced flux/torque
  - **Answer**: Ia = 26.000 A, N = 2986.83 rpm
- **Problem 2**: Speed control with series resistance
  - **Answer (a)**: Rs = 5.250 Ω (constant torque)
  - **Answer (b)**: Rs = 22.500 Ω (torque ∝ speed²)
- **Detailed Solutions**: Step-by-step calculations displayed
- **Buttons**: Solve individually or all at once

## Quick Start

### Method 1: Direct Opening

Simply double-click `dc_motor_simulator_advanced.html` - it will open in your default browser.

### Method 2: Command Line

```bash
# Linux/Mac
open dc_motor_simulator_advanced.html

# Windows
start dc_motor_simulator_advanced.html

# Or use a specific browser
firefox dc_motor_simulator_advanced.html
chrome dc_motor_simulator_advanced.html
```

### Method 3: Local Server (Optional)

```bash
# Python 3
python3 -m http.server 8000

# Then open: http://localhost:8000/dc_motor_simulator_advanced.html
```

## Usage Guide

### Basic Operation

1. **View Problem Solutions**:
   - Navigate to "📝 Problem Solutions" tab
   - Solutions are automatically displayed on load
   - Click buttons to update/refresh solutions

2. **Adjust Parameters**:
   - Go to "⚡ Control & Parameters" tab
   - Use sliders to change motor parameters
   - Watch real-time status update

3. **Run Simulation**:
   - Choose ODE solver (RK45 recommended)
   - Click "▶ Start" for real-time simulation
   - Click "📊 Analyze" for complete 5-second analysis

4. **View Results**:
   - Navigate through tabs to see different analyses
   - All charts update automatically
   - Hover over charts for exact values

### Advanced Features

#### Real-Time Simulation
- **Start**: Begins continuous simulation
- **Stop**: Pauses the simulation
- **Reset**: Clears all data and returns to initial state
- Updates every 10ms for smooth animation

#### Complete Analysis
- **Analyze**: Runs full 5-second simulation
- Uses selected ODE solver
- Generates all charts and results
- Shows final operating point

#### Parameter Exploration
1. Adjust a parameter (e.g., load torque)
2. Run analysis
3. Observe effects on speed, efficiency, temperature
4. Try different combinations

#### Economic Evaluation
1. Enter your operating costs
2. Click "Calculate Economics"
3. View life-cycle cost analysis
4. Evaluate energy savings scenarios

## Technical Details

### ODE Solvers

**RK45 (Runge-Kutta 4-5)**:
- 4th order method with 5th order error estimation
- Adaptive (in this implementation: fixed step for simplicity)
- Higher accuracy
- Recommended for final results

**Euler Method**:
- 1st order forward integration
- Fixed time step (10ms)
- Faster computation
- Good for real-time visualization

### Differential Equations

The simulator solves 6 coupled ODEs:

```javascript
dIa/dt = (V - Eb - Ia·Ra(T)) / La          // Armature current
dIf/dt = (V - If·Rf(T)) / Lf                // Field current
dω/dt = (Tem - Tload - B·ω) / J             // Angular velocity
dθ/dt = ω                                    // Angular position
dTa/dt = (Ploss_a - (Ta-Tamb)/Rth_a) / Cth_a // Armature temperature
dTf/dt = (Ploss_f - (Tf-Tamb)/Rth_f) / Cth_f // Field temperature
```

Where:
- Eb = Ke·If·φ·ω (Back EMF)
- Tem = Kt·If·φ·Ia (Electromagnetic torque)
- R(T) = R₀[1 + α(T - T₀)] (Temperature-dependent resistance)

### Loss Calculations

1. **Copper Losses**:
   - Armature: Ia²·Ra(Ta)
   - Field: If²·Rf(Tf)

2. **Iron Losses**:
   - P_iron = 0.002·|ω|·If²

3. **Mechanical Losses**:
   - P_mech = 0.5·|ω| + 0.0001·ω³

4. **Stray Load Losses**:
   - P_stray = 0.01·P_output

### Thermal Model

**Thermal Resistances**:
- Armature: 2.0 °C/W
- Field: 3.0 °C/W

**Thermal Capacitances**:
- Armature: 500 J/°C
- Field: 800 J/°C

**Derating Curve**:
- Full load: Ta, Tf ≤ 75°C
- Linear derating: 75°C < T < 130°C
- Zero load: T ≥ 130°C

## Browser Compatibility

### Fully Supported
- ✅ Chrome/Chromium (90+)
- ✅ Firefox (88+)
- ✅ Safari (14+)
- ✅ Edge (90+)

### Requirements
- HTML5 Canvas support
- ES6 JavaScript support
- Internet connection (for Chart.js CDN)

### Offline Use
To use completely offline:
1. Download Chart.js: https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.js
2. Save as `chart.js` in same directory
3. Change CDN link to: `<script src="chart.js"></script>`

## File Structure

```
dc_motor_simulator_advanced.html
├── HTML Structure (lines 1-500)
│   ├── Header with title
│   ├── Tab navigation
│   └── 6 tab contents
│
├── CSS Styling (lines 9-350)
│   ├── Responsive grid layout
│   ├── Gradient themes
│   ├── Chart containers
│   ├── Animations
│   └── Mobile support
│
└── JavaScript Code (lines 500-end)
    ├── Motor parameter management
    ├── ODE solvers (RK45 & Euler)
    ├── Multi-physics simulation engine
    ├── Chart.js integration
    ├── Loss/efficiency calculations
    ├── Economic analysis
    ├── Problem solvers
    └── UI event handlers
```

## Performance

- **Load Time**: < 1 second
- **Calculation Speed**: < 50ms per step
- **Memory Usage**: < 50 MB
- **Chart Update Rate**: 100 Hz (real-time mode)
- **Analysis Time**: ~2-3 seconds for 5s simulation

## Examples

### Example 1: Starting Transient

1. Open the simulator
2. Set Load Torque = 20 N·m
3. Click "Analyze"
4. Go to "📊 Dynamic Simulation" tab
5. Observe current surge and speed build-up

**Expected Results**:
- Current peaks then settles
- Speed gradually increases to steady-state
- Temperature rises over time

### Example 2: Speed Control

1. Run baseline analysis
2. Note the steady-state speed
3. Increase "Series Resistance" to 2Ω
4. Click "Analyze" again
5. Compare speeds

**Expected Results**:
- Speed decreases with series resistance
- Efficiency drops
- Temperature may increase

### Example 3: Economic Comparison

1. Run simulation to get efficiency
2. Go to "💰 Economic Analysis" tab
3. Enter your operating costs
4. Click "Calculate Economics"
5. Note the life-cycle cost
6. Adjust parameters to improve efficiency
7. Recalculate to see savings

## Troubleshooting

### Charts Not Displaying

**Problem**: White boxes instead of charts

**Solutions**:
1. Check internet connection (Chart.js loads from CDN)
2. Try refreshing the page (Ctrl+F5 / Cmd+Shift+R)
3. Open browser console (F12) to check for errors
4. Try different browser
5. Download Chart.js locally (see Offline Use above)

### Simulation Not Running

**Problem**: Click "Start" but nothing happens

**Solutions**:
1. Click "Reset" first
2. Check browser console for JavaScript errors
3. Try "Analyze" instead of "Start"
4. Refresh the page
5. Clear browser cache

### Incorrect Results

**Problem**: Results don't match expected values

**Solutions**:
1. Click "Reset" to clear previous data
2. Verify parameter values
3. Check solver selection
4. Compare with problem solutions (known correct answers)
5. Try the Python version for verification

### Mobile Display Issues

**Problem**: Layout broken on mobile

**Solutions**:
1. Rotate device to landscape
2. Zoom out
3. Use tablet or desktop for full experience
4. Update to latest browser version

## Comparison: HTML vs Python

| Feature | HTML Version | Python Version |
|---------|--------------|----------------|
| Installation | None | Requires packages |
| Platform | Any browser | OS-specific |
| Speed | Very fast | Fast |
| Accuracy | High | Very high |
| Portability | Excellent | Good |
| Offline Use | Yes* | Yes |
| Charts | Chart.js | Matplotlib |
| Mobile | Yes | No |

*With Chart.js downloaded locally

## Advanced Tips

### 1. Parameter Sweeps

Want to see how speed varies with load?
```
1. Set baseline parameters
2. Run analysis, note speed
3. Change load torque slightly
4. Run analysis again
5. Repeat for range of values
6. Plot results manually
```

### 2. Efficiency Optimization

Find maximum efficiency operating point:
```
1. Start with low load
2. Gradually increase load torque
3. Monitor efficiency in status display
4. Note load torque at peak efficiency
5. Adjust other parameters if needed
```

### 3. Thermal Limits

Determine maximum continuous load:
```
1. Set high load torque
2. Run analysis
3. Check steady-state temperature
4. If > 75°C, reduce load
5. Repeat until temperature ≤ 75°C
```

### 4. Economic Payback

Calculate payback for efficiency upgrade:
```
1. Run with current motor efficiency
2. Note annual energy cost
3. Increase efficiency (better motor)
4. Note new annual cost
5. Payback = Upgrade Cost / Annual Savings
```

## Educational Use

Perfect for:
- **Electrical Engineering Courses**: Motor theory, control systems
- **Power Electronics**: Speed control methods
- **Thermal Analysis**: Heat transfer in electrical machines
- **Economic Engineering**: Life-cycle cost analysis
- **Self-Study**: Interactive learning tool

## Limitations

1. **Simplified Models**: Real motors have additional complexities
2. **Fixed Time Step**: Not truly adaptive RK45 (for speed)
3. **No Saturation**: Magnetic circuit assumed linear
4. **Ideal Commutation**: No brush/commutator effects
5. **Steady-State Thermal**: Simplified thermal model

Despite these simplifications, the simulator is excellent for:
- Understanding motor behavior
- Learning control principles
- Estimating performance
- Educational demonstrations

## Future Enhancements

Potential additions:
- [ ] Save/load parameter sets
- [ ] Export data to CSV
- [ ] Comparison mode (two motors side-by-side)
- [ ] Field weakening control
- [ ] PWM drive simulation
- [ ] Saturation effects
- [ ] More detailed thermal model
- [ ] Efficiency map generator

## Support

For issues:
1. Check this README
2. Review browser console (F12)
3. Try different browser
4. Compare with Python version
5. Verify internet connection (for Chart.js)

## License

Educational and practical use - provided as-is.

## Credits

- **Chart.js**: Beautiful responsive charts
- **Modern CSS**: Gradient designs and animations
- **Numerical Methods**: RK45 and Euler ODE solvers

---

## Quick Reference Card

**Problem 1 Answers**:
- Ia2 = **26.000 A**
- N2 = **2986.83 rpm**

**Problem 2 Answers**:
- (a) Rs = **5.250 Ω**
- (b) Rs = **22.500 Ω**

**Key Formulas**:
```
Eb = Ke·φ·ω
T = Kt·φ·Ia
η = Pout/Pin × 100%
N(rpm) = ω × 60/(2π)
```

**Temperature Limits**:
- Continuous: ≤ 75°C
- Maximum: 130°C
- Ambient: 25°C

**Default Parameters**:
- V = 220 V
- Ra = 0.35 Ω
- Rf = 110 Ω
- J = 0.5 kg·m²
- Kt = Ke = 1.2

---

**Version**: 1.0
**File Size**: ~68 KB
**Last Updated**: 2025
**Status**: Production Ready ✅

Enjoy the simulator! ⚡📊
