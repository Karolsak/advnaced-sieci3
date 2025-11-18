#!/bin/bash

# DC Motor Braking Simulator Launcher Script
# This script checks dependencies and launches the simulator

echo "======================================================================"
echo "    DC Motor Braking Multi-Physics Simulator Launcher"
echo "======================================================================"
echo ""

# Check Python version
echo "Checking Python version..."
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Found Python $PYTHON_VERSION"

REQUIRED_VERSION="3.7"
if ! python3 -c "import sys; exit(0 if sys.version_info >= (3, 7) else 1)"; then
    echo "ERROR: Python 3.7 or higher is required!"
    echo "Please upgrade Python and try again."
    exit 1
fi
echo "✓ Python version OK"
echo ""

# Check for required packages
echo "Checking required packages..."

check_package() {
    if python3 -c "import $1" 2>/dev/null; then
        echo "✓ $1 installed"
        return 0
    else
        echo "✗ $1 NOT installed"
        return 1
    fi
}

MISSING_PACKAGES=0

check_package "numpy" || MISSING_PACKAGES=1
check_package "matplotlib" || MISSING_PACKAGES=1
check_package "scipy" || MISSING_PACKAGES=1
check_package "tkinter" || MISSING_PACKAGES=1

echo ""

if [ $MISSING_PACKAGES -eq 1 ]; then
    echo "ERROR: Some required packages are missing!"
    echo ""
    echo "To install missing packages, run:"
    echo "  pip3 install -r requirements.txt"
    echo ""
    echo "For tkinter (if missing):"
    echo "  Ubuntu/Debian: sudo apt-get install python3-tk"
    echo "  Fedora: sudo dnf install python3-tkinter"
    echo ""
    exit 1
fi

echo "All dependencies satisfied!"
echo ""
echo "======================================================================"
echo "Launching DC Motor Braking Simulator..."
echo "======================================================================"
echo ""

# Launch the simulator
python3 dc_motor_braking_advanced_simulator.py

exit_code=$?

if [ $exit_code -ne 0 ]; then
    echo ""
    echo "======================================================================"
    echo "Simulator exited with error code: $exit_code"
    echo "======================================================================"
fi

exit $exit_code
