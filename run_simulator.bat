@echo off
REM DC Motor Braking Simulator Launcher Script for Windows

echo ======================================================================
echo     DC Motor Braking Multi-Physics Simulator Launcher
echo ======================================================================
echo.

echo Checking Python installation...
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH!
    echo Please install Python 3.7 or higher from python.org
    pause
    exit /b 1
)

python --version
echo.

echo Checking required packages...
python -c "import numpy, matplotlib, scipy, tkinter" >nul 2>&1
if errorlevel 1 (
    echo ERROR: Some required packages are missing!
    echo.
    echo To install missing packages, run:
    echo   pip install -r requirements.txt
    echo.
    pause
    exit /b 1
)

echo All dependencies satisfied!
echo.
echo ======================================================================
echo Launching DC Motor Braking Simulator...
echo ======================================================================
echo.

python dc_motor_braking_advanced_simulator.py

if errorlevel 1 (
    echo.
    echo ======================================================================
    echo Simulator exited with an error
    echo ======================================================================
    pause
)

exit /b %errorlevel%
