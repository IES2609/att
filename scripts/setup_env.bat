@echo off
REM Setup script for CSV capture environment on Windows

setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set VENV_DIR=%SCRIPT_DIR%.venv

echo Setting up Python virtual environment...
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo X Failed to find Python
    echo Please install Python 3.8+ from https://www.python.org/
    echo Make sure to check "Add Python to PATH" during installation
    pause
    exit /b 1
)

echo Python found:
python --version

REM Create virtual environment
if not exist "%VENV_DIR%" (
    echo Creating virtual environment...
    python -m venv "%VENV_DIR%"
    if errorlevel 1 (
        echo X Failed to create virtual environment
        pause
        exit /b 1
    )
    echo. OK Virtual environment created at %VENV_DIR%
) else (
    echo. OK Virtual environment already exists
)

echo.
echo Installing dependencies...
"%VENV_DIR%\Scripts\pip" install --upgrade pip setuptools wheel
if errorlevel 1 (
    echo X Failed to upgrade pip
    pause
    exit /b 1
)

"%VENV_DIR%\Scripts\pip" install -r requirements_csv_capture.txt
if errorlevel 1 (
    echo X Failed to install dependencies
    pause
    exit /b 1
)

echo. OK Dependencies installed successfully
echo.
echo ============================================
echo Setup complete! To use the script:
echo.
echo Option 1 - Direct execution (recommended):
echo   %VENV_DIR%\Scripts\python.exe capture_csv_from_device.py -p COM3 -b 1000000
echo.
echo Option 2 - Activate environment first:
echo   %VENV_DIR%\Scripts\activate.bat
echo   python capture_csv_from_device.py -p COM3 -b 1000000
echo.
echo Note: Replace COM3 with your actual serial port number
echo       Use -l or --list flag to see available ports
echo ============================================
echo.
pause
