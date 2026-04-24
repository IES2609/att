# Setup script for CSV capture environment on Windows (PowerShell)
# Run with: powershell -ExecutionPolicy Bypass -File setup_env.ps1

$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$VENV_DIR = Join-Path $SCRIPT_DIR ".venv"

Write-Host "Setting up Python virtual environment..." -ForegroundColor Green
Write-Host ""

# Check if Python is installed
try {
    $pythonVersion = & python --version 2>&1
    Write-Host "Python found: $pythonVersion"
} catch {
    Write-Host "X Failed to find Python" -ForegroundColor Red
    Write-Host "Please install Python 3.8+ from https://www.python.org/" -ForegroundColor Yellow
    Write-Host "Make sure to check 'Add Python to PATH' during installation" -ForegroundColor Yellow
    Read-Host "Press Enter to exit"
    exit 1
}

# Create virtual environment
if (-not (Test-Path $VENV_DIR)) {
    Write-Host "Creating virtual environment..."
    & python -m venv $VENV_DIR
    if ($LASTEXITCODE -ne 0) {
        Write-Host "X Failed to create virtual environment" -ForegroundColor Red
        Read-Host "Press Enter to exit"
        exit 1
    }
    Write-Host "OK Virtual environment created at $VENV_DIR" -ForegroundColor Green
} else {
    Write-Host "OK Virtual environment already exists" -ForegroundColor Green
}

Write-Host ""
Write-Host "Installing dependencies..."

$PIP_EXE = Join-Path $VENV_DIR "Scripts\pip.exe"

& $PIP_EXE install --upgrade pip setuptools wheel
if ($LASTEXITCODE -ne 0) {
    Write-Host "X Failed to upgrade pip" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

& $PIP_EXE install -r requirements_csv_capture.txt
if ($LASTEXITCODE -ne 0) {
    Write-Host "X Failed to install dependencies" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "OK Dependencies installed successfully" -ForegroundColor Green
Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "Setup complete! To use the script:" -ForegroundColor Green
Write-Host ""
Write-Host "Option 1 - Direct execution (recommended):" -ForegroundColor Cyan
Write-Host "  .\setup_env.ps1  # Run this script first" -ForegroundColor White
$pythonExe = Join-Path $VENV_DIR "Scripts\python.exe"
Write-Host "  $pythonExe capture_csv_from_device.py -p COM3 -b 1000000" -ForegroundColor White
Write-Host ""
Write-Host "Option 2 - Activate environment first:" -ForegroundColor Cyan
$activateScript = Join-Path $VENV_DIR "Scripts\Activate.ps1"
Write-Host "  & '$activateScript'" -ForegroundColor White
Write-Host "  python capture_csv_from_device.py -p COM3 -b 1000000" -ForegroundColor White
Write-Host ""
Write-Host "Note: Replace COM3 with your actual serial port number" -ForegroundColor Yellow
Write-Host "      Use -l or --list flag to see available ports" -ForegroundColor Yellow
Write-Host "============================================" -ForegroundColor Green
Write-Host ""
Read-Host "Press Enter to exit"
