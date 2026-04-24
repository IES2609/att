#!/bin/bash
# Setup script for CSV capture environment

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

echo "Setting up Python virtual environment..."

# Create virtual environment
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    if [ ! -f "$VENV_DIR/bin/python3" ]; then
        echo "✗ Failed to create virtual environment"
        echo "Make sure python3-venv is installed: sudo apt install python3.12-venv"
        exit 1
    fi
    echo "✓ Virtual environment created at $VENV_DIR"
else
    echo "✓ Virtual environment already exists"
fi

# Activate virtual environment and install dependencies
echo "Installing dependencies..."
"$VENV_DIR/bin/pip" install --upgrade pip setuptools wheel
"$VENV_DIR/bin/pip" install -r requirements_csv_capture.txt

if [ $? -eq 0 ]; then
    echo "✓ Dependencies installed successfully"
else
    echo "✗ Failed to install dependencies"
    exit 1
fi

echo ""
echo "✓ Setup complete! To use the script:"
echo ""
echo "Option 1 - Direct execution (recommended):"
echo "  $VENV_DIR/bin/python3 capture_csv_from_device.py -p /dev/ttyACM0 -b 1000000"
echo ""
echo "Option 2 - Activate environment first:"
echo "  source $VENV_DIR/bin/activate"
echo "  python3 capture_csv_from_device.py -p /dev/ttyACM0 -b 1000000"
echo ""
