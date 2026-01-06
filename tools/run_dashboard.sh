#!/bin/bash
# Run the MotorLab Dashboard
# This script handles the virtual environment automatically

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

# Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    source "$VENV_DIR/bin/activate"
    pip install -r "$SCRIPT_DIR/requirements.txt"
else
    source "$VENV_DIR/bin/activate"
fi

# Set Qt plugin path for PyQt6
export QT_QPA_PLATFORM_PLUGIN_PATH="$VENV_DIR/lib/python3.14/site-packages/PyQt6/Qt6/plugins"
export QT_PLUGIN_PATH="$VENV_DIR/lib/python3.14/site-packages/PyQt6/Qt6/plugins"

# Run the dashboard
python "$SCRIPT_DIR/motorlab_dashboard.py"
