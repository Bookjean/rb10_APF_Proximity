#!/bin/bash
# Setup script for rb10_APF_Proximity virtual environment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/apf_venv"

echo "=========================================="
echo "Setting up rb10_APF_Proximity virtual environment"
echo "=========================================="

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
else
    echo "Virtual environment already exists at $VENV_DIR"
fi

# Activate virtual environment
echo "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Check if PyKDL is available (should be installed via apt)
echo "Checking for PyKDL..."
if ! python3 -c "import PyKDL" 2>/dev/null; then
    echo "WARNING: PyKDL not found. Please install it via apt:"
    echo "  sudo apt install python3-orocos-kdl"
    echo "  or"
    echo "  sudo apt install ros-humble-orocos-kdl"
else
    echo "PyKDL is available"
fi

# Install requirements
echo "Installing Python dependencies..."
pip install -r "$SCRIPT_DIR/requirements.txt"

# Create ROS2 paths file for venv (so ROS2 packages can be found)
echo "Setting up ROS2 Python path..."
VENV_SITE_PACKAGES="$VENV_DIR/lib/python3.10/site-packages"
if [ -d "$VENV_SITE_PACKAGES" ]; then
    ROS2_PATHS_FILE="$VENV_SITE_PACKAGES/ros2_paths.pth"
    cat > "$ROS2_PATHS_FILE" << EOF
/opt/ros/humble/lib/python3.10/site-packages
/opt/ros/humble/local/lib/python3.10/dist-packages
/usr/lib/python3/dist-packages
EOF
    echo "Created ROS2 paths file: $ROS2_PATHS_FILE"
fi

echo ""
echo "=========================================="
echo "Virtual environment setup complete!"
echo "=========================================="
echo ""
echo "To activate the virtual environment, run:"
echo "  source $VENV_DIR/bin/activate"
echo ""
echo "Or use the alias (add to ~/.bashrc):"
echo "  alias apf_venv='source $VENV_DIR/bin/activate'"
echo ""
echo "Note: The virtual environment is named 'apf_venv' to avoid confusion with other venvs."
echo ""

