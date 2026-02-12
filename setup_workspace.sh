#!/bin/bash
# Quick setup script for rb_ws workspace

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f ~/rb_ws/install/setup.bash ]; then
    source ~/rb_ws/install/setup.bash
    echo "✓ rb_ws workspace sourced"
else
    echo "⚠ Warning: ~/rb_ws/install/setup.bash not found. Build the workspace first:"
    echo "  cd ~/rb_ws && colcon build"
fi

# Source virtual environment if it exists
if [ -f ~/rb_ws/src/rb10_APF_Proximity/apf_venv/bin/activate ]; then
    source ~/rb_ws/src/rb10_APF_Proximity/apf_venv/bin/activate
    echo "✓ APF virtual environment activated"
fi

