#!/bin/bash

### This script can be run to start boxbot, replacing the need to run many command line arguments.

# Activate Python virtual environment
source /home/boxbot/venv/bin/activate

# Source ROS 
source /home/boxbot/boxbot_ws/install/setup.bash

# Launch ROS2 nodes from launch file
ros2 launch boxbot_main boxbot_launch.py
