#!/bin/bash

### This script can be run to build box bot package

# Source underlay
source /opt/ros/jazzy/setup.bash

# Start venv
source ~/venv/bin/activate

# Build package
DBUS_SESSION_BUS_ADDRESS="" _PYTHON_INTERPRETER=/home/boxbot/venv/bin/python3 colcon build --symlink-install