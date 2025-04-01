#!/bin/bash

# Source ROS2
echo "Sourcing ROS2"
source /opt/ros/foxy/setup.bash

# Source ROS2 Virtual Environment
echo "Sourcing ROS2 Virtual Environment"
source /opt/ros2_venv/bin/activate

# Add virtualenv to PYTHONPATH
echo "Adding virtualenv to PYTHONPATH"
export PYTHONPATH=/opt/ros2_venv/lib/python3.8/site-packages/:${PYTHONPATH}

# Source the workspace
echo "Sourcing the workspace"
source install/setup.sh

# Set the ROS_DOMAIN_ID
echo "Setting ROS_DOMAIN_ID to 30"
export ROS_DOMAIN_ID=30

ros2 run mdc_controller talker