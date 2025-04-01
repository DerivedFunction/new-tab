#!/bin/bash
source /opt/ros/humble/setup.bash
make clean
colcon build --packages-select mdc_car laptop depth_sub
chmod +x *.sh
