# Source your ROS version's default setup
source /opt/ros/humble/setup.bash

# Source the local setup
source install/setup.bash

# colcon searchs the `src` directory for the package.xml file with the 
# package name `depth_sub` and builds it. `package.xml` is located in the `src/depth_sub` directory.
# The following dependencies from http://download.ros.org/schema/package_format3.xsd
# The package will be in the `resource` directory of the workspace
# rclpy: ROS Client Library for the Python language.
# std_msgs: Standard ROS Messages including common message types representing primitive data types
#           and other basic message constructs, such as multiarrays.
# geometry_msgs: provides messages for common geometric primitives such as points, vectors, and poses
# cv_bridge: for image data
# sensor_msgs: sensor data
colcon build --packages-select depth_sub


# Run the depth_sub package using using depth_subscriber: depth_sub.py's main function
ros2 run depth_sub depth_subscriber

# SSH into the capstone project device (if needed)
# ssh capstone@mdc-nx.local