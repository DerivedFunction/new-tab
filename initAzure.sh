source /opt/ros/humble/setup.bash
cd Azure_Kinect_ROS_Driver
chmod +x *.sh
colcon build
./runAzure.sh
