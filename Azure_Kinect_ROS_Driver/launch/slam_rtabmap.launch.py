import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Paths to configuration files
    azure_kinect_ros_driver_pkg_dir = get_package_share_directory('azure_kinect_ros_driver')
    rtabmap_ros_pkg_dir = get_package_share_directory('rtabmap_ros')
    
    # Get the URDF file for the camera from your driver package
    urdf_file_path = os.path.join(azure_kinect_ros_driver_pkg_dir, 'urdf', 'azure_kinect.urdf.xacro')

    # Get the default RViz configuration file from rtabmap_ros
    rviz_config_file = os.path.join(rtabmap_ros_pkg_dir, 'launch', 'config', 'rgbd.rviz')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Define parameters for the RTAB-Map nodes
    rtabmap_parameters = {
        'frame_id': 'camera_base',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'use_sim_time': use_sim_time,
        'qos_image': 2,
        'qos_imu': 2,
        'Reg/Strategy': '1', # 0=Vis, 1=Icp, 2=VisIcp
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Vis/MinInliers': '15',
        'Odom/ResetCountdown': '10',
        'Grid/FromDepth': 'true'
    }

    # Remappings to connect the Kinect driver topics to RTAB-Map's expected topics
    rtabmap_remapping = [
        ('rgb/image', '/rgb/image_raw'),
        ('rgb/camera_info', '/rgb/camera_info'),
        ('depth/image', '/depth_to_rgb/image_raw') # Use registered depth image
    ]
    
    return LaunchDescription([
        
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_path]),
                'use_sim_time': use_sim_time
            }]
        ),

        # Azure Kinect ROS Driver Node
        Node(
            package='azure_kinect_ros_driver',
            executable='node',
            name='k4a',
            parameters=[{
                'depth_enabled': True,
                'depth_mode': 'NFOV_UNBINNED',
                'color_enabled': True,
                'color_resolution': '720P',
                'fps': 30,
                'point_cloud': True,
                'rgb_point_cloud': True,
                'point_cloud_in_depth_frame': False,
                'synchronized_images_only': True,
                'imu_rate_target': 100,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        # RTAB-Map Visual Odometry Node
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remapping,
            arguments=['--delete_db_on_start']
        ),

        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remapping,
            arguments=['--delete_db_on_start']
        ),
        
        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])