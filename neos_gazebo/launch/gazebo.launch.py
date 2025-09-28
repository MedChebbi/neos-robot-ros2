from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package paths
    neos_gazebo_pkg_path = get_package_share_directory('neos_gazebo')
    
    # Find ros_gz_sim package
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([neos_gazebo_pkg_path, 'worlds', 'simple_world.sdf']),
        description='Path to the world file'
    )
    
    return LaunchDescription([
        world_file_arg,
        
        # Set environment variables
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([neos_gazebo_pkg_path, 'models'])
        ),
        
        # Launch Gazebo Sim using official launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={
                'gz_args': LaunchConfiguration('world_file')
            }.items(),
        ),

        # Bridge robot control topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Robot control
                '/model/neos/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/neos/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                
                # LiDAR sensor
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            ],
            remappings=[
                # Remap to standard ROS2 topic names
                ('/model/neos/cmd_vel', '/cmd_vel'),
                ('/model/neos/odometry', '/odom'),
                ('/scan', '/scan'),
            ],
            output='screen'
        ),
        
        # Static transform for LiDAR frame (Gazebo naming)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_publisher',
            arguments=['0', '0', '0.09', '0', '0', '0', 'base_link', 'neos/base_link/lidar_sensor'],
            output='screen'
        ),
    ])