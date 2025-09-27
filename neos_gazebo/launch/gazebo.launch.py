from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package paths
    neos_gazebo_pkg_path = get_package_share_directory('neos_gazebo')
    neos_description_pkg_path = get_package_share_directory('neos_description')
    
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
                
                # Joint states
                '/model/neos/joint_state@sensor_msgs/msg/JointState@gz.msgs.JointState',
                
                # Pose information (use PoseStamped for TF compatibility)
                '/model/neos/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose_V',
            ],
            remappings=[
                # Remap to standard ROS2 topic names
                ('/model/neos/cmd_vel', '/cmd_vel'),
                ('/model/neos/odometry', '/odom'),
                ('/model/neos/joint_state', '/joint_states'),
                ('/model/neos/pose', '/robot_pose'),
            ],
            output='screen'
        ),
        
        # Robot State Publisher (for TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': True,
                'robot_description': Command([
                    'xacro ', PathJoinSubstitution([
                        neos_description_pkg_path, 'urdf', 'neos.xacro'
                    ])
                ])
            }],
            output='screen'
        ),
        
        # Joint State Publisher (for manual joint control)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': True
            }],
            output='screen'
        ),
    ])