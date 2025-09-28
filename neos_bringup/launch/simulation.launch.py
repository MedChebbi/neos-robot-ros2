from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('neos_gazebo'), 'worlds', 'simple_world.sdf'
        ]),
        description='Path to the world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,
        
        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('neos_gazebo'), 'launch', 'gazebo.launch.py'
                ])
            ),
            launch_arguments={
                'world_file': LaunchConfiguration('world_file')
            }.items(),
        ),
        
        # Launch robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('neos_bringup'), 'launch', 'robot.launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items(),
        ),
    ])
