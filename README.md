# N.E.O.S - ROS2 Learning Project

A differential drive robot for learning ROS2, Perception, and Nav2.

## Robot Description
- **Type**: Differential drive robot
- **Wheels**: 2 driven wheels + 2 caster wheels
- **Dimensions**: 40cm diameter base, 10cm height
- **Control**: geometry_msgs/Twist

## Packages
- `neos_description`: Robot URDF/XACRO and visualization
- `neos_gazebo`: Gazebo simulation
- `neos_bringup`: Launch files and system integration
- `neos_perception`: Perception algorithms (Python)

## Quick Start

### Prerequisites
- ROS2 kilted

### Build
```bash
cd /path/to/robot_ws
colcon build
source install/setup.bash
```

### Launch
```bash
# Visualize in RViz
ros2 launch neos_description display.launch.py model:=urdf/neos.urdf

# Launch in Gazebo (coming soon)
ros2 launch neos_gazebo gazebo.launch.py
```

## Learning Path
1. Robot Description & URDF
2. Gazebo Simulation & Teleop
3. Perception Tasks
4. Nav2 Integration

## License
Apache-2.0
