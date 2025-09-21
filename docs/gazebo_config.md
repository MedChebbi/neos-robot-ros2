# Gazebo Configuration Documentation

## Overview
This document contains the complete Gazebo configuration for the N.E.O.S robot, including differential drive plugin settings, world configurations, and testing procedures.

## Differential Drive Plugin Configuration

### Plugin Location
File: `neos_gazebo/models/neos.sdf`

### Complete Plugin Configuration
```xml
<plugin
  filename="gz-sim-diff-drive-system"
  name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.3</wheel_separation>
  <wheel_radius>0.05</wheel_radius>
  <odom_publish_frequency>50</odom_publish_frequency>
  <max_linear_acceleration>1.0</max_linear_acceleration>
  <min_linear_acceleration>-1.0</min_linear_acceleration>
  <max_angular_acceleration>2.0</max_angular_acceleration>
  <min_angular_acceleration>-2.0</min_angular_acceleration>
  <max_linear_velocity>0.5</max_linear_velocity>
  <min_linear_velocity>-0.5</min_linear_velocity>
  <max_angular_velocity>1.0</max_angular_velocity>
  <min_angular_velocity>-1.0</min_angular_velocity>
  <topic>cmd_vel</topic>
</plugin>
```

### Parameter Explanations

| Parameter | Value | Description |
|-----------|-------|-------------|
| `left_joint` | `left_wheel_joint` | Name of the left wheel joint |
| `right_joint` | `right_wheel_joint` | Name of the right wheel joint |
| `wheel_separation` | `0.3` | Distance between wheel centers (30cm) |
| `wheel_radius` | `0.05` | Wheel radius (5cm) |
| `odom_publish_frequency` | `50` | Odometry publishing frequency (Hz) |
| `max_linear_velocity` | `0.5` | Maximum linear velocity (m/s) |
| `max_angular_velocity` | `1.0` | Maximum angular velocity (rad/s) |
| `topic` | `cmd_vel` | Command topic for velocity control |

## Robot Dimensions

### Base Link
- **Shape**: Cylinder
- **Diameter**: 0.4m (40cm)
- **Height**: 0.1m (10cm)
- **Mass**: 10kg

### Wheels
- **Type**: Driven wheels
- **Radius**: 0.05m (5cm)
- **Width**: 0.02m (2cm)
- **Separation**: 0.3m (30cm)
- **Mass**: 0.5kg each

### Caster Wheels
- **Type**: Spherical casters
- **Radius**: 0.03m (3cm)
- **Mass**: 0.1kg each
- **Position**: Front and rear of robot

## World Files

### Available Worlds
1. **`empty.sdf`** - Basic empty world with ground plane
2. **`simple_world.sdf`** - Simple world with basic furniture
3. **`living_room.sdf`** - Complex living room environment

### World Usage
```bash
# Launch with specific world
ros2 launch neos_gazebo gazebo.launch.py world:=simple_world.sdf
```

## Testing Commands

### Basic Movement Tests
```bash
# Forward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"

# Backward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: -0.5, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"

# Left rotation
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.5}"

# Right rotation
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -0.5}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### Continuous Movement
```bash
# Move forward continuously
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.3, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```

## Troubleshooting

### Common Issues

1. **Robot not moving**
   - Check if differential drive plugin is loaded
   - Verify joint names match URDF
   - Check wheel separation and radius values

2. **Robot moving too fast/slow**
   - Adjust `max_linear_velocity` and `max_angular_velocity`
   - Modify acceleration limits

3. **Odometry not publishing**
   - Check `odom_publish_frequency` setting
   - Verify topic names

### Debug Commands
```bash
# Check if plugin is loaded
gz topic -l | grep cmd_vel

# Monitor odometry
ros2 topic echo /odom

# Check joint states
ros2 topic echo /joint_states
```

## File Structure
```
neos_gazebo/
├── models/
│   └── neos.sdf          # Robot model with differential drive plugin
├── worlds/
│   ├── empty.sdf         # Empty world
│   ├── simple_world.sdf  # Simple world with furniture
│   └── living_room.sdf   # Complex living room
├── launch/
│   └── gazebo.launch.py  # Gazebo launch file
└── docs/
    └── gazebo_config.md  # This documentation file
```

## References
- [Gazebo Sim Documentation](https://gazebosim.org/docs)
- [ROS2 Navigation](https://navigation.ros.org/)
- [Differential Drive Plugin](https://github.com/gazebo-sim/gz-sim/tree/main/src/systems/diff_drive)
