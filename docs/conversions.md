# File Conversion Commands

## Automated Conversion (Recommended)
Use the Python script to convert XACRO to SDF while preserving Gazebo plugins:

```bash
# From the neos_bringup/scripts directory
python3 update_urdf_sdf.py

# With backup of existing SDF
python3 update_urdf_sdf.py --backup

# Custom file paths
python3 update_urdf_sdf.py --xacro custom.xacro --urdf custom.urdf --sdf custom.sdf
```

This script automatically:
- Converts XACRO → URDF → SDF
- Preserves existing Gazebo plugins (diff-drive, etc.)
- Creates backups if requested

## Manual Conversion (Not Recommended)
⚠️ **Warning**: Manual conversion will lose Gazebo plugins!

### XACRO to URDF
```bash
xacro neos.xacro > neos.urdf
```

### URDF to SDF
```bash
gz sdf -p neos.urdf > neos.sdf
```