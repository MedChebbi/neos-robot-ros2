# File Conversion Commands

## XACRO to URDF
```bash
xacro neos.xacro > neos.urdf
```

## URDF to SDF
```bash
gz sdf -p neos.urdf > neos.sdf
```

## XACRO to SDF (one step)
```bash
xacro neos.xacro | gz sdf -p > neos.sdf
```
