# Camera Sensor Rules From Competition Spec

This workspace follows the camera part of section `2-6-2. Sensor`.

## Allowed Cameras

- Maximum camera count: 4
- Model: Camera
- Ground Truth: None
- Viz Bounding Box 2D/3D: Off
- Network: UDP
- Frame Rate: 30 Hz or lower

## Fixed Cameras

Front:

- Position `(x, y, z)`: `1.9, 0.0, 1.2 m`
- Rotation `(roll, pitch, yaw)`: `0, 2, 0 deg`
- Max resolution: `1280 x 720`
- FOV: `90 deg`
- ROS topic: `/sensors/camera/front/compressed`
- Default Ubuntu receive port: `1001`

Left:

- Position `(x, y, z)`: `1.15, 0.65, 1.2 m`
- Rotation `(roll, pitch, yaw)`: `0, 10, 70 deg`
- Max resolution: `640 x 480`
- FOV: `130 deg`
- ROS topic: `/sensors/camera/left/compressed`
- Default Ubuntu receive port: `1011`

Right:

- Position `(x, y, z)`: `1.15, -0.65, 1.2 m`
- Rotation `(roll, pitch, yaw)`: `0, 10, 290 deg`
- Max resolution: `640 x 480`
- FOV: `130 deg`
- ROS topic: `/sensors/camera/right/compressed`
- Default Ubuntu receive port: `1021`

Auxiliary camera:

- One extra camera may be used within the rule range.
- ROS topic: `/sensors/camera/aux/compressed`
- Default Ubuntu receive port: `1031`
- Disabled by default in launch files.

## Recommended Roles

Front camera:

- Lane offset estimation
- Stop-line detection
- Traffic-light color candidate detection
- Front object classification in a later model-based detector

Left and right cameras:

- Side-lane observation
- Intersection side approach monitoring
- Cut-in or side obstacle visual confirmation
- Blind-spot style context for later fusion with LiDAR

Auxiliary camera:

- Use only after the fixed three cameras are stable.
- Candidate roles: rear monitoring, wide front traffic-light view, or intersection context.

## MORAI UDP Port Rule Used In This Repo

The port values in this repo are Ubuntu receive ports. In MORAI camera UDP settings, set each camera destination IP to the Ubuntu algorithm PC IP and destination port to the matching value.

Default mapping:

| Camera | Ubuntu receive port | ROS topic |
| --- | ---: | --- |
| Front | `1001` | `/sensors/camera/front/compressed` |
| Left | `1011` | `/sensors/camera/left/compressed` |
| Right | `1021` | `/sensors/camera/right/compressed` |
| Aux | `1031` | `/sensors/camera/aux/compressed` |
