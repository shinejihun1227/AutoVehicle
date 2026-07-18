# AutoVehicle Fresh Baseline

This repository is reset to a minimal MORAI + ROS Noetic baseline.

The stack is intentionally simple:

1. `yeoms_base_bridge`: MORAI UDP GPS to ROS localization, and ROS Twist to MORAI UDP CtrlCmd.
2. `yeoms_path_recorder`: record `/localization/ego_pose` to CSV.
3. `yeoms_pure_pursuit`: follow a recorded CSV path with Pure Pursuit only.
4. `yeoms_bringup`: launch files that combine the packages.

Coordinate convention:

- `map.x`: East
- `map.y`: North
- `map.z`: Up
- `yaw = atan2(dy, dx)`
- `yaw = 0`: East, positive yaw is counter-clockwise
- positive steering means left turn

Start with path recording, then Pure Pursuit. Add Stanley only after this baseline is stable.
