# AutoVehicle Fresh Baseline

This repository contains comparison ROS workspaces for MORAI + ROS Noetic autonomous driving experiments.

## Workspaces

- `ros_ws`: Pure Pursuit baseline.
- `ros_ws2`: Adaptive Pure Pursuit comparison version.
- `ros_ws3`: Stanley comparison version.
- `ros_ws4`: Stanley + Pure Pursuit hybrid comparison version.

## `ros_ws` Baseline

The baseline stack is intentionally simple:

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

Start with path recording, then Pure Pursuit. Add advanced controllers only after this baseline is stable.

## `ros_ws2` Adaptive Pure Pursuit

`ros_ws2` extends the baseline with:

- speed-based dynamic lookahead,
- path-curvature based lookahead reduction,
- cross-track-error steering compensation,
- curve-aware speed limiting,
- additional diagnostic topics for comparison.

Use `ros_ws2/UBUNTU_COMMANDS.md` for the Ubuntu copy/build/run commands.

## `ros_ws3` Stanley

`ros_ws3` adds a Stanley controller workspace for comparison with Pure Pursuit methods.

Stanley uses:

- heading error between vehicle yaw and path yaw,
- cross-track error from the front axle to the nearest path segment,
- speed-dependent cross-track correction,
- optional curve-aware speed limiting,
- diagnostic topics for heading, cross-track, and steering terms.

Use `ros_ws3/UBUNTU_COMMANDS.md` for the Ubuntu copy/build/run commands.

## `ros_ws4` Stanley + Pure Pursuit Hybrid

`ros_ws4` computes Pure Pursuit steering and Stanley steering in the same controller, then blends them into one final command.

Use it to compare:

- Pure Pursuit target-point steering,
- Stanley heading and cross-track correction,
- blended steering behavior,
- the effect of changing `hybrid_stanley_weight`.

Use `ros_ws4/UBUNTU_COMMANDS.md` for the Ubuntu copy/build/run commands.
