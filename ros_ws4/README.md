# ROS Workspace 4: Stanley + Pure Pursuit Hybrid

`ros_ws4` is the fourth comparison workspace. It uses the same MORAI UDP bridge and GPS path recorder pattern as the earlier workspaces, but the lateral controller computes both Pure Pursuit and Stanley steering and blends them into one command.

Main packages:

- `yeoms4_base_bridge`: GPS UDP localization and MORAI CtrlCmd UDP sender.
- `yeoms4_path_recorder`: CSV path recorder.
- `yeoms4_hybrid_controller`: Stanley + Pure Pursuit hybrid controller.
- `yeoms4_bringup`: launch files for recording and driving.

Hybrid steering:

```text
final_steering = (1 - stanley_weight) * pure_pursuit_steering
               + stanley_weight * stanley_steering
```

The default `stanley_weight` is `0.45`. If adaptive blending is enabled, the controller increases the Stanley weight when heading error, cross-track error, or local path curvature becomes large.

See `UBUNTU_COMMANDS.md` for the full update and run commands.
