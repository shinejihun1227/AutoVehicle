# ROS Workspace 2: Adaptive Pure Pursuit

`ros_ws2` is the second comparison workspace. It keeps the same GPS/UDP bridge idea as `ros_ws`, but replaces the baseline controller with an adaptive Pure Pursuit controller.

Main packages:

- `yeoms2_base_bridge`: GPS UDP localization and MORAI CtrlCmd UDP sender.
- `yeoms2_path_recorder`: CSV path recorder.
- `yeoms2_adaptive_pure_pursuit`: adaptive Pure Pursuit controller.
- `yeoms2_bringup`: launch files for recording and driving.

Copy packages under `ros_ws2/src` into your Ubuntu catkin workspace:

```bash
cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_base_bridge ~/morai_stanley_ws/src/
cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_path_recorder ~/morai_stanley_ws/src/
cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_adaptive_pure_pursuit ~/morai_stanley_ws/src/
cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_bringup ~/morai_stanley_ws/src/
```

See `UBUNTU_COMMANDS.md` for the full update and run commands.
