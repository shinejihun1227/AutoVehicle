# ROS Workspace 3: Stanley

`ros_ws3` is the third comparison workspace. It uses the same GPS/UDP bridge and path recorder pattern as `ros_ws`, but replaces the lateral controller with Stanley control.

Main packages:

- `yeoms3_base_bridge`: GPS UDP localization and MORAI CtrlCmd UDP sender.
- `yeoms3_path_recorder`: CSV path recorder.
- `yeoms3_stanley`: Stanley controller.
- `yeoms3_bringup`: launch files for recording and driving.

Copy packages under `ros_ws3/src` into your Ubuntu catkin workspace:

```bash
cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_base_bridge ~/morai_stanley_ws/src/
cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_path_recorder ~/morai_stanley_ws/src/
cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_stanley ~/morai_stanley_ws/src/
cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_bringup ~/morai_stanley_ws/src/
```

See `UBUNTU_COMMANDS.md` for the full update and run commands.
