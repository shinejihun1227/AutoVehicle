# Ubuntu Commands For `ros_ws4`

Update and copy the hybrid packages:

```bash
cd ~/AutoVehicle
git pull

cd ~/morai_stanley_ws

rm -rf src/yeoms4_base_bridge
rm -rf src/yeoms4_bringup
rm -rf src/yeoms4_path_recorder
rm -rf src/yeoms4_hybrid_controller

cp -r ~/AutoVehicle/ros_ws4/src/yeoms4_base_bridge src/
cp -r ~/AutoVehicle/ros_ws4/src/yeoms4_bringup src/
cp -r ~/AutoVehicle/ros_ws4/src/yeoms4_path_recorder src/
cp -r ~/AutoVehicle/ros_ws4/src/yeoms4_hybrid_controller src/

chmod +x src/yeoms4_base_bridge/scripts/*.py
chmod +x src/yeoms4_path_recorder/scripts/*.py
chmod +x src/yeoms4_hybrid_controller/scripts/*.py

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
```

Record a path:

```bash
roslaunch yeoms4_bringup record_gps_path.launch \
  output_file:=$HOME/morai_recorded_paths/01.csv
```

Drive with the hybrid controller:

```bash
roslaunch yeoms4_bringup drive_hybrid.launch \
  waypoint_file:=$HOME/morai_recorded_paths/01.csv \
  target_speed_mps:=4.0 \
  max_speed_mps:=4.0 \
  hybrid_stanley_weight:=0.45 \
  stanley_gain:=0.35 \
  softening_gain:=2.0 \
  min_curve_speed_mps:=2.5 \
  min_lookahead_m:=3.0 \
  max_lookahead_m:=12.0
```

Useful comparison topics:

```bash
rostopic echo /control/hybrid_steering_rad
rostopic echo /control/hybrid_raw_steering_rad
rostopic echo /control/hybrid_pure_pursuit_raw_steering_rad
rostopic echo /control/hybrid_stanley_raw_steering_rad
rostopic echo /control/hybrid_stanley_weight
rostopic echo /control/hybrid_heading_error_rad
rostopic echo /control/hybrid_cross_track_error_m
rostopic echo /control/hybrid_speed_limit_mps
```
