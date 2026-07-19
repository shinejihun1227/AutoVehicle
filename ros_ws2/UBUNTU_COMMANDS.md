# Ubuntu Commands For `ros_ws2`

Update and copy the adaptive Pure Pursuit packages:

```bash
cd ~/AutoVehicle
git pull

cd ~/morai_stanley_ws

rm -rf src/yeoms2_base_bridge
rm -rf src/yeoms2_bringup
rm -rf src/yeoms2_path_recorder
rm -rf src/yeoms2_adaptive_pure_pursuit

cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_base_bridge src/
cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_bringup src/
cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_path_recorder src/
cp -r ~/AutoVehicle/ros_ws2/src/yeoms2_adaptive_pure_pursuit src/

chmod +x src/yeoms2_base_bridge/scripts/*.py
chmod +x src/yeoms2_path_recorder/scripts/*.py
chmod +x src/yeoms2_adaptive_pure_pursuit/scripts/*.py

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
```

Record a path:

```bash
roslaunch yeoms2_bringup record_gps_path.launch \
  output_file:=$HOME/morai_recorded_paths/01.csv
```

Drive with Adaptive Pure Pursuit:

```bash
roslaunch yeoms2_bringup drive_adaptive_pure_pursuit.launch \
  waypoint_file:=$HOME/morai_recorded_paths/01.csv \
  target_speed_mps:=6.0 \
  max_speed_mps:=6.0 \
  min_curve_speed_mps:=2.5 \
  min_lookahead_m:=3.0 \
  max_lookahead_m:=14.0
```

Useful comparison topics:

```bash
rostopic echo /control/adaptive_pp_steering_rad
rostopic echo /control/adaptive_pp_cross_track_error_m
rostopic echo /control/adaptive_path_curvature_radpm
rostopic echo /control/adaptive_curve_ratio
rostopic echo /control/adaptive_speed_limit_mps
```
