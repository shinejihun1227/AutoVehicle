# Ubuntu Commands For `ros_ws3`

Update and copy the Stanley packages:

```bash
cd ~/AutoVehicle
git pull

cd ~/morai_stanley_ws

rm -rf src/yeoms3_base_bridge
rm -rf src/yeoms3_bringup
rm -rf src/yeoms3_path_recorder
rm -rf src/yeoms3_stanley

cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_base_bridge src/
cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_bringup src/
cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_path_recorder src/
cp -r ~/AutoVehicle/ros_ws3/src/yeoms3_stanley src/

chmod +x src/yeoms3_base_bridge/scripts/*.py
chmod +x src/yeoms3_path_recorder/scripts/*.py
chmod +x src/yeoms3_stanley/scripts/*.py

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
```

Record a path:

```bash
roslaunch yeoms3_bringup record_gps_path.launch \
  output_file:=$HOME/morai_recorded_paths/01.csv
```

Drive with Stanley:

```bash
roslaunch yeoms3_bringup drive_stanley.launch \
  waypoint_file:=$HOME/morai_recorded_paths/01.csv \
  target_speed_mps:=4.0 \
  max_speed_mps:=4.0 \
  stanley_gain:=0.35 \
  softening_gain:=2.0 \
  min_curve_speed_mps:=2.5
```

Useful comparison topics:

```bash
rostopic echo /control/stanley_steering_rad
rostopic echo /control/stanley_raw_steering_rad
rostopic echo /control/stanley_heading_error_rad
rostopic echo /control/stanley_cross_track_error_m
rostopic echo /control/stanley_crosstrack_term_rad
rostopic echo /control/stanley_speed_limit_mps
```
