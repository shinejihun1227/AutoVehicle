# Yeom's Control

This package contains the first Stanley control baseline.

## Package Structure

```text
yeoms_control
|-- config/
|   `-- stanley.yaml
|-- launch/
|   `-- stanley_test.launch
|-- scripts/
|   `-- stanley_controller.py
`-- waypoints/
    |-- test_straight.csv
    `-- test_curve.csv
```

## Inputs

Default input topics:

```text
/localization/ego_pose   geometry_msgs/PoseStamped
/localization/ego_twist  geometry_msgs/TwistStamped
```

Alternative input:

```text
/localization/ego_odom   nav_msgs/Odometry
```

Set `use_odometry: true` in `config/stanley.yaml` to use odometry.

## Coordinate Convention

The controller uses the project-wide ROS ENU convention:

```text
map.x: East
map.y: North
map.z: Up
yaw=0: +x/East
yaw positive: counter-clockwise
steering positive: left turn
```

See `docs/20_coordinate_convention.md` for the full convention and Stanley
cross-track error sign rule.

## Output

Default output:

```text
/control/ctrl_cmd        geometry_msgs/Twist
```

This is useful for early algorithm tests. After MORAI CtrlCmd message fields are
confirmed, run with:

```bash
roslaunch yeoms_control stanley_test.launch command_type:=morai
```

## Ubuntu Setup Note

On the Ubuntu ROS PC, make the script executable if needed:

```bash
chmod +x ~/morai_stanley_ws/src/yeoms_control/scripts/stanley_controller.py
```

Then build:

```bash
cd ~/morai_stanley_ws
catkin_make
source devel/setup.bash
```

## First Test

```bash
roslaunch yeoms_control stanley_test.launch
```

Check:

```bash
rostopic echo /control/ctrl_cmd
rostopic echo /control/stanley_target_index
rostopic echo /control/stanley_cross_track_error
rostopic echo /control/stanley_steering_rad
rostopic echo /control/stanley_heading_error_rad
rostopic echo /control/stanley_crosstrack_term_rad
```

## GPS + Stanley Dry Run

Use this launch after GPS localization publishes `/localization/ego_pose` and
`/localization/ego_twist` correctly:

```bash
roslaunch morai_competition_bringup gps_stanley_dry_run.launch
```

This computes `/control/ctrl_cmd` only. It does not send UDP control packets to
MORAI yet.

## GPS + Stanley UDP Drive

Use this only after the dry run publishes stable `/control/ctrl_cmd` values:

```bash
roslaunch morai_competition_bringup gps_stanley_udp_drive.launch
```

The default waypoint file is `test_straight_slow.csv`, which limits the first
test to `1.0 m/s`.

Check:

```bash
rostopic echo /control/ctrl_cmd
rostopic echo /udp_bridge/ctrl_cmd_debug
```

## First Tuning Notes

If the vehicle oscillates left and right on a straight path, reduce steering
aggressiveness first:

```yaml
stanley_gain: 0.35
softening_gain: 2.0
crosstrack_error_gain: 0.75
steering_filter_alpha: 0.35
max_steer_rate_radps: 0.6
path_yaw_lookahead_m: 3.0
waypoint_smoothing_window: 5
allow_target_backtrack: false
```

Recorded GPS paths can be jagged because each waypoint contains GPS noise.
`waypoint_smoothing_window` smooths the loaded path, and
`path_yaw_lookahead_m` calculates path heading from a point several meters ahead
instead of the immediately next point.

If the vehicle consistently turns the wrong way, change
`udp_bridge.yaml -> ctrl_sender.invert_steering` and test again. If the first
path is not aligned with the vehicle's starting direction, regenerate waypoints
or start the vehicle facing the test path direction.

## Record A Driven Path

Drive the vehicle manually once and record the GPS localization path as a
Stanley-compatible CSV:

```bash
roslaunch morai_competition_bringup gps_waypoint_record.launch
```

By default, the file is saved under:

```text
~/morai_recorded_paths/path_YYYYMMDD_HHMMSS.csv
```

You can also choose the output path:

```bash
roslaunch morai_competition_bringup gps_waypoint_record.launch \
  output_file:=$HOME/morai_recorded_paths/kcity_test_01.csv \
  target_speed_mps:=1.0 \
  min_distance_m:=0.2 \
  min_time_s:=0.1
```

After recording, follow that path with Stanley and UDP control:

```bash
roslaunch morai_competition_bringup gps_stanley_udp_drive.launch \
  waypoint_file:=$HOME/morai_recorded_paths/kcity_test_01.csv
```

Recorded CSV format:

```csv
x,y,z,target_speed,lat,lon,alt,origin_lat,origin_lon,origin_alt,imu_qx,imu_qy,imu_qz,imu_qw,imu_angular_velocity_x,imu_angular_velocity_y,imu_angular_velocity_z,imu_linear_acceleration_x,imu_linear_acceleration_y,imu_linear_acceleration_z
0.000000,0.000000,0.000000,1.000,37.24097500,126.77435000,0.000,37.24097500,126.77435000,0.000,0.00000000,0.00000000,0.00000000,1.00000000,0.00000000,0.00000000,0.00000000,0.00000000,0.00000000,9.81000000
```

The `origin_lat/origin_lon` columns store the GPS origin used during recording.
`z/alt` are populated when MORAI sends GPS altitude data. IMU columns are filled
from `/udp_bridge/imu` when IMU UDP packets are available.

When replaying the path, pass the same origin to the localization node:

```bash
rosrun yeoms_control waypoint_origin.py $HOME/morai_recorded_paths/kcity_test_01.csv
```

Example output:

```text
origin_lat:=37.24097500 origin_lon:=126.77435000 origin_alt:=0.000
```

Then replay:

```bash
roslaunch morai_competition_bringup gps_stanley_udp_drive.launch \
  waypoint_file:=$HOME/morai_recorded_paths/kcity_test_01.csv \
  origin_lat:=37.24097500 \
  origin_lon:=126.77435000 \
  origin_alt:=0.000
```

Older CSV files without `lat/lon` should be recorded again, or replayed from the
exact same vehicle start position.
