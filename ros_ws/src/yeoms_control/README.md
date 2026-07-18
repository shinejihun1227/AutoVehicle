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
```

If the vehicle consistently turns the wrong way, change
`udp_bridge.yaml -> ctrl_sender.invert_steering` and test again. If the first
path is not aligned with the vehicle's starting direction, regenerate waypoints
or start the vehicle facing the test path direction.
