# Yeom's Control

This package contains the first Stanley control baseline.

## Package Structure

```text
yeoms_control
├── config/
│   └── stanley.yaml
├── launch/
│   └── stanley_test.launch
├── scripts/
│   └── stanley_controller.py
└── waypoints/
    ├── test_straight.csv
    └── test_curve.csv
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

This is useful for early algorithm tests. After MORAI CtrlCmd message fields are confirmed, run with:

```bash
roslaunch yeoms_control stanley_test.launch command_type:=morai
```

## Ubuntu Setup Note

On the Ubuntu ROS PC, make the script executable if needed:

```bash
chmod +x ~/morai_competition_ws/src/yeoms_control/scripts/stanley_controller.py
```

Then build:

```bash
cd ~/morai_competition_ws
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
