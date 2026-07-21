# Competition Global Path Usage

The confirmed competition global path file uses whitespace-separated `x y z` rows:

```text
-131.68979755061446 -428.3310229377821 28.543960281954277
-131.44967144399345 -427.8924578823263 28.544341139013788
```

The path-following controllers in `ros_ws`, `ros_ws2`, `ros_ws3`, and `ros_ws4` now support both formats:

- recorded CSV: `x,y,z,yaw,speed,target_speed`
- competition TXT: `x y z`

For TXT paths, `z` is loaded only as map elevation reference and the current lateral controllers use `x, y` for tracking. Speed still comes from the launch argument `target_speed_mps`.

## Start Pose Policy

All path-following controllers now start from waypoint index `0` by default:

```text
start_from_first_waypoint:=true
```

This prevents the controller from starting at an arbitrary nearest point on a closed or loop-like course.

The controllers also align the loaded path to the first received ego pose by default:

```text
align_path_to_ego_start:=true
```

This is important for the confirmed competition TXT path. The TXT file stores MORAI/K-City map-like `x y z` coordinates, while the current GPS localizer publishes an ENU frame using the first GPS packet as `(0, 0)`. Without alignment, the controller can see the path and the vehicle in different coordinate frames, which often appears as an immediate wrong-side turn at launch.

With `align_path_to_ego_start:=true`, the controller translates and rotates the entire path once at startup:

- TXT or CSV first waypoint becomes the current ego position.
- Path first-segment heading becomes the current ego yaw.
- After that one-time transform, the controller tracks the path normally in the same local frame as `/localization/ego_pose`.

If you intentionally set MORAI ego pose to the absolute map path coordinates and use a localization source that already publishes the same map frame, turn this off:

```text
align_path_to_ego_start:=false
```

Important limitation:

```text
ROS CtrlCmd cannot teleport or rotate the MORAI ego vehicle.
```

The controller can force the tracking target to begin at the first waypoint and can transform the path into the ego local frame, but it cannot physically move the MORAI ego vehicle.

When each controller starts, it prints the required start pose:

```text
Required MORAI start pose: x=... y=... yaw=... rad ... deg
```

If you want the controller to wait until the vehicle is near that start pose, run with:

```text
require_start_pose:=true
```

Default tolerance:

```text
start_position_tolerance_m:=3.0
start_yaw_tolerance_rad:=0.7
```

## Ubuntu File Placement

Copy the competition path into the Ubuntu workspace path directory:

```bash
mkdir -p ~/morai_recorded_paths
cp ~/Downloads/2026_molit_comp_global_path.txt ~/morai_recorded_paths/
```

If the file is on Windows, move it into Ubuntu using your shared folder, drag-and-drop, or SCP, then place it here:

```bash
~/morai_recorded_paths/2026_molit_comp_global_path.txt
```

## Recommended First Test

Start with low speed. The path has thousands of points and should be tested with a conservative controller first:

```bash
roslaunch yeoms_bringup drive_pure_pursuit.launch \
  waypoint_file:=$HOME/morai_recorded_paths/2026_molit_comp_global_path.txt \
  target_speed_mps:=3.0 \
  max_speed_mps:=3.0 \
  start_from_first_waypoint:=true \
  align_path_to_ego_start:=true
```

Then compare the other controllers using the same file.

Adaptive Pure Pursuit:

```bash
roslaunch yeoms2_bringup drive_adaptive_pure_pursuit.launch \
  waypoint_file:=$HOME/morai_recorded_paths/2026_molit_comp_global_path.txt \
  target_speed_mps:=3.0 \
  max_speed_mps:=3.0 \
  min_curve_speed_mps:=2.0 \
  min_lookahead_m:=3.0 \
  max_lookahead_m:=14.0 \
  start_from_first_waypoint:=true \
  align_path_to_ego_start:=true
```

Stanley:

```bash
roslaunch yeoms3_bringup drive_stanley.launch \
  waypoint_file:=$HOME/morai_recorded_paths/2026_molit_comp_global_path.txt \
  target_speed_mps:=3.0 \
  max_speed_mps:=3.0 \
  stanley_gain:=0.30 \
  softening_gain:=2.5 \
  min_curve_speed_mps:=2.0 \
  start_from_first_waypoint:=true \
  align_path_to_ego_start:=true
```

Hybrid:

```bash
roslaunch yeoms4_bringup drive_hybrid.launch \
  waypoint_file:=$HOME/morai_recorded_paths/2026_molit_comp_global_path.txt \
  target_speed_mps:=3.0 \
  max_speed_mps:=3.0 \
  hybrid_stanley_weight:=0.40 \
  stanley_gain:=0.30 \
  softening_gain:=2.5 \
  min_curve_speed_mps:=2.0 \
  min_lookahead_m:=3.0 \
  max_lookahead_m:=12.0 \
  start_from_first_waypoint:=true \
  align_path_to_ego_start:=true
```

## What Changed In Code

The following controllers now load `.txt` global paths directly:

- `ros_ws/src/yeoms_pure_pursuit/scripts/pure_pursuit_controller.py`
- `ros_ws2/src/yeoms2_adaptive_pure_pursuit/scripts/pure_pursuit_controller.py`
- `ros_ws3/src/yeoms3_stanley/scripts/stanley_controller.py`
- `ros_ws4/src/yeoms4_hybrid_controller/scripts/hybrid_controller.py`

They also perform a full-path nearest search on the first localization update. After the initial match, each controller returns to local-window search for efficiency and smoother index progression.

## Checks During First Run

Check that the target index starts near the vehicle's actual spawn location:

```bash
rostopic echo /control/target_index
rostopic echo /control/adaptive_target_index
rostopic echo /control/stanley_target_index
rostopic echo /control/hybrid_target_index
```

Use the topic that matches the controller you are running.

If the vehicle immediately drives backward, turns toward the wrong side, or targets the end of the path, check:

- whether `align_path_to_ego_start:=true` is enabled when using the competition TXT path with GPS-origin localization,
- whether the path file is copied correctly,
- whether GPS localization uses the same map coordinate convention,
- whether `invert_steering` is still correct in the UDP CtrlCmd sender config.
