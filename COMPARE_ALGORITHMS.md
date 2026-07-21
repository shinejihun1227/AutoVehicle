# MORAI Controller Comparison Guide

This guide compares the four current ROS workspaces.

Use `EXPERIMENT_LOG.md` for human-readable notes and `experiment_results_template.csv` for spreadsheet-style comparison.

## Common Setup

Use the same recorded path for every algorithm:

```bash
export WAYPOINT_FILE=$HOME/morai_recorded_paths/01.csv
export TEST_SPEED=4.0
```

Before every test, confirm these topics are alive:

```bash
rostopic echo /localization/ego_pose
rostopic echo /localization/ego_twist
rostopic echo /control/ctrl_cmd
```

Keep the MORAI start pose, route, weather, traffic, and sensor settings identical. Do not compare algorithms using different CSV paths.

## Workspaces

| Workspace | Algorithm | Main package | Drive launch |
| --- | --- | --- | --- |
| `ros_ws` | Pure Pursuit baseline | `yeoms_pure_pursuit` | `yeoms_bringup drive_pure_pursuit.launch` |
| `ros_ws2` | Adaptive Pure Pursuit | `yeoms2_adaptive_pure_pursuit` | `yeoms2_bringup drive_adaptive_pure_pursuit.launch` |
| `ros_ws3` | Stanley | `yeoms3_stanley` | `yeoms3_bringup drive_stanley.launch` |
| `ros_ws4` | Stanley + Pure Pursuit Hybrid | `yeoms4_hybrid_controller` | `yeoms4_bringup drive_hybrid.launch` |

## What To Compare

Straight road:

- Check whether steering stays near zero.
- If steering oscillates on a straight CSV, first inspect localization yaw and CSV point spacing.
- Useful metric: peak absolute steering and repeated left-right sign changes.

Initial launch:

- Check whether the vehicle immediately turns in the wrong direction.
- If this happens in every algorithm, suspect coordinate-frame mismatch, yaw convention, steering sign, or route direction before tuning controller gains.
- For the competition TXT path with GPS-origin localization, keep `align_path_to_ego_start:=true` so the path is translated into the same local frame as `/localization/ego_pose`.
- Keep `rotate_path_to_ego_yaw:=false` unless the initial localization yaw is verified. GPS yaw is often unreliable before the vehicle moves.
- If this happens only in Stanley or Hybrid, reduce Stanley gain or Stanley blend weight.

Left and right turns:

- Check entry timing, maximum cross-track error, and whether the car cuts inside or goes wide.
- If the vehicle cuts inside, increase lookahead or reduce steering gain.
- If the vehicle turns late and goes wide, reduce lookahead or increase correction gain carefully.

End of path:

- Confirm the controller stops at the final waypoint.
- If it keeps driving, inspect `finish_radius_m` and whether the target index reaches the end.

## Tuning Files

Pure Pursuit baseline:

- File: `ros_ws/src/yeoms_pure_pursuit/config/pure_pursuit.yaml`
- Controller code: `ros_ws/src/yeoms_pure_pursuit/scripts/pure_pursuit_controller.py`
- Increase `lookahead_m` or `min_lookahead_m` to reduce oscillation.
- Decrease `lookahead_m` to turn earlier.
- Decrease `max_steer_rad` or `max_steer_rate_radps` if steering is too aggressive.

Adaptive Pure Pursuit:

- File: `ros_ws2/src/yeoms2_adaptive_pure_pursuit/config/adaptive_pure_pursuit.yaml`
- Controller code: `ros_ws2/src/yeoms2_adaptive_pure_pursuit/scripts/pure_pursuit_controller.py`
- Tune `min_lookahead_m`, `max_lookahead_m`, and `lookahead_speed_gain` first.
- Tune `curvature_threshold_radpm`, `heading_error_threshold_rad`, and `cross_track_threshold_m` if curve behavior changes too late or too early.
- Tune `crosstrack_steer_gain` only after yaw and path spacing are stable.

Stanley:

- File: `ros_ws3/src/yeoms3_stanley/config/stanley.yaml`
- Controller code: `ros_ws3/src/yeoms3_stanley/scripts/stanley_controller.py`
- Reduce `stanley_gain` if cross-track correction causes left-right shaking.
- Increase `softening_gain` if low-speed steering is too aggressive.
- Reduce `heading_error_gain` if the vehicle snaps toward path yaw.
- Increase `path_yaw_preview_m` if path yaw is noisy.

Hybrid:

- File: `ros_ws4/src/yeoms4_hybrid_controller/config/hybrid_controller.yaml`
- Controller code: `ros_ws4/src/yeoms4_hybrid_controller/scripts/hybrid_controller.py`
- Reduce `hybrid_stanley_weight` if the car oscillates like Stanley.
- Increase `hybrid_stanley_weight` if Pure Pursuit is too slow to recover from cross-track error.
- Increase `min_lookahead_m` and `max_lookahead_m` if both raw steering topics oscillate.
- Lower `max_stanley_weight` if adaptive blending becomes too aggressive in curves.

## Topic Checklist

Pure Pursuit:

```bash
rostopic echo /control/pure_pursuit_steering_rad
rostopic echo /control/pure_pursuit_raw_steering_rad
rostopic echo /control/curve_speed_limit_mps
```

Adaptive Pure Pursuit:

```bash
rostopic echo /control/adaptive_pp_steering_rad
rostopic echo /control/adaptive_pp_raw_steering_rad
rostopic echo /control/adaptive_pp_cross_track_error_m
rostopic echo /control/adaptive_lookahead_distance_m
```

Stanley:

```bash
rostopic echo /control/stanley_steering_rad
rostopic echo /control/stanley_raw_steering_rad
rostopic echo /control/stanley_heading_error_rad
rostopic echo /control/stanley_cross_track_error_m
```

Hybrid:

```bash
rostopic echo /control/hybrid_steering_rad
rostopic echo /control/hybrid_pure_pursuit_raw_steering_rad
rostopic echo /control/hybrid_stanley_raw_steering_rad
rostopic echo /control/hybrid_stanley_weight
rostopic echo /control/hybrid_cross_track_error_m
```

## Practical Tuning Order

1. Verify `/localization/ego_pose` and `/localization/ego_twist`.
2. Record a clean `01.csv` with dense enough points and stable speed.
3. Run `ros_ws` Pure Pursuit at `3.0 m/s`.
4. Increase to `4.0 m/s` only after straight-road steering is stable.
5. Compare `ros_ws2`, `ros_ws3`, and `ros_ws4` on the exact same path.
6. Tune only one parameter at a time and save the before/after topic values.

If all algorithms oscillate, do not tune the controller first. Check yaw, coordinate convention, steering sign, CSV spacing, and UDP localization stability.
