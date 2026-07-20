# MORAI Algorithm Experiment Log

Use the same route, same MORAI start pose, same sensor settings, and same speed for one comparison set.

## Fixed Test Condition

- Date:
- MORAI map:
- Vehicle:
- Waypoint file: `$HOME/morai_recorded_paths/01.csv`
- Target speed: `4.0 m/s`
- Max speed: `4.0 m/s`
- Start pose:
- Route description:
- Sensor/UDP setting changes:

## Result Table

| Algorithm | Workspace | Success | Straight oscillation | Curve entry | Curve exit | Initial wrong turn | End stop | Main problem | Next tuning |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Pure Pursuit baseline | `ros_ws` |  |  |  |  |  |  |  |  |
| Adaptive Pure Pursuit | `ros_ws2` |  |  |  |  |  |  |  |  |
| Stanley | `ros_ws3` |  |  |  |  |  |  |  |  |
| Stanley + Pure Pursuit Hybrid | `ros_ws4` |  |  |  |  |  |  |  |  |

## Numeric Checklist

Record approximate values from `rostopic echo` or visual observation.

| Algorithm | Max steering rad | Max cross-track error m | Avg speed km/h | Worst section | Notes |
| --- | ---: | ---: | ---: | --- | --- |
| Pure Pursuit baseline |  |  |  |  |  |
| Adaptive Pure Pursuit |  |  |  |  |  |
| Stanley |  |  |  |  |  |
| Hybrid |  |  |  |  |  |

## Topic Commands

Pure Pursuit baseline:

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

## Interpretation Rules

- If all algorithms oscillate on a straight path, inspect localization yaw, CSV spacing, steering sign, and coordinate convention first.
- If only Stanley oscillates, reduce `stanley_gain`, increase `softening_gain`, or increase `path_yaw_preview_m`.
- If only Pure Pursuit turns late, reduce lookahead.
- If Pure Pursuit cuts inside, increase lookahead.
- If Hybrid oscillates like Stanley, reduce `hybrid_stanley_weight` or `max_stanley_weight`.
- If Hybrid behaves too slowly like Pure Pursuit, increase `hybrid_stanley_weight` carefully.

## Next Tuning Decision

After one full comparison, choose only one change:

- Change target speed.
- Change lookahead.
- Change Stanley gain.
- Change steering filter/rate limit.
- Re-record CSV path.

Do not change multiple controller parameters in the same test run.
