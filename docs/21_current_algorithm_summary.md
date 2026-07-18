# Current Autonomous Driving Structure

## Goal

This project currently implements a first working MORAI K-City autonomous driving loop:

1. Receive MORAI sensor data through UDP.
2. Convert raw UDP GPS/IMU data into ROS topics.
3. Record the driven route as a CSV waypoint path.
4. Replay the recorded CSV path with Stanley lateral control.
5. Convert ROS control commands back into MORAI Ego Ctrl Cmd UDP packets.

The present target is not final competition performance yet. It is a stable baseline that lets us verify coordinate systems, path recording, path tracking, control direction, speed limits, and logging.

## Unified Coordinate System

All driving code is unified to a ROS-style local map frame:

| Item | Convention |
| --- | --- |
| `map.x` | East direction from the GPS origin |
| `map.y` | North direction from the GPS origin |
| `map.z` | Up/altitude offset from the GPS origin |
| `yaw = 0` | Vehicle heading toward +x/East |
| Positive yaw | Counter-clockwise rotation |
| `base_link.x` | Vehicle forward direction |
| `base_link.y` | Vehicle left direction |
| Positive steering | Left turn |

GPS latitude/longitude is converted into local East-North displacement from a fixed origin. The same local x/y frame is used for both CSV waypoint recording and Stanley path tracking, so the controller and recorded path use one coordinate system.

## Sensor And Topic Flow

MORAI sends sensor data to Ubuntu over UDP:

| MORAI data | UDP destination | ROS output |
| --- | ---: | --- |
| GPS | `3001` | `/localization/ego_pose`, `/localization/ego_twist`, `/udp_bridge/gps_debug` |
| IMU | `4001` | `/udp_bridge/imu`, `/udp_bridge/imu_debug` |
| CollisionData | `907` | monitored as raw UDP for now |
| Competition Vehicle Status | `909` | monitored as raw UDP for now |
| Ego Ctrl Cmd | MORAI receives on `9093` | generated from `/control/ctrl_cmd` |

For the current driving baseline, GPS is the main localization source. IMU is recorded together with the path so that later work can improve yaw filtering, sensor fusion, and vehicle-state estimation.

## Driving Algorithm

The current path-following algorithm is Stanley control.

The controller receives:

- Current vehicle pose from `/localization/ego_pose`.
- Current speed from `/localization/ego_twist`.
- Recorded path from a CSV file such as `~/morai_recorded_paths/kcity_test_01.csv`.

The controller computes:

- Nearest path target point.
- Path heading from a forward lookahead point.
- Heading error between vehicle yaw and path yaw.
- Signed cross-track error in the unified ENU map frame.
- Steering command:

```text
steering = heading_error_gain * heading_error
         + crosstrack_error_gain * atan2(stanley_gain * cross_track_error,
                                         vehicle_speed + softening_gain)
```

The output is published as `/control/ctrl_cmd`:

- `linear.x`: target speed in m/s.
- `angular.z`: steering angle in radians.

Then `udp_ctrl_cmd_sender.py` converts this ROS command into the MORAI Ego Ctrl Cmd UDP packet.

## Current Speed Setup

The previous baseline was intentionally slow:

- Stanley target speed: `1.0 m/s`.
- UDP sender max speed: `1.0 m/s`.
- MORAI debug velocity: about `3.6 km/h`.

The current baseline is slightly faster:

- Stanley replay speed override: `1.5 m/s`.
- Stanley max target speed: `1.5 m/s`.
- UDP sender max speed: `1.5 m/s`.
- Expected MORAI debug velocity: about `5.4 km/h`.

If the vehicle oscillates more at this speed, lower `target_speed_override_mps` first before changing the coordinate system again.

## Current Architecture

```text
MORAI SIM on Windows
  |
  | UDP GPS / IMU / status / camera / lidar
  v
Ubuntu ROS Noetic
  |
  +-- yeoms_udp_bridge
  |     +-- UDP GPS -> ROS Pose/Twist
  |     +-- UDP IMU -> ROS Imu
  |     +-- ROS Twist cmd -> MORAI UDP Ego Ctrl Cmd
  |
  +-- yeoms_control
  |     +-- waypoint_recorder.py -> CSV path
  |     +-- stanley_controller.py -> /control/ctrl_cmd
  |
  +-- morai_competition_bringup
        +-- launch files that connect localization, control, and UDP actuation
```

## Development Process So Far

1. Confirmed Windows-MORAI and Ubuntu-ROS network connectivity.
2. Verified raw UDP packets with monitor and `tcpdump`.
3. Decided not to depend on uncertain Competition Vehicle Status offsets yet.
4. Parsed GPS and IMU UDP directly.
5. Converted GPS into one local `map` coordinate frame.
6. Built a CSV route recorder.
7. Built Stanley path tracking against the recorded CSV.
8. Fixed steering sign and cross-track error convention for ENU coordinates.
9. Added waypoint spacing, smoothing, lookahead yaw, steering filtering, and steering rate limiting to reduce shaking.
10. Increased the test replay speed from `1.0 m/s` to `1.5 m/s`.

## Next Improvements

The next tuning work should focus on oscillation reduction:

- Increase `path_yaw_lookahead_m` if steering reacts too sharply.
- Decrease `stanley_gain` or `crosstrack_error_gain` if it hunts left-right around the path.
- Increase `softening_gain` if low-speed steering is too aggressive.
- Record smoother waypoints by driving steadily and avoiding abrupt manual steering.
- Add IMU/GPS fusion later so yaw is less dependent on GPS displacement alone.
