# Yeom's UDP Bridge

This package is the UDP front-end for MORAI.

## Confirmed Network Values

```text
Windows MORAI IP: 192.168.0.151
Ubuntu algorithm IP: 192.168.0.200

Camera UDP receive port: 1001
LiDAR UDP receive port: 2001
GPS UDP receive port: 3001
IMU UDP receive port: 4001
CollisionData UDP receive port: 907
Competition Vehicle Status UDP receive port: 909
Ego Ctrl Cmd MORAI receive port: 9093
Ego Ctrl Cmd local port: 9094

Note: MORAI's Host Port is the simulator-side source port. The UDP bridge must bind
to the Destination Port shown in packet captures.
```

## First Check

Run this before writing parsers:

```bash
roslaunch yeoms_udp_bridge udp_monitor.launch
```

Then check:

```bash
rostopic echo /udp_bridge/packet_info
```

If packets arrive, the network and MORAI UDP settings are working.

## Recommended Localization Check

Use GPS first because the MORAI GPS UDP payload is an ASCII `$GPRMC` sentence, so
we do not need to guess binary byte offsets.

```bash
roslaunch yeoms_udp_bridge udp_gps_localization.launch
```

Then check:

```bash
rostopic echo /localization/ego_pose
rostopic echo /localization/ego_twist
rostopic echo /udp_bridge/gps_debug
```

The GPS localization node sets the first received GPS position as `(0, 0)`.
`x` is east and `y` is north. Speed and yaw are estimated from GPS position
changes by default because MORAI GPRMC speed/course fields may stay at zero in
some UDP configurations.

Noise handling is configured in `config/udp_bridge.yaml`:

```yaml
gps_adapter:
  min_position_delta_m: 0.35
  speed_filter_alpha: 0.25
  position_filter_alpha: 0.45
  stationary_speed_decay: 0.55
  min_yaw_update_speed_mps: 0.35
  max_yaw_jump_rad: 0.85
```

If the vehicle is stopped but `speed` remains high, increase
`min_position_delta_m` or decrease `speed_filter_alpha`. If yaw reacts too
slowly while driving, decrease `min_yaw_update_speed_mps` or increase
`max_yaw_jump_rad`.

## Vehicle Status Parser

`udp_status_to_localization.py` starts in `parser_mode: raw` because the exact competition packet byte layout must be confirmed from MORAI docs or example code.

After offsets are known, update `config/udp_bridge.yaml`:

```yaml
status_adapter:
  parser_mode: "offset"
  x_offset: 0
  y_offset: 8
  yaw_offset: 16
```

The node will then publish:

```text
/localization/ego_pose
/localization/ego_twist
```

These are the inputs used by `yeoms_control`.

## Ego Ctrl Cmd UDP Sender

`udp_ctrl_cmd_sender.py` converts `/control/ctrl_cmd` into the MORAI UDP
`Ego Ctrl Cmd` packet.

Current mapping:

```text
/control/ctrl_cmd.linear.x   target speed in m/s
/control/ctrl_cmd.angular.z  steering angle in rad
```

The sender converts speed to km/h, normalizes steering to `-1.0..1.0`, and sends
a 55-byte UDP packet to MORAI:

```text
target_ip:   192.168.0.151
target_port: 9093
local_port:  9094
```

For the first real vehicle movement test, use the top-level launch:

```bash
roslaunch morai_competition_bringup gps_stanley_udp_drive.launch
```

Monitor the outgoing packet conversion:

```bash
rostopic echo /udp_bridge/ctrl_cmd_debug
```
