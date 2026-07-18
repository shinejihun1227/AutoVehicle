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
