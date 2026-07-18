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

## Experimental Localization Check

`udp_status_to_localization.py` currently uses `parser_mode: photo_guess`.
This is an experimental parser inferred from the captured 181-byte `#MoraiInfo$`
screenshots. It should be used only to confirm that localization topics can be
published before the official packet layout is finalized.

Run localization only, without sending vehicle control packets:

```bash
roslaunch yeoms_udp_bridge udp_localization_guess.launch
```

Then check:

```bash
rostopic echo /localization/ego_pose
rostopic echo /localization/ego_twist
rostopic echo /udp_bridge/status_debug
```

Current guessed offsets are:

```yaml
status_adapter:
  parser_mode: "photo_guess"
  numeric_type: "float32"
  yaw_offset: 86
  x_offset: 94
  y_offset: 98
  auto_origin: true
```

The node publishes:

```text
/localization/ego_pose
/localization/ego_twist
/udp_bridge/status_debug
```

These localization topics are the inputs used by `yeoms_control`.
