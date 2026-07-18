# Coordinate Convention

This project uses one internal coordinate convention for localization,
recorded waypoints, Stanley control, and UDP control conversion.

## Internal ROS Convention

```text
map frame:
  x: East, meters
  y: North, meters
  z: Up, meters
  yaw: 0 rad faces +x/East
  yaw positive: counter-clockwise toward +y/North

base_link frame:
  x: vehicle forward
  y: vehicle left
  z: vehicle up

control:
  steering > 0: left turn
  steering < 0: right turn
```

## Sensor Conversion

`udp_gps_to_localization.py` converts GPS latitude/longitude to a local ENU-like
map frame:

```text
x = East offset from GPS origin
y = North offset from GPS origin
z = altitude offset from GPS origin
```

The first GPS point is used as the origin when no fixed origin is supplied. When
following a recorded path, use the same `origin_lat`, `origin_lon`, and
`origin_alt` that were saved in the CSV.

## Stanley Sign Convention

The Stanley controller assumes positive steering turns left. Cross-track error
is positive when the path is left of the vehicle. For a path segment with
heading `path_yaw` and a front-axle vector from path point to vehicle
`(dx, dy)`, the project uses:

```text
cte = dx * sin(path_yaw) - dy * cos(path_yaw)
```

Example: if the path points east and the vehicle is north/left of the path, then
`cte < 0`, so the controller commands negative steering to move right.

## MORAI UDP Steering

`udp_ctrl_cmd_sender.py` converts Stanley steering radians to MORAI normalized
steering command using:

```text
steering_cmd = steering_rad / max_steer_rad
```

If MORAI reacts in the opposite direction, change:

```yaml
ctrl_sender:
  invert_steering: true
```
