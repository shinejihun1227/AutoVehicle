# Coordinate and Stanley Driving Plan

## 1. Why Coordinate Setup Comes First

Stanley controller is simple, but it is very sensitive to coordinate assumptions. If yaw direction, steering sign, or waypoint frame is wrong, the vehicle will immediately steer away from the path.

Before tuning Stanley gain, confirm these three values:

```text
ego_x, ego_y
ego_yaw
waypoint_x, waypoint_y
```

All three must use the same map frame.

## 2. Recommended First Coordinate Policy

For the first MORAI driving test, use MORAI vehicle state position as the map coordinate if available.

Recommended starting policy:

```text
frame: map
x: MORAI ego position x
y: MORAI ego position y
yaw: MORAI heading converted to radians, or IMU yaw
speed: MORAI ego velocity magnitude
```

Avoid raw latitude/longitude conversion in the first Stanley test. GPS-to-map conversion can be added after basic path tracking works.

## 3. ROS State Topics for Stanley

The Stanley package currently expects one of these input styles.

### Option A. Pose + Twist

```text
/localization/ego_pose   geometry_msgs/PoseStamped
/localization/ego_twist  geometry_msgs/TwistStamped
```

### Option B. Odometry

```text
/localization/ego_odom   nav_msgs/Odometry
```

For the first implementation, create a small localization adapter node later that converts MORAI vehicle status into one of these forms.

## 4. Axis Convention

Use the standard ROS planar convention:

```text
map x: forward/east-like map axis
map y: left/north-like map axis
yaw = 0 rad: vehicle heading along +x
yaw > 0: counter-clockwise rotation
steering > 0: left turn
```

If MORAI steering direction is opposite, fix it with a single sign parameter in the bridge/control sender, not inside every algorithm.

## 5. Stanley Formula

```text
steering = heading_error + atan(k * cross_track_error / (velocity + softening_gain))
```

Where:

```text
heading_error = path_yaw - ego_yaw
cross_track_error = signed lateral distance from front axle to path
k = Stanley gain
softening_gain = low-speed stabilizer
```

Initial values:

```text
k: 0.7
softening_gain: 1.0
target speed: 3.0 m/s
max steering: 40 deg
```

## 6. First Test Order

1. Publish ego pose and twist.
2. Run straight waypoint test at 3 m/s.
3. Verify target waypoint index increases.
4. Verify cross-track error approaches 0.
5. If vehicle turns away from path, check steering sign and yaw direction.
6. Run curve waypoint test.
7. Only after this, create K-City route waypoints.

## 7. Common Failure Patterns

### Vehicle turns in the opposite direction

Likely causes:

- steering sign is reversed
- yaw sign is reversed
- cross-track error sign is reversed

### Vehicle oscillates around the path

Likely causes:

- Stanley gain too high
- target speed too high
- yaw data noisy
- waypoint spacing too sparse

### Vehicle follows the path but offset remains

Likely causes:

- ego position is rear axle but controller assumes front axle
- wheelbase is wrong
- waypoint coordinate frame is shifted

### Vehicle does not move

Likely causes:

- control command is not connected to MORAI CtrlCmd
- ExternalCtrl mode is not active
- longi_type is wrong
- accel/brake command range is wrong

