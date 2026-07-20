# LiDAR Sensor Rules From Competition Spec

This workspace follows the LiDAR part of section `2-6-2. Sensor`.

## Allowed 3D LiDAR

- Count: maximum 1
- Model: `VLP16`
- Intensity type: `Intensity`
- Rotation Rate: free
- Maximum Rotation Rate: `15 Hz`
- Recommended Rotation Rate: `10 Hz` or lower
- Network: UDP

## Recommended MORAI Setting

| Item | Value |
| --- | --- |
| LiDAR model | `VLP16` |
| Intensity type | `Intensity` |
| Destination IP | Ubuntu algorithm PC IP |
| Destination port | `2001` |
| Source port | `2000` |
| Rotation rate | `10 Hz` |

## ROS Topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | Full VLP16 point cloud |
| `/perception/lidar/roi_points` | `sensor_msgs/PointCloud2` | Cropped forward ROI points |
| `/perception/lidar/nearest_obstacle_m` | `std_msgs/Float32` | Nearest obstacle distance in ROI |
| `/perception/lidar/obstacle_stop_required` | `std_msgs/Bool` | True when nearest obstacle is within stop distance |
| `/perception/lidar/obstacle_summary` | `std_msgs/String` | Human-readable ROI diagnostic |
| `/perception/lidar/packet_info` | `std_msgs/String` | UDP packet and frame status |

## Recommended Roles

LiDAR should be used for geometry and distance:

- front obstacle distance
- collision-risk stop flag
- obstacle avoidance trigger
- candidate object position for camera-LiDAR fusion
- side obstacle confirmation if ROI is widened later

Camera should classify visual context. LiDAR should confirm distance and shape.
