# ROS Workspace LiDAR

`ros_ws_lidar` is a LiDAR-focused MORAI workspace. It is separated from the driving controller workspaces so VLP16 UDP reception and obstacle perception can be tested independently.

Main packages:

- `yeoms_lidar_bridge`: receives MORAI VLP16 UDP packets and publishes ROS `PointCloud2`.
- `yeoms_lidar_perception`: extracts a simple forward ROI and publishes nearest-obstacle diagnostics.
- `yeoms_lidar_bringup`: launch files for LiDAR-only tests.

Default MORAI LiDAR setting:

- Model: `VLP16`
- Intensity type: `Intensity`
- Network: `UDP`
- Recommended rotation rate: `10 Hz` or lower
- Maximum rotation rate: `15 Hz`
- MORAI LiDAR source port: `2000`
- Ubuntu algorithm receive port: `2001`
- ROS point cloud topic: `/sensors/lidar/points`

The first implementation is a baseline parser and obstacle-distance detector. It is meant to verify LiDAR input, check packet rate, and create ROS topics that later clustering, tracking, and camera-LiDAR fusion can use.

See `SENSOR_RULES.md` for the rule-based sensor summary.
