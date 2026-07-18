# ROS Topic and Message Plan

## 1. Topic Naming Rules

토픽 이름은 기능과 출처가 드러나도록 구성한다.

```text
/morai/<raw_or_bridge_data>
/sensors/<sensor_name>/<data>
/localization/<state>
/perception/<result>
/planning/<target>
/control/<command>
/diagnostics/<status>
```

## 2. Sensor Topics

| Domain | Topic | Message Candidate | Source |
| --- | --- | --- | --- |
| GPS | `/sensors/gps/fix` | `sensor_msgs/NavSatFix` or MORAI GPS msg | UDP or ROS |
| IMU | `/sensors/imu/data` | `sensor_msgs/Imu` | UDP or ROS |
| LiDAR front | `/sensors/lidar/front/points` | `sensor_msgs/PointCloud2` | ROS-Velodyne |
| LiDAR rear | `/sensors/lidar/rear/points` | `sensor_msgs/PointCloud2` | ROS-Velodyne |
| Camera front | `/sensors/camera/front/image_raw` | `sensor_msgs/Image` | UDP or ROS |
| Vehicle status | `/morai/vehicle_status` | MORAI status msg or custom bridge msg | UDP |
| Collision | `/morai/collision` | MORAI collision msg or custom bridge msg | UDP |

## 3. Internal Topics

| Domain | Topic | Message Candidate | Purpose |
| --- | --- | --- | --- |
| Localization | `/localization/ego_pose` | `geometry_msgs/PoseStamped` | ego pose |
| Localization | `/localization/ego_twist` | `geometry_msgs/TwistStamped` | velocity |
| Perception | `/perception/obstacles` | custom or marker array | obstacle candidates |
| Perception | `/perception/mission_objects` | custom | AI mission objects |
| Planning | `/planning/target_path` | `nav_msgs/Path` | local target path |
| Planning | `/planning/target_speed` | `std_msgs/Float32` | target speed |
| Control | `/control/ctrl_cmd` | MORAI ctrl cmd or custom | steering, accel, brake |
| Diagnostics | `/diagnostics/health` | `diagnostic_msgs/DiagnosticArray` | run state |

## 3.1 Stanley Baseline Topics

The first Stanley controller package uses the following internal interface.

| Domain | Topic | Message | Purpose |
| --- | --- | --- | --- |
| Localization | `/localization/ego_pose` | `geometry_msgs/PoseStamped` | ego x, y, yaw |
| Localization | `/localization/ego_twist` | `geometry_msgs/TwistStamped` | ego speed |
| Localization | `/localization/ego_odom` | `nav_msgs/Odometry` | optional combined pose/twist input |
| Planning | `/planning/target_path` | `nav_msgs/Path` | loaded waypoint path |
| Control | `/control/ctrl_cmd` | `geometry_msgs/Twist` or `morai_msgs/CtrlCmd` | Stanley output |
| Debug | `/control/stanley_target_index` | `std_msgs/Int32` | current target waypoint index |
| Debug | `/control/stanley_cross_track_error` | `std_msgs/Float32` | signed lateral error |
| Debug | `/control/stanley_steering_rad` | `std_msgs/Float32` | steering command before bridge conversion |

## 4. Control Output

최종 출력은 규정상 `[UDP] Ego Ctrl Cmd`를 기준으로 한다.

제어 필드 초안:

```text
steering: lateral control
accel: longitudinal acceleration command
brake: longitudinal brake command
longi_type: 1
```

ROS 내부에서는 `/control/ctrl_cmd`를 만들고, 최종 sender가 UDP 패킷으로 변환한다.

## 5. Launch Group Draft

```text
morai_bringup.launch
  - network config loader
  - UDP receiver/sender
  - rosbridge if needed

sensors.launch
  - gps receiver
  - imu receiver
  - lidar receiver
  - camera receiver

autonomy.launch
  - localization
  - perception
  - planning
  - control
  - diagnostics

competition.launch
  - bringup
  - sensors
  - autonomy
  - logging
```

## 6. Debug Commands

```bash
rostopic list
rostopic hz /sensors/gps/fix
rostopic hz /sensors/imu/data
rostopic hz /sensors/lidar/front/points
rostopic echo /control/ctrl_cmd
rqt_graph
rosbag record /localization/ego_pose /planning/target_path /control/ctrl_cmd
```

## 7. Topic Design TODO

- MORAI 예제 패키지의 실제 message type 확인
- 대회 제공 판정 프로그램의 네트워크 설정 파일 확인
- GPS 좌표계 변환 기준 확인
- LiDAR ROS-Velodyne launch 파라미터 확정
- Camera 개수와 설치 위치는 미션 공지 후 확정
