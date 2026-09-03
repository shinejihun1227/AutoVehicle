# Detection·Safety Stop·Control Mux 초기 구성

이번 단계는 카메라 4대, LiDAR, 인식 결과, safety stop, control mux를 연결할 수
있는 ROS 인터페이스와 초기 구현을 구성한 것이다. 카메라 lane·traffic detector는
연결 검증용 고전적 threshold 구현이며, 카메라 장애물 detector는 학습 모델을
연결하기 전까지 빈 결과를 발행한다.

## 최종 토픽 구조

```text
/control/ctrl_cmd                 Pure Pursuit nominal command
/detection/lane                    LaneDetection
/detection/traffic_light           TrafficLight
/detection/lidar_obstacles         ObstacleArray
/detection/camera_obstacles        ObstacleArray
/detection/fused_obstacles         ObstacleArray
/detection/fused_safety_stop       SafetyStop
/ctrl_cmd                          control_mux final command
```

`/ctrl_cmd`는 `control_mux`만 발행하고, `morai_udp_drive_bridge`는 이 최종 명령만
MORAI로 전송한다.

## 패키지

- `common/morai_perception_msgs`: 공통 인식·안전 메시지
- `control/control_mux`: 최종 제어 중재 및 테스트 publisher
- `detection/morai_camera_perception`: 카메라 health, 차선, 신호, 장애물 인터페이스
- `detection/lidar_obstacle_detector`: LiDAR TF·ROI·nearest obstacle·safety stop
- `detection/morai_sensor_fusion`: camera·LiDAR obstacle fusion

## 빌드

```bash
cd ~/morai_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## control_mux 단독 검증

```bash
roslaunch control_mux control_mux_test.launch \
  stop_after_sec:=5.0 \
  clear_after_sec:=10.0
```

다른 터미널에서 확인한다.

```bash
rostopic echo /control/mux_status
rostopic echo /ctrl_cmd
```

처음 5초는 nominal command가 통과하고, 5~10초에는 brake stop, 10초 이후에는
nominal command가 다시 통과해야 한다.

가상 차량까지 연결하려면 다음을 사용한다.

```bash
roslaunch control_mux control_mux_closed_loop.launch
```

Pure Pursuit가 `/control/ctrl_cmd`에 nominal 명령을 발행하고, `control_mux`가
`/ctrl_cmd`를 발행하며, 가상 차량이 최종 명령을 소비한다. `stop_after_sec` 이후
가상 차량의 속도가 0으로 감소하는지 확인한다.

## 통합 런치

```bash
roslaunch morai_bringup perception_control_bringup.launch \
  enable_control:=false
```

기본값은 실제 제어를 끈 상태이다. 실제 MORAI 제어는 다음 조건을 확인한 뒤에만
켜야 한다.

- 카메라 4개 status가 receiving
- `/lidar3D`와 `velodyne -> base_link` TF 정상
- `/detection/fused_safety_stop`이 주기적으로 발행됨
- `/control/mux_status`가 nominal 또는 stop으로 예상대로 변함
- `/ctrl_cmd`가 stop 상황에서 velocity 0, brake 1인지 확인

## 현재 구현의 한계

- 카메라 차선·신호는 calibration 전의 고전적 threshold baseline이다.
- 카메라 장애물은 아직 학습 모델이 없어 빈 `ObstacleArray`를 발행한다.
- LiDAR detector는 초기 ROI와 nearest point 기반이며 정교한 군집화가 필요하다.
- `velodyne` TF가 MORAI에서 제공되지 않으면 실제 장착값으로 static TF를 추가해야 한다.
- lane 결과는 기본적으로 control mux의 조향을 직접 바꾸지 않는다. 지도 기반
  Pure Pursuit와 충돌하지 않도록 `lane_correction_enabled=false`가 기본이다.
