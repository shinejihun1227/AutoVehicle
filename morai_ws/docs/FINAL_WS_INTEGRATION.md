# final_ws 통합 안내

## 통합 기준

`final_ws`는 다음 두 소스를 하나의 `morai_ws` catkin workspace로 합친다.

- 현재 주행 스택: GPS/IMU UDP, MGeo ENU EKF, Pure Pursuit, 곡률 기반 속도 제한,
  km/h 기준 PI, `longlCmdType=1` accel/brake, `control_mux`, MORAI CtrlCmd UDP
- ROI `dev/merged_code` (`c752050`): `camera_perception`, `lidar_perception`,
  `common` 메시지, `morai_network`, `sensor_runtime` 및 관련 센서 문서

ROI의 구형 `purepursuit_mgeo`, UDP bridge, `morai_bringup`은 가져오지 않았다.
현재 주행 코드와 패키지명이 겹치고, 속도·명령 계약이 달라 두 주행 스택을 동시에
설치하면 어떤 노드가 최종 명령을 발행하는지 불명확해지기 때문이다.

## 기본 실행

최종 통합 진입점은 다음이다.

```bash
source /opt/ros/noetic/setup.bash
source ~/morai_ws/devel/setup.bash
roslaunch morai_bringup final_ws_bringup.launch enable_control:=false
```

기본적으로 ROI LiDAR tracking과 카메라 lane/YOLO 스택을 사용한다. 카메라 모델이나
GPU/CPU 의존성을 준비하지 않은 경우에는 먼저 카메라를 끈 채 LiDAR와 주행 토픽을
검증한다.

```bash
roslaunch morai_bringup final_ws_bringup.launch \
  enable_control:=false enable_roi_camera:=false
```

모든 검증이 끝난 뒤 실제 제어를 켠다.

```bash
roslaunch morai_bringup final_ws_bringup.launch \
  enable_control:=true
```

## 토픽 연결

```text
GPS/IMU UDP
  -> EKF /localization/odometry
  -> curvature_speed_purepursuit
  -> /control/ctrl_cmd
  -> control_mux
  -> /ctrl_cmd
  -> MORAI UDP

ROI LiDAR tracking
  -> /perception/lidar/tracked_obstacles_map
  -> roi_sensor_safety_adapter
ROI camera traffic/pedestrian/intersection state
  -> Boolean stop topics
  -> roi_sensor_safety_adapter
  -> /detection/fused_safety_stop
  -> control_mux

ROI camera lane geometry
  -> /detection/lane (LaneDetection)
  -> camera_localization_fallback_controller
  -> /control/camera_fallback_cmd
  -> control_mux
```

ROI LiDAR의 confirmed obstacle은 map 좌표로 들어오므로 adapter가 EKF의 현재 위치와
yaw를 사용해 `base_link` 기준 전방 corridor로 변환한다. 카메라의 `ObjectInfoArray`
는 픽셀 검출 결과이므로 거리 기반 장애물 융합에는 직접 사용하지 않고, ROI 브랜치가
제공하는 신호등·보행자·교차로 정지 Bool만 안전정지에 반영한다.

## 단위와 명령 계약

- 목표 속도·현재 속도·곡률 속도 제한 모니터링: `km/h`
- 곡률: `1/m`, 물리 가속도 제한: `m/s²`
- `accel`, `brake`: `0.0~1.0` 정규화 페달
- 최종 제어: `longlCmdType=1`, accel/brake 직접 제어

## GPS/IMU 품질 기반 차선 fallback

`final_ws_bringup.launch`는 `enable_lane_fallback` 기본값이 `true`다. 실제 MORAI
제어를 켜기 전에는 `enable_control:=false`로 차선·센서 상태를 먼저 확인한다.

```text
NORMAL
  -> 기존 MGeo Pure Pursuit nominal 통과
GPS_NOISE / IMU_NOISE
  -> nominal 조향 + 차선 보정
GPS_BLACKOUT + 차선 confidence 0.80 이상이 5회 연속
  -> 차선 조향 중심 fallback, 속도 7.2km/h(2.0m/s) 상한
GPS_BLACKOUT + 차선 불량 또는 nominal stale
  -> accel=0, brake=1 정지
```

ROI `live_overlay.py`는 기존 차선 종류·정지선 Bool과 함께 `/detection/lane`을
발행한다. `confidence`는 좌우 차선 가시성, 차선 폭(3.3m), 검출 길이, 프레임
연속성을 합친 품질 점수다. 차선 fallback은 한쪽 차선이나 순간 검출을 주 제어로
사용하지 않는다.

## 주의사항

현재 기존 센서 스택을 사용하는 `perception_control_bringup.launch`와 ROI 센서
스택을 사용하는 `final_ws_bringup.launch`를 동시에 실행하지 않는다. 두 스택이 같은
UDP 센서 포트 또는 안전정지 토픽을 사용할 수 있다.

`enable_control:=false`에서는 주행 명령을 MORAI에 보내지 않으며, `control_mux`는
신선도 조건이 만족되지 않으면 정지 명령을 발행한다. 실제 주행 전에는 다음 토픽을
확인한다.

```bash
rostopic echo /localization/odometry
rostopic echo /perception/lidar/tracked_obstacles_map
rostopic echo /detection/fused_safety_stop
rostopic echo /control/mux_status
rostopic echo /localization/sensor_quality
rostopic echo /detection/lane
rostopic echo /stability/camera_fallback_status
rostopic echo /ctrl_cmd
```
