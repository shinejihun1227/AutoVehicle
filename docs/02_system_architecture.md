# 시스템 아키텍처

## 1. 전체 구조

```text
MORAI SIM
  |
  | UDP / ROS
  v
ROS1 Noetic on Ubuntu 20.04
  |
  +-- sensing
  |     +-- gps
  |     +-- imu
  |     +-- lidar
  |     +-- camera
  |
  +-- localization
  |     +-- ego pose
  |     +-- velocity
  |     +-- heading
  |
  +-- perception
  |     +-- obstacle detection
  |     +-- lane/road context
  |     +-- AI mission recognition
  |
  +-- planning
  |     +-- global waypoint following
  |     +-- local obstacle avoidance
  |     +-- speed planning
  |
  +-- control
        +-- steering command
        +-- accel/brake command
        +-- fail-safe command
```

## 2. 설계 원칙

### 규정 우선

센서, 네트워크, OS, ROS 버전은 대회 규정에서 허용한 범위만 사용한다.

### ROS 통합

센서 데이터는 가능하면 ROS topic으로 통합한다. 알고리즘 노드 사이의 인터페이스를 명확하게 만들고, `rostopic`, `rqt`, `rosbag`으로 디버깅 가능하게 하기 위함이다.

### UDP 제어 우선

차량 제어는 규정에 명시된 `[UDP] Ego Ctrl Cmd`를 기준으로 설계한다. ROS 내부 control node가 최종 제어 값을 계산한 뒤 UDP sender가 MORAI로 전송하는 구조를 목표로 한다.

### One-command Run

대회 당일에는 operator가 자율주행 코드 실행 이후 PC를 조작할 수 없다. 따라서 모든 노드는 launch 파일 하나로 실행되도록 정리한다.

## 3. 노드 후보

```text
/morai_bridge
/sensor_gps_node
/sensor_imu_node
/sensor_lidar_node
/sensor_camera_node
/localization_node
/perception_lidar_node
/perception_camera_node
/mission_ai_node
/global_path_node
/local_planner_node
/vehicle_controller_node
/udp_control_sender_node
/health_monitor_node
```

## 4. 좌표계 초안

```text
map
  -> odom
    -> base_link
      -> gps_link
      -> imu_link
      -> lidar_front
      -> lidar_rear
      -> camera_front
```

초기에는 TF 구조를 단순하게 유지한다. 센서 위치가 확정되면 `config/sensors.yaml`에 translation/rotation 값을 기록하고 launch 단계에서 반영한다.

## 5. 데이터 흐름

### Localization

입력:
- GPS
- IMU
- Competition Vehicle Status

출력:
- ego pose
- heading
- velocity

주요 사용처:
- global path tracking
- local planner
- control

### Perception

입력:
- 3D LiDAR
- Camera
- CollisionData

출력:
- obstacle candidates
- mission object candidates
- risk state

주요 사용처:
- local obstacle avoidance
- AI mission decision
- fail-safe

### Planning

입력:
- ego pose
- global route or checkpoint list
- obstacle candidates
- mission state

출력:
- target waypoint
- target velocity
- target lane/behavior

### Control

입력:
- target waypoint
- target velocity
- current ego state

출력:
- steering
- accel
- brake

## 6. 초기 알고리즘 선택

### 경로 추종

초기 baseline은 Pure Pursuit를 사용한다. 차량 모델이 명확하고 구현이 단순하며, waypoint 기반 주행 안정성을 빠르게 확보할 수 있다.

이후 개선 후보:
- Stanley controller
- LQR lateral controller
- MPC

### 속도 제어

초기 baseline은 PID 기반 speed controller를 사용한다.

개선 후보:
- 곡률 기반 speed profile
- 장애물 거리 기반 adaptive speed
- checkpoint 접근 감속

### 장애물 회피

초기 baseline은 LiDAR clustering과 rule-based local path shift를 사용한다.

개선 후보:
- occupancy grid
- dynamic window approach
- lattice planner

### AI 미션

카메라 중심 인식이 필요한 경우를 대비해 별도 `mission_ai_node`로 분리한다. 미션 공지 전에는 모델을 확정하지 않고, 데이터 수집과 inference 구조만 준비한다.

