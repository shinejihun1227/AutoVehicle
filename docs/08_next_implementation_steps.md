# Next Implementation Steps

## Step 1. Ubuntu 20.04 ROS Environment

목표:
- 실제 대회 PC 또는 동일한 Ubuntu 20.04 데스크톱에서 ROS1 Noetic 환경을 만든다.

작업:
- ROS Noetic 설치
- `~/morai_competition_ws` 생성
- 이 저장소의 `ros_ws/src` 내용을 workspace `src`에 연결 또는 복사
- `catkin_make` 실행
- `scripts/bashrc_snippet.sh` 내용을 `~/.bashrc`에 반영

검증:

```bash
source ~/.bashrc
cd ~/morai_competition_ws
catkin_make
roslaunch morai_competition_bringup competition.launch
```

## Step 2. MORAI Example Package 확인

목표:
- MORAI 공식 예제 패키지에서 실제 message type, topic name, launch 구조를 확인한다.

확인할 것:
- control command message type
- vehicle status message type
- GPS/IMU/LiDAR/Camera topic name
- rosbridge 사용 방식
- UDP 설정 파일 구조

결과 반영:
- `docs/03_ros_topic_plan.md`
- `configs/network.yaml`
- `configs/sensors.yaml`

## Step 3. Bridge Layer 구현

목표:
- MORAI 입력/출력과 팀 내부 ROS topic을 분리한다.

구현 후보:
- `morai_competition_bridge`
- UDP receiver
- UDP control sender
- MORAI message adapter

검증:
- vehicle status 수신
- collision data 수신
- control command 송신

## Step 4. Localization Baseline 구현

목표:
- GPS/IMU/status를 사용해 ego pose, heading, velocity를 안정적으로 만든다.

출력:
- `/localization/ego_pose`
- `/localization/ego_twist`

## Step 5. Pure Pursuit Baseline 구현

목표:
- waypoint 기반으로 차량을 안정적으로 움직인다.

출력:
- `/planning/target_path`
- `/planning/target_speed`
- `/control/ctrl_cmd`

