# Competition Execution Checklist

## 1. Before Submission

- Ubuntu 20.04 desktop PC 준비
- ROS1 Noetic 설치 확인
- catkin workspace build 확인
- MORAI 예제 패키지 및 팀 패키지 build 확인
- one-command launch 실행 확인
- PC 재부팅 후에도 launch 실행 확인
- 네트워크 설정값 문서화
- 최종 코드 freeze

## 2. Before Run

- Server PC, Client PC, Team PC LAN 연결 확인
- sensor setting file load 확인
- mission scenario load 확인
- rosbridge 또는 UDP bridge 실행 확인
- GPS 수신 확인
- IMU 수신 확인
- LiDAR 수신 확인
- Camera 수신 확인
- vehicle status 수신 확인
- control command 송신 확인
- 판정 프로그램 reset/start 확인

## 3. Operator Run Sequence

1. PC 수령 및 자리 세팅
2. 준비 파일 확인
3. 미션 시나리오 load 확인
4. 팀 PC에서 ros_bridge 또는 bridge node 실행
5. 센서 및 네트워크 연결 확인
6. keyboard mode 및 P gear 설정 확인
7. 자율주행 코드 실행
8. 실행 이후 PC 조작 중지
9. 판정 프로그램 준비
10. 주행 종료 후 결과 저장

## 4. One-command Launch Goal

최종 목표:

```bash
roslaunch morai_competition_bringup competition.launch
```

이 명령 하나로 다음이 실행되어야 한다.

- network bridge
- sensor receivers
- localization
- perception
- planning
- control
- diagnostics
- rosbag logging

## 5. Failure Cases

### Sensor Not Receiving

대응:
- IP/port 확인
- MORAI sensor setting file 확인
- ROS topic hz 확인
- UDP receiver log 확인

### Control Not Applied

대응:
- `Ego Ctrl Cmd` target IP/port 확인
- longi type 1 설정 확인
- steering/accel/brake command range 확인
- ExternalCtrl mode 확인

### Vehicle Leaves Route

대응:
- localization frame 확인
- waypoint 좌표계 확인
- steering sign 확인
- lookahead distance 조정

### Vehicle Stops

대응:
- target speed 확인
- brake command stuck 여부 확인
- obstacle false positive 확인
- fail-safe condition 확인

## 6. Logs to Save

```text
date/time
mission name
map
sensor setting file
network setting
git commit hash
launch command
score
failure point
operator notes
rosbag path
```

