# final_ws 보완본: Docker 설정과 시험 명령

2026-09-09. 정지선·방향별 신호·좌우회전 속도/출구 정렬·GPS 음영 차선 유지를
반영한 코드의 실행 안내다. 호스트 명령은 **Ubuntu/Linux의 Bash** 기준이다.
컨테이너는 기존 Dockerfile의 Ubuntu 20.04 / ROS Noetic / Python 3.8을 사용한다.
현재 작업 PC에는 실행 가능한 Docker/WSL 배포판이 없어 이미지 빌드와 실제 MORAI
주행은 수행하지 못했다. 로컬 오프라인 시험 통과와 컨테이너 검증을 구분한다.

## 1. 코드 가져오기

새로 받는 경우, 호스트에서:

```bash
git clone --branch final_ws --single-branch \
  https://github.com/shinejihun1227/AutoVehicle.git AutoVehicle
cd AutoVehicle
```

기존 clone은 저장한 작업이 있는지 `git status --short`로 확인한 뒤:

```bash
cd /실제/AutoVehicle/경로
git switch final_ws
git pull --ff-only origin final_ws
git log -1 --oneline
```

아래 빌드는 AutoVehicle 저장소 루트에서 실행한다. 이미지에 코드가 복사되므로
호스트에서 `git pull`만 하고 이전 컨테이너를 재시작하면 새 코드가 반영되지 않는다.
새 태그로 이미지를 빌드하고 새 이름의 컨테이너를 만든다.

## 2. 이미지 빌드

Docker Engine과 NVIDIA 드라이버/Container Toolkit 설치는
[기존 설치 안내](DOCKER_FINAL_WS_FROM_SCRATCH_KO.md)의 2~4절을 따른다.
NVIDIA GPU 구성은 [공식 설치 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)를 참조한다.

```bash
export CODE_REF="$(git rev-parse HEAD)"
# 제공된 MORAI SDK 대응 SHA가 있으면 아래 값 대신 그 40자리 SHA를 사용한다.
export MORAI_MSGS_REF="$(git ls-remote \
  https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git \
  refs/heads/main | awk '{print $1}')"
[[ "$MORAI_MSGS_REF" =~ ^[0-9a-f]{40}$ ]] || { echo 'MORAI 메시지 SHA 확인 실패'; exit 1; }

# 오프라인 검사 기준은 cpu. GPU 환경이 준비됐으면 cu121로 설정한다.
export TORCH_FLAVOR=cpu
export IMAGE="morai-final-ws:${CODE_REF:0:8}-${TORCH_FLAVOR}"
sudo docker build --progress=plain \
  --build-arg CODE_REVISION="$CODE_REF" \
  --build-arg MORAI_MSGS_REF="$MORAI_MSGS_REF" \
  --build-arg TORCH_FLAVOR="$TORCH_FLAVOR" \
  -f morai_ws/docker/final_ws/Dockerfile -t "$IMAGE" morai_ws
```

Dockerfile은 catkin 빌드와 5개 주행 회귀 시험 묶음을 실행한다. 메시지·모델·의존성은
이미지에 포함된다. CPU 실시간 추론이 늦어 차선 관측의 0.15초 제한을 넘으면
제어는 정지한다. GPU는 `TORCH_FLAVOR=cu121`로 빌드하고 실행 시 `--gpus all`을 추가한다.
외부 메시지 저장소 main의 선택은 대회 SDK 호환 검증을 대신하지 않는다.

## 3. 오프라인 시험

호스트에서 네트워크 없는 컨테이너로 실행한다. ROS master나 MORAI는 필요 없다.

```bash
sudo docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/run_regression.py

# GPS 음영구간 테스트만 선택
sudo docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/run_regression.py --suite blackout

# 대회 경로 우선, 신호 방향 불일치, 조향 경로 대조 테스트
sudo docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/run_regression.py --suite turn

# 실제 모델 로드와 빈 영상 1회 추론
sudo docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/smoke_models.py
```

GPU 모델 검사는 마지막 명령에 `--gpus all`을 docker 옵션으로 추가하고,
Python 인자로 `--device cuda`를 추가한다. 회귀 시험 성공 표시는 `REGRESSION_PASS`,
모델 smoke 성공 표시는 `OFFLINE_SMOKE_PASS`다.

보완 시점 로컬 회귀 결과는 359개 통과(카메라 67, 정지선 54, 회전/신호 190,
음영/센서 41, 곡률 7)다. 시험은 합성 차선/경로와 대체 ROS 입력을 사용하므로,
실제 영상 정확도·MORAI 차량 응답은 아래 센서/주행 단계에서 확인한다.

## 4. 센서 네트워크와 컨테이너 설정

예시 MORAI PC 주소 `192.168.0.151`을 실제 값으로 바꾼다. MORAI 센서의 목적지 IP는
**Docker를 실행하는 Ubuntu 호스트의 LAN IP**다. Linux의 `--network host`는
호스트 네트워크를 공유한다. 같은 센서 포트를 점유하는 launch를 중복 실행하지 않는다.
[Docker host 네트워크 설명](https://docs.docker.com/engine/network/drivers/host/)

| 데이터 | Docker 호스트 수신 포트 또는 MORAI 수신 포트 |
|---|---|
| Cam1 차선 | UDP 1101 |
| Cam4 신호·객체 | UDP 1131 |
| VLP16 LiDAR | UDP 2001 |
| GPS / IMU | UDP 3001 / 4001 |
| Ego 상태 | UDP 909 |
| 차량 제어 | MORAI UDP 9093, 송신 출발 포트 9094 |
| 방향지시등 | MORAI UDP 9097 |

호스트에서 새 설정 폴더와 컨테이너를 만든다. 기존 보정 파일이 있으면 변경 내용과
비교해 새 파일에 반영한다. 초기 설정을 복사하면 Cam4는 미보정 상태다.

```bash
export SETTINGS="$HOME/morai-final-test/settings-${CODE_REF:0:8}"
export LOGS="$HOME/morai-final-test/logs-${CODE_REF:0:8}"
export CONTAINER="morai-final-${CODE_REF:0:8}"
mkdir -p "$SETTINGS" "$LOGS"
cp -n morai_ws/src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml \
  "$SETTINGS/turn_signal_maneuvers.yaml"

# GPU 이미지는 --gpus all 추가
sudo docker run -dit --name "$CONTAINER" --network host --shm-size=1g \
  -v "$SETTINGS:/opt/morai-config:ro" -v "$LOGS:/logs" "$IMAGE" bash
sudo docker exec -it "$CONTAINER" /usr/local/bin/morai-entrypoint bash
```

`docker exec`에는 entrypoint를 명시해 ROS·Python venv·catkin 환경을 불러온다.
호스트 전체 소스로 이미지의 workspace를 덮어 마운트하지 않는다.
Windows Docker Desktop은 Linux 컨테이너와 WSL2 환경을 사용하며, Desktop 4.34 이상에서
Settings → Resources → Network → Enable host networking을 켜야 이 방식의 TCP/UDP
연결을 사용할 수 있다. 위 명령은 Bash 구문이며 PowerShell 구문이 아니다.
Desktop/WSL2에서는 센서 목적지와 실제 UDP 도착을 별도로 확인한다.

## 5. 제어 OFF로 센서 관찰

**컨테이너 내부**에서 실행한다. `MORAI_HOST_IP`는 매 새 터미널에서 설정한다.

```bash
export MORAI_HOST_IP=192.168.0.151
xvfb-run -a -s '-screen 0 1920x1080x24' \
  roslaunch morai_bringup final_ws_bringup.launch \
  workspace_path:="$MORAI_WS" morai_host_ip:="$MORAI_HOST_IP" \
  turn_signal_maneuvers_file:=/opt/morai-config/turn_signal_maneuvers.yaml \
  enable_control:=false enable_turn_signal:=false \
  lateral_accel_limit_mps2:=1.0 \
  max_speed_kph:=3.0 fallback_speed_cap_kph:=3.0 roi_lidar_rviz:=false
```

현재 카메라 코드가 창을 사용하므로 Xvfb를 적용했다. 실제 창을 보려면 기존 설치
안내의 X11 설정을 적용하고 `xvfb-run`을 제거한다. roslaunch가 ROS master를 자동 시작한다.
제어 OFF에서는 주행용 nominal 명령이 발행되지 않아 `nominal_stale` 등의 정지가
나올 수 있다. 이 단계는 센서/인식 관찰이며 차량을 강제로 멈추는 기능은 아니다.

호스트의 별도 터미널에서 같은 컨테이너에 진입한 뒤:

```bash
rostopic echo -n 1 /localization/sensor_quality
rostopic echo -n 1 /perception/camera/lane_quality
rostopic echo -n 1 /detection/lane
rostopic echo -n 1 /perception/camera/stopline
rostopic echo -n 1 /stability/camera_fallback_status
rostopic echo -n 1 /control/maneuver_status
rostopic hz /detection/lane
```

`rostopic hz`는 Ctrl+C로 종료한다. 발행 빈도와 함께 `lane_source_age_sec`가 기본
0.15초 안에 드는지 확인한다. 차선은 7~14m 양쪽 경계가 필요하며,
`lane_primary_usable`에는 품질 0.80, 서로 다른 영상 5개, 최소 0.20초 확인이 필요하다.

`maneuver_status`에서 `reference_path_match: true`를 먼저 확인한다.
`reference_path_not_received`이면 `/experimental/curvature_reference_path` 발행 노드를,
`reference_path_*_mismatch`이면 조향 노드와 fusion의 `path_file` 및 좌표계를 확인한다.
두 노드를 같은 대회 경로로 실행해야 하며 이 검사를 꺼서 통과시키지 않는다.
`route_direction`이 가려는 방향, `signal_allowed_directions`가 현재 확인된 신호의
허용 방향이다. 일치 여부는 `route_signal_compatible`, 최종 진입 허가는 `permission`이다.

## 6. 보정 후 MORAI 저속 시험

현재 `signal_camera.calibrated=false`이므로 교차로 통과는 차단된다. 실제 Cam4
해상도/FOV/장착 자세와 지도 투영을 검증해 설정 파일에 반영해야 한다.
`calibrated`만 true로 바꾸는 것으로 보정이 되지 않는다. 추가로
[미해결 사항](KNOWN_ISSUES_FINAL_WS.md)의 보행자 입력 소실 및 LiDAR 차선 변경
간격 데이터 지연 문제는 남아 있다. 아래는 원인과 보정을 확인한 뒤 사용하는
MORAI 시험 명령이며 대회 주행 승인 절차를 대신하지 않는다.

관찰 launch를 종료하고 MORAI에서 정지·시작 위치·외부 UDP 제어를 확인한 뒤,
컨테이너에서 실행한다.

```bash
export MORAI_HOST_IP=192.168.0.151
xvfb-run -a -s '-screen 0 1920x1080x24' \
  roslaunch morai_bringup final_ws_bringup.launch \
  workspace_path:="$MORAI_WS" morai_host_ip:="$MORAI_HOST_IP" \
  turn_signal_maneuvers_file:=/opt/morai-config/turn_signal_maneuvers.yaml \
  enable_control:=true enable_turn_signal:=true \
  lateral_accel_limit_mps2:=1.0 \
  max_speed_kph:=3.0 fallback_speed_cap_kph:=3.0 \
  lane_stable_samples:=5 lane_stable_sec:=0.20 lane_control_timeout_sec:=0.15 \
  blackout_max_duration_sec:=15.0 blackout_max_distance_m:=30.0 \
  turn_signal_lead_time_sec:=5.0 roi_lidar_rviz:=false
```

| 시험 | 확인할 동작 |
|---|---|
| 직선/완만한 곡선 | 경로·차선 이탈과 조향 진동, 설정 속도 준수 |
| 적색·황색·신호 미확인 | 정지선 접근 감속·대기, 확인된 허가 후 출발 |
| 좌·우회전 | 방향지시등 선행 5초, 방향별 허가, 회전 속도 제한, 출구 정렬 |
| 직진 경로 + 좌/우회전 전용 신호 | 회전하지 않고 직진 허가를 정지선에서 대기, 방향지시등 OFF |
| 직진 경로 + 직진/좌회전 동시 신호 | 대회 경로를 따라 직진 |
| 좌/우회전 경로 + 다른 방향 전용 신호 | 경로 방향 유지, 자기 방향 허가까지 정지선 대기 |
| 정상 주행 후 실제 GPS 음영 구간 | IMU·속도·차선 유효 시 제한된 차선 유지 |
| 음영 중 차선 품질 저하/영상 중단 | 가속 0·제동, 마지막 확정 영상 0.25초 이후 정지 |
| GPS 복구 | 1초 연속 정상 확인 후 경로 조향으로 전환 |

위 시험 명령은 전체 최고속도 3km/h다. 좌회전 15/우회전 10km/h의 방향별 상한은
제거했다. 모든 방향이 공통 `lateral_accel_limit_mps2`로 곡률 속도를 계산하며,
`max_speed_kph`가 전체 최고속도를 제한한다. 두 launch 인자가 nominal과 fusion에
동시에 전달된다. 이전 설정 파일의 `turn_left_speed_kph`, `turn_right_speed_kph`,
`turn_lateral_accel_mps2`는 무시하고 경고하므로 삭제한다. 곡률 제한의 계산값은
`turn_curve_speed_kph`, 최종 접근 상한은 `turn_speed_limit_kph`에서 확인한다.
GPS 음영에서 새 좌우회전이나
차선 변경을 허용하지 않으며, 교차로 접근·15초/30m 초과·IMU/속도 소실이면 정지한다.
합성 dropout은 3절 오프라인 시험에서 재현한다. 주행 토픽에 가짜 정상 상태나
가짜 녹색을 발행해 통과시키는 방식은 사용하지 않는다.

별도 컨테이너 터미널에서 로그를 저장한다.

```bash
rosbag record --split --size=1024 -O /logs/final_ws_updated \
  /gps /Imu /Ego_topic /localization/odometry /localization/sensor_quality \
  /detection/lane /perception/camera/lane_quality /stability/camera_fallback_status \
  /perception/camera/stopline /detection/traffic_light \
  /control/maneuver_status /control/turn_signal_state /ctrl_cmd
```

종료는 MORAI에서 정지/외부 제어 해제를 확인한 뒤 launch와 bag을 Ctrl+C로 종료한다.
시나리오 Reset·순간이동 후에는 ROS 주행 노드를 재시작해 위치·경로 진행·허가를 초기화한다.
