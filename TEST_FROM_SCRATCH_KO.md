# 새 Docker에서 곡률 기반 주행·GPS blackout 테스트하기

이 문서는 `codex/curvature-only-drive` 브랜치의 코드를 Ubuntu 22.04 호스트에서 새 Docker로 처음 구성하고, ROS Noetic·MORAI 메시지·필요 라이브러리 설치부터 곡률 기반 주행과 GPS blackout 대응까지 순서대로 검증하기 위한 실행 문서다.

이 문서의 기본 원칙은 다음과 같다.

- 기존 Docker는 삭제하지 않고 새 컨테이너를 별도로 만든다.
- 실제 MORAI GPS·IMU를 사용한다. 고정 noise injector는 기본 테스트에서 사용하지 않는다.
- `morai_msgs`는 현재 GitHub 저장소에 포함되어 있지 않으므로 MORAI ROS 패키지를 별도로 준비한다.
- 먼저 `enable_control:=false`로 토픽·상태·곡률을 확인하고, 마지막에만 `true`로 차량 제어를 켠다.
- `morai_udp_ekf_curvature_only.launch`와 다른 주행 launch를 동시에 실행하지 않는다.

## 0. 테스트에 사용할 값 확인

아래 값은 예시다. MORAI PC와 Ubuntu PC에서 `0902최신` 센서·네트워크 설정 파일을 확인한 뒤 실제 값으로 바꾼다.

```text
MORAI simulator PC      : 192.168.0.148   → morai_host_ip
Ubuntu/Docker host PC   : 192.168.0.185   → MORAI destination IP
GPS UDP port            : 3001
IMU UDP port            : 4001
Control UDP remote port : 9093
Control UDP source port : 9094
```

`morai_host_ip`는 최종 `CtrlCmd`를 보낼 MORAI 시뮬레이터 PC 주소다. 센서 패킷의 destination IP는 Ubuntu 호스트 주소여야 한다. IP가 바뀌면 launch 명령의 `morai_host_ip`와 MORAI 센서 설정을 함께 바꾼다.

## 1. 기존 Docker 보존

Ubuntu 호스트에서 기존 설정을 백업한다. 기존 컨테이너는 아직 삭제하지 않는다.

```bash
sudo docker inspect morai_noetic_gui > morai_noetic_gui.inspect.json
sudo docker ps -a
```

기존 컨테이너에만 있는 MORAI ROS 패키지가 있다면 다음으로 위치를 확인한다.

```bash
sudo docker exec -it morai_noetic_gui bash
find / -path '*morai_msgs/package.xml' -print 2>/dev/null
exit
```

찾은 `morai_msgs` 디렉터리는 새 Docker에 복사할 때 사용할 수 있다.

## 2. 새 ROS Noetic Docker 생성

Ubuntu 호스트에서 실행한다. Ubuntu 22.04 호스트에서도 Docker 내부는 Ubuntu 20.04 Focal + ROS Noetic으로 고정한다.

```bash
xhost +SI:localuser:root

sudo docker pull ros:noetic-ros-base-focal

sudo docker run --name morai_noetic_clean \
  --network host \
  -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros:noetic-ros-base-focal \
  bash
```

`--network host`는 MORAI의 UDP 센서·제어 패킷을 Ubuntu 호스트 네트워크에서 직접 사용하기 위한 설정이다. 기존 컨테이너가 별도의 UDP 포트 매핑이나 GUI 설정을 사용했다면 `morai_noetic_gui.inspect.json`과 비교한다.

컨테이너를 나중에 다시 실행할 때는 다음을 사용한다.

```bash
sudo docker start morai_noetic_clean

sudo docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  morai_noetic_clean \
  bash
```

## 3. Docker 내부 기본 패키지 설치

새 컨테이너 내부에서 실행한다.

```bash
apt-get update

DEBIAN_FRONTEND=noninteractive apt-get install -y \
  git \
  build-essential \
  cmake \
  python3-pip \
  python3-catkin-pkg \
  python3-rosdep \
  python3-opencv \
  python3-numpy \
  ros-noetic-catkin \
  ros-noetic-tf \
  ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-diagnostic-msgs \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport
```

ROS 환경을 먼저 source한다.

```bash
source /opt/ros/noetic/setup.bash
```

`rosdep`가 새 컨테이너에 초기화되어 있지 않은 경우에만 초기화한다.

```bash
rosdep init
rosdep update
```

`rosdep init`에서 이미 초기화되었다는 메시지가 나오면 무시하고 `rosdep update`를 실행한다.

## 4. GitHub 주행 코드 내려받기

저장소 최상위와 실제 catkin workspace가 한 단계 다르므로, `/root/AutoVehicle` 아래에 clone한다.

```bash
cd /root

git clone \
  -b codex/curvature-only-drive \
  https://github.com/shinejihun1227/AutoVehicle.git \
  AutoVehicle

cd /root/AutoVehicle/morai_ws
```

실제 workspace 경로는 다음이다.

```text
/root/AutoVehicle/morai_ws
```

`/root/AutoVehicle`에서 `catkin_make`를 실행하지 않는다.

## 5. `morai_msgs` 준비

### 5-1. 반드시 확인할 점

현재 GitHub 저장소에는 다음 커스텀 메시지는 포함되어 있다.

```text
morai_perception_msgs
```

하지만 MORAI 시뮬레이터가 사용하는 다음 패키지는 저장소에 포함되어 있지 않다.

```text
morai_msgs
```

따라서 `morai_msgs`는 MORAI ROS SDK, MORAI 예제 workspace 또는 기존 정상 동작 workspace에서 받은 **동일 버전의 ROS Noetic 패키지**를 준비해야 한다. `pip install morai_msgs`로 설치하는 Python 라이브러리가 아니며, `package.xml`, `CMakeLists.txt`, `msg/`를 포함하는 catkin 메시지 패키지다.

현재 주행 코드에서 필요한 대표 메시지는 다음과 같다.

```text
morai_msgs/CtrlCmd
morai_msgs/GPSMessage
morai_msgs/EgoVehicleStatus
```

### 5-2. 새 Docker로 복사

Ubuntu 호스트에 `morai_msgs` 원본 디렉터리가 `/home/<사용자>/morai_msgs`로 준비되어 있다고 가정한다.

호스트 터미널에서 실행한다.

```bash
sudo docker cp \
  /home/<사용자>/morai_msgs \
  morai_noetic_clean:/root/AutoVehicle/morai_ws/src/common/
```

그 다음 Docker 내부에서 확인한다.

```bash
cd /root/AutoVehicle/morai_ws
test -f src/common/morai_msgs/package.xml
find src/common/morai_msgs/msg -maxdepth 1 -type f -print
```

만약 기존 컨테이너에서만 `morai_msgs`를 찾았다면, 호스트로 먼저 복사한 뒤 새 컨테이너로 넣는다.

```bash
sudo docker cp \
  morai_noetic_gui:/경로/찾은/morai_msgs \
  /home/<사용자>/morai_msgs

sudo docker cp \
  /home/<사용자>/morai_msgs \
  morai_noetic_clean:/root/AutoVehicle/morai_ws/src/common/
```

메시지 필드가 현재 MORAI 시뮬레이터 버전과 다르면 빌드가 되더라도 UDP 데이터 해석이나 `CtrlCmd` 송신이 맞지 않을 수 있으므로, MORAI에서 제공한 패키지 버전을 그대로 사용한다.

## 6. 의존성 확인 및 catkin 빌드

Docker 내부에서 실행한다.

```bash
source /opt/ros/noetic/setup.bash
cd /root/AutoVehicle/morai_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y

catkin_make
source devel/setup.bash
```

메시지와 패키지가 실제로 검색되는지 확인한다.

```bash
rospack find morai_msgs
rospack find morai_perception_msgs
rospack find stability_stack
rospack find curvature_speed_purepursuit
rospack find sensor_noise_estimator

rosmsg show morai_msgs/CtrlCmd
rosmsg show morai_msgs/GPSMessage
rosmsg show morai_msgs/EgoVehicleStatus
rosmsg show morai_perception_msgs/LaneDetection
```

여기서 `morai_msgs`가 검색되지 않으면 주행 launch를 실행하지 말고 5단계로 돌아가 메시지 패키지를 먼저 넣는다.

## 7. 새 터미널 준비

`roscore`, 주행 launch, 토픽 확인용으로 Docker 터미널을 여러 개 사용한다. 각 새 터미널에서 다음을 먼저 실행한다.

호스트:

```bash
sudo docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  morai_noetic_clean \
  bash
```

Docker 내부:

```bash
source /opt/ros/noetic/setup.bash
source /root/AutoVehicle/morai_ws/devel/setup.bash
```

## 8. ROS Master 실행

터미널 1에서 실행한다.

```bash
roscore
```

`roslaunch`가 자동으로 Master를 시작하는 경우도 있지만, 여러 launch와 토픽을 확인할 때는 별도 `roscore`를 먼저 실행하는 편이 오류를 줄인다.

## 9. MORAI 네트워크와 원본 센서 확인

터미널 2에서 주행 launch 실행 전에 토픽 목록을 확인한다.

```bash
rostopic list | grep -E 'gps|Imu|camera|lidar|ctrl_cmd'
```

MORAI에서 센서가 전송된 뒤 다음 결과가 0이 아닌 주기로 나와야 한다.

```bash
rostopic hz /gps
rostopic hz /Imu
```

원본 메시지 내용도 한 번 확인한다.

```bash
rostopic echo -n 1 /gps
rostopic echo -n 1 /Imu
```

센서 주기가 0이거나 토픽 자체가 없으면 다음을 확인한다.

1. MORAI 센서 설정의 destination IP가 Ubuntu 호스트 IP인지 확인한다.
2. `morai_noetic_clean`이 `--network host`로 실행되었는지 확인한다.
3. UDP 포트 3001, 4001이 방화벽에 막히지 않았는지 확인한다.
4. `morai_host_ip`가 현재 MORAI 시뮬레이터 PC IP와 일치하는지 확인한다.

## 10. 곡률 기반 주행 preview

이 launch는 카메라·LiDAR·가상 차량·인위적인 noise injector를 사용하지 않는다. 실제 GPS·IMU를 robust filter와 EKF에 넣고, MGeo Global Path의 3점 곡률을 이용해 Pure Pursuit 조향과 목표 속도를 계산한다.

터미널 2에서 제어를 끈 상태로 실행한다.

```bash
roslaunch stability_stack morai_udp_ekf_curvature_only.launch \
  workspace_path:=/root/AutoVehicle/morai_ws \
  path_file:=/root/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_control:=false \
  stop_on_gps_blackout:=false
```

실제 IP가 다르면 `192.168.0.148`을 MORAI PC 주소로 바꾼다.

### 10-1. Localization과 곡률 토픽 확인

터미널 3에서 확인한다.

```bash
rostopic hz /localization/gps
rostopic hz /localization/gps_filtered
rostopic hz /Imu_filtered
rostopic hz /localization/odometry
```

```bash
rostopic echo -n 1 /localization/gps_health
rostopic echo -n 1 /stability/curvature_only_gps_status
rostopic echo /experimental/curvature_value
rostopic echo /experimental/curvature_speed_limit
rostopic echo /experimental/curvature_speed_command
rostopic echo /experimental/curvature_steering
rostopic echo /experimental/curvature_progress
rostopic echo /experimental/curvature_goal_reached
```

정상적으로 직선과 곡선을 통과하면 다음 현상이 보여야 한다.

- 직선에서 곡률 절댓값이 작고 곡률 기반 속도 제한이 상대적으로 높다.
- 좌회전·우회전에서 곡률 부호가 바뀌고 조향값도 그 방향에 맞게 바뀐다.
- 고곡률 구간에서 목표 속도가 낮아진다.
- 경로 마지막 점 근처에서 `curvature_goal_reached`가 true가 되고 최종 속도가 0에 수렴한다.

## 11. GPS blackout·센서 이상 처리 확인

현재 `morai_udp_ekf_curvature_only.launch`의 처리 흐름은 다음과 같다.

```text
/gps, /Imu
  → GPS 위경도 변환 /localization/gps
  → GPS jump/median filter, IMU spike/EMA filter
  → ENU EKF /localization/odometry
  → 곡률 기반 Pure Pursuit /control/curvature_only_cmd
  → blackout 안정화 /control/curvature_only_stable_cmd
  → control_mux /ctrl_cmd
  → MORAI UDP
```

### 11-1. Blackout 시나리오 실행

MORAI 시나리오 또는 센서 설정에서 GPS 수신 중단/blackout 조건을 활성화한다. 가능하면 네트워크 전체를 끊지 말고 GPS 센서만 중단한다. 그러면 IMU와 ROS Master는 계속 동작하는 상태에서 blackout 처리를 관찰할 수 있다.

다음 토픽을 관찰한다.

```bash
rostopic echo /localization/gps_health
rostopic echo /stability/curvature_only_gps_status
rostopic hz /localization/odometry
rostopic echo /control/curvature_only_stable_cmd
```

현재 기본 동작은 다음과 같다.

1. `/gps` timeout·status 이상·위치 jump를 `gps_blackout_detector`가 감지한다.
2. GPS update 신뢰도를 낮추고 EKF는 IMU prediction을 유지한다.
3. blackout 동안에도 곡률 경로와 IMU 상태를 사용해 조향을 계속 계산한다.
4. 조향 변화율과 속도 변화율을 제한해 갑작스러운 명령을 막는다.
5. GPS가 5회 연속 유효해지면 정상 상태로 점진적으로 복귀한다.

현재 launch의 주요 설정은 다음과 같다.

```text
gps_blackout_timeout_sec : 0.5 s
gps_recovery_valid_samples: 5회
blackout_speed_mps       : 2.0 m/s
recovering_speed_mps     : 2.0 m/s
stop_on_gps_blackout     : false
```

`stop_on_gps_blackout:=false`는 blackout에서 IMU prediction과 기존 속도 profile을 유지하는 설정이다. fail-safe 정지를 비교하려면 preview에서 다음처럼 별도로 실행한다.

```bash
stop_on_gps_blackout:=true
```

### 11-2. Blackout 통과 기준

- `/localization/gps_health`가 blackout 상태로 바뀐다.
- GPS가 끊겨도 `/localization/odometry`가 계속 발행된다.
- `/control/curvature_only_stable_cmd`의 조향값이 급격히 튀지 않는다.
- `stop_on_gps_blackout:=false`에서는 설정한 blackout 속도 정책이 유지된다.
- GPS가 복구된 뒤 유효 샘플 5회 이후 정상 명령으로 돌아온다.
- 복구 시점에 위치가 한 번에 크게 점프하지 않는다.

## 12. 실제 차량 제어 ON

Preview에서 센서·EKF·곡률·blackout 상태가 모두 정상인 것을 확인한 후, preview launch를 `Ctrl+C`로 종료한다. 곡률 주행 launch만 남아 있는지 확인한 뒤 제어를 켠다.

```bash
roslaunch stability_stack morai_udp_ekf_curvature_only.launch \
  workspace_path:=/root/AutoVehicle/morai_ws \
  path_file:=/root/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_control:=true \
  stop_on_gps_blackout:=false
```

최종 명령 흐름은 다음 한 경로만 사용한다.

```text
curvature PP
  → /control/curvature_only_cmd
  → blackout stability controller
  → /control/curvature_only_stable_cmd
  → control_mux
  → /ctrl_cmd
  → MORAI UDP
```

다음으로 `/ctrl_cmd`의 publisher가 중복되지 않았는지 확인한다.

```bash
rostopic info /ctrl_cmd
```

곡률 주행 launch와 카메라 fallback launch, 기존 Pure Pursuit launch를 동시에 실행하지 않는다. 모두 `/ctrl_cmd` 또는 동일 UDP 포트를 사용하므로 명령이 충돌할 수 있다.

## 13. 실제 센서 noise 측정은 별도 수행

고정 noise를 넣어 주행하는 대신 실제 MORAI 원본 GPS·IMU의 분포를 측정하려면 주행 launch를 종료한 뒤 다음을 실행한다.

```bash
roslaunch sensor_noise_estimator sensor_noise_measurement.launch \
  csv_path:=/root/AutoVehicle/morai_ws/data/measurements/raw_sensor_noise.csv
```

확인 토픽:

```bash
rostopic echo /localization/noise_statistics
rostopic echo /localization/noise_statistics_json
rostopic hz /localization/noise_statistics
```

이 측정 launch는 차량 제어 명령을 발행하지 않는다. 정지선·신호 대기 구간에서 GPS 중심/분산, GPS gap, gyro 정지 평균, IMU 분산을 수집한 뒤 공식 noise 범위와 비교한다.

주행 launch와 동시에 실행하면 GPS·IMU UDP 포트가 중복될 수 있으므로, 측정 전용으로 실행하거나 이미 raw 센서 source가 있는 경우에만 `start_raw_sources:=false`를 사용한다.

## 14. 인위적인 noise filter 성능 시험은 선택 사항

아래 launch는 고정된 noise injector를 포함하는 실험용 구성이다.

```bash
roslaunch stability_stack morai_udp_ekf_curvature_stability.launch \
  workspace_path:=/root/AutoVehicle/morai_ws \
  path_file:=/root/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  enable_sensor_noise:=true \
  enable_control:=false
```

공식 MORAI noise 범위가 확정되기 전에는 실제 주행 결과의 대표값으로 사용하지 않는다. 이 구성은 필터가 인위적인 noise·outlier·dropout을 얼마나 완화하는지 확인하는 A/B 실험용이다.

## 15. 빌드·주행 합격 기준

다음 항목을 모두 기록하면 1차 통합 테스트를 완료한 것으로 본다.

```text
[ ] catkin_make가 오류 없이 완료됨
[ ] rospack find morai_msgs 성공
[ ] rosmsg show CtrlCmd/GPSMessage/EgoVehicleStatus 성공
[ ] /gps, /Imu 수신 주기 정상
[ ] /localization/gps_filtered, /Imu_filtered 발행 정상
[ ] /localization/odometry 발행 정상
[ ] 직선·좌회전·우회전에서 곡률/조향 부호 정상
[ ] 고곡률 구간에서 속도 제한이 동작함
[ ] 마지막 경로점에서 goal reached 및 정지 동작
[ ] GPS blackout 감지 및 IMU prediction 확인
[ ] GPS 복구 후 연속 유효 샘플 기준으로 정상 복귀
[ ] /ctrl_cmd publisher가 중복되지 않음
[ ] 주행 로그·blackout 시간·복구 시간 저장
```

## 16. 대표 오류 해결

### `Could not find a package configuration file ... morai_msgs`

`morai_msgs`가 `/root/AutoVehicle/morai_ws/src`에 없다. MORAI ROS 패키지를 다시 복사하고 `catkin_make`를 다시 실행한다.

### `Package 'stability_stack' not found`

빌드 경로 또는 source 순서가 잘못된 경우가 많다.

```bash
cd /root/AutoVehicle/morai_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack find stability_stack
```

### 빌드가 예전 패키지를 참조함

한 터미널에서 여러 workspace의 `setup.bash`를 source했는지 확인한다. 새 테스트에서는 `/root/AutoVehicle/morai_ws/devel/setup.bash`를 마지막에 source한다. 기존 workspace를 자동으로 source하는 `.bashrc`도 확인한다.

### 센서 토픽이 생성되지 않음

ROS 문제가 아니라 MORAI destination IP·Ubuntu 호스트 네트워크·UDP 포트 문제일 가능성이 높다. `--network host`, destination IP, 방화벽, MORAI 포트 설정을 다시 확인한다.

### `/ctrl_cmd`가 나오지 않음

Preview에서는 `enable_control:=false`이므로 제어 명령이 최종 송신되지 않는다. 실제 주행 때만 `enable_control:=true`로 실행한다. 또한 `morai_host_ip`가 시뮬레이터 PC 주소인지 확인한다.

### 카메라 창이 나타나지 않음

카메라는 이 문서의 곡률·blackout 필수 테스트가 끝난 뒤 별도로 확인한다. GUI가 없는 환경에서는 `show_camera_windows:=false`로 실행하고 `/detection/lane`, `/detection/lane_debug/compressed` 토픽을 확인한다.
