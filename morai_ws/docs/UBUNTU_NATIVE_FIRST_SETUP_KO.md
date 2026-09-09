# 새 Ubuntu PC에서 대회 주행 환경 처음 설치하기

기준: 2026-09-10. **Ubuntu 20.04 Desktop 64비트 + ROS1 Noetic + Python 3.8**, Docker 없이 실행한다.
Ubuntu 설치 후 첫 터미널부터 센서 확인과 주행 시험까지 순서대로 진행하는 문서다.
모든 코드 블록은 **Ubuntu Bash 터미널**에 입력한다. `$HOME`은 로그인한 사용자의 홈이며,
사용자명이 `msclab`이면 `/home/msclab`이다. Windows 경로나 Markdown 링크 문법을 명령에 넣지 않는다.

| 항목 | 이번 구성 |
|---|---|
| Ubuntu 알고리즘 PC | `192.168.0.200` |
| MORAI 시뮬레이터 PC | `192.168.0.161` |
| 주행 코드 | `shinejihun1227/AutoVehicle`의 **`final_ws`** |
| 대회 메시지 | `MORAI-Autonomous/MORAI-ROS_morai_msgs`의 **`beta_drive`** |
| 설치 위치 | `$HOME/AutoVehicle/morai_ws` |
| 정상 주행 최고속도 | **30 km/h**; 곡률·신호·정지 조건에 따라 감속 |
| GPS 음영 차선 주행 상한 | **3 km/h**; 유효한 센서와 제한된 구간에서만 사용 |
| 차량 방향지시등 | 램프 UDP 송신 및 5초 선행 조건 제외 |
| 유지하는 기능 | 대회 경로 추종, 정지선·도로 신호등 인식, 곡률 속도 제어, 차선 기반 음영 대응 |

`AutoVehicle`과 `morai_msgs`는 **서로 다른 Git 저장소**다. 주행 코드에 `beta_drive`를 적용하거나
메시지 저장소를 기본 `main`으로 받지 않는다. `beta_drive`의 `CtrlCmd`는 `steering`,
`EgoVehicleStatus`는 `wheel_angle` 필드를 사용한다.

설치 명령은 단계별로 실행한다. `(...)` 안의 `set -e`는 오류가 나면 해당 단계만 멈추며
바깥 터미널을 종료하지 않는다. 실패하면 출력된 실제 오류를 해결한 뒤 다음 단계로 간다.
이 문서의 셸 문법과 저장소 설정은 검사했지만 새 Ubuntu의 설치·MORAI 주행을 대신 실행한 것은 아니다.

## 1. Ubuntu와 네트워크 준비

OS가 없다면 [Ubuntu 20.04.6 Desktop amd64 이미지](https://releases.ubuntu.com/20.04/)로
Ubuntu 20.04를 먼저 설치한다. 아래는 설치가 끝난 Ubuntu에서 진행한다.

```bash
lsb_release -ds
uname -m
/usr/bin/python3 --version
ip -br addr
ip route
```

각각 Ubuntu 20.04, `x86_64`, Python 3.8 계열을 확인한다. `ROS_IP`를 설정하는 것만으로
PC의 IP가 바뀌지는 않는다. Ubuntu **설정 → 네트워크 → 유선 연결의 설정 → IPv4**에서
실제 네트워크에 맞게 `192.168.0.200`을 할당한다. 공유기에서 주소를 예약하는 방법도 가능하다.
넷마스크·게이트웨이·DNS는 현재 네트워크 값을 사용한다. `/24` 네트워크라면 넷마스크는
`255.255.255.0`이며, 게이트웨이를 무조건 `.1`로 가정하지 않는다.

VirtualBox를 쓴다면 MORAI PC에서 Ubuntu VM의 `.200`으로 UDP를 보낼 수 있도록
브리지 어댑터 등 네트워크를 구성하고 VM 안의 `ip -br addr`로 확인한다.
Ubuntu와 MORAI가 같은 장치의 IP를 중복 사용하면 안 된다. VM의 3D 가속 설정만으로
CUDA 사용 가능 여부를 판단하지 말고 5절의 실제 GPU 검사를 사용한다.

```bash
ping -c 4 192.168.0.161
```

설치 중에는 Ubuntu에서 인터넷에도 접속할 수 있어야 한다. ping 성공은 UDP 수신 확인과 별개이며,
Windows에서 ICMP만 차단된 경우에는 ping이 실패할 수도 있다.

## 2. ROS1 Noetic 설치 — 아직 ROS가 없는 PC

`/opt/ros/noetic/setup.bash`가 이미 있고 `source` 후 `rosversion -d`가 `noetic`이면
ROS 저장소를 다시 추가하지 않고 3절로 간다.

새 PC에서는 공식 Noetic 이미지도 사용하는 **Noetic final 스냅샷 저장소와 서명 키**를 이용한다.
아래는 그 [공식 구성](https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-core/Dockerfile)을
Ubuntu 설치 명령으로 옮긴 것이다. Docker를 설치하는 명령이 아니다.

```bash
(
set -e -o pipefail
test "$(lsb_release -sc)" = focal
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg2 dirmngr software-properties-common
sudo add-apt-repository -y universe

MORAI_GPG_DIR="$(mktemp -d)"
chmod 700 "$MORAI_GPG_DIR"
MORAI_ROS_KEY=4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
gpg --homedir "$MORAI_GPG_DIR" --batch \
  --keyserver hkps://keyserver.ubuntu.com --recv-keys "$MORAI_ROS_KEY"
gpg --homedir "$MORAI_GPG_DIR" --batch --export "$MORAI_ROS_KEY" \
  > "$MORAI_GPG_DIR/ros1-snapshots.gpg"
test -s "$MORAI_GPG_DIR/ros1-snapshots.gpg"
sudo install -m 644 "$MORAI_GPG_DIR/ros1-snapshots.gpg" \
  /usr/share/keyrings/ros1-snapshots-archive-keyring.gpg
echo 'deb [signed-by=/usr/share/keyrings/ros1-snapshots-archive-keyring.gpg] http://snapshots.ros.org/noetic/final/ubuntu focal main' \
  | sudo tee /etc/apt/sources.list.d/ros1-snapshots.list

sudo apt-get update
sudo apt-get install -y ros-noetic-ros-base
source /opt/ros/noetic/setup.bash
rosversion -d
)
```

마지막 출력은 `noetic`이어야 한다. `apt-get update`에서 서명 오류가 나면 설치를 진행하지 않고
키 다운로드·저장소 설정을 확인한다. 기존 ROS 저장소가 있는 PC에 위 설정을 중복 추가하지 않는다.

## 3. 빌드·센서·화면 표시 의존성 설치

```bash
(
set -e
sudo apt-get update
sudo apt-get install -y \
  git build-essential cmake nano \
  python3-dev python3-pip python3-venv python3-catkin-pkg python3-rospkg \
  python3-empy python3-yaml python3-rosdep python3-nose \
  python3-numpy python3-scipy python3-opencv \
  ros-noetic-catkin ros-noetic-rospy ros-noetic-message-generation \
  ros-noetic-message-runtime ros-noetic-tf ros-noetic-tf2-ros \
  ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
  ros-noetic-diagnostic-msgs ros-noetic-visualization-msgs \
  ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-rviz \
  iproute2 iputils-ping tcpdump xvfb xauth libgl1 libglib2.0-0 \
  libsm6 libxext6 libxrender1 libxkbcommon-x11-0 libxcb-xinerama0
)
```

이 문서는 필요한 의존성을 직접 설치하므로 별도의 `rosdep init`이 필수 단계는 아니다.
apt 설치를 `sudo pip`로 대체하지 않는다.

## 4. 홈 폴더에 주행 코드와 대회 메시지 받기

```bash
(
set -e
cd "$HOME"
git clone --branch final_ws --single-branch \
  https://github.com/shinejihun1227/AutoVehicle.git AutoVehicle
git clone --branch beta_drive --single-branch \
  https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git \
  "$HOME/AutoVehicle/morai_ws/src/common/morai_msgs"

git -C "$HOME/AutoVehicle" branch --show-current
git -C "$HOME/AutoVehicle/morai_ws/src/common/morai_msgs" branch --show-current
git -C "$HOME/AutoVehicle/morai_ws/src/common/morai_msgs" rev-parse HEAD
)
```

브랜치 출력은 순서대로 **`final_ws`, `beta_drive`**다. 확인한 메시지 커밋은
`45c6baf148f2327f4c9fefd48262f25bdfe4b567`이며, 대회가 이후 커밋을 별도로 지정하면 그 기준을 따른다.
이미 같은 폴더가 있으면 clone을 반복하지 않는다. 다른 메시지 브랜치로 설치한 PC는
[기존 설치의 beta_drive 전환 절차](FINAL_WS_NATIVE_NO_LAMPS_TEST_KO.md#3-morai_msgs-메시지-소스와-catkin-빌드)를 사용한다.

메시지 준비 관계는 다음과 같다. `.msg` 파일을 하나씩 임의로 만들 필요는 없다.

| ROS 패키지 | 소스가 생기는 위치 |
|---|---|
| `morai_msgs` | 별도 beta_drive clone → `src/common/morai_msgs` |
| `common` | 주행 코드에 포함 → `src/common/common_msgs` |
| `morai_perception_msgs` | 주행 코드에 포함 → `src/common/morai_perception_msgs` |
| `lidar_perception` | 주행 코드에 포함 → `src/detection/lidar_perception` |

## 5. Python 가상환경 생성과 모델 라이브러리 설치

새 PC에서는 아래 가상환경 하나를 빌드와 실행에 함께 사용한다. **생성 명령이 성공한 다음**
`activate`를 source해야 한다. `--system-site-packages`는 apt의 ROS 모듈을 함께 사용하기 위한 옵션이다.

```bash
(
set -e
source /opt/ros/noetic/setup.bash
/usr/bin/python3 -m venv --system-site-packages "$HOME/morai-final-venv"
source "$HOME/morai-final-venv/bin/activate"
python3 -m pip install pip==24.3.1 setuptools==69.5.1 wheel==0.45.1
python3 -m pip install torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r "$HOME/AutoVehicle/morai_ws/docker/final_ws/requirements.txt"
python3 -m pip check
python3 -c 'import sys, rospy, cv2, torch, ultralytics; print(sys.executable); print("PYTHON_IMPORT_OK")'
)
```

성공 기준은 `pip check`의 `No broken requirements found`와 `PYTHON_IMPORT_OK`다.
경로의 `docker/final_ws/requirements.txt`는 공용 Python 패키지 목록일 뿐 Docker를 실행하지 않는다.
설치에 실패하면 `activate` 파일 존재만으로 완료됐다고 판단하지 않는다.

기본 명령은 CPU용이다. NVIDIA GPU가 실제로 사용 가능한 Ubuntu라면 같은 환경에서 다음으로
전환할 수 있다. [PyTorch 공식 2.4.1 설치 조합](https://pytorch.org/get-started/previous-versions/#v241)을 사용한다.

```bash
(
set -e
source "$HOME/morai-final-venv/bin/activate"
nvidia-smi
python3 -m pip install --force-reinstall --no-deps \
  torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu121
python3 -m pip install torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cu121
python3 -m pip check
python3 -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
)
```

CUDA 출력이 `True`여야 GPU를 사용한다. CPU·VM 환경에서 영상 처리가 지연되면 설치가 정상이어도
차선 입력의 최신성 조건을 충족하지 못할 수 있다. 특히 음영 주행의 차선 제어 시간 제한은 기본 0.15초다.

## 6. catkin 빌드와 생성 메시지 확인

```bash
(
set -e
source /opt/ros/noetic/setup.bash
source "$HOME/morai-final-venv/bin/activate"
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
cd "$MORAI_WS"
catkin_make -j2 -l2 --force-cmake -DPYTHON_EXECUTABLE="$(command -v python3)"
source "$MORAI_WS/devel/setup.bash"
python3 "$MORAI_WS/docker/final_ws/check_morai_messages.py"
rospack find morai_bringup
rospack find morai_msgs
rosmsg show morai_msgs/CtrlCmd
python3 -c 'from common.msg import ObjectInfoArray; print("COMMON_MESSAGE_OK")'
)
```

`MORAI_MESSAGES_OK`, `COMMON_MESSAGE_OK`가 나오고 `CtrlCmd`에는 **`steering`**이 있어야 한다.
`front_steer`/`rear_steer`가 보이면 beta_drive 소스·빌드 성공·현재 환경 로딩 경로를 다시 확인한다.
빌드는 `morai_ws`에서 수행하며, 다른 PC의 `build`/`devel`을 복사해서 쓰지 않는다.

## 7. 매 터미널에서 사용할 환경 파일 만들기

앞 단계가 성공한 뒤 한 번 생성한다. 아래 파일은 이번 문서에서 만든 가상환경을 사용한다.

```bash
cat > "$HOME/morai_native_env.sh" <<'EOF'
source /opt/ros/noetic/setup.bash || return 1
source "$HOME/morai-final-venv/bin/activate" || return 1
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
source "$MORAI_WS/devel/setup.bash" || return 1
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=192.168.0.200
unset ROS_HOSTNAME
export YOLO_AUTOINSTALL=false
export QT_X11_NO_MITSHM=1
# 코드 기본값. MORAI의 실제 Cmd Control 수신 포트를 확인하고 맞춘다.
export MORAI_CONTROL_REMOTE_PORT=9093
EOF

source "$HOME/morai_native_env.sh"
python3 -c 'import sys; print(sys.executable)'
```

앞으로 새 Ubuntu 터미널마다 `source "$HOME/morai_native_env.sh"`를 먼저 실행한다.
`ROS_MASTER_URI` 값에 `[http://...](http://...)` 같은 Markdown 문법을 넣지 않는다.
환경 파일의 IP 설정은 Ubuntu 네트워크 설정 자체를 바꾸지 않는다.

최종 폴더는 다음과 같다.

```text
$HOME/
  AutoVehicle/morai_ws/
    src/common/morai_msgs/       # beta_drive 소스
    src/                        # 주행·인식·측위 소스
    data/routes/                # 대회 경로
    data/scenarios/             # 샘플 시나리오
    build/                      # catkin 빌드 결과
    devel/                      # 생성 메시지·실행 환경
  morai-final-venv/              # Python 환경
  morai_native_env.sh            # 매 터미널 source
  morai-native-config/           # 다음 단계에서 만드는 개인 보정값
  morai-native-logs/             # 실행 로그
```

## 8. MORAI 시나리오·센서·차량 제어 설정

MORAI PC에서 대회 지도 **`R_KR_PR_K-city_2025`**를 선택한다. 샘플 시나리오를 쓴다면
Ubuntu 저장소의 `data/scenarios/2026_molit_comp_sample_scene.json`을 MORAI PC에서도 접근 가능한
곳으로 복사하고 **Edit → Scenario → Load Scenario**에서 불러온다.
시나리오 JSON은 초기 장면이며, 주행 경로는 아래 TXT 파일로 별도 지정된다.

```text
$HOME/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt
```

시나리오를 불러온 뒤 센서가 실제 배치되어 있고 UDP 출력이 연결됐는지 확인한다.
[MORAI Network Settings](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/ui)에서
Host IP는 `192.168.0.161`, Destination IP는 `192.168.0.200`으로 맞추고 연결한다.
이번 launch는 **MORAI와 UDP로 통신하고 Ubuntu 내부에서 ROS 토픽을 사용한다.**
별도 rosbridge 서버나 MORAI 측 ROS 토픽 설정을 추가하는 구성은 아니다.

| 기능 | MORAI 측 | Ubuntu 측 | 확인 상태 |
|---|---|---|---|
| Ego Vehicle Status | Host `1910` 송신 | Destination `1911` 수신 | 사용자 확인값 |
| Cam1 차선·정지선 | Host `1100` 예시 | Destination `1101` 수신 | 코드 기준으로 설정 필요 |
| Cam4 신호등·객체 | Host `1130` 예시 | Destination `1131` 수신 | 코드 기준으로 설정 필요 |
| VLP16 LiDAR | Host `2000` | Destination `2001` 수신 | 코드 기본값, 확인 필요 |
| GPS | 센서 설정의 송신 포트 | Destination `3001` 수신 | 코드 기본값, 확인 필요 |
| IMU | 센서 설정의 송신 포트 | Destination `4001` 수신 | 코드 기본값, 확인 필요 |
| Cmd Control | **수신 `9093`** | 송신 소켓 `9094` | **실제 MORAI 포트 미확인** |
| Sensor Sync | Host `9097` | Destination `9098` | 사용자 확인값; 이 launch는 사용하지 않음 |

차량 제어 메시지는 **Ego Ctrl Cmd**에 맞춘다. `9093`은 확인된 대회 고정값이 아니라 코드 기본값이다.
실제 차량 제어 수신 포트가 다르면 `morai_native_env.sh`의 `MORAI_CONTROL_REMOTE_PORT`를 수정하고 다시 source한다.
Ego Status의 `1910/1911`이나 Sensor Sync의 `9097/9098`을 차량 제어 포트 대신 넣지 않는다.
Sensor Sync 포트를 방향지시등 송신에 사용하지 않는다. 램프용 센서 토픽은 만들지 않는다.

센서 종류와 출력 형식도 수신 코드와 맞아야 한다. 이 구성은 카메라 압축영상 UDP,
VLP16 LiDAR UDP, GPS NMEA(`GPRMC`/`GPGGA`), MORAI IMU 바이너리 UDP를 사용한다.
포트만 맞고 메시지 형식이 다르면 패킷이 도착해도 ROS 토픽이 갱신되지 않을 수 있다.

MORAI를 실행 상태로 두고, 실제 주행 시 Ego Controller는 **AV-ExternalCtrl**(이전 명칭 Auto)을 사용한다.
`Status Initialization`은 제어 모드 전환 시 상태를 초기화할지 선택하는 옵션이다.
ON이면 초기화, OFF이면 이전 상태 유지이며, 초기화 진행률이나 출발 대기 표시가 아니다.
[공식 주행 모드 설명](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-18)

## 9. 개인 보정 파일과 오프라인 검사

```bash
source "$HOME/morai_native_env.sh"
mkdir -p "$HOME/morai-native-config" "$HOME/morai-native-logs"
cp -n "$MORAI_WS/src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml" \
  "$HOME/morai-native-config/turn_signal_maneuvers.yaml"
nano "$HOME/morai-native-config/turn_signal_maneuvers.yaml"
```

`signal_camera`에는 Cam4의 **실제 원본 해상도, 수평 FOV, 차량 기준 x/y/z, yaw/pitch**를 넣는다.
지도 신호등이 영상의 해당 신호등에 올바르게 투영되는 것을 확인한 후 `calibrated: true`로 바꾼다.
기본 `0`들은 미입력 표시이며, `false`를 `true`로 바꾸는 것만으로 보정되지 않는다.
기본 `calibrated: false` 상태에서는 교차로 통과가 차단된다. Cam1의 보정도 실제 장착값과 맞아야 하며,
Cam1 값을 Cam4에 복사하지 않는다. [보정 상세](SCENARIO_DRIVING_TEST_KO.md)

```bash
source "$HOME/morai_native_env.sh"
python3 "$MORAI_WS/docker/final_ws/run_regression.py" --workspace "$MORAI_WS"
python3 "$MORAI_WS/docker/final_ws/smoke_models.py" --workspace "$MORAI_WS" --device cpu
rosrun turn_signal_controller inspect_route_signals.py \
  --path-file "$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  --mgeo-dir "$MORAI_WS/src/detection/camera_perception/lane/mgeo/R_KR_PR_K-city_2025"
```

회귀 검사는 `REGRESSION_PASS`, 모델 검사는 `OFFLINE_SMOKE_PASS`를 확인한다. GPU 환경은
모델 검사의 `--device cpu`를 `--device cuda`로 바꾼다. 기본 경로의 신호 구간은 6개
(직진 4·좌회전 1·우회전 1), `unknown_count: 0`이 기대값이다.
이 검사들은 실제 센서 수신·보정 정확도·시뮬레이터 주행 성공을 보장하지 않는다.

## 10. 먼저 제어 OFF로 센서 수신 확인

MORAI에서 Ego 차량을 정지시킨 상태로 센서 출력을 시작한다. **터미널 1**에서 다음을 실행한다.

```bash
source "$HOME/morai_native_env.sh"
roslaunch morai_bringup final_ws_native_no_lamps.launch \
  enable_control:=false \
  morai_host_ip:=192.168.0.161 \
  ego_status_port:=1911 \
  control_remote_port:="$MORAI_CONTROL_REMOTE_PORT" \
  turn_signal_maneuvers_file:="$HOME/morai-native-config/turn_signal_maneuvers.yaml"
```

이 launch가 센서·인식·측위·제어 노드를 함께 시작하며, 필요한 경우 roscore도 시작한다.
별도의 주행 launch나 UDP 수신기를 중복 실행하지 않는다. **제어 OFF에서는 차량 제어 UDP가
송신되지 않으므로 모드가 자동 전환되지 않아도 이상이 아니다.** OFF는 비상정지 명령이 아니다.

**터미널 2**에서 환경을 source하고 수신 주기를 확인한다.

```bash
source "$HOME/morai_native_env.sh"
timeout 5s rostopic hz /Ego_topic
timeout 5s rostopic hz /gps
timeout 5s rostopic hz /Imu
timeout 5s rostopic hz /localization/odometry
timeout 5s rostopic hz /morai/lidar/live_points
timeout 5s rostopic hz /perception/lidar/tracked_obstacles_map
timeout 5s rostopic hz /detection/lane
timeout 5s rostopic echo -n 1 /perception/camera/lane_quality
timeout 5s rostopic echo -n 1 /control/maneuver_status
```

`hz`는 `average rate`가 반복 출력되는지 확인한다. 토픽 이름이 존재하는 것만으로 수신 성공은 아니다.
`timeout`은 5초 뒤 종료하기 위한 것이며 종료 코드 124 자체는 고장이 아니다.
제어 OFF에서는 `nominal_stale`가 발생할 수 있으므로 먼저 원시 센서와 위치 추정, 인식 출력을 확인한다.
LiDAR 원시 토픽이 있어도 추적 결과가 없으면 위치 추정·추적 노드까지 확인한다.

수신이 없으면 MORAI 연결 상태와 목적지 IP/포트를 확인하고 다음으로 실제 UDP 도착을 본다.
필터는 검사할 센서 포트 하나씩 바꿔 쓴다. 아래는 LiDAR `2001` 예시다.

```bash
ss -lunp
sudo timeout 8s tcpdump -ni any -nn -c 5 \
  'udp and src host 192.168.0.161 and dst port 2001'
```

도착이 없으면 네트워크·MORAI 송신 설정을, 도착하지만 토픽이 없으면 포트 바인드·파서·노드를 확인한다.
방화벽이 켜져 있다면 해당 센서 포트와 송신 PC에 한정해 수신을 허용한다.

## 11. 실제 주행 실행 — 최고 30 km/h

센서·위치 추정·보정 확인 후 **터미널 1의 관찰 launch를 Ctrl+C로 종료**한다.
Ego 시작 위치와 방향을 대회 경로에 맞추고 MORAI를 실행 상태로 둔다.
다음 명령 하나가 실제 주행 실행 명령이다. 시작 위치를 이동하거나 시나리오를 다시 로드했다면
위치 추정과 경로 진행 상태도 초기화되도록 주행 launch 전체를 다시 시작한다.

```bash
source "$HOME/morai_native_env.sh"
mkdir -p "$HOME/morai-native-logs"
(
set -o pipefail
roslaunch morai_bringup final_ws_native_no_lamps.launch \
  enable_control:=true \
  morai_host_ip:=192.168.0.161 \
  ego_status_port:=1911 \
  control_remote_port:="$MORAI_CONTROL_REMOTE_PORT" \
  max_speed_kph:=30.0 \
  fallback_speed_cap_kph:=3.0 \
  path_file:="$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  turn_signal_maneuvers_file:="$HOME/morai-native-config/turn_signal_maneuvers.yaml" \
  2>&1 | tee "$HOME/morai-native-logs/drive-$(date +%Y%m%d-%H%M%S).log"
)
```

별도 터미널에서 확인한다.

```bash
source "$HOME/morai_native_env.sh"
timeout 5s rostopic echo -n 1 /ctrl_cmd
timeout 5s rostopic echo -n 1 /control/mux_status
timeout 5s rostopic echo -n 1 /control/maneuver_status
timeout 5s rostopic echo -n 1 /stability/camera_fallback_status
rosparam get /morai_udp_drive_bridge/control_output_enabled
```

송신 활성 값은 `true`여야 한다. `accel: 0`, `brake: 1`은 코드의 정지 명령이다.
`longlCmdType: 1`은 가속·브레이크 제어이므로 `velocity: 0`만으로 고장이라고 판단하지 않는다.
실제 속도는 `/Ego_topic` 속도 벡터의 크기(m/s)에 3.6을 곱해 km/h로 확인한다.
30 km/h는 최고속도이며, 좌·우회전 속도는 공통 곡률·횡가속도 제한과 정지 조건에 따라 낮아진다.

## 12. 주행이 안 될 때 확인 순서

| 출력·증상 | 해석과 다음 확인 |
|---|---|
| `activate: No such file` | 5절의 가상환경 생성 성공 여부. 파일만 임의로 만들지 않는다. |
| `devel/setup.bash` 없음 | 6절 catkin 빌드가 완료됐는지 확인 |
| `front_steer`가 출력됨 | beta_drive 전환·재빌드·환경 재로딩 후 기존 노드 재시작 |
| `roi_lidar_stale` | 유효한 최신 LiDAR 추적 결과를 못 받음. UDP → 원시 토픽 → odometry → 추적 토픽 순서로 확인 |
| `camera_observation_stream_stale` | Cam4 UDP 1131, YOLO 노드·모델 로드·처리 지연 확인 |
| `signal_observation_stale` | 신호등 관측 갱신 여부 확인. 녹색 신호가 없다는 의미와 다름 |
| `odometry_or_route_unavailable` | GPS 3001·IMU 4001, EKF odometry, 경로 진행 출력 확인 |
| `camera_fallback_status_stale` | fallback 노드 상태와 메시지 형식, 입력 갱신 확인 |
| `nominal_stale_or_not_type1` | 제어 ON 여부와 곡률 제어 → 정지선 → fallback 출력 확인 |
| `signal_camera_uncalibrated` | 9절 Cam4 보정과 개인 YAML 전달 여부 확인 |
| `reference_path_*` | 경로 파일·좌표계·조향 경로와 신호 판단 경로 일치 여부 확인 |
| `SAFE_STOP` | `reason`에 여러 입력 문제가 함께 나올 수 있음. 정지 조건을 끄지 말고 입력부터 복구 |

AV-ExternalCtrl로도 전환되지 않으면 주행 launch를 켠 채 아래를 확인한다.

```bash
source "$HOME/morai_native_env.sh"
timeout 5s rosnode ping -c 2 /morai_udp_drive_bridge
rosparam get /morai_udp_drive_bridge
sudo timeout 5s tcpdump -ni any -nn -c 5 'udp and dst host 192.168.0.161'
```

코드 기본값에서는 `.200:9094 → .161:9093`, 제어 패킷 길이 55바이트가 기대된다.
Ubuntu에서 송신이 보이는 것만으로 MORAI 수신·적용이 확인되지는 않는다. Cmd Control의
메시지 종류·UDP 연결·실제 수신 포트와 MORAI 실행/제어 모드를 확인한다.
현재 브리지는 정지 패킷에도 `ctrl_mode=2`를 넣으므로, 센서 문제로 정지하는 것과
모드 전환 자체가 안 되는 문제를 각각 확인한다. `control_output_enabled`는 시작할 때 읽으므로
`rosparam set`만으로 송신을 켜려 하지 말고 `enable_control:=true`로 launch를 재시작한다.

## 13. 시험 순서, 종료, 다음 날 실행

| 시험 | 확인할 동작 |
|---|---|
| 정상 GPS 직진·곡선 | 대회 경로 추종, 최대 30 km/h, 곡률에 따른 감속 |
| 적색·황색·미확인 신호와 정지선 | 정지선 접근 감속·대기 |
| 좌회전 경로와 좌회전 허가 | 신호 허가 후 대회 경로대로 회전 |
| 직진 경로와 좌회전 전용 신호 | 좌회전하지 않고 직진 허가 대기 |
| 우회전 경로와 허가 신호 | 경로대로 회전; 기본 설정은 적색 우회전을 자동 허용하지 않음 |
| GPS 음영 | 정상 상태를 먼저 확보한 뒤 교차로에서 떨어진 구간에서 GPS만 소실시켜 시험 |

음영 제어는 기본 15초/30m 예산과 3 km/h 상한, 유효한 차선·IMU·속도 조건을 사용한다.
GPS와 IMU 공용 브리지를 종료하면 IMU도 끊기므로 GPS 음영 시험이 되지 않는다.
보행자·차선 변경 시험의 잔여 제한은 [알려진 문제](KNOWN_ISSUES_FINAL_WS.md)를 함께 확인한다.

종료할 때 MORAI에서 차량을 먼저 정지시키고 주행 launch를 Ctrl+C로 종료한다.
프로세스 종료 자체를 차량 비상정지로 가정하지 않는다. 다음 날에는 재설치하지 않고
환경 파일을 source한 뒤 센서 확인과 11절 주행 명령을 사용한다.

코드를 업데이트할 때는 주행을 종료한 상태에서 실행한다. 메시지 업데이트는 대회에서 지정한
변경이 있을 때 beta_drive 저장소에서 별도로 수행하고 다시 빌드한다.

```bash
(
set -e
source "$HOME/morai_native_env.sh"
cd "$HOME/AutoVehicle"
git status --short
git pull --ff-only origin final_ws
cd "$MORAI_WS"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE="$(command -v python3)"
source "$MORAI_WS/devel/setup.bash"
python3 "$MORAI_WS/docker/final_ws/check_morai_messages.py"
)
```

로컬 수정 때문에 pull이 실패하면 변경 내용을 보관하고 병합한다. `reset --hard`나
`build/devel`의 무조건 삭제를 설치 절차로 사용하지 않는다.

## 14. SSH로 사용할 경우

Ubuntu에서 SSH 서버를 설치한 뒤 Windows PowerShell에서 접속할 수 있다.

```bash
# Ubuntu에서 한 번
sudo apt-get install -y openssh-server
sudo systemctl enable --now ssh
```

```powershell
# Windows에서: msclab은 실제 Ubuntu 사용자명으로 바꾼다.
ssh msclab@192.168.0.200
```

첫 접속의 호스트 키 지문은 Ubuntu에서 `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`로
표시한 값과 대조한다. 접속 후에는 Ubuntu Bash 명령을 사용한다.
SSH에 화면 표시 환경이 없으면 `source` 후 10절 또는 11절의 `roslaunch` 앞에
`xvfb-run -a -s '-screen 0 1920x1080x24'`를 붙인다. 카메라 창은 가상 화면에서 실행되므로
사용자 모니터에 보이지 않는다. 영상을 직접 보며 처음 점검할 때는 Ubuntu 데스크톱 터미널이 편하다.
