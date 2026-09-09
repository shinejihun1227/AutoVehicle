# Ubuntu 홈 폴더 설치: 방향지시등 없이 final_ws 시험하기

ROS가 아직 없는 새 Ubuntu PC라면 [처음 설치부터 실제 주행까지 한 문서로 정리한 안내](UBUNTU_NATIVE_FIRST_SETUP_KO.md)를 사용한다.
Ubuntu 네트워크, ROS 설치, final_ws/beta_drive clone, 가상환경 생성, 메시지 빌드, 센서 확인 순서가 포함되어 있다.

대상: **Ubuntu 20.04 + ROS1 Noetic**, Docker를 사용하지 않는 Ubuntu 네이티브 실행.
아래 명령은 Windows PowerShell이 아니라 **Ubuntu의 Bash 터미널**에서 실행한다.
Windows에서 SSH로 접속했다면 접속 후 Ubuntu 터미널에 입력한다.

이번 시험에서는 차량 좌·우 방향지시등의 **UDP 송신과 송신 선행 5초 조건**을 제외한다.
도로의 신호등 인식, 대회 경로 방향, 정지선, 곡률 속도 제어, GPS 음영 차선 제어는 유지한다.
방향지시등은 이 코드에서 수신하는 센서 토픽이 아니라 차량 램프를 켜는 출력이다.
`/control/turn_signal_state`는 코드가 발행하는 진단 토픽이며 MORAI에서 받는 센서가 아니다.

`enable_turn_signal:=false`만 주면 기존 융합 제어는 좌·우회전의 5초 조건을
충족하지 못한다. 새 `final_ws_native_no_lamps.launch`는
`test_without_turn_signals:=true`도 함께 적용한다. 일반 `final_ws_bringup.launch`의
시험 옵션 기본값은 `false`이며 기존 방향지시등 동작은 보존한다.

## 1. 홈 디렉터리에 Git clone

ROS Noetic이 이미 설치된 Ubuntu 기준이다. 먼저 확인한다.

```bash
ls /opt/ros/noetic/setup.bash
/usr/bin/python3 --version
sudo apt-get update
sudo apt-get install -y git
cd "$HOME"
git clone --branch final_ws --single-branch \
  https://github.com/shinejihun1227/AutoVehicle.git AutoVehicle
cd "$HOME/AutoVehicle"
git branch --show-current
git log -1 --oneline
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
```

`final_ws`가 출력되어야 한다. 폴더 구조는 다음과 같다.

```text
/home/Ubuntu사용자명/
  AutoVehicle/                       ← Git 저장소
    morai_ws/                        ← catkin workspace
      src/
      config/
      data/
      build/                         ← 빌드 후 생성
      devel/                         ← 빌드 후 생성, ROS 환경과 메시지
  morai-final-venv/                   ← 아래에서 생성하는 Python 환경
  morai-native-config/                ← 개인별 카메라 보정 설정
  morai_native_env.sh                 ← 터미널마다 source할 환경 파일
```

이미 `$HOME/AutoVehicle`이 있으면 위 `git clone`을 반복하지 않는다. 그 폴더에서
`git status`와 `git remote -v`로 저장소·작업 내용을 확인한 뒤, 수정 사항이 없을 때
`git switch final_ws`와 `git pull --ff-only origin final_ws`를 실행한다.
충돌하거나 수정 사항이 있으면 기존 작업을 보관한 뒤 병합하며 `reset --hard`로 지우지 않는다.

## 2. Ubuntu·ROS 의존성과 Python 환경 설치

```bash
sudo apt-get install -y \
  ca-certificates curl build-essential cmake nano \
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

source /opt/ros/noetic/setup.bash
/usr/bin/python3 -m venv --system-site-packages "$HOME/morai-final-venv"
source "$HOME/morai-final-venv/bin/activate"
python -m pip install pip==24.3.1 setuptools==69.5.1 wheel==0.45.1
python -m pip install torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r "$HOME/AutoVehicle/morai_ws/docker/final_ws/requirements.txt"
python -m pip check
```

`docker/final_ws/requirements.txt`는 기존 검증용 Python 버전 목록이다.
이 파일을 설치해도 Docker가 설치되거나 실행되는 것은 아니다.
`--system-site-packages`는 apt로 설치한 ROS Python 모듈을 같은 환경에서 쓰기 위해 필요하다.
기존 venv가 다른 Python 버전이라면 재활용하지 말고 Python 3.8 환경을 사용한다.

위 명령은 CPU 기준이다. CPU 추론이 느려 차선 영상 나이가 0.15초를 넘으면
GPS 음영 주행 조건을 충족하지 못할 수 있다. NVIDIA 드라이버와 GPU가 준비되어 있다면,
같은 venv에서 CPU torch를 다음으로 교체하고 CUDA 확인·모델 검사를 다시 한다.

```bash
nvidia-smi
python -m pip install --force-reinstall --no-deps \
  torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu121
python -m pip install torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cu121
python -m pip check
python -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
```

CUDA 결과가 `True`여야 GPU를 쓸 수 있다. 지원 wheel 조합의 출처는
[PyTorch 이전 버전 설치 안내](https://pytorch.org/get-started/previous-versions/)다.
실제 lane/YOLO 실행은 CUDA 사용 가능 여부를 확인한다.

## 3. morai_msgs 메시지 소스와 catkin 빌드

저장소의 `morai_perception_msgs`, `common` 등은 함께 clone되지만 외부 SDK 메시지인
`morai_msgs`는 별도 준비해야 한다. 기존 대회 SDK에 맞는 ROS1 `morai_msgs`가 있다면
그 패키지를 `$MORAI_WS/src/common/morai_msgs`에 복사한다. 같은 이름의 패키지는
workspace 안에 **한 개만** 둔다. `.msg` 파일은 빌드하면 Python 메시지 클래스로 생성된다.

이 대회는 [공식 ROS1 메시지의 beta_drive 브랜치](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs/tree/beta_drive)를 사용한다.
이전 안내의 브랜치 없는 clone은 잘못된 안내였다. `main`의 `front_steer`와
`front_steer_angle`은 현재 코드가 사용하는 `steering`과 `wheel_angle` 계약과 다르다.
import 성공만으로 호환 여부를 확인할 수 없다. 기존 메시지 폴더가 없다면 다음을 실행한다.

```bash
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
git clone --branch beta_drive --single-branch \
  https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git \
  "$MORAI_WS/src/common/morai_msgs"
git -C "$MORAI_WS/src/common/morai_msgs" rev-parse HEAD
test -f "$MORAI_WS/src/common/morai_msgs/msg/CtrlCmd.msg"
test -f "$MORAI_WS/src/common/morai_msgs/msg/EgoVehicleStatus.msg"
test -f "$MORAI_WS/src/common/morai_msgs/msg/GPSMessage.msg"
```

위 clone은 해당 폴더가 없을 때만 실행한다. 2026-09-10에 확인한 `beta_drive` HEAD는
`45c6baf148f2327f4c9fefd48262f25bdfe4b567`이다. 대회가 별도 커밋을 지정하면 그 커밋을 사용한다.

**이미 기본 브랜치로 clone했다면** 모든 주행 launch를 Ctrl+C로 종료한 뒤 다음을 실행한다.
`AutoVehicle`은 `final_ws`, 그 안의 별도 저장소 `morai_msgs`는 `beta_drive`다.
메시지 저장소에 로컬 수정이 있으면 아래 블록은 변경하지 않고 종료한다. 수정본은
`src` 밖에 보관한 후 진행하며 `reset --hard`로 덮어쓰지 않는다.

```bash
(
set -e
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
MORAI_MSGS_DIR="$MORAI_WS/src/common/morai_msgs"
git -C "$MORAI_MSGS_DIR" status --short
if [ -n "$(git -C "$MORAI_MSGS_DIR" status --porcelain)" ]; then
  echo 'morai_msgs에 로컬 수정이 있습니다. 보관 후 진행하세요.'
  exit 1
fi
git -C "$MORAI_MSGS_DIR" fetch origin refs/heads/beta_drive:refs/remotes/origin/beta_drive
if git -C "$MORAI_MSGS_DIR" show-ref --verify --quiet refs/heads/beta_drive; then
  git -C "$MORAI_MSGS_DIR" switch beta_drive
else
  git -C "$MORAI_MSGS_DIR" switch --track -c beta_drive origin/beta_drive
fi
git -C "$MORAI_MSGS_DIR" merge --ff-only origin/beta_drive
git -C "$MORAI_MSGS_DIR" branch --show-current
)
```

브랜치 전환 성공 후 **메시지를 다시 빌드한다.** 기존 시스템 Python 환경을 사용한다면
venv가 없어도 되지만 빌드와 실행의 Python은 같아야 한다. 아래 블록은 실패 시
그 뒤의 확인을 실행하지 않으며, 바깥 터미널은 종료하지 않는다.

```bash
(
set -e
source /opt/ros/noetic/setup.bash
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
if [ -f "$HOME/morai-final-venv/bin/activate" ]; then
  source "$HOME/morai-final-venv/bin/activate"
fi
cd "$MORAI_WS"
catkin_make -j2 -l2 --force-cmake -DPYTHON_EXECUTABLE="$(command -v python3)"
source "$MORAI_WS/devel/setup.bash"
rospack find morai_msgs
rospack find morai_bringup
rosmsg show morai_msgs/CtrlCmd
rosmsg show morai_msgs/EgoVehicleStatus
rosmsg show morai_perception_msgs/StopLineDetection
python3 docker/final_ws/check_morai_messages.py
python3 -c 'import rospy, cv2, torch, ultralytics; from common.msg import ObjectInfoArray; print("IMPORT_OK")'
)
```

`catkin_make`는 `src`가 아닌 `morai_ws`에서 실행한다. Windows의 `build/devel`이나
이전에 다른 경로에서 빌드한 결과를 복사하지 않는다. 이후 실행도 빌드한 것과 같은
Python 환경에서 한다. 기존 `.bashrc`가 다른 workspace를 source한다면 새 터미널에서 아래 환경을
마지막에 source하고 `rospack find morai_bringup`이 새 clone을 가리키는지 확인한다.
`MORAI_MESSAGES_OK`가 출력되고 `CtrlCmd`에 `steering`, `EgoVehicleStatus`에
`wheel_angle`이 보여야 한다. 여전히 `front_steer`가 보이면 실행 중인 구버전 노드,
다른 workspace의 생성 메시지 또는 빌드 실패를 확인한다. 브랜치 변경 전에 실행한
노드는 재빌드 후 반드시 다시 시작한다. 필드 검사 성공은 센서 수신이나 주행 성공을 의미하지 않는다.

## 4. 매 터미널에서 사용할 환경 파일 만들기

```bash
cat > "$HOME/morai_native_env.sh" <<'EOF'
source /opt/ros/noetic/setup.bash || return 1
if [ -f "$HOME/morai-final-venv/bin/activate" ]; then
  source "$HOME/morai-final-venv/bin/activate" || return 1
fi
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
source "$MORAI_WS/devel/setup.bash" || return 1
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=192.168.0.200
unset ROS_HOSTNAME
export YOLO_AUTOINSTALL=false
export QT_X11_NO_MITSHM=1
EOF
source "$HOME/morai_native_env.sh"
```

앞으로 **새 Ubuntu 터미널마다** `source "$HOME/morai_native_env.sh"`를 먼저 실행한다.
ROS master는 Ubuntu에 둔다. MORAI Windows PC의 `.161`을 `ROS_IP`에 넣지 않는다.

## 5. MORAI 네트워크

Ubuntu PC에 `192.168.0.200`이 실제로 할당되어 있어야 한다.
MORAI PC는 `192.168.0.161`, 센서 전송 대상 PC는 `192.168.0.200`으로 맞춘다.
네트워크 확인:

```bash
ip -br addr
ping -c 4 192.168.0.161
```

ping 응답만으로 UDP 수신까지 검증되는 것은 아니다. Windows에서 ICMP가 차단될 수도 있다.

| 기능 | 현재 사용할 설정 | 코드 연결 |
|---|---|---|
| Ego Vehicle Status | MORAI Host 1910 / Destination 1911 | Ubuntu UDP 1911 수신 → `/Ego_topic` |
| Cam1 차선·정지선 | Destination 1101 | Ubuntu UDP 1101 수신 |
| Cam4 YOLO 신호등·객체 | Destination 1131 | Ubuntu UDP 1131 수신 |
| VLP16 LiDAR | Destination 2001 | Ubuntu UDP 2001 수신 |
| GPS | Destination 3001 | Ubuntu UDP 3001 수신 → `/gps` |
| IMU | Destination 4001 | Ubuntu UDP 4001 수신 → `/Imu` |
| 차량 제어 Cmd Control | 코드 기본값 MORAI 수신 9093, Ubuntu 송신 소켓 9094 | `/ctrl_cmd` → MORAI `.161:9093` |
| Sensor Sync | 사용자 설정 Host 9097 / Destination 9098 유지 | 이번 launch에서 사용하지 않음 |
| 차량 방향지시등 | 별도 포트/토픽 설정 불필요 | 송신 소켓을 생성하지 않음 |

Ego와 Sensor Sync 포트는 사용자가 확인한 값이다. 나머지는 코드 기본값이므로
MORAI 설정 화면과 맞춘다. 특히 **차량 제어의 MORAI 수신 포트가 9093인지** 확인한다.
다르면 실행 시 `control_remote_port:=실제수신포트`를 추가한다.
`1910`은 MORAI 측 Ego 포트이므로 Ubuntu 수신 포트로 넣지 않는다.

Sensor Sync의 9097을 방향지시등 송신 대상으로 사용하지 않는다.
MORAI 문서에서 Sensor Sync Data는 `SaveSensorData` 센서 저장 명령으로 구분되어 있으며,
방향지시등 기능과 별개다: [MORAI ROS 인터페이스 설명](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R1.0/ros-1).
이 코드의 카메라·GPS·IMU 처리를 위해 Sensor Sync의 ROS 토픽을 따로 만들 필요는 없다.

관찰 launch를 실행한 상태에서 수신 문제를 조사할 때:

```bash
ss -lunp
sudo tcpdump -ni any -c 30 \
  'src host 192.168.0.161 and udp and (dst port 1911 or dst port 1101 or dst port 1131 or dst port 2001 or dst port 3001 or dst port 4001)'
```

동일 UDP 포트에 기존 ROS 노드나 별도 파서를 중복 실행하지 않는다.
방화벽이 켜져 있다면 실제 센서 포트에 대한 `.161`에서의 수신을 허용한다.

## 6. 차량을 움직이지 않는 오프라인 검사

```bash
source "$HOME/morai_native_env.sh"
python "$MORAI_WS/docker/final_ws/run_regression.py" --workspace "$MORAI_WS"
python "$MORAI_WS/docker/final_ws/smoke_models.py" --workspace "$MORAI_WS" --device cpu
rosrun turn_signal_controller inspect_route_signals.py \
  --path-file "$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  --mgeo-dir "$MORAI_WS/src/detection/camera_perception/lane/mgeo/R_KR_PR_K-city_2025"
```

GPU 환경의 모델 검사에는 `--device cuda`를 사용한다.
첫 명령은 센서·ROS 경계를 mock한 알고리즘 회귀 시험이며 실제 카메라 추론을 하지 않는다.
두 번째는 실제 모델 로드·빈 영상 추론과 생성된 ROS 메시지를 검사한다.
모델 3개는 저장소의 `models/yolov8n.pt`, `models/best0902.pt`, `lane/lane_seg_best.pt`다.
세 번째는 기본 대회 경로의 신호 구간 6개(직진 4, 좌회전 1, 우회전 1),
`unknown_count: 0`, `unverified_stopline_count: 0`이 기대값이다.

특정 알고리즘만 다시 검사하려면 `--suite stopline`, `--suite turn`,
`--suite blackout`, `--suite curvature`, `--suite camera` 중 하나를 첫 명령에 붙인다.

## 7. 제어 OFF로 센서·인식 관찰

MORAI에서 차량을 정지시키고 해당 대회 경로가 있는 지도를 선택한다.
GPS·IMU·Ego·카메라·LiDAR UDP 출력을 시작한다. 이전 주행 launch는 종료한다.

**터미널 1 — Ubuntu 데스크톱에서 창을 보며 실행:**

```bash
source "$HOME/morai_native_env.sh"
roslaunch morai_bringup final_ws_native_no_lamps.launch enable_control:=false
```

**SSH로 접속해 `DISPLAY`가 없는 경우**, 위 roslaunch 대신 다음을 사용한다.
가상 디스플레이에서 동작하므로 영상 창이 사용자 화면에는 보이지 않는다.

```bash
source "$HOME/morai_native_env.sh"
xvfb-run -a -s '-screen 0 1920x1080x24' \
  roslaunch morai_bringup final_ws_native_no_lamps.launch enable_control:=false
```

둘 중 하나만 실행한다. roslaunch가 roscore를 자동 시작하므로 별도의 roscore는 필요 없다.
`enable_control:=false`는 이 launch의 차량 제어 UDP를 끄는 설정이지 비상정지 명령이 아니다.

**터미널 2 — 수신 및 진단 확인:**

```bash
source "$HOME/morai_native_env.sh"
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /localization/sensor_quality
rostopic echo -n 1 /detection/lane
rostopic echo -n 1 /perception/camera/lane_quality
rostopic echo -n 1 /perception/camera/stopline
rostopic echo -n 1 /perception/traffic_light/directional_state
rostopic echo -n 1 /control/maneuver_status
rostopic echo -n 1 /stability/camera_fallback_status
rostopic hz /detection/lane
```

`rostopic hz`는 Ctrl+C로 종료한다. 수신이 없는 `rostopic echo`도 Ctrl+C로 빠져나온다.
제어 OFF에서 주행 명령이 없어 `nominal_stale`가 나올 수 있으므로 이때는
통과 허가보다 각 센서의 값·시각·영상 품질을 먼저 본다.

새 모드의 적용 여부는 다음 파라미터로도 확인한다.

```bash
rosparam get /final_ws_maneuver_fusion/test_without_turn_signals
rosparam get /final_ws_maneuver_fusion/lamp_output_enabled
```

각각 `true`, `false`가 기대값이다. 제어가 계산되는 `/control/maneuver_status`에서는
`test_without_turn_signals: true`, `indicator_lead_required: false`,
`lamp_udp_enabled: false`, `lamp_transmit_ok: false`로 표시된다.
`/control/turn_signal_state`의 `OFF`는 정상이다. 램프 송신 성공을 가짜로 보고하지 않는다.

## 8. Cam4 보정 설정

기본 설정의 `signal_camera.calibrated`는 **false**다. 이 상태에서는 방향지시등 조건을
제외해도 교차로 통과가 차단된다. 포트나 방향지시등 문제로 오해하지 않는다.

```bash
mkdir -p "$HOME/morai-native-config"
cp -n "$MORAI_WS/src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml" \
  "$HOME/morai-native-config/turn_signal_maneuvers.yaml"
nano "$HOME/morai-native-config/turn_signal_maneuvers.yaml"
```

Cam4의 실제 원본 해상도, 수평 FOV, 차량 기준 위치 x/y/z, yaw/pitch를 입력하고
지도상의 자기 차로 신호등이 영상의 해당 신호등에 투영되는지 확인한 뒤
`calibrated: true`로 바꾼다. **false를 true로 바꾸는 것만으로 보정이 되지 않는다.**
이 모듈은 차량 좌표 x 전방/y 좌측/z 위, yaw 좌향 양수, pitch 광축 상향 양수다.
MORAI 표시 각도의 부호와 기준점이 같은지 확인한다.
Cam1~3의 `lane/cam_set.json`을 Cam4 장착값으로 복사하지 않는다.
차선 BEV용 Cam1 보정도 실제 센서와 맞아야 한다.

저장 후 launch를 재시작할 때 다음 인자를 추가하면 개인 설정을 읽는다.

```text
turn_signal_maneuvers_file:=$HOME/morai-native-config/turn_signal_maneuvers.yaml
```

파일명에 `turn_signal`이 남아 있어도 이 YAML에는 **경로와 도로 신호등의 연결·보정**이
포함되어 있다. 방향지시등을 제외한다는 이유로 해당 패키지나 YAML 전체를 삭제하면 안 된다.

## 9. MORAI 주행 시험 명령과 확인 항목

관찰 launch를 Ctrl+C로 종료한 뒤, MORAI 시작 위치·차량 방향을 대회 경로에 맞추고
외부 UDP 차량 제어 모드를 선택한다. Cam4와 좌표 보정, 오프라인 검사 결과를 확인한다.
아래는 **MORAI 시뮬레이터 시험 명령**이다. 이 변경은
[기존 미해결 사항](KNOWN_ISSUES_FINAL_WS.md)의 보행자 입력 소실 및 LiDAR 차선 변경
정보 지연 문제를 수정하지 않는다. 보행자·차선 변경 시험에는 해당 잔여 문제를 별도로 확인해야 한다.

**터미널 1:**

```bash
source "$HOME/morai_native_env.sh"
roslaunch morai_bringup final_ws_native_no_lamps.launch \
  enable_control:=true \
  turn_signal_maneuvers_file:="$HOME/morai-native-config/turn_signal_maneuvers.yaml"
```

SSH라면 7절처럼 `roslaunch` 앞에 `xvfb-run -a -s '-screen 0 1920x1080x24'`를 붙인다.
이 launch가 필요한 인식·측위·제어 노드를 함께 시작하므로 개별 실행파일을 중복 실행하지 않는다.
자동 적용값은 MORAI `.161`, Ego 수신 `1911`, 램프 제외, 정상 주행 최고속도 `30km/h`,
GPS 음영 차선 주행 상한 `3km/h`, 공통 횡가속도 한계 `1m/s²`다.

**터미널 2에서 진단을 보며 한 항목씩 시험한다:**

| 시험 | 기대 동작 / 진단 |
|---|---|
| 정상 GPS 직진·곡선 | 대회 경로 추종, `reference_path_match: true`; 조향 진동·경로 오차 확인 |
| 정지선 + 적색/황색/미확인 | 접근 감속, `permission: false`, 정지선 앞 대기 |
| 좌회전 경로 + 좌회전 허가 신호 | 신호 연속 확인 후 경로대로 좌회전; 방향지시등 5초 대기는 제외 |
| 우회전 경로 + 우회전 허가 신호 | 경로대로 우회전; 기본 정책은 일반 녹색 우회전 허용, 적색 자동 우회전 금지 |
| 직진 경로 + 좌회전 전용 신호 | 좌회전하지 않고 직진 허가를 기다림 |
| 좌/우회전 경로 + 다른 방향 전용 신호 | 경로를 바꾸지 않고 자기 방향 허가를 기다림 |
| GPS 음영, 유효한 차선·IMU·속도 | 교차로에서 떨어진 검증된 구간에서 제한적인 차선 유지 |
| 음영 중 차선 소실·저품질, IMU/속도 소실 | 가속 해제·정지 |
| GPS 복구 | 정상 상태를 1초 연속 확인한 뒤 경로 제어로 복귀 |

`route_direction`은 대회 경로 방향, `signal_allowed_directions`는 현재 신호의 허용 방향,
`route_signal_compatible`은 둘의 일치, `permission`은 최종 진입 허가다.
시험 모드에서 `indicator_ready: true`는 지시등 선행 조건이 면제됐다는 뜻이다.
그 값만으로 신호나 경로 검사를 통과한 것은 아니다.

회전 속도는 좌회전 15/우회전 10 같은 고정값이 아니다.
공통 `3.6 * sqrt(lateral_accel_limit_mps2 / abs(curvature))` km/h와
전체 `max_speed_kph`, 접근·정지 제한 중 더 낮은 값을 적용한다.
`30km/h`는 정상 주행의 최고속도이며 항상 30km/h로 달린다는 뜻은 아니다.
곡률이 크거나 정지선·신호 조건에 걸리면 더 낮은 속도를 적용한다.
계산값은 `turn_curve_speed_kph`, 적용 상한은 `turn_speed_limit_kph`를 확인한다.
속도 상한을 바꾸면 같은 launch의 `max_speed_kph:=값`을 사용한다.
`/ctrl_cmd`는 accel/brake 방식이므로 `velocity: 0`만 보고 고장으로 판단하지 않는다.
실제 속도는 `/Ego_topic`의 속도 벡터 크기(m/s)에 3.6을 곱해 확인한다.

GPS 음영 시험은 먼저 정상 GPS로 위치와 차선 상태를 확보한 다음 수행한다.
교차로/차선 변경 예정 구간에서 충분히 떨어진 곳(기본 경계 여유 40m)에서 MORAI의
GPS 음영을 적용하고 **GPS만** 소실되는지 확인한다. GPS/IMU 공용 브리지를 종료하면
IMU도 끊겨 차선 유지가 아닌 정지가 정상이다. 차선은 기본 품질 0.80 이상,
서로 다른 5개 영상·최소 0.20초 안정 상태·영상 나이 0.15초 이내가 요구된다.
음영 유지 한도는 15초 또는 30m 중 먼저 도달한 조건이며, 3km/h에서는 시간 한도가
먼저 걸릴 수 있다. 음영에서 새 교차로 진입·회전·차선 변경을 허용하는 모드는 아니다.

**터미널 3 — 로그 저장:**

```bash
source "$HOME/morai_native_env.sh"
mkdir -p "$MORAI_WS/logs"
rosbag record --split --size=1024 -O "$MORAI_WS/logs/native_no_lamps" \
  /gps /Imu /Ego_topic /localization/odometry /localization/sensor_quality \
  /detection/lane /perception/camera/lane_quality /stability/camera_fallback_status \
  /perception/camera/stopline /detection/traffic_light \
  /perception/traffic_light/directional_state /detection/fused_safety_stop \
  /control/maneuver_status /control/turn_signal_state /ctrl_cmd
```

종료할 때는 MORAI에서 차량을 먼저 정지시키고 launch·rosbag을 Ctrl+C로 종료한다.
launch 종료 자체를 비상정지로 가정하지 않는다.

## 10. 다음 실행·업데이트와 자주 만나는 문제

경로 시작점에서 목표속도가 계속 0으로 남는 문제를 수정했다.
`initial_speed_kph: 0`은 이제 시작 위치의 속도를 영구적으로 0에 고정하지 않는다.
출발 명령은 0에서 시작해 `max_accel_mps2`에 따라 시간 기준으로 증가하며,
곡률·종점 제동 상한과 하위 신호/정지선 검사는 계속 적용된다.
이 수정으로 모든 정지 원인이 해결되는 것은 아니므로, 실행되지만 움직이지 않으면
주행 launch를 켠 상태에서 새 터미널로 다음 출력을 확인한다.

```bash
source "$HOME/morai_native_env.sh"
timeout 4s rostopic echo -n 1 /control/mux_status
timeout 4s rostopic echo -n 1 /control/maneuver_status
timeout 4s rostopic echo -n 1 /ctrl_cmd
timeout 4s rostopic echo -n 1 /experimental/curvature_progress
timeout 4s rostopic echo -n 1 /experimental/curvature_speed_command
rosparam get /morai_udp_drive_bridge/control_output_enabled
rosparam get /morai_udp_drive_bridge/control_remote_port
```

`/ctrl_cmd`의 `accel: 0`, `brake: 1`이면 소프트웨어 정지 명령이므로
`mux_status.reasons`, `maneuver_status.reason`과 `signal_selection_reason`을 확인한다.
`accel > 0`, `brake: 0`인데 차가 움직이지 않으면 차량 제어 수신 포트,
외부 제어 모드, UDP 도착 여부를 확인한다. ROS 토픽 발행만으로 MORAI 수신이 확인되지는 않는다.
`velocity: 0`은 accel/brake 제어 방식에서는 정상이다.

다음 날에는 설치를 반복하지 않고 환경 파일을 source한 뒤 관찰/주행 launch를 실행한다.
코드 업데이트는 주행을 종료하고 수행한다.

```bash
source "$HOME/morai_native_env.sh"
cd "$HOME/AutoVehicle"
git status --short
git pull --ff-only origin final_ws
cd "$MORAI_WS"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE="$(command -v python3)"
source "$MORAI_WS/devel/setup.bash"
python3 "$MORAI_WS/docker/final_ws/check_morai_messages.py"
```

| 증상 | 확인할 부분 |
|---|---|
| 새 launch를 못 찾음 | `git branch --show-current`, `git pull`, 재빌드, `rospack find morai_bringup` 경로 |
| `No module named morai_msgs` | 외부 메시지 소스, catkin 성공 여부, devel source |
| `/Ego_topic`이 없음 | Ubuntu 수신 1911, MORAI Destination IP `.200`, Host 포트 1910과 혼동 여부 |
| `Address already in use` | `ss -lunp`로 기존 UDP 수신기 확인; 중복 launch 종료 |
| 창/Qt/`DISPLAY` 오류 | Ubuntu 데스크톱에서 실행하거나 SSH에서는 Xvfb 사용 |
| `nominal_stale` | 제어 OFF 관찰 중인지, 제어 ON이면 경로/주행 노드가 정상인지 |
| `signal_camera_uncalibrated` | Cam4 보정 후 개인 YAML 경로 전달 여부 |
| `reference_path_*` 오류 | 조향과 fusion이 같은 경로 파일/좌표계를 사용하는지 |
| 방향지시등 때문에 회전 대기 | 새 launch 사용 여부; 시험 파라미터 true, lamp 출력 false 확인 |
| 차선은 보이지만 음영에서 정지 | 영상 나이·품질·7~14m 관측 범위·IMU·Ego·교차로 거리·유지 시간 확인 |

로컬 검증은 ROS/UDP 경계를 대체한 회귀 시험과 launch 정적 검사다.
실제 Ubuntu catkin 빌드, GPU 추론 지연, MORAI 센서 수신과 주행 결과는 Ubuntu에서
위 절차로 확인해야 한다.
