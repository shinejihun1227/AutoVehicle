# final_ws: 새 Docker 구성부터 MORAI 주행 시험까지

작성: 2026-09-08. 대상 저장소는 `shinejihun1227/AutoVehicle`, 브랜치는 `final_ws`다.

2026-09-09 보완: 최신 재빌드·회귀 시험·음영구간 실행 명령은
[보완본 Docker 빠른 실행](FINAL_WS_UPDATED_DOCKER_TEST_KO.md)을 사용한다.

**현재 상태:** 작업 Windows PC에는 실행 가능한 Docker/WSL 배포판이 없어 이미지 빌드·컨테이너 실행·MORAI 주행을 수행하지 못했다. 아래는 저장소 코드와 공식 설치 문서를 대조해 만든 실행 절차이며, 현장에서 검증할 구성이다. 원격 clone의 커밋은 `git log -1`로 확인한다.

**현재 코드는 주행 승인본이 아니다.** 최초 리뷰 5건 중 수정된 항목과 남은 보행자 입력 소실·LiDAR 간격 데이터 지연 문제는 [현재 문제 상태](KNOWN_ISSUES_FINAL_WS.md)를 참조한다. Cam4 보정도 필요하다. 빌드, 오프라인 검사, **제어 OFF 센서 관찰**을 먼저 수행한다.

## 1. 사용할 환경과 파일

기존 문서의 별도 Ubuntu 알고리즘 PC를 기준으로 한다. Windows MORAI PC와 같은 유선망의 **Ubuntu 22.04/24.04 x86_64 + Docker Engine**을 가정한다. Docker 안은 프로젝트 설정이 없는 공식 `ros:noetic-ros-base-focal`에서 시작해 Ubuntu 20.04, ROS1 Noetic, Python 3.8로 구성한다. MORAI SIM 자체는 Windows에서 실행하며 이 이미지에 설치하지 않는다.

| 구성 | 경로/역할 |
|---|---|
| Dockerfile | `morai_ws/docker/final_ws/Dockerfile`: OS 의존성, Python 환경, MORAI 메시지, catkin 빌드 |
| Python 버전 제약 | `docker/final_ws/requirements.txt` |
| 환경 진입 | `docker/final_ws/entrypoint.sh`: ROS·venv·devel source; 주행 자동 실행 없음 |
| 모델 점검 | `docker/final_ws/smoke_models.py`: ROS 메시지 import와 차선·YOLO 모델 로드/한 번 추론 |
| 이미지 내 workspace | `/opt/AutoVehicle/morai_ws` |
| 최종 launch | `morai_bringup final_ws_bringup.launch` |
| 시나리오 | `data/scenarios/2026_molit_comp_sample_scene.json` |
| 경로 | `data/routes/2026_molit_comp_global_path.txt` |

이 최종 launch는 UDP를 직접 수신/송신한다. ROS 네이티브 모드나 rosbridge를 별도로 동시에 켜지 않는다. 기존 `codex/curvature-only-drive` 문서는 곡률 단독 실험용이며 카메라·LiDAR·신호 융합의 최종 실행 절차가 아니다.

ROS Noetic은 2025-05-31 지원이 종료되었다. 기존 ROS1 코드 호환용 시험 환경이며 최신 보안 지원 환경이라고 볼 수 없다. 외부 인터넷에 ROS master를 공개하지 말고, 검증한 이미지·패키지 목록을 보관한다. [ROS 공식 종료 안내](https://discourse.ros.org/t/new-packages-for-noetic-2025-05-29-final/44003)

## 2. Ubuntu 호스트 준비

이후 **호스트** 명령은 Ubuntu Bash에서 실행한다. Windows PowerShell에 붙여 넣지 않는다.

```bash
uname -m
lsb_release -ds
ip -br -4 address
sudo docker version
sudo docker ps -a
```

`x86_64`와 Docker의 Client/Server가 확인되어야 한다. 기존 컨테이너는 삭제하지 않는다. Docker가 없으면 [Docker Engine Ubuntu 공식 설치 절차](https://docs.docker.com/engine/install/ubuntu/)의 apt 저장소 설치 방식으로 설치한 뒤 `sudo docker run --rm hello-world`로 확인한다. 기존 Docker가 정상이라면 재설치할 필요가 없다. 이 절차는 Docker 소켓을 컨테이너에 마운트하지 않으며 `--privileged`도 사용하지 않는다.

네트워크 예시를 실제 PC 주소로 바꾼다.

```text
MORAI Windows PC : 192.168.0.151   (명령의 목적지)
Ubuntu host PC   : 192.168.0.200   (센서의 목적지)
```

Linux Engine의 `--network host`를 사용한다. `-p` 포트 매핑은 추가하지 않는다. 이 방식은 호스트와 네트워크를 공유하므로 같은 UDP 수신 포트를 쓰는 다른 주행 프로그램을 동시에 실행할 수 없다. [Docker host 네트워크 설명](https://docs.docker.com/engine/network/drivers/host/)

Windows Docker Desktop은 4.34 이상에서 host networking을 별도 활성화할 수 있지만 Linux Engine과 동일한 설치 절차는 아니다. 현재 PC에는 Desktop/WSL이 없으며, 설치와 재부팅·센서 UDP 연결 확인이 추가로 필요하다. 이 문서는 그 환경을 실행 검증한 것으로 취급하지 않는다.

## 3. GitHub에서 별도 clone 받기

**선행 조건:** 이번 코드의 `final_ws` push가 완료되었는지 확인한다. 기존 작업 디렉터리에 덮어쓰지 않고 새 clone을 만든다. 아래 디렉터리가 이미 있으면 다른 이름을 사용한다.

```bash
mkdir -p "$HOME/morai-final-test"
cd "$HOME/morai-final-test"
git clone --branch final_ws --single-branch \
  https://github.com/shinejihun1227/AutoVehicle.git AutoVehicle
cd AutoVehicle
git status --short
git log -1 --oneline
test -f morai_ws/docker/final_ws/Dockerfile
test -f morai_ws/src/control/turn_signal_controller/scripts/maneuver_fusion_node.py
export CODE_REF="$(git rev-parse HEAD)"
```

`test`가 실패하면 구버전 clone이거나 Docker 문서까지 아직 push되지 않은 상태다. 진행하지 않는다. 인증이 필요한 저장소에서는 본인 Git 인증을 사용하고 토큰을 Dockerfile/URL/문서에 넣지 않는다.

기본 모델은 이미 Git으로 추적 중이다. 아래 크기는 이번 로컬 확인값이며 100MB 미만이다. 작은 텍스트 파일이면 모델 본체가 아닌 포인터일 수 있으므로 진행하지 않는다.

| 파일 (`morai_ws/src/detection/camera_perception/` 기준) | 바이트 |
|---|---:|
| `lane/lane_seg_best.pt` | 97,898,559 |
| `models/best0902.pt` | 22,608,810 |
| `models/yolov8n.pt` | 6,549,796 |

```bash
wc -c morai_ws/src/detection/camera_perception/lane/lane_seg_best.pt \
  morai_ws/src/detection/camera_perception/models/best0902.pt \
  morai_ws/src/detection/camera_perception/models/yolov8n.pt
```

## 4. MORAI 메시지 버전 선택과 이미지 빌드

`morai_msgs`는 이 저장소에 없다. Dockerfile은 [MORAI 공식 ROS1 메시지 저장소](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs)를 별도로 가져온다. ROS2 저장소가 아니다. 공식 저장소는 Ubuntu 20.04/Noetic 시험 이력을 명시한다. **해당 대회 SDK와 메시지 정의의 일치는 별도 확인 사항**이다.

대회 제공 SDK의 대응 커밋을 알고 있으면 그 40자리 SHA를 사용한다. 모르면 우선 공식 ROS1 main의 SHA를 기록해 빌드 후보로 사용할 수 있다. 이 선택 자체가 대회 버전 호환 승인은 아니다.

```bash
# Ubuntu 호스트, AutoVehicle 저장소 루트에서
export MORAI_MSGS_REF="$(git ls-remote \
  https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git \
  refs/heads/main | awk '{print $1}')"
[[ "$MORAI_MSGS_REF" =~ ^[0-9a-f]{40}$ ]] || { echo '메시지 SHA 확인 실패'; exit 1; }
printf 'code=%s\nmorai_msgs=%s\n' "$CODE_REF" "$MORAI_MSGS_REF"

sudo docker build --progress=plain \
  --build-arg CODE_REVISION="$CODE_REF" \
  --build-arg MORAI_MSGS_REF="$MORAI_MSGS_REF" \
  --build-arg TORCH_FLAVOR=cpu \
  -f morai_ws/docker/final_ws/Dockerfile \
  -t morai-final-ws:cpu morai_ws
```

Docker build context는 마지막 인자의 **`morai_ws`**다. 저장소 루트 전체를 보내지 않는다. 이미지에 프로젝트·지도·모델이 포함되며 루트의 보고서/발표자료는 포함하지 않는다. Dockerfile은 정리된 새 빌드 디렉터리에서 `catkin_make -j2 -l2`를 수행한다. 소스는 `/opt/AutoVehicle/morai_ws`에 복사된다.

Python 환경은 `/opt/morai-venv --system-site-packages`다. ROS apt 모듈을 유지하면서 NumPy 1.24.4, SciPy 1.10.1, GUI OpenCV 4.10.0.84, Torch 2.4.1/Torchvision 0.19.1을 설치한다. `best0902.pt`의 저장 메타데이터가 **Ultralytics 8.4.138**이므로 같은 버전으로 고정했다. 공식 메타데이터도 Python >=3.8을 지원한다. [Ultralytics 해당 버전 의존성](https://github.com/ultralytics/ultralytics/blob/v8.4.138/pyproject.toml), [PyTorch 이전 버전 설치](https://pytorch.org/get-started/previous-versions/#v241)

이 파일은 주요 버전 제약이지 모든 전이 의존성/apt 패키지까지 고정한 완전한 lockfile이 아니다. 성공한 이미지 안에는 다음 기록이 생성된다.

```text
/opt/morai-build/code-revision.txt
/opt/morai-build/morai-msgs-revision.txt
/opt/morai-build/pip-freeze.txt
/opt/morai-build/apt-packages.txt
```

빌드 실패 시 마지막 오류를 해결하고 다시 빌드한다. apt 서명 오류를 `--allow-unauthenticated`로 우회하지 않는다. 오래된 Noetic 이미지/ROS 저장소 키 문제면 공식 키 갱신 방법과 이미지 태그를 확인한다. `morai_msgs` 필드 불일치는 대회 SDK 정의를 확인한다. 모델 오류를 해결하려고 주행 중 자동 다운로드/설치를 허용하지 않는다.

### NVIDIA GPU 선택 사항

CPU 이미지는 설치·오프라인 검증용 기준이며 실시간 처리 성능을 보장하지 않는다. 센서 입력은 0.5초, 신호 입력은 0.8초 기본 제한이 있어 느린 추론은 주행 차단을 유발할 수 있다. GPU가 있다면 호스트에서 `nvidia-smi`와 [NVIDIA Container Toolkit 공식 설치/설정](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)을 먼저 완료한다. 드라이버 변경은 다른 작업에 영향을 줄 수 있으므로 별도로 확인한다.

```bash
sudo docker build --progress=plain \
  --build-arg CODE_REVISION="$CODE_REF" \
  --build-arg MORAI_MSGS_REF="$MORAI_MSGS_REF" \
  --build-arg TORCH_FLAVOR=cu121 \
  -f morai_ws/docker/final_ws/Dockerfile \
  -t morai-final-ws:cu121 morai_ws
```

아래 컨테이너 생성 시 이미지명을 `morai-final-ws:cu121`로 바꾸고 `--gpus all`을 추가한다. GPU 확인이 실패하면 CPU로 조용히 대체해 주행하지 말고 드라이버·CUDA 호환성을 확인한다.

## 5. 새 컨테이너 생성: 자동 주행 없음

**호스트**, 같은 저장소 루트에서 실행한다. 설정과 로그는 호스트에 별도 보존한다.

```bash
mkdir -p "$HOME/morai-final-test/settings" "$HOME/morai-final-test/logs"
cp -n morai_ws/src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml \
  "$HOME/morai-final-test/settings/turn_signal_maneuvers.yaml"

sudo docker run -dit --name morai_final_clean \
  --network host --shm-size=1g \
  -v "$HOME/morai-final-test/settings:/opt/morai-config:ro" \
  -v "$HOME/morai-final-test/logs:/logs" \
  morai-final-ws:cpu bash

sudo docker ps --filter name=morai_final_clean
sudo docker exec -it morai_final_clean /usr/local/bin/morai-entrypoint bash
```

`morai_final_clean` 이름이 이미 사용 중이면 다른 이름으로 새로 만든다. 기존 컨테이너를 삭제하는 명령은 필요 없다. 이 이미지는 root로 동작하므로 Ego UDP 수신 포트 909를 바인드할 수 있다. 호스트 로그 파일이 root 소유로 생길 수 있다. 컨테이너 전체 workspace를 호스트 폴더로 덮어 마운트하면 이미지의 `devel`과 메시지 빌드를 가리므로 위 두 디렉터리만 마운트한다.

`docker exec ... bash`만 실행하면 entrypoint가 자동 재실행되지 않는다. 새 터미널은 항상 위와 같이 **`/usr/local/bin/morai-entrypoint bash`**로 연다.

### 카메라 창을 직접 보는 경우: Ubuntu X11

Xvfb는 창을 가상 화면에만 띄운다. 실제 영상 투영/보정 검증에 눈으로 볼 창이 필요하면 Ubuntu 데스크톱의 `DISPLAY`와 X11/XWayland 소켓이 있는지 확인한 후, 아래 GUI 컨테이너를 별도로 만든다. `xhost`가 없으면 호스트의 `x11-xserver-utils`가 필요하다. 원격 SSH나 순수 Wayland에서 그대로 동작한다고 가정하지 않는다.

```bash
# Ubuntu 데스크톱 호스트. 기존 주행 launch는 먼저 종료한다.
test -n "$DISPLAY" && test -d /tmp/.X11-unix
xhost +SI:localuser:root
sudo docker run -dit --name morai_final_gui --network host --shm-size=1g \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$HOME/morai-final-test/settings:/opt/morai-config:ro" \
  -v "$HOME/morai-final-test/logs:/logs" morai-final-ws:cpu bash
sudo docker exec -it morai_final_gui /usr/local/bin/morai-entrypoint bash
```

이 컨테이너에서는 8/10절 명령의 `xvfb-run ...` 첫 줄을 빼고 `roslaunch`부터 실행한다. 센서 포트가 공유되므로 `morai_final_clean` 안의 launch와 동시에 실행하지 않는다. GUI 시험 종료 후 호스트에서 `xhost -SI:localuser:root`로 허용을 되돌린다. `xhost +`처럼 모든 클라이언트를 허용하지 않는다.

## 6. 네트워크 없는 오프라인 검증

먼저 컨테이너를 띄우기 전/후와 무관하게 **호스트**에서 격리된 일회성 시험을 수행할 수 있다. 모델은 팀의 신뢰한 체크포인트만 사용한다. `.pt` 로드는 pickle을 실행할 수 있다.

```bash
sudo docker run --rm --network none morai-final-ws:cpu \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/smoke_models.py
```

`OFFLINE_SMOKE_PASS`가 나와야 한다. 빈 영상 추론은 파일/API 호환성 확인이지 인식 정확도 시험이 아니다. GPU는 같은 명령에 `--gpus all`, GPU 이미지, 마지막 `--device cuda`를 사용한다.

이후 **컨테이너 내부**에서:

```bash
cd "$MORAI_WS"
rospack find morai_msgs
rospack find turn_signal_controller
rosmsg show morai_msgs/CtrlCmd
rosmsg show morai_msgs/GPSMessage
rosmsg show morai_msgs/EgoVehicleStatus
catkin_make run_tests -j2 -l2
catkin_test_results --verbose build/test_results

# catkin에 등록되지 않은 기존 LiDAR unittest도 별도 실행
python -m unittest discover -s src/detection/lidar_perception/test -p 'test_*.py'

rosrun turn_signal_controller inspect_route_signals.py \
  --path-file "$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  --mgeo-dir "$MORAI_WS/src/detection/camera_perception/lane/mgeo/R_KR_PR_K-city_2025"
```

제공 경로의 정적 대응 기대값은 신호 문맥 6개(직진 4, 우회전 1, 좌회전 1)다. 이는 실제 영상에서 신호를 잘 읽는다는 의미가 아니다. 테스트 실패가 있으면 주행 단계로 넘어가지 않는다.

## 7. MORAI 지도·시나리오·센서 설정

**MORAI Windows PC**에서 수행한다.

1. 실제 대회 지정 버전/라이선스와 `R_KR_PR_K-city_2025` 지도를 확인한다. 저장소의 과거 규정 요약과 지도/차량 표기가 다르므로 설치 결과를 확인한다.
2. Scenario의 Load에서 `2026_molit_comp_sample_scene.json`을 연다. 전체 경로·UI 설명은 [시나리오 실행 안내](SCENARIO_DRIVING_TEST_KO.md)를 따른다.
3. Ego 초기 위치 약 `(-131.486,-427.961,28.883)m`, yaw `62.515°`와 실제 차량 모델을 확인한다. 시작/끝이 같은 폐곡선이므로 초기 경로 진행도가 약 2184m로 잡히거나 goal=true면 주행하지 않는다.
4. 센서는 JSON 시나리오와 별도다. 전송 목적지를 **Ubuntu 호스트 IP**로 바꾼다. 센서 전송을 위해 시뮬레이션은 진행하되 Ego는 정지/수동 상태로 유지한다.

| 데이터 | MORAI 송신/수신 포트 | Ubuntu/Docker 포트 | 방식 |
|---|---:|---:|---|
| Cam1 차선 | 송신 1100 | 수신 1101 | Camera UDP |
| Cam4 신호·객체 | 송신 1130 | 수신 1131 | Camera UDP |
| VLP16 LiDAR | 송신 2000 | 수신 2001 | VLP16 UDP |
| GPS | 센서 설정 | 수신 3001 | NMEA GPRMC/GPGGA |
| IMU | 센서 설정 | 수신 4001 | MORAI binary UDP |
| Ego 상태 | 송신 908 | 수신 909 | MORAI status UDP |
| 차량 제어 | 수신 9093 | 송신 9094 | EgoCtrlCmd UDP |
| 방향지시등 | 수신 9097 | 송신 포트는 OS 할당 | Lamps UDP |

좌/우 카메라 `1110→1111`, `1120→1121` 계약도 있지만 최종 ROI launch는 두 영상을 자동 수신하지 않는다. `0.0.0.0`은 Docker 코드의 로컬 바인드 주소이지 센서 목적지 주소가 아니다. `sensor_ports.yaml`을 편집하는 것만으로 최종 launch의 포트가 모두 바뀌지 않는다. 위 포트와 실제 launch를 맞춘다.

**호스트**에서 충돌/패킷 확인:

```bash
sudo ss -lunp | grep -E ':(909|1101|1131|2001|3001|4001|9094)\b'
sudo tcpdump -ni any -c 30 \
  'udp and (dst port 1101 or dst port 1131 or dst port 2001 or dst port 3001 or dst port 4001 or dst port 909)'
```

기존 프로그램이 수신 포트를 점유하면 소유 프로그램을 확인해 해당 프로그램만 종료한다. 방화벽 전체를 끄지 않는다. Ubuntu는 MORAI PC에서 들어오는 표의 센서 UDP만, Windows는 Ubuntu에서 들어오는 9093/9097 UDP만 허용하도록 범위를 제한한다. ROS master/노드는 같은 컨테이너 내부 통신을 사용하므로 외부 11311 공개가 필요 없다.

## 8. 제어 OFF로 센서·위치·인식 관찰

**컨테이너 터미널 A**:

```bash
xvfb-run -a -s '-screen 0 1920x1080x24' \
  roslaunch morai_bringup final_ws_bringup.launch \
  workspace_path:="$MORAI_WS" \
  path_file:="$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  morai_host_ip:=192.168.0.151 \
  turn_signal_maneuvers_file:=/opt/morai-config/turn_signal_maneuvers.yaml \
  enable_control:=false enable_turn_signal:=false \
  roi_lidar_rviz:=false max_speed_kph:=3.0 fallback_speed_cap_kph:=3.0
```

IP는 실제 MORAI 주소로 바꾼다. roslaunch가 ROS master를 자동 시작하므로 별도 roscore는 필요 없다. 기존 master/주행 노드와 중복 실행하지 않는다. 현재 카메라는 `imshow`를 호출하므로 기본값은 Xvfb 가상 화면이다. `show_raw_preview=0`은 모든 창을 끄는 옵션이 아니며 `final_ws_bringup.launch`에 없는 `show_camera_windows` 인자를 추가하지 않는다.

**컨테이너 터미널 B** (호스트에서 entrypoint로 새로 진입):

```bash
rostopic list
rostopic hz /gps
rostopic hz /Imu
rostopic hz /localization/odometry
rostopic hz /detection/traffic_light
rostopic hz /detection/lane
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /perception/camera/stopline
rostopic echo -n 1 /perception/traffic_light/directional_state
rostopic echo -n 1 /control/maneuver_status
rostopic info /ctrl_cmd
```

`rostopic hz`는 각 명령을 Ctrl+C로 끝낸 뒤 다음 명령을 실행한다. UDP 수신 노드가 실행되기 전에는 `/gps`, `/Imu`가 생기지 않는 것이 정상이다. fresh 토픽 발행만 보지 말고 header 시각과 지연, 입력 원본 단절도 확인한다.

이 launch의 `enable_control:=false`는 차량 제어 UDP를 끄며 방향지시등도 명시적으로 OFF다. 따라서 이 단계에서 정상적인 주행 명령·5초 점등 확인을 기대하면 안 된다. `nominal_stale`, 미보정 차단이 나타날 수 있다. **OFF 설정은 비상정지가 아니다.** 기존 송신기가 보낸 명령을 취소하거나 MORAI 차량을 강제로 정지시키지 않는다.

호스트에서 실제 송신 OFF도 확인한다. 20초 동안 9093/9097 방향 패킷이 없어야 한다. `timeout` 종료 코드 124 자체는 제한시간 종료다.

```bash
sudo timeout 20 tcpdump -ni any \
  'udp and dst host 192.168.0.151 and (dst port 9093 or dst port 9097)'
```

## 9. 보정과 주행 전 통과 조건

호스트의 `settings/turn_signal_maneuvers.yaml`은 읽기 전용 마운트로 컨테이너에 전달된다. 실제 센서 자료로 편집하고 launch를 재시작한다.

- **Cam4:** 현재 `calibrated:false`, 해상도/FOV/장착값은 0인 미입력 상태다. Cam4의 실제 width/height, 수평 FOV, x/y/z, pitch/yaw를 입력하고 지도 신호 투영과 bbox 좌표가 일치하는지 정지 상태에서 검증한다. Cam1 값을 복사하지 않는다. pitch는 이 모듈에서 광축 위쪽이 양수다.
- **위치:** MGeo 원점 `(302595,4124145,0)`, UTM52N, 차량 좌표축/heading을 확인한다. `localization_alignment.yaml`의 미검증 상태는 실제 launch를 막는 자동 게이트가 아니므로 사람이 확인해야 한다.
- **앞범퍼:** 뒤차축 기준 오프셋 3.845m와 차선 BEV의 거리 원점이 실제 차량에 맞는지 확인한다. `stopline_hold_distance_m=0.5`는 설정 목표이며 실제 정지 오차 보장은 아니다.
- **신호:** `require_route_signal_context:true`를 유지한다. 카메라 미보정·잘못된 지도·UNKNOWN을 강제 녹색 처리하거나 timeout을 늘려 차단을 없애지 않는다.
- **미해결 코드:** 보행자 입력 소실과 LiDAR 차선 변경 간격 데이터 지연 문제는 남아 있다. 다른 수정 항목의 회귀 결과와 현장 시험 조건은 최신 문제 상태 문서에서 확인한다.
- **대회 입력 제한:** 인위적인 noise/dropout을 넣지 않는다. 시나리오 JSON의 신호 정답을 실시간 통행 허가 입력으로 사용하지 않는다.

진행 조건은 센서 수신, 실제 모델 추론 주기, 차량·지도 일치, 좌표/카메라 보정, 입력 단절 정지, 단일 제어 송신기와 감독자의 MORAI 정지 수단 확보다.

## 10. 조건 해결 후 저속 주행 실행

**현재 미해결 baseline에서는 실행하지 않는다.** 수정·보정 완료 이미지로 컨테이너를 새 이름으로 만들고 설정을 연결한 뒤 진행한다. 기존 관찰 launch는 Ctrl+C로 종료한다. MORAI에서 차량을 정지·시작점에 배치하고, 실제 UI에서 외부 UDP 제어 수신과 주행 기어를 확인한다. 숫자만 보고 UI 기어를 추정하지 않는다.

**컨테이너**, 감독 하에 실행:

```bash
xvfb-run -a -s '-screen 0 1920x1080x24' \
  roslaunch morai_bringup final_ws_bringup.launch \
  workspace_path:="$MORAI_WS" \
  path_file:="$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  morai_host_ip:=192.168.0.151 \
  turn_signal_maneuvers_file:=/opt/morai-config/turn_signal_maneuvers.yaml \
  enable_control:=true enable_turn_signal:=true \
  turn_signal_lead_time_sec:=5.0 \
  stopline_front_reference_offset_m:=3.845 stopline_hold_distance_m:=0.5 \
  max_speed_kph:=3.0 fallback_speed_cap_kph:=3.0 roi_lidar_rviz:=false
```

경로 추종 → 정지선 → 차선 fallback → 조작/신호 융합 → control mux → `/ctrl_cmd` → UDP 순서다. 신호 화살표는 경로가 요구하는 회전의 허가이지 경로를 임의로 좌/우로 바꾸는 명령이 아니다. 차선 변경은 실제 변경 경로와 maneuvers 구간 설정이 별도로 필요하다.

다른 터미널에서 `/control/maneuver_status`, `/control/turn_signal_state`, `/ctrl_cmd`, `/Ego_topic`, `/experimental/curvature_progress`를 기록한다. 좌/우 방향지시등은 로컬 송신 기준 5초 선행을 검사하며, 시뮬레이터에 실제로 켜졌는지는 UI/영상과 대조한다. UDP 전송 성공은 시뮬레이터 수신 ACK가 아니다.

시험 순서는 직선 3km/h → 적색 정지/허용 신호 출발 → 좌회전/우회전 → 방향지시등 5초 → 앞범퍼 0.5m 정지 오차 → 보행자/장애물 → 입력별 단절 정지 → 전체 경로다. 입력 단절은 별도 검증 시나리오에서만 수행하고 센서에 인위적인 noise를 추가하지 않는다. GPS blackout은 정상 위치 이력과 최신 차선·IMU·속도·전방 안전 조건이 모두 있을 때 교차로 접근 범위 밖에서 15초/30m 이내 차선 유지를 허용한다. 자세한 조건은 최신 빠른 실행 문서와 차선 fallback 설명서를 참조한다.

## 11. 로그, 종료, 다시 실행

주행 launch 실행 중 **컨테이너 다른 터미널**:

```bash
rosbag record --split --size=1024 -O /logs/final_ws_trial \
  /gps /Imu /Ego_topic /localization/odometry \
  /detection/lane /perception/camera/stopline /detection/traffic_light \
  /perception/traffic_light/directional_state /control/maneuver_status \
  /control/turn_signal_state /ctrl_cmd /experimental/curvature_progress
```

이 최소 bag에는 원본 영상/전체 point cloud가 없다. 영상 투영·오인식 분석에는 실제 카메라 영상과 센서 설정 export를 추가 보관해야 한다. 호스트 여유 공간을 확인하고 기록을 중단할 때도 Ctrl+C로 인덱스 저장을 기다린다.

종료는 **MORAI에서 정지/외부 제어 해제 확인 → 주행 launch Ctrl+C → bag 종료 → 컨테이너 종료** 순서다. 프로세스 종료 시 제동 패킷 전송 구현은 있지만 네트워크가 끊기면 전달을 보장하지 못한다. `docker kill`이나 ROS 종료만을 비상정지 수단으로 삼지 않는다.

```bash
# 호스트: 정지 확인 이후
sudo docker stop -t 20 morai_final_clean

# 나중에 같은 설치 환경 재사용
sudo docker start morai_final_clean
sudo docker exec -it morai_final_clean /usr/local/bin/morai-entrypoint bash
```

시나리오 Reset/순간이동 후에는 주행 ROS 노드를 모두 재시작해 EKF·경로 진행도·진입 허가·점등 타이머를 초기화한다. bag 재생 시에는 실차/시뮬레이터 제어 송신기를 켜지 않는다. 상세 격리 재생은 [시나리오 시험 문서](SCENARIO_DRIVING_TEST_KO.md)를 따른다.

성공한 이미지를 그대로 재사용하려면 호스트에서 `sudo docker image inspect morai-final-ws:cpu`와 `/opt/morai-build` 기록을 보관한다. 현장 검증이 끝난 이미지를 `docker save`로 별도 보관하면 재설치 시 apt/pip 최신 결과에 의존하지 않을 수 있다. 이 문서 작성 시점에는 아직 빌드 성공 이미지가 없다.

## 12. 작성 환경에서 수행한 검사

- 최초 작성 시 로컬 Python unittest 252개 통과. 2026-09-09 보완 후 주행 관련 회귀 시험은 334개 통과(카메라 67, 정지선 54, 회전/신호 168, 음영/센서 41, 곡률 4). 서로 다른 시험 범위의 숫자이며 ROS가 없는 로컬 모의 시험이다.
- Python 파일의 3.8 문법, launch/package XML 구문, 문서 Bash 구문과 최종 launch 인자 이름 확인.
- Dockerfile은 미빌드, CPU/GPU 모델 smoke 미실행, ROS catkin 시험 미실행, 실제 UDP 수신/송신 및 주행 미실행.
