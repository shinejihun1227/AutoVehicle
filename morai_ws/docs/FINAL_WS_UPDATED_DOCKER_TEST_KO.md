# final_ws: Docker 첫 설치부터 메시지·환경 설정·MORAI 시험까지

2026-09-09. 정지선·방향별 신호·좌우회전 속도/출구 정렬·GPS 음영 차선 유지를
반영한 코드의 실행 안내다. 처음 사용하는 사람은 **0 → 1 → 2 → 3 → 4 → 5 → 6절**
순서로 진행한다. 이미 만든 컨테이너에 다시 접속하려면 7절을 사용한다.
호스트 명령은 **Ubuntu/Linux의 Bash** 기준이다.
컨테이너는 기존 Dockerfile의 Ubuntu 20.04 / ROS Noetic / Python 3.8을 사용한다.
현재 작업 PC에는 실행 가능한 Docker/WSL 배포판이 없어 이미지 빌드와 실제 MORAI
주행은 수행하지 못했다. 로컬 오프라인 시험 통과와 컨테이너 검증을 구분한다.

## 0. 명령을 어디서 실행하는지와 Docker 첫 설치

MORAI SIM은 Windows에서 실행한다. 아래 Docker에는 ROS 알고리즘과 인식 모델을
설치한다. Ubuntu 호스트에 ROS Noetic이나 MORAI 메시지를 따로 설치할 필요는 없다.

| 문서의 표시 | 실제 터미널 | 역할 |
|---|---|---|
| **Windows PowerShell** | Windows Terminal의 관리자 PowerShell | Windows 사용자만 WSL 최초 설치 |
| **호스트 Bash** | Ubuntu PC 터미널 또는 Docker Desktop과 연결된 WSL Ubuntu 터미널 | Git clone, 이미지 빌드, 컨테이너 생성·접속 |
| **컨테이너 내부** | `docker exec`로 들어간 뒤의 Bash | ROS 환경 확인, 메시지 조회, catkin 빌드, roslaunch |

이미지는 설치가 끝난 실행 환경이고, 컨테이너는 그 이미지로 만든 실행 공간이다.
`docker build`는 이미지 생성, `docker run`은 새 컨테이너 생성,
`docker exec`는 실행 중인 컨테이너에 추가 접속하는 명령이다.
`docker run`을 접속할 때마다 반복하면 새 컨테이너를 만들게 된다.

### 0-1. Ubuntu PC를 사용하는 경우

**Ubuntu 호스트 Bash**에서 확인한다. 이 안내의 호스트 기준은 Ubuntu 22.04/24.04
`x86_64`다. 컨테이너 안의 Ubuntu 20.04와 호스트 버전은 달라도 된다.

```bash
uname -m
cat /etc/os-release
df -h "$HOME"
```

Docker가 이미 설치돼 있으면 `docker version`에서 Client와 Server가 모두 나오는지
확인하고 아래 설치를 건너뛴다. 아래는 Docker가 없는 새 Ubuntu의 설치 명령이다.
기존 `docker.io` 등과 충돌한다면 임의로 기존 컨테이너를 지우지 말고
[Docker 공식 Ubuntu 설치 안내](https://docs.docker.com/engine/install/ubuntu/)의 충돌 패키지 절을 확인한다.

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl git python3
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo docker run --rm hello-world
```

정상이면 `Hello from Docker!`가 표시된다. 이후 예제는 `sudo` 없는 `docker` 명령으로
통일한다. Ubuntu에서는 아래 그룹 설정 후 **로그아웃하고 다시 로그인**한다.
`docker` 그룹은 호스트 관리자 수준의 권한을 갖는다.
[Docker 그룹 설정 근거](https://docs.docker.com/engine/install/linux-postinstall/)

```bash
sudo groupadd -f docker
sudo usermod -aG docker "$USER"
```

다시 로그인한 **호스트 Bash**에서:

```bash
docker version
docker run --rm hello-world
docker ps -a
```

그룹을 사용하지 않는 환경에서는 이 문서의 호스트 `docker` 명령 앞에 `sudo`를 붙인다.

### 0-2. Windows PC에서 Docker Desktop을 사용하는 경우

Ubuntu PC를 준비했다면 이 절은 건너뛴다. **Windows PowerShell을 관리자 권한**으로
열고, WSL Ubuntu가 없는 경우 설치한다.

```powershell
wsl --install -d Ubuntu-24.04
```

재부팅 요청이 나오면 재부팅한다. Ubuntu를 처음 열어 Linux 사용자 이름과 비밀번호를
만든다. 기존 WSL이 있으면 중복 설치하지 말고 아래 결과를 확인한다.
[Microsoft WSL 설치 안내](https://learn.microsoft.com/en-us/windows/wsl/install)

```powershell
wsl --update
wsl -l -v
```

Ubuntu 행의 `VERSION`이 2여야 한다. 1이면 해당 배포판 이름으로 변환한다.

```powershell
wsl --set-version Ubuntu-24.04 2
```

[Docker Desktop Windows 설치 페이지](https://docs.docker.com/desktop/setup/install/windows-install/)에서
설치하고 Docker Desktop을 실행한다. 설정에서 다음을 확인한다.

1. Linux containers 모드, **Use WSL 2 based engine** 사용.
2. **Resources → WSL Integration**에서 사용하는 Ubuntu 활성화.
3. **Resources → Network → Enable host networking** 활성화 후 Apply/restart.
   이 기능은 Docker Desktop 4.34 이상에서 지원한다.

이 구성에서는 WSL Ubuntu 안에 0-1절의 Docker Engine을 추가 설치하지 않는다.
[WSL 연동 설명](https://docs.docker.com/desktop/features/wsl/),
[Desktop host networking 설명](https://docs.docker.com/engine/network/drivers/host/)

이제 **Windows Terminal의 Ubuntu 탭**을 연다. 이후 호스트 명령은 이 Bash에서 실행한다.

```bash
sudo apt-get update
sudo apt-get install -y git ca-certificates curl python3
docker version
docker run --rm hello-world
```

`docker`가 없거나 Server에 연결하지 못하면 Desktop 실행 상태와 WSL Integration을
먼저 확인한다. 이후의 `export`, `source`, `$(...)` 명령을 PowerShell에 붙여 넣지 않는다.
소스는 아래에서 만드는 WSL의 `$HOME/morai-final-test`에 둔다.

### 0-3. NVIDIA GPU를 사용할 경우

CPU로 설치·오프라인 검사를 시작하면 이 절을 건너뛴다. GPU 사용 여부는
2절의 이미지 종류와 4절의 컨테이너 실행 옵션까지 맞춰야 한다.

**실제 Ubuntu PC + Docker Engine**에서는 드라이버를 설치한 후 호스트에서
`nvidia-smi`가 먼저 성공해야 한다. 드라이버 설치는 GPU에 맞는
[NVIDIA 공식 드라이버 안내](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/)
를 따른다. 그 다음 Container Toolkit을 설치한다.
[공식 Toolkit 설치·Docker 설정](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

```bash
nvidia-smi
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

Docker 재시작은 실행 중인 다른 컨테이너에도 영향을 줄 수 있으므로 초기 설정 때
수행한다. **Windows Docker Desktop**은 WSL2 GPU 지원 Windows NVIDIA 드라이버와
최신 WSL을 사용한다. 위의 Linux 드라이버·Docker 데몬 설정을 WSL에 그대로 적용하지 않는다.
[Docker Desktop GPU 안내](https://docs.docker.com/desktop/features/gpu/)

## 1. 코드 가져오기

새로 받는 경우, **호스트 Bash**에서:

```bash
mkdir -p "$HOME/morai-final-test"
cd "$HOME/morai-final-test"
git clone --branch final_ws --single-branch \
  https://github.com/shinejihun1227/AutoVehicle.git AutoVehicle
cd AutoVehicle
```

기존 clone은 저장한 작업이 있는지 `git status --short`로 확인한 뒤:

```bash
cd "$HOME/morai-final-test/AutoVehicle"
git switch final_ws
git pull --ff-only origin final_ws
git log -1 --oneline
```

기존 clone의 위치가 다르면 `cd` 경로만 실제 위치로 바꾼다. 새 clone/기존 clone
중 하나만 실행한다. 다음 파일 확인은 두 경우 모두 **저장소 루트**에서 수행한다.

```bash
pwd
git status --short
test -f morai_ws/docker/final_ws/Dockerfile
test -f morai_ws/src/common/morai_perception_msgs/msg/StopLineDetection.msg
wc -c morai_ws/src/detection/camera_perception/lane/lane_seg_best.pt \
  morai_ws/src/detection/camera_perception/models/best0902.pt \
  morai_ws/src/detection/camera_perception/models/yolov8n.pt
```

`test`는 성공하면 출력이 없다. 모델 파일의 현재 크기는 순서대로 약 97.9MB,
22.6MB, 6.55MB다. 파일이 없거나 작은 텍스트 포인터뿐이면 모델 본체를 먼저 확보한다.

아래 빌드는 AutoVehicle 저장소 루트에서 실행한다. 이미지에 코드가 복사되므로
호스트에서 `git pull`만 하고 이전 컨테이너를 재시작하면 새 코드가 반영되지 않는다.
새 태그로 이미지를 빌드하고 새 이름의 컨테이너를 만든다.

## 2. 필요한 메시지 파일과 이미지 빌드

### 2-1. 어떤 메시지 패키지가 필요한가

`.msg`는 통신 데이터의 정의이고, Python에서 import하는 메시지 코드는
**catkin 빌드 때 생성**된다. `.msg` 파일만 복사하거나 pip로 설치하는 것으로 끝나지 않는다.

| ROS 패키지 이름 | 소스 위치 / 확보 방법 | 주요 메시지 |
|---|---|---|
| `morai_msgs` | Dockerfile이 MORAI 공식 ROS1 저장소를 `src/common/morai_msgs`에 clone | `CtrlCmd`, `GPSMessage`, `EgoVehicleStatus` |
| `morai_perception_msgs` | 프로젝트에 포함: `src/common/morai_perception_msgs` | `LaneDetection`, `TrafficLight`, `StopLineDetection`, `SafetyStop`, `SensorQuality` |
| `common` | 프로젝트에 포함: **폴더 이름은 `src/common/common_msgs`** | `ObjectInfo`, `ObjectInfoArray` |
| `lidar_perception` | 프로젝트에 포함: `src/detection/lidar_perception` | `LidarObstacleArray`, `MergeGapObstacleArray` |
| `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs` | Dockerfile에서 ROS apt 패키지로 설치 | Header, IMU, PointCloud2, Pose, Odometry, Path 등 |

위 경로는 컨테이너의 `/opt/AutoVehicle/morai_ws` 기준이다.
`ObjectInfoArray`의 import는 `from common.msg import ObjectInfoArray`다.
폴더 이름을 보고 `common_msgs.msg`로 import하면 안 된다.

외부 메시지는 [MORAI 공식 ROS1 메시지 저장소](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs)를
사용한다. ROS2 버전과 섞지 않는다. 대회에서 제공한 SDK와 대응하는 커밋을 알고 있으면
그 **40자리 SHA**를 선택한다. 모르면 아래 명령으로 main의 SHA를 기록해 빌드 후보로
사용할 수 있지만, 대회 SDK와 필드가 맞는지는 별도 확인해야 한다.

### 2-2. 코드·메시지 버전을 정하고 빌드

**호스트 Bash**, AutoVehicle 저장소 루트에서 실행한다.

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
docker build --progress=plain \
  --build-arg CODE_REVISION="$CODE_REF" \
  --build-arg MORAI_MSGS_REF="$MORAI_MSGS_REF" \
  --build-arg TORCH_FLAVOR="$TORCH_FLAVOR" \
  -f morai_ws/docker/final_ws/Dockerfile -t "$IMAGE" morai_ws
```

GPU를 준비했다면 위의 `export TORCH_FLAVOR=cpu`를 `export TORCH_FLAVOR=cu121`로
바꾼 뒤 `IMAGE` 설정과 빌드를 진행한다. CPU 이미지에 `--gpus all`만 붙이는 것으로
CUDA용 PyTorch가 설치되지는 않는다.

마지막 `morai_ws`가 **빌드 컨텍스트**다. 저장소 루트의 `.`로 바꾸지 않는다.
빌드 로그에서 오류 없이 이미지 태그까지 생성된 것을 확인한 뒤:

```bash
docker image inspect "$IMAGE" --format '{{.Id}}'
```

Dockerfile은 다음 작업을 순서대로 자동 수행한다. **처음 만든 컨테이너에 들어가서
ROS·메시지를 다시 설치할 필요가 없도록 이미지 빌드에 포함한 것이다.**

1. `ros:noetic-ros-base-focal`에서 시작: Ubuntu 20.04 / ROS1 Noetic / Python 3.8.
2. catkin, ROS 기본 메시지, OpenCV 관련 시스템 라이브러리, Xvfb 설치.
3. `/opt/morai-venv` 생성. ROS apt 모듈을 함께 사용하는 `--system-site-packages` 환경.
4. Torch·YOLO 등 `docker/final_ws/requirements.txt`의 Python 의존성 설치.
5. 프로젝트 소스·지도·경로·모델 복사, 선택한 SHA의 `morai_msgs` 다운로드.
6. `catkin_make`로 사용자 메시지 코드 생성과 패키지 빌드.
7. 메시지/Python import 검사와 5개 회귀 시험 묶음 실행.
8. 코드 SHA·MORAI 메시지 SHA·설치 패키지 목록을 `/opt/morai-build`에 기록.

따라서 정상 빌드된 이미지에는 메시지·모델·의존성이 모두 들어 있다.
CPU 실시간 추론이 늦어 차선 관측의 0.15초 제한을 넘으면
제어는 정지한다. GPU는 `TORCH_FLAVOR=cu121`로 빌드하고 실행 시 `--gpus all`을 추가한다.
외부 메시지 저장소 main의 선택은 대회 SDK 호환 검증을 대신하지 않는다.

### 2-3. 새 터미널에서도 사용할 이름과 경로 저장

**같은 호스트 Bash**에서 실행한다. 컨테이너 이름에는 CPU/GPU 구분도 넣는다.

```bash
export SETTINGS="$HOME/morai-final-test/settings-${CODE_REF:0:8}"
export LOGS="$HOME/morai-final-test/logs-${CODE_REF:0:8}"
export CONTAINER="morai-final-${CODE_REF:0:8}-${TORCH_FLAVOR}"
mkdir -p "$SETTINGS" "$LOGS"
declare -p CODE_REF MORAI_MSGS_REF TORCH_FLAVOR IMAGE SETTINGS LOGS CONTAINER \
  > "$HOME/morai-final-test/current-session.sh"
```

이후 **호스트에서 새 터미널을 열 때마다** 아래를 먼저 실행한다.
이 파일은 본인이 위에서 저장한 변수 파일이며, 컨테이너 안의 파일이 아니다.

```bash
source "$HOME/morai-final-test/current-session.sh"
printf 'image=%s\ncontainer=%s\n' "$IMAGE" "$CONTAINER"
```

## 3. 오프라인 시험

호스트에서 네트워크 없는 컨테이너로 실행한다. ROS master나 MORAI는 필요 없다.

```bash
docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/run_regression.py

# GPS 음영구간 테스트만 선택
docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/run_regression.py --suite blackout

# 대회 경로 우선, 신호 방향 불일치, 조향 경로 대조 테스트
docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/run_regression.py --suite turn

# 실제 모델 로드와 빈 영상 1회 추론
docker run --rm --network none "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/smoke_models.py
```

GPU 모델 검사는 GPU 이미지로 다음 명령을 사용한다.

```bash
docker run --rm --network none --gpus all "$IMAGE" \
  python /opt/AutoVehicle/morai_ws/docker/final_ws/smoke_models.py --device cuda
```

회귀 시험 성공 표시는 `REGRESSION_PASS`,
모델 smoke 성공 표시는 `OFFLINE_SMOKE_PASS`다.

보완 시점 로컬 회귀 결과는 359개 통과(카메라 67, 정지선 54, 회전/신호 190,
음영/센서 41, 곡률 7)다. 시험은 합성 차선/경로와 대체 ROS 입력을 사용하므로,
실제 영상 정확도·MORAI 차량 응답은 아래 센서/주행 단계에서 확인한다.

## 4. 새 컨테이너 생성·접속·ROS 환경과 메시지 확인

### 4-1. 센서 네트워크 확인

예시 MORAI PC 주소 `192.168.0.151`을 실제 값으로 바꾼다. MORAI 센서의 목적지 IP는
**Docker를 실행하는 Ubuntu 호스트의 LAN IP**다. Linux의 `--network host`는
호스트 네트워크를 공유한다. 같은 센서 포트를 점유하는 launch를 중복 실행하지 않는다.
[Docker host 네트워크 설명](https://docs.docker.com/engine/network/drivers/host/)

예를 들어 MORAI PC가 `192.168.0.151`, Ubuntu PC가 `192.168.0.200`이면 센서 목적지는
`192.168.0.200`, 차량·방향지시등 명령의 목적지는 `192.168.0.151`이다.
Linux 호스트 IP는 `ip -br -4 address`, Windows의 LAN IP는 PowerShell의 `ipconfig`로
확인한다. Desktop 사용 시 센서 목적지는 Windows 호스트의 도달 가능한 주소를 기준으로
검사하며, WSL 내부의 임시 IP를 그대로 대입하지 않는다.

| 데이터 | Docker 호스트 수신 포트 또는 MORAI 수신 포트 |
|---|---|
| Cam1 차선 | UDP 1101 |
| Cam4 신호·객체 | UDP 1131 |
| VLP16 LiDAR | UDP 2001 |
| GPS / IMU | UDP 3001 / 4001 |
| Ego 상태 | UDP 909 |
| 차량 제어 | MORAI UDP 9093, 송신 출발 포트 9094 |
| 방향지시등 | MORAI UDP 9097 |

MORAI에서 사용하는 K-City 지도·시나리오·대회 경로를 맞추고, 위 센서의 송신을 켠다.
이 launch는 MORAI와 **UDP로 통신**한다. ROS 네이티브 송신이나 rosbridge 설치가
이 구성의 선행 조건은 아니다. `--network host`에서는 `-p`를 함께 지정하지 않는다.

### 4-2. 설정 폴더와 컨테이너 생성

호스트에서 새 설정 폴더와 컨테이너를 만든다. 기존 보정 파일이 있으면 변경 내용과
비교해 새 파일에 반영한다. 초기 설정을 복사하면 Cam4는 미보정 상태다.

```bash
source "$HOME/morai-final-test/current-session.sh"
cd "$HOME/morai-final-test/AutoVehicle"
mkdir -p "$SETTINGS" "$LOGS"
cp -n morai_ws/src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml \
  "$SETTINGS/turn_signal_maneuvers.yaml"

# CPU/GPU 이미지 종류에 맞춰 GPU 전달 옵션을 선택한다.
GPU_ARGS=()
if [ "$TORCH_FLAVOR" = cu121 ]; then
  GPU_ARGS=(--gpus all)
fi
docker run -dit --name "$CONTAINER" --network host --shm-size=1g \
  "${GPU_ARGS[@]}" \
  -v "$SETTINGS:/opt/morai-config:ro" -v "$LOGS:/logs" "$IMAGE" bash
docker ps --filter "name=$CONTAINER"
```

`Up`이면 실행 중이다. 같은 이름의 컨테이너가 이미 있으면 `run`을 반복하지 말고
7절의 재접속 절차를 사용한다. `cp -n`은 기존 보정 파일을 덮어쓰지 않는다.
호스트 clone을 다른 곳에 뒀다면 위 `cd`를 실제 위치로 바꾼다.

`$SETTINGS`는 컨테이너의 `/opt/morai-config`에 읽기 전용으로 연결하고,
`$LOGS`는 `/logs`에 저장용으로 연결한다. 설정은 **호스트 파일을 편집**한다.
전체 workspace를 마운트하면 이미지 안의 `devel`과 메시지 빌드를 가릴 수 있으므로
위의 두 폴더만 연결한다. 기본 컨테이너 사용자는 root이며, 로그가 호스트에서도
root 소유로 생성될 수 있다.

### 4-3. 컨테이너에 접속하고 환경 확인

아직 **호스트 Bash**에서 실행한다.

```bash
source "$HOME/morai-final-test/current-session.sh"
docker exec -it "$CONTAINER" /usr/local/bin/morai-entrypoint bash
```

이후는 **컨테이너 내부**다. 예시 프롬프트는 `root@...:/opt/AutoVehicle/morai_ws#`이며,
명령을 복사할 때 프롬프트 자체는 입력하지 않는다.

```bash
whoami
pwd
echo "$MORAI_WS"
echo "$ROS_DISTRO"
which python
python --version
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
```

| 항목 | 기본 이미지에서 기대하는 값 |
|---|---|
| 사용자 | `root` |
| workspace | `/opt/AutoVehicle/morai_ws` |
| ROS 배포판 | `noetic` |
| Python 경로 | `/opt/morai-venv/bin/python` |
| Python 버전 | 3.8.x |
| ROS master | `http://127.0.0.1:11311` |
| ROS_IP | `127.0.0.1` |

`morai-entrypoint`는 매 접속 시 아래 세 환경을 순서대로 불러온다.
`docker exec ... bash`만 사용해 접속했다면 **컨테이너 내부에서** 직접 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source /opt/morai-venv/bin/activate
source /opt/AutoVehicle/morai_ws/devel/setup.bash
cd "$MORAI_WS"
```

각각 ROS 명령·기본 메시지, 프로젝트 Python 라이브러리, 생성된 사용자 메시지·패키지
검색 경로를 준비한다. `PYTHONPATH`나 `ROS_PACKAGE_PATH`를 빈 값으로 덮어쓰지 않는다.
매번 entrypoint로 접속하면 `.bashrc`를 따로 편집하지 않아도 된다.

위 loopback ROS 설정은 **모든 ROS 노드가 같은 컨테이너 안에서 동작**하기 때문이다.
MORAI 센서 목적지 IP나 `morai_host_ip`와는 용도가 다르다. 컨테이너 밖의 다른 PC에서
ROS 노드를 연결하는 구성은 별도 설정이 필요하다.

### 4-4. 필요한 메시지가 실제로 생성됐는지 확인

**컨테이너 내부**에서 실행한다. 이 확인에는 `roscore`가 필요하지 않다.

```bash
cd "$MORAI_WS"
rospack find morai_msgs
rospack find morai_perception_msgs
rospack find common
rospack find lidar_perception
rospack find morai_bringup

rosmsg show morai_msgs/CtrlCmd
rosmsg show morai_msgs/GPSMessage
rosmsg show morai_msgs/EgoVehicleStatus
rosmsg show morai_perception_msgs/StopLineDetection
rosmsg show morai_perception_msgs/LaneDetection
rosmsg show common/ObjectInfoArray
rosmsg show lidar_perception/MergeGapObstacleArray
```

패키지 경로가 출력되고 각 메시지의 필드 목록이 보여야 한다. Python에서도 확인한다.

```bash
python - <<'PY'
import sys
import rospy, cv2, numpy, torch, torchvision, ultralytics
from morai_msgs.msg import CtrlCmd, GPSMessage, EgoVehicleStatus
from morai_perception_msgs.msg import LaneDetection, StopLineDetection, SafetyStop, SensorQuality
from common.msg import ObjectInfoArray
from lidar_perception.msg import MergeGapObstacleArray
required = {'longlCmdType', 'accel', 'brake', 'steering', 'velocity', 'acceleration'}
assert required.issubset(set(CtrlCmd.__slots__)), 'CtrlCmd fields differ from this controller'
print('PYTHON', sys.executable, sys.version.split()[0])
print('MESSAGES_OK')
print('TORCH', torch.__version__, 'CUDA_AVAILABLE', torch.cuda.is_available())
PY
```

CPU 이미지에서는 `CUDA_AVAILABLE False`가 정상이다. GPU 이미지에 GPU를 전달해
실행한 경우에는 True여야 한다. 외부 SDK·설치 버전 기록은 다음으로 확인한다.

```bash
cat /opt/morai-build/code-revision.txt
cat /opt/morai-build/morai-msgs-revision.txt
git -C "$MORAI_WS/src/common/morai_msgs" rev-parse HEAD
python -m pip check
```

`morai_msgs`의 두 SHA가 일치해야 한다. 메시지 소스는
`$MORAI_WS/src/common/morai_msgs/msg`, 생성된 Python 코드는 보통
`$MORAI_WS/devel/lib/python3/dist-packages` 아래에 있다.

### 4-5. 메시지나 코드를 수정했을 때 컨테이너에서 재빌드

정상 이미지를 처음 실행할 때는 2절에서 이미 빌드했으므로 이 단계가 필수는 아니다.
컨테이너에서 `.msg`나 소스를 수정했거나 생성 코드를 다시 확인할 때 사용한다.
실행 중인 launch를 종료하고 **컨테이너 내부**에서:

```bash
source /opt/ros/noetic/setup.bash
source /opt/morai-venv/bin/activate
cd "$MORAI_WS"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/opt/morai-venv/bin/python3
source "$MORAI_WS/devel/setup.bash"
python docker/final_ws/run_regression.py
```

catkin에 등록된 테스트도 실행하려면 이어서:

```bash
catkin_make run_tests -j2 -l2
catkin_test_results --verbose build/test_results
```

`.msg` 변경은 파일을 저장한 뒤 빌드하고, 각 ROS 터미널에서 `devel/setup.bash`를
다시 source한 다음 관련 노드를 재시작해야 반영된다. `catkin_make`는 workspace
루트에서 실행하며 `src` 안에서 실행하지 않는다. Noetic은 `catkin_make`를 사용한다.

외부 `morai_msgs`가 없으면 먼저 잘못된 이미지/마운트인지 확인하고 2절에서 선택한
SDK SHA로 이미지를 다시 빌드한다. 프로젝트용 `common`이나 `morai_perception_msgs`를
이름이 비슷한 외부 패키지로 대체하지 않는다. 정상 Dockerfile 구성에는 `rosdep init`
또는 메시지별 수동 다운로드가 추가로 필요하지 않다.

컨테이너에서 수정한 소스는 그 컨테이너에만 남는다. 재사용할 변경은 호스트 저장소에
반영한 뒤 이미지를 다시 빌드한다. 이미지에는 저장소의 `.git`이 복사되지 않으므로
코드 업데이트용 `git pull`은 **호스트 clone**에서 실행한다.

### 4-6. 주행 설정 파일 확인

**컨테이너 내부**에서 설정이 보이는지 먼저 확인한다.

```bash
ls -l /opt/morai-config/turn_signal_maneuvers.yaml
cat /opt/morai-config/turn_signal_maneuvers.yaml
test -f "$MORAI_WS/data/routes/2026_molit_comp_global_path.txt"
```

**호스트의 별도 Bash**에서 보정 파일을 편집한다. 아래는 `nano` 편집기를 쓰는 예다.

```bash
source "$HOME/morai-final-test/current-session.sh"
sudo apt-get install -y nano
nano "$SETTINGS/turn_signal_maneuvers.yaml"
```

`nano`에서는 Ctrl+O → Enter로 저장하고 Ctrl+X로 나온다. Cam4의 `signal_camera`
해상도, FOV, 위치·방향을 반영한다. 저장소 `lane/cam_set.json`의 Cam1 값을
Cam4 값으로 그대로 복사하지 않는다.
`calibrated`는 실제 투영을 검증한 후 설정한다. 설정을 저장한 뒤 launch를 재시작한다.
곡률 공통값 `max_speed_kph`, `lateral_accel_limit_mps2`는 5~6절의 launch 인자로 전달한다.

대회 경로와 신호 지도 연결도 **컨테이너 내부**에서 검사할 수 있다.

```bash
rosrun turn_signal_controller inspect_route_signals.py \
  --path-file "$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  --mgeo-dir "$MORAI_WS/src/detection/camera_perception/lane/mgeo/R_KR_PR_K-city_2025"
```

저장소의 기본 경로는 연결된 신호 구간 6개(직진 4, 좌회전 1, 우회전 1)가 기대값이다.
다른 경로를 쓰면 개수도 달라진다. 이 검사는 지도 연결 확인이며 영상 신호 인식 시험은 아니다.

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

센서 launch가 실행 중인 터미널은 그대로 둔다. **호스트의 별도 터미널**에서:

```bash
source "$HOME/morai-final-test/current-session.sh"
docker exec -it "$CONTAINER" /usr/local/bin/morai-entrypoint bash
```

그 다음 **컨테이너 내부의 두 번째 터미널**에서:

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

## 7. 종료·다시 접속·코드 업데이트

### 7-1. 오늘 작업을 마칠 때

1. MORAI에서 차량을 정지시키고 외부 제어 해제를 확인한다.
2. launch와 rosbag을 실행한 각각의 터미널에서 Ctrl+C를 누른다.
3. `docker exec`로 접속했던 쉘에서 `exit`를 입력해 호스트로 나온다.

컨테이너까지 중지하려면 **호스트 Bash**에서:

```bash
source "$HOME/morai-final-test/current-session.sh"
docker stop "$CONTAINER"
docker ps -a --filter "name=$CONTAINER"
```

`docker exec` 쉘의 `exit`는 접속 종료이며, 컨테이너 자체의 중지와 다르다.
`docker stop`은 컨테이너 파일을 삭제하지 않는다. 호스트의 `$LOGS`에는 rosbag이 남는다.

### 7-2. 다음 날 같은 컨테이너에 다시 접속

Windows 사용자는 Docker Desktop을 실행한 다음 WSL Ubuntu 터미널을 연다.
Ubuntu 사용자는 Docker 서비스가 실행 중인지 확인한다. **호스트 Bash**에서:

```bash
source "$HOME/morai-final-test/current-session.sh"
docker ps -a --filter "name=$CONTAINER"
```

상태가 `Exited`이면 먼저 시작한다. `Up`이면 `start`를 건너뛴다.

```bash
docker start "$CONTAINER"
```

접속한다.

```bash
docker exec -it "$CONTAINER" /usr/local/bin/morai-entrypoint bash
```

접속 후 ROS·venv·메시지 환경은 자동으로 준비된다. 5절 또는 6절을 실행할 터미널에서
`MORAI_HOST_IP`를 실제 값으로 다시 설정한다. 센서/주행 노드는 자동으로 시작되지 않는다.

### 7-3. GitHub의 새 코드를 반영

MORAI와 기존 launch를 종료한 뒤 **호스트 Bash의 저장소**에서:

```bash
cd "$HOME/morai-final-test/AutoVehicle"
git status --short
```

수정 사항이 있으면 먼저 저장·커밋하거나 별도로 보관한다. 준비가 끝나면:

```bash
git switch final_ws
git pull --ff-only origin final_ws
git log -1 --oneline
```

그 다음 **2-2절 빌드 → 2-3절 변수 저장 → 3절 검사 → 4-2절 새 컨테이너 생성**을
반복한다. 새 코드 SHA로 이미지 태그·컨테이너 이름이 달라진다.
기존 컨테이너를 `restart`하는 것만으로 이미지 속 코드가 갱신되지는 않는다.

새 `$SETTINGS`는 초기 설정이므로 기존 Cam4 보정값을 비교해 반영한다.
이전 YAML의 `turn_left_speed_kph`, `turn_right_speed_kph`, `turn_lateral_accel_mps2`는
제거된 항목이며 새 launch의 공통 속도 파라미터를 사용한다.
같은 코드로 이미지만 다시 만들었다면 새 컨테이너 이름을 선택하거나 기존 컨테이너를
의도적으로 교체해야 한다. 기존 컨테이너를 자동 삭제하는 명령은 이 안내에 포함하지 않는다.

## 8. 처음 실행할 때 자주 만나는 오류

| 증상 | 먼저 확인할 내용 |
|---|---|
| 호스트에서 `docker: command not found` | Ubuntu는 0-1절 설치, WSL은 Desktop 실행·WSL Integration 확인 |
| `Cannot connect to the Docker daemon` | Ubuntu의 Docker 서비스 또는 Windows의 Docker Desktop 실행 상태 확인 |
| Docker socket `permission denied` | Ubuntu의 docker 그룹 적용 후 재로그인 또는 호스트 명령에 `sudo` 사용 |
| `invalid reference format` / 컨테이너 이름이 비어 있음 | 호스트에서 `source "$HOME/morai-final-test/current-session.sh"` 실행 |
| 컨테이너 이름이 이미 사용 중 | `docker ps -a`로 확인 후 7-2절로 재접속. 새 생성이면 다른 이름 사용 |
| `MORAI_MSGS_REF` 검사 또는 clone 실패 | 2절의 SHA가 40자리인지, 호스트/빌드 네트워크에서 GitHub에 연결되는지 확인 |
| `roslaunch` / `rospack`을 찾지 못함 | 현재 컨테이너 내부인지 확인하고 4-3절의 entrypoint 또는 source 실행 |
| `No module named morai_msgs` / 메시지 패키지를 찾지 못함 | 4-3절 환경 로딩 → 4-4절 소스/생성 여부 → 4-5절 빌드 순서로 확인 |
| `No module named common_msgs` | 이 저장소의 ROS 패키지는 `common`. `from common.msg import ...` 사용 |
| `CtrlCmd` 필드 오류 | 선택한 MORAI ROS1 SDK와 컨트롤러 메시지 정의를 비교. 임의 필드 이름 변경으로 우회하지 않음 |
| `catkin_make`가 패키지를 못 찾음 | `cd "$MORAI_WS"` 위치, 메시지 소스, ROS source 상태 확인 |
| 모델 `.pt` 없음 / import 오류 | 1절 모델 파일 크기, 2절 빌드 성공, 3절 `smoke_models.py` 확인 |
| GPU 이미지인데 CUDA가 False | `cu121` 이미지인지, 생성 시 `--gpus all`을 넣었는지, 호스트 드라이버 상태 확인 |
| `Could not connect to display` | 기본 실행 명령의 `xvfb-run` 사용. 실제 창이 필요하면 X11 설정 후 실행 |
| 센서 토픽이 없거나 갱신되지 않음 | MORAI의 센서 송신·목적지 IP·UDP 포트·host networking·방화벽 확인 |
| `Address already in use` | 같은 UDP 포트를 사용하는 다른 launch/컨테이너가 실행 중인지 확인 |
| `reference_path_not_received` / 경로 mismatch | 곡률 노드의 기준 경로 발행과 fusion이 같은 `path_file`을 쓰는지 확인 |
| `signal_camera_uncalibrated` | Cam4의 실제 보정 미완료 상태. 4-6절의 장착값과 투영 확인 필요 |
| 제어 OFF에서 `nominal_stale` | 센서 관찰 단계에서 주행 명령이 발행되지 않는 상태일 수 있음. 5절 설명 확인 |

문제를 문의할 때는 **호스트의 이미지/컨테이너 이름**, 컨테이너의
`/opt/morai-build/code-revision.txt`, `morai-msgs-revision.txt`, 오류가 난 명령과
마지막 오류 출력을 함께 기록하면 같은 환경에서 원인을 확인하기 쉽다.
