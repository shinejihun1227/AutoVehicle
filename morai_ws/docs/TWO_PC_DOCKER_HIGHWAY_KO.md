# MORAI + RTX 4090 Docker: 복사해서 따라 하는 안내

**처음 설치할 때는 1~8번, 다음에 컴퓨터를 켰을 때는 9번만 따라 한다.**
기능별 시험은 10번, 속도·조향 설정은 11번, 문제 해결은 12번이다.
**10번 이후는 필요한 항목만 선택한다. 모든 주행 예시를 차례로 실행하는 것이 아니다.**

코드 블록 **하나 전체를 복사 → 붙여넣기 → Enter → 완료 확인** 순서로 진행한다.
여러 블록을 한꺼번에 붙이지 않는다. 오류가 나면 그 블록의 출력부터 확인한다.
`sudo` 비밀번호는 Ubuntu 로그인 비밀번호다. 입력할 때 글자가 안 보이는 것이 정상이다.
설치 블록의 `( ... )`는 오류가 나도 터미널 창 자체가 닫히지 않게 하는 구문이다.

## 1. 어느 컴퓨터에서 무엇을 하는지

사진으로 확인한 **실제 Ubuntu는 22.04.4 LTS**, GPU는 **RTX 4090 24GB**,
NVIDIA 드라이버는 **580.178.04**다. Docker 안에는 **Ubuntu 20.04 + ROS Noetic** 환경을 만든다.

```text
Ubuntu 제어 PC: 192.168.0.185
  Ubuntu 22.04.4 = 호스트 = 컴퓨터에 직접 설치된 운영체제
    Docker Engine = 컨테이너를 켜고 끄는 프로그램
      morai-highway-gpu = 우리가 만들 컨테이너
        Ubuntu 20.04 환경 + ROS Noetic + 인식/주행 코드
                    ↕ 센서 수신 / 차량 제어 송신
Windows MORAI PC: 192.168.0.148
  MORAI 시뮬레이터
```

**이미지**는 설치한 환경과 코드를 묶은 패키지, **컨테이너**는 그것을 실행하는 공간이다.
컨테이너는 호스트의 커널과 GPU를 공유한다. PC를 Ubuntu 20.04로 다시 부팅하는 것은 아니다.

**별도 표시가 없으면 모든 명령은 Ubuntu에서 `Ctrl+Alt+T`로 연 일반 터미널에 붙인다.**
이것을 **호스트 터미널**이라고 부른다. Docker GUI 앱을 열 필요는 없다.
Windows에서는 MORAI만 설정한다. Windows PowerShell에 아래 Bash 명령을 붙이지 않는다.

| 명령 | 하는 일 | 언제 쓰는가 |
|---|---|---|
| `bash run_highway.sh build` | 설치된 환경과 코드를 이미지로 만듦 | 최초/코드 갱신 때 |
| `bash run_highway.sh start` | 저장된 컨테이너를 켬 | 부팅 후 |
| `bash run_highway.sh shell` | 컨테이너 내부 터미널에 접속 | ROS 진단할 때 |
| `bash run_test.sh show` | 저장한 시험 종류와 값 확인 | 실행 전 |
| `bash run_test.sh monitor` | 센서/제어 계산 확인, 차량 송신은 끔 | 주행 전 |
| `bash run_test.sh drive` | 저장한 값으로 MORAI 주행 명령 송신 | 주행할 때 |
| `bash run_highway.sh stop` | 컨테이너 종료 | 작업 종료 때 |

위 명령은 모두 호스트의 `$HOME/AutoVehicle/morai_ws/docker/final_ws` 폴더에서 실행한다.

## 2. 처음 한 번: Docker 설치

**실행 위치: Ubuntu 호스트 터미널.** Docker 설치와 부팅 시 Engine 자동 시작 설정이다.
기존 호스트 Docker 서비스가 있으면 설치 부분을 건너뛰고 그 서비스를 사용한다.
Docker Desktop이 있어도 이 문서의 대상은 호스트 Engine의 `default` 연결이다.

```bash
(
set -euo pipefail
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg git pciutils
if ! systemctl cat docker.service >/dev/null 2>&1; then
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi
sudo systemctl enable --now docker
sudo docker --context default run --rm hello-world
)
```

**완료 기준:** `Hello from Docker!`가 나온다. 패키지 충돌 오류가 나면 다음 단계로 가지 말고
그 오류를 확인한다. 출처: [Docker 공식 Ubuntu 설치 안내](https://docs.docker.com/engine/install/ubuntu/).

## 3. 처음 한 번: Docker에서 RTX 4090 사용 설정

**실행 위치: Ubuntu 호스트 터미널.** 컨테이너가 NVIDIA GPU에 접근하도록 연결한다.
사진에서 드라이버는 이미 정상 확인됐다. 드라이버를 다시 설치하는 단계는 아니다.

```bash
(
set -euo pipefail
nvidia-smi
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo docker --context default run --rm --runtime=nvidia --gpus all ubuntu:20.04 nvidia-smi
)
```

**완료 기준:** 마지막에도 `NVIDIA GeForce RTX 4090` 표가 나온다. 처음에는 이미지 다운로드가 진행된다.
Docker 재시작은 기존 컨테이너에 영향을 줄 수 있으므로 주행 전에 수행한다.

`nvidia-smi`의 `CUDA Version: 13.0`은 드라이버가 지원하는 CUDA 버전이다.
우리 이미지는 CUDA 12.1용 PyTorch인 `cu121`을 사용한다. 최신 드라이버의 하위 호환으로
이전 CUDA 프로그램을 실행하므로 숫자를 맞추려고 드라이버를 다시 설치할 필요는 없다.
출처: [NVIDIA Toolkit 설치](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html),
[GPU 컨테이너 확인](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html),
[CUDA 호환성](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html).

## 4. 처음 한 번: GitHub 코드와 내 PC 설정 저장

**실행 위치: Ubuntu 호스트 터미널.** 홈 폴더에 `AutoVehicle`을 받는다.
이미 받은 폴더라면 `final_ws`를 갱신한다. Git이 로컬 수정 때문에 멈추면 그 메시지를 확인한다.
수정 파일을 삭제하거나 `reset --hard`로 없애지 않는다.

```bash
(
set -euo pipefail
if [ ! -d "$HOME/AutoVehicle" ]; then
  git clone --branch final_ws --single-branch https://github.com/shinejihun1227/AutoVehicle.git "$HOME/AutoVehicle"
else
  git -C "$HOME/AutoVehicle" switch final_ws
  git -C "$HOME/AutoVehicle" pull --ff-only origin final_ws
fi
test -f "$HOME/AutoVehicle/morai_ws/docker/final_ws/run_test.sh"
git -C "$HOME/AutoVehicle" log -1 --oneline
)
```

다음 블록은 IP·GPU·컨테이너 이름을 `highway.env`에 저장한다.
이미 파일이 있으면 날짜가 붙은 백업을 먼저 남긴다. 시험 파라미터 파일은 없을 때만 만든다.

```bash
(
set -euo pipefail
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
if [ -f highway.env ]; then cp -p highway.env "highway.env.backup-$(date +%Y%m%d-%H%M%S)"; fi
cat > highway.env <<'EOF'
UBUNTU_IP=192.168.0.185
MORAI_IP=192.168.0.148
CONTAINER_NAME=morai-highway-gpu
IMAGE_NAME=morai-final:highway-gpu
TORCH_FLAVOR=cu121
EOF
if [ ! -f highway-test.env ]; then cp highway-test.env.example highway-test.env; fi
bash run_test.sh show
)
```

**완료 기준:** `Profile=full`, `max_speed_kph=5.0`과 설정 목록이 나온다.
기존 `highway-test.env`를 수정했다면 그 값이 나온다. `show`는 차량 제어를 보내지 않는다.

| 파일 | 저장하는 것 | 다시 켜도 남는가 |
|---|---|---|
| `highway.env` | IP, GPU 사용, 이미지·컨테이너 이름 | 남는다 |
| `highway-test.env` | 시험 종류, 속도·조향·차간 간격 등 | 남는다 |
| 원본 대회 경로 TXT | 기본 주행 경로 | 원본을 수정하지 않는다 |

두 `.env` 파일은 내 PC용이므로 Git에 올라가지 않고 일반적인 git pull로 덮어쓰지 않는다.

## 5. 처음 한 번: 환경 빌드와 GPU 연산 확인

**실행 위치: Ubuntu 호스트 터미널.** ROS·Python 라이브러리·대회 `beta_drive` 메시지와
팀원 차선 모델을 설치하고 코드를 빌드한다. 호스트에 ROS/가상환경을 따로 설치하지 않는다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh build
```

**완료 기준:** 오류 없이 명령 입력 프롬프트로 돌아온다. `ERROR`, `failed to solve`가 나오면
빌드 실패다. 그 경우 마지막 오류를 확인하고 start로 넘어가지 않는다.

빌드 성공 후:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh start
```

컨테이너만 켜진다. 아직 센서 수신/주행은 시작하지 않는다. 같은 이름이 이미 있으면 재사용한다.
예전 이미지로 만든 같은 이름의 컨테이너가 있다면 13번의 갱신 절차를 먼저 적용한다.

다음은 환경과 실제 GPU 연산 확인이다. **호스트 터미널**에 그대로 붙인다.
명령 안의 `docker exec`가 컨테이너 내부 실행을 대신한다.

```bash
(
set -euo pipefail
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
source highway.env
sudo docker --context default exec "$CONTAINER_NAME" cat /etc/os-release
sudo docker --context default exec "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint rosversion -d
sudo docker --context default exec "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
  python -c "from morai_msgs.msg import CtrlCmd; assert 'steering' in CtrlCmd.__slots__; print('MORAI_MSG_OK')"
sudo docker --context default exec "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
  python -c "import torch; assert torch.cuda.is_available(), 'CUDA unavailable'; x=torch.ones(4, device='cuda'); torch.cuda.synchronize(); print('GPU_OK', torch.cuda.get_device_name(0), 'CUDA', torch.version.cuda, 'sum', x.sum().item())"
)
```

**완료 기준:** Ubuntu `20.04`, ROS `noetic`, `MORAI_MSG_OK`,
`GPU_OK NVIDIA GeForce RTX 4090 CUDA 12.1 sum 4.0`에 해당하는 출력이 나온다.
호스트 22.04와 컨테이너 20.04가 다른 것이 정상이다.
`nvidia-smi`는 드라이버 연결, `GPU_OK`는 PyTorch 연산까지 확인한 것이다.

## 6. 처음 한 번: Windows MORAI 설정

**작업 위치: Windows MORAI 화면.** 아래 센서를 설정하고 시나리오를 실행한다.
모든 센서의 **Destination IP는 `192.168.0.185`**다. 예전 `.200`을 사용하지 않는다.

| MORAI 항목 | 설정 |
|---|---|
| Windows IP | `192.168.0.148` |
| GPS | Destination Port `3001` |
| IMU | Destination Port `4001` |
| Ego Vehicle Status | Host Port `1910`, Destination Port `1911` |
| 차선 인식 Camera 1 | Destination Port `1101` |
| YOLO 카메라 | Destination Port `1131` |
| LiDAR | Host Port `2000`, Destination Port `2001` |
| Cmd Control | MORAI 수신 `192.168.0.148:9093`, Ubuntu 송신 포트 `9094` |
| Sensor Sync | 기존 `9097/9098`; 이 launch에서는 사용하지 않음 |

GPS·IMU·카메라 Host Port는 실제 MORAI 설정을 유지한다. 두 카메라 포트는 별도 스트림이며
full 시험에는 둘 다 보내야 한다. 방향지시등 UDP는 요구하지 않는다.
Ubuntu 네트워크를 공유하므로 MORAI에 Docker의 `172.17.*` IP를 넣지 않는다.
[Docker host 네트워크 설명](https://docs.docker.com/engine/network/drivers/host/).

차선 거리 계산에 사용하는 Camera 1 보정값:

| 항목 | 현재 파일 값 |
|---|---|
| 해상도 / 수평 FOV | `1280 × 720` / `90°` |
| 차량 기준 위치 x / y / z | `1.900 / 0.000 / 1.200 m` |
| roll / pitch / yaw | `0 / 2 / 0°` |

이는 `src/detection/camera_perception/lane/cam_set.json`의 센서 1 값이다. 실제 장착값과
일치해야 차선·정지선 거리 계산이 맞는다. 그 파일 안의 옛 `127.0.0.1`/`9291`은 여기서
UDP 접속에 쓰지 않는다. 보정 JSON을 수정하면 13번대로 이미지를 다시 빌드한다.

Vehicle Controller의 **Status Initialization 반복 적용을 해제**하고 시뮬레이션을 실행한다.
차량 시작 위치는 대회 경로 위에 둔다.

## 7. 첫 실행: 제어 송신 없이 입력 확인

**Ubuntu 호스트 터미널 A**에서:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_test.sh monitor
```

로그가 계속 나오는 것이 정상이다. **이 터미널은 열어 둔다.** monitor는 저장한 시험 종류를
실행하되 MORAI 제어 송신은 끈다. ROS 내부에 `/ctrl_cmd`가 있어도 차량에 보내지 않는다.

`Ctrl+Alt+T`로 **Ubuntu 호스트 터미널 B**를 새로 열고:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh shell
```

**이제 이 터미널 B만 컨테이너 내부다.** 아래를 붙인다.

```bash
timeout -k 2s 5s rostopic echo -n 1 /localization/odometry
timeout -k 2s 5s rostopic echo -n 1 /perception/camera/lane_info
timeout -k 2s 5s rostopic echo -n 1 /perception/lidar/tracked_obstacles_map
timeout -k 2s 5s rostopic echo -n 1 /control/curvature_status
timeout -k 2s 5s rostopic echo -n 1 /control/stopline_status
timeout -k 2s 5s rostopic echo -n 1 /control/mux_status
rostopic info /ctrl_cmd
```

**확인할 것:** 위치·차선·LiDAR 메시지가 수신되고 상태의 정지 사유를 읽을 수 있어야 한다.
기본 full의 `/ctrl_cmd` Publisher는 `/control_mux` 하나다. 장애물이 없을 때 빈 목록,
정지선이 안 보일 때 `valid=false`는 가능하다. 메시지 자체가 안 나오는 것과 구분한다.
timeout 시간 만료는 정상 수신의 증거가 아니다. curvature 모드 토픽은 10-1에서 확인한다.

## 8. 첫 주행과 종료

**터미널 A**에서 Ctrl+C로 monitor를 끝내고 프롬프트가 돌아오면:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_test.sh drive
```

이때부터 MORAI에 주행 명령을 보낸다. 최초 설정의 최고속도는 **5 km/h**다.
정지선·장애물·센서 누락으로 멈추면 12번에서 이유를 확인한다.
같은 컨테이너나 다른 컨테이너에서 주행 launch를 동시에 두 개 켜지 않는다.

**종료:** MORAI에서 차량 정지/Manual 전환을 확인하고 터미널 A에서 Ctrl+C를 누른다.
ROS 노드 종료만으로 차량이 즉시 제동한다고 가정하지 않는다. 이후 터미널 A에서:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh stop
```

터미널 B의 `exit`는 B 접속만 끝낸다. 그것으로 A에서 실행한 주행이 종료되지는 않는다.

## 9. 다음에 컴퓨터를 켰을 때

**재설치·clone·build하지 않는다.** 같은 컨테이너와 설정 파일을 사용한다.
IP가 바뀌면 예전 설정을 그대로 쓸 수 없으므로 고정 IP 또는 공유기의 주소 예약을 유지한다.

**① Windows:** MORAI에서 같은 시나리오/센서를 불러온다. IP `.148`, 센서 목적지 `.185`,
센서 송신, Status Initialization 상태를 확인하고 차량을 시험 시작 위치로 둔다.

**② Ubuntu 터미널 A:**

```bash
(
set -euo pipefail
sudo systemctl start docker
ip -br -4 addr
nvidia-smi
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh start
bash run_test.sh show
bash run_test.sh monitor
)
```

IP가 `192.168.0.185`인지, show에 원하는 시험 종류와 파라미터가 나오는지 확인한다.
ROS 서버가 기억하는 것이 아니라 **`highway-test.env`에서 매번 다시 읽는 방식**이다.

**③ Ubuntu 터미널 B:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh shell
```

컨테이너 내부에서:

```bash
timeout -k 2s 5s rostopic echo -n 1 /localization/odometry
timeout -k 2s 5s rostopic echo -n 1 /control/mux_status
```

curvature 모드는 mux 대신 `/experimental/curvature_speed_command`를 확인한다.

**④ 터미널 A:** Ctrl+C로 monitor 종료 후:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_test.sh drive
```

ROS source·가상환경·roscore는 도구가 처리한다. 기존 `morai_native_env.sh`는 사용하지 않는다.
부팅만으로 차량 제어가 시작되지는 않는다. 같은 파일 설정을 재사용하는 것이며,
PI 적분값·경로 진행 상태·차로 변경 횟수는 launch를 시작할 때 새로 초기화한다.

## 10. 기능별 테스트

먼저 기존 주행을 종료한다. 아래 모드 선택은 파일에 저장되므로 다음 부팅에도 유지된다.

| `TEST_PROFILE` | 실행 기능 | 필요한 입력/장면 |
|---|---|---|
| `curvature` | 원본 경로 + 곡률 속도·조향 | GPS/IMU, 차량 없는 도로 |
| `obstacle` | 차선 + LiDAR 우회 + 변경 경로 곡률 추종 | GPS/IMU, Camera 1101, LiDAR |
| `merge` | obstacle + 요청에 따른 왼쪽 차로 진입 | 위 센서 + 옆 차로 앞뒤 차량 |
| `full` | merge + YOLO 보행자/교차로 + 정지선·신호 | 위 센서 + YOLO Camera 1131 |

### 10-1. 곡률 기반 제어만

**호스트 터미널 A:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^TEST_PROFILE=.*/TEST_PROFILE=curvature/' highway-test.env
bash run_test.sh show
bash run_test.sh drive
```

`morai_udp_ekf_purepursuit.launch`로 원본 경로를 추종한다. **카메라·LiDAR 충돌 정지와 신호 정지는 없다.**
장애물 없는 시뮬레이터 도로에서 속도와 커브 추종을 분리 확인한다.

**컨테이너 터미널 B:**

```bash
timeout -k 2s 5s rostopic echo -n 1 /experimental/curvature_value
timeout -k 2s 5s rostopic echo -n 1 /experimental/curvature_speed_command
timeout -k 2s 5s rostopic echo -n 1 /experimental/curvature_steering
timeout -k 2s 5s rostopic echo -n 1 /ctrl_cmd
```

이 모드의 `/ctrl_cmd` Publisher는 `/curvature_speed_purepursuit`다. mux 토픽은 없다.
직선에서 곡률 0은 가능하다. 커브에서도 계속 0이면 경로·위치·진행 지점을 확인한다.

### 10-2. 팀원 차선 인식 + 장애물 우회

**호스트 터미널 A:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^TEST_PROFILE=.*/TEST_PROFILE=obstacle/' highway-test.env
bash run_test.sh show
bash run_test.sh drive
```

처음에는 빈 도로에서 차선 검출, 이후 경로 앞 작은 정적 장애물로 우회 경로를 확인한다.
공간이 없거나 경계가 불확실하면 감속·정지할 수 있다. YOLO·정지선 제어는 꺼진다.
고속도로 요청 차로 변경은 꺼져도 장애물을 피하는 인접 차로 왕복 우회는 조건에 따라 가능하다.

**컨테이너 터미널 B:**

```bash
timeout -k 2s 5s rostopic echo -n 1 /perception/camera/lane_info
timeout -k 2s 5s rostopic echo -n 1 /bypass_lane_guard/plan_status
timeout -k 2s 5s rostopic echo -n 1 /avoidance_path_manager/state
timeout -k 2s 5s rostopic echo -n 1 /control/curvature_status
```

**현재 차선 기능의 의미:** 차선은 우회 가능한 범위, 차로 변경 경로 생성과 변경 후 중심 추종에 쓰인다.
일반 직진 조향에 추가하는 기존 mux의 차선 보정은 `lane_correction_enabled=false`다.
obstacle을 켠다고 그 보정까지 켜지는 것은 아니다.

### 10-3. 차량 사이로 끼어들기/왼쪽 차로 변경

**호스트 터미널 A:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^TEST_PROFILE=.*/TEST_PROFILE=merge/' highway-test.env
bash run_test.sh show
bash run_test.sh drive
```

왼쪽 인접 차로와 점선이 보이는 구간에 앞뒤 차량을 배치한다.
**컨테이너 터미널 B**에서 아래 요청을 보내고 이 터미널을 열어 둔다.

```bash
rostopic pub -r 5 /planning/highway_lane_change_request std_msgs/Bool 'data: true'
```

상태는 별도 터미널 C에서 `shell`로 접속해 확인한다.

```bash
timeout -k 2s 5s rostopic echo -n 1 /highway_lane_strategy/state
timeout -k 2s 5s rostopic echo -n 1 /perception/merge_gap/results
```

간격 부족 시 WAIT_GAP에서 원 경로를 유지하며 속도를 조절한다. 점선·인접 차로 실제 경계·
앞뒤 간격·예측 충돌 검사를 통과해야 변경한다. 요청은 강제 진입 허가가 아니다.
변경 후 안쪽 차로를 유지하다 원 경로와의 관계가 복귀 조건을 만족하면 합류한다.
일정 시간이 지나면 아무 곳에서나 무조건 복귀하는 기능은 아니다.

요청 송신을 Ctrl+C로 끝내면, 아직 착수하지 않은 요청은 약 1초 후 만료된다.
이미 변경 중이면 즉시 반대 방향으로 꺾지 않는다. 기본 변경 횟수는 1회다.
다시 시험하려면 launch 종료 → 차량/시나리오 위치 초기화 → 재실행한다.

### 10-4. 정지선·신호까지 통합 확인

**호스트 터미널 A:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^TEST_PROFILE=.*/TEST_PROFILE=full/' highway-test.env
bash run_test.sh show
bash run_test.sh drive
```

Camera 1101과 1131을 모두 송신한다. 정지선/신호가 보이는 장면에서 적색 정지 → 녹색 확인 후
출발을 시험한다. **컨테이너 터미널 B:**

```bash
timeout -k 2s 5s rostopic echo -n 1 /perception/camera/stopline
timeout -k 2s 5s rostopic echo -n 1 /perception/traffic_light/state
timeout -k 2s 5s rostopic echo -n 1 /control/stopline_status
timeout -k 2s 5s rostopic echo -n 1 /control/mux_status
```

full은 **이 안내의 통합 시험 범위**라는 뜻이다. 기존 전체 maneuver fusion의 대회 경로별
좌/우회전 신호 선택, 방향지시등 UDP, GPS blackout 차선 fallback은 이 launch에 포함되지 않는다.
신호 화살표만 보고 경로와 무관한 방향으로 회전하는 시험 명령으로 해석하지 않는다.

## 11. 속도·조향 파라미터 수정

**주행을 종료한 뒤 호스트에서 설정 파일 수정 → show 확인 → 재실행한다.**
한 번에 한두 값만 바꿔 비교한다. `rosparam set`만으로 즉시 적용된다고 가정하지 않는다.

최고속도를 10 km/h로 저장하는 예:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^MAX_SPEED_KPH=.*/MAX_SPEED_KPH=10.0/' highway-test.env
bash run_test.sh show
bash run_test.sh drive
```

급커브를 크게 돌아 나갈 때 저속에서 전방 주시 거리를 조정하는 예:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^MAX_SPEED_KPH=.*/MAX_SPEED_KPH=5.0/' highway-test.env
sed -i 's/^LOOKAHEAD_TIGHT_MIN_M=.*/LOOKAHEAD_TIGHT_MIN_M=2.0/' highway-test.env
sed -i 's/^LOOKAHEAD_CURVATURE_GAIN=.*/LOOKAHEAD_CURVATURE_GAIN=8.0/' highway-test.env
bash run_test.sh show
bash run_test.sh drive
```

다른 값을 편집하려면:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
nano highway-test.env
```

화살표로 이동해 숫자를 바꾼다. **Ctrl+O → Enter로 저장 → Ctrl+X로 종료**한다.
설정 파일은 `MAX_SPEED_KPH=10.0`, 직접 roslaunch는 `max_speed_kph:=10.0` 형식이다.
`run_test.sh`가 연결하므로 변수 이름은 그대로 두고 숫자만 바꾼다.

| 상황 | 파일 변수 / 최초값 | 조정 방법 |
|---|---|---|
| 전체 속도 상한 | `MAX_SPEED_KPH=5.0` | 5 → 10 → 15 → 30 순 시험. 실제 속도 강제 고정은 아님 |
| 커브가 빠름 | `LATERAL_ACCEL_LIMIT_MPS2=1.0` | 0.8처럼 낮춤. 곡률 속도 제한은 대략 `3.6*sqrt(a/abs(곡률))` |
| 속도 증가·감소 계획 | `MAX_ACCEL_MPS2=1.0`, `MAX_DECEL_MPS2=1.5` | 목표속도 변화와 PI 페달 변환에 함께 사용. 아래 설명 확인 |
| 속도 오차에 대한 반응 | `SPEED_KP=0.8` | 가속/제동 반복 시 0.6으로 비교. 먼저 추정속도 확인 |
| 지속적인 오차 보정 | `SPEED_KI=0.05` | 오버슈트 진단 시 0.0으로 적분을 끄고 비교 가능 |
| 누적 오차 상한 | `SPEED_INTEGRAL_LIMIT_KPH_S=10.8` | 적분이 오래 남으면 낮춰 비교 |
| 작은 속도 오차 허용 | `SPEED_ERROR_DEADBAND_KPH=0.1` | 0.2로 비교 가능. 범위 안에서는 accel/brake 둘 다 0 |
| 직선 전방 주시 | `LOOKAHEAD_MIN_M=4.0`, `LOOKAHEAD_GAIN=0.35`, `LOOKAHEAD_MAX_M=12.0` | 멀리 보면 부드럽지만 회전 진입이 늦을 수 있음 |
| 급커브 전방 주시 | `LOOKAHEAD_TIGHT_MIN_M=2.2`, `LOOKAHEAD_CURVATURE_GAIN=6.0` | min을 조금 낮추거나 gain을 높여 비교 |
| 곡률로 미리 조향 | `STEERING_FEEDFORWARD_WEIGHT=0.35` | 먼저 위치/부호/주시 거리 확인 후 소폭 조정 |
| 조향 변화 속도 | `MAX_STEERING_RATE_RAD_S=0.5` | 급하면 낮춤. 늦으면 저속에서 높여 비교. 최대 각도와 다름 |
| 끼어들기 앞/뒤 여유 | `FRONT_MIN_GAP_M=6.0`, `REAR_MIN_GAP_M=7.0` | 8.0/10.0처럼 높이면 더 넓은 간격 요구 |
| 속도별 간격 / 충돌 시간 | `TIME_HEADWAY_S=1.5`, `MIN_TTC_S=3.0` | 2.0/4.0처럼 높이면 더 여유 있게 진입 |
| 변경 횟수 | `MAX_LANE_CHANGES=1` | 초기에는 1 유지. launch 재실행 때 초기화 |
| 차선 GPU | `LANE_INFO_DEVICE=cuda` | RTX 4090은 유지. CPU 시험을 의도할 때만 cpu |
| 차선 처리 간격 | `LANE_INFO_EVERY=1` | 2는 프레임을 건너뜀. 관측 간격도 커짐 |
| 차선 신뢰도/유효시간 | `LANE_MIN_CONFIDENCE=0.45`, `LANE_INFO_TIMEOUT_S=0.6` | GPU/네트워크/보정부터 확인. 출발 목적으로 임계값을 무작정 완화하지 않음 |
| 우회 후보 평가속도 | `EVALUATION_SPEED_MPS=2.0` | 후보 검사값. 실제 목표속도나 km/h가 아님 |
| 정지선 기준점 | `STOPLINE_FRONT_REFERENCE_OFFSET_M=3.845` | 후륜 기준점→앞 범퍼 가정. 실제 원점/카메라 보정 확인 후 조절 |
| LiDAR 장착 변환 | `LIDAR_X_M`, `LIDAR_Y_M`, `LIDAR_Z_M`, `LIDAR_YAW_DEG` | 실제 base_link 기준 위치(m)/yaw(도). 최초 0은 실측 보정값이 아님 |

차간 간격 값은 끼어들기 후보용이다. 별도 앞차 추종 간격 등이 모두 함께 바뀌는 것은 아니다.
curvature 시험에는 차선·차간 간격 값이 적용되지 않는다.

`MAX_ACCEL_MPS2`는 실제 차량 가속도를 그 숫자로 보장하지 않는다. 현재 PI는
`오차=(목표km/h-추정km/h)/3.6`, `요청가속도=Kp*오차+Ki*누적오차`를 계산한 뒤
양수면 `accel=요청가속도/MAX_ACCEL_MPS2`, 음수면 `brake=-요청가속도/MAX_DECEL_MPS2`로 정규화한다.
같은 요청가속도에서는 MAX_ACCEL을 낮추면 accel이 오히려 커질 수 있다. 동시에 속도 계획도
변하므로 이 값 하나만으로 부드러움을 단정하지 않는다.

통합 시험의 `/control/curvature_status`에서 `target_speed_kph`와 `measured_speed_kph`를 비교한다.
후자는 odometry x/y 속도 크기 × 3.6이라는 추정치다. 독립 비교에는 MORAI 화면 속도를 함께 기록한다.
`longlCmdType=1`은 accel/brake 제어이므로 `/ctrl_cmd.velocity=0`만으로 고장이라고 판단하지 않는다.

설정이 잘 맞으면 **호스트에서** 백업한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
cp -p highway-test.env "highway-test.env.good-$(date +%Y%m%d-%H%M%S)"
```

## 12. 안 움직일 때 확인

**컨테이너 터미널 B**, full/obstacle/merge 기준:

```bash
timeout -k 2s 5s rostopic echo -n 1 /control/mux_status
timeout -k 2s 5s rostopic echo -n 1 /control/stopline_status
timeout -k 2s 5s rostopic echo -n 1 /highway_lane_strategy/state
```

| 출력/증상 | 할 일 |
|---|---|
| `brake: 1.0` | 위 상태 토픽의 정지 사유 확인 |
| `roi_lidar_stale` | LiDAR 송신/목적지 .185:2001 확인 |
| `odometry_missing_or_invalid`, `odom_stale` | GPS .185:3001, IMU .185:4001, odometry 확인 |
| `lane_info_missing_or_stale` | Camera .185:1101, GPU_OK, 보정 확인 |
| `target_lane_far_boundary_missing` | 옆 차로 바깥 경계가 관측되는 장면인지 확인 |
| `managed_trajectory_stale` | 계획기 상태/노드 오류/처리 지연 확인 |
| 정지선 HOLD | 신호 토픽과 stopline_status 사유 확인 |
| AV-External로 안 바뀜 | monitor가 아닌 drive인지, MORAI .148:9093 확인 |
| 가속 명령인데 속도 0 | 시뮬레이션 실행/Status Initialization/기어/제어 상태 확인 |

센서 메시지가 없으면 **호스트**에서 패킷 확인:

```bash
sudo -v
sudo timeout --signal=INT --kill-after=2s 8s tcpdump -nn -i any \
  'udp and host 192.168.0.148 and (port 3001 or port 4001 or port 1911 or port 1101 or port 1131 or port 2001 or port 9093)'
```

`listening on any`는 대기 시작이다. 아래 IP/포트 패킷 줄이 있어야 수신 증거다.
tcpdump가 호스트에 없으면 `sudo apt-get install -y tcpdump`로 설치한다.
Ubuntu 방화벽이 켜져 있고 센서 UDP를 막을 때만:

```bash
sudo ufw status
sudo ufw allow from 192.168.0.148 to 192.168.0.185 port 1101,1131,1911,2001,3001,4001 proto udp
```

nvidia runtime 오류는 3번, Docker 연결 오류는 `sudo systemctl start docker`와
`sudo docker --context default info`부터 확인한다. Docker 그룹 변경/재로그인은 필수가 아니다.
실행 도구가 필요하면 sudo를 사용한다.

## 13. 코드 또는 IP를 업데이트할 때

**파라미터 숫자만 수정:** `highway-test.env` 수정 → launch 종료 → `run_test.sh drive`.
이미지 재빌드는 필요 없다.

**Python·모델·보정 JSON·launch 변경:** 컨테이너에는 빌드 당시 코드가 들어 있다.
호스트에서 git pull만 해서는 컨테이너 코드가 바뀌지 않는다. 주행 종료 후 **호스트**에서:

```bash
(
set -euo pipefail
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh stop
git -C "$HOME/AutoVehicle" pull --ff-only origin final_ws
cp -p highway.env "highway.env.backup-$(date +%Y%m%d-%H%M%S)"
NEW_CONTAINER_NAME="morai-highway-gpu-$(date +%Y%m%d-%H%M%S)"
sed -i "s/^CONTAINER_NAME=.*/CONTAINER_NAME=$NEW_CONTAINER_NAME/" highway.env
bash run_highway.sh build
bash run_highway.sh start
bash run_test.sh show
)
```

새 이름을 저장했으므로 다음 부팅에도 그 컨테이너를 쓴다. 이전 컨테이너는 정지 상태로 보관한다.
`highway-test.env`는 유지된다. 5번 GPU 연산과 monitor를 다시 확인한다.
빌드 실패 시 오류 수정 후 build부터 재시도한다.

**IP 변경:** `nano highway.env`로 실제 IP를 수정하고 MORAI 센서 목적지도 수정한다.
ROS_IP는 컨테이너 생성 시 저장되므로 기존 컨테이너를 stop한 뒤 새 이름으로 start한다.
IP만 변경했다면 build는 생략할 수 있다. 같은 LAN/IP 유지는 PC·공유기에서 설정한다.

## 14. 기록 보관과 검증 범위

주행 중 **컨테이너 터미널 B**에서 기록한다. Ctrl+C로 기록을 끝낸다.

```bash
mkdir -p /root/.ros/bags
rosbag record -O "/root/.ros/bags/highway_$(date +%Y%m%d-%H%M%S).bag" \
  /localization/odometry /perception/camera/lane_info /perception/camera/stopline \
  /perception/lidar/tracked_obstacles_map /perception/merge_gap/results \
  /avoidance_path_manager/active_path /highway_lane_strategy/active_path \
  /highway_lane_strategy/state /control/curvature_status /control/stopline_status \
  /control/mux_status /ctrl_cmd
```

기록을 호스트 홈으로 복사하려면 **호스트 터미널**에서:

```bash
(
set -euo pipefail
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
source highway.env
mkdir -p "$HOME/morai-test-logs"
sudo docker --context default cp "$CONTAINER_NAME:/root/.ros/bags/." "$HOME/morai-test-logs/"
cp highway.env highway-test.env "$HOME/morai-test-logs/"
sudo docker --context default exec "$CONTAINER_NAME" cat /opt/morai-build/code-revision.txt \
  > "$HOME/morai-test-logs/code-revision.txt"
)
```

기록은 `morai-highway-logs` Docker 볼륨에 남는다. 파일 관리자에서 호스트의
`$HOME/morai-test-logs`를 열면 복사한 기록과 설정을 볼 수 있다.

기존 인식·제어 오프라인 회귀 테스트는 492개 통과했다. 실제 Docker/ROS/MORAI 성공을 뜻하지는 않는다.
빌드는 catkin·메시지·모델 CPU 추론·회귀·launch 해석을 검사한다. GPU 연산과 시나리오 주행은
새 Ubuntu에서 확인한다. 이 실행 환경은 Noetic/Focal이며 대회 Docker 허용 여부는 운영 안내를 따른다.
코드 연결은 [팀원 코드 통합 설명](HIGHWAY_TEAM_INTEGRATION_KO.md)에 정리되어 있다.
