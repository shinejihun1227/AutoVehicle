# Windows MORAI + Ubuntu Docker 처음 설정과 통합 주행 시험

Ubuntu PC IP: **192.168.0.185**. Windows MORAI IP는 마지막에 확인한
**192.168.0.148**을 기본값으로 사용한다. Windows `ipconfig`에서 달라졌으면
`highway.env`의 MORAI_IP와 MORAI 네트워크 설정을 함께 변경한다.

Ubuntu 호스트가 22.04여도 컨테이너 내부는 Ubuntu 20.04 + ROS1 Noetic이다.
아래 절차는 Ubuntu amd64 PC 기준이며, 호스트에 ROS 또는 기존 morai-final-venv를
설치할 필요가 없다. 규정의 OS/ROS 실행 환경은 이 구성으로 맞출 수 있지만
Docker 사용 자체가 허용되는지는 대회 규정 원문/운영 안내로 확인해야 한다.

## 1. 새 Ubuntu에서 버전과 GPU 확인

```bash
cat /etc/os-release
uname -m
ip -br -4 addr
lspci | grep -Ei 'vga|3d|display'
nvidia-smi
```

`lspci`에 NVIDIA가 있으면 NVIDIA GPU가 있다. `nvidia-smi`가 없거나 실패하는 것은
드라이버 문제일 수도 있으므로 GPU가 없다는 뜻은 아니다. `nvidia-smi`가 정상이고
GPU 이름·드라이버가 표시되면 GPU 컨테이너 구성을 진행할 수 있다.
CPU로도 구성할 수 있으나 실시간 추론 속도는 이 문서에서 보장하지 않는다.

## 2. Ubuntu 호스트에 Docker Engine 설치

신규 Ubuntu 22.04 기준. 다음 명령은 **컨테이너 밖 Ubuntu 터미널**에서 실행한다.
이미 Docker가 정상 동작한다면 설치를 반복하지 않고 다음 단계로 간다.
설치 출처: [Docker 공식 Ubuntu 설치 안내](https://docs.docker.com/engine/install/ubuntu/).

```bash
sudo apt update
sudo apt install -y ca-certificates curl git
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

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo docker run --rm hello-world
```

기존 `docker.io`, Podman 등과 충돌한다는 오류가 있으면 오류의 패키지명을 확인하고
공식 설치 안내의 충돌 패키지 절차를 적용한다. 아래 실행 스크립트는 docker 권한이
없으면 sudo를 사용하므로 Docker 그룹 변경이나 재로그인은 필수가 아니다.

## 3. Ubuntu 홈에 코드 가져오기

```bash
cd "$HOME"
git clone --branch final_ws --single-branch https://github.com/shinejihun1227/AutoVehicle.git
cd "$HOME/AutoVehicle"
git log -1 --oneline
test -f morai_ws/src/bringup/morai_bringup/launch/final_ws_highway_bringup.launch

cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
cp -n highway.env.example highway.env
nano highway.env
```

내용은 다음과 같이 둔다.

```bash
UBUNTU_IP=192.168.0.185
MORAI_IP=192.168.0.148
CONTAINER_NAME=morai-highway
IMAGE_NAME=morai-final:highway
TORCH_FLAVOR=cpu
```

`highway.env`는 PC별 설정이며 Git에서 제외된다. 기존 clone이면 무조건 재복제하지 말고
`git status`로 수정 내용을 확인한 후 `git pull --ff-only origin final_ws`로 갱신한다.

## 4. NVIDIA GPU를 쓸 때만 추가 구성

`nvidia-smi`가 호스트에서 먼저 정상 동작해야 한다. GPU/드라이버가 확인되지 않은 동안은
`TORCH_FLAVOR=cpu`로 빌드할 수 있다. NVIDIA GPU가 확인되면
[NVIDIA Container Toolkit 공식 설치 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)에 따라
Toolkit 저장소와 패키지를 설치한 후 아래처럼 Docker runtime을 설정한다.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

이후 `highway.env`에서 `TORCH_FLAVOR=cu121`로 변경한다. 이미 CPU 컨테이너를 생성했다면
기존 컨테이너는 보관하고 `CONTAINER_NAME=morai-highway-gpu`처럼 새 이름을 사용한다.
드라이버·GPU가 CUDA 12.1 및 이 이미지의 PyTorch 조합을 지원해야 한다.

## 5. 이미지 빌드와 컨테이너 생성

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh build
bash run_highway.sh start
bash run_highway.sh shell
```

빌드 단계에서 자동으로 설치/검사하는 내용:

- Ubuntu 20.04 + ROS Noetic + Python 가상환경 및 인식 라이브러리.
- 대회 `morai_msgs`의 **beta_drive** 브랜치 SHA를 고정해 clone.
- 프로젝트의 공통 메시지와 패키지 catkin 빌드, `CtrlCmd.steering` 계약 검사.
- 팀원 차선 모델의 고정 SHA256 다운로드, 모델 로드 및 실제 빈 영상 추론.
- 회귀 테스트와 launch include 해석.

시작만 해서는 센서/제어가 실행되지 않는다. `shell`은 위 환경을 source한 컨테이너
터미널을 연다. 예전 `$HOME/morai_native_env.sh`를 이 안에서 source하지 않는다.

컨테이너 내부 확인:

```bash
cat /etc/os-release
rosversion -d
python -c "from morai_msgs.msg import CtrlCmd; print(CtrlCmd.__slots__)"
python -c "import torch; print(torch.__version__); print('CUDA:', torch.cuda.is_available())"
echo "$MORAI_WS"
```

기대: Ubuntu 20.04, noetic, `steering` 필드, `/opt/AutoVehicle/morai_ws`.
GPU로 빌드/실행했다면 CUDA가 True여야 한다. 이미지 안에서 보고 있는 코드가 정확히
어느 버전인지 `cat /opt/morai-build/code-revision.txt`로 확인한다.
`exit`는 이 셸만 나간다. 다른 Ubuntu 터미널에서도 `bash run_highway.sh shell`로 다시 접속한다.

## 6. Windows MORAI와 네트워크 연결

두 PC를 같은 LAN에 연결한다. 센서 데이터는 Windows → Ubuntu, 차량 제어는 Ubuntu → Windows다.
Linux `--network host`를 사용해 컨테이너가 Ubuntu의 네트워크를 공유하므로
`-p` 포트 매핑과 `172.17.*` Docker 주소를 MORAI에 넣을 필요가 없다.
근거: [Docker host 네트워크 설명](https://docs.docker.com/engine/network/drivers/host/).

| 항목 | Windows MORAI 쪽 | Ubuntu 컨테이너 쪽 |
|---|---|---|
| Windows IP / 제어 목적지 | 192.168.0.148 | `morai_host_ip` |
| 센서 Destination IP | **192.168.0.185** | UDP bind는 0.0.0.0 |
| GPS 송신 Destination Port | 3001 | `gps_port` |
| IMU 송신 Destination Port | 4001 | `imu_port` |
| Ego Status | Host 1910, Destination 1911 | `ego_status_port=1911` |
| 차선용 Camera 1 | Destination 1101 | `lane_info_port` |
| YOLO용 카메라 | Destination 1131 | `yolo_port` |
| LiDAR | Host 2000, Destination 2001 | `roi_lidar_host_port` / `roi_lidar_port` |
| Cmd Control | Windows 수신 9093 | Ubuntu 송신/bind 9094 |
| Sensor Sync | 기존 9097/9098 | 이 launch에서 사용하지 않음 |

GPS/IMU/카메라의 Windows Host Port는 실제 MORAI 설정을 유지하고 Destination Port만
표와 맞춘다. Sensor Sync는 방향지시등 포트가 아니다. 이 launch는 방향지시등 UDP를 보내지 않는다.
MORAI Vehicle Controller의 **Status Initialization 반복 적용을 꺼 두고** 시뮬레이션을 진행한다.

Ubuntu 연결/UDP 진단:

```bash
ip route get 192.168.0.148
ping -c 3 192.168.0.148
sudo -v
sudo timeout --signal=INT --kill-after=2s 8s tcpdump -nn -i any \
  'udp and host 192.168.0.148 and (port 3001 or port 4001 or port 1911 or port 1101 or port 1131 or port 2001 or port 9093)'
```

패킷 없이 `listening on any`만 나오면 아직 센서 데이터 수신 증거가 없다. Windows 센서 송신,
목적지 IP, 방화벽을 먼저 확인한다. ping이 차단되어도 UDP가 통과하는 경우가 있으므로 tcpdump를 함께 본다.
Ubuntu UFW가 켜져 있을 때만 필요한 UDP를 허용한다.

```bash
sudo ufw status
sudo ufw allow from 192.168.0.148 to 192.168.0.185 port 1101,1131,1911,2001,3001,4001 proto udp
```

Windows에서 Cmd Control 수신이 막히면 관리자 PowerShell에서 필요한 범위만 연다.

```powershell
New-NetFirewallRule -DisplayName 'MORAI control from Ubuntu' -Direction Inbound -Action Allow -Protocol UDP -LocalPort 9093 -RemoteAddress 192.168.0.185
```

## 7. 송신 없이 전체 입력 확인

먼저 MORAI 시나리오·센서를 켠다. 다른 주행 launch를 동시에 실행하지 않는다.
Ubuntu 호스트 터미널 A:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh monitor
```

Ubuntu 호스트 터미널 B에서 컨테이너 셸을 열고 검사한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh shell
```

이제 컨테이너 안에서:

```bash
timeout -k 2s 5s rostopic echo -n 1 /localization/odometry
timeout -k 2s 5s rostopic echo -n 1 /perception/camera/lane_info
timeout -k 2s 5s rostopic echo -n 1 /detection/lane
timeout -k 2s 5s rostopic echo -n 1 /perception/camera/stopline
timeout -k 2s 5s rostopic echo -n 1 /perception/lidar/tracked_obstacles_map
timeout -k 2s 5s rostopic echo -n 1 /avoidance_path_manager/state
timeout -k 2s 5s rostopic echo -n 1 /highway_lane_strategy/state
timeout -k 2s 5s rostopic echo -n 1 /control/stopline_status
timeout -k 2s 5s rostopic echo -n 1 /control/mux_status
timeout -k 2s 5s rostopic echo -n 1 /control/curvature_status
rostopic info /ctrl_cmd
```

`/ctrl_cmd`의 Publisher는 `/control_mux` 하나여야 한다. 대상이 보이지 않는 프레임의
`stopline.valid=false`나 빈 장애물 배열은 정상일 수 있다. 토픽 자체가 끊기는 것과 구분한다.
정지 이유는 `roi_lidar_stale`, `managed_trajectory_stale`, `predicted_collision_id_*`,
`target_lane_far_boundary_missing`, `lane_info_missing_or_stale`처럼 각 상태에 표시된다.

## 8. 실제 MORAI 제어 켜기

터미널 A에서 monitor를 Ctrl+C로 종료한 다음:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh drive max_speed_kph:=5.0
```

차선 인식과 장애물 회피를 먼저 분리 시험하려면 YOLO·정지선 정지를 명시적으로 끌 수 있다.
LiDAR와 경로 충돌 검사는 유지된다.

```bash
bash run_highway.sh drive max_speed_kph:=5.0 \
  enable_yolo:=false enable_stopline_control:=false
```

정지선·신호까지 시험할 때는 첫 번째 기본 명령을 사용한다. YOLO를 끈 채 정지선 제어만
켜면 유효한 녹색 신호가 없어 정지선 HOLD가 유지될 수 있다. 브레이크를 강제로 0으로
발행해서 풀지 않고 `/control/stopline_status`의 사유를 확인한다.

1. 직선·곡선, 빈 차로: 경로와 조향 부호, 목표/추정 속도 확인.
2. 작은 정적 장애물: 차선 안 우회. 공간이 없으면 감속·정지 또는 검증된 인접 차로 우회.
3. 왼쪽 앞차·뒤차를 배치: 요청 전에는 원 경로 유지, 간격 부족 시 WAIT_GAP.
4. 안전한 간격 형성: 요청을 보내면 변경 → INNER_HOLD → 원 경로 합류 시 복귀.
5. 변경 도중 뒤차 접근·전방 끼어듦·센서 중단: 상태와 최종 제동을 확인.
6. 정지선 + 적색/녹색: 거리 감소 → 정지 → 녹색 확인 후 출발.

고속도로 요청(컨테이너 터미널 B, 해당 시험 구간에서 실행):

```bash
rostopic pub -r 5 /planning/highway_lane_change_request std_msgs/Bool 'data: true'
```

## 9. 파라미터 변경과 기록

파라미터는 대부분 시작할 때 읽으므로 launch를 Ctrl+C로 끈 후 옵션을 바꿔 재실행한다.
`rosparam set`만으로 즉시 바뀐다고 가정하지 않는다.

```bash
bash run_highway.sh drive \
  max_speed_kph:=15.0 lateral_accel_limit_mps2:=1.0 \
  max_accel_mps2:=1.0 max_decel_mps2:=1.5 \
  front_min_gap_m:=8.0 rear_min_gap_m:=10.0 time_headway_s:=2.0 min_ttc_s:=4.0
```

| 상황 | 먼저 조정할 값 |
|---|---|
| 전체 속도 | `max_speed_kph` (기본 30, 초기 시험 5) |
| 곡선이 빠름 | `lateral_accel_limit_mps2` 낮추기 |
| 급커브를 크게 돌아 나감 | `lookahead_curvature_gain` 높이기, `lookahead_tight_min_m` 낮추기 |
| 조향이 급함 | `max_steering_rate_rad_s` 낮추기. 너무 낮으면 커브 추종이 늦어짐 |
| 진입 공간을 더 넉넉하게 | `front_min_gap_m`, `rear_min_gap_m`, `time_headway_s`, `min_ttc_s` 높이기 |
| 인식 끊김 | GPU/해상도/처리시간 확인 후 `lane_info_timeout_s`; 원인 확인 없이 크게 늘리지 않기 |
| 차선 변경을 더 수행할 계획 | `max_lane_changes` (기본 1), 미션/경로 검증 후 변경 |
| 정지 위치가 앞뒤로 어긋남 | 카메라 장착값과 `stopline_front_reference_offset_m` 확인 |

컨테이너 안에서 rosbag 기록:

```bash
mkdir -p /root/.ros/bags
rosbag record -O /root/.ros/bags/highway_test.bag \
  /localization/odometry /perception/camera/lane_info /perception/camera/stopline \
  /perception/lidar/tracked_obstacles_map /perception/merge_gap/results \
  /avoidance_path_manager/active_path /highway_lane_strategy/active_path \
  /highway_lane_strategy/state /control/curvature_status /control/stopline_status \
  /control/mux_status /ctrl_cmd
```

호스트로 복사:

```bash
sudo docker cp morai-highway:/root/.ros/bags/highway_test.bag "$HOME/highway_test.bag"
```

## 10. 중지·재접속·코드 갱신

주행 중지는 주행 터미널 Ctrl+C이며, MORAI의 제어 정지/Manual 전환도 확인한다.
컨테이너 셸에서 `exit`하거나 다른 터미널을 닫는 것으로 주행이 멈추는 것은 아니다.
컨테이너 전체 중지: `bash run_highway.sh stop`. 다시 열기: `start`, `shell` 순서.

이미지는 빌드 시점의 코드를 담는다. 호스트에서 git pull만 해도 기존 컨테이너의 코드가
바뀌지는 않는다. 갱신 후 `build`, `stop`을 실행하고 `highway.env`의 컨테이너 이름을
새 이름으로 바꾼 뒤 `start`한다. 예전 컨테이너는 보관된다.
로그는 Docker named volume에 저장되어 컨테이너 재생성과 분리된다.

Windows VS Code의 Remote-SSH로 `192.168.0.185`의 Ubuntu에 접속해
`~/AutoVehicle`을 열고 편집·터미널 실행을 할 수 있다. 실제 실행 코드는 위 빌드 절차로
이미지에 반영한다. Docker/ROS 설치와 모델 검사 통과가 실제 주행 성능 검증을 대신하지는 않는다.
