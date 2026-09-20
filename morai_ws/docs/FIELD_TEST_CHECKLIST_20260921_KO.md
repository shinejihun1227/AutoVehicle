# MORAI 현장 테스트 절차 — 2026-09-21

> **적용 범위:** 이 문서는 2026-09-21에 로컬 작업 폴더에 반영한 ROI `cb41b5f` 통합본의 시험 절차다.
> 문서가 GitHub에 있어도 해당 로컬 코드와 모델 검증 파일이 모두 원격에 반영되었다는 뜻은 아니다.
> T01에서 실제 Ubuntu 작업 폴더의 실행 파일·모델·검사 결과를 확인한 뒤 진행한다.
> Ubuntu ROS Noetic의 Bash 명령을 기준으로 작성했다. Docker를 쓰면 ROS가 설치된 컨테이너 안에서 실행하고 `MORAI_WS`를 컨테이너 경로로 바꾼다.

## 빠른 이동

- [시험 순서와 현재 상태](#test-order)
- [T01 코드·빌드·모델 확인](#section-1)
- [T02 센서 수신과 제어 OFF](#section-2)
- [T03 Cam4 보정과 신호 연결](#section-3)
- [T04~T05 출발·속도 상승·곡률 단독 비교](#section-4)
- [T06 신호등·정지선·재출발](#section-5)
- [T07 인식 소실과 T09 선택 시험](#section-7)
- [T08 최신 고속도로 기능](#section-6)
- [로그 저장과 정지 원인 찾기](#section-8)
- [결과 기록표와 종료 순서](#section-9)

<a id="test-order"></a>

## 시험 순서와 현재 상태

곡률 기반 주행과 신호등 정지선 제어 및 최신 ROI 기능 검증

작성 기준 2026년 9월 21일   대상 Ubuntu ROS Noetic와 MORAI 시뮬레이터

먼저 출발·속도 상승·신호 정지 문제를 확인한 뒤 최신 고속도로 기능을 시험한다. 각 시험은 같은 시작 위치와 설정으로 반복하고, 이상이 생긴 시점의 화면과 로그를 함께 남긴다. **첫날은 T01~T06부터 진행하고, 완료한 항목을 아래에 표시한다.**

### 먼저 알아둘 현재 상태

ROI 기준 커밋은 `cb41b5f`이다. 모델 4개는 원본과 해시가 같으며 오프라인 테스트 398개가 통과했다. ROS 빌드·실제 GPU 추론·MORAI 주행은 현장에서 확인해야 한다. Cam4 설정값은 입력되어 있으나 `calibrated: false`이며, 영상 투영 확인이 끝나기 전에는 교차로 통과 시험을 완료할 수 없다.

| 순서 | 시험 | 진행 조건 |
| --- | --- | --- |
| T01 | 코드 복사와 빌드 및 모델 검사 | 가장 먼저 실시 |
| T02 | 제어 OFF 상태의 센서 수신 | T01 통과 후 |
| T03 | Cam4 투영과 신호 연결 확인 | 차량 정지 상태에서 실시 |
| T04 | 3 km/h 출발과 직선 속도 상승 | 센서·위치 확인 후 |
| T05 | 7.2 km/h 곡률 단독과 통합 비교 | T04 통과 후 같은 구간에서 비교 |
| T06 | 적색 정지와 녹색 재출발 | T03 통과 및 보정 설정 적용 후 |
| T07 | 잘못된 신호와 인식 소실 | T06 통과 후 저속에서 |
| T08 | 고속도로 차선 변경과 우회 | 기본 시험 완료 후 별도 launch |
| T09 | GPS 소실과 회전교차로 | 선택 시험 서로 다른 실행 구성 |


- [ ] T01 빌드·메시지·회귀 검사·모델 검사 통과
- [ ] T02 제어 OFF에서 센서 수신과 경로 일치 확인
- [ ] T03 Cam4 투영 확인 및 개인 보정 설정 적용
- [ ] T04 3 km/h 출발·재출발 확인
- [ ] T05 7.2 km/h에서 곡률 단독과 통합 속도 비교
- [ ] T06 적색 정지·정지 유지·녹색 재출발을 각 3회 확인
- [ ] T07 다른 신호·인식 소실 시 잘못 출발하지 않는지 확인
- [ ] T08 고속도로 차선 변경·우회·차선 복구 확인
- [ ] T09 필요한 GPS 소실·회전교차로 선택 시험 수행
- [ ] 시험별 bag·영상·설정·결과표 저장

### 실행 파일을 구분한다

기본 시험은 `final_ws_native_no_lamps.launch`를 사용한다. 곡률 주행·경로별 신호·정지선·GPS fallback을 포함하며 차량 방향지시등 송신만 제외한다. 최신 고속도로 시험은 `final_ws_highway_bringup.launch`를 사용한다. 후자는 기본 구성의 방향별 교차로 허가와 GPS fallback을 포함하지 않는다.

시작 전 기록: 시험자 __________  날짜 __________  MORAI IP __________  Ubuntu IP __________

지도와 시나리오 __________  차량 모델 __________  시험 결과 폴더 __________

모든 주행 시험은 MORAI에서 실시한다. 이상 시 MORAI에서 먼저 차량을 정지하거나 일시정지하고 외부 제어를 해제한 뒤 launch를 종료한다. Ctrl+C만으로 차량이 즉시 멈춘다고 가정하지 않는다.

<a id="section-1"></a>

## 1 코드와 모델 준비

**T01**  현재 Windows 작업 폴더의 최신 `morai_ws`를 Ubuntu에 반영한다. 수정 내용은 원격 저장소나 다른 PC에 자동 전송되지 않았다. `git pull`만으로 이번 수정이 들어왔다고 판단하지 않는다. Ubuntu의 개인 설정·기존 수정은 보관하고, Windows의 `build`·`devel`은 옮기지 않는다.

### Ubuntu Bash에서 빌드한다

```bash
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
source /opt/ros/noetic/setup.bash
if [ -f "$HOME/morai-final-venv/bin/activate" ]; then
  source "$HOME/morai-final-venv/bin/activate"
fi
cd "$MORAI_WS"
catkin_make -j2 -l2 --force-cmake \
  -DPYTHON_EXECUTABLE="$(command -v python3)"
```

빌드가 성공한 뒤에만 아래를 실행한다. ROS와 모델 의존성이 없는 새 PC는 저장소의 [Ubuntu 최초 설치 안내](UBUNTU_NATIVE_FIRST_SETUP_KO.md)의 설치 절차를 먼저 완료한다.

```bash
source "$MORAI_WS/devel/setup.bash"
rospack find morai_bringup
rospack find camera_perception
python3 docker/final_ws/check_morai_messages.py
python3 -B docker/final_ws/run_regression.py
python3 docker/final_ws/smoke_models.py \
  --workspace "$MORAI_WS" --device cpu
```

GPU를 사용할 장비는 같은 Python 환경에서 `--device cuda` 검사도 수행한다. CPU 검사 통과가 실시간 추론 속도를 보장하지는 않는다. `rospack find` 결과는 이번 Ubuntu workspace를 가리켜야 한다.

### 최신 코드 표시와 모델 해시를 대조한다

```bash
python3 - <<'PY'
import hashlib, json, os
from pathlib import Path
w = Path(os.environ["MORAI_WS"])
m = json.loads((w / "config/roi_upstream_manifest.json").read_text())
print("ROI", m["commit"])
for entry in m["models"]:
    f = w / entry["workspace_path"]
    actual = hashlib.sha256(f.read_bytes()).hexdigest()
    assert actual == entry["sha256"], "MODEL_MISMATCH: " + str(f)
    print("MODEL_OK", f.name)
PY
```

통과 기준: 빌드 성공, `MORAI_MESSAGES_OK`, 회귀 검사 6개 묶음 통과, `OFFLINE_SMOKE_PASS`, 모델 4개 `MODEL_OK`. 문서 작성 시 회귀 검사는 398개다. 커밋 표시는 `cb41b5fa4598d680d715cde9a0deaf6a5a1035f3`이다. 표시 파일만 존재하는 것은 전체 소스 반영의 증거가 아니므로 새 launch 파일도 확인한다.

실패 시 저장: 빌드 마지막 오류 전체, Python 경로와 버전, 모델 검사 출력. 실패한 상태로 주행 단계로 넘어가지 않는다.

<a id="section-2"></a>

## 2 센서 수신과 제어 OFF 확인

**T02**  아래 환경 블록은 새 터미널마다 실행한다. ROS 노드는 모두 Ubuntu 한 PC에서 실행하고 MORAI 센서는 UDP로 받는 구성이다. MORAI의 UDP 목적지는 실제 Ubuntu IP로 지정한다. 아래 ROS의 `127.0.0.1`을 센서 목적지 IP로 넣으면 안 된다.

```bash
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
source /opt/ros/noetic/setup.bash
if [ -f "$HOME/morai-final-venv/bin/activate" ]; then
  source "$HOME/morai-final-venv/bin/activate"
fi
source "$MORAI_WS/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export TEST_DIR="$HOME/morai-field/$(date +%Y%m%d)"
mkdir -p "$TEST_DIR"
cd "$MORAI_WS"
```

| 항목 | 현재 시험값 또는 확인 사항 |
| --- | --- |
| 지도와 경로 | R_KR_PR_K-city_2025와 해당 global path 사용. RViz map 프레임에서 시작 위치·방향 확인 |
| UDP 수신 | 차선 1101 / Cam4 YOLO 1131 / GPS 3001 / IMU 4001 / Ego 1911 |
| LiDAR와 제어 | LiDAR 2000 → 2001 / 제어 Ubuntu 9094 → MORAI 9093 |
| MORAI 주소 | 명령의 192.168.0.161은 현재 예시. 현장 MORAI IP가 다르면 바꾼다 |

```bash
roslaunch morai_bringup final_ws_native_no_lamps.launch \
  workspace_path:="$MORAI_WS" \
  morai_host_ip:=192.168.0.161 ego_status_port:=1911 \
  enable_control:=false max_speed_kph:=3.0 \
  2>&1 | tee "$TEST_DIR/off_$(date +%H%M%S).log"
```

다른 터미널에서 10초씩 수신 상태를 확인한다. `timeout`의 종료 코드 124는 지정된 관찰 시간이 끝났다는 뜻이다.

```bash
timeout 10s rostopic hz /localization/odometry
timeout 10s rostopic hz /perception/camera/stopline
timeout 10s rostopic hz /detection/traffic_light
timeout 10s rostopic hz /perception/camera/lane_info
rostopic echo -n 1 /localization/sensor_quality
rostopic echo -n 1 /control/maneuver_status
rostopic info /ctrl_cmd
```

통과 기준: 영상·위치·LiDAR 결과가 계속 갱신되고 오류로 노드가 종료되지 않는다. `/ctrl_cmd` 발행자는 선택된 mux 하나여야 한다. `reference_path_match`는 true여야 한다. 카메라 UDP 1101을 받는 프로세스도 하나여야 한다. 정지선이 안 보일 때 `valid=false`인 것은 정상이며 메시지 수신 자체가 끊긴 것과 구분한다.

**제어 OFF에서는 차량이 움직이지 않는 것이 정상이다.** nominal 발행이 꺼져 있어 `nominal_stale_or_not_type1` 같은 정지 이유가 표시될 수도 있다. 이 단계에서는 최종 주행 허가보다 센서 수신과 경로·영상 좌표를 확인한다.

<a id="section-3"></a>

## 3 Cam4 보정과 신호 연결 확인

**T03**  Cam4는 신호등 YOLO용 UDP 1131 카메라다. 차선과 정지선에 쓰는 Cam1 설정과 별도로 확인한다. `calibrated=false` 상태에서 녹색인데 출발하지 않는 현상을 가속 제어 오류로 판정하지 않는다.

| 확인 항목 | Cam4 기준값 |
| --- | --- |
| 위치 m | x 3.43 / y 0.01 / z 0.61 |
| 방향과 영상 | roll pitch yaw 모두 0° / 640 × 480 / 수평 FOV 90° |
| 좌표 기준 | 코드는 base_link x 전방 y 좌측 z 위를 사용. MORAI 장착 기준점과 일치하는지 확인 |

정지선 약 30 m·20 m·10 m 앞에서 각각 정지한 뒤 원본 Cam4 화면, YOLO 박스, 차량 위치와 방향을 기록한다. 순간이동으로 위치를 바꿨다면 주행 launch를 재시작해 경로 진행 상태를 초기화한다. 실제 간격은 현장에서 측정해 기록한다.

### 제어 OFF에서 투영 픽셀을 확인한다

다음은 정지 상태의 기하 확인용 읽기 전용 명령이다. 기본 설정값으로 예상 신호 픽셀과 검출 박스를 출력하며 `calibrated`를 바꾸지 않는다. 교차로 접근 경로에 있어 `event.signal_points`가 있어야 한다. 영상 시각과 자세의 동기화 검증은 로그로 별도 확인한다.

```bash
python3 - <<'PY'
import json, math, os, rospy, yaml
from pathlib import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common.msg import ObjectInfoArray
from turn_signal_controller.signal_association import project_signal
rospy.init_node("cam4_stationary_check", anonymous=True)
w = Path(os.environ["MORAI_WS"])
cfg = w / "src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml"
camera = yaml.safe_load(cfg.read_text())["signal_camera"]
status = json.loads(rospy.wait_for_message(
    "/control/maneuver_status", String, timeout=10).data)
odom = rospy.wait_for_message("/localization/odometry", Odometry, timeout=10)
obs = rospy.wait_for_message("/detection/traffic_light", ObjectInfoArray, timeout=10)
p, q = odom.pose.pose.position, odom.pose.pose.orientation
ego = dict(x=p.x, y=p.y, z=p.z,
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
    roll=math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
    pitch=math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))))
heads = (status.get("event") or {}).get("signal_points", [])
print("HEADS", len(heads), "STAMP_DELTA",
      (odom.header.stamp-obs.header.stamp).to_sec())
for head in heads:
    print("PROJECT", head["id"], project_signal(head, ego, camera))
for obj in obs.objects:
    print("BOX", obj.class_name, obj.conf, obj.x_center, obj.y_center,
          obj.width, obj.height)
PY
```

통과 기준: 각 위치에서 자기 경로 신호의 예상 픽셀이 같은 물리적 신호등·검출 박스에 대응하고 옆 차로 신호와 뒤바뀌지 않는다. `(u,v)` 원점은 원본 영상 좌상단이다. `HEADS 0`, 투영 `None`, 지속적인 위치 오차는 미확인으로 기록한다. 코드의 박스 확장 25 px는 매칭 허용폭이며 보정 오차 합격 기준으로 쓰지 않는다.

보정이 확인되면 개인 YAML에 같은 값과 `calibrated: true`를 저장하고 기본 launch에 `turn_signal_maneuvers_file:=개인파일절대경로`를 전달한다. `cp -n`은 기존 개인 파일을 갱신하지 않는다. 확인이 안 되면 false를 유지하고 T06의 통과 시험은 보류한다. 주행 중 신호 연결은 영상·자세 시각 차이 50 ms 이내도 필요하다.

<a id="section-4"></a>

## 4 출발과 속도 상승 확인

**T04**  신호·정지선·장애물이 없는 경로 시작부에서 실시한다. MORAI 외부 UDP 제어 모드와 주행 가능 기어를 확인하고 차량을 경로 위에 같은 방향으로 배치한다. 관찰용 launch를 종료한 뒤 아래 하나만 실행한다.

```bash
roslaunch morai_bringup final_ws_native_no_lamps.launch \
  workspace_path:="$MORAI_WS" \
  morai_host_ip:=192.168.0.161 ego_status_port:=1911 \
  enable_control:=true max_speed_kph:=3.0 \
  2>&1 | tee "$TEST_DIR/start_$(date +%H%M%S).log"
```

개인 Cam4 YAML을 사용하는 경우 `turn_signal_maneuvers_file` 인자도 추가한다. T04는 교차로 진입 시험이 아니다. 실제 속도·정지 원인을 보는 터미널과 화면 녹화를 함께 켠다.

```bash
rostopic echo /experimental/curvature_speed_command
# 위 관찰은 Ctrl+C로 끝내고 다음 항목도 확인한다.
rostopic echo -n 1 /control/maneuver_status
rostopic echo -n 1 /stability/camera_fallback_status
rostopic echo -n 1 /ctrl_cmd
rostopic echo -n 1 /localization/odometry/twist/twist/linear
```

통과 기준: 정상 센서·정지 요청 없음 조건에서 출발점의 목표 속도가 0에 고정되지 않고 양수로 증가하며 실제 차량도 출발한다. 정지 후 재출발도 반복한다. 준비 완료 후 약 5초 동안 목표나 차량이 전혀 반응하지 않으면 실패 시점을 기록하고 [로그 저장과 정지 원인 찾기](#section-8)의 진단 순서로 확인한다. 5초는 초기 현장 점검 기준이다.

속도 단위: 곡률 `speed_limit`·`speed_command`는 km/h, 고속도로 `target_speed_mps`는 m/s다. odometry의 실제 속도는 `sqrt(vx²+vy²)×3.6` km/h로 비교한다. `longlCmdType=1`에서는 `velocity=0`이어도 accel/brake로 주행하므로 velocity 필드만으로 출발 실패를 판단하지 않는다.

### T05 같은 직선에서 곡률 단독과 통합을 비교한다

먼저 기본 통합 launch의 `max_speed_kph`만 7.2로 올려 시험한다. 그다음 완전히 종료하고 아래 곡률 단독 구성을 같은 위치·경로·상한으로 실행한다. 단독 구성에는 카메라 신호·장애물 정지가 없으므로 비어 있는 MORAI 직선 비교 구간에서만 사용한다.

```bash
roslaunch morai_bringup morai_udp_ekf_purepursuit.launch \
  workspace_path:="$MORAI_WS" \
  morai_host_ip:=192.168.0.161 ego_status_port:=1911 \
  enable_control:=true use_curvature_speed_planner:=true \
  max_speed_kph:=7.2 command_topic:=/ctrl_cmd \
  2>&1 | tee "$TEST_DIR/curve_only_$(date +%H%M%S).log"
```

비교 기록: 곡률 상한, 목표 속도, 실제 속도, 최종 accel/brake, fallback mode를 같은 구간에서 비교한다. 통합 구성만 낮으면 정지·fallback·신호 판단부터 확인한다. 곡선에서 곡률 상한이 내려가는 것은 정상이다. 충분히 긴 직선에서 불필요한 제한이 없는데 목표 속도가 지속적으로 낮으면 실패로 기록한다. 첫날은 3 → 7.2 km/h 순서로 확인하며 30 km/h 검증을 통과로 간주하지 않는다.

<a id="section-5"></a>

## 5 신호등과 정지선 시험

**T06**  기본 통합 launch와 검증된 Cam4 개인 YAML을 사용한다. 처음에는 3 km/h로 실시하고 성공하면 7.2 km/h에서 반복한다. 각 항목은 같은 접근 방향에서 3회 시행한다. 실제 주행 여부와 인식값을 같은 화면·시간으로 기록한다.

| 항목 | 시험 방법 | 통과 기준 |
| --- | --- | --- |
| 적색 접근 | 진입 전에 적색 상태를 유지하고 정지선으로 접근 | 정지선 전에 감속·정지. 적색 중 재가속·선 침범 없음 |
| 정지 유지 | 정지한 뒤 적색을 5초 이상 유지 | 정지 위치가 앞으로 밀리지 않고 accel이 허가되지 않음 |
| 녹색 재출발 | 정지 상태에서 자기 경로 신호를 녹색으로 변경 | 유효한 신호 확인 후 부드럽게 재출발. 순간적인 풀가속 없음 |
| 황색 접근 | 교차로 진입 전에 황색으로 변경 | 진입을 허가하지 않고 정지 동작. 이미 교차로에 진입한 상태와 구분 |
| 다른 차로 신호 | 자기 경로는 적색이고 주변 신호만 녹색인 장면 | 주변 녹색 때문에 출발하지 않음. 선택 신호 ID도 확인 |
| 방향 불일치 | 직진 경로에서 좌회전 허가만 보이는 장면 | 경로를 좌회전으로 바꾸거나 직진 허가로 오해하지 않음 |

### 정지 위치를 실제 앞범퍼 기준으로 잰다

설정 목표는 정지선 앞 **0.5 m**다. 첫 개발 시험의 권장 확인 범위는 0.3~0.7 m로 정하되, 이는 공식 대회 합격 기준이나 현재 성능 보장이 아니다. 범위를 벗어나면 접근 속도와 실제 거리를 기록하고 조정 대상으로 남긴다. 정지선 침범은 별도로 실패 처리한다.

기본 구성은 `maneuver_fusion`이 정지선 제어를 소유한다. `/control/stopline_status`에서 enabled=false가 보여도 기본 구성에서는 정상일 수 있다. 실제 판단은 `/control/maneuver_status`의 `mode`, `reason`, `front_bumper_distance_m`, `target_clearance_m`을 확인한다. 카메라 원시 거리에서 앞범퍼 오프셋을 이중 차감하지 않는다.

```bash
rostopic echo /control/maneuver_status
# 별도 터미널에서 각 관측을 저장하거나 확인한다.
rostopic echo -n 1 /perception/camera/stopline
rostopic echo -n 1 /detection/traffic_light
rostopic echo -n 1 /perception/traffic_light/directional_state
```

녹색은 화면에 보였다는 이유만으로 바로 통과 판정하지 않는다. `selected_signal_id`, `selected_signal_state`, `signal_selection_reason`, `route_signal_compatible`, `permission`을 함께 확인한다. 같은 신호·허가 상태를 서로 다른 영상 3개 이상, 0.3초 이상 확인하는 조건이 있다. 다른 안전 조건이 있으면 더 기다리거나 정지할 수 있다.

현재 모델의 실제 클래스명을 기준으로 시험한다. 화살표를 인식하더라도 경로 자체를 바꾸는 명령은 아니다. 보행자와 LiDAR 정지 요청이 함께 있으면 녹색이어도 출발하지 않는 것이 정상이다.

<a id="section-6"></a>

## 6 최신 고속도로 기능 시험

**T08**  기본 시험을 마친 뒤 별도 시나리오로 실시한다. 모든 기존 launch를 종료하고 아래를 먼저 제어 OFF로 실행한다. 센서·경로 상태가 정상일 때 종료 후 `enable_control:=true`로 다시 실행한다.

```bash
roslaunch morai_bringup final_ws_highway_bringup.launch \
  workspace_path:="$MORAI_WS" \
  morai_host_ip:=192.168.0.161 ego_status_port:=1911 \
  enable_control:=false max_speed_kph:=7.2 \
  2>&1 | tee "$TEST_DIR/highway_$(date +%H%M%S).log"
```

이 구성은 전역 경로 기반 교차로 허가와 GPS fallback을 시작하지 않는다. timestamped 정지선 제어와 하위 안전 정지는 유지된다. 기본 신호 시험 대신 이 launch를 사용해 교차로 통과 성공을 판정하지 않는다.

| 항목 | 시험 장면 | 확인 결과 |
| --- | --- | --- |
| 기본 통과 | 고속도로 활성 조건이 없는 정상 도로 | 기존 path manager 경로를 따라가며 이유 없는 정지 없음 |
| 차선 변경 | 왼쪽 점선과 충분한 합류 간격이 있는 고속도로 | WAIT_GAP → LANE_CHANGE → INNER_HOLD. 변경 후 원래 외측 경로로 즉시 당겨지지 않음 |
| 변경 금지 | 왼쪽 실선 또는 옆 차로 가까운 전후 차량 | 해당 차선 변경을 시작하지 않음. 상태 reason과 gap 결과 기록 |
| 변경 끝 정지 | 차선 변경 경로 끝에 가까워지는 구간 | 유한 경로 끝 때문에 불필요하게 멈추지 않고 차선 유지로 전환 |
| 앞차와 정적 우회 | 느린 앞차와 정지 장애물을 별도 장면으로 준비 | 앞차 추종 감속과 정적 우회가 구분됨. 0.60 m/s 이하가 우회 시작 속도 조건 |
| 인식 가림 | 앞차가 잠깐 차선 경계를 가리는 장면 | 조향 급변 없이 유지·감속·복구. 오래 소실되면 정지 |

```bash
rostopic echo /highway_lane_strategy/state
# 다른 터미널 또는 순서대로 확인한다.
rostopic echo -n 1 /highway_lane_strategy/target_speed_mps
rostopic echo -n 1 /highway_lane_strategy/stop_required
rostopic echo -n 1 /perception/merge_gap/available
rostopic echo -n 1 /perception/camera/lane_info
```

핵심 기록: state, reason, target_speed_mps, lane_changes_done, lane_info의 output_status와 timestamp, 실제 속도. 짧은 인식 소실에서 옛 geometry의 timestamp가 새 관측처럼 갱신되지 않는지 확인한다. 재변경은 횡방향·방향 정렬과 이동거리·시간 조건을 다시 만족해야 한다.

7.2 km/h에서 2 m/s가 나오는 것은 정상이다. 별도 2 m/s 제한 제거 확인은 저속 시험을 통과한 뒤, 충분한 빈 직선에서 상한을 10.8 km/h로 높여 시행한다. 앞차·곡률·복구 제한이 없을 때 고속도로 목표가 2 m/s를 넘을 수 있는지 확인한다. 상한에 항상 도달해야 한다는 기준은 아니다.

<a id="section-7"></a>

## 7 인식 소실과 선택 시나리오 시험

### T07 기본 구성에서 카메라 관측 소실

3 km/h 이하로 접근하거나 정지한 상태에서 MORAI의 Cam4 송신을 잠깐 중단한다. 카메라 노드가 완전히 종료되는 경우와 영상만 끊기는 경우를 구분해 기록한다. 신호 미인식이 자동 녹색으로 처리되어 출발해서는 안 된다. 차선 카메라 송신 소실도 별도 시험하고 소실 직후와 복구 후 판단을 확인한다.

통과 기준: 오래된 관측으로 새 통행 허가를 만들지 않고 감속·정지한다. 송신 복구 후에는 새 시각의 유효한 관측과 확인 조건을 거쳐 재출발한다. 이미 교차로에 진입한 뒤의 신호 변화는 접근 중 시험과 분리해 기록한다. timeout 값을 늘려 실패를 가리지 않는다.

### T08 추가 시험 차선 유지 중 관측 소실

고속도로 launch의 INNER_HOLD에서 짧은 차선 가림과 3초를 넘는 가림을 따로 시험한다. 최신 코드의 복구 시간은 lane-invalid 판정 이후 최대 3초이므로 송신 중단 순간과 정지 순간의 차이가 정확히 3초일 필요는 없다. 복구 구간에는 감속하고, 제한 시간을 지나면 정지하며 새 관측이 정상화되면 복구하는지 본다.

### T09A 기본 구성에서 GPS 소실

기본 launch를 다시 사용한다. 교차로·정지선·신호가 없는 구간에서 정상 GPS·IMU·카메라 주행 이력을 먼저 만든 뒤 MORAI의 GPS만 중단한다. IMU와 카메라는 계속 보내고 3 km/h 이하에서 확인한다.

```bash
rostopic echo /stability/camera_fallback_status
# 별도 터미널에서 확인한다.
rostopic echo /localization/sensor_quality
```

통과 기준: 차선 fallback 조건이 충족되면 제한 속도 안에서 잠시 유지하고, 조건이 부족하면 정지한다. 계속 움직여야만 합격인 시험이 아니다. 현재 기본 wrapper의 fallback 상한은 3 km/h이며 최대 15초·30 m 제한 중 먼저 도달하는 조건이 적용된다. 차선 제어는 관측 나이 0.15초 제한도 확인한다. GPS 복구 후에는 안정 확인을 거쳐 nominal로 돌아가는지 기록한다.

### T09B 고속도로 구성에서 회전교차로 진입 판단

고속도로 launch의 회전교차로 gate는 미션 요청이 있어야 작동한다. 제어 OFF 상태에서 먼저 진입 위치와 충돌 차량을 준비하고 요청을 발행한다. 다른 미션 발행자가 없는 전용 시험에서만 아래 명령을 사용한다.

```bash
rostopic pub -r 5 /planning/merge_request std_msgs/Bool 'data: true'
# 다른 터미널에서 확인한다.
rostopic echo /roundabout_merge_gate/status
```

충돌 차량이 있으면 stop_required가 true인지, 충분히 비면 확인 시간을 거쳐 allowed가 true인지 본다. GO여도 하위 교차로·보행자·신호 정지가 남아 있으면 차량은 멈출 수 있다. 이 gate는 기존 정지를 강제로 해제하지 않는다.

```bash
# 요청 발행 터미널을 Ctrl+C로 끝낸 뒤 미션 요청을 해제한다.
rostopic pub -1 /planning/merge_request std_msgs/Bool 'data: false'
```

<a id="section-8"></a>

## 8 로그 저장과 정지 원인 찾기

각 시험을 시작하기 전에 별도 터미널에서 녹화를 시작한다. 아래 TEST_DIR는 [센서 수신과 제어 OFF 확인](#section-2)의 환경 블록에서 만든 날짜별 폴더다. rosbag은 시험 종료 시 Ctrl+C로 마무리한다. 화면 녹화에는 차량·신호·정지선과 실패 시각이 보여야 한다. UDP 원본 카메라는 아래 bag에 영상으로 자동 저장되지 않으므로 MORAI와 인식 창 녹화를 별도로 남긴다.

```bash
rosparam dump "$TEST_DIR/params_$(date +%H%M%S).yaml"
rosbag record --lz4 -O "$TEST_DIR/run_$(date +%H%M%S).bag" \
  /gps /Imu /Ego_topic /localization/odometry \
  /localization/sensor_quality /detection/lane \
  /perception/camera/lane_quality /perception/camera/lane_info \
  /perception/camera/stopline /detection/traffic_light \
  /perception/traffic_light/state /perception/traffic_light/directional_state \
  /perception/lidar/tracked_obstacles_map /detection/fused_safety_stop \
  /experimental/curvature_speed_limit /experimental/curvature_speed_command \
  /experimental/curvature_progress /experimental/active_path_source \
  /control/ctrl_cmd /control/stopline_cmd /control/camera_fallback_cmd \
  /control/maneuver_cmd /ctrl_cmd /control/stopline_status \
  /control/maneuver_status /stability/camera_fallback_status \
  /highway_lane_strategy/state /highway_lane_strategy/active_path \
  /highway_lane_strategy/target_speed_mps /highway_lane_strategy/stop_required \
  /roundabout_merge_gate/status /planning/merge_request
```

실행 구성에 없는 토픽은 녹화 데이터가 없는 것이 정상이다. 영상과 bag은 같은 시험 ID로 묶고 실패 전후 최소 10초를 남긴다. 설정 YAML, launch 명령과 로그, 모델 검사 출력도 함께 보관한다.

| 관측한 현상 | 다음 확인 순서 |
| --- | --- |
| 목표 속도가 계속 0 | 곡률 speed_limit → 주행 상태 reason → 정지선/신호 → path/merge stop |
| 목표는 양수 최종 accel은 0 | maneuver_status → fallback_status → fused_safety_stop → 각 제어 단계 명령 비교 |
| 최종 accel은 양수 차량은 정지 | enable_control → MORAI 외부 제어/기어 → IP와 9093 수신 → 실제 UDP 연결 |
| 녹색인데 정지 | signal_camera_uncalibrated → selected_signal_id/state → 방향 호환 → stale/pose 동기화 |
| 3 km/h 부근에서 제한 | fallback 활성 여부와 quality 상태 확인. nominal 상한과 혼동하지 않음 |
| 고속도로에서 2 m/s | 설정 상한 7.2 km/h인지 확인 → 앞차/곡률/복구 제한 → 실제 speed override |
| 재출발하지 않음 | 신호 확인 프레임과 timestamp → 안전 정지 잔존 → lane/merge 복구 상태 |

순서대로 비교할 제어 경로: 기본 구성은 `/control/ctrl_cmd` → stopline 통과 → camera_fallback → maneuver → `/ctrl_cmd`이다. 고속도로 구성은 adaptive nominal → stopline → mux이다. 문제 확인을 위해 `/ctrl_cmd`에 별도 가속 메시지를 강제로 보내지 않는다.

<a id="section-9"></a>

## 9 결과 기록과 시험 종료

실행 전에 아래 시험 ID를 정하고, 각 행에 통과·실패·보류 중 하나를 적는다. 시나리오 조건을 만들지 못했거나 Cam4 보정이 끝나지 않았으면 실패와 구분해 보류로 기록한다. 저속 통과를 고속·전체 코스 통과로 확대하지 않는다.

| 시험 | 상한 km/h | 결과 | 측정값 또는 실패 시각 |
| --- | --- | --- | --- |
| T01 빌드 모델 | 해당 없음 |  |  |
| T02 제어 OFF | 3 |  |  |
| T03 Cam4 투영 | 정지 |  | 30 / 20 / 10 m 지점 |
| T04 출발 재출발 | 3 |  |  |
| T05 곡률 단독 | 7.2 |  | 상한 / 목표 / 실제 |
| T05 기본 통합 | 7.2 |  | 상한 / 목표 / 실제 |
| T06 적색 정지 | 3 → 7.2 |  | 앞범퍼 간격 1차 / 2차 / 3차 |
| T06 녹색 재출발 | 3 → 7.2 |  | 허가 시각 / 출발 시각 |
| T06 다른 신호 | 3 |  | 선택 신호 ID / 상태 |
| T07 카메라 소실 | 3 이하 |  | 소실 / 정지 / 복구 시각 |
| T08 차선 변경 | 7.2 |  | 상태 변화 / 차선 유지 |
| T08 우회와 추종 | 7.2 |  | 장애물 속도 / 판단 사유 |
| T08 인식 복구 | 저속 |  | 가림 시간 / 정지 / 재개 |
| T09A GPS 소실 | 3 이하 |  | mode / 관측 나이 / 복귀 |
| T09B 회전교차로 | 제어 OFF부터 |  | gate 판단 / 하위 정지 사유 |

### 문제가 생겼을 때 함께 남길 내용

시험 ID와 반복 번호 __________

실패 당시 MORAI 시각과 bag 시각 __________

예상 동작 __________

실제 동작 __________

정지 이유 또는 상태 문자열 __________

사용한 launch 전체 명령 __________

bag 파일명과 영상 파일명 __________

### 종료 순서

MORAI에서 차량 정지 또는 일시정지 → 외부 제어 해제 → 시험용 request 발행 종료 및 해제 → rosbag Ctrl+C로 정상 저장 → launch Ctrl+C 종료 → bag 파일과 영상 재생 확인 순서로 마친다. 로그와 시험표를 함께 가져오면 속도 제한과 인식·제어 문제를 구분할 수 있다.

작성 근거: 현재 workspace의 `ROI_UPDATE_20260921_KO.md`, `FINAL_WS_NATIVE_NO_LAMPS_TEST_KO.md`, `MANEUVER_SENSOR_FUSION.md`, 관련 launch·제어 노드 및 `roi_upstream_manifest.json`. 이 문서는 현장 시험 절차이며 아직 수행하지 않은 시험의 결과를 의미하지 않는다.
