# 대회 샘플 시나리오 로드와 최종 ROS 주행 시험

작성 기준: 2026-09-08, 이 저장소의 실제 JSON·경로·launch·센서 설정과 로컬 회귀 테스트를 확인했다. ROS 빌드, MORAI 실행이나 주행은 수행하지 않았다. 단위 테스트·mock UDP 테스트의 성공은 시뮬레이터 주행 검증을 의미하지 않는다. 아래 명령은 Ubuntu 시험 환경에서 실행할 절차다.

## 1. 실제 입력 파일과 지도·차량 불일치

Windows 기준 저장소는 `E:/duawl-data/Documents/MoraiSimulation`이다. 다음 경로는 `morai_ws` 기준이다.

| 역할 | 실제 파일 | 적용 대상 |
|---|---|---|
| 샘플 시나리오 | [2026_molit_comp_sample_scene.json](../data/scenarios/2026_molit_comp_sample_scene.json) | MORAI의 Ego·NPC·보행자·객체·신호 초기 상태 |
| 주행 경로 | [2026_molit_comp_global_path.txt](../data/routes/2026_molit_comp_global_path.txt) | ROS `path_file`로 읽는 별도 `x y z` 경로 |
| 지도 원점 | [global_info.json](../data/mgeo/R_KR_PR_K-city_2025/global_info.json) | UTM52N → map 좌표 기준 |
| 고정 카메라 원본 | [data/sensors/cam_set.json](../data/sensors/cam_set.json) | 카메라 1~3의 장착·영상 설정 |
| 차선 인식 보정 원본 | [lane/cam_set.json](../src/detection/camera_perception/lane/cam_set.json) | 차선 카메라용 설정; YOLO 카메라 보정으로 대체 사용 금지 |
| 최종 실행 | [final_ws_bringup.launch](../src/bringup/morai_bringup/launch/final_ws_bringup.launch) | UDP 센서, 위치 추정, 인식, 융합, 최종 명령 |
| 경로·신호 융합 설정 | [turn_signal_maneuvers.yaml](../src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml) | 조작 계획과 신규 경로·신호 대응/카메라 보정 설정 |

샘플 JSON의 `mapInfo.mapName`은 **`R_KR_PR_K-city_2025`**, 전역 좌표계는 `UTM52N`, 시나리오 좌표계는 `ENU`다. 원점은 `(302595, 4124145, 0)`이고 MGeo의 `local_origin_in_global`과 일치한다. `workspace_origin=[40,-174,-925]`는 이 GPS 원점으로 사용하지 않는다.

[규정집 요약](세부규정집_요약.md)은 지도명을 `R-KR_PG_K-City_2025`, 차량을 `2023_Hyundai_ioniq5`, 대회용 버전을 `25.S4.MolitComp03`으로 기록한다. 반면 [vehicle_model.yaml](../config/vehicle_model.yaml)은 `2025_Hyundai_ioniq5`다. 최종 주행 launch는 휠베이스 3.000m, 최대 앞바퀴각 40°, 뒤차축 기준 앞범퍼 오프셋 3.845m를 사용한다. 이 차이는 실제 설치 지도·Ego 차량·원점을 확인해서 해소해야 한다.

샘플 Ego는 모델 문자열 없이 `DataID=20200012`만 저장한다. 이 ID가 특정 연식의 IONIQ5라는 로컬 대응표는 확인하지 못했다. 시나리오 로드가 최종 제어기의 차량 모델과 동일함을 보장하지 않는다. 현재 설치된 MORAI/Launcher/UI 버전도 확인하지 못했으며, JSON의 `version=1.0`과 MGeo의 `3.0`은 시뮬레이터 UI 버전이 아니다.

## 2. MORAI에서 JSON 로드

1. MORAI에서 샘플의 지도 `R_KR_PR_K-city_2025`를 선택한다. 비슷한 이름의 지도를 동일 버전이라고 가정하지 않는다.
2. **Edit → Scenario → Load Scenario**를 연다. Windows 원본은 `E:/duawl-data/Documents/MoraiSimulation/morai_ws/data/scenarios/2026_molit_comp_sample_scene.json`이다.
3. 원하는 JSON과 불러올 데이터 범위를 선택하고 **Load**를 누른다. Ego까지 불러올지 확인하고, 로드 후 실제 차량 모델·위치·자세를 재확인한다. 목록에 파일이 없다면 해당 설치 버전의 파일 선택/저장 위치를 확인한다. 저장소만으로 특정 사용자 시나리오 폴더나 단축키를 확정할 수 없다.
4. 센서 설정은 별도로 준비하고 아래 네트워크 목적지와 보정 상태를 맞춘다. 시나리오 JSON에는 실제 카메라·GPS·IMU·LiDAR 장착 목록이 없다.
5. 최초 검증은 정지 상태로 시작한다. 센서 수신 시험에서는 Play 상태로 두되 차량에 외부 주행 명령을 적용하는 단계는 뒤의 통과 조건 이후로 진행한다.

공식 근거: MORAI의 [Scenario editor basics](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/scenario-editor-basics)는 Scenario 메뉴의 Load Scenario에서 JSON·로드 범위를 선택해 Load하도록 안내한다. 같은 문서는 라이선스에 없는 차량이 기본 모델로 대체될 수 있다고 설명한다. 따라서 실제 로드 결과의 차량 확인이 필요하다. 이 메뉴 경로는 공식 문서 기준이며 현재 설치 UI에서 직접 검증한 결과는 아니다.

**JSON을 로드해도 ROS 경로는 설정되지 않는다.** `waypointDataList`는 Ego의 최종 Pure Pursuit 경로 파일을 대신하지 않는다. ROS에는 TXT 경로를 `path_file`로 별도 전달한다. `CtrlCmd`는 차량을 시작점으로 순간이동시키는 명령이 아니다.

## 3. 저장된 시작 상태와 경로 정렬

실제 JSON과 TXT를 읽어 계산한 값이다.

| 항목 | 값 |
|---|---|
| Ego 위치 / 방식 | `(-131.485992, -427.960999, 28.883000)` m / `Absolute` |
| Ego yaw | `62.515°` |
| Ego 초기 상태 | 거의 정지, `gear=1`, `currentControlMode=1`, `initLink=""` |
| 경로 첫 점 | `(-131.689798, -428.331023, 28.543960)` m |
| 시작점 XY 거리 / 첫 선분 방향 | `0.422439m` / `61.298115°` |
| 방향 차이 / Z 차이 | `1.216885°` / `0.339039m` |
| 경로 원본 점 수 / XY 길이 | `4430` / `2184.611723m` |
| 폐곡선 / 연속 중복 XY 점 | 처음과 마지막 점 동일 / `38`개 |

기어·제어 모드 숫자의 UI 명칭을 JSON만으로 단정하지 않는다. 실제 네트워크 제어 활성 상태와 차량의 주행 가능 기어를 확인한다. 제어 UDP 구현은 명령에 `ctrl_mode=2`, `gear=4`를 넣는다.

시나리오에는 NPC 9대, 보행자 1명, 객체 1개, 차량 스폰 지점 4개, 교통신호 16개, shaded area 1개, waypoint 그룹 2개가 있다. 스폰 지점 ID 10/6/8은 주기 10초, ID 4는 5초지만 **저장된 `isSpawning`은 모두 false**다. Play 이후 NPC가 지속 생성되는지는 현장에서 확인한다. V2I는 `isAlive=false`다. 신호 상태 숫자나 shaded area 존재만으로 현재 신호 색/실제 GPS blackout을 단정하지 않는다.

정렬 확인 사항:

- 최종 GPS 변환은 UTM52N에서 `(302595,4124145,0)`을 차감한다. 최초 GPS 수신 위치를 `(0,0)`으로 다시 잡거나 경로에 별도 임의 회전·평행이동을 중복 적용하지 않는다.
- 현재 GPS 장착 가정은 뒤차축 `base_link` 기준 `(0,0,1.2)`m, IMU는 `(0,0,0)`과 회전 0이다. 경로 Z와 Ego Z의 차이를 그대로 XY 보정값으로 사용하지 않는다.
- [localization_alignment.yaml](../config/localization_alignment.yaml)은 `transform_validated:false`, `awaiting_live_bag`다. 최종 launch는 이 파일을 자동 합격 게이트로 읽지 않는다. 파일이 존재한다는 이유로 보정 완료로 판단하지 않는다.
- RViz Fixed Frame을 `map`으로 놓고 `/experimental/curvature_reference_path`, `/localization/odometry`를 함께 확인한다. `/Ego_topic`은 위치·heading 비교용이며 최종 EKF의 위치 보정 입력이 아니다.
- 개발용 최초 기준은 GPS 정지 분산 각 축 0.5m 이하, GPS/Ego 평균 XY 잔차 1m 이하, 직선 yaw 잔차 3° 이하, 초기 경로 횡오차 1m 이하다. 공식 대회 판정 기준은 아니다. [좌표정렬 절차](좌표정렬_실행절차.md) 참조.
- 현재 곡률 제어기는 최초 위치를 전체 경로의 최근접 선분에 투영한다. 문서의 'waypoint 0부터 시작'은 배치 정책이지 강제 시작 인자 구현이 아니다. 폐곡선 시작/끝 근처에서 `/experimental/curvature_progress`가 약 2184m 또는 `curvature_goal_reached=true`로 시작하면 주행하지 않는다.

## 4. 실제 센서 설정과 YOLO 1131 보정 차단 조건

두 `cam_set.json` 모두 카메라 3개만 있고 GPS/IMU/LiDAR 목록은 비어 있다. 카메라 주기는 약 0.05초다.

| 카메라 | 위치 m | RPY deg | 해상도 / FOV | 원본 UDP → 최종 계약 |
|---|---|---|---|---|
| 1 전방·차선 | `(1.9,0,1.2)` | `(0,2,0)` | `1280×720` / `90°` | `9290→9291` → `1100→1101` |
| 2 좌측 | `(1.15,0.65,1.2)` | `(0,10,70)` | `640×480` / `130°` | `9292→9293` → `1110→1111` |
| 3 우측 | `(1.15,-0.65,1.2)` | `(0,10,290)` | `640×480` / `130°` | `9294→9295` → `1120→1121` |
| 4 YOLO·신호 | **미확인** | **미확인** | **미확인** | 원본 없음 → `1130→1131` |

원본의 Host/Destination IP는 모두 `127.0.0.1`이다. 다른 PC의 Ubuntu로 보낼 때 그대로 사용할 수 없다. 최종 카메라 스택은 **lane=1101, YOLO=1131**의 UDP 영상을 직접 읽는다. 좌·우 포트 1111/1121 수신은 이 최종 카메라 launch에서 자동 실행하지 않는다.

보정 소스 조사 결과: [camera_perception.launch](../src/detection/camera_perception/launch/camera_perception.launch)의 `lane_cam_set`은 차선 프로세스에만 전달된다. [camera_feature_runner.py](../src/detection/camera_perception/scripts/camera_feature_runner.py)와 [camera_object_detection_node.py](../src/detection/camera_perception/scripts/camera_object_detection_node.py)는 YOLO용 포트 1131을 연결하지만, 조사한 설정에는 camera 4의 width/height/FOV/장착 위치·각도를 확정하는 센서 원본이 없다. YOLO의 `inference-size=416`은 추론 크기이며 원본 영상 해상도나 카메라 내부 파라미터를 증명하지 않는다.

신호등을 MGeo의 해당 경로·링크와 엄격하게 대응시키는 신규 구현은 [turn_signal_maneuvers.yaml](../src/control/turn_signal_controller/config/turn_signal_maneuvers.yaml)의 **`signal_camera` 보정 설정에 `calibrated` boolean 기본 false**를 요구한다. 실제 1131 송신 카메라의 원본 width/height, FOV, 차량 기준 extrinsics와 영상 좌표 변환을 확인하기 전에는 true로 바꾸지 않는다. 차선 카메라의 1280×720/90°/장착값을 camera 4에 복사해 보정을 통과시키지 않는다.

현재 YAML의 신규 필드는 다음과 같다. **모든 0은 Cam4 실측값이 없는 미입력 표시**이며, 차량 원점에 광각 0° 카메라가 실제 설치되었다는 뜻이 아니다.

```yaml
require_route_signal_context: true
signal_camera:
  calibrated: false
  width: 0
  height: 0
  horizontal_fov_deg: 0.0
  x: 0.0
  y: 0.0
  z: 0.0
  pitch_deg: 0.0
  yaw_deg: 0.0
```

`width`/`height`는 신호 검출 좌표에 대응하는 실제 영상 크기, `horizontal_fov_deg`는 수평 FOV, `x/y/z`는 차량 기준 장착 위치다. YAML의 좌표 규약은 `base_link` x전방/y좌측/z위, `yaw_deg` 좌향 양수, **`pitch_deg` 광축이 위를 향하면 양수**다. MORAI 센서 UI의 pitch 부호를 그대로 복사하지 말고 이 규약과 투영 결과를 확인한다. 영상 resize/crop과 검출 bbox 좌표도 같은 보정 기준을 사용해야 한다.

`require_route_signal_context:true`를 유지한다. `route_preview_m=30`, `turn_threshold_deg=25`의 곡률 추정은 이 값을 false로 둔 레거시 시험용이며 엄격 대응의 대체 합격 조건이 아니다. `maneuvers:[]`도 MGeo 경로·신호 연결이 필요 없다는 뜻이 아니다. 신규 보정을 확인한 뒤에는 YAML을 읽는 fusion을 재시작하고 실제 `/final_ws_maneuver_fusion/signal_camera` 파라미터를 확인한다.

| 신규 차단 사유 | 운용 해석과 해소 조건 |
|---|---|
| `signal_camera_uncalibrated` | 실제 YOLO 카메라 보정 미확인. 1131 센서 원본과 투영 검증을 확보한다. false를 강제로 우회하지 않는다. |
| `signal_unconfirmed` | 해당 경로에 적용되는 신호 관측/대응이 확정되지 않음. 다른 차로 신호, 모호한 검출, 오래된 입력을 허가로 사용하지 않는다. |
| `route_context_unavailable` | 사용 중인 경로와 MGeo 링크·신호의 대응 문맥을 확보하지 못함. 지도 버전, 경로 파일, 링크/신호 데이터와 차량 위치를 확인한다. |
| `unassociated_visible_signal` | 보이는 신호등을 지도상의 대상과 연결하지 못함. 보정·영상·대상 ID 확인이 필요하다. |
| `unmapped_stopline_before_context` | 연결한 지도 정지선보다 가까운 정지선을 관측함. 먼 교차로의 허가로 통과하지 않는다. |
| `signal_localization_unreliable` | 교차로 이벤트 유무와 무관하게 엄격 모드에서 위치 품질 저하 시 정지한다. |
| `next_junction_guard` | 다음 신호 정지선 또는 차선 변경 시작점의 제동 제한. 이전 교차로 허가는 다음 조작까지 이어지지 않는다. |

`data/mgeo/R_KR_PR_K-city_2025`의 제공 파일은 global_info/link/node이고, 카메라 `lane/mgeo/R_KR_PR_K-city_2025`에는 traffic_light_set 등 추가 파일이 있다. 최종 launch의 신규 `signal_mgeo_path` 기본값은 **`$(find camera_perception)/lane/mgeo/R_KR_PR_K-city_2025`**다. 지도 폴더 이름만으로 완전한 신호 데이터가 있다고 가정하지 않는다. 실제 `/final_ws_maneuver_fusion/signal_mgeo_path`와 필수 파일 존재를 확인한다. 색상이 녹색이라는 이유만으로 보정·경로 대응 차단을 해제하지 않는다.

## 5. IP·포트와 빌드 준비

### 정적 경로·신호 연결 사전 검사

빌드/source 후 ROS master나 MORAI를 켜지 않고도 실행할 수 있다. 파일을 읽어
결과를 출력할 뿐, 센서나 차량에 접속하지 않는다.

```bash
rosrun turn_signal_controller inspect_route_signals.py \
  --path-file "$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  --mgeo-dir "$(rospack find camera_perception)/lane/mgeo/R_KR_PR_K-city_2025"
```

현재 저장된 TXT와 MGeo를 오프라인 대조한 결과는 아래와 같다. 거리는 경로
시작점부터 누적 XY 거리이며, 실제 카메라 정확도/시뮬레이터 통과 검증은 아니다.

| MGeo 링크 | 방향 | 정지선 s(m) | 링크 끝 s(m) |
|---|---|---:|---:|
| A2256W000215 | 직진 | 208.459 | 237.423 |
| A2256W000222 | 직진 | 315.906 | 346.742 |
| A2256W000599 | 직진 | 436.798 | 462.942 |
| A2256W000318 | 우회전 | 550.619 | 580.868 |
| A2256W000083 | 좌회전 | 609.877 | 635.113 |
| A2256W000126 | 직진 | 1877.745 | 1996.731 |

6곳 모두 정적 지도 `on_stop_line` 연결이 확인됐다. 이 밖의 실제 신호 제어
구간까지 모두 포함한다는 보증은 아니다. 현장에 추가 신호/정지선이 있으면
`unmapped_signal_or_stopline`으로 정지하므로 지도/경로를 대조해야 한다.

### 네트워크와 의존성

`morai_host_ip`는 **시뮬레이터 PC** 주소다. 센서의 Destination IP는 **Ubuntu 알고리즘 PC** 주소다. `0.0.0.0`은 Ubuntu 수신 bind 값이고 원격 목적지 IP가 아니다. 현재 시험 기준은 MORAI `192.168.0.148`, Ubuntu `192.168.0.200`이지만, 실제 네트워크 설정과 연결 상태를 먼저 확인한다.

| 데이터 | 방향 / 포트 | 최종 ROS 입력·출력 |
|---|---|---|
| 전방 차선 영상 | MORAI `1100` → Ubuntu `1101` | `/detection/lane`, `/perception/camera/stopline` |
| YOLO 영상 | MORAI `1130` → Ubuntu `1131` | `/detection/traffic_light`, 신호·보행자 판단 |
| 좌·우 영상 | MORAI `1110/1120` → Ubuntu `1111/1121` | 최종 ROI 스택 기본 수신 대상 아님 |
| VLP16 LiDAR | MORAI `2000` → Ubuntu `2001` | `/morai/lidar/live_points`, `/perception/lidar/tracked_obstacles_map` |
| GPS NMEA0183 | MORAI 센서 송신 포트 → Ubuntu `3001` | `/gps` (`morai_msgs/GPSMessage`) |
| MORAI IMU | MORAI 센서 송신 포트 → Ubuntu `4001` | `/Imu` (`sensor_msgs/Imu`) |
| Ego 상태 | MORAI `908` → Ubuntu `909` | `/Ego_topic` (`morai_msgs/EgoVehicleStatus`) |
| 차량 제어 | Ubuntu `9094` → MORAI `9093` | `/ctrl_cmd` (`morai_msgs/CtrlCmd`) |
| 방향지시등 | Ubuntu → MORAI `9097` | 별도 LampControl UDP; 송신 소스 포트는 여기서 고정값으로 제시하지 않음 |

GPS는 GPRMC/GPGGA, LiDAR는 VLP16 UDP, IMU는 MORAI binary 형식에 맞춘다. [sensor_ports.yaml](../config/sensor_ports.yaml)의 'native_ros 우선' 설명과 달리 **현재 final launch는 UDP 수신/송신 노드를 직접 실행**한다. `/gps`, `/Imu`, `/Ego_topic`, `/ctrl_cmd`를 동시에 연결하는 rosbridge/native ROS 스택을 중복 실행하지 않는다. 이 경로에는 rosbridge가 필수 구성요소가 아니다.

Linux의 UDP 909 수신은 실행 환경의 낮은 포트 bind 권한이 필요할 수 있다.
`Permission denied`이면 해당 수신 권한/컨테이너 설정을 확인한다. 권한 오류를
GPS나 경로 문제로 해석하지 않는다.

Ubuntu 20.04 + ROS Noetic 또는 문서화된 Noetic/Focal Docker 환경을 사용한다. Ubuntu 22.04 호스트에서는 Noetic/Focal 컨테이너를 구분한다. Docker는 UDP 수신이 가능한 네트워크가 필요하며 저장소의 시험 예시는 `--network host`를 사용한다. 현재 카메라 실행에는 OpenCV 표시 경로도 있으므로 DISPLAY/X11 사용 여부를 확인한다. [새 Docker 가이드](../../TEST_FROM_SCRATCH_KO.md)는 환경 준비 참고용이며 그 문서의 곡률 전용 launch는 최종 통합 launch와 다르다.

`morai_msgs`는 이 저장소에 없다. 실제 MORAI 설치와 맞는 ROS1 catkin 메시지 패키지를 먼저 `src` 아래 준비해야 한다. `pip install morai_msgs` 대상이 아니다. 저장소 내 메시지는 `morai_perception_msgs`, `common` 등이며 폴더명 `common_msgs`의 실제 패키지명은 `common`이다.

```bash
export MORAI_WS=/root/AutoVehicle/morai_ws  # 실제 Ubuntu workspace로 변경
source /opt/ros/noetic/setup.bash
cd "$MORAI_WS"
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source "$MORAI_WS/devel/setup.bash"
rospack find morai_msgs
rospack find morai_perception_msgs
rospack find common
rospack find morai_bringup
rosmsg show morai_msgs/CtrlCmd
rosmsg show morai_msgs/GPSMessage
rosmsg show morai_msgs/EgoVehicleStatus
python3 -c 'import cv2, numpy, scipy, rospkg, torch, torchvision, ultralytics'
```

ROS의 rospy/tf/nav_msgs/sensor_msgs/geometry_msgs/std_msgs, 메시지 생성 도구와 Python NumPy/OpenCV/SciPy/rospkg가 필요하다. 카메라 코드가 사용하는 PyTorch/torchvision/ultralytics는 `rosdep`만으로 모두 준비된다고 가정하지 않는다. Noetic Python 및 GPU/CUDA 또는 CPU 환경에 맞는 호환 버전을 별도 준비하고 import·모델 로딩을 확인한다. GPS 좌표 변환은 순수 Python 구현이므로 pyproj를 필수 의존성으로 추가할 필요는 없다.

로컬에 실제 존재하는 모델은 `camera_perception/lane/lane_seg_best.pt`(97,898,559 bytes), `models/yolov8n.pt`(6,549,796 bytes), `models/best0902.pt`(22,608,810 bytes)다. Ubuntu 복사본에도 존재하는지 확인한다. 파일 존재 확인은 추론 실행 검증이 아니다.

## 6. 최종 launch의 실제 인자

기준은 [final_ws_bringup.launch](../src/bringup/morai_bringup/launch/final_ws_bringup.launch)와 포함되는 [perception_control_bringup.launch](../src/bringup/morai_bringup/launch/perception_control_bringup.launch), [morai_udp_ekf_purepursuit.launch](../src/bringup/morai_bringup/launch/morai_udp_ekf_purepursuit.launch)다.

| 최상위에서 실제 선언된 인자 | 기본값 |
|---|---|
| `workspace_path`, `path_file` | `$HOME/morai_ws`, 해당 workspace의 `data/routes/2026_molit_comp_global_path.txt` |
| `morai_host_ip`, `enable_control` | `192.168.0.148`, `false` |
| `max_speed_kph` | `7.2`; 하위 PP의 max/target 속도 인자 모두에 전달 |
| `enable_roi_camera`, `enable_roi_lidar`, `roi_enable_lane`, `roi_enable_yolo` | 모두 `true` |
| `roi_lidar_rviz` | `false` |
| `enable_lane_fallback`, `enable_stopline_control`, `enable_maneuver_fusion` | 모두 `true` |
| `fallback_speed_cap_kph`, `stop_without_camera` | `7.2`, `true` |
| `lane_assist_confidence`, `lane_fallback_confidence`, `lane_stable_samples` | `0.55`, `0.80`, `5` |
| `fallback_entry_delay_sec`, `lane_loss_grace_sec`, `recovery_stable_sec` | `0.50`, `0.25`, `1.0` |
| `stopline_topic` | `/perception/camera/stopline` |
| `stopline_front_reference_offset_m`, `stopline_hold_distance_m` | `3.845`, `0.5` |
| `stopline_max_decel_mps2`, `stopline_planning_decel_mps2` | `1.5`, `1.0` |
| `enable_turn_signal`, `turn_signal_lead_time_sec`, `turn_signal_remote_port` | `enable_control`과 같은 기본값, `5.0`, `9097` |
| `turn_signal_maneuvers_file` | `turn_signal_controller/config/turn_signal_maneuvers.yaml` |
| `signal_mgeo_path` | `$(find camera_perception)/lane/mgeo/R_KR_PR_K-city_2025` |

**최상위에 없는 인자를 추가하지 않는다.** 조사한 final launch에는 `target_speed_kph`, `gps_port`, `imu_port`, `ego_status_port`, `control_remote_port`, `control_source_port`, `roi_lane_port`, `roi_yolo_port`, `lane_cam_set`, `signal_camera_calibrated`가 선언되어 있지 않다. 하위 launch에 있는 인자가 자동으로 최상위에 노출되지는 않는다. 포트가 다르면 현재 final이 수신하는 위 포트에 MORAI 설정을 맞추거나, 주 구현 담당자가 지원하는 연결 구성을 먼저 확정한다. 카메라 보정은 `signal_camera_calibrated:=true` 같은 미정의 인자로 켜는 것이 아니라 실제 YAML의 `signal_camera.calibrated`를 검증 결과와 함께 관리한다.

최종 기본값은 **`max_speed_kph=7.2`(2m/s)**지만 이 가이드의 초기 시험은 실제 지원 인자인 **`max_speed_kph:=3.0 fallback_speed_cap_kph:=3.0`**, 즉 **3km/h(약 0.833m/s)**로 제한한다. `max_speed_kph`는 하위 PP의 `max_speed_kph`와 `target_speed_kph` 모두로 전달된다. `fallback_speed_cap_kph`는 fallback 상한이므로 두 값을 함께 지정한다. 제어기는 설정을 시작 시 읽으므로 실행 중 `rosparam set`만으로 재설정됐다고 가정하지 않는다.

## 7. Dry-run과 저속 주행 통과 조건

MORAI에서 센서 송신을 준비하고, 위 빌드 환경을 source한 Ubuntu 터미널에서 실행한다. 다음 IP는 실제 시뮬레이터 주소로 바꾼다.

```bash
export MORAI_IP=192.168.0.148
roslaunch morai_bringup final_ws_bringup.launch \
  workspace_path:="$MORAI_WS" \
  path_file:="$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  morai_host_ip:="$MORAI_IP" \
  max_speed_kph:=3.0 fallback_speed_cap_kph:=3.0 \
  enable_control:=false
```

**이번 수정 이후 최종 launch의 `enable_control:=false`는 차량 제어 UDP와 LampControl을 모두 송신하지 않는다.** 기존에는 nominal 발행만 끄고 브리지가 브레이크를 송신했으나, 이제 `control_output_enabled=false`가 브리지까지 전달되어 제어 socket/bind/timer를 만들지 않고 종료 시에도 제어 패킷을 보내지 않는다. Ego 상태 수신은 계속 동작한다. ROS 내부 mux가 정지 명령을 발행해도 MORAI로 전달되지 않는다. 브리지의 독립 launch 기본값은 호환성을 위해 `true`이므로 다른 주행/브리지 launch를 동시에 실행하지 않는다. 실제 제어(`true`)에서는 stale 명령과 종료 시 full brake를 유지한다.

무송신은 차량의 정지를 보장하는 기능이 아니다. 이전 제어 명령이 남지 않도록 차량을 정지시키고 수동 제어 상태에서 관찰 시험을 시작한다. `rosparam get /morai_udp_drive_bridge/control_output_enabled`가 false인지 확인한다.

카메라 의존성을 준비하는 동안에는 `enable_roi_camera:=false`를 추가한 수신 진단이 가능하다. 이것은 전체 신호·카메라 안전 기능의 주행 합격이 아니다. 필요한 보정·센서가 없는 상태에서 융합 기능을 꺼서 통과시키지 않는다.

별도 source된 터미널에서 확인한다.

```bash
rostopic hz /gps
rostopic hz /Imu
rostopic hz /localization/odometry
rostopic hz /morai/lidar/live_points
rostopic hz /detection/lane
rostopic echo -n 1 /localization/gps
rostopic echo -n 1 /Ego_topic
rostopic echo -n 1 /localization/sensor_quality
rostopic echo -n 1 /experimental/curvature_progress
rostopic echo -n 1 /experimental/curvature_goal_reached
rosparam get /curvature_speed_purepursuit/max_speed_kph
rosparam get /final_ws_camera_localization_fallback/fallback_speed_cap_kph
rosparam get /final_ws_maneuver_fusion/require_route_signal_context
rosparam get /final_ws_maneuver_fusion/signal_camera
rosparam get /final_ws_maneuver_fusion/signal_mgeo_path
rostopic info /ctrl_cmd
```

`rostopic hz`는 각 명령을 Ctrl+C로 끝내거나 각각 별도 터미널에서 확인한다. `/gps`·`/Imu`는 이 launch가 UDP를 받은 뒤 생성하므로, 아무 수신 노드도 켜기 전부터 토픽이 있어야 한다는 순서로 진단하지 않는다.

| 확인 목적 | 정확한 토픽 |
|---|---|
| GPS 원본/변환/상태 | `/gps`, `/gps_mgeo`, `/localization/gps`, `/localization/gps_health`, `/localization/sensor_quality` |
| EKF / 차량 비교 | `/Imu`, `/localization/odometry`, `/localization/pose`, `/Ego_topic` |
| 경로/진행/속도 | `/experimental/curvature_reference_path`, `/experimental/curvature_progress`, `/experimental/curvature_goal_reached`, `/experimental/curvature_speed_limit`, `/experimental/curvature_speed_command`, `/experimental/curvature_steering` |
| 차선/정지선 | `/detection/lane`, `/perception/camera/lane_quality`, `/perception/camera/stopline` |
| 신호 검출/판단 | `/detection/traffic_light`, `/perception/traffic_light/state`, `/perception/traffic_light/stop_required`, `/perception/traffic_light/directional_state` |
| LiDAR/보행자/교차로 | `/perception/lidar/tracked_obstacles_map`, `/perception/pedestrian_crossing/stop_required`, `/perception/intersection/driving_unavailable` |
| 합류/안전 | `/perception/merge_gap/available`, `/perception/merge_gap/unavailable`, `/detection/fused_safety_stop` |
| 제어 판단 | `/control/maneuver_status`, `/control/turn_signal_state`, `/control/stopline_status`, `/stability/camera_fallback_status`, `/control/mux_status` |

`/detection/traffic_light`는 ROI 카메라의 `common` 메시지 계약이다. legacy `morai_perception_msgs/TrafficLight` 입력으로 혼용하지 않는다. 최종 mux의 legacy 신호 입력은 `/detection/roi_traffic_light_unused`로 분리되어 있다. 최종 카메라는 UDP를 직접 읽으므로 `/camera/front/image/compressed`나 `/lidar3D`가 반드시 존재해야 하는 것은 아니다.

현재 기본 명령 연결은 다음과 같다.

```text
/control/ctrl_cmd
  -> /control/stopline_cmd          (fusion 사용 시 wrapper는 pass-through)
  -> /control/camera_fallback_cmd
  -> /control/maneuver_cmd
  -> control_mux -> /ctrl_cmd -> MORAI UDP
```

Dry-run에서는 nominal 명령 발행이 꺼져 있어 명령 stale/정지 상태가 나올 수 있다. 이를 이유로 freshness 조건을 해제하지 않는다. 관측 가능한 센서·경로·보정 조건을 먼저 통과시키고, 아래 실제 제어 단계에서는 nominal부터 최종 명령까지 다시 확인한다.

주행 전 통과 조건은 다음과 같다.

1. 지도·Ego 모델·원점·경로 시작 위치가 확인됐고, 초기 진행거리가 시작 구간이며 goal이 false다.
2. GPS/IMU/EKF·LiDAR·lane/YOLO 입력의 주기와 timestamp가 유효하다. 초기 시험의 PP/fallback 속도 상한 파라미터가 모두 3.0km/h인지 확인한다.
3. 신규 signal_camera 보정과 MGeo 경로·신호 대응이 확인됐다. `signal_camera_uncalibrated`, `signal_unconfirmed`, `route_context_unavailable`가 있는 구간은 출발 허가로 취급하지 않는다.
4. `/detection/fused_safety_stop`, `/control/maneuver_status`, `/control/mux_status`에서 정지 이유를 설명할 수 있다. `/ctrl_cmd` 발행자는 의도한 단일 mux이고 다른 주행 launch가 없다.
5. 실제 앞범퍼 기준 정지 거리와 조향 부호를 확인할 준비가 됐다. `longlCmdType=1`, accel/brake는 0~1, ROS steering은 rad다. 속도 모니터는 km/h이며 odometry 속도는 m/s다. 0.5m는 앞범퍼 정지 목표이지 실측 오차 보증이 아니다.

통과 후 preview를 Ctrl+C로 종료하고 같은 시작 상태에서 다음을 실행한다. `enable_control`은 roslaunch 인자이므로 기존 프로세스의 parameter 하나를 바꾸는 방식으로 전환하지 않는다.

```bash
roslaunch morai_bringup final_ws_bringup.launch \
  workspace_path:="$MORAI_WS" \
  path_file:="$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
  morai_host_ip:="$MORAI_IP" \
  max_speed_kph:=3.0 fallback_speed_cap_kph:=3.0 \
  enable_control:=true
```

초기 저속 직진·짧은 곡선, 정지선 접근/정지, 해당 경로의 신호 대응, 보행자·LiDAR 정지, 방향지시등 선행 점등을 순서대로 확인한다. 방향지시등은 기본 5초 선행이며 UDP 송신 성공과 MORAI 실제 점등은 별도로 확인한다. 필요한 입력을 중단했을 때 정지하는지 확인하고, 다른 차로의 녹색 신호가 현재 경로의 출발 허가가 되지 않는지도 확인한다. GPS-only 중단 시험은 실제 센서 상태 변화와 fallback/정지를 관측하며 시나리오의 shaded area만으로 성공을 판정하지 않는다.

## 8. Reset·teleport·시나리오 재로드

시나리오 재로드, Ego 위치/자세 변경, teleport, 시간 되감기 후에는 **fusion을 포함한 최종 launch 전체를 종료하고 재시작**한다. 차량을 정지시키고 명령 수신을 통제한 뒤 리셋하고, 새 센서 상태가 들어오는 시작점에서 dry-run 조건부터 다시 확인한다.

현재 곡률 제어기는 최근 경로 선분과 단조 증가 진행거리를 유지한다. fusion에는 조작·신호 허가·점등 시간 등 상태가 있으므로 과거 상태가 새 위치에 적용되면 안 된다. MORAI에서 scene만 리셋하는 것은 ROS 내부 상태 초기화가 아니다. fusion 재시작만으로 EKF/경로 진행 상태까지 초기화되는 것도 아니다. 공식의 Reset Spawned Actors 역시 전체 ROS 상태 리셋 명령이 아니다.

## 9. Bag 기록과 외부 송신 없는 오프라인 확인

온라인 시험에서 파일명이 겹치지 않도록 새 기록 이름을 사용한다. 다음은 원본 센서와 판단·명령을 함께 저장하는 예다.

```bash
cd "$MORAI_WS/data/rosbags"
rosbag record -O scenario_drive_01.bag \
  /gps /Imu /Ego_topic /clock /tf /tf_static \
  /localization/gps /localization/odometry /localization/gps_health \
  /localization/sensor_quality \
  /experimental/curvature_reference_path /experimental/curvature_progress \
  /experimental/curvature_goal_reached /experimental/curvature_speed_command \
  /detection/lane /perception/camera/stopline \
  /detection/traffic_light /perception/traffic_light/directional_state \
  /perception/traffic_light/stop_required \
  /morai/lidar/live_points /perception/lidar/tracked_obstacles_map \
  /perception/pedestrian_crossing/stop_required \
  /perception/intersection/driving_unavailable \
  /detection/fused_safety_stop /control/maneuver_status /control/mux_status \
  /control/turn_signal_state /control/ctrl_cmd /control/stopline_cmd \
  /control/camera_fallback_cmd /control/maneuver_cmd /ctrl_cmd
```

없는 토픽은 기록되지 않는다. `/clock`도 실제 발행 여부를 확인한다. 이 목록은 YOLO/차선 결과를 저장하지만 UDP 원본 영상 자체를 자동 보존하지 않는다. 영상 추론을 다시 검증하려면 별도의 실제 영상 기록이 필요하다. `rosbag info`로 토픽·메시지 수·시간 범위를 확인한다.

**오프라인 재생에 final launch나 UDP drive/lamp bridge를 실행하지 않는다. MORAI 제어 UDP를 재연결하지 않는다. 기록 및 재계산의 모든 `/ctrl_cmd` 출력은 반드시 별도 토픽으로 remap한다.** 수정된 final의 `enable_control:=false`는 무송신이지만, 독립 브리지나 다른 송신 프로세스까지 끄지는 않으므로 오프라인에서는 별도 master·송신 노드 부재·토픽 분리를 함께 확인한다.

먼저 온라인 launch를 종료한다. 독립적인 오프라인 터미널들에서 같은 환경을 source하고 아래 환경변수를 각각 적용한다. 다음 예는 ROS master도 분리한다.

```bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
# 터미널 A
roscore -p 11321
```

같은 환경변수를 적용한 터미널 B에서는 저장된 결과를 관찰하는 재생만 실행한다.

```bash
rosnode list
rosbag info "$MORAI_WS/data/rosbags/scenario_drive_01.bag"
rosbag play --pause "$MORAI_WS/data/rosbags/scenario_drive_01.bag" \
  /ctrl_cmd:=/replay/recorded/ctrl_cmd \
  /control/ctrl_cmd:=/replay/recorded/nominal_cmd \
  /control/stopline_cmd:=/replay/recorded/stopline_cmd \
  /control/camera_fallback_cmd:=/replay/recorded/camera_fallback_cmd \
  /control/maneuver_cmd:=/replay/recorded/maneuver_cmd
```

재생 전 `rosnode list`에 drive bridge, lamp 송신 노드, 온라인 final 노드가 없는지 확인한다. 공간 키로 재생을 시작한 뒤 `/replay/recorded/ctrl_cmd`와 기록된 상태를 비교한다. 이 예는 저장된 판단을 읽는 절차이며 제어기를 다시 계산하는 검증은 아니다.

제어기 재계산을 추가할 때도 수신/송신 UDP 노드와 직접 lamp 송신을 모두 제외한 별도 오프라인 구성이 필요하다. 새 최종 명령은 `/replay/recomputed/ctrl_cmd`로 remap하고, 기록된 중간 명령과 새 명령의 이름도 분리한다. 과거 stamp·ROS clock·벽시계 freshness의 차이를 처리하지 않은 bag 재생을 주행 합격으로 판단하지 않는다. 오프라인 결과를 실제 MORAI UDP로 보내서 확인하는 절차는 사용하지 않는다.

## 10. 검증 기록에 남길 내용

실제 simulator/Launcher 버전, 지도 선택 이름, 로드된 Ego 모델, 센서 원본 파일과 camera 4 보정 근거, PC IP/포트, 사용한 경로와 신규 융합 YAML, launch 명령, 시작 위치·yaw, 각 토픽 주기, 차단 사유, bag 이름, 정지선 실측 오차와 점등 관측 결과를 기록한다.

현재 확인된 것은 로컬 파일 내용과 연결 구조다. YOLO camera 4 보정, 설치 UI 버전, 최종 지도·차량 명칭 일치, ROS 빌드 및 실제 MORAI 주행은 별도 현장 검증 항목으로 남아 있다. 신규 엄격 MGeo/신호 대응의 단위 테스트 결과가 추가되더라도 이 항목들이 자동으로 검증 완료되지는 않는다.

2026-09-08 로컬 회귀 검증: 융합/지도 연결/신호 대응 116개, 카메라 42개,
정지선 31개, LiDAR 합류 19개, UDP 브리지 14개로 총 222개 테스트가 통과했다.
ROS·UDP 경계는 mock이고 지도/경로 대조는 실제 저장 파일을 사용했다.
Python 소스 193개와 launch/package XML 49개의 구문 검사도 통과했다.
