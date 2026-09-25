# MORAI 현장 테스트 체크리스트

**개정일: 2026-09-25 · 코드 기준: `final_ws` / `9856893`**

이 문서는 현재 GitHub에 반영된 개발 코드를 Ubuntu·MORAI에서 검증하는 절차다. 기존 링크를 유지하기 위해 파일명은 `FIELD_TEST_CHECKLIST_20260921_KO.md`를 그대로 사용한다.

**첫 시험은 `curvature` → `curvature_signal` 순서로 진행한다.** 출발·속도·신호 정지를 확인한 다음 `obstacle` → `merge` → `full`로 확장한다. 한 번에 한 프로필만 실행한다.

- [시험 순서와 실행 구성](#test-order)
- [T01 코드·컨테이너·모델 준비](#section-1)
- [T02 제어 OFF와 센서 확인](#section-2)
- [T03~T04 출발·속도 상승·곡률 주행](#section-4)
- [T05 Cam4와 경로별 신호 연결](#section-3)
- [T06~T07 신호 정지·재출발·속도 비교](#section-5)
- [T08~T10 장애물·끼어들기·통합 시험](#section-6)
- [T11 입력 소실과 복구](#section-7)
- [정지 원인 진단과 로그 저장](#section-8)
- [결과 기록표](#section-9)

<a id="test-order"></a>

## 1. 무엇을 검증하는가

이전 로컬 ROI 통합본의 `398개 테스트`, `roi_upstream_manifest.json`, `final_ws_dynamic_path_bringup.launch`를 이번 GitHub 버전의 확인 기준으로 사용하지 않는다. 현재 실행 구성은 아래와 같다.

| `TEST_PROFILE` | 실행 launch | 시험 범위 | 포함하지 않는 기능 |
| --- | --- | --- | --- |
| `curvature` | `morai_udp_ekf_purepursuit.launch` | 원래 경로의 곡률 주행·속도 제어 | 카메라·LiDAR·신호 정지·장애물 회피 |
| `curvature_signal` | `final_ws_curvature_signal.launch` | 곡률 주행 + 경로 방향에 연결된 신호·정지선 | 장애물 회피·보행자/교차 차량 정지·GPS 차선 fallback |
| `obstacle` | `final_ws_highway_bringup.launch` | 카메라 차선 경계 + LiDAR 장애물 우회 | 요청 기반 끼어들기·YOLO 신호 정지 |
| `merge` | 위와 같음 | 장애물 우회 + 요청 기반 왼쪽 차로 변경 | YOLO 신호 정지 |
| `full` | 위와 같음 | 고속도로 구성 + YOLO·정지선·보행자/교차 차량 판단 | `curvature_signal`의 경로별 좌·우회전 신호 허가·GPS fallback |

`full`이라는 이름이 모든 주행 기능을 포함한다는 뜻은 아니다. **좌회전 화살표·다른 차로 신호 구분은 `curvature_signal`에서 시험한다.** `full`은 일반 신호 상태를 사용하는 `stopline_controller`가 실행되며, 방향별 신호 허가를 대신 검증할 수 없다. `final_ws_native_no_lamps.launch`의 GPS fallback 시험은 이 절차와 별도로 진행한다.

| 순서 | 시험 | 프로필 | 권장 최초 최고속도 |
| --- | --- | --- | ---: |
| T01 | 코드·모델·실행 파일 확인 | 주행 없음 | — |
| T02 | 센서 수신·제어 OFF | 시험할 각 프로필의 `monitor` | — |
| T03 | 정지 상태 출발·재출발 | `curvature` | 3 km/h |
| T04 | 직선 가속·커브 감속·조향 | `curvature` | 5 → 10 km/h |
| T05 | Cam4 보정·경로/신호 연결 | `curvature_signal` / `monitor` | — |
| T06 | 적색 정지·유지·녹색 재출발 | `curvature_signal` | 3 → 5 km/h |
| T07 | 곡률 단독과 신호 통합 속도 비교 | 위 두 프로필 | 같은 5 → 10 km/h |
| T08 | 빈 도로·정적 장애물·회피 불가 | `obstacle` | 5 km/h |
| T09 | 요청·간격·요청 만료·차로 변경 | `merge` | 5 km/h |
| T10 | 고속도로 제어와 신호/안전 정지 결합 | `full` | 5 km/h |
| T11 | 센서 입력 소실·정지·복구 | 해당 프로필 | 3 km/h |

속도는 이번 시험의 시작값이다. 합격한 뒤에만 같은 시나리오에서 단계적으로 높인다. 최고속도 설정은 실제 주행속도의 보장값이 아니며 곡률·선행차·정지선·입력 상태에 따라 더 낮아진다.

**작성 시 확인:** 위 코드 기준으로 Windows에서 오프라인 테스트 **512개 / 7개 묶음**이 통과했다(회피 61, LiDAR 50, 카메라 73, 정지선 54, 신호/회전 208, GPS fallback 41, 곡률 25). ROS 통신과 모델 추론 경계는 모의한 검사다. Ubuntu의 빌드·GPU 추론·실제 센서 수신·MORAI 주행 결과는 현장에서 기록해야 한다.

<a id="section-1"></a>

## 2. T01 — 코드·컨테이너·모델 준비

### 2.1 Ubuntu 호스트에서 코드 받기

기존 주행을 종료한 뒤 실행한다. 개인 설정은 `highway.env`, `highway-test.env`와 별도 보정 파일에 보관한다. `git status`에 기존 수정이 있거나 `pull`이 실패하면 해당 변경을 먼저 보존하고 해결한다.

```bash
cd "$HOME/AutoVehicle"
git status --short
git branch --show-current
git switch final_ws
git pull --ff-only origin final_ws
git rev-parse HEAD
git merge-base --is-ancestor 98568937f433ec6fd1c8a2d1ad659fcac6ae9c68 HEAD
```

마지막 명령이 성공하면 이 문서의 기준 커밋을 포함한다. 실제 시험 커밋은 `git rev-parse HEAD` 결과로 기록한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
[ -f highway.env ] || cp highway.env.example highway.env
[ -f highway-test.env ] || cp highway-test.env.example highway-test.env
nano highway.env
nano highway-test.env
```

확인할 값:

| 설정 파일 | 확인 내용 |
| --- | --- |
| `highway.env` | 실제 `UBUNTU_IP`, `MORAI_IP`, `CONTAINER_NAME`, `IMAGE_NAME`; GPU 이미지는 `TORCH_FLAVOR=cu121` |
| `highway-test.env` | 첫 시험 `TEST_PROFILE=curvature`, `MAX_SPEED_KPH=3.0`; GPU 시험은 `LANE_INFO_DEVICE=cuda` |
| 같은 파일 | GPS 3001, IMU 4001, 차량 상태 1911, 차량 제어 9093, 카메라 1101/1131, LiDAR 2000/2001을 실제 MORAI 설정과 대조 |
| 같은 파일 | `LIDAR_X_M/Y_M/Z_M/YAW_DEG`는 실제 장착값; Cam4 좌표를 LiDAR 설정에 복사하지 않음 |

### 2.2 컨테이너에도 업데이트 적용

**호스트의 `git pull`만으로 기존 Docker 안의 코드가 바뀌지는 않는다.** 사용 중인 이미지의 구성에 따라 한 방법을 선택한다.

- **기존 최신 고속도로 이미지가 있고, 최근 경로 전달 수정과 신호 전용 launch만 누락된 경우:** 아래 부분 업데이트를 사용한다.
- **이미지 버전을 모르거나 오래된 이미지·새 PC인 경우:** [Docker 설치 안내](TWO_PC_DOCKER_HIGHWAY_KO.md)에 따라 현재 소스로 이미지를 빌드한다. 기존 컨테이너를 정지하고 `highway.env`에 새로운 이미지·컨테이너 이름을 지정한 뒤 `bash run_highway.sh build`, `bash run_highway.sh start`를 실행한다. 같은 이름의 기존 컨테이너를 `start`하면 새 이미지로 바뀌지 않는다. 개인 보정값도 새 컨테이너에 적용한다.

부분 업데이트 명령 — **Ubuntu 호스트**:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh stop
bash install_avoidance_transport.sh
bash install_curvature_signal.sh
bash run_highway.sh start
bash run_highway.sh shell
```

`install_avoidance_transport.sh`는 정지한 컨테이너의 지정된 파일만 갱신하고 기존 파일을 `~/morai-update-backups/`에 보관한다. `install_curvature_signal.sh`는 기존 신호 카메라 보정 파일을 보존한다. 어느 명령이든 실패하면 다음 단계로 넘기지 않는다.

### 2.3 컨테이너 안에서 검사

```bash
cd /opt/AutoVehicle/morai_ws
cat /opt/morai-build/code-revision.txt
rospack find morai_bringup
python docker/final_ws/check_morai_messages.py
python -B docker/final_ws/run_regression.py
python docker/final_ws/check_highway_launch.py
python docker/final_ws/fetch_highway_model.py
python docker/final_ws/smoke_highway_model.py
```

`code-revision.txt`는 **이미지 빌드 당시**의 커밋이다. 부분 업데이트 후에도 이전 값이 남으므로 이 값만으로 최신 파일 적용을 판단하지 않는다. 아래 T02에서 경로 전달 파라미터도 확인한다.

현재 `curvature_signal`·고속도로 프로필의 차선/정지선 모델은 **`models/highway_best.pt`**다. 예전 `lane/lane_seg_best.pt`만 확인하면 현재 실행 모델을 검증한 것이 아니다.

- 다운로드 기준: ROI `13b740689c280d04fd456b00b854efbd793c38c2`의 `models/best.pt`.
- SHA-256: `0784b486d445480640df15893bf499a0e99aa05a58e859f03fa80393435a5b3b`.
- `fetch_highway_model.py`는 이 해시를 검증하고, `smoke_highway_model.py`는 6개 클래스 모델을 **CPU에서 실제 추론**한다.
- YOLO는 `best0902.pt`와 `yolov8n.pt`를 사용한다. 커스텀 모델 누락으로 기본 모델로 대체된 로그가 있으면 신호 시험을 시작하지 않는다.

GPU를 사용하는 경우 같은 컨테이너에서 추가 확인한다.

```bash
nvidia-smi
python -c 'import torch; print(torch.__version__, torch.cuda.is_available()); assert torch.cuda.is_available(); print(torch.cuda.get_device_name(0))'
python docker/final_ws/smoke_models.py --workspace "$PWD" --device cuda
```

**통과 기준:** `MORAI_MESSAGES_OK`, `REGRESSION_PASS 7 suites`, `HIGHWAY_LAUNCH_OK`, `CURVATURE_SIGNAL_LAUNCH_OK`, `HIGHWAY_MODEL_OK`, `HIGHWAY_MODEL_INFERENCE_OK`. GPU 사용 시 추가 검사도 성공해야 한다. 검사 중 오류·traceback이 있으면 실패다. CPU 모델 검사와 CUDA 사용 가능 확인만으로 실시간 성능을 확정하지 않고 T02에서 입력 지연을 확인한다.

검사를 마쳤으면 `exit`로 호스트로 돌아온다. 아래 실행 명령은 별도 표시가 없으면 **Ubuntu 호스트**에서 수행한다.

<a id="section-2"></a>

## 3. T02 — 제어 OFF와 센서 수신

### 공통 실행 방법

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^TEST_PROFILE=.*/TEST_PROFILE=curvature/' highway-test.env
sed -i 's/^MAX_SPEED_KPH=.*/MAX_SPEED_KPH=3.0/' highway-test.env
bash run_test.sh show
bash run_test.sh monitor
```

`show`에서 프로필·IP·포트·최고속도를 확인한다. `monitor`는 센서와 계산을 실행하며 해당 UDP bridge의 차량 제어 송신을 끈다. `curvature_signal`·고속도로 프로필에서는 ROS 제어 명령도 계속 발행한다. `curvature`의 `monitor`는 제어 명령 발행까지 끄므로 `/ctrl_cmd` 메시지가 없을 수 있으며, 이때는 곡률 속도 진단 토픽을 확인한다. 다른 주행 launch나 별도 UDP 송신기가 함께 실행 중이면 안 된다.

주행으로 전환할 때는 MORAI에서 정지 상태를 확인하고, 위 터미널의 launch를 `Ctrl+C`로 종료한 뒤 `bash run_test.sh drive`를 실행한다. **시험 종료·이상 발생 시에는 MORAI에서 먼저 차량 정지/일시정지 및 외부 제어 해제를 확인한다. `Ctrl+C` 자체가 차량의 즉시 정지를 보장하지 않는다.**

### 별도 터미널에서 확인

호스트에서 `bash run_highway.sh shell`로 접속한 후 **컨테이너 안**에서 실행한다.

```bash
timeout -k 2s 5s rostopic hz /localization/odometry
timeout -k 2s 5s rostopic echo -n 1 /localization/odometry
rostopic info /ctrl_cmd
rosparam get /morai_udp_drive_bridge/control_output_enabled
```

| 프로필 | 추가 관측 토픽 | 확인할 내용 |
| --- | --- | --- |
| `curvature` | `/experimental/curvature_speed_command`, `/experimental/curvature_goal_reached` | 위치가 갱신되고 목표 도착으로 잘못 판정되지 않음 |
| `curvature_signal` | `/perception/camera/lane_info`, `/perception/camera/stopline`, `/detection/traffic_light`, `/control/maneuver_status` | 카메라 메시지 시각 갱신, 경로 연결 상태, Cam4 설정 |
| `obstacle`·`merge`·`full` | `/perception/camera/lane_info`, `/perception/lidar/tracked_obstacles_map`, `/highway_lane_strategy/state` | 실제 차선·장애물 좌표, 입력 신선도, 정지 이유 |

예: `timeout -k 2s 5s rostopic echo -n 1 /perception/camera/lane_info`. 제한 시간이 지나 종료되는 것은 관측 종료이며, 출력이 없는 경우 입력을 점검한다.

`monitor`의 `control_output_enabled`는 `false`여야 한다. 최종 `/ctrl_cmd` Publisher는 프로필별 **하나**여야 한다: `curvature`는 `/curvature_speed_purepursuit`, `curvature_signal`은 `/curvature_signal_controller`, 고속도로 프로필은 `/control_mux`. 구성이 바뀔 때 앞 프로필의 노드가 남지 않았는지 확인한다.

고속도로 프로필의 경로 전달 수정 적용 여부:

```bash
rosparam get /bypass_lane_guard/require_atomic_plan_path
rosparam get /avoidance_path_manager/require_atomic_plan_path
rosparam get /highway_lane_strategy/base_trajectory_topic
rosparam get /adaptive_curvature_purepursuit/trajectory_topic
```

순서대로 `true`, `true`, `/avoidance_path_manager/trajectory`, `/highway_lane_strategy/trajectory`여야 한다. 경로·정지 여부·목표속도를 같은 판단 메시지로 전달하는 구성이다.

차선 정보는 시각이 계속 전진하고 `frame_id=base_link`, x 전방/y 좌측이어야 한다. 정상 구간에서 `lane_valid=true`, 충분한 `confidence`와 중심선이 나오는지 본다. 기본 신선도 기준은 0.6초이며 오래된 입력을 새 데이터처럼 계속 사용하면 실패다. 정지선이 없는 구간의 `stopline.valid=false`는 정상이다. LiDAR는 RViz에서 차량 전방 장애물과 `/perception/lidar/tracked_obstacles_map`의 지도 위치가 맞는지 확인한다.

**통과 기준:** 필요한 입력이 지속 갱신되고, 단일 제어 출력·카메라 포트 중복 수신 없음·MORAI 제어 송신 OFF가 확인된다. `curvature_signal`에는 `/control/mux_status`가 없으며, 고정 경로 제어에는 `/control/curvature_status`가 없다. 해당 토픽이 없다는 이유만으로 실패 처리하지 않는다.

<a id="section-4"></a>

## 4. T03~T04 — 출발·속도 상승·곡률 주행

**장소:** 신호·장애물이 없는 원래 경로의 직선 구간. 경로 끝이나 교차로 안에서 시작하지 않는다. `curvature`는 신호와 장애물 정지를 실행하지 않는다.

1. `curvature`, `MAX_SPEED_KPH=3.0`으로 `show` → `monitor` 확인 후 `drive`한다.
2. 정지 상태에서 출발하고 다시 같은 시작점에 놓아 3회 반복한다. 이동 후 위치를 강제로 바꿨다면 launch도 다시 시작해 경로 진행 상태를 초기화한다.
3. 통과 후 `MAX_SPEED_KPH=5.0`, 다음 `10.0`으로 각각 launch를 재시작한다.
4. 같은 직선에서 가속을 확인한 뒤 커브 진입 전 감속, 커브 안 조향, 탈출 후 재가속을 확인한다.

| 관측값 | 토픽·기록 방법 |
| --- | --- |
| 계획 속도 제한 / 속도 명령 | `/experimental/curvature_speed_limit`, `/experimental/curvature_speed_command` — **km/h** |
| 실제 속도 | `/localization/odometry`의 `3.6 × sqrt(vx² + vy²)` 또는 MORAI 속도계 |
| 가속·브레이크·조향 | `/ctrl_cmd` |
| 잘못된 목표 도착 | `/experimental/curvature_goal_reached` |

**권장 판정 기준:** 조건이 정상일 때 3초 안에 속도 명령·가속 명령이 증가하고 출발한다. 직선에서 속도 명령이 안정된 뒤 실제 속도가 목표의 ±1 km/h 범위에 3초 이상 머무는지 기록한다. 커브에서 제한속도가 낮아지는 것은 정상이며 그때는 낮아진 목표와 비교한다. 차선 침범·지속적인 좌우 진동·직선에서 이유 없는 정지는 실패다. 수치는 현장 비교를 위한 기준이며 대회 규정 수치가 아니다.

출발 실패 시 최고속도를 올리지 말고 [정지 원인 진단](#section-8)의 목표속도 → 제어 명령 → UDP 순서로 원인을 구분한다.

<a id="section-3"></a>

## 5. T05 — Cam4 보정과 경로별 신호 연결

프로필을 `curvature_signal`로 바꾸고 먼저 `monitor`를 실행한다. 설정은 `SIGNAL_CONFIG_FILE`이 가리키는 **컨테이너 내부 파일**에서 읽는다.

| 항목 | 현재 입력값 / 확인 사항 |
| --- | --- |
| Camera 4 / UDP | 1131 |
| 위치 x, y, z | 3.43, 0.01, 0.61 m |
| roll, pitch, yaw | 0°, 0°, 0° |
| 원본 영상 | 640 × 480 |
| MORAI FOV | 90°; 수평/수직 축은 현장 확인 필요 |
| `horizontal_fov_deg` | 현재 90.0 후보값; **수직** FOV 90°라면 640×480의 수평값은 약 106.2602° |
| `calibrated` | 기본 `false`; 영상·지도 투영 검증 전에는 유지 |

1. MORAI에서 센서 장착 기준점과 시야각 축을 확인한다. z=0.61을 임의로 지면 기준 높이로 바꾸지 않는다.
2. 차량을 정지시킨 상태로 서로 다른 신호등 접근 위치에서 실제 영상 속 신호등과 해당 MGeo 신호 위치의 투영을 대조한다. 검출 상자만 보이는 것으로 투영 검증이 끝난 것은 아니다. 대조 결과를 확인할 수 없으면 T05를 미완료로 남긴다.
3. 보정값이 맞는 것을 확인한 뒤 개인 설정의 `calibrated: true`를 적용한다. 호스트의 `morai_ws/config/curvature_signal.yaml`을 편집했다면 launch를 종료하고 아래 명령으로 복사한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash install_curvature_signal.sh --config
bash run_test.sh show
bash run_test.sh monitor
```

`--config`는 컨테이너의 기본 보정 파일을 덮어쓴다. 기존 개인 보정값을 먼저 보관한다. `SIGNAL_CONFIG_FILE`을 별도 경로로 지정했다면 그 파일에 직접 적용해야 한다.

**컨테이너에서 실제 로드값과 상태 확인:**

```bash
rosparam get /curvature_signal_controller/signal_camera
timeout -k 2s 5s rostopic echo -n 1 /control/maneuver_status
```

**통과 기준:** `reference_path_match=true`, 영상의 해당 신호와 `selected_signal_id`가 일치하고 `route_direction`이 시험 경로 방향과 맞는다. `signal_selection_reason`에 보정 미완료·경로 미연결·신호 연결 실패가 남아 있으면 주행 시험으로 넘어가지 않는다. 상태 JSON은 투영 영상 확인을 대신하지 않는다.

`signal_camera_uncalibrated`에 의한 정지는 보정 미완료 상태에서 예상되는 동작이다. 단순히 주행시키려고 `calibrated`만 바꾸지 않는다. 자세한 설정 설명은 [곡률·신호 전용 실행 안내](CURVATURE_SIGNAL_ONLY_KO.md)를 참고한다.

<a id="section-5"></a>

## 6. T06~T07 — 신호 정지·재출발·속도 비교

### T06. 적색 정지와 방향별 허가

T05 통과 후 `curvature_signal`, 최고속도 3 km/h부터 시작한다. 정지선 앞의 같은 위치에서 각 조건을 3회 반복하고, 통과 후 5 km/h에서 다시 확인한다.

| 시험 조건 | 기대 동작 / 남길 증거 |
| --- | --- |
| 적색 신호 접근 | 정지선 전에 감속·정지. 앞 범퍼가 선을 넘지 않음 |
| 적색 유지 | 가속 0, 제동 유지, 차량이 서서히 밀려 나가지 않음 |
| 해당 진행 방향 녹색 확인 | 연속 확인 후 재출발. 짧은 한 프레임 변화만으로 출발하지 않음 |
| 직진 경로 + 좌회전만 허용 | 직진 통행을 허가하지 않음 |
| 좌회전 경로 + 직진만 허용 | 좌회전 통행을 허가하지 않음 |
| 좌회전 경로 + 좌회전 허용 | 해당 신호 연결·정지 해제 조건 충족 후 경로를 따라 좌회전 |
| 다른 차로/교차로의 녹색 | 해당 경로 신호로 잘못 선택해 출발하지 않음 |
| 인식 불명확·영상 소실 | 사유를 남기고 잘못된 출발 허가를 내지 않음 |

`SIGNAL_RIGHT_ON_GREEN=true`가 기본이다. 우회전 시험은 이 설정과 `signal_allowed_directions`를 함께 기록한다. 현재 코드의 허가 동작 검증이며 대회 규정 충족을 이 결과만으로 판정하지 않는다.

**정지 위치 기록:** `/control/maneuver_status`의 `front_bumper_distance_m`, `target_clearance_m`, `reason`, `permission`, `selected_signal_id`와 MORAI 실제 앞 범퍼 위치를 함께 저장한다. 기본 목표 여유는 0.5 m다. 권장 1차 판정은 실제 앞 범퍼가 선을 넘지 않고 선 앞 0~1.0 m에 정지하며, 속도 0.3 km/h 이하가 3초 이상 유지되는 것이다. 계산 거리와 화면의 차이가 반복되면 기준점·거리 추정을 먼저 점검한다.

`STOPLINE_FRONT_REFERENCE_OFFSET_M=3.845`는 현재 차량 기준점에서 앞 범퍼까지의 설정값이다. 거리 오차를 숨기기 위해 임의 조정하지 말고 실제 차량 모델 치수와 기준점을 대조한다. 이 프로필에는 보행자·교차 차량 회피가 없으므로 해당 객체는 이 시험 시나리오에서 분리한다.

### T07. 기존보다 속도가 안 오르는지 비교

1. 신호 영향과 장애물이 없는 동일 직선, 같은 시작 위치·최고속도·가속도·곡률 설정을 사용한다.
2. `curvature`를 실행해 5 km/h, 다음 10 km/h 결과를 저장한다.
3. launch를 종료하고 `curvature_signal`로 같은 조건을 반복한다.
4. 속도 명령·실제 속도·목표 도달 시간·`/control/ctrl_cmd`·최종 `/ctrl_cmd`·신호 상태를 비교한다.

**통과 기준:** 유효한 입력과 통행 조건에서 통합 구성도 목표속도로 수렴하며, 이유 없는 영속적인 제동·7.2 km/h 고정 제한·출발 불가가 없다. 재출발 가속 상승 제한으로 인한 짧은 차이는 따로 기록한다. 신호 판단이 정지를 요구하는 구간의 감속은 성능 저하로 분류하지 않는다.

고속도로 프로필에서도 T08의 빈 도로 조건으로 5→10 km/h 비교를 추가한다. 현재 `cruise_speed_mps`는 `MAX_SPEED_KPH/3.6`으로 연결되어 있다. `EVALUATION_SPEED_MPS=2.0`은 후보 경로 평가 설정이며 이것만 보고 차량 최고속도가 7.2 km/h라고 판단하지 않는다. 실제 제한은 `/highway_lane_strategy/state`의 `target_speed_mps`와 `/control/curvature_status`의 목표/경로 제한을 대조한다.

<a id="section-6"></a>

## 7. T08~T10 — 장애물 회피와 요청 기반 끼어들기

프로필을 바꿀 때마다 기존 launch 종료 → `highway-test.env` 수정 → `show` → `monitor` → `drive` 순서로 실행한다. 아래의 회피·차로 변경은 MORAI 시나리오에서 시험하며 LiDAR와 차선 입력이 필요하다.

### T08. `obstacle` — 장애물 우회

| 순서 | 시나리오 | 통과 기준 |
| --- | --- | --- |
| 1 | 빈 도로 | 원래 경로를 따라 진행. `NORMAL` 상태와 불필요한 정지 없음 |
| 2 | 경로 위 정적 장애물 + 우회 가능한 차선 공간 | 후보 승인 후 우회. 충돌·차선 경계 침범 없이 통과하고 원래 경로로 복귀 |
| 3 | 양옆도 막혀 유효한 경로가 없음 | `STOP_NO_SAFE_PATH` 등 사유와 함께 정지. 무리하게 통과하지 않음 |
| 4 | 우회 중 새 장애물이 확정 경로를 막음 | `AVOIDING_BLOCKED`/충돌 예측 사유를 남기고 제동. 과거 경로 허가로 계속 가속하지 않음 |
| 5 | 장애물 제거 후 정상 입력 유지 | 가능한 경로가 다시 승인되면 재출발. 반복적인 멈춤/출발이나 경로 왕복 전환이 없음 |

순서 2는 다른 위치에서도 3회 반복한다. `/bypass_lane_guard/status`, `/avoidance_path_manager/state`, `/highway_lane_strategy/state`, `/control/curvature_status`를 함께 저장한다. 정적 우회 대상 속도 기준은 launch의 `bypass_static_speed_max_mps=0.60`이며, 움직이는 차량을 정적 장애물처럼 취급하는지를 별도로 관찰한다.

### T09. `merge` — 우리 차의 왼쪽 차로 변경

`merge`를 켜는 것만으로 차로 변경이 시작되지는 않는다. 요청은 차로 변경 의사이며 차선·간격·충돌 검사를 통과해야 착수한다. 장애물 회피 자체는 요청이 없어도 동작할 수 있다.

1. 빈 도로에서 요청 없이 진행해 `mission_request_active=false`를 확인한다.
2. 왼쪽 인접 차로와 점선이 보이는 허용 구간에서, **컨테이너의 별도 터미널**에 아래 요청을 실행한다.

```bash
rostopic pub -r 5 /planning/highway_lane_change_request std_msgs/Bool 'data: true'
```

3. 간격이 충분한 상황에서 `OFF` → `WAIT_GAP` → `LANE_CHANGE` → `INNER_HOLD` 등 상태와 실제 경로를 확인한다. 후속 `REJOIN`/`DONE`은 경로·복귀 조건에 따라 달라지므로 모든 시나리오에 같은 순서를 강제하지 않는다.
4. 후방에서 빠르게 접근하는 차량·앞차 근접·실선·대상 차로 경계 미검출 조건을 각각 구성한다. 조건이 충족되기 전에는 차로 변경을 시작하지 않아야 한다.
5. **착수 전 `WAIT_GAP`에서** 요청 터미널의 `Ctrl+C`를 눌러 송신을 끝낸다. 약 1초 후 요청 만료와 미착수를 확인한다. 착수 후 요청 중단은 즉시 반대 방향 복귀를 의미하지 않는다.

기본 간격 설정은 전방 6 m, 후방 7 m, 시간 간격 1.5 s, TTC 3.0 s이며 속도와 상대 움직임에 따라 함께 판단한다. 단순 거리만 만족했다고 합격 처리하지 않는다. 관측 토픽은 `/morai/lidar/merge_gap/results`, `/perception/merge_gap/available`, `/perception/merge_gap/unavailable`이다.

**통과 기준:** 충분한 조건에서만 1회 차로 변경하고, 막힌 조건에서는 대기/감속한다. 차로 변경 후 카메라 중심선이 바뀌어도 이전 차로로 급히 꺾이지 않으며 유한한 변경 경로의 끝에서 이유 없이 멈추지 않는다. `WAIT_GAP`은 대기 상태명이며 반드시 정지를 뜻하지 않는다. 기본 `MAX_LANE_CHANGES=1`이므로 반복 시험은 차량을 재배치하고 launch를 재시작한다.

### T10. `full` — 고속도로 제어와 정지 기능 결합

T08·T09 통과 후 진행한다. Camera 1131과 YOLO 입력이 추가로 필요하다.

- 빈 도로에서 가속하고 장애물 회피·요청 차로 변경이 유지되는지 확인한다.
- 직진 신호의 적색·녹색과 정지선을 조합해 정지·재출발을 확인한다.
- 보행자 횡단과 교차 차량을 하나씩 추가해 `/detection/fused_safety_stop`, `/control/mux_status`와 최종 제동의 연결을 확인한다.
- 여러 정지 조건이 겹치면 하나가 해제되어도 나머지가 남아 있는 동안 출발하지 않아야 한다.

**통과 기준:** 위 단계에서 검증한 경로 제어를 유지하며 필요한 정지 조건을 최종 명령에 반영한다. 좌회전 허용 신호 때문에 멈추는 문제를 여기서 임의로 우회하지 말고, 경로별 신호 시험은 T06으로 분리한다.

<a id="section-7"></a>

## 8. T11 — 입력 소실과 복구

저속 또는 정지 상태에서 MORAI의 **센서 송신 하나만** 중단했다가 복구한다. 로직을 우회하는 강제 허용 토픽을 발행하지 않는다.

| 시험 | 관측·판정 |
| --- | --- |
| 위치 입력 중단 | `/localization/odometry`가 만료되면 제동하는지 확인. 현재 프로필에는 차선 기반 GPS fallback이 없으므로 차선만 보고 계속 진행하는 것을 기대하지 않음 |
| GPS 송신만 중단 | EKF가 odometry를 계속 발행할 수 있으므로 실제 위치 오차와 출력 시각을 함께 확인. GPS 소실만으로 자동 정지가 보장된다고 해석하지 않음 |
| `curvature_signal`의 Camera 1131 중단 | 관측 만료 사유와 제동 확인. 이전 녹색으로 새 교차로 진입을 허가하지 않음 |
| Camera 1101 중단 | 차선/정지선 시각이 갱신되지 않고 `STALE`로 전환되는지 확인. 고속도로에서는 새 차로 변경·미검증 우회를 승인하지 않아야 함 |
| 고속도로 LiDAR 중단 | `roi_lidar_stale` 또는 상위 입력 만료 사유와 제동 확인 |
| 센서 복구 | 새 입력·경로·허가 조건을 다시 확인한 뒤 복구. 출발 후 같은 입력이 반복 만료되는지 관찰 |

Camera 1101 소실이 모든 상태에서 즉시 정지를 뜻하지는 않는다. 정상 원경로·차로 변경 중·차로 유지 중 판단을 나누어 기록한다. 진행 중 경로가 유지되는 경우 유지 시간·속도·종료 사유를 남기고, 오래된 관측을 새 관측으로 취급하는지 확인한다.

**통과 기준:** 해당 입력을 요구하는 판단에서 만료 상태를 감지하고 필요한 제한/정지가 적용된다. 복구 실패·출발과 정지 반복은 사유와 로그를 남겨 미통과로 처리한다.

<a id="section-8"></a>

## 9. 멈추거나 속도가 안 오를 때

### 목표 → 중간 명령 → 최종 명령 → 실제 차량 순서로 확인

| 관측 | 먼저 확인할 곳 |
| --- | --- |
| 곡률 속도 명령부터 0 | 위치 만료, 경로 시작/끝 오인식, `/experimental/curvature_goal_reached`, 경로 진행량 |
| 곡률 명령은 가속인데 신호 통합 최종 명령은 제동 | `/control/maneuver_status`의 `reason`, `signal_selection_reason`, `reference_path_match` |
| 고속도로 목표 자체가 낮음 | `target_speed_mps`, `path_speed_limit_kph`, 선행차 간격·곡률·상위 정지 사유 |
| `/control/ctrl_cmd`는 가속, `/control/stopline_cmd`는 제동 | `/control/stopline_status` |
| `/control/stopline_cmd`는 가속, `/ctrl_cmd`는 제동 | `/control/mux_status`, `/detection/fused_safety_stop` |
| 최종 `/ctrl_cmd`도 가속인데 차량이 안 움직임 | `monitor`/`drive`, UDP 목적지 IP·9093, MORAI 외부 제어·기어·일시정지 상태, 중복 송신기 |

`curvature`에서는 `/ctrl_cmd`가 곡률 제어기 출력이다. `curvature_signal`에서는 `/control/ctrl_cmd` → `/ctrl_cmd`를 비교한다. 고속도로에서는 `/control/ctrl_cmd` → `/control/stopline_cmd` → `/ctrl_cmd`를 비교한다.

고속도로 프로필의 8초 읽기 전용 진단 — **별도 호스트 터미널**:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_test.sh diagnose | tee "$HOME/morai-diagnosis-$(date +%Y%m%d-%H%M%S).log"
```

`count: 0`인 필수 입력은 수신부터 점검한다. `STOP_PLANNER_NOT_READY`는 계획/입력 준비 상태, `STOP_NO_SAFE_PATH`는 승인 가능한 경로 유무, `predicted_collision_id_...`는 예측 충돌을 확인한다. 이 진단은 고속도로용이므로 `curvature_signal`에서 회피 토픽이 없다는 것은 정상이다.

### 재현용 rosbag

**컨테이너의 별도 터미널**에서 해당 시험 시작 전에 기록한다. 필요한 토픽이 없는 프로필에서는 해당 토픽의 기록이 없는 것이 정상이다.

```bash
mkdir -p /root/.ros/field-tests
rosbag record -O "/root/.ros/field-tests/run_$(date +%Y%m%d_%H%M%S).bag" \
  /localization/odometry /ctrl_cmd /control/ctrl_cmd /control/stopline_cmd \
  /experimental/curvature_speed_limit /experimental/curvature_speed_command \
  /experimental/curvature_progress /experimental/curvature_goal_reached \
  /control/maneuver_status /control/curvature_status /control/stopline_status \
  /control/mux_status /detection/fused_safety_stop \
  /perception/camera/lane_info /perception/camera/stopline \
  /detection/traffic_light /perception/traffic_light/directional_state \
  /perception/traffic_light/state /perception/lidar/tracked_obstacles_map \
  /bypass_lane_guard/status /avoidance_path_manager/state \
  /avoidance_path_manager/trajectory /highway_lane_strategy/state \
  /highway_lane_strategy/trajectory /planning/highway_lane_change_request \
  /morai/lidar/merge_gap/results /perception/merge_gap/available \
  /perception/merge_gap/unavailable
```

시험 종료 후 기록 터미널에서 `Ctrl+C`로 bag을 닫는다. `run_highway.sh` 구성의 `/root/.ros`는 Docker 로그 볼륨에 저장된다. 호스트로 가져오려면:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
source highway.env
mkdir -p "$HOME/morai-field-results"
docker --context default cp "$CONTAINER_NAME:/root/.ros/field-tests/." "$HOME/morai-field-results/"
```

Docker 명령에 권한 오류가 나면 같은 `docker --context default` 명령 앞에 `sudo`를 사용한다. bag에는 원본 카메라 영상이 포함되지 않으므로 MORAI 화면 녹화도 같이 남긴다.

<a id="section-9"></a>

## 10. 결과 기록표

시험 전 기록: 날짜 / 시험자 / 코드 커밋 / 이미지·컨테이너 / 지도·시나리오 / 차량 모델 / 시작 위치 / 실제 IP / 사용 보정 파일.

각 실행마다 `bash run_test.sh show` 결과와 실제 로드한 Cam4 파라미터를 저장한다. 파라미터를 바꾼 시험은 새 실행으로 기록한다.

| 시험 | 결과(통과/실패/미실시) | 속도·반복 횟수 | 관측값·실패 사유 | bag·영상·진단 로그 |
| --- | --- | --- | --- | --- |
| T01 코드·모델·검사 |  |  |  |  |
| T02 입력·단일 제어 출력 |  |  |  |  |
| T03 출발·재출발 |  | 3 km/h / 3회 |  |  |
| T04 직선·커브 |  | 5→10 km/h |  |  |
| T05 Cam4·신호 연결 |  | 서로 다른 접근 위치 |  |  |
| T06 적색 정지·녹색 재출발 |  | 조건별 3회 | 실제 정지 거리 / 계산 거리 |  |
| T07 속도 비교 |  | 같은 설정으로 비교 | 목표 도달 시간 / 안정 속도 |  |
| T08 우회·회피 불가 |  | 5 km/h | 최소 간격 / 복귀 / 상태 |  |
| T09 요청 차로 변경 |  | 5 km/h | 성공 / 거절 / 요청 만료 |  |
| T10 고속도로 통합 |  | 5 km/h | 정지 조건별 결과 |  |
| T11 소실·복구 |  | 3 km/h | 중단 센서 / 감지·정지 시간 |  |

**첫날 완료 목표는 T01~T07이다.** 출발 불가·속도 저하·신호 정지 문제를 각각 분리해 확인한 뒤 회피·끼어들기로 넘어간다. 오프라인 검사 통과를 현장 주행 합격으로 기록하지 않는다.

## 코드 확인 근거

- [프로필 실행 스크립트](../docker/final_ws/run_test.sh) · [시험 파라미터 기본값](../docker/final_ws/highway-test.env.example)
- [곡률·신호 전용 launch](../src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch) · [고속도로 launch](../src/bringup/morai_bringup/launch/final_ws_highway_bringup.launch)
- [차선 정보·정지선 변환](../src/detection/camera_perception/src/camera_perception/lane_info_contract.py) · [현재 차선 모델 다운로드·해시](../docker/final_ws/fetch_highway_model.py)
- [회피·끼어들기 경로 전달 수정 안내](AVOIDANCE_MERGE_VERIFICATION_KO.md) · [곡률·신호 전용 설정 안내](CURVATURE_SIGNAL_ONLY_KO.md)
