# 곡률 경로 추종 + 정지선·신호등 전용 실행

`final_ws_curvature_signal.launch`, Docker 프로필 `curvature_signal`.
기존 대회 경로 `2026_molit_comp_global_path.txt`를 그대로 따라간다.
곡률로 조향·목표속도를 계산하고, 정지선·해당 경로의 신호로 가속과 브레이크를 제한한다.

```text
GPS 3001 + IMU 4001 → EKF /localization/odometry
원본 경로 + EKF → curvature_speed_purepursuit → /control/ctrl_cmd
카메라 1101 → real_lane → /perception/camera/stopline
카메라 1131 → YOLO → 신호등 위치·방향 상태
위 입력 + MGeo 경로/신호 연결 → curvature_signal_controller → /ctrl_cmd → UDP 9093
```

LiDAR, 장애물 회피, 끼어들기, 차선 조향 보정, blackout fallback, 보행자·교차 차량 정지는 실행하지 않는다.
카메라 차선 모델은 정지선 거리 측정용으로 실행된다. 방향지시등 UDP와 5초 대기도 사용하지 않는다.
신호등은 경로 방향을 바꾸지 않는다. 직진 경로는 직진 신호, 좌회전 경로는 좌회전 허용 신호를 확인한다.
기존 직진용 `stopline_controller`를 중복 실행하지 않으므로 허용된 좌회전 화살표를 다시 정지로 덮어쓰지 않는다.

## 1. 기존 Docker에 추가하기 — Ubuntu 호스트 터미널

기존 주행 터미널에서 `Ctrl+C`로 ROS 주행 launch를 끝낸 뒤 실행한다.
기존 final_ws Docker 이미지가 설치된 환경에서는 이번 추가분에 이미지 재빌드나 catkin 재빌드가 필요 없다.
새 컨테이너를 만드는 경우에는 기존 Docker 안내대로 먼저 이미지를 빌드한다.

```bash
cd "$HOME/AutoVehicle"
git pull --ff-only origin final_ws
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh start
bash install_curvature_signal.sh
```

설치 스크립트는 새 launch와 설정 파일만 기존 컨테이너에 복사한다. 차량은 출발하지 않는다.
컨테이너에 이미 `curvature_signal.yaml`이 있으면 보정값을 보존한다.
`git pull`만으로 기존 컨테이너 안의 파일은 바뀌지 않으므로 설치 명령도 필요하다.

## 2. 신호등 카메라 설정 — 처음 한 번

`morai_ws/config/curvature_signal.yaml`의 `signal_camera`에는 **MORAI Camera 1131의 실제 값**이 필요하다.
저장소 기본값은 미입력 상태다. 이 상태에서는 교차로 신호 통행을 허용하지 않는다.

| 설정 | 입력할 값 |
|---|---|
| `width`, `height` | YOLO 입력 원본 영상의 가로·세로 픽셀 |
| `horizontal_fov_deg` | 수평 시야각(도). MORAI 항목이 수평/수직 중 무엇인지 확인 |
| `x`, `y`, `z` | 차량 기준점에서 카메라 광학 중심까지 거리(m), x 전방/y 좌측/z 위 |
| `pitch_deg`, `yaw_deg` | 이 투영 코드의 pitch는 위를 향하면 양수, yaw는 좌향 양수. MORAI 표기 축을 확인 |
| `calibrated` | 위 값과 영상의 지도 신호등 투영이 맞는 것을 확인한 뒤 `true` |

카메라 roll은 0인 모델이다. 차선 카메라 1101의 값을 대신 사용하거나 `calibrated`만 바꾸지 않는다.

**호스트에서 설정을 편집하고 컨테이너에 적용:**

```bash
nano "$HOME/AutoVehicle/morai_ws/config/curvature_signal.yaml"
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash install_curvature_signal.sh --config
```

`--config`는 호스트의 설정 파일로 컨테이너 설정을 덮어쓴다. 적용은 다음 launch 실행부터다.
별도 파일을 쓰려면 `highway-test.env`의 `SIGNAL_CONFIG_FILE`에 **컨테이너 내부 경로**를 지정한다.

## 3. 프로필 선택과 실행 — Ubuntu 호스트 터미널

`highway.env`의 `MORAI_IP=192.168.0.147`, `UBUNTU_IP=192.168.0.185`를 실제 주소와 맞춘다.
다음 명령은 이전에 사용한 최고속도 45km/h를 선택한다. 커브·정지선에서는 이보다 낮은 목표속도를 사용한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^TEST_PROFILE=.*/TEST_PROFILE=curvature_signal/' highway-test.env
sed -i 's/^MAX_SPEED_KPH=.*/MAX_SPEED_KPH=45.0/' highway-test.env
bash run_test.sh show
bash run_test.sh drive
```

차량 명령을 MORAI에 보내지 않고 계산·인식만 확인하려면 `drive` 대신 `monitor`를 실행한다.
`monitor`에서도 ROS 제어 토픽은 생성되며, 이 launch가 시작한 UDP bridge의 제어 송신만 비활성화된다.
동시에 다른 주행 launch/UDP 송신기를 실행하지 않는다.

## 4. 실행 상태 확인 — 새 터미널

**호스트에서 컨테이너 접속:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh shell
```

**컨테이너 안에서:**

```bash
timeout -k 2s 5s rostopic echo -n 1 /control/maneuver_status
timeout -k 2s 5s rostopic echo -n 1 /perception/camera/stopline
timeout -k 2s 5s rostopic echo -n 1 /perception/traffic_light/directional_state
timeout -k 2s 5s rostopic echo -n 1 /control/ctrl_cmd
timeout -k 2s 5s rostopic echo -n 1 /ctrl_cmd
rostopic info /ctrl_cmd
```

최종 `/ctrl_cmd`의 Publisher는 `/curvature_signal_controller` 하나다.
이 launch에는 `/control/mux_status`, 회피 경로 관리 상태 토픽이 없다.
핵심 상태는 `/control/maneuver_status`의 `reason`, `signal_selection_reason`, `route_direction`,
`signal_allowed_directions`, `permission`, `reference_path_match`다.

| 상태/사유 | 의미 |
|---|---|
| `signal_camera_uncalibrated` | 1131 카메라 보정이 미완료 |
| `route_context_unavailable` | 경로와 MGeo 신호 연결을 만들지 못함 |
| `unassociated_visible_signal` | 화면의 신호등을 경로에 해당하는 지도 신호등으로 확정하지 못함 |
| `camera_observation_stream_stale` | 카메라/신호 관측 메시지가 끊김 |
| `odometry_or_route_unavailable` | 위치·속도 또는 경로 투영이 유효하지 않음 |
| `reference_path_not_received` 또는 `reference_path_match=false` | 곡률 제어기의 전체 경로를 확인하지 못했거나 불일치 |
| `awaiting_confirmed_green` | 현재 경로 방향에 맞는 허용 신호 확인을 기다림 |

`UNKNOWN`인 새 영상과 영상 수신 끊김은 구분한다. 실제 입력 손실·빨간불·미확인 신호는 정지를 유지한다.
초록불 단일 프레임이나 다른 차로 신호만으로 정지를 해제하지 않는다.

## 5. 다시 켰을 때

보정과 프로필은 저장되어 있다. 매번 설치·복사·sed를 반복할 필요 없다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh start
bash run_test.sh show
bash run_test.sh drive
```

속도는 `highway-test.env`의 `MAX_SPEED_KPH`, 커브 감속은 `LATERAL_ACCEL_LIMIT_MPS2`,
경로 조향은 `LOOKAHEAD_*`, `STEERING_FEEDFORWARD_WEIGHT`, `MAX_STEERING_RATE_RAD_S`로 조정한다.
정지선 여유는 `STOPLINE_HOLD_DISTANCE_M`, 감속 계획은 `STOPLINE_PLANNING_DECEL_MPS2`로 조정한다.
`STOPLINE_FRONT_REFERENCE_OFFSET_M`는 위치 기준점에서 앞 범퍼까지의 거리이며 시험으로 임의 조정하는 값이 아니다.
값을 바꾸면 실행 중인 launch를 `Ctrl+C`로 끝내고 다시 `drive`한다.

오프라인 검증은 ROS/UDP 및 영상 추론 경계를 모의한 테스트다. 실제 카메라 보정과 MORAI 정지 위치 검증은 별도다.
