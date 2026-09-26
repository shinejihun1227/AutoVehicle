# 2번 주행 — CAM1·CAM4 화면과 정지 사유 확인

**Ubuntu 바탕화면의 일반 터미널에서 실행한다. 명령 앞에 sudo를 붙이지 않는다.** CAM1·CAM4는 RViz 또는 브라우저로 볼 수 있다.

## 업데이트 — 한 번

차량을 정지시키고 기존 주행을 `Ctrl+C`로 종료한 뒤, 블록을 하나씩 실행한다.

```bash
cd "$HOME/AutoVehicle" && git pull --ff-only origin final_ws
```

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
```

```bash
bash run_highway.sh stop && bash install_curvature_signal.sh && bash run_highway.sh start
```

`highway.env`에 지정된 현재 컨테이너의 곡률 제어 노드·계산 모듈·카메라 화면을 함께 갱신한다. 기존 코드·설정은 백업하고, 카메라 보정 파일과 모델 가중치는 보존한다.

## 실행

영상·인식 결과만 먼저 확인하려면:

```bash
bash run_test.sh monitor 2
```

**monitor에서는 차량이 움직이지 않는다.** 실제 주행은 위 실행을 `Ctrl+C`로 끝낸 뒤:

```bash
bash run_test.sh drive 2
```

## RViz로 보기

위의 `monitor 2` 또는 `drive 2`는 켜 둔다. **새 Ubuntu 터미널**에서:

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws" && bash run_test.sh rviz
```

`CAM1 - Lane and Stop Line`, `CAM4 - Traffic Light` 이미지 패널이 등록된 RViz가 열린다. 패널 제목을 끌어 나란히 배치할 수 있다. 기존 모델의 차선·정지선 마스크와 신호등 검출 상자를 보여주며, 별도 카메라 수신이나 추론을 실행하지 않는다.

| RViz Image Topic | Transport Hint | 영상 |
|---|---|---|
| `/debug/cameras/cam1/image` | `raw` | CAM1 인식 결과 |
| `/debug/cameras/cam4/image` | `raw` | CAM4 인식 결과 |

현재 주행 이미지로 화면 전용 임시 컨테이너를 띄우므로 Ubuntu 호스트에 ROS를 따로 설치할 필요가 없다. RViz를 닫으면 화면만 종료되고 주행은 계속된다. `xauth`가 없다는 메시지가 나올 때만 `sudo apt-get install xauth`를 실행한다. `No Image received`이면 위 **업데이트**를 컨테이너까지 적용했는지와 CAM1·CAM4 수신 상태를 확인한다.

RViz는 영상 확인용이다. 정지 사유는 `bash run_test.sh diagnose 2` 또는 아래 브라우저에서 확인한다. RViz 수신 카운트가 증가하는지도 확인한다.

## 브라우저로 보기

주소창에 **`http://127.0.0.1:8765`**를 입력한다. 새 Ubuntu 터미널에서 다음 명령을 실행해도 된다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws" && bash run_test.sh view
```

`view`는 서버 미응답·데스크톱 세션 없음·브라우저 실행 실패를 구분해 출력한다. 주소를 직접 입력해도 연결되지 않으면 주행 터미널의 `camera_debug_dashboard` 오류와 컨테이너 업데이트 여부를 확인한다.

## 브라우저 화면에서 확인할 것

| 화면 | 표시 내용 |
|---|---|
| CAM1 · UDP 1101 | 현재 `highway_best.pt`의 차선·정지선 마스크, 후처리 경계, 정지선 거리, 신뢰도 |
| CAM4 · UDP 1131 | 현재 신호등 `best0902.pt`와 사물 `yolov8n.pt`의 검출 상자·분류·신뢰도, 제어기가 선택한 신호 |
| 상단 상태 | MONITOR 여부, 정지·제동 사유, 신호 연결 실패 사유 |
| 하단 값 | 실제 속도, 곡률 목표속도, 최종 가속·브레이크, 경로 일치·도착 상태 |

실제로 로드한 모델 경로·추론 시간·프레임 지연도 표시한다. 원본 보기로 같은 추론 프레임의 마스크·상자를 끌 수 있다. 데이터가 끊기면 마지막 영상에 **지연/입력 중단**을 표시한다.

## 출발하지 않을 때

**2번 실행을 켜 둔 채 새 Ubuntu 터미널**에서, 아래 블록을 하나씩 실행한다. 첫 명령은 진단 도구를 호스트에 받으며 실행 중인 컨테이너는 수정하지 않는다.

```bash
cd "$HOME/AutoVehicle" && git pull --ff-only origin final_ws
```

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws" && bash run_test.sh diagnose 2
```

약 6초 동안 경로·위치·제어 입력을 수집한다. 출력 전체를 사진으로 남긴다. `pub=NONE`은 발행 노드 미등록, `count=0`은 진단 중 메시지 미수신이다. 기준 경로는 최초 1회 수신으로 정상이며, GPS·IMU의 수신 횟수만으로 위치가 유효하다고 판단하지 않는다.

`stopline`에는 검출 여부·거리·신뢰도, `lights`에는 신호등 개수·분류가 나온다. `cam1_rviz`·`cam4_rviz` 수신이 없으면 컨테이너의 화면 코드 업데이트와 카메라 추론 실행 상태를 확인한다. `nominal`은 가속인데 `final`은 브레이크이면 신호·정지선 제어의 `status.reason`을 확인한다.

| 표시 | 확인 내용 |
|---|---|
| `MONITOR — 차량 제어 송신 꺼짐` | 영상 확인 모드다. 종료 후 `drive 2`로 실행한다. |
| `reference_path_not_received` | 곡률 제어 노드의 기준 경로 미수신이다. 최신 노드는 위치 입력 전에도 경로를 발행한다. 노드 시작 오류·컨테이너 코드 버전·ROS 연결을 확인한다. |
| `camera_observation_stream_stale` | 이전 프로필에서 쓰는 사유다. 현재 센서 기반 프로필은 신호가 없다는 이유만으로 정지하지 않는다. |
| `signal_camera_uncalibrated` | MGeo 신호 투영을 사용하는 이전 모드의 사유다. 현재 센서 기반 프로필은 지도 투영 보정을 사용하지 않는다. |
| `unassociated_visible_signal` | 신호등 검출은 있지만 진행 경로의 신호등으로 연결하지 못했다. 선택 신호 ID와 보정값·위치를 확인한다. |
| `unmapped_signal_or_stopline`, `route_context_unavailable`, `no_projected_targets` | MGeo 신호 연결을 사용하는 이전 모드의 사유다. 현재 센서 기반 프로필은 MGeo 연결·지도 투영을 사용하지 않는다. |
| CAM1 정지선만 검출되고 CAM4 신호가 없음/UNKNOWN | 정지선만으로 정지하지 않고 곡률 기준 경로를 계속 따라야 한다. |
| CAM1 정지선과 유효한 CAM4 신호가 동시에 검출됨 | 경로 곡률로 추정한 방향에 신호가 허용되는지 확인한다. 빨간불·허용되지 않는 화살표는 정지한다. |
| `signal_localization_unreliable` / `odometry_or_route_unavailable` | GPS·IMU 수신과 차량의 경로상 위치를 확인한다. |
| `nominal_stale_or_not_type1` | 곡률 제어 노드가 종료됐거나 정상 명령을 내보내지 못했다. 주행 터미널 오류를 확인한다. |
| 가속 명령이 있는데 계속 정지 | MORAI 외부 제어 모드, 수신 주소 `192.168.0.147:9093`, 기어와 차량 상태를 확인한다. |

저장소 기본 Cam4는 **`calibrated: false`**다. 이것만으로 현재 정지 원인을 단정할 수는 없으며, 화면의 실제 `reason`과 `signal_selection_reason`을 함께 읽는다.

`SAFE_STOP`, `accel: 0`, `brake: 1`이면 제어기가 의도적으로 제동 중이다. `reference_path_not_received`, `odometry_or_route_unavailable`, `nominal_stale_or_not_type1`이 함께 나오면 경로·위치·곡률 명령부터 확인한다. 이 로그만으로 신호등 모델 오류라고 판단하거나 정지 조건을 해제하지 않는다.

화면은 기존 추론의 결과를 최대 5 FPS로 복사해 보여준다. 카메라 UDP 수신·모델 추론을 추가로 실행하지 않고, 제어 판단이나 정지 조건도 바꾸지 않는다. 브라우저를 닫아도 주행은 계속된다.
