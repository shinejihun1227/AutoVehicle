# 2번 주행 — CAM1·CAM4 화면과 정지 사유 확인

**Ubuntu 일반 터미널에서 실행한다.** 화면은 Ubuntu 브라우저에 표시된다.

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

`highway.env`에 지정된 현재 컨테이너를 갱신한다. 기존 코드·설정은 백업하고, 카메라 보정 파일과 모델 가중치는 보존한다. 별도 모델을 다시 학습하거나 교체하지 않는다.

## 실행

영상·인식 결과만 먼저 확인하려면:

```bash
bash run_test.sh monitor 2
```

**monitor에서는 차량이 움직이지 않는다.** 실제 주행은 위 실행을 `Ctrl+C`로 끝낸 뒤:

```bash
bash run_test.sh drive 2
```

Ubuntu 브라우저가 자동으로 열린다. 열리지 않으면 주소창에 **`http://127.0.0.1:8765`**를 입력한다.
새 터미널에서 다음 명령을 실행해도 된다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws" && bash run_test.sh view
```

## 화면에서 확인할 것

| 화면 | 표시 내용 |
|---|---|
| CAM1 · UDP 1101 | 현재 `highway_best.pt`의 차선·정지선 마스크, 후처리 경계, 정지선 거리, 신뢰도 |
| CAM4 · UDP 1131 | 현재 신호등 `best0902.pt`와 사물 `yolov8n.pt`의 검출 상자·분류·신뢰도, 제어기가 선택한 신호 |
| 상단 상태 | MONITOR 여부, 정지·제동 사유, 신호 연결 실패 사유 |
| 하단 값 | 실제 속도, 곡률 목표속도, 최종 가속·브레이크, 경로 일치·도착 상태 |

실제로 로드한 모델 경로·추론 시간·프레임 지연도 표시한다. 원본 보기로 같은 추론 프레임의 마스크·상자를 끌 수 있다. 데이터가 끊기면 마지막 영상에 **지연/입력 중단**을 표시한다.

## 출발하지 않을 때

| 표시 | 확인 내용 |
|---|---|
| `MONITOR — 차량 제어 송신 꺼짐` | 영상 확인 모드다. 종료 후 `drive 2`로 실행한다. |
| `camera_observation_stream_stale` | CAM1·CAM4 Destination IP가 `192.168.0.185`인지, 포트가 1101·1131인지, 모델이 실행됐는지 확인한다. |
| `signal_camera_uncalibrated` | Cam4 보정 미확인 상태다. [보정 절차](CURVATURE_SIGNAL_ONLY_KO.md)에 따라 영상·지도 투영을 검증한다. 이 값만 임의로 true로 바꾸지 않는다. |
| `unassociated_visible_signal` | 신호등 검출은 있지만 진행 경로의 신호등으로 연결하지 못했다. 선택 신호 ID와 보정값·위치를 확인한다. |
| `signal_localization_unreliable` / `odometry_or_route_unavailable` | GPS·IMU 수신과 차량의 경로상 위치를 확인한다. |
| `nominal_stale_or_not_type1` | 곡률 제어 노드가 종료됐거나 정상 명령을 내보내지 못했다. 주행 터미널 오류를 확인한다. |
| 가속 명령이 있는데 계속 정지 | MORAI 외부 제어 모드, 수신 주소 `192.168.0.147:9093`, 기어와 차량 상태를 확인한다. |

저장소 기본 Cam4는 **`calibrated: false`**다. 이것만으로 현재 정지 원인을 단정할 수는 없으며, 화면의 실제 `reason`과 `signal_selection_reason`을 함께 읽는다.

화면은 기존 추론의 결과를 최대 5 FPS로 복사해 보여준다. 카메라 UDP 수신·모델 추론을 추가로 실행하지 않고, 제어 판단이나 정지 조건도 바꾸지 않는다. 브라우저를 닫아도 주행은 계속된다.
