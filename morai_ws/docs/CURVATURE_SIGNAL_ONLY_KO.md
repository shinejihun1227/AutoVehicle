# 곡률 경로 + 지정 교차로 정지선·신호등 시험

이 프로필은 기존 대회 경로를 곡률 Pure Pursuit로 추종하면서 MGeo에서 지정한 네 신호 구간의 정지선과 진행 방향을 사용한다. 네 번째 구간에는 우회전·좌회전 정지선이 연속해 있어 설정에는 총 다섯 정지선 ID가 들어간다. CAM1이 해당 정지선을 확인하고 CAM4가 유효한 진행 신호를 인식해야 통과한다. CAM4 화면상의 검출 위치를 MGeo 신호등 위치로 투영하지는 않는다.

```text
GPS / IMU → EKF 위치·속도 ─┐
곡률 기준 경로 ────────────┴→ 곡률 Pure Pursuit → 조향·속도 명령
MGeo 지정 정지선 + CAM1 확인 + CAM4 방향별 신호 ─→ 교차로 제어 → 최종 명령
```

판단 규칙은 다음과 같다.

- 지정된 정지선에 접근하면 해당 MGeo 위치를 이용해 신호가 없거나 불명이어도 정지 준비를 한다. 정지선이 보이지 않으면 허용 신호만으로 통과하지 않는다.
- CAM1 정지선 위치가 현재 지정 정지선과 2 m 이내로 맞고, CAM4 한 프레임에 유효 신호 후보가 하나여야 신호를 적용한다.
- 진행 방향은 해당 MGeo 연결의 직진·좌회전·우회전을 따른다. 기존 방향별 신호 판단을 적용하며, 빨강·노랑·불명 신호 또는 진행 방향과 맞지 않는 화살표에서는 멈춘다. 초록 통과는 서로 다른 프레임의 확인이 필요하다.
- 지정 목록 밖의 정지선은 신호 제어와 독립적인 정지선 속도 제한을 만들지 않는다. 조향은 계속 곡률 기준 경로가 담당한다.
- CAM4 신호등의 화면 위치와 지도 신호등 ID를 연결할 보정값은 없다. 여러 후보가 동시에 보이면 통과 허가를 내지 않는다. 이 구성을 MORAI 모니터 모드에서 먼저 확인한다.

선택한 MGeo 정지선 ID와 예상 거리는 [교차로 매핑 표](ROUTE_SIGNAL_GATE_PROPOSAL_KO.md)에 정리했다. 위치가 실제 주행과 다르면 `config/curvature_signal.yaml`의 ID 목록을 수정하고 컨테이너에 다시 설치한다.

## Ubuntu에서 업데이트

기존 주행 launch를 `Ctrl+C`로 종료한 뒤 실행한다.

```bash
cd "$HOME/AutoVehicle"
git pull --ff-only origin final_ws
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh stop
bash install_curvature_signal.sh --config
bash run_highway.sh start
bash run_test.sh models
```

`bash run_test.sh monitor 2` 또는 `drive 2` 실행 시 컨테이너의 launch, 제어 노드, 선택 정지선 설정과 MGeo 파일을 검사한다. 구버전이면 주행 launch를 시작하지 않는다. 정상 실행 중 대시보드 상세 상태의 제어 모드는 `route_camera`여야 한다.

설치 후 모니터 모드로 인식과 명령을 먼저 확인한다.

```bash
bash run_test.sh monitor 2
```

선택 목록 밖의 정지선에서는 신호 제어 제동이 생기지 않아야 한다. 선택된 교차로에서는 CAM4 신호가 `UNKNOWN` 또는 무효일 때 정지하고, CAM1 위치가 MGeo 정지선과 맞는지 `mapped_stopline_confirmed`로 확인한다. 진행 방향을 허용하는 신호가 확인된 뒤 통과하는지 관찰한다. 실제 차량 제어는 확인이 끝난 뒤 `monitor`를 종료하고 `drive`로 실행한다.

Ubuntu 브라우저는 `http://127.0.0.1:8765`에서 확인한다. 상세 상태에서 `route_direction`, `signal`, `signal_selection_reason`, `reason`, `accel`, `brake`를 본다. 지도 투영 분홍 표시는 이 모드에서 사용하지 않는다.

CAM1 카드의 독립적인 정지선 접근 속도 제한은 이 프로필에서 꺼져 있다. 지정 교차로의 감속과 정지는 최종 제어 노드가 담당한다.

## CAM4 모델 업데이트 (2026-09-27)

ROI `dev/merged_sensor`의 `a994022`까지 확인해 CAM4 기본 사물 모델을 `yolov8s.pt`, 신호·장애물 모델을 `best0917.pt`로 교체했다. 새 모델의 `Green`, `Red`, `Yellow`, `red_left`, `red_yellow` 라벨은 기존 방향별 신호 판단에 연결한다. 같은 신호등을 추적할 수 있으면 녹색·진행 화살표는 최근 5프레임 중 3회 확인한 뒤 전달하고, 빨간색·노란색은 해당 프레임부터 전달한다. 모델 추론 결과의 원본 프레임 시각과 CAM1·CAM4 대시보드는 유지한다.

`best0917.pt`는 위 ROI 커밋의 파일이며 SHA-256은 `6812d43beda5ab6ff18881198f34a49f79d641efbf2768220b34c74dd11aa9f5`다. ROI 브랜치에는 `yolov8s.pt` 파일이 없어서 [Ultralytics 공식 v8.3.0 배포본](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s.pt)을 함께 저장했다(SHA-256 `1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a`).

원본 브랜치의 중앙 일부만 남기는 가로 크롭은 적용하지 않았다. CAM4 화면 오른쪽의 신호등도 필요하며, 전체 프레임에서 사용자 차량에 보이는 신호 후보를 계속 확인해야 하기 때문이다. 새 모델 파일은 `install_curvature_signal.sh`가 체크섬을 검사한 뒤 정지된 도커 컨테이너에 복사한다. 컨테이너 업데이트 후 `bash run_test.sh models`로 모델 로딩·추론을 확인한다. 이어 `monitor 2`에서 CAM4 모델 표시가 `best0917.pt · 로드됨`인지 확인하고 빨강/초록 전환을 관찰한다.

좌·우회전 조향이 늦으면 `highway-test.env`의 `MAX_STEERING_RATE_RAD_S`가 0.5처럼 낮지 않은지 먼저 확인한다. 0.5 rad/s에서는 핸들 명령 변화가 느리게 제한된다. 시뮬레이터 모니터 시험의 시작값으로 `MAX_STEERING_RATE_RAD_S=1.2`, `STEERING_FEEDFORWARD_WEIGHT=0.50`, `LOOKAHEAD_GAIN=0.25`를 한 항목씩 적용한다. `LOOKAHEAD_GAIN`을 낮추면 속도가 높을 때 조향 목표점이 가까워져 반응이 빨라지고, 피드포워드 비중을 높이면 경로 곡률에 더 일찍 반응한다. 급격한 조향이나 좌우 흔들림이 생기면 직전 값으로 되돌린다. 물리 조향 각도 제한인 `max_steering_rad`와 조향 부호는 차량 설정을 확인하지 않고 바꾸지 않는다.

## 주요 설정

- 최고속도와 곡률 감속: `highway-test.env`의 `MAX_SPEED_KPH`, `LATERAL_ACCEL_LIMIT_MPS2`
- 정지선 기준점 보정: `STOPLINE_FRONT_REFERENCE_OFFSET_M` — 앞 범퍼 위치 실측값을 사용한다.
- 정지선 여유 거리: `STOPLINE_HOLD_DISTANCE_M`
- 신호 유효 시간: `config/curvature_signal.yaml`의 `signal_timeout_sec`. CAM1 정지선은 MGeo 위치와 대조해 확인한다.

이 코드는 카메라 신호등과 지도 신호등을 직접 대응시키지 않는다. CAM4가 잘못 검출한 단일 후보는 여전히 통과 판단에 영향을 줄 수 있으므로 `monitor`로 실제 교차로별 동작을 확인한다.
