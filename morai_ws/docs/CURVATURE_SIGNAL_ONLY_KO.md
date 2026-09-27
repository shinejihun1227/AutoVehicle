# 곡률 경로 + 카메라 정지선·신호등 시험

이 프로필은 기존 대회 경로를 곡률 Pure Pursuit로 추종하면서 CAM1 정지선과 CAM4 신호 인식을 함께 확인한다. MGeo의 교차로·신호등 연결과 지도상의 신호 투영은 사용하지 않는다.

```text
GPS / IMU → EKF 위치·속도 ─┐
곡률 기준 경로 ────────────┴→ 곡률 Pure Pursuit → 조향·속도 명령
CAM1 정지선 + CAM4 신호 인식 ─→ 신호 조건 제어 → 최종 명령
```

판단 규칙은 단순하다.

- CAM1 정지선만 보이고 CAM4에서 유효한 신호를 인식하지 못하면 정지선만으로 멈추지 않고 곡률 경로를 계속 주행한다.
- 정지선이 먼저 검출되면 신호 인식 여부와 별개로 속도 목표를 기본 30 km/h로 낮춘다. 제한은 정지선의 경로 위치를 기억해 차량 진행도가 정지선을 지난 뒤 기본 5 m 더 이동할 때 해제한다. 그 뒤에는 곡률 속도 계획으로 복귀한다.
- 정지선과 유효한 신호가 함께 관측되면, 정지선 전후의 실제 기준 경로 곡률로 진행 방향을 추정하고 신호 상태를 적용한다.
- 경로 방향을 허용하지 않는 빨간불·화살표는 정지하고, 허용 신호는 정지선을 통과한다. CAM4 신호만 있고 정지선이 없으면 신호만으로 정지하지 않는다.
- 조향은 계속 기존 곡률 기준 경로가 담당한다. 이 시험 모드는 MGeo 기반 신호 연결을 대신할 만큼 신호등과 차로의 연관성을 보장하지 않으므로 MORAI 시뮬레이션에서만 검증한다.

## Ubuntu에서 업데이트

기존 주행 launch를 `Ctrl+C`로 종료한 뒤 실행한다.

```bash
cd "$HOME/AutoVehicle"
git pull --ff-only origin final_ws
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh stop
bash install_curvature_signal.sh
bash run_highway.sh start
bash run_test.sh models
```

`bash run_test.sh monitor 2` 또는 `drive 2` 실행 시 먼저 컨테이너 안의 launch와 제어 노드가 센서 전용 버전인지 검사한다. 구버전이면 주행 launch를 시작하지 않고 업데이트 안내를 출력한다. 정상 실행 중 대시보드 상세 상태의 제어 모드는 `sensor_only`여야 한다.

설치 후 모니터 모드로 인식과 명령을 먼저 확인한다.

```bash
bash run_test.sh monitor 2
```

CAM1에서 정지선이 보이지만 CAM4 신호가 `UNKNOWN` 또는 무효인 구간에서는 최종 명령이 제동으로 바뀌지 않아야 한다. 정지선과 CAM4 유효 신호가 동시에 들어오는 경우에만 신호 상태에 따른 제어가 작동하는지 확인한다. 실제 차량 제어는 확인이 끝난 뒤 `monitor`를 종료하고 `drive`로 실행한다.

Ubuntu 브라우저는 `http://127.0.0.1:8765`에서 확인한다. 상세 상태에서 `route_direction`, `signal`, `signal_selection_reason`, `reason`, `accel`, `brake`를 본다. 지도 투영 분홍 표시는 이 모드에서 사용하지 않는다.

CAM1 카드에 `정지선 접근 속도 제한 중 · 목표 30.0 km/h`가 보이면 제한이 적용된 상태다. 정지선을 통과한 뒤 5 m 지점부터 자동 해제된다. 주요 설정은 Ubuntu의 `highway-test.env`에 있으며 `STOPLINE_APPROACH_SPEED_KPH`, `STOPLINE_CAP_RELEASE_AFTER_M`, `STOPLINE_CAP_MAX_DETECTION_RANGE_M`, `STOPLINE_CAP_MIN_CONFIDENCE`를 바꾼 뒤 주행 launch를 다시 시작한다.

## CAM4 모델 업데이트 (2026-09-27)

ROI `dev/merged_sensor`의 `a994022`까지 확인해 CAM4 기본 사물 모델을 `yolov8s.pt`, 신호·장애물 모델을 `best0917.pt`로 교체했다. 새 모델의 `Green`, `Red`, `Yellow`, `red_left`, `red_yellow` 라벨은 기존 방향별 신호 판단에 연결한다. 같은 신호등을 추적할 수 있으면 녹색·진행 화살표는 최근 5프레임 중 3회 확인한 뒤 전달하고, 빨간색·노란색은 해당 프레임부터 전달한다. 모델 추론 결과의 원본 프레임 시각과 CAM1·CAM4 대시보드는 유지한다.

`best0917.pt`는 위 ROI 커밋의 파일이며 SHA-256은 `6812d43beda5ab6ff18881198f34a49f79d641efbf2768220b34c74dd11aa9f5`다. ROI 브랜치에는 `yolov8s.pt` 파일이 없어서 [Ultralytics 공식 v8.3.0 배포본](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s.pt)을 함께 저장했다(SHA-256 `1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a`).

원본 브랜치의 중앙 일부만 남기는 가로 크롭은 적용하지 않았다. CAM4 화면 오른쪽의 신호등도 필요하며, 전체 프레임에서 사용자 차량에 보이는 신호 후보를 계속 확인해야 하기 때문이다. 새 모델 파일은 `install_curvature_signal.sh`가 체크섬을 검사한 뒤 정지된 도커 컨테이너에 복사한다. 컨테이너 업데이트 후 `bash run_test.sh models`로 모델 로딩·추론을 확인한다. 이어 `monitor 2`에서 CAM4 모델 표시가 `best0917.pt · 로드됨`인지 확인하고 빨강/초록 전환을 관찰한다.

좌·우회전 조향이 늦으면 `highway-test.env`의 `MAX_STEERING_RATE_RAD_S`가 0.5처럼 낮지 않은지 먼저 확인한다. 0.5 rad/s에서는 핸들 명령 변화가 느리게 제한된다. 시뮬레이터 모니터 시험의 시작값으로 `MAX_STEERING_RATE_RAD_S=1.2`, `STEERING_FEEDFORWARD_WEIGHT=0.50`, `LOOKAHEAD_GAIN=0.25`를 한 항목씩 적용한다. `LOOKAHEAD_GAIN`을 낮추면 속도가 높을 때 조향 목표점이 가까워져 반응이 빨라지고, 피드포워드 비중을 높이면 경로 곡률에 더 일찍 반응한다. 급격한 조향이나 좌우 흔들림이 생기면 직전 값으로 되돌린다. 물리 조향 각도 제한인 `max_steering_rad`와 조향 부호는 차량 설정을 확인하지 않고 바꾸지 않는다.

## 주요 설정

- 최고속도와 곡률 감속: `highway-test.env`의 `MAX_SPEED_KPH`, `LATERAL_ACCEL_LIMIT_MPS2`
- 정지선 기준점 보정: `STOPLINE_FRONT_REFERENCE_OFFSET_M` — 앞 범퍼 위치 실측값을 사용한다.
- 정지선 여유 거리: `STOPLINE_HOLD_DISTANCE_M`
- 정지선과 신호 관측 동시성: CAM1·CAM4 입력 프레임의 시각 차이가 `signal_timeout_sec` 이내여야 한다.

이 코드는 카메라·지도 정합을 검증한 완성된 도로 주행 시스템이 아니다. 카메라 인식이 끊기면 이 모드에서 정지선 제어를 하지 않으므로, 우선 `monitor`로 충분히 확인하고 시뮬레이터에서만 시험한다.
