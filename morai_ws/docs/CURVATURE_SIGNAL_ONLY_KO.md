# 곡률 경로 + 지정 교차로 정지선·신호등 시험

이 프로필은 기존 대회 경로를 곡률 Pure Pursuit로 추종하면서 MGeo에서 지정한 네 신호 구간의 정지선과 진행 방향을 사용한다. 네 번째 구간에는 우회전·좌회전 정지선이 연속해 있어 설정에는 총 다섯 정지선 ID가 들어간다. CAM1이 해당 정지선을 확인하고 CAM4가 유효한 진행 신호를 인식해야 통과한다. CAM4 화면상의 검출 위치를 MGeo 신호등 위치로 투영하지는 않는다.

```text
GPS / IMU → EKF 위치·속도 ─┐
곡률 기준 경로 ────────────┴→ 곡률 Pure Pursuit → 조향·속도 명령
MGeo 지정 정지선 + CAM1 확인 + CAM4 방향별 신호 ─→ 교차로 제어 → 최종 명령
```

판단 규칙은 다음과 같다.

- 지정된 정지선에 접근하면 해당 MGeo 위치를 이용해 신호가 없거나 불명이어도 정지 준비를 한다. 정지선이 보이지 않으면 허용 신호만으로 통과하지 않는다.
- CAM1 정지선 위치가 현재 지정 정지선과 8 m 이내로 맞고, CAM4 한 프레임에 유효 신호 후보가 한 신호등으로 확인되어야 신호를 적용한다. 실제 지도 위치와의 차이는 상태의 `event.stopline_match_error_m`에서 확인한다.
- 진행 방향은 해당 MGeo 연결의 직진·좌회전·우회전을 따른다. 기존 방향별 신호 판단을 적용하며, 빨강·노랑·불명 신호 또는 진행 방향과 맞지 않는 화살표에서는 멈춘다. 초록 통과는 서로 다른 프레임의 확인이 필요하다.
- 지정 목록 밖의 정지선은 신호 제어와 독립적인 정지선 속도 제한을 만들지 않는다. 기본 조향은 계속 곡률 기준 경로가 담당한다.
- CAM4 신호등의 화면 위치와 지도 신호등 ID를 연결할 보정값은 없다. 빨강과 좌회전 화살표가 각각 검출되더라도 크기와 위치가 가까워 한 신호등으로 볼 수 있을 때만 `RED_LEFT`로 묶어 좌회전만 허가한다. 겹친 동일 초록 검출은 한 후보로 처리하지만, 서로 떨어진 신호등 등 다른 복수 후보는 통과 허가를 내지 않는다. 직진은 `GREEN`을 연속된 새 프레임으로 확인한 뒤에만 재출발한다.
- 다른 구간의 목표 최고속도는 35 km/h이고, 지정 MGeo 정지선 80 m 전부터 최대 30 km/h까지 연속적으로 낮춘다. 경로를 1 m 간격으로 다시 읽어 앞쪽 급커브의 목표 속도를 계산하며, 측정 속도가 제한을 초과하면 최종 명령에서 제동한다. 페달 상승률을 제한해 평상시 가감속을 부드럽게 한다.
- 정지 목표는 앞범퍼가 정지선에서 7 m 남았을 때다. 계산된 제동 시작 지점보다 20 m 일찍 브레이크 명령을 점진적으로 올리고, 거의 정지한 상태에서 목표까지 0.8 m 이내이면 완전 정차 상태를 유지한다. CAM4 신호 프레임과 검출 객체 프레임의 도착 순서가 어긋난 순간에는 출발 허가를 보류하되, 이전에 쌓은 초록 확인 기록은 즉시 지우지 않는다. 두 프레임이 대조된 뒤에만 통과를 허가한다.

선택한 MGeo 정지선 ID와 예상 거리는 [교차로 매핑 표](ROUTE_SIGNAL_GATE_PROPOSAL_KO.md)에 정리했다. 위치가 실제 주행과 다르면 `config/curvature_signal.yaml`의 ID 목록을 수정하고 컨테이너에 다시 설치한다.

## Ubuntu에서 업데이트

기존 주행 launch를 `Ctrl+C`로 종료한 뒤 실행한다.

```bash
cd "$HOME/AutoVehicle"
git pull --ff-only origin final_ws
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh stop
bash install_curvature_signal.sh
bash run_highway.sh start
bash run_test.sh turn safe
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

## 커브에서 차선을 밟을 때

첫 큰 좌회전에서 차가 커브 바깥쪽으로 밀려 벽에 닿으면 먼저 주행 launch를
`Ctrl+C`로 종료한다. 현재 경로의 첫 급좌회전은 출발점에서 약 77~100 m이고,
기존 주행에서는 첫 좌회전 진입 속도가 약 60 km/h까지 올라간 사례가 있었다.
아래 명령은 전체 목표 최고속도를 35 km/h로 설정하고, 전방 곡률을
1 m 간격으로 계산해 급커브 진입 전에 감속하도록 한다. 조향 명령 변화율 제한은 1.2 rad/s,
lookahead gain은 0.25, 곡률 조향 피드포워드 비중은 0.50으로 설정한다.
정지선 여유 거리는 7 m, 정지 접근 계획 감속은 0.7 m/s²,
브레이크 사전 상승 구간은 20 m로 설정한다. 이 설정은
`highway-test.env.bak`에 이전 값을 보관하며 다른 시험 프로필에도 적용된다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_test.sh turn safe
bash run_test.sh monitor 2
```

모니터 확인 후 `Ctrl+C`로 종료하고 `bash run_test.sh drive 2`를 실행한다. 35 km/h는 전체 최고속도이며 첫 급좌회전의 목표속도는 곡률에 따라 훨씬 낮아진다.
벽에 닿거나 차선을 밟으면 해당 구간의 실제 속도, 경로 횡오차, 조향 명령과
앞바퀴 반응을 확인한다. 현장 실측 없이 최적 주행을 보증할 수는 없다.

이 프로필의 기본 조향은 고정된 GPS 경로를 따르며 CAM1 차선 중심을 사용하지 않는다.
커브마다 밟는 쪽이 달라지면 경로 전체를 한쪽으로 평행 이동시키지 않는다. 먼저
`/experimental/curvature_path_lateral_error_m`를 확인한다. 양수는 차량 중심이 경로의
왼쪽, 음수는 오른쪽이다. 직선에서는 거의 0이고 커브에서만 커지면 lookahead,
조향 변화율, 피드포워드, 해당 구간의 속도를 순서대로 조정한다. 직선에서도 한쪽
오차가 일정하면 경로 중심이나 GPS/EKF 정렬을 확인한다.

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_test.sh speed 5
bash run_test.sh show 2
bash run_test.sh drive 2
```

새 터미널에서 컨테이너 셸에 들어가 아래 값을 관찰한다.

```bash
bash run_highway.sh shell
rostopic echo /experimental/curvature_path_lateral_error_m
rostopic echo /detection/lane
rostopic echo /experimental/curvature_steering
```

`rostopic echo`는 계속 출력되므로 명령마다 `Ctrl+C`로 종료하고 다음 명령을 실행한다.

조향이 커브 진입보다 늦으면 `MAX_STEERING_RATE_RAD_S`를 먼저 0.5에서 1.2로
올리고 같은 커브를 비교한다. 반응은 빠르지만 커브 안쪽을 잘라 가면
`LOOKAHEAD_GAIN`을 0.35에서 0.25로 낮춰 비교한다. 경로 곡률이 조향보다
앞서 변하는 구간에서는 `STEERING_FEEDFORWARD_WEIGHT`를 0.35에서 0.50으로
올려 비교한다. 한 번에 한 값만 바꾸고 launch를 다시 시작한다. 차가 커브에서
크게 흔들리면 직전 값으로 되돌린다.

CAM1 중심선이 실제 주행 차로와 맞는 것을 확인한 뒤에는 제한된 차선 중심
보정을 켤 수 있다. CAM1의 연속된 유효 관측 세 개가 들어올 때만 적용하고,
보정 조향은 기본값으로 최대 0.06 rad이다. 신호등·정지선 제어는 기존 경로
구성을 유지한다. 이 기능은 GPS 경로의 큰 오차를 해결하는 용도가 아니다.

```bash
bash run_test.sh lane on
bash run_test.sh show 2
bash run_test.sh monitor 2
```

모니터에서 `/experimental/curvature_lane_correction_active`가 `true`이고
`/experimental/curvature_lane_correction_rad`의 방향이 차선 중심 쪽인지 확인한다.
그 뒤 launch를 종료하고 `bash run_test.sh drive 2`로 5 km/h부터 비교한다.
카메라가 차선을 잘못 잡거나 보정 방향이 틀리면 `bash run_test.sh lane off`로
끄고 launch를 다시 시작한다. 이 기능은 실제 MORAI 주행에서 아직 검증되지
않았으므로 차선 안쪽 유지가 확인될 때까지 속도를 올리지 않는다.

## 주요 설정

- 최고속도와 곡률 감속: `highway-test.env`의 `MAX_SPEED_KPH`, `LATERAL_ACCEL_LIMIT_MPS2`, `CURVE_PLANNING_DECEL_MPS2`
- 평상시 페달 변화율: `PEDAL_ACCEL_RISE_RATE_PER_SEC`, `PEDAL_BRAKE_RISE_RATE_PER_SEC`, `PEDAL_RELEASE_RATE_PER_SEC`
- 지정 MGeo 정지선 접근 속도: `STOPLINE_APPROACH_SPEED_KPH` (기본 30 km/h), 런치 인자 `mapped_stopline_approach_distance_m` (기본 80 m)
- 정지선 기준점 보정: `STOPLINE_FRONT_REFERENCE_OFFSET_M` — 앞 범퍼 위치 실측값을 사용한다.
- 정지선 여유 거리: `STOPLINE_HOLD_DISTANCE_M` (앞범퍼 기준 7 m)
- 정지 접근 제동: `STOPLINE_PLANNING_DECEL_MPS2`, `STOPLINE_BRAKE_RAMP_DISTANCE_M`, `STOPLINE_SETTLE_DISTANCE_M`
- CAM1과 MGeo 정지선 허용 오차: `config/curvature_signal.yaml`의 `route_stopline_match_tolerance_m`. 신호 허가가 나지 않으면 `mapped_stopline_confirmed`, `event.stopline_match_error_m`, `route_cam4_candidate_count`, `signal_selection_reason`을 먼저 확인한다.
- 신호 유효 시간: `config/curvature_signal.yaml`의 `signal_timeout_sec`. CAM1 정지선은 MGeo 위치와 대조해 확인한다.

이 코드는 카메라 신호등과 지도 신호등을 직접 대응시키지 않는다. CAM4가 잘못 검출한 단일 후보는 여전히 통과 판단에 영향을 줄 수 있으므로 `monitor`로 실제 교차로별 동작을 확인한다.
