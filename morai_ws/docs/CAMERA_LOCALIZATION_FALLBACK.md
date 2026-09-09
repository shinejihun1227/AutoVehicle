# GPS 음영구간 차선 유지와 경로 주행 복귀

## 최종 통합 구성

현재 기본 실행은 `morai_bringup/final_ws_bringup.launch`다.
기존 곡률 속도 계획 + PI accel/brake + Pure Pursuit 뒤에서 차선 fallback이
조향·속도를 제한하고, maneuver_fusion과 control_mux가 최종 안전 조건을 적용한다.
차선 유지는 GPS 위치를 복원하거나 새로운 회전/차선 변경 경로를 만드는 기능이 아니다.

| 상황 | 현재 동작 |
|---|---|
| 센서 정상, 정상 주행 중 | 기존 경로 조향 사용, 잘못된 명령·속도 입력은 정지 |
| GPS만 음영, 진입 확인 중 | 기본 0.5초 확인 동안 정지; degraded 보조 주행으로 빠지지 않음 |
| GPS만 음영, 차선·IMU·속도 유효 | 경로 조향을 섞지 않고 차선 조향 사용, 기본 7.2km/h 상한 |
| 차선 일시 소실/품질 저하 | 마지막 확정 영상 이후 최대 0.25초 기존 조향 유지 + 가속 0 + 최소 brake 0.2 |
| 소실 지속·심한 차선 튐·IMU/속도/품질 상태 불량 | 마지막 조향을 유지하며 제동; 차륜을 갑자기 중앙으로 꺾지 않음 |
| GPS 정상 복구 | 최소 1초 연속 정상 확인 후 조향 변화율 제한으로 경로 제어 복귀 |
| 교차로/회전 판단 필요 또는 음영 주행 예산 초과 | 차선 중심만 따라 진입하지 않고 정지 |

brake 0.2는 정규화된 명령값이지 보장된 감속도가 아니다. 속도 상한도 실제
차량 응답을 검증해야 한다. 경로 복귀 중에도 저속 상한을 적용하며, 가속 페달
상승은 초당 최대 0.5, 조향 변화는 기본 초당 0.8rad로 제한한다.
IMU까지 불량한 SENSOR_DEGRADED 상태는 예전 PP+차선 혼합 제어 대신 정지한다.
레거시 `stop_without_camera=false`와 `fallback_nominal_weight`로 안전 조건을 해제하지 않는다.

## 차선 중심으로 조향하는 계산

ROI 모델의 자차 좌표는 x가 전방, y가 좌측이다. `lateral_offset_m`은
전방 7m 차로 중심 y의 음수이며, 양수이면 차로 중심이 차량 오른쪽에 있다는 뜻이다.
`heading_error_rad`는 7m와 14m 중심점을 잇는 방향이며 왼쪽 방향이 양수다.
따라서 횡오차를 양수 조향에 그대로 더하던 기존 식은 복원 방향이 반대였다.

현재는 두 관측점을 복원해 각각 Pure Pursuit 곡률을 계산한다.

```text
y7  = -lateral_offset_m
y14 = y7 + (14 - 7) * tan(heading_error_rad)
k7  = 2*y7  / (7²  + y7²)
k14 = 2*y14 / (14² + y14²)
curvature = (0.8*k7 + 0.7*k14) / (0.8 + 0.7)
steering = atan(wheelbase * curvature)
```

축간거리는 기존 경로 제어와 같은 기본 3m다. 이후 조향 각도·변화율 제한을 적용한다.
방향오차를 조향각에 직접 더하면 전방 도로 곡률까지 중복 보정해 곡선 안쪽으로
치우칠 수 있으므로, 14m 목표점 복원에 사용한다. `lateral_gain`과 `heading_gain`은
이제 두 목표점의 **곡률 가중치**이며, 종전 식의 조향 증폭 계수와 의미가 다르다.
`lane_preview_distance_m=7`, `lane_heading_distance_m=14`는 ROI 관측 기준과
일치해야 한다. 이 거리를 임의로 바꾸어 조향 세기를 조정하면 안 된다.

## 차선과 입력을 신뢰하는 조건

- 차선·속도·품질 메시지의 원본 ROS timestamp와 로컬 수신시간을 모두 검사한다.
  0 timestamp, 오래된/과도한 미래 데이터, 순서가 뒤바뀐 데이터는 새 증거로 세지 않는다.
  허용하는 미래 시각 오차는 최대 50ms이며 정밀 센서 동기화의 보장은 아니다.
- 차선 confidence 0.80 이상인 서로 다른 영상 5개와 최소 0.20초 연속 확인이 필요하다.
  영상 시각과 실제 수신 시각 모두 확인하므로, 밀린 영상의 일괄 수신만으로 출발하지 않는다.
  중간 품질 샘플을 누적한 뒤 좋은 영상 한 장으로 전환하지 않는다.
  최신 조향 관측의 허용 나이는 기본 0.15초(`lane_control_timeout_sec`)다.
  이를 넘으면 가속을 없애고 제동하며, 새 영상으로 연속 검증을 다시 시작한다.
  필터에도 원본·수신 시각을 저장해 0.3초보다 오래된 오차를 제거한다.
  같은 시각의 영상이 상충하는 결과로 도착하면 기존 차선 허가를 폐기한다.
- 횡오차/방향오차가 NaN이거나 설정 범위(기본 2m/0.6rad)를 넘으면 사용하지 않는다.
  급격한 프레임 변화(0.75m 또는 0.30rad 초과)도 새 안정화가 필요하다.
- ROI 품질 계산은 7~14m를 0.5m 간격으로 검사한다. 각 차선의 실제 검출 범위가
  7m와 14m를 모두 포함해야 하며, 모든 검사점의 좌우 폭은 3.3±0.8m여야 한다.
  좌우 차선 뒤바뀜, 중간·먼 거리에서의 교차/벌어짐, 근거가 없는 외삽은
  높은 가중평균 점수로 통과할 수 없도록 기하 조건으로 차단한다.
  한쪽 차선만 보이는 경우는 주 제어 품질에 도달하지 못한다.
- 센서 품질 감시는 유효하지 않은 GPS health를 NORMAL로 처리하지 않는다.
  IMU의 유한한 측정값·자세와 원본 시각을 확인한다. 이는 모든 센서 노이즈를
  검출한다는 뜻이 아니며 편향·공분산·지도 정합 오차는 별도 검증이 필요하다.

ROS 시간이 되돌아가면 이력과 이전 허가를 폐기한다. 시뮬레이션이 멈춘 동안
벽시계 시간만 흘렀다고 진입·복구 확인 시간을 채우지 않는다.
제어 주기 사이에 짧게 들어온 GPS 불량도 복구 대기를 다시 시작시킨다.
음영구간에서 속도를 잃으면 이동거리를 확인할 수 없으므로 주행 예산을 잠그며,
속도만 돌아왔다고 재출발하지 않는다. 정상 GPS 확인과 조향 복귀 완료가 필요하다.

## 최종 통합 제어의 음영구간 예외

차선 노드가 명령을 만들어도 최종 주행이 자동으로 허용되는 것은 아니다.
maneuver_fusion은 최신 `/stability/camera_fallback_status`와 명령을 함께 검사한다.

허용 조건은 모두 충족해야 한다.

1. 음영 진입 전에 정상 GPS·지도 경로 위치·정상 주행 상태를 확인한 이력이 있음.
2. IMU와 EKF 속도/경로 투영이 유효하고, 차선 제어가 유효한 상태임.
3. 활성 회전/차선 변경이 없으며, 지도상 다음 조작의 접근 범위 밖임.
   접근 범위는 최소 40m이며 속도에 따라 늘어남.
4. 최신 영상에 정지선·신호등이 없고, 교차로 검출 상태도 최신의 비활성 상태임.
   영상 스트림이 끊긴 것을 '아무것도 없음'으로 처리하지 않음.
5. 정상 위치 이후 기본 15초와 이동 30m의 예산을 모두 만족함.
6. LiDAR·보행자 등 최종 안전 입력에 정지 요구가 없음.

하나라도 빠지면 정지한다. 음영 안에서 처음 노드를 켠 경우는 정상 위치 이력이
없으므로 차선만 보고 출발하지 않는다. 복구 도중 GPS가 잠깐 정상으로 나타나도
예산을 초기화하지 않는다. 제한은 정상 확인과 조향 복귀 완료 후 초기화한다.
15초·30m는 검증되지 않은 긴 추측 주행을 제한하는 보수적 기본값이며,
긴 음영구간 완주를 보장하지 않는다. 실제 지도·센서 오차 검증 없이 늘리지 않는다.

차선 주행 예외는 교차로 신호 허가가 아니다. 새로운 좌우회전·합류·장애물 회피
경로를 자동 생성하지 않으며, GPS 음영구간 차선 패널티 예외 규정과도 별개다.
mission_evaluator의 구역 판정이나 대회 채점 규칙은 이 변경에서 수정하지 않았다.

## 실행 및 확인

Ubuntu ROS Noetic 환경에서 빌드 후 우선 미리보기로 검증한다.

```bash
cd ~/morai_ws
catkin_make
source devel/setup.bash
roslaunch morai_bringup final_ws_bringup.launch \
  enable_control:=false \
  fallback_speed_cap_kph:=7.2 \
  blackout_max_duration_sec:=15.0 \
  blackout_max_distance_m:=30.0

rostopic echo /localization/sensor_quality
rostopic echo /stability/camera_fallback_status
rostopic echo /control/maneuver_status
rostopic echo /control/maneuver_cmd
```

차선 fallback 상태의 mode, lane_primary_usable, lane_good_streak,
blackout_distance_m, blackout_budget_exhausted와 통합 상태의
blackout_lane_corridor, blackout_corridor_reason을 함께 확인한다.
fallback 상태의 stamp는 판단 발행 시각이며, 차선 원본 관측 시각을 대체하지 않는다.
`lane_source_stamp`, `lane_source_age_sec`, `lane_control_timeout_sec`로 영상 지연도 확인한다.
명령과 상태가 한 주기 어긋나도 새 제동 상태가 이전 가속 명령을 제한하도록 한다.

기존 `stability_stack/morai_udp_ekf_curvature_camera_fallback.launch`는
별도의 실험 구성이다. 최종 ROI 설정/지도 보호와 동일하다고 간주하지 않는다.
독립 차선 노드에도 최신 교차로 상태 입력이 필요하며, 입력이 없으면 음영 주행은 정지한다.

## 검증 범위와 남은 한계

회귀시험은 ROS 메시지/시간/UDP 경계를 대역으로 바꾼 Python 테스트다.
중복·지연 영상, NaN, IMU 소실, 진입 대기, 차선 소실 제동, 실제 복구 대기,
예산 초과, 최종 통합의 교차로 금지와 장애물 우선 정지를 확인한다.

`stability_stack/test/test_lane_closed_loop.py`는 실제 `DetectionResult` 오차 계산과
품질 평가, fallback 콜백을 연결하고 축간거리 3m의 운동학적 차량 모델로 검증한다.
속도 2m/s, 음영 13초 조건에서 좌우 0.7m 편차와 좌우 0.07rad 방향 오차를 각각
줄이고, 좌·우 반경 60m 곡선에서 최대 중심 이탈이 0.35m 미만인지 확인한다.
이 시험은 합성 차선 경계를 입력하며 영상 신경망의 검출 성능이나 실제 차량 응답을
측정하는 시험은 아니다. 중복·정지 영상·버스트 수신·속도 소실·짧은 GPS 재소실은
`test_camera_fallback.py`, 원거리 경계 검증은 `camera_perception/test/test_lane_quality.py`에서 검사한다.

2026-09-09 보완 후 회귀시험 334개 통과: stability_stack 41개,
camera_perception 67개, stopline_control 54개, turn_signal_controller 168개,
curvature_speed_purepursuit 4개. 런치·패키지 XML과 Python 구문 검사도 통과했다.

실제 ROS 빌드·MORAI 차량 동역학·날씨별 차선 오인식·IMU drift는 여기서 검증하지 않았다.
실제 시험 전 차선 횡오차 부호, 카메라 미터 보정, 제동 응답을 저속으로 확인해야 한다.
최종 ROI 차선 모델과 레거시 Hough 차선 노드는 구분해야 한다.
신호 카메라 Cam4의 `signal_camera.calibrated=false` 기본값은 그대로이며,
검증된 보정값이 없으면 교차로 통행은 계속 차단한다.
