# 정지선 제어와 독립 Mission Evaluator

## 1. Float64 거리만 발행한다는 의미

기존 `/perception/camera/stopline_distance_m`은 `std_msgs/Float64`라서 메시지 안에
`data` 숫자 하나만 있다. 이 숫자에는 다음 정보가 없다.

- 카메라 프레임이 촬영된 시각 또는 추론 결과의 `header.stamp`
- 어떤 좌표계의 거리인지 나타내는 `frame_id`
- 검출 유효성·confidence

ROS subscriber가 받을 때의 현재 시각을 따로 기록할 수는 있지만, 그것은 카메라가
프레임을 촬영한 시각이 아니다. 카메라 UDP 지연, 추론 시간 변동, ROS queue 대기
때문에 `5.0 m`이라는 오래된 결과가 새 결과처럼 보일 수 있다. 정지선 거리와
신호 상태를 같은 프레임 기준으로 비교해야 하는 정지선 제어·통과 판정에서는
이 차이가 정지선 앞/뒤 판단을 흔들 수 있다.

그래서 기존 Float64는 모니터링/하위 호환용으로 남기고, 다음 토픽을 추가했다.

`/perception/camera/stopline` (`morai_perception_msgs/StopLineDetection`)

```text
Header header
bool valid
float64 distance_m
float64 confidence
```

신호도 기존 Bool `/perception/traffic_light/stop_required`를 유지하면서
`/perception/traffic_light/state` (`morai_perception_msgs/TrafficLight`)를
병행 발행한다. 이 메시지는 원본 객체 배열의 header 시각과 현재 프레임의
`GREEN`, `RED`, `YELLOW`, `UNKNOWN` 관측 상태를 담는다. 미검출은 UNKNOWN이며
`valid=false`이다. 정지선 제어는 이 timestamped state를 사용한다.
기존 Bool의 false는 초록불 증거로 사용하지 않는다. 현재 Bool 경로도 미검출만으로
정지를 해제하지 않으며, 새 GREEN이 원본/수신 시각 모두에서 기본 0.5초 이상
확인돼야 해제한다. 타임스탬프가 없는 Bool 소비자는 별도로 입력 소실을 감시해야 한다.
선택 옵션 `allow_legacy_stop_request=true`도 Bool true의 정지 요청만 허용하고,
false로 정지를 해제하지 않는다. 기본값은 false이다.

시각 계약:

- lane·YOLO 모두 완성된 UDP 이미지의 **로컬 수신 ROS 시각**을 기록하고,
  디코딩·프레임 대기·추론 후에도 그대로 발행한다. 발행 시각으로 덮어쓰지 않는다.
- 이는 실제 촬영 시각이 아니다. 네트워크 도착 전 지연은 포함할 수 없으며,
  MORAI sec/nsec와 ROS clock의 대응 관계가 검증되기 전에는 둘을 혼용하지 않는다.
- 제어기는 원본 ROS 시각의 나이와 subscriber 수신 후 monotonic 나이를 모두
  검사한다. 정지선·신호 기본 허용 나이는 0.8초이며, 0 시각, 오래된 결과,
  0.05초보다 먼 미래, 중복·역순 프레임을 사용하지 않는다.
- 두 카메라 결과를 정확히 같은 프레임으로 동기화하는 기능은 아니다.
  각각의 신선도를 검사하고 정지 의도를 유지하는 구조다.

정지선 거리의 frame은 `base_link`이고 BEV ego 원점 기준이다. confidence는
정지선 후보의 **횡방향 폭·픽셀 지지율·두께에 따른 기하학적 점수**다. 차선 quality와
독립적이며 통계적인 검출 확률이 아니다. 신호 confidence는 신뢰도 임계값을
통과한 호환 가능한 동일 클래스 관측의 점수이며, 서로 다른 신호가 충돌하면
`UNKNOWN`, confidence=0으로 발행한다.

## 2. 정지선 제어 흐름

현재 `final_ws_bringup.launch`는 `enable_maneuver_fusion=true`가 기본이다.
앞단 `stopline_controller.py`는 통과 전용이고, 차선 fallback 뒤의
`maneuver_fusion_node.py`가 공통 정지선 core와 지도·방향별 신호 판단을 실행한다.
`enable_maneuver_fusion=false`인 경우 앞단 정지선 노드가 직접 제어한다.
두 경로 모두 이번 보완을 적용했다.

```text
curvature_speed_purepursuit
        /control/ctrl_cmd (nominal, type 1 accel/brake)
                         |
                         v
              stopline_controller.py
       /control/stopline_cmd
                         |
              camera fallback (선택)
                         |
              maneuver_fusion (기본 활성)
        /control/maneuver_cmd
                         |
                    control_mux
                         |
                      /ctrl_cmd
```

정지선 제어기는 type 1 accel/brake 명령에 **제한**을 더한다. 기존 곡률 속도
계획·km/h PI를 교체하지 않으며 steering도 유지한다. accel은 기존 값 이하,
brake는 기존 값 이상만 허용한다. 어느 쪽에서든 brake가 있으면 accel은 0이다.
모든 제한을 적용한 뒤 가속 페달의 증가만 초당 0.5로 제한한다(20Hz에서
한 주기 최대 0.025 증가). 제동과 가속 페달 감소는 지연시키지 않는다.
따라서 정지 중 기존 PI의 가속 요청이 커져도 정지 해제와 동시에 페달 1.0을
내보내지 않는다. 이는 **페달 변화율**이지 실제 차량 가속도/jerk 보장은 아니다.

| 상황 | 처리 |
|---|---|
| 정지 요청 없고 신호/정지선 근거도 없음 | NOMINAL: 기존 주행 명령 유지 |
| 초록불이 연속 확인됐고 기존 정지 요청 없음 | NOMINAL: 기존 주행 명령 유지 |
| 빨간불·노란불, 또는 정지선은 있는데 신호 불명 | 정지 요청을 기억하고 정지선 접근 제어 |
| 정지선·속도가 유효하며 목표가 전방 | APPROACH: 남은 거리와 속도로 가속 제한·감속 |
| 검출이 잠깐 끊김 | 마지막 거리에서 odometry 속도 적분으로 이동량 차감 |
| 정지 요청 중 거리 없음·예측 만료·속도 불명 | SAFE_STOP: accel=0, brake=1 |
| 카메라 기반 정지 목표를 추적하다 예측 한도/속도 공백으로 상실 | 다른 정지선 검출만으로 해제하지 않고 정지 유지 |
| 지도에 결합된 동일 정지선 ID와 유효한 현재 위치가 다시 확인됨 | 동일 목표만 재추적 가능; 다른 ID/카메라 검출만으로 복구 불가 |
| 정지 목표의 0.03m 허용오차 안에 진입 | 속도가 아주 낮아도 제동을 풀지 않고 HOLD 유지 |
| 새 초록불이 2개 이상이며 관측 시간으로 0.3초 이상 연속 확인 + 유효한 속도 | 정지 요청·HOLD 해제; 페달 상승률 제한을 거쳐 재출발 |

미검출, 오래된 초록불, Bool false, 낮은 confidence는 정지 해제 조건이 아니다.
ROS clock 역행도 이력을 초기화하고 HOLD로 전환한다. 속도가 없을 때 먼저 받은
정지선도 정지 요청을 보존하고, 유효 시간이 남아 있으면 속도 복구 후 사용한다.
NaN/Inf 속도를 담은 새 odometry는 이전 정상 속도를 즉시 무효화한다. 정지 중
속도가 유효하지 않으면 초록불만으로 정지 해제를 허용하지 않는다. 통합 노드는
기존 지도 신호등 ID·진행 방향·방향지시등 선행 시간 조건도 계속 검사한다.

거리 계산과 제한:

- 속도는 `/localization/odometry`의 m/s를 사용한다. 로그와 목표속도는 km/h다.
  odometry도 원본 stamp와 수신 후 경과시간을 검사한다(기본 0.5초).
- 공통 core는 관측 시각 이후 **속도 이력을 사다리꼴 적분**하고
  `front_reference_offset_m`과 함께 검출 당시 거리에서 뺀다. 감속 중 마지막
  속도만 곱해 이동량을 적게 계산하는 문제를 줄인다.
- 속도 이력 시작 전 최대 0.05초만 첫 속도로 근사한다. 긴 공백이나 중간 속도
  누락을 임의로 메우지 않는다. 이력이 없으면 그 거리로 가속을 허용하지 않고
  사용 가능한 새 관측을 기다린다.
- 지도 통합 노드는 검출 시각에 가장 가까운 odometry 위치를 경로에 투영한다.
  시각 차이가 0.05초를 넘거나 map 좌표가 아니면 결합하지 않는다. 활성 교차로에서
  유효한 정지선의 시각 대응이 실패하면 `stopline_pose_unsynchronized`로 정지한다.
- 카메라 기반 마지막 유효 목표는 최대 8초 또는 12m까지만 예측한다.
  ROS 시각과 monotonic 경과시간을 각각 검사하며, 서로 다른 시계의 초를 빼지 않는다.
  제어 주기가 0.5초 넘게 비거나 속도가 끊기면 거리 예측을 폐기한다.
- 이미 추적하던 카메라 기반 정지 목표를 놓치면 `tracking_fault=true`로 유지한다.
  더 먼 정지선 한 프레임이 들어왔다고 새 목표로 바꾸거나 가속하지 않는다.
  지도 경로가 보장한 **같은 ID**의 현재 목표만 예외적으로 재추적할 수 있다.
  초록불 확인 후 정상 통과한 교차로의 이력은 다음 교차로를 막지 않는다.
- `remaining = 정지선까지의 앞바퀴 기준 거리 - hold_distance_m`으로 두고,
  계획 감속도 `a`, 반응시간 `tau`로 속도 상한을 계산한다.

```text
target_speed_mps = sqrt((a*tau)^2 + 2*a*remaining) - a*tau
braking_distance = speed*tau + speed^2/(2*a)
접근 범위 = max(20m, braking_distance + hold_distance + trigger_margin)
```

따라서 기존처럼 20m 안에서만 감속하지 않는다. 기본 계획 감속도는 1.0 m/s²,
반응시간은 0.3초다. 필요한 감속도를 `max_decel_mps2=1.5`로 나누고 0~1로
제한해 brake를 요청한다. **1.5는 차량에서 보장된 실측 감속도가 아니라
제어의 보정 파라미터다.** 속도·도로·차량에 따른 실제 pedal 응답은 별도 검증해야 한다.

ROI 통합 실행에서는 정지선 제어 활성화 시 safety adapter의 **신호 Bool 입력만**
끊어 중복 최대 제동을 막는다. LiDAR·보행자·교차로 안전정지는 계속 최우선이다.
정지선 노드 종료나 출력 중단은 뒤 단계의 nominal timeout으로 정지 처리된다.
`enable_maneuver_fusion=false`인 단독 구성에서 `enable_stopline_control=false`이면
기존 명령을 통과시키고 ROI 신호 Bool의
즉시정지 경로를 복원한다. 신호 무시 옵션이 아니다. 일반 비-ROI launch는
정지선 제어 기본값이 false여서 기존 신호 제어 경로를 보존한다.

### 2.1 실행 전 차량 기준점 보정

현재 launch는 후륜축 원점→앞범퍼를 가정한
`stopline_front_reference_offset_m=3.845`(축거 3.000 + 전방 오버행 0.845)를 사용한다.
이번 수정에서 이 차량 설정은 바꾸지 않았다. **실제 MORAI 차량과 BEV 원점이
이 가정과 같은지 검증해야 한다.** 앞바퀴 기준 평가에서는 범퍼와 앞바퀴의 차이도
구분한다. 순수 core API의 offset 기본값은 0이므로 직접 구성 시에도 설정을 전달한다.
`stopline_hold_distance_m=0.5`는 선택한 기준점이 정지선보다 0.5m 앞에서 멈추도록
하는 여유거리다. 0.03m 허용오차는 측정 거리의 제어 조건이며 실제 정지 정확도가
±3cm라는 뜻이 아니다.

최상위 launch에서 조정할 값:

- `stopline_front_reference_offset_m`: 현재 3.845m, 실제 차량/원점 확인 필요
- `stopline_hold_distance_m`: 현재 0.5m
- `stopline_stop_tolerance_m`: 기본 0.03m, 목표 근처 HOLD 진입 허용오차
- `stopline_accel_rise_rate_per_sec`: 기본 0.5/s, 페달 증가 제한
- `stopline_planning_decel_mps2`: 기본 1.0m/s², 계획용
- `stopline_max_decel_mps2`: 기본 1.5m/s², brake 정규화용

### 2.2 재빌드와 검증

ROS Noetic Docker 안에서 workspace 루트로 이동해 Python 패키지와 메시지 빌드를
갱신한다. 이미 ROS 의존성을 설치한 환경 기준이다.

```bash
cd /root/morai_ws  # 자신의 Docker workspace 경로로 변경
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -B -m unittest discover -s src/control/stopline_control/test -v
python3 -B -m unittest discover -s src/control/turn_signal_controller/test -v
python3 -B -m unittest discover -s src/detection/camera_perception/test -v
roslaunch morai_bringup final_ws_bringup.launch workspace_path:="$PWD" enable_control:=false
```

기본 통합 모드의 상태·명령은 `/control/maneuver_status`, `/control/maneuver_cmd`다.
`stopline_tracking_fault`, `accel_rise_limited`, `reason`을 함께 확인한다.
단독 모드의 상태·명령은 `/control/stopline_status`, `/control/stopline_cmd`다.
공통 관측 토픽은 `/ctrl_cmd`,
`/perception/camera/stopline`, `/perception/traffic_light/state`다. 실제 차량 명령을
허용하기 전에 stamp 나이, frame, 신호 색상, 측정 속도, 앞바퀴 기준 거리를 확인한다.
그다음 저속에서 RED 접근→근거리 검출 소실→정지 유지→GREEN 재출발을 시험하고,
신호/odom 지연·소실, 연속 교차로, 보행자·장애물 긴급정지도 확인한다.

자동 테스트는 순수 제어 로직, 이상적인 가감속 모델, ROS 대역을 이용한 실제
callback 연결, 카메라 timestamp 전달을 검사한다. **실제 ROS 통신·MORAI 차량
정지거리 검증을 대체하지 않는다.** 현재 정지선 인식 범위보다 제동거리가 길면
요청 brake가 1이어도 정지선을 지키지 못할 수 있다. 장거리 정지선/지도 정보를
확보하고 검증된 접근 속도를 정해야 한다. 기본 통합 모드의 기존 지도-신호등 ID·
회전 방향 결합은 유지했다. 카메라 장착값과 지도 정지선 위치 검증은 필요하다.
비통합 모드의 일반 색상 신호를 경로별 신호 결합과 동등하게 해석하면 안 된다.

### 2.3 신호·속도 이상 후 재출발 보호 (2026-09-09)

신호가 허용 상태였더라도 속도 입력이 없거나 유효하지 않으면 가속을 차단하고
정지 명령을 유지한다. 속도 복구만으로 정지 대기를 해제하지 않으며, 복구 후 새로운
시각의 유효 GREEN 관측 2개 이상이 기본 0.3초 확인 구간을 채워야 한다.
통합 모드에서는 기존 지도 신호 대응·진행 방향·연속 인식·방향지시등 조건도 필요하다.

기본 0.5초를 넘는 제어 공백이나 유효하지 않은 제어 시각도 이전 신호 확인 이력을
취소한다. 추적 중이던 정지 목표는 무작정 더 먼 검출로 교체하지 않는다. 같은 ID의
지도 목표가 새로 확인된 경우의 보수적 접근은 기존대로 허용하지만, 정지선 통과
허가에는 새 신호 확인이 필요하다.

같은 시각의 GREEN과 RED/UNKNOWN처럼 관측이 모순되어도 허가를 취소한다.
동일 내용의 중복 관측은 확인 횟수나 유효 시간을 연장하지 않는다. ROS 시각 역행은
기존 reset 절차로 처리한다. 이 동작은 통합 노드뿐 아니라 단독 정지선 core에도 적용된다.
순서 처리와 늦은 pose 재평가는 [통합 제어 문서](MANEUVER_SENSOR_FUSION.md)를 참고한다.

`brake=1.0`은 최대 정규화 제동 **요청**이며 실제 감속도/정지 성공을 보증하지 않는다.
새 테스트는 `test_stopline_regressions.py`, `test_signal_temporal_safety.py`에서
속도 복구, 제어 공백, 중복·모순 신호, RED→LEFT, 늦은 pose와 관측 만료를 확인한다.
보행자 Bool 원본 시각과 LiDAR 끼어들기 간격 원본 시각의 미해결 문제는 이번
정지선 수정 범위에 포함하지 않았으며 [미해결 사항](KNOWN_ISSUES_FINAL_WS.md)에 분리했다.

### 2.4 정지선 후보 분리와 신호 충돌 처리 (2026-09-09)

`camera_perception/lane/stopline_geometry.py`가 세그멘테이션 BEV의 정지선
클래스를 거리별 후보로 나눈다. 8m와 24m에 정지선이 함께 보이면 둘의 중간값이
아닌 8m 후보를 선택한다. 각 후보는 차량 중앙 양쪽 지지, 횡방향 폭, 중앙의
큰 빈틈, 두께, 기울기를 검사한다. 옆 도로의 서로 떨어진 조각이나 세로로 긴
오검출을 하나의 정지선으로 합치지 않는다. 기울어진 정지선은 `x = a*y + b`를
적합해 차량 중앙 `y=0`에서의 거리 `b`를 사용한다.

기본 기하 조건은 좌우 2m 관심영역, 최소 폭 1.5m, 중앙 양쪽 0.4m 이상 지지,
최대 횡방향 빈틈 0.3m, 최대 두께 0.7m, 횡방향 대비 최대 각도 35도다.
인접 거리 행의 빈틈은 0.15m까지 허용한다. 폭은 두께의 3배 이상이어야 한다.
실제 BEV 보정·정지선 폭·카메라 영상을 이용해 조정할 초기값이며, 합성 마스크
시험만으로 실제 검출률을 확인한 것은 아니다. 곡선 경로 전체를 투영한 ROI나
새 신경망 학습을 추가한 것은 아니다.

`live_overlay.py`는 선택한 후보의 기하학적 점수를 `StopLineDetection.confidence`로
전달한다. 원본 프레임 시각과 `base_link` 거리 기준은 유지한다. 단일 프레임의
거리 후보에 시간 평균을 적용하지 않으며, 접근 중 검출 소실은 기존 제어 core의
속도 적분·예측 한도·정지 유지로 처리한다.

일반 신호 토픽 `/perception/traffic_light/state`와 Bool 경로는 이제 다음 기준을 쓴다.

- RED와 GREEN 등 서로 다른 신호등의 클래스가 충돌하면 UNKNOWN으로 처리한다.
- GREEN 계열 문자열 포함 여부만으로 허가하지 않고 지원하는 클래스를 검사한다.
- 기본 confidence 0.5 미만, NaN/Inf, 범위를 벗어난 점수는 허가 근거에서 제외한다.
- 일반 토픽은 직진 기준이다. LEFT/RIGHT/RED_LEFT/RED_RIGHT는 직진 정지 상태이며,
  방향별 원본 클래스는 `/perception/traffic_light/directional_state`로 별도 보존한다.
  좌·우회전 주행은 기본 지도 결합 `maneuver_fusion`의 방향별 허가를 사용한다.
- Bool 정지 해제에는 서로 다른 GREEN 프레임 2개 이상과 기본 0.5초의 원본 시각 및
  수신 시각 확인 구간이 필요하다. 미검출·충돌·중복·역순·오래된 프레임은 해제하지 않는다.
  노드 종료 시에는 Bool 정지를 요청한다. Bool 소비자의 입력 timeout은 계속 필요하다.
- 단독 정지선 core도 최초 정지선 획득 시 단 한 번의 GREEN으로 통과시키지 않는다.
  기본 0.3초 이상 새 GREEN 2개 확인 조건을 최초 접근에도 적용한다.

카메라 패키지 launch의 조정값은 `traffic_light_min_confidence`,
`traffic_light_input_timeout_sec`, `traffic_light_clear_confirmation_s`다.
기본 통합 모드의 지도 신호 ID 결합, 방향별 허가, 방향지시등 조건은 그대로 적용된다.

회귀 시험은 `test_stopline_geometry.py`, `test_traffic_signal.py`,
`test_camera_timestamps.py`, `test_initial_green.py`를 포함한다. 합성 마스크에서
거리 선택→신호 충돌→검출 소실→정지 유지→GREEN 재출발을 이상적인 차량 모델로
연결해 검사한다. 실제 ROS 빌드·모델 추론·MORAI 정지거리 검증은 별도로 필요하다.

## 3. 독립 Mission Evaluator

패키지: `mission_evaluator`

실행 예:

```bash
roslaunch mission_evaluator mission_evaluator.launch route_length_m:=1000.0
```

이 launch는 `final_ws_bringup.launch`에 include되어 있지 않다. 따라서 evaluator는
제어 명령을 발행하지 않고, 실제 운영 코드에 섞이지 않는다.

입력:

| 토픽 | 용도 |
|---|---|
| `/Ego_topic` | 실제 Ego 속도(km/h 계산용) |
| `/experimental/curvature_progress` | 경로 진행거리(m) |
| `/localization/sensor_quality` | GPS blackout 상태 |
| `/mission/lane_contact` | 바퀴 1개 기준 차선 접촉 판정 결과 Bool |
| `/CollisionData` | 충돌 객체 식별 및 재충돌 분리 |

출력:

| 토픽 | 내용 |
|---|---|
| `/mission/status` | 현재 규칙 상태 JSON |
| `/mission/penalty_event` | 발생한 패널티 이벤트 JSON |
| `/mission/summary` | 누적 패널티와 최근 이벤트 |
| `/mission/current_region` | highway 예외/blackout 및 차선 면제 상태 |

규칙은 `config/mission_rules.yaml`에서 설정한다. 경로 5% 판정은 `route_length_m`을
직접 줄 수도 있고, 기본값 `0`으로 두면 launch의 `path_file`에서 x/y 누적 길이를
자동 계산한다. 실제 경로 파일을 사용하지 않는 경우에는 `route_length_m`을
명시한다.
고주로 예외구간과 GPS blackout 구간은 progress metre 구간으로 설정한다.

현재 저장소의 인식 결과에는 “바퀴 1개가 실선/중앙선에 접촉했는지”를 직접
나타내는 메시지가 없으므로 evaluator는 이를 임의로 추정하지 않는다. 별도
footprint-line 교차 판정기가 `/mission/lane_contact`를 발행하면 3초 지속 및
3초 단위 반복 패널티를 평가한다. blackout 지역이거나 센서 품질이
`GPS_BLACKOUT`이면 차선 패널티 타이머를 면제한다.
