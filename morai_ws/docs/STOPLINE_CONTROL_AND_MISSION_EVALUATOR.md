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
기존 Bool의 false(미검출에 의한 해제 포함)는 초록불 증거로 사용하지 않는다.
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

정지선 거리의 frame은 `base_link`이고 BEV ego 원점 기준이다. 별도 정지선
확률 모델은 없어서 confidence는 **검출 유효성의 1/0 표시**다. 차선 quality와
독립적이며, 1.0을 통계적인 100% 정확도로 해석하면 안 된다. 신호 confidence는
선택한 색상에 해당하는 객체의 점수만 사용한다.

## 2. 정지선 제어 흐름

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
                    control_mux
                         |
                      /ctrl_cmd
```

정지선 제어기는 type 1 accel/brake 명령에 **제한**을 더한다. 기존 곡률 속도
계획·km/h PI를 교체하지 않으며 steering도 유지한다. accel은 기존 값 이하,
brake는 기존 값 이상만 허용한다. 어느 쪽에서든 brake가 있으면 accel은 0이다.

| 상황 | 처리 |
|---|---|
| 정지 요청 없고 신호/정지선 근거도 없음 | NOMINAL: 기존 주행 명령 유지 |
| 초록불이 유효하고 기존 정지 요청 없음 | NOMINAL: 기존 주행 명령 유지 |
| 빨간불·노란불, 또는 정지선은 있는데 신호 불명 | 정지 요청을 기억하고 정지선 접근 제어 |
| 정지선·속도가 유효하며 목표가 전방 | APPROACH: 남은 거리와 속도로 가속 제한·감속 |
| 검출이 잠깐 끊김 | 마지막 거리에서 odometry 속도 적분으로 이동량 차감 |
| 정지 요청 중 거리 없음·예측 만료·속도 불명 | SAFE_STOP: accel=0, brake=1 |
| 정지 목표 도달 | HOLD: accel=0, brake=1 유지 |
| 새 초록불이 2개 이상이며 관측 시간으로 0.3초 이상 연속 확인 | 정지 요청·HOLD 해제; 기존 PI 명령에 따라 재출발 |

미검출, 오래된 초록불, Bool false, 낮은 confidence는 정지 해제 조건이 아니다.
ROS clock 역행도 이력을 초기화하고 HOLD로 전환한다. 속도가 없을 때 먼저 받은
정지선도 정지 요청을 보존하고, 유효 시간이 남아 있으면 속도 복구 후 사용한다.

거리 계산과 제한:

- 속도는 `/localization/odometry`의 m/s를 사용한다. 로그와 목표속도는 km/h다.
  odometry도 원본 stamp와 수신 후 경과시간을 검사한다(기본 0.5초).
- 검출 당시 거리에서 `front_reference_offset_m`과 추론 지연 동안의 추정
  이동거리(`현재 속도 × 관측 나이`)를 뺀다. 지연 보상은 근사치다.
- 마지막 유효 검출 이후 최대 8초 또는 12m까지만 속도 적분을 허용한다.
  제어 주기가 0.5초 넘게 비거나 속도가 끊기면 거리 예측을 폐기한다.
- 활성 정지 목표를 더 멀리 옮기지 않는다. 초록불에서 통과한 정지선과
  만료된 이력은 다음 교차로의 새 정지선 수용을 막지 않는다.
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
`enable_stopline_control=false`이면 기존 명령을 통과시키고 ROI 신호 Bool의
즉시정지 경로를 복원한다. 신호 무시 옵션이 아니다. 일반 비-ROI launch는
정지선 제어 기본값이 false여서 기존 신호 제어 경로를 보존한다.

### 2.1 실행 전 차량 기준점 보정

`stopline_front_reference_offset_m`은 BEV ego 원점에서 차량 앞바퀴 판정 기준까지의
전방 거리다. 기본 0.0은 미보정 값이며 **그대로 앞바퀴 정지선 준수를 보장하지 않는다.**
MORAI 차량 형상과 BEV 원점을 확인해 실측값을 넣는다. 범퍼 기준을 선택한다면
그에 맞게 측정한다. `stopline_hold_distance_m=0.7`은 그 기준점이 정지선보다
0.7m 앞에서 멈추도록 하는 여유거리다.

최상위 launch에서 조정할 값:

- `stopline_front_reference_offset_m`: 차량에서 측정한 기준점 offset
- `stopline_hold_distance_m`: 기본 0.7m
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
python3 -B -m unittest discover -s src/detection/camera_perception/test -v
roslaunch morai_bringup final_ws_bringup.launch workspace_path:="$PWD" enable_control:=false
```

관측 토픽은 `/control/stopline_status`, `/control/stopline_cmd`, `/ctrl_cmd`,
`/perception/camera/stopline`, `/perception/traffic_light/state`다. 실제 차량 명령을
허용하기 전에 stamp 나이, frame, 신호 색상, 측정 속도, 앞바퀴 기준 거리를 확인한다.
그다음 저속에서 RED 접근→근거리 검출 소실→정지 유지→GREEN 재출발을 시험하고,
신호/odom 지연·소실, 연속 교차로, 보행자·장애물 긴급정지도 확인한다.

자동 테스트는 순수 제어 로직, 이상적인 가감속 모델, ROS 대역을 이용한 실제
callback 연결, 카메라 timestamp 전달을 검사한다. **실제 ROS 통신·MORAI 차량
정지거리 검증을 대체하지 않는다.** 현재 정지선 인식 범위보다 제동거리가 길면
요청 brake가 1이어도 정지선을 지키지 못할 수 있다. 장거리 정지선/지도 정보를
확보하고 검증된 접근 속도를 정해야 한다. 신호등과 내 경로·회전 방향의 연결도
별도 과제이며, 이번 수정은 기존 카메라 팀의 색상 우선순위 정책을 유지한다.

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
