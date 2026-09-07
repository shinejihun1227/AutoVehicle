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
병행 발행한다. 이 메시지는 원본 객체 배열의 header 시각과 `GREEN`, `RED_STOP`,
`YELLOW_STOP` 같은 판단 상태를 담는다. 정지선 제어기는 이 timestamped 신호를
우선 사용하고, state 토픽이 없는 구형 구성에서만 Bool을 fallback으로 사용한다.

현재 카메라 pipeline은 별도 정지선 확률을 제공하지 않으므로 valid 검출 시 lane
quality confidence를 보수적인 confidence로 넣는다. 추후 모델 confidence가
생기면 이 필드를 교체한다.

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

정지선 제어기는 다음 조건이 모두 맞을 때만 nominal 명령의 종방향 값을 바꾼다.

1. `/perception/traffic_light/stop_required`가 최신이고 `true`
2. timestamp가 있는 StopLineDetection이 최신이고 valid
3. 정지선이 접근 거리 안에 있음

차량 속도는 `/localization/odometry`의 m/s를 사용한다. 남은 정지선 거리로
허용속도 `sqrt(2 * max_decel * remaining_distance)`를 구하고, 현재 속도가 그보다
크면 brake를 0~1로 올린다. 정지선에 도달하면 `accel=0, brake=1`로 유지한다.
steering은 nominal 값을 그대로 통과시킨다. 기본 최대 감속도는 1.5 m/s²,
정지선 유지거리는 0.5 m이다.

`final_ws_bringup.launch`에서는 `enable_stopline_control=true`가 기본이며,
검증 시 false로 설정하면 `/control/ctrl_cmd`를 그대로 통과시킨다.

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
