# v1.1 대회 규정 기반 연습용 랩타임·미션 평가

확인한 기준: [규정집 v1.1, 2026-09-08 수정](https://morai.atlassian.net/wiki/external/YTRiNWMwZTNjODc3NDBkYzgxMTMzOWMwMWY3YWRkZDc),
[공식 체크포인트 좌표](https://morai.atlassian.net/wiki/external/NjQzYWJlMjA5YzRkNDM4MDlhMmUzMGZjZWJjN2E2OTY).
본문 최종 수정 시각은 `2026-09-08T01:32:53.649Z`로 확인했다.

이 기능은 **연습용 추정 계측기**이며 주최측 Unity 판정 프로그램을 대체하지 않는다.
규정의 최종 성공/실패·실격·복귀 판단은 주최측 프로그램과 운영요원이 한다.
네트워크 제한 때문에 주최측만 볼 수 있는 판정 정보가 있으므로, 입력이 없거나
오래된 항목을 `0점 감점 / 위반 없음`으로 확정하지 않는다.

## 1. 만들어진 launch

| Launch | 기능 |
|---|---|
| `roslaunch mission_evaluator competition_practice.launch` | 기존 final_ws 주행 스택 + 연습용 평가 + 선택적 충돌 UDP 수신/rosbag |
| `roslaunch mission_evaluator competition_lap.launch` | 이미 실행 중인 주행/rosbag을 관찰하는 평가기만 실행 |

기존 `final_ws_bringup.launch`, 주행 제어 코드, 구형 `mission_evaluator.launch`는 변경하지 않았다.
구형 평가기와 새 평가기를 동시에 띄우면 서로 다른 결과가 혼동될 수 있으므로 새 계측에는 위 launch만 쓴다.
계측기는 `/ctrl_cmd`를 발행하지 않고 control_mux에도 개입하지 않는다.
**평가 실패·완주·abort·reset은 차량 정지 명령이 아니다.**
`enable_control=false`도 실제 차량/시뮬레이터의 비상정지를 대신하지 않는다.

## 2. 반영한 규정과 판정 범위

| 항목 | v1.1 기준 | 연습용 구현 |
|---|---|---|
| 출발 | 출발 후 60초 이내 전체 경로의 5% 통과 | 시작점 근처에서 움직임 감지 후 계측. 수동 start 서비스도 제공 |
| 완주 | 지정 미션/체크포인트를 순서대로 수행 | 15개 체크포인트를 반경 3m 안에서 순서대로 통과 + 마지막 START/END 도달 |
| 제한 시간 | 실제 주행 시간 15분 | 900초 초과 시 `FAILED`. 패널티를 합한 시간이 아닌 원래 경과 시간 기준 |
| 속도 | 일반 구간 60km/h | 초과 첫 관측 +15초, 초과 지속 3초마다 +15초 |
| 속도 예외 | A2256W000411 시작 ~ A2256W000153 끝 | MGeo 링크 전체 방향/경로 대응을 검사해 진행 거리 구간 계산 |
| 차선 | 바퀴 하나라도 실선/중앙선 접촉 3초마다 +5초 | 기본: 지도 선분과 추정 바퀴 위치 접촉. 정확한 외부 판정 입력으로 교체 가능 |
| 충돌/끼어들기 | 객체 충돌마다 +15초, 지속 접촉 중복 없음, 재충돌 추가 | 실제 CollisionData 스냅샷의 객체 종류+ID로 접촉/해제 추적 |
| 신호 | 앞바퀴의 정지선 통과 시각 신호 위반 +15초 | 앞차축 기준 지도 정지선 통과 + 해당 교차로에 연관된 카메라 인식 상태로 **추정** |
| GPS 음영 | 해당 구역의 차로 감점 제외 | 검증된 진행 거리 구간이 있으면 사용, 없으면 GPS blackout 상태를 **추정 근거**로 사용 |
| 복귀 | 운영요원·팀 승인 후 이전 체크포인트 복귀 +15초 | 사용자가 시뮬레이터를 복귀시킨 뒤 승인 서비스 호출. 자동 teleport 없음 |
| 랜덤 미션 | 해당 장애물 회피·끼어들기 규정 적용 | 충돌 및 경로/체크포인트 판정에 포함. 별도의 임의 패널티 없음 |
| 시간·날씨 | 변화 대응, 독립 감점 없음 | 감점 항목을 임의로 추가하지 않음 |
| 순위 | 완주 우선, 실제+패널티 시간. 미완주는 체크포인트 달성도 우선 | 같은 계측 프로세스의 시도별 best result 제공 |

방향지시등 5초 선행은 기존 제어 로직의 요구사항이며, 확인한 v1.1 채점표에 별도의
방향지시등 패널티가 없으므로 임의의 감점 시간을 만들지 않았다.
PC 제출 지연 등 운영상 감점도 구체적인 값이 미공개여서 자동 부과하지 않는다.

### 현재 경로에 대응한 수치

- 경로 XY 길이: **2,184.611723m**.
- 출발 미션 5% 지점: **109.230586m**.
- 속도 예외: **1,118.741751m 이상 ~ 1,741.720989m 미만**.
- 위 범위는 체크포인트 10의 위치에서 13의 위치까지와 일치한다.
- 첫 점/마지막 점은 동일하다. 단순한 도착점 거리만으로 완주 처리하지 않는다.
- 체크포인트의 yaw는 참고용으로 보존한다. 공개 좌표 페이지에는 각도 허용 오차가 없어서 임의의 yaw 실패 기준은 적용하지 않는다.
- 반경은 XY 거리로 계산한다. 운영측의 3D 거리/차량 기준점 처리와는 대조가 필요하다.

설정은 `mission_evaluator/config/competition_rules_v1_1.json`에 있다.
`practice` 아래의 시작 감지 속도·허용 지연·경로 이탈 임계값·바퀴 치수는 **연습 계측 설정**이지 공식 수치가 아니다.
기본 60km/h를 지도 내부의 다른 `max_speed` 필드로 대체하지 않는다.
고속 예외 구간에 대한 별도 속도 상한도 채점표에 없으므로 점수를 위해 임의로 만들지 않는다.
속도 패킷이 위치보다 먼저 도착해 예외 구간 경계의 어느 쪽인지 확정할 수 없는 경우,
최근 속도×위치 지연과 `speed_region_pose_error_m`의 범위 안에서는 임의의 +15초 대신
경계 시간 정렬 `unassessed`를 기록한다. 그래서 공식 판정기와 경계부 기록이 다를 수 있다.

## 3. 첫 실행: 차량 제어 없이 계측 확인

아래는 ROS Noetic Docker/Ubuntu 환경에서 실행한다. Windows PowerShell 명령이 아니다.
Docker 준비 자체는 `DOCKER_FINAL_WS_FROM_SCRATCH_KO.md`를 따른다.

```bash
cd ~/morai_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

python3 src/mission/mission_evaluator/scripts/inspect_competition_course.py \
  --path data/routes/2026_molit_comp_global_path.txt \
  --mgeo data/mgeo/R_KR_PR_K-city_2025

roslaunch mission_evaluator competition_practice.launch \
  workspace_path:=$HOME/morai_ws \
  morai_host_ip:=192.168.0.151 \
  enable_control:=false \
  collision_udp_enabled:=true \
  collision_port:=9092
```

관찰 터미널:

```bash
source ~/morai_ws/devel/setup.bash
rostopic echo /evaluation/status
rostopic echo /evaluation/event
rostopic echo /evaluation/result
```

실제 작업공간이 `/workspace/morai_ws`라면 `workspace_path`를 그 경로로 바꾼다.
MORAI → Ubuntu 목적지 CollisionData UDP 포트를 `9092`로 설정한다.
이 번호는 저장된 NetworkModule 예제의 기본값이며 대회가 고정한 번호가 아니다.
Docker에서는 기존 문서의 host network/LAN 설정을 맞춘다.
이미 같은 포트를 받는 프로그램이 있으면 **두 수신기를 띄우지 않는다**.

이미 주행 스택이 떠 있으면 다음처럼 평가기만 실행한다.

```bash
roslaunch mission_evaluator competition_lap.launch \
  workspace_path:=$HOME/morai_ws \
  collision_udp_enabled:=true
```

이 경우 상태 UDP 909, GPS/IMU/카메라/LiDAR 수신기는 중복 실행하지 않는다.

## 4. 타이머와 결과 해석

기본은 시작점 반경 3m 안에서 새 Ego 속도 `abs(vx) >= 0.5km/h`를 관찰하면 출발한다.
속도는 이 저장소의 `/Ego_topic.velocity.x` 단위인 m/s에 3.6을 곱한다.
v1.1 Competition Vehicle Status에서 제공하지 않는 `velocity.y/z`나
`position.x/y/z`는 사용하지 않는다. 위치는 `/localization/odometry`의 map 좌표를 사용한다.
외부 브리지가 vx를 km/h로 발행한다면 먼저 내부 계약(m/s)으로 변환해야 한다.

주최측의 판정 프로그램 시작 버튼 시각을 우리 노드가 수신하는 것은 아니다.
그 시각에 맞추고 싶으면 `auto_start:=false`로 실행하고 시작점에서 다음을 호출한다.

```bash
rosservice call /evaluation/start '{}'
```

`lap_time_sec`는 출발부터 모든 체크포인트 및 마지막 START/END의 3m 영역 도달까지,
`penalty_time_sec`는 누적 패널티,
`adjusted_time_sec`는 두 값을 합한 기록이다.
완주 후에는 시간이 계속 늘거나 새로운 패널티가 추가되지 않는다.
도착 검출은 odometry 샘플 시각 기준이므로 샘플 주기만큼 시간 오차가 있을 수 있다.

상태 흐름:

```text
WAITING → RUNNING → FINISHED
              ├→ FAILED        (60초 출발 미션 / 900초 제한 시간)
              ├→ NEEDS_REVIEW   (체크포인트 누락 / 위치 점프 / 경로 이탈 추정)
              └→ INVALID       (ROS 시간 역행)
```

`NEEDS_REVIEW`에서는 랩타임은 계속 흐르지만 체크포인트/완주는 확정하지 않는다.
공식 실패인지 승인 후 복귀 대상인지는 운영 판단이 필요하다.
시뮬레이터에서 마지막 통과 체크포인트로 복귀한 뒤:

```bash
rosservice call /evaluation/approve_recovery '{}'
```

현재 위치가 해당 체크포인트 반경 3m 안이고 새 위치 정보가 있어야 승인된다.
승인 시 +15초이며 기존 타이머는 초기화하지 않는다. 계측기는 차량을 이동시키지 않는다.

새 시도:

```bash
rosservice call /evaluation/reset '{}'
```

이는 **계측기만** 초기화한다. 기존 경로 추종기는 경로 진행도를 단조 증가시키므로,
두 번째 실제 주행은 시나리오를 초기화하고 **주행 launch도 재시작**해야 한다.
MORAI 시간을 되돌렸다면 충돌 UDP 수신기도 재시작한다(이전 패킷 순서 watermark 유지).
같은 평가 프로세스에서 reset으로 진행한 기록들은 `/evaluation/best_attempt`에 비교된다.
시도 수는 제한하지 않는다. 공식 대회는 2회이며, 연습은 반복 가능하게 했다.
평가 프로세스를 다시 시작하면 best 목록은 새로 시작하지만 이전 파일은 보존된다.

## 5. 정확한 판정을 위해 필요한 연결/보정

### 충돌

수신기는 저장소 `src/common/morai_network/lib/define/CollisionData.py`의
packed little-endian 181byte / timestamp + 최대 5객체 형식을 구현한다.
`#CollisionData$`, data length 148, CRLF를 검사하며 손상·중복·역순 패킷은 채점하지 않는다.
객체 종류와 ID를 묶어 식별하며, 전체 0으로 채워진 슬롯을 빈 슬롯으로 해석한다.
실제 경쟁용 빌드가 같은 헤더·레이아웃·빈 슬롯 규약인지 **패킷 캡처로 먼저 검증**해야 한다.
확인 전 기본값 `collision_protocol_verified=false`로 결과에 검증 미완료가 표시된다.

지속 접촉은 한 번만 가산하고, 새 빈/해제 스냅샷을 받은 뒤 다시 접촉하면 추가한다.
통신 단절을 해제로 간주하지 않으므로, 패킷 누락 사이에 발생한 재충돌은 확정할 수 없다.
충돌 시에만 이벤트를 보내고 해제/빈 스냅샷을 보내지 않는 빌드라면 별도 어댑터가 필요하다.
5개 슬롯이 모두 차면 추가 객체 누락 가능성을 기록한다.

### 차선과 GPS 음영

기본 `lane_mode=map_estimate`는 지도 선분과 앞/뒤 바퀴 중심을 비교한다.
기본 윤거 1.6m·타이어 반폭 0.12m는 **측정되지 않은 연습값**이다.
현재 pose가 뒤차축 중심인지, 실제 윤거·타이어 접촉 폭·지도 선 위치가 맞는지 보정해야 한다.
정지선/횡단보도는 차선 접촉 대상으로 쓰지 않는다.
현재 NGII 분류 501/503/505/506 중 실선/중앙선만 사용하고,
어느 쪽이 허용되는지 모호한 solid/broken 혼합 선은 건너뛴다.
이 누락과 미보정 상태는 결과의 `unassessed_reasons`에 남는다.
차량 네 바퀴가 실제 주행 영역 밖인지까지 Unity collider처럼 정확히 재현하는 기능은 아니다.
8m 경로 이탈 검사는 **연습용 경고 임계값**이지 공식 도로 경계가 아니다.

공식 blackout 좌표는 확인한 텍스트에 없고, 저장된 샘플 시나리오의 noise zone은
활성화 상태가 아니어서 공식 면제 구역으로 하드코딩하지 않았다.
실측/공식 구간을 알면 `blackout_regions`에 다음 형식으로 넣는다.

```json
[{"start_s_m": 100.0, "end_s_m": 120.0}]
```

위 숫자는 형식 예시이며 실제 구간이 아니다.
빈 설정에서는 최신 SensorQuality blackout으로 대체하지만 결과를 추정으로 명시한다.
GPS 데이터 누락이 항상 공식 음영 구역 진입이라는 뜻은 아니다.

정확한 바퀴 접촉 판정기를 별도로 연결할 경우 `lane_mode:=external`로 하고
`/evaluation/lane_snapshot`에 `std_msgs/String` JSON을 **소스 시각을 보존해 주기적으로** 발행한다.

```json
{"timestamp_sec": 123.45, "valid": true, "contact": true, "blackout_exempt": false}
```

오래된 Bool을 타이머에서 새 timestamp로 다시 찍어 보내면 안 된다.
차선 중심 offset만으로 실선·중앙선 접촉 여부를 확정해서도 안 된다.

### 신호

도로 굴곡만으로 교차로를 만들지 않고 기존 MGeo의 신호/정지선 연관 정보를 쓴다.
진행 거리와 앞차축 위치가 해당 정지선 평면을 정방향으로 통과할 때,
`/control/maneuver_status`의 교차로 ID·선택된 신호 ID·상태와
새 `/perception/traffic_light/directional_state`를 대조한다.
대상 신호가 불명확하거나 오래됐으면 +0초로 성공 판정하지 않고 `signal_crossing_unassessed`로 기록한다.
현재 방식은 카메라 인식과 제어 노드 수신 시각에 의존하며, 공식 시뮬레이터 신호 위상과
앞바퀴 collider 통과 시각이 아니므로 **추정 신호 채점**이다.
기존 `signal_camera.calibrated=false`를 사실 확인 없이 true로 바꾸지 않는다.

### 네트워크 범위

새 시뮬레이터 연결은 허용된 **CollisionData UDP 수신**뿐이다.
주행 위치는 기존 GPS/IMU localization, 속도는 Competition Vehicle Status의 vx를 사용한다.
금지된 ObjectInfo, 정답 Ego 위치, 신호등 ground-truth 네트워크를 추가하지 않았다.
내부 ROS 토픽은 허용 UDP를 참가팀 코드에서 변환한 정보다.
기존 상태 브리지의 181byte 레이아웃이 대회 빌드의 Competition Vehicle Status와
실제로 호환되는지 별도 확인은 여전히 필요하다. 이 작업은 기존 상태 브리지를 바꾸지 않았다.

## 6. 실제 주행 전 조건

`KNOWN_ISSUES_FINAL_WS.md`의 미해결 안전 이슈, Cam4 신호 카메라 보정,
센서 시각/좌표 정렬 검증을 끝낸 뒤 시뮬레이터의 안전한 연습 환경에서만 제어를 켠다.
이번 변경은 계측 기능이며 기존 주행 안전 문제를 해결한 것은 아니다.

조건을 충족했을 때 실행 형태:

```bash
roslaunch mission_evaluator competition_practice.launch \
  workspace_path:=$HOME/morai_ws \
  morai_host_ip:=192.168.0.151 \
  enable_control:=true \
  collision_udp_enabled:=true \
  record_bag:=true
```

기존 안전 기본값 `max_speed_kph=7.2`, `fallback_speed_cap_kph=7.2`는 올리지 않았다.
이 속도에서는 정차 없이 달려도 약 1,092초가 필요해 **15분 규정 안에 완주할 수 없다**.
실제 시험에서 제어기·제동거리·곡률별 안전성을 검증하면서 속도를 조정해야 한다.
`max_speed_kph`는 제어기 내부 목표 상한이며, 계측기의 공식 60km/h 판정 기준과 별개다.
기존 Type 1(accel/brake) 출력 방식을 바꾸지 않는다.
고속 구간 계측 면제가 그 구간에서 무조건 속도를 올리라는 제어 명령은 아니다.

## 7. 저장 결과와 rosbag 재현

기본 저장 위치: `~/.ros/competition_runs/<시각_UUID>/`

- `metadata.json`: 규정 사본·경로 SHA256·구간 대응·계측 시계.
- `events.jsonl`: 각 미션/패널티/평가 불가 이벤트를 즉시 기록.
- `events.csv`: 종료 시 정리한 이벤트 표.
- `result.json`: 종료 상태·원래 랩타임·패널티·보정 기록·이벤트 전체.

시도마다 새 폴더를 만들어 이전 결과를 덮어쓰지 않는다.
`official_score=false`, `assessment=PRACTICE_ESTIMATE`는 정상적인 표시다.
`coverage_complete=false` 또는 `unassessed_reasons`가 있으면, 보정 기록은
**관찰된 패널티만 더한 값**이며 실제 공식 기록보다 짧을 수 있다.

ROS 시간 정책:

- `/use_sim_time=false`: 실제 경과 시간. MORAI 일시정지만 해도 타이머는 계속 간다.
- `/use_sim_time=true`: 외부 `/clock` 기준. `/clock`이 없으면 진행하지 않는다.
- 계측 중 ROS 시간이 역행하면 `INVALID`. 조용히 새 랩으로 바꾸지 않는다.
- 별도 신호를 모르는 상태에서 차량 정지를 일시정지로 간주하지 않는다. 신호대기는 랩타임에 포함된다.

녹화한 입력을 재생할 때는 라이브 주행/UDP 수신기를 모두 끄고, 새 roscore에서:

```bash
rosparam set /use_sim_time true
roslaunch mission_evaluator competition_lap.launch collision_udp_enabled:=false
```

다른 터미널에서 필요한 **입력 토픽만** 재생한다(녹화된 `/evaluation/status/result` 중복 발행 방지).

```bash
rosbag play --clock /absolute/path/practice.bag --topics \
  /localization/odometry /Ego_topic /localization/sensor_quality \
  /evaluation/collision_snapshot /evaluation/lane_snapshot \
  /control/maneuver_status /perception/traffic_light/directional_state
```

처음 출발 이전부터 입력이 기록된 bag을 사용한다. 중간 지점에서 시작하는 bag을
완주 랩으로 간주하지 않는다. Ctrl+C/정상 종료 시 중단 결과를 저장한다.

## 8. 검증 범위

ROS 독립 단위 테스트와 실제 제공 경로 좌표를 순서대로 공급하는 모의 주행 테스트를 포함한다.

```bash
python3 -m unittest discover -s src/mission/mission_evaluator/test -v
```

출발/완주 오인 방지, 순서 누락, 위치 점프, 시계 역행, 입력 누락, 패널티 반복 주기,
충돌 해제·재접촉, 속도 예외 경계, UDP 손상 패킷 및 launch 분리를 검사한다.
2026-09-09 로컬 검증: 평가 패키지 66개 테스트와 기존 stopline 제어 연결 11개 회귀 테스트 통과.
ROS 콜백 테스트는 메시지/시간 stub을 사용했으며 실제 ROS 마스터 실행 결과가 아니다.
Windows에서 로컬 테스트했으며 ROS Noetic catkin 빌드, Docker 실제 실행,
대회용 MORAI 연결·실주행·주최측 판정 프로그램과의 결과 일치는 아직 검증하지 않았다.
