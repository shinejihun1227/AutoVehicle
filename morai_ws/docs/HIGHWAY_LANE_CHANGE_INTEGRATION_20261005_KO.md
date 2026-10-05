# 고속도로 차선 변경 통합과 기능별 역할

## 적용 범위

- 원본: [ROI `test/rrt`의 `30e321a`](https://github.com/hyunho0429/ROI/commit/30e321a83c703bef49dc6656b9efa60daaa8dc75) (2026-10-04 확인). 이 브랜치의 현재 차선 변경 경로는 이름과 달리 RRT 탐색 경로가 아니라 **카메라에서 측정한 차선 경계에 대한 Frenet/5차 다항식 경로**다.
- 이 workspace의 `highway_lane_strategy_node.py`와 `lane_geometry.py`, `motion.py`, 고속도로 환경 판정 함수를 최신 구현으로 갱신했다. 카메라의 단일 UDP/추론 스트림과 현재 주행 제어기, 정지선 제어기, `control_mux`는 유지했다.
- 고속도로 시나리오 진입점은 `morai_bringup/launch/final_ws_highway_bringup.launch`다. `purepursuit_mgeo/launch/morai_avoidance_highway_roundabout_final.launch`도 그 launch를 가리킨다. 일반 경로 주행 진입점 `final_ws_bringup.launch`에는 고속도로 전략을 켜지 않았다.

## 누가 최종 결정을 내리는가

```text
GPS/IMU → EKF 위치·속도 ──────────────────────────────────────────────┐
카메라 1회 추론 → 차선/실선·점선/정지선/YOLO → 고속도로 환경 판정 ─┤
LiDAR 추적 → 정적 장애물 후보/차로 간격/안전정지 ──────────────────┤
                                                                       ↓
MGeo 전역 경로 → 정적 장애물 우회 → 경로 관리자 → 고속도로 차선 전략
         └── 경로 거리 s·지도 위치 검증 → 고속도로 구간 요청 / 회전교차로 합류 요청
                                                    │ 경로·정지·목표 속도
                                                    ↓
                           곡률 기반 Pure Pursuit + PI 가감속 (명령 1곳)
                                                    ↓
                           정지선 제어 → control_mux 안전 우선 → MORAI UDP
```

고속도로 전략은 `/ctrl_cmd`를 발행하지 않는다. 활성화 전과 종료 후에는 경로 관리자의 경로·정지 상태를 전달하고, 활성화 중에만 주행 경로와 목표 속도를 고른다. 곡률 기반 제어기는 그 경로를 추종한다. 정지선 제어와 `control_mux`의 안전정지는 이 뒤에 적용된다. 회전교차로 합류 게이트는 설정된 요청 구간에서만 추가 정지 신호를 낸다. `route_mission_gate`는 경로 거리, 지도 좌표와 카메라 고속도로 판정을 함께 확인한다.

## 상황별 기능과 해결 문제

| 상황 | 담당 기능 | 문제를 해결하는 방식 | 고속도로 launch에서의 상태 |
|---|---|---|---|
| 일반 경로 추종 | MGeo 경로, EKF, 곡률 기반 Pure Pursuit | 현재 위치와 전역 경로의 곡률로 조향·속도 명령을 만든다 | 활성 |
| 차로 내 정적 장애물 | Frenet 우회 후보, 차선 가드, 경로 관리자 | 장애물 충돌 후보를 피하는 경로를 선택하고 안전한 경로가 없으면 정지한다 | 고속도로 전략 진입 전/종료 후 경로 공급. 차선 변경은 이 기능에 맡기지 않음 |
| 고속도로 진입 확인 | MGeo 경로 구간, 카메라 YOLO 차량(car/bus/truck)+인접 점선 또는 반복 관측된 다차로 패턴, 교차로 우선 판정 | 지정된 경로 구간과 카메라 판정이 함께 맞을 때만 차선 변경 전략을 시작한다 | 활성. 다차로 패턴 보조 판정은 이 launch에서만 켬 |
| 왼쪽 차선 변경 준비 | 고속도로 전략 `WAIT_GAP`, 카메라 차선, LiDAR merge gap | 측정된 현재 차로 중심을 유지하며 인접한 **가장 가까운** 왼쪽 경계가 점선인지, 앞뒤 간격·TTC·예상 경로 충돌·곡률이 안전한지 확인한다 | 활성. 외부 merge gap 확인도 유지 |
| 왼쪽 차선 변경 | `LANE_CHANGE`의 차선 기준 5차 경로 | 한 번에 한 차로만 이동하고, 실제 속도에 맞는 경로 길이와 횡가속도 한계를 확인한다. 진입 후에는 가까운 충돌과 선행차 긴급 상황을 재평가한다 | 최대 2회, 제동과 진입 후 충돌 정지 켬 |
| 변경 후 차로 유지·다음 변경 | `INNER_HOLD`, 차선 중앙/실선 잠금, 재변경 거리·시간 확인 | 원래 전역 경로로 즉시 끌려가는 것을 막고, 새 차로에 안정적으로 안착한 뒤에만 다음 변경을 허용한다 | 활성 |
| 지정 지점 이후 대회 경로 복귀 | 경로 거리 `handoff_s_m`, `REJOIN`, `DONE` | 지정 지점 전에는 카메라 차로 중심 경로를 유지한다. 지점을 지난 뒤 전역 경로가 같은 차로에 있고 충돌 검사가 통과하면 부드럽게 인계한다 | 인계 지점 미측정으로 자동 복귀 대기 |
| 선행차 접근·급위험 | LiDAR 추적, 고속도로 목표 속도/정지, 센서 안전 어댑터 | 같은 주행 경로의 선행차를 골라 차간거리·TTC로 감속한다. 보행자·근접 장애물·센서 이상은 최종 안전 계층이 정지시킨다 | 활성 |
| 신호·정지선 | 카메라 신호/정지선, 정지선 제어기 | 조향 경로와 별도로 앞범퍼 정지 위치까지 감속·정지한다 | 활성 |
| 회전교차로 합류 | MGeo 구간 요청, 지도상 충돌 구역, LiDAR 차량 추적 속도 | 자차와 순환 차량의 충돌 구역 점유 시간을 비교하고 겹치면 대기한다. 진입 승인 뒤에도 센서 상태와 가까운 충돌을 재확인한다 | **위치 미측정으로 자동 요청 비활성**. 좌표·링크 보정 전에는 합류 판단을 켜지 않음 |
| GPS 불량·차선 대체, 신호 방향 융합·방향지시등 | 센서 품질, 카메라 위치 대체, `maneuver_fusion`, 방향지시등 | 일반 경로 주행의 GPS blackout/교차로 방향별 신호 및 사전 점등을 담당한다 | 일반 `final_ws_bringup.launch`에서 활성. 고속도로 전용 launch에서는 경로/명령 중복을 피하려고 비활성 |

## 병합 시 충돌을 줄인 연결 규칙

1. 카메라 UDP 포트 1101은 `live_overlay.py` 하나가 수신한다. 같은 추론 결과로 차선·정지선·고속도로 차선 정보를 발행한다. 별도 `live_lane_info_publisher_v2.py` 프로세스를 동시에 실행하지 않는다.
2. `lane_info.timestamp`는 기존 ROS 수신 시각 계약을 유지한다. `observation_wall_timestamp`는 수신 시각의 wall-clock 표현으로 추가해 차선 관측 당시의 EKF 자세로 재투영한다. HELD 출력은 최초 FRESH 시각을 유지해 새 차선 변경의 근거가 되지 않는다.
3. 인접 왼쪽 **실선** 관측은 점선 hold를 즉시 취소한다. 다차로 패턴은 고속도로 활성화 보조 신호이며, 실제 차선 통과 허가는 가장 가까운 경계·LiDAR 간격·경로 충돌 검사가 따로 결정한다.
4. 전역 경로 우회 기능의 장애물 차선 변경은 고속도로 launch에서 꺼 두었다. 고속도로 전략이 켜지기 전/끝난 후 정적 우회는 경로 관리자가 맡고, 고속도로 활성 중 차로 이동은 고속도로 전략만 맡는다.
5. 목표 속도는 `max_speed_kph` → `target_speed_mps` → 고속도로 전략 → 곡률 제어기 순으로 전달한다. 정지선/합류/안전 mux는 이후에도 정지 권한을 갖는다. 새 고속도로 전략의 제동 해제 옵션은 사용하지 않는다.
6. 고속도로 차선 변경 허용 구간은 현재 경로 파일의 SHA-256과 `s=1118.742~1741.721 m`에 묶었다. 경로가 달라지면 설정을 그대로 재사용하지 않고 기동 단계에서 거부한다. 구간 밖에서는 새 차선 변경을 시작하지 않고, 이미 시작한 변경은 완료한다. 차로 중심 유지에서 전역 경로로 넘어가는 지점은 별도 `handoff_s_m`으로 지정한다.

## 고속도로 차로 유지에서 대회 경로로 인계

`route_mission_regions.json`의 `highway.end_s_m`은 **새 차선 변경을 시작할 수 있는 구간의 끝**이다. `highway.handoff_s_m`은 **차로 중심 주행을 끝내고 대회 전역 경로로 돌아갈 수 있는 최소 경로 거리**다. 두 지점은 별도로 측정한다. `handoff_s_m`이 `end_s_m`보다 뒤이면 그 사이에는 새 차선 변경 없이 현재 차로 중심을 유지한다.

인계 지점은 아직 확인되지 않아 `handoff_enabled=false`, `handoff_s_m=null`이다. 이 상태에서 고속도로 구간의 끝에 도달하면 `/route_mission_gate/ready=false`로 주행 제어기가 정지한다. 진행 중인 차선 변경이 있으면 완료된 뒤 정지한다. 임의 위치에서 전역 경로로 바뀌지 않도록 한 설정이다. 기록 도구로 지점의 `map` 좌표와 경로 거리 `s`를 측정한 후 `handoff_s_m`을 입력하고 `handoff_enabled=true`로 바꾼다. 전역 경로와 실제 주행 차로가 충분히 가까워지고, 차선 경계와 장애물까지 확인할 수 있는 지점을 고른다. 기존 경로 파일이 달라지면 해시와 모든 지점을 다시 확인한다.

설정을 바꾼 뒤에는 고속도로 launch를 다시 시작한다. `handoff_s_m`은 경로 시작점부터의 누적 거리이며 지도 좌표나 MGeo 링크 ID를 직접 넣는 칸이 아니다.

```bash
rosrun purepursuit_mgeo record_route_landmarks.py --output ~/highway_handoff_landmarks.jsonl
```

지점 통과 시 `/planning/highway_handoff_due=true`가 된다. 주행 전략은 진행 중인 차선 변경을 끝낸 뒤, 현재 차로 경계 안에서 만들 수 있는 복귀 경로인지, 전역 경로와의 거리, LiDAR 충돌, 경로 관리자 정지 상태를 확인한다. 이 조건이 충족될 때 `REJOIN`을 거쳐 `DONE`에서 대회 경로를 사용한다. 지점이 지났다는 이유만으로 떨어진 경로에 바로 조향 명령을 넘기지 않는다. `/highway_lane_strategy/state`의 `handoff_permitted`, `state`와 `/route_mission_gate/status`의 `handoff_due`로 전환 상태를 볼 수 있다.

## MGeo 위치 측정과 회전교차로 보정

위치 설정은 `purepursuit_mgeo/config/route_mission_regions.json`이다. 현재는 회전교차로의 양보선, 자차와 순환 차로의 충돌 중심, 순환 차량의 이동 방향 링크가 확인되지 않았으므로 `roundabout.enabled=false`와 위치값 `null`을 유지했다. 임의 좌표로 합류를 승인하지 않는다.

1. RViz에서 `/localization/odometry`와 전역 경로를 `map` 좌표계에 표시한다. 회전교차로 접근 시작점, 양보선, 충돌 중심, 충돌 구역을 지난 종료점을 기록한다. 주행 중 아래 기록 도구에서 각 지점마다 Enter를 누르면 그 순간의 `map` 좌표와 `/experimental/curvature_progress`가 JSONL로 남는다. 회전교차로에 선 곳에서 측정하고, 눈으로만 추정한 값을 입력하지 않는다.

```bash
rosrun purepursuit_mgeo record_route_landmarks.py --output ~/roundabout_landmarks.jsonl
```
2. 기록한 지도 좌표를 아래 명령에 넣어 경로 거리 `s`, 경로 이탈 거리, 가장 가까운 MGeo 링크 ID와 방향을 확인한다. 경로가 같은 장소를 여러 번 지나는 경우 `--hint-s`에 관측된 진행 거리를 넣는다.

```bash
python3 morai_ws/src/control/purepursuit_mgeo/scripts/locate_route_region.py \
  --path-file morai_ws/data/routes/2026_molit_comp_global_path.txt \
  --link-set-file morai_ws/data/mgeo/R_KR_PR_K-city_2025/link_set.json \
  --xy <map_x> <map_y> --hint-s <observed_s>
```

3. 접근 정지를 시작할 `request_start_s_m`, 양보선 `yield_s_m`, 충돌 중심 `conflict_s_m`, 종료 `request_end_s_m`을 경로 진행 순으로 입력한다. `request_start_s_m`은 양보선에 닿기 전에 현재 속도에서 정지할 거리만큼 앞에 두되, 순환 차로가 LiDAR 관측 범위에 들어오는 위치를 선택한다. `conflict_xy_map`은 자차 경로와 **순환 차량 경로가 실제로 겹치는 지도 좌표**다. `circulating_link_ids`는 이 지점을 통과하는 순환 차로 링크를 차량 진행 순으로 적는다. 가까운 링크 ID만 보고 자동으로 방향을 확정하지 말고 MGeo 연결 관계와 시뮬레이터 주행 방향을 확인한다.
4. 값과 경로 파일의 SHA-256이 맞는 것을 확인한 뒤 `enabled=true`로 바꾼다. 요청 구간이 고속도로 구간과 겹치거나 링크가 끊어져 있거나 충돌 원이 두 경로와 만나지 않으면 노드가 설정을 거부한다. `/route_mission_gate/status`, `/roundabout_merge_gate/status`, `/roundabout_merge_gate/stop_required`로 판단 상태를 관찰한다.

회전교차로 게이트는 LiDAR 추적 물체를 순환 차로 링크에 대응시키고, 차량의 진행 방향 속도에 오차 여유를 더해 충돌 구역 도착·이탈 시간을 예측한다. 자차는 정지 후 출발 지연과 가속도 범위를 적용해 점유 시간을 잡는다. 시간 구간이 겹치거나 출처 시각이 오래된 센서 데이터면 정지 신호를 낸다. 가려진 차량이나 트래킹되지 않은 차량은 이 예측에 포함될 수 없으므로 현장 데이터로 탐지 범위와 정지 위치를 확인해야 한다.

## 실행과 현재 한계

```bash
roslaunch morai_bringup final_ws_highway_bringup.launch enable_control:=false
```

기본 상한은 7.2 km/h이며, launch 기본값은 차량 제어를 끈다. 실제 고속 주행 성능, 카메라 모델의 다차로 인식률, LiDAR 좌표계/센서 시각과 MORAI 현장 차량의 제동 거리는 이 코드 병합만으로 입증되지 않았다. 회전교차로 위치가 미측정이라 자동 합류는 아직 사용할 수 없다. 또한 고속도로 전용 launch의 방향지시등 선행 점등·GPS 차선 대체는 아직 일반 주행 launch와 공통 제어 흐름으로 묶지 않았다. 이 기능이 필요한 실주행에서는 시나리오 경로·센서·조향·정지 토픽을 함께 확인해야 한다.
