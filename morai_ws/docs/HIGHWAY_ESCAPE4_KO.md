# 고속도로 escape4 병합 (final_ws_curvature_signal.launch)

기본 주행(신호등·정지선)과 회전교차로가 들어 있는 `final_ws_curvature_signal.launch`에 고속도로 로직 escape4를 넣었습니다. `enable_highway`(기본 `true`)로 켜고 끕니다. `false`면 병합 전과 똑같이 동작합니다.

## 1. 동작

| 구간 (전역경로 s) | 운전 |
|---|---|
| 고속도로 밖 | 메인 제어기(`curvature_speed_purepursuit`)가 지금처럼 운전 |
| 1160.2 ~ 1590.2 (고속도로 구간) | escape4가 카메라 차선 기준으로 진입차로 → L1 → L2 → L3 차로 변경. L3에서는 앞차만 따라감. 순항 54 km/h, 최고 79.2 km/h |
| 1590.2 ~ 1741.7 | 전역경로를 그대로 따라감. 고속도로 끝(A2256W000153 끝) 10 m 앞부터 28 km/h |
| 1741.7 이후 | 다시 메인 제어기 |

명령 흐름은 다음과 같습니다.

```
메인 제어기 ──/control/ctrl_cmd_main──────┐
고속도로 제어기 ──/control/ctrl_cmd_highway──┴▶ highway_command_mux ──/control/ctrl_cmd──▶ fusion ──/ctrl_cmd──▶
```

- **`highway_command_mux`:** s 1160.2부터 1741.7까지는 고속도로 제어기의 명령을, 그 밖에서는 메인 제어기의 명령을 fusion에 넘깁니다. 고속도로 쪽 상태나 명령이 0.5초 넘게 끊겨도 메인으로 넘깁니다. 바뀔 때 `HIGHWAY_MUX highway controller (...)` / `HIGHWAY_MUX main controller (...)` 로그가 나옵니다.
- **메인 제어기:** 고속도로에서도 계속 돌아갑니다. 명령만 쓰이지 않습니다.
- **장애물 회피:** 고속도로에서는 쓰지 않습니다(고속도로 방식만).

## 2. 추가한 노드 (`enable_highway`일 때만)

| 노드 | 파일 | 하는 일 |
|---|---|---|
| `highway_base_path` | `scripts/highway_base_path_node.py` | 전역경로 중 차 주변(뒤 10 m ~ 앞 150 m)을 10 Hz로 escape4에 줌 |
| `highway_lane_strategy` | `scripts/highway_overtake_escape4_node.py` | escape4 플래너(`highway_overtake_escape4.py`) |
| `highway_curvature_purepursuit` | `scripts/curvature_pp_overtake_node.py` | 팀원 `adaptive_curvature_purepursuit_node.py`를 그대로 불러와 escape4 경로·속도를 따라감. 구간 안에서만 최고속도·횡가속도 한도를 올림. 디버그 토픽은 `/highway_control/*`로 바꿈 |
| `highway_command_mux` | `scripts/highway_command_mux_node.py` | 위 명령 선택 |

fusion(`curvature_signal_controller`)은 같은 `maneuver_fusion_node.py`를 `scripts/maneuver_fusion_overtake_node.py`로 감싸 실행합니다. 고속도로 구간 안에서만 속도 상한을 `highway_max_speed_kph`로 올리고, 그 밖에서는 원래 그대로입니다. LiDAR 추적기는 회전교차로와 같은 것을 같이 씁니다(회전교차로를 꺼도 고속도로가 켜져 있으면 실행).

## 3. 실행

이전과 같습니다.

```bash
bash run_test.sh monitor 2
bash run_test.sh drive 2
```

고속도로를 빼고 돌리려면 `highway-test.env`에 `HIGHWAY_ENABLED=false`를 넣습니다. 직접 실행할 때는 `roslaunch ... enable_highway:=false`입니다.

설정값은 다음 파일에 있습니다.

- `src/control/purepursuit_mgeo/config/highway_overtake.yaml`: 구간 좌표, 간격 판단
- `src/control/purepursuit_mgeo/config/highway_overtake_escape4.yaml`: escape4 값. 위 파일 다음에 덮어씀
- 런치 인자: `highway_cruise_kph`, `highway_max_speed_kph`, `highway_lateral_accel_mps2`, `highway_end_s`

## 4. 바뀐 기존 파일

| 파일 | 변경 |
|---|---|
| `src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch` | 인자 7개, LiDAR 실행 조건, 메인 제어기 출력 토픽(`enable_highway`일 때 `/control/ctrl_cmd_main`), fusion 래퍼, 위 노드 4개 |
| `docker/final_ws/check_highway_launch.py` | 고속도로 켬/끔 × 회전교차로 켬/끔 네 조합을 모두 검사 |
| `docker/final_ws/test_check_highway_launch.py` | 고속도로 경우 테스트 3개 추가 |
| `src/control/turn_signal_controller/test/test_curvature_signal_launch.py` | fusion을 노드 이름으로 찾음. 메인 제어기 출력 토픽 식 |
| `docker/final_ws/run_test.sh` | case 2에 `enable_highway:=${HIGHWAY_ENABLED:-true}` |
| `docker/final_ws/highway-test.env.example` | `HIGHWAY_ENABLED=true` |
| `docker/final_ws/curvature_runtime_files.sh` | 고속도로 yaml 두 개를 설치 목록에 추가 |
| `src/control/purepursuit_mgeo/CMakeLists.txt` | 새 노드 스크립트 6개 등록 |

## 5. 확인

- 오프라인: `python3 morai_ws/tools/validate_new_final.py`. `src/control/purepursuit_mgeo/test/test_highway_section.py`(선택 규칙, 경로 구간, 런치 연결)가 함께 돌아갑니다.
- Docker 빌드 중 `check_highway_launch.py`가 네 조합의 노드 구성을 확인합니다.
- 주행 중 볼 로그:
  - `HIGHWAY_MUX highway controller (WAIT_LANE)`: 고속도로 구간 시작
  - `left line under the ego (camera): crossing N`: 차로 변경 중 선을 넘음
  - `final lane reached (changes=3, crossings=3, ...)`: L3 도착
  - `HIGHWAY_MUX main controller (past_highway_end)`: 고속도로 끝

## 6. 나중에 장애물을 합칠 때

경로를 바꾸는 기능은 고속도로처럼 따로 운전합니다. 그래서 `highway_command_mux`가 "지금 누가 운전하나"를 정하는 한 곳입니다. 장애물 회피를 넣을 때는 이렇게 하면 됩니다.

1. 회피 제어기의 명령을 `/control/ctrl_cmd_avoidance` 같은 별도 토픽으로 냅니다.
2. 선택기(`purepursuit_mgeo/highway_section.py`의 `mux_choice`와 `highway_command_mux_node.py`)에 회피 쪽 조건을 더합니다.
3. 고속도로 구간에서는 지금처럼 고속도로가 우선합니다.

## 7. 알아둘 점

- 고속도로 끝 이후(s 1741.7~)는 메인 제어기의 `MAX_SPEED_KPH`로 달립니다. 톨게이트까지 30 km/h 제한은 따로 처리하지 않았습니다.
- 들여온 것은 고속도로 차로 변경(escape4)뿐입니다. v61 장애물 회피 스택은 들여오지 않았습니다.
- escape4 자체 설명(넘은 선 세기, 옆속도 0.8 m/s 상한, 추월 3초 규칙)은 고속도로 작업 저장소(`whddbs3061/car`, `highway-overtake-v61` 브랜치)의 `docs/HIGHWAY_OVERTAKE_V61_KO.md` 9~11절에 있습니다.
