# 장애물 우회·끼어들기 통합 검증과 정지 원인 확인

대상: `final_ws_highway_bringup.launch`, `TEST_PROFILE=obstacle/merge/full`.
Ubuntu 호스트 `192.168.0.185`, MORAI Windows `192.168.0.147`.
아래 명령은 별도 표시가 없으면 **Ubuntu 호스트 터미널**에서 실행한다.

## 1. 장애물을 보면 멈추는 코드인가?

항상 정지하는 것은 아니다. 원래 대회 경로를 막는 장애물이 있으면 Frenet 우회 후보를
만들고, 카메라 차선 경계와 차량 크기·곡률·충돌 검사를 통과한 경로를 곡률 Pure Pursuit로
추종한다. 원본 `2026_molit_comp_global_path.txt`는 수정하지 않는다.

| 상황 | 현재 동작과 확인 위치 |
| --- | --- |
| 현재 차선 안에 충분한 우회 공간 | 우회 경로 추종, PathManager `AVOIDING_BYPASS` |
| 같은 차선에서는 통과 불가, 인접 차로 점선·경계·간격 검증 통과 | 인접 차로로 나갔다 복귀하는 장애물 우회, `AVOIDING_LANE_CHANGE` |
| 양쪽이 막혔거나 차선 경계가 불확실 | `STOP_NO_SAFE_PATH`, `/bypass_lane_guard/status`의 `lane`, `base_bypass_road_check`, `lane_change_reject` 확인 |
| 선택한 우회 경로에도 충돌이 예상됨 | 경로 재검사에서 정지. 다른 경로가 안전하다는 근거 없이 브레이크를 해제하지 않음 |
| 카메라·라이다·위치·계획 입력이 만료됨 | 입력별 정지. YOLO가 검출되는 것만으로 모든 입력이 정상인 것은 아님 |
| 앞차가 느리거나 다른 차가 내 경로로 끼어듦 | 추종 속도 감소 또는 충돌 예측에 따른 정지 |
| 우리 차가 왼쪽 차로로 끼어들기를 요청함 | `WAIT_GAP`에서 차선·앞뒤 간격 검사 후 `LANE_CHANGE` |
| `full`에서 정지선·신호 판단이 정지를 요구함 | 우회 경로가 있어도 최종 정지 가능. `/control/stopline_status` 확인 |

통합 런치의 라이다 안전 어댑터는 단순히 정면 사각형에 장애물이 있다는 이유만으로
우회를 차단하지 않는다. 승인된 주행 경로의 충돌 판단과 센서 최신성을 사용한다.

## 2. 코드에서 재현하여 수정한 문제

- 계획 경로와 안전 판단을 별도 ROS 토픽의 최근 값으로 조합했다. 경로 N+1이 먼저 오고
  상태 N+1이 아직 오지 않으면, 정상적인 전달 순서 차이에도 번호 불일치로 정지할 수 있었다.
  `plan_status`에 해당 경로 좌표·좌표계·생성 시각을 함께 넣어 한 번에 받는다.
- 우회를 처음 확정하는 타이머에서 기존 직진 경로를 발행한 뒤 우회 경로를 다시 발행했다.
  이제 그 주기에서 선택한 경로를 한 번만 발행한다.
- PathManager → 차로 변경 → 곡률 제어도 경로·정지·속도가 같은 판단에 속하도록
  `/avoidance_path_manager/trajectory`, `/highway_lane_strategy/trajectory`로 전달한다.
  기존 Path/Bool/Float64 토픽은 확인·시각화용으로 남는다.
- 새 메시지의 잘못된 좌표, 만료된 생성 시각, 누락 필드, 입력 중단은 정지 조건이다.
  별도 토픽의 과거 `stop=false`가 새 정지 판단을 덮어쓰지 못한다.
- `/highway_lane_strategy/state`와 `/control/curvature_status`에 상위 정지 이유와
  차로 변경 요청 활성 여부를 추가했다.

검증 범위는 실제 노드 콜백을 이용한 ROS 모의 전송 테스트, 우회 후보/충돌 검사,
곡률 제어 및 라이다 회귀 테스트다. **사용자 Ubuntu의 ROS 통신, Docker 실행과
MORAI 실주행을 원격으로 확인한 결과는 아니다.** 실제 정지 원인은 4절 로그로 대조한다.

## 3. 기존 Docker에 이번 수정 적용

`git pull`만 하면 호스트 파일만 갱신된다. 현재 이미지에는 소스가 복사되어 있으므로
기존 컨테이너의 코드도 갱신해야 한다. 이번 변경은 Python·launch 파일만으로 적용할 수 있다.
이미지 재빌드나 모델 재설치는 필요하지 않다. **주행 터미널에서 Ctrl+C로 launch를 종료한다.**

```bash
cd "$HOME/AutoVehicle"
git pull --ff-only origin final_ws
cd morai_ws/docker/final_ws
bash run_highway.sh stop
bash install_avoidance_transport.sh
bash run_highway.sh start
```

설치 스크립트는 `highway.env`의 컨테이너 하나만 대상으로 한다. 실행 중인 컨테이너에는
적용하지 않으며, 기존 파일은 호스트 `~/morai-update-backups/`에 백업한다.
차선 모델, ROS 메시지, 카메라 보정, `highway.env`, `highway-test.env`는 유지한다.
중간에 오류가 나면 주행하지 말고 오류를 확인한다. 해당 경로가 없는 오래된 이미지라면
기존 Docker 안내의 이미지 빌드·새 컨테이너 생성 절차를 사용한다.

컨테이너에서 테스트를 실행한다. 이 테스트는 차량 제어를 송신하지 않는다.

```bash
bash run_highway.sh shell
```

**컨테이너 안에서:**

```bash
cd /opt/AutoVehicle/morai_ws
python docker/final_ws/run_regression.py --suite avoidance
python docker/final_ws/run_regression.py --suite curvature
python docker/final_ws/run_regression.py --suite lidar
python docker/final_ws/check_highway_launch.py
exit
```

각 `REGRESSION_PASS`와 `HIGHWAY_LAUNCH_OK`를 확인한다. 위 `exit`는 호스트 터미널로 돌아온다.

## 4. 통합 주행과 8초 진단

먼저 장애물·끼어들기 동작을 신호 정지와 구분해서 본다. `merge`는 장애물 우회와
우리 차의 요청 기반 차로 변경을 모두 실행하며, YOLO·정지선 제어를 켜지 않는다.
기존 속도·조향 설정은 그대로 유지되므로 `show`에서 실제 값을 먼저 확인한다.

**호스트 터미널 A:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
sed -i 's/^TEST_PROFILE=.*/TEST_PROFILE=merge/' highway-test.env
bash run_test.sh show
bash run_test.sh monitor
```

`monitor`는 인식·계획을 실행하면서 UDP 차량 제어 송신을 끈다. MORAI 센서 송신을 켜고
아래 진단으로 입력과 계획을 먼저 확인한다. 그 후 터미널 A에서 Ctrl+C를 누르고:

```bash
bash run_test.sh drive
```

**정지하거나 멈췄다 출발하는 순간, 별도 호스트 터미널 B:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_test.sh diagnose | tee "$HOME/morai-avoidance-diagnosis.log"
```

8초 동안 판단이 바뀐 시각, 정지 이유별 수신 횟수, 입력 주기와 마지막 상태를 출력한다.
종료되지 않는 ROS 초기화 등에 대비해 바깥 실행 제한도 있다. 제어 명령이나 차로 변경
요청은 보내지 않는다. 이 로그를 공유하면 정지한 정확한 계층을 구분할 수 있다.

| 로그 | 해석 |
| --- | --- |
| `count: 0`인 odometry 또는 LiDAR | 해당 입력이 수신되지 않음. 네트워크·센서·노드를 먼저 점검 |
| `roi_lidar_stale` | 라이다 데이터 생성 시각/수신 간격 문제 |
| `STOP_PLANNER_NOT_READY`, `upstream_or_sensor_stale` | 우회 계획이나 필요한 센서 입력이 준비되지 않음 |
| `STOP_NO_SAFE_PATH` | 통과 가능한 경로가 승인되지 않음. guard의 차선/우회 거절 이유 확인 |
| `AVOIDING_BLOCKED`, `predicted_collision_id_...` | 확정 경로에서 장애물 충돌 검사에 걸림 |
| `OFF`, `mission_request_active: false`, `stop: false` | 장애물 우회는 가능하지만 요청 기반 끼어들기는 아직 시작하지 않음 |
| `WAIT_GAP` | 차선/간격/후방 차량/확인 시간 조건을 기다림. 상태 자체가 정지 명령은 아님 |
| 곡률 제어는 가속, 최종 `/ctrl_cmd`만 브레이크 | stopline 또는 mux/safety의 이유 확인 |

필요하면 첫 검증 속도만 낮춘다. 목표속도 45를 그대로 두어야 한다는 뜻은 아니다.

```bash
sed -i 's/^MAX_SPEED_KPH=.*/MAX_SPEED_KPH=5.0/' highway-test.env
```

## 5. 우리 차의 끼어들기 요청 시험

`merge` 런치만 켜도 자동으로 왼쪽 차로에 들어가는 구조가 아니다. **시험 경로상 왼쪽
차로 변경을 해도 되는 구간**에 차량을 놓고, 점선·인접 차로 양쪽 경계·앞뒤 차량이
카메라와 라이다로 관측되는 상태에서 요청한다.

**별도 호스트 터미널 C:**

```bash
cd "$HOME/AutoVehicle/morai_ws/docker/final_ws"
bash run_highway.sh shell
```

**컨테이너 안에서:**

```bash
rostopic pub -r 5 /planning/highway_lane_change_request std_msgs/Bool 'data: true'
```

요청을 보내도 안전 조건이 충족되어야 차로를 변경한다. 준비 중 요청 송신을 Ctrl+C로
끝내면 약 1초 후 요청이 만료된다. 이미 차로 변경에 착수했다면 요청 중단이 즉각적인
반대 방향 복귀를 뜻하지는 않는다. 기본 변경 횟수는 1회다.

현재 간격 판단 상세 JSON 토픽은 `/morai/lidar/merge_gap/results`다.
허용/불가 Bool 토픽은 `/perception/merge_gap/available`, `/perception/merge_gap/unavailable`이다.

신호·정지선까지 함께 시험할 때는 주행 launch 종료 후 `TEST_PROFILE=full`로 바꿔 실행한다.
이 경우 Camera 1101뿐 아니라 YOLO Camera 1131도 필요하다.
