# 신호·방향지시등·정지선 최종 융합 제어

`final_ws_bringup.launch`의 기본 제어 경로:

```text
GPS + IMU -> EKF -> 경로 Pure Pursuit -> 차선 fallback
                                          |
카메라 방향별 신호 + 정지선 + 경로 --------> maneuver_fusion -> control_mux -> CtrlCmd UDP
LiDAR 장애물/합류 공간 + 보행자 정지 ------->      |              ^
                                                +-> LampControl UDP
```

`maneuver_fusion_node.py`가 정지선 제어를 담당하므로 기존 stopline wrapper는
이 모드에서 pass-through다. Fallback 뒤에서 브레이크 제한을 적용하고, 마지막
`control_mux`도 안전 정지와 명령 timeout을 검사한다. 신호가 허용돼도 장애물이나
보행자 정지를 해제하지 않는다. 종방향 명령은 모두 `longlCmdType=1`이다.

## 방향 판단과 점등

신호등의 화살표는 해당 방향의 **통행 허가**이고 목적 경로를 바꾸는 명령이 아니다.
최종 기본 모드는 `require_route_signal_context: true`다. 정적 MGeo의 신호등
`link_id_list`와 경로 전체 링크의 위치·진행 방향을 대조하고, `related_signal`의
직진/좌회전/우회전 의미를 사용한다. 도로 곡률만으로 교차로 회전을 결정하지 않는다.
평행 차로·반대 방향·겹치는 경로 등 모호한 연결은 UNKNOWN으로 정지한다.
경로의 해당 링크가 전부 포함되지 않으면 연결하지 않는다. 비보호 좌회전·유턴은
별도 정책이 없어 UNKNOWN이다. 기본 30m/25도 추정은 레거시 시험 모드에서만 사용한다.

지도는 **신호등의 위치와 연결 관계만** 제공한다. 현재 색은 시나리오 JSON이나
MGeo의 `value`를 읽지 않고 `/detection/traffic_light`의 실제 카메라 검출에서 얻는다.
영상 시각과 50ms 이내의 EKF 자세로 대상 신호등을 영상에 투영한다. 검출 bbox와
유일하게 대응해야 하며, 다른 신호등도 대응 가능하면 UNKNOWN이다. 화면 중앙이나
최대 신뢰도만으로 신호등을 고르지 않는다. 좁거나 낮게 보이는 신호등을 배제하던
기존 가로세로 비율/높이 필터는 제거했다.

동일 신호등·동일 허가 상태를 서로 다른 영상 3개 이상, 0.3초 이상 확인한다.
동일 영상의 재처리는 횟수에 포함하지 않고, 적색/황색/모호한 관측은 허가를 즉시
취소한다. 카메라 timeout, 경로 변경, 위치 품질 저하도 확인 이력을 초기화한다.
`/control/maneuver_status`의 `selected_signal_id`, `selected_signal_state`,
`signal_selection_reason`, `route_context_count`로 판단 근거를 확인한다.

실제 YOLO용 Cam4/1131 카메라 보정이 저장소에 없어 `signal_camera.calibrated=false`가
기본이다. 원본 해상도·수평 FOV·장착 위치/방향·지도 위치/높이 정렬을 검증하기 전에는
교차로 통행을 허가하지 않는다. 차선 카메라 설정을 복사해서 우회하면 안 된다.
투영은 왜곡 없는 핀홀, 중앙 주점, 정사각 픽셀, 카메라 roll=0을 가정한다.
이 조건을 만족하지 않는 카메라는 먼저 보정 모델을 확장해야 한다.

`/perception/traffic_light/directional_state`는 전체 검출의 진단용 방향 정보다.
엄격 모드의 통과 허가는 이 집계가 아니라 위의 경로 연결·영상 확인 결과로 결정한다.
`GREEN`은 직진과 우회전, `LEFT`/`RED_LEFT`는 좌회전, `RIGHT`/`RED_RIGHT`는
우회전만 허용한다. `GREEN_LEFT`/`GREEN_RIGHT`는 직진과 해당 방향을 함께 허용한다.
이 의미는 기존 YOLO 클래스의 점등 조합 계약이며, 사용 모델의 실제 라벨과 맞춰야 한다.
서로 다른 신호등의 상충 결과를 합쳐 허가하지 않는다. 일반 `Arrow`, 낮은 신뢰도,
오래된 결과는 UNKNOWN이다. 황색과 적색은 정지한다. 적색 우회전은 구현하지 않았다.
`right_on_green: false`를 설정하면 우회전 화살표가 있어야 허용한다.

좌·우회전 의도가 확정되면 해당 방향지시등을 켜고, 최소 5초 연속 UDP 송신 후
신호 허가를 확인해 출발한다. 시뮬레이션 시간과 실제 경과시간 모두 5초를 요구한다.
송신 실패·입력 끊김·방향 변경은 점등 시간을 초기화한다. UDP 성공은 송신 성공이며
시뮬레이터의 램프 점등 ACK는 아니다. 실제 점등은 MORAI에서 확인해야 한다.

허가 후 앞범퍼가 정지선을 통과한 조작은 경로 끝까지 유지한다. 회전 도중
신호등이 시야 밖으로 사라져도 같은 조작을 이어가되 장애물 정지는 계속 적용한다.
회전 종료는 해당 MGeo 링크의 실제 경로상 끝이다. 종료한 tick 안에 다음 교차로를
다시 검색하고 신호 확인 이력을 초기화하여 이전 녹색 허가를 넘기지 않는다.
현재 교차로 안에서도 다음 정지선에 대한 독립 제동 제한을 적용한다. 차량 길이보다
교차로 사이가 짧으면 뒷차축이 이전 링크 끝에 닿기 전 정지할 수 있으며,
`next_junction_guard`가 유지되면 지도상의 종료 경계를 현장에서 재검증해야 한다.
현재 대회 경로의 6개 연결 구간은 이처럼 겹치지 않는다. 임의로 가속해 해제하지 않는다.

차선 변경은 기존 경로에 변경 구간이 포함돼 있어야 하며 `maneuvers`에
`kind: lane_change`, `start_s_m`, `end_s_m`, `direction`을 등록한다. 5초 점등과
해당 방향의 최신 LiDAR `confirmed_available`을 모두 요구한다. 비어 있는 목록에서도
MGeo 기반 교차로 판단은 동작한다. 수동 조작과 지도 교차로는 시작거리 순으로 처리하고,
같은 시작점에서는 지도 연결을 우선한다. 신호 제어 구간과 겹치는 차선 변경은 거부한다. 지도와
연결되지 않은 수동 회전은 엄격 모드에서 통행을 허가하지 않는다.
이 코드는 새로운 차선 변경 궤적을 생성하지 않는다.

## 앞범퍼 0.5m 정지 목표

```text
앞범퍼~정지선 거리 = BEV ego~정지선 거리 - 앞범퍼 오프셋 - 영상 지연 중 이동거리
남은 제동거리      = 앞범퍼~정지선 거리 - 0.5m
```

기본 앞범퍼 오프셋 `3.845m`는 저장소의 뒤차축 base_link 가정과
`vehicle_model.yaml`의 휠베이스 `3.000m` + 앞 오버행 `0.845m`를 사용한다.
MORAI 차량 원점이나 BEV 좌표 원점이 다르면 `stopline_front_reference_offset_m`을
실측해 바꿔야 한다. 차체 중심을 원점으로 쓰는 경우 3.845m를 그대로 쓰면 안 된다.

속도에 따른 제동거리와 반응 지연을 반영해 감속하고, 목표 근처에서는 제동을 유지한다.
영상 거리에는 수신 이후 지연을 보정한다. 별도 미측정 센서 내부 지연은 포함되지 않는다.
정적 지도에서 `on_stop_line` 노드까지 경로 연결이 검증된 경우, 카메라 미검출에도
그 지도 목표를 EKF 진행거리와 결합한다. 정지선이 확인되지 않은 링크 진입점은
보수적 정지용으로만 사용하며, 녹색이어도 카메라의 일치 관측 전에는 통과하지 않는다.
지도 연결이 없는 곳에서 신호등/정지선을 보면 UNKNOWN 정지를 유지한다.
레거시 모드의 카메라 정지선 소실은 최대 8초/12m 속도 추정으로만 보완한다.
오도메트리 끊김·시간 역행·예측 유효 범위 초과는 정지 처리한다.
0.5m는 제어 목표이며 실제 오차는 카메라 보정과 MORAI 제동 응답에 달려 있다.

## 실행과 확인

시나리오 로드, 네트워크, Cam4 보정, 저속 단계별 시험은
[시나리오 주행 시험 가이드](SCENARIO_DRIVING_TEST_KO.md)를 먼저 따른다.

Ubuntu ROS Noetic에서 새 패키지를 빌드한다.

```bash
cd ~/morai_ws
catkin_make
source devel/setup.bash
roslaunch morai_bringup final_ws_bringup.launch enable_control:=false
```

기본 미리보기에서는 차량 제어 UDP와 LampControl을 모두 송신하지 않는다.
Ego 상태 수신과 ROS 내부 판단은 유지된다. 실제 제어 실행:

```bash
roslaunch morai_bringup final_ws_bringup.launch \
  enable_control:=true morai_host_ip:=192.168.0.151 \
  turn_signal_remote_port:=9097 \
  stopline_front_reference_offset_m:=3.845 stopline_hold_distance_m:=0.5
rostopic echo /control/maneuver_status
rostopic echo /control/turn_signal_state
```

MORAI Network Settings에 **TurnSignalLampControl 수신 UDP**의 IP/포트를 맞춘다.
패킷은 저장소의 `TurnSignalLampControl.py` 정의와 동일한 33바이트다.
네트워크 설정 위치는 [MORAI 공식 UDP 설정](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/udp)을 참고한다.
기존 lamp-only `turn_signal_controller.launch`는 독립 송신 실험용으로,
최종 통합 런치와 동시에 실행하면 안 된다. 그것만으로 조작 지연을 제어하지는 않는다.

검증은 ROS 대역/UDP 송신 mock 및 단순 종방향 차량 모델로 수행했다.
실제 ROS 빌드, MORAI 점등, 실차량 정지 오차와 경로 방향 판정은 이 Windows 환경에서
실행하지 않았다. 다른 모델/맵/차량 원점에 대한 자동 보정 기능은 없다.
