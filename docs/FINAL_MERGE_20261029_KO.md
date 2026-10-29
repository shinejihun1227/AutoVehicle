# 2026-10-29 최종 기능 병합

이 문서는 첫 병합 당시 기록이다. 현재 고속도로 노드와 Competition 차량 상태
9081 수신·실행 설정은 [후속 통합 안내](COMPETITION_OVERTAKE_WAIT5_20261029_KO.md)를 따른다.

기준: x010806-sudo/morai_ws_full main `b3e0747b0d9ad2a96f50da07a7dfee1f06d1f5c6`.
대상: shinejihun1227/AutoVehicle의 새 `morai_ws_full` 브랜치.
이 브랜치 루트가 catkin workspace다. 기존 브랜치의 `morai_ws/` 하위 구조와 다르다.
final_ws/jihun_ws/merge_ws와 원본 저장소는 변경하지 않는다.

## 첨부 패치의 실제 연결

- v9_priority_overlay_v61.zip의 highway_lane_strategy_node.py를 사용한다.
  기존 v61 wrapper가 uploaded_highway_lane_strategy_node.py를 로드해 교체 파일을
  실행하지 않던 점을 수정했다. 일반 회피 중 고속도로 신규 활성 보류는 유지한다.
  상태 JSON에 timestamp를 추가했다. 이것이 없으면 회피 노드가 고속도로 상태를
  미수신으로 처리하여 계속 정지할 수 있다.
- morai_dynamic_wait5_tunnel_patch.zip의 6개 runtime 파일을 병합했다.
  기본 저장소에 빠져 있던 route_avoidance_gate.py를 별도로 구현하고,
  avoidance_regions.yaml을 회피기와 최종 신호 제어기 양쪽에 로드한다.
  경로 해시는 좌표 소수점 6자리의 정규화된 순서에 대해 다시 계산한다.
  변경된 경로나 역순 경로는 동일 미션 구간으로 자동 승인하지 않는다.
- 해당 패치의 지도 좌표와 현재 경로를 대조했다. 정적 구간 약 271.9~384.3m,
  동적 구간 약 407.3~488.3m다. 이 구간은 이번 첨부 설정이며 임의 생성한 것이 아니다.
- 동적 장애물 지도 속도 2m/s 이상, 경로 차단 시 대기한다. 실제 속도 0.2m/s 이하로
  연속 5초 정차한 후 회피 요청한다. 최신 실행 경로와 일치하는 유효 우회 경로가
  확인되기 전에는 출발하지 않는다. 추적 ID 소실은 통과 증거로 쓰지 않는다.
- EKF 거리 배율 학습 모듈을 포함했다. 허용된 GPS와 속도/IMU 적분을 비교하며,
  Ego 위치 진단값은 배율 학습 입력으로 쓰지 않는다.

## 신호와 정차

- 사용자 확인 기준: **앞범퍼가 정지선 7m 앞에서 정차**.
  stopline_hold_distance_m=7.0, front_reference_offset_m=3.845를 전달한다.
  3.845는 기존 뒤차축 기준 차량 모델을 유지한 값이다. 실제 제어 원점이 다르면 재보정한다.
- 신호 가중치 best0917.pt와 기존 경로별 신호 그룹, 적색 즉시 제한,
  같은 교차로 진입 확정 후 재정지 방지 코어는 보존했다.
- 최초 녹색도 출발 가능하다. 원본 영상 시각이 다른 2개 이상 프레임과
  0.15초 연속 허용을 요구하며 적색→녹색 전이는 요구하지 않는다.
- 이미지/위치 대응 허용 간격을 실행 설정에서 50ms→150ms로 완화하고,
  지도 투영에 정밀 대응이 없을 때만 영상 여유를 25px→40px로 늘렸다. 다른 교차로/반대편 신호와
  충돌하는 후보는 여전히 거절한다. 화면 전체 최고 confidence를 선택하지 않는다.
- 다음 이미지가 위치 대응을 기다리는 최초 100ms 동안은 아직 유효한 직전
  확인 결과를 매 tick 취소하지 않는다. 새 적색/황색·모호한 대응·시간 초과는 제한한다.
- 이미 해당 교차로와 대응되어 확인한 녹색의 검출 소실을 다른 교차로의 적색이
  취소하지 않게 했다. 동일 교차로의 허가는 기존 시간/거리 한계 내에서만 유지한다.
  다음 교차로와 회차 시 초기화하며 장애물/위치/통신 정지는 그대로 적용한다.

## 좁은 차선과 속도

- 일반 adaptive 제어기가 읽지 않던 enable_lane_centering과 관련 설정을 연결했다.
  최신 CAM1의 실제 관측 양쪽 차선과 원래 영상 시각에 대응하는 차량 위치를 사용한다.
  실행 경로의 정상 커브 모양을 빼고 잔여 편차만 제한된 조향 보정으로 적용한다.
- 정상 차선 폭 하한 2.7m에서 좁아지는 관측을 거절하던 부분을 2.3m로 조정했다.
  양쪽 실측·신뢰도·형상 일관성 검사는 유지하고 보정량/변화율을 제한한다.
  관측 폭 2.8m 미만인 구간은 일반 속도 상승보다 25km/h 상한을 우선한다.
- 의도적 차선 변경 및 원래 경로에서 벗어난 우회 경로에서는 일반 차선 보정을
  적용하지 않는다. 고속도로 INNER_HOLD는 첨부 V9의 자체 차선 중심 처리를 유지한다.
- 전체 통합 launch의 일반 최고속도는 43.2→53.2km/h다. 기존 곡률 속도에
  최대 10km/h를 더하되 횡가속도 2m/s², 가감속 가능 거리, 다른 미션 제한을 넘지 않는다.
  정지·동적 구간 20km/h·회전교차로 설정은 10km/h씩 가산하지 않는다.
  실제 속도가 모든 지점에서 정확히 10km/h 증가한다는 뜻은 아니다.
- adaptive가 loop_route를 실제로 읽고 종점 정지와 시작점 감속을 해제한다.
  연결된 폐경로의 전방 목표점은 seam을 넘어 다음 회차로 이어진다.
- 경로·정지·속도를 하나의 trajectory 메시지로 전달하고, 동일 형상 재수신은
  곡률을 매번 다시 계산하지 않는다. 잘못된 새 경로는 이전 허가를 취소한다.

## 실행

기존에 ROS Noetic과 모델 의존성이 설치된 Ubuntu/컨테이너 안의 **별도 새 폴더**에서:

```bash
git clone --single-branch --branch morai_ws_full https://github.com/shinejihun1227/AutoVehicle.git ~/morai_ws_full
cd ~/morai_ws_full
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
bash RUN_INTEGRATED.sh monitor
```

주행 실행은 monitor를 종료한 다음 같은 환경에서:

```bash
bash RUN_INTEGRATED.sh drive
```

직접 실행하는 런치는 `morai_bringup integrated_obstacle_v61.launch`다.
개인 시험 MORAI IP 기본값은 192.168.10.14이며 Ubuntu는 기존 192.168.10.21 환경을 사용한다.
GPS 3001 / IMU 4001 / LiDAR 2001 / CAM1 1101 / CAM4 1131,
차량 상태 1911 / 제어 목적지 9093 / 제어 송신 9094는 현재 네트워크와 대조한다.
예: `bash RUN_INTEGRATED.sh drive max_speed_kph:=35.0`으로 상한을 낮출 수 있다.
기존 `docker/final_ws/run_test.sh drive 2`는 새 통합 실행기가 아니다.
`RUN_INTEGRATED_NO_OBSTACLE.sh`는 회피 제외 진단용 7.2km/h 기본값을 유지한다.

## 확인 범위와 남은 현장 검증

`python3 integration_tests/run_offline.py`가 변경 동작, 패치, 신호/정지 코어,
경로 제어 테스트와 Python/XML 검사를 수행하고 test-results에 전체 결과를 저장한다.
이는 ROS 통신·catkin 빌드·영상 추론·실차/시뮬레이터 주행 검증을 대신하지 않는다.

현장에서는 정차 오차, 녹색 확인→브레이크 해제 지연, 좁은 차선 여유,
V9 차선 변경/복귀, 5초 대기 후 회피 및 터널 복구를 확인한다.
V9는 첨부본 요청대로 적용했으며 기존 v61의 모든 충돌 예측 세부 검사가 동일하지는 않다.

본선 규정 기준은 v1.2/14개 체크포인트다. 원본에 남은 v1.1 평가기와 일반 Ego
181바이트 입력은 이번 기능 병합만으로 본선 호환이 되지 않는다. Competition Vehicle
Status의 실제 전달 규격·포트 확인은 별도 미해결 항목이다. 출처가 없는 패킷 형식을
임의 생성하거나 금지된 위치/신호 입력으로 대체하지 않았다.
