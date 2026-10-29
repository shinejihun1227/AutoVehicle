# 고속도로 교체·완전판 회피·Competition 상태 통합

2026-10-29 작업. 브랜치는 기존 `morai_ws_full`이며 `final_ws`, `jihun_ws`,
`merge_ws`는 변경하지 않았다. GitHub에 추가돼 있던 `test1029`도 보존했다.

## 반영한 원본

- 고속도로: whddbs3061/car의 `highway-overtake-v61`,
  `5cafadecc1365aaa32e7e3705ab4c79b6eef2bb2`.
  기본 플래너는 `highway_overtake_node.py` + `highway_overtake.py`다.
  원본의 별도 escape 플래너도 포함하지만 기본으로 활성화하지 않는다.
- 회피: 사용자 첨부 `morai_wait5_tunnel_complete.zip`,
  SHA256 `cf887ca4b47923f606f5db0f5ec4b578993df918b490a00a88eba6312a977a0f`.
  8개 런타임 파일을 현재 코드와 대조했다. 기존 wait5·동적 guard·EKF 배율은
  이미 같은 버전이었다. 누락 gate의 완전판과 이진 좌표 fingerprint를 반영했다.
  기존의 신선한 장애물 입력 검사, 신호 개선, 짧은 제어 공백 보완은 유지했다.
  ZIP 전체 launch/fusion을 덮어써 과거 0.5m·3m 정차 설정으로 되돌리지 않았다.

## 주행 연결

`integrated_obstacle_v61.launch`가 다음을 한 번에 실행한다.

```text
GPS/IMU → EKF 위치
Competition UDP 9081 → 차량 전후 속도(m/s)
LiDAR → wait5/터널 회피 → 고속도로 overtake → adaptive 경로 제어
adaptive/회전교차로 중재 → 카메라 신호·정지선 최종 제어 → CtrlCmd UDP
```

외부 CtrlCmd 송신 브리지는 하나다. 회피 플래너는 직접 차량 명령을 송신하지 않는다.
경로·정지·속도는 기존 atomic trajectory로 전달한다. 새 고속도로 코드의 비활성
구간에서는 회피 경로가 통과하며, 고속도로가 활성화되면 일반 회피의 새 진입을 보류한다.
라이다/위치의 원본 시각도 검사한다. 동일 경로라도 고속도로 진입·이탈·구간 신호 만료 때
속도 프로파일을 다시 적용해 캐시된 이전 제한이 남지 않게 했다.

기존 요구 유지:

- 앞범퍼 기준 선택된 정지선 **7m 전 정차**.
- 경로별 신호 그룹, 외형 필터·신뢰도 비교, 확인된 진입 후 신호 소실 재정지 방지.
- 좁은 차선 보정과 25km/h 상한, 곡률/횡가속도 제한.
- 짧은 일반 명령 공백은 추가 최대 0.2초만 이어가며 최종 신호·장애물 제동이 우선.
- 일반 최고속도 53.2km/h. 고속도로 순항 54, 기본 상한 **59km/h**.
  원본 별도 실행본의 79.2km/h 기본값을 그대로 올리지 않았다.
  고속도로 경계는 원본 좌표(s 약 1160.24~1590.25m)와 현재 경로 fingerprint로 확인한다.

회피는 새 동적 대상(지도 속도 2m/s 이상)이 경로를 막으면 정차하고,
실제 속도 0.2m/s 이하에서 연속 5초를 채운 뒤 기존 정적 회피 플래너에 요청한다.
유효하고 최신인 실행 우회 경로가 확인되어야 출발하며, 5초 뒤 무조건 출발하지 않는다.
터널 및 GPS 음영 회피, 상위 정지·회전교차로 우선권도 유지한다.
완전판 gate의 진행도는 노드 실행 동안 한 방향으로 증가한다. 씬 리셋/순간이동 시
런치를 재시작해야 하며, 모든 미션의 다회차 재활성화는 검증하지 않았다.

## Competition Vehicle Status 수신

수정한 브리지 파일은 `morai_udp_drive_bridge.launch`,
`morai_udp_drive_bridge_node.py`, `protocol.py`다. 상위 launch도 전달 인자를 변경했다.

- 기본 입력 `status_protocol:=competition`, Ubuntu 목적지 **9081**.
- 팀의 기존 `path_planning/morai_udp_competition_status.py`와 보관된 NetworkModule
  packed 구조체를 대조해 `#MoraiInfo$` 181/229바이트 형식을 지원한다.
  선언 데이터 길이는 각각 152/200이며 실제 datagram 전체를 받은 후 헤더·길이·CRLF·
  timestamp·사용 필드의 유한성을 검사한다. 다른 형식은 추측해서 읽지 않고 폐기한다.
- 229바이트 패킷을 181바이트로 잘라 받던 수신 문제를 제거했다.
- 같은 원본 시각·역순 패킷은 수신 시간을 새로 붙여 반복 발행하지 않는다.
  씬 리셋 후 원본 시각이 되돌아가면 런치를 재시작한다.
- 허용된 전후 방향 속도 `velocity.x`를 km/h→m/s로 한 번 변환한다.
  속도 제어·EKF 속도 보조·신호 제어는 이 값의 크기를 읽는다.
- 제공되지 않는 xyz 위치·xyz 가속도·y/z 속도·타이어 정보·지도 링크 정보는
  Competition parser의 측정값에 포함하지 않는다. 호환 ROS `/Ego_topic`의 해당
  벡터 항목은 NaN이며 측정된 0으로 쓰지 않는다. 위치는 GPS/IMU EKF에서 얻는다.
- 내부 ROS 메시지 이름은 호환을 위해 `morai_msgs/EgoVehicleStatus`를 유지한다.
  이것은 외부 일반 Ego Vehicle Status를 구독한다는 뜻이 아니다.
- 진단 토픽 `/morai_udp_drive_bridge/status_metadata`에 원본/수신 시각·형식·포트·
  속도 단위·AV 모드/기어와 필드 가용성을 게시한다.

`9081`은 이번 사용자가 지정한 Ubuntu 수신 포트다. 본선 매뉴얼의 Windows
loopback 9088→9099와 동일한 포트라고 간주하지 않는다. 판정 프로그램을 경유하는
본선에서는 실제 전달 목적지를 확인해야 한다. 실제 대회 패킷 캡처는 아직 검증하지
못했으며, 이 변경만으로 전체 대회 규정 준수·완주를 보장하지 않는다.

## 기존 Docker 환경에서 업데이트 및 실행

호스트에서 기존 컨테이너 shell로 들어간 뒤, 새 통합 저장소가 있는
컨테이너 터미널에서 실행한다. `/root/morai_ws_full`은 새 브랜치를 받은 폴더다.

```bash
cd /root/morai_ws_full
git pull --ff-only origin morai_ws_full
source /opt/AutoVehicle/morai_ws/devel/setup.bash
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/opt/morai-venv/bin/python3
```

빌드 성공 후 같은 터미널에서:

```bash
source /root/morai_ws_full/devel/setup.bash
bash /root/morai_ws_full/RUN_INTEGRATED.sh drive \
  morai_host_ip:=192.168.10.14 ego_status_port:=9081 status_protocol:=competition
```

직접 실행하는 통합 런치 이름은 그대로 `morai_bringup integrated_obstacle_v61.launch`다.
관측만 할 때는 `drive` 대신 `monitor`를 사용하며 둘을 동시에 실행하지 않는다.
카메라 대시보드는 Ubuntu 브라우저 `http://127.0.0.1:8765`다.

개인 테스트 MORAI Network Settings에서 **Competition Vehicle Status**를 선택하고,
Destination을 **192.168.10.21:9081**로 맞춘다. CtrlCmd는 Ubuntu 9094→MORAI 9093,
GPS 3001 / IMU 4001 / LiDAR 2001 / CAM1 1101 / CAM4 1131을 유지한다.
신호색은 계속 카메라로 인식한다.

다른 터미널에서 같은 새 workspace를 source한 뒤 상태 수신을 확인할 수 있다.

```bash
rostopic echo /morai_udp_drive_bridge/status_metadata
```

선택 인자:

- `max_speed_kph:=35.0`: 일반 구간 상한. 고속도로도 낮추려면
  `highway_cruise_kph:=35.0 highway_max_speed_kph:=35.0`를 함께 전달한다.
- `highway_node_type:=highway_overtake_escape_node.py`: 원본 저장소의 별도
  앞차 회피 variant. 기본 안정판과 다른 간격/감속 정책이므로 별도 실주행 확인이 필요하다.

## 검증 범위

`python3 integration_tests/run_offline.py`가 기존 신호/정지·짧은 제어 공백,
완전판 회피 40개, 원본 wait5 47개, 고속도로 순수 플래너/모의 주행/ROS 경계,
Competition 양쪽 패킷 형식과 단위·누락 필드·중복 시각·건전한 입력 중단을 검사한다.
고속도로의 원본 파이썬 모의 주행은 실제 MORAI 주행이 아니다.
이번 환경에서는 ROS/catkin 빌드·실제 UDP 왕복·모델 추론·폐루프 주행은 실행하지 못했다.
최종 테스트 수와 출처/파일 해시는 `COMPETITION_OVERTAKE_WAIT5_PROVENANCE.json`에 기록한다.

최종 검증: **529개 통과**, Python 3.8 문법 검사 및 통합 launch 인자 연결 검사 통과.
