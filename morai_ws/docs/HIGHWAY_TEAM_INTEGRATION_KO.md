# dev/test_highway 통합 내역과 제어 연결

기준: AutoVehicle `3330360` + ROI `dev/test_highway`의
`13b740689c280d04fd456b00b854efbd793c38c2`.
원본 `2026_molit_comp_global_path.txt`는 입력 전용이며 이 통합에서 수정하지 않는다.

## 실행 구조

```text
Windows MORAI ── UDP ── Ubuntu Docker (Ubuntu 20.04 + ROS Noetic)
  GPS/IMU → UDP bridge → EKF → /localization/odometry
  Camera 1101 → real_lane → /perception/camera/lane_info_raw
                           → lane_info_contract
                             ├ /perception/camera/lane_info
                             ├ /detection/lane
                             ├ /perception/camera/stopline
                             └ 점선/실선/정지선 Bool·거리
  LiDAR 2001 → tracking → /perception/lidar/tracked_obstacles_map
                     └ 도로 방향 보정 → merge_gap

원본 경로 + LiDAR → Frenet 후보 생성 → bypass_lane_guard → avoidance_path_manager
                                     차선/공간 검사         경로 확정/복귀
                                                           ↓
차선 + 앞뒤 차량 + 진입 요청 → highway_lane_strategy → active_path / stop / target_speed_mps
                                                           ↓
                                  adaptive_curvature_purepursuit → /control/ctrl_cmd
카메라 정지선·신호 → stopline_controller                       → /control/stopline_cmd
주행경로 충돌 판단·센서 freshness·보행자/교차로 정지 → control_mux → /ctrl_cmd → UDP 9093
```

`/ctrl_cmd` 발행자는 control_mux 하나다. 팀원의 별도 Pure Pursuit와 현재 제어기를
동시에 실행하지 않는다. `enable_control:=false`일 때도 ROS 계산/진단 명령은 나오지만
UDP bridge의 차량 제어 송신은 꺼져 있다.

## 반영 파일

| 기능 | 파일 | 반영 방식 |
|---|---|---|
| 6클래스 차선 모델 | `camera_perception/post_processing/real_lane.py` | 팀원 알고리즘 그대로 |
| 추론·후처리 실행 | `post_processing/real_lane_node.py` | 팀원 알고리즘 + 기존 UDP 수신기·ROS IMU 연결 |
| 메시지 변환 | `scripts/lane_info_contract_node.py`, `src/camera_perception/lane_info_contract.py` | 신규 |
| 프레임 간 회전 보정 | `src/camera_perception/ros_lane_imu.py` | `/Imu` 공유, 추가 IMU UDP 수신기 없음 |
| 장애물 후보 | `purepursuit_mgeo/scripts/avoidance_frenet_debug_node.py` 및 `src/purepursuit_mgeo/frenet_*.py`, `trajectory_safety.py` | 팀원 Frenet/OBB 검사 |
| 차선 내 회피·옆 차로 확인 | `scripts/bypass_lane_guard_node.py` | 팀원 코드 + 실제 경계 확인 |
| 확정 경로 관리 | `scripts/avoidance_path_manager_node.py` | 팀원 경로 확정/동일 방향 갱신/복귀 |
| 고속도로 끼어들기 | `scripts/highway_lane_strategy_node.py` | 팀원 전략 + 명시 요청, 횟수 제한, 전 상태 충돌 검사 |
| 도로 방향 기준 차량 간격 | `lidar_perception/scripts/lidar_merge_gap_node.py` 및 `src/lidar_perception/lidar_merge_gap.py` | 팀원 최신 변경 + 차선 timestamp 검사 |
| 실제 곡률 제어 | `curvature_speed_purepursuit/scripts/adaptive_curvature_purepursuit_node.py` | 기존 곡률 preview/feedforward/PI를 동적 경로에 적용 |
| 최종 안전 결합 | `morai_sensor_fusion/scripts/roi_sensor_safety_adapter.py` | 경로 기반 stop을 최종 mux에 전달, 입력 지연은 계속 정지 |
| 전체 실행 | `morai_bringup/launch/final_ws_highway_bringup.launch` | 새 통합 launch |

모델은 `models/highway_best.pt`로 저장한다. 소스 저장소에 98 MB 가중치를 중복 추가하지
않고 Docker 빌드에서 정확한 팀원 커밋의 `best.pt`를 내려받아 SHA256을 확인한다.
SHA256: `0784b486d445480640df15893bf499a0e99aa05a58e859f03fa80393435a5b3b`.
기존 5클래스 `lane_seg_best.pt`와 서로 교체해서 사용하면 안 된다.

## 차량이 있을 때의 끼어들기

1. 평상시 원래 대회 경로/승인된 우회 경로를 추종한다. 고속도로 검출 Bool만으로
   선호 차로를 바꾸지 않는다. `/planning/highway_lane_change_request`가 필요하다.
2. 현재 차선의 왼쪽이 실제 관측된 점선인지 확인한다. 유도선 `from_guide`,
   관측 없이 예측한 `coasted`, 차선을 밟는 `straddling`만으로 변경을 허용하지 않는다.
   옆 차로의 바깥쪽 경계도 검출되어야 한다.
3. 목표 차로 앞차·뒤차의 현재 간격, 상대속도, 변경 완료 예상 시점의 간격과 TTC를 계산한다.
   기준값은 앞 6 m, 뒤 7 m, 시간 간격 1.5 s, TTC 3 s다. 차량 외곽을 고려한 간격이다.
   팀원 merge-gap 토픽과 경로 충돌 검사도 통과해야 한다.
4. 빈 공간이 부족하면 현재 경로를 유지하며 앞차에 맞춰 감속한다. 끼어들기 속도를
   순항속도에 억지로 맞추는 최소속도 비율은 통합 launch에서 0으로 설정했다.
5. 조건이 연속으로 충족되면 완만한 변경 경로를 확정한다. 도중에 카메라 좌/우 경계가
   바뀌어도 반대편으로 갑자기 새 경로를 만들지 않는다. 변경 중 접근하는 뒤차도
   예상 충돌 검사에서 제외하지 않는다.
6. 변경 완료 후 관측한 차선 중심을 따라간다. 원래 경로와 합류하는 구간에서 복귀한다.
   기본 고속도로 전략 차선변경은 한 번이며, 한 실행 안에서 자동으로 반복하지 않는다.

시험 요청은 Ubuntu Docker 안의 별도 터미널에서 **차로 변경을 허용한 구간에서만** 보낸다.

```bash
rostopic pub -r 5 /planning/highway_lane_change_request std_msgs/Bool 'data: true'
```

Ctrl+C로 중지하면 아직 경로를 확정하기 전 요청은 1초 후 만료된다. 이미 차선 변경을
시작했다면 경로를 갑자기 되돌리지 않고 충돌 검사와 완료/복귀 절차를 이어간다.
자동 대회 운용에서는 수동 명령 대신 검증한 경로 구간의 미션 노드가 이 요청을 발행해야 한다.
이 통합은 대회 고속도로 구간 좌표를 임의로 만들거나 MGeo의 주행 방향을 바꾸지 않는다.

## 장애물 회피와 차선 변경의 관계

작은 정적 장애물은 현재 차선 안의 가능한 우회 경로를 먼저 사용한다. 공간이 부족하면
점선과 옆 차로 경계 및 차량 간격을 확인해 옆 차로로 나갔다 원 경로로 돌아오는 후보를
검사한다. 안전한 후보가 없으면 정지한다. 움직이는 차량에 대한 충돌 검사와 앞차 추종은
고속도로 전략이 비활성인 일반 상태에서도 최종 active_path를 기준으로 수행한다.

`evaluation_speed_mps=2.0`은 저속으로 실행 가능한 후보의 기하학적 평가속도다.
실제 주행 속도는 각 active_path의 곡률 제한과 앞차 제한으로 다시 계산한다.
최고속도 30 km/h를 모든 우회 후보의 평가속도로 고정해 저속 회피까지 탈락시키지 않는다.

팀원의 기본 launch는 카메라 경계와 무관한 BYPASS를 허용했으나, 이 통합 launch는
`enforce_camera_lane_bounds_on_bypass:=true`가 기본이다. 차선 관측이 없거나 실선 밖으로
나가야 하는 경우 정지할 수 있다. 이는 도로 폴리곤이 없는 상태에서 주행 가능 공간을
가정하지 않기 위한 설정이다. 교차로 유도선만 있는 구간의 자유 회피는 추가 도로 경계
검증이 필요하다. 경계를 무시하는 옵션을 일반 해결책으로 사용하지 않는다.

기존 고정 전방 사각형의 LiDAR stop은 우회 경로가 안전해도 원 경로 앞 장애물을 보고
멈출 수 있었다. 새 launch에서는 경로별 충돌 판정이 이 역할을 맡는다. LiDAR/위치/경로
판정의 입력이 끊기면 여전히 최종 mux에서 정지한다.

## 좌표·속도·정지선

카메라 결과는 `base_link`, x 전방 m, y 왼쪽 m. 기존 LaneDetection의 계약에 맞춰
횡오차는 7 m 앞 중심선 y의 음수, 방향오차는 7~14 m 중심선의 방향으로 변환한다.
header.stamp는 Windows 촬영 시계나 추론 종료 시각 대신 Ubuntu의 프레임 수신 시각이다.
그래서 Windows/Ubuntu 시계 차이가 인식 freshness를 왜곡하지 않는다. 실제 촬영-수신
지연까지 보상하는 센서 시간 동기화가 구현된 것은 아니다.

정지선은 `covers_front`와 기하학적 지지도를 만족할 때 StopLineDetection으로 전달한다.
차선 신뢰도와 정지선 지지도는 따로 검사한다. `stopline_front_reference_offset_m` 기본
3.845 m는 기존 후륜 기준점 가정이므로 실제 MORAI 좌표 원점·장착값과 맞춰야 한다.
`lane/cam_set.json`은 캘리브레이션 파일이다. 그 안의 예전 UDP 주소를 실제 송수신
설정으로 사용하지 않고 launch의 IP/port를 사용한다. 카메라 위치/각도/FOV/해상도는
실제 MORAI Camera 1 설정과 반드시 일치시켜야 한다.

속도는 `/localization/odometry.twist.twist.linear`의 x/y 크기를 사용한다.
새 경로의 곡률로 공간 속도 프로파일을 다시 계산하며, 곡률 제한과 앞차 속도 제한 중
작은 값을 PI 제어에 전달한다. `/Ego_topic`은 이 제어기의 필수 입력이 아니다.
GPS blackout에서는 이 launch에 기존 blackout fallback이 포함되지 않아 위치 신뢰성
문제를 별도로 검증해야 한다. 이 launch를 blackout 시험 완료 상태로 간주하지 않는다.

경로는 CSV를 매 프레임 읽고 쓰는 방식이 아니다. 원본 TXT를 유지하고 `nav_msgs/Path`로
갱신한다. `/avoidance_path_manager/active_path`와 `/highway_lane_strategy/active_path`를
rosbag으로 기록하면 결정된 경로를 다시 확인할 수 있다.

## 테스트 범위

로컬에서는 ROS transport를 대체한 카메라/정지선/신호/blackout/곡률/회피/LiDAR
회귀 테스트를 실행한다. Docker 빌드에는 catkin 컴파일, beta_drive 메시지 필드 확인,
실제 roslaunch include 해석, 새 모델 실제 로드·빈 영상 추론 검사를 포함했다.
Windows 개발 환경에는 Docker Engine과 ROS가 없으므로 이 Docker 빌드 및 MORAI 실주행은
새 Ubuntu에서 아래 안내에 따라 실행해야 한다.

실행 절차: [두 PC + Docker 처음 설정](TWO_PC_DOCKER_HIGHWAY_KO.md).
