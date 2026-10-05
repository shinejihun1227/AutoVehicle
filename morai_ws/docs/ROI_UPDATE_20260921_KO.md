# ROI 코드 업데이트 — 2026-09-21

ROI `dev/merged_code`를 확인한 최신 커밋은
[`cb41b5fa4598d680d715cde9a0deaf6a5a1035f3`](https://github.com/hyunho0429/ROI/commit/cb41b5fa4598d680d715cde9a0deaf6a5a1035f3)이다.
커밋 시각은 2026-09-14 21:17:56 KST이다. 최초 센서 통합 기준 `c752050`부터의
변경 파일 14개를 검토하여 현재 workspace의 패키지·제어 계약에 맞게 반영했다.
원본 파일별 처리 내역과 모델 해시는 [roi_upstream_manifest.json](../config/roi_upstream_manifest.json)에 기록했다.

## 반영 내용

- 최신 Frenet 우회, 경로 관리자, 우회 전달, 고속도로 차선 전략, 회전교차로 진입 판단 코드를 반영했다.
  공통 경로·충돌 검사 코드는 catkin Python 패키지 내부에 배치했다.
- 정적 우회 시작 조건에 장애물 속도 `0.60 m/s` 이하 조건이 추가됐다. 움직이는 장애물의
  레코드와 후보 충돌 검사는 유지된다. LiDAR 안전 정지와 고속도로 추종 제어도 별도로 동작한다.
- 고속도로 차선 중심은 최근 5프레임의 중앙값·저역 필터·프레임별 이동량 제한을 사용한다.
  차선 변경 도중 중심 필터를 고정하고, 유한 경로의 끝에 도달하기 전에 차선 유지로 전환한다.
  한쪽 경계가 없어도 유효한 중심선이면 차선 유지를 허용하되 새 차선 변경에는 엄격한 검사를 적용한다.
  인식 소실 시 최신 원본의 제한된 복구 구간을 사용하고 시간이 지나면 정지한다.
- 카메라 `live_overlay.py` 한 프로세스가 기존 정지선·차선 메시지와 새 `/perception/camera/lane_info`를
  함께 발행한다. UDP 1101 수신과 모델 추론을 중복 실행하지 않는다. 기존 timestamped 메시지를 유지한다.
  `HELD` 차선은 원래 관측 시각을 유지하고, 주행 노드는 지연·중복·미래 시각 관측을 거부한다.
- 원본의 새 `purepursuit_mgeo_node.py` 기능은 현재 `adaptive_curvature_purepursuit_node.py`에 연결했다.
  곡률 속도 제한, km/h PI, `longlCmdType=1`, 단일 nominal 명령 발행을 유지한다.
  별도 `purepursuit_mgeo_node2.py`는 구형 제어기의 중복이므로 설치하지 않는다.
- 새 고속도로 launch의 순항 속도는 `max_speed_kph / 3.6`에서 정한다. 상한만 올려도 원본의
  별도 2 m/s 제한이 남지 않는다. 차선 유지·앞차·곡률·복구 제한은 계속 적용된다.
  정지 대기 중에는 가속 목표를 0으로 초기화하여 재출발 시 시간에 따라 다시 증가시킨다.
- `lane_info_runner.py`와 `lane_info_semantic_adapter.py`는 원본 호환용으로 설치하되 통합 launch에서는
  시작하지 않는다. `lane_camera`와 같은 포트로 추가 실행하면 안 된다.

모델은 교체하지 않았다. `lane_seg_best.pt`, `best0902.pt`, `yolov8n.pt`,
`lane_segmentation.onnx` 네 파일 모두 최신 원본과 Git blob 해시가 동일하다.
이번 upstream 변경은 새로 학습한 가중치가 아니라 주행 로직 변경이다.

Cam4 설정 `x=3.43, y=0.01, z=0.61`, `roll/pitch/yaw=0`, `640×480`, 수평 FOV 90°는 유지한다.
`signal_camera.calibrated=false`도 유지한다. 좌표값을 입력한 것만으로 영상 투영까지 검증된 것은 아니며,
해당 값이 false인 동안 경로별 신호 연결을 사용하는 기본 구성의 교차로 통행 허가는 제한된다.

## 빌드와 기존 신호등·정지선 시험

이 수정이 적용된 `morai_ws`를 Ubuntu ROS Noetic 환경에서 사용한다.
현재 수정은 로컬 workspace에 있으며 원격 GitHub 또는 별도 Ubuntu 복사본으로 자동 전송되지 않는다.
다른 PC에서는 변경 파일을 옮긴 후 빌드해야 한다. 기존 실행 중인 launch를 종료하고 하나만 실행한다.

```bash
cd ~/AutoVehicle/morai_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -B docker/final_ws/run_regression.py
```

처음 요청한 **곡률 기반 주행 + 경로별 신호등·정지선 정지**는 기존 진입점을 사용한다.
다음 명령은 제어 송신 없이 센서와 판단 상태를 확인한다.

```bash
roslaunch morai_bringup final_ws_native_no_lamps.launch \
  workspace_path:="$PWD" morai_host_ip:=192.168.0.161 ego_status_port:=1911 \
  enable_control:=false max_speed_kph:=7.2
```

MORAI 저속 주행 시험에서는 위 명령의 `enable_control:=true`로 실행한다.
경로가 다른 곳에 있다면 `workspace_path`를 실제 `morai_ws` 절대 경로로 지정한다.
정상 주행 상한은 `max_speed_kph`로 바꾸고, GPS 소실 시 차선 fallback 제한과 구분한다.
설정 상한에 도달한다는 의미는 아니다. 곡률·신호·정지선·센서 상태에 따라 목표 속도가 낮아질 수 있다.

## 최신 고속도로 기능 별도 시험

```bash
roslaunch morai_bringup final_ws_highway_bringup.launch \
  workspace_path:="$PWD" morai_host_ip:=192.168.0.161 ego_status_port:=1911 \
  enable_control:=false max_speed_kph:=7.2
```

이 구성은 최신 차선 변경·차선 유지·정적 우회 알고리즘 시험용이다. 기본값은 제어 송신 OFF이고,
주행 시험은 `enable_control:=true`로 켠다. 원본 이름인
`roslaunch purepursuit_mgeo morai_avoidance_highway_roundabout_final.launch ...`도 이 구성으로 연결된다.

연결은 `Frenet → bypass guard → path manager → highway strategy → adaptive curvature PP →
stopline controller → control_mux → UDP`이다. 차선 변경은 카메라 환경 판단 또는 mission request와
차선 경계·LiDAR 합류 간격 조건을 사용한다. 회전교차로는 `/planning/merge_request` 입력이 있어야
진입 판단을 시작한다. 미션 위치를 자동 생성하지 않는다.

이 별도 구성에는 timestamped 정지선 제어와 LiDAR·보행자·교차로 안전 정지가 연결되지만,
고정 전역 경로를 전제로 한 `maneuver_fusion` 및 GPS blackout fallback은 시작하지 않는다.
따라서 기본 구성의 **방향별 신호 연결·회전 허가 기능까지 통합 검증된 대체 실행 파일이 아니다.**
회전교차로 GO 판단으로 기존 교차로 안전 정지를 자동 해제하지도 않는다.
고속도로 경로와 교차로 시나리오를 섞은 전체 코스 주행에는 기존 진입점의 검증 절차를 먼저 따른다.

## 속도가 안 오르거나 출발하지 않을 때

```bash
rostopic echo /experimental/curvature_speed_limit
rostopic echo /experimental/curvature_speed_command
rostopic echo /control/stopline_status
rostopic echo /detection/fused_safety_stop
```

고속도로 구성에서는 `/highway_lane_strategy/state`, `/highway_lane_strategy/target_speed_mps`,
`/highway_lane_strategy/stop_required`, `/roundabout_merge_gate/status`도 확인한다.
목표 속도 자체가 낮은지, 정지 요청이 있는지, 센서 수신이 끊겼는지를 분리해서 확인한다.
기존 출발점의 0 속도 프로파일로 출발하지 못하던 수정은 유지했고 회귀 시험도 통과했다.
모든 출발 불가 원인을 해결했다는 뜻은 아니다.

## 확인 결과

Python 오프라인 테스트 **398개 통과**: 카메라 69, 곡률 20, 경로 계획 14, 정지선 54,
신호·회전 융합 200, GPS fallback 41. 모델 추론과 ROS I/O는 테스트 경계에서 대체한다.
실제 차선 안정화 수학 코드, 경로 계산, PI, 정지/복구 조건 및 launch 인자 연결은 실행해 확인했다.
4개 모델의 해시도 upstream과 대조했다.

현재 Windows 환경에서는 ROS catkin 빌드, GPU 모델 추론, MORAI 실제 주행을 수행하지 않았다.
따라서 속도 상승·정지 위치·차선 변경 성공 여부는 Ubuntu/MORAI 시험 결과로 최종 확인해야 한다.
