# ji_ujin_merge 실행과 구성

`new_final_ws`의 `9b6b31f`를 기준으로 만든 브랜치입니다. 기존 브랜치는 변경하지 않았습니다.
신호등·곡률 속도 계획·종방향 PID·최종 명령 중재·UDP 송신은 기존 것을 사용합니다.
차량 상태는 Competition Vehicle Status만 수신합니다. GPS/IMU 센서는 계속 사용합니다.
`/Ego_topic`과 `morai_msgs/EgoVehicleStatus`는 내부 호환 이름이며 일반 Ego 상태 수신을 의미하지 않습니다.
Competition에서 제공하지 않는 위치·가속도·횡속도는 사용하지 않습니다.

## Ubuntu에서 처음 준비

기존 주행 터미널은 Ctrl+C로 종료합니다. 이전 컨테이너를 동시에 실행해 센서 포트를 점유하지 않게 합니다.
새 폴더에서 실행하므로 기존 `new_final_ws` 폴더와 설정은 보존됩니다.

```bash
git clone --single-branch --branch ji_ujin_merge https://github.com/shinejihun1227/AutoVehicle.git ~/AutoVehicle_ji_ujin_merge
cd ~/AutoVehicle_ji_ujin_merge/morai_ws/docker/final_ws
bash prepare_ji_ujin.sh "$HOME/AutoVehicle_new_final_ws/morai_ws/docker/final_ws"
```

준비 명령은 기존 `highway.env`, `highway-test.env`를 새 폴더에 복사하고 백업합니다.
IP·Competition 포트·최고속도·정지선 정차 여유 등은 유지합니다.
컨테이너/이미지 이름은 `morai-ji-ujin-net10` / `morai-ji-ujin:latest`로 분리하고,
CAM1 장치를 `auto`, 두 신규 기능을 켬으로 설정합니다. 준비만 하며 주행하지 않습니다.
기존 폴더 이름이 다르면 위 첫 인자를 실제 폴더로 바꾸세요.

```bash
grep -E '^(UBUNTU_IP|MORAI_IP|CONTAINER_NAME|IMAGE_NAME|TORCH_FLAVOR)=' highway.env
grep -E '^(COMPETITION_STATUS_PORT|GPS_PORT|IMU_PORT|LANE_INFO_PORT|YOLO_PORT|ROI_LIDAR_PORT|LIDAR_.*|MAX_SPEED_KPH|STOPLINE_HOLD_DISTANCE_M)=' highway-test.env
```

개인 IP는 Ubuntu `192.168.10.21`, Windows `192.168.10.14`입니다.
Competition 수신 포트는 **기존 설정과 MORAI Destination Port가 같아야 합니다**.
기존 개인 설정이 `9099`이면 유지하세요. 예제 기본값은 `9081`이므로 예제에서 시작한 경우 반드시 맞춥니다.
Host Port와 Destination Port를 서로 바꾸지 않습니다. 코드 수신 포트는 Ubuntu의 Destination Port입니다.
GPS 3001, IMU 4001, CAM1 1101, CAM4 1131, LiDAR 2001 역시 양쪽 설정이 같아야 합니다.
Windows에는 일반 Ego 상태 대신 Competition Vehicle Status를 켭니다.

LiDAR 위치·회전은 현재 시뮬레이터 센서 설정과 대조하세요. 전달했던 BDT 설정과 동일하게
`x=1.22, y=0, z=1.23, yaw=0`을 사용하는 경우에만 다음 준비 명령을 사용합니다.

```bash
bash prepare_ji_ujin.sh "$HOME/AutoVehicle_new_final_ws/morai_ws/docker/final_ws" --lidar-bdt
```

`TORCH_FLAVOR=cpu`도 실행할 수 있습니다. 기존 NVIDIA 설정이 `cu121`이면 준비 명령이 유지합니다.
CAM1 `auto`는 사용 가능한 장치를 선택합니다. ROS를 Ubuntu 호스트에 별도로 설치할 필요는 없습니다.

## 처음 이미지 만들고 실행

새 LiDAR 메시지/실행 노드가 확실히 등록되도록 이 브랜치의 이미지를 한 번 빌드합니다.
`run_highway.sh start`는 이미지 빌드 명령이 아닙니다.

```bash
cd ~/AutoVehicle_ji_ujin_merge/morai_ws/docker/final_ws
python3 fetch_highway_model.py
bash run_highway.sh build
bash run_highway.sh start
bash run_highway.sh stop
bash install_curvature_signal.sh --config
bash run_highway.sh start
bash run_test.sh show 2
bash run_test.sh monitor 2
```

`monitor 2`는 차량 제어를 보내지 않으므로 AV 전환/출발하지 않습니다.
영상과 GPS·IMU·Competition·LiDAR 입력을 확인한 후 Ctrl+C로 종료하고 주행합니다.

```bash
bash run_test.sh drive 2
```

통합 런치는 기존 이름인 `morai_bringup/final_ws_curvature_signal.launch`입니다.
`test/roundabout`의 컨테이너나 `morai_ws_full`의 RUN_INTEGRATED.sh를 사용하지 않습니다.
브라우저는 Ubuntu에서 `http://127.0.0.1:8765/`입니다.

새 터미널에서 상태를 확인할 수 있습니다.

```bash
cd ~/AutoVehicle_ji_ujin_merge/morai_ws/docker/final_ws
bash run_test.sh diagnose 2
```

일반적인 다음 실행은 `bash run_highway.sh start` 후 `bash run_test.sh drive 2`입니다.
코드 업데이트는 주행 종료 → git pull → 컨테이너 stop → 설치 → start 순서입니다.

```bash
cd ~/AutoVehicle_ji_ujin_merge
git pull --ff-only origin ji_ujin_merge
cd morai_ws/docker/final_ws
bash run_highway.sh stop
bash install_curvature_signal.sh --config
bash run_highway.sh start
bash run_test.sh drive 2
```

## 터널: GPS 음영에서 카메라 우선

기존 EKF의 `IMU_COMPETITION_DEAD_RECKONING` 상태일 때만 작동합니다.
양쪽 실제 차선의 신뢰도·관측 시각·연속 프레임을 확인하고,
카메라 중앙선의 7~14m 앞 형상으로 조향을 계산합니다. 지도 경로가 직진이어도 관측 차선을 따릅니다.
최소 4프레임/0.2초를 확인하고 약 0.5초 동안 카메라 비중을 올립니다.
조향각·조향 변화율 제한과 기존 종방향 제어는 유지합니다.
GPS 복귀 시 약 1초에 걸쳐 지도 조향으로 돌아옵니다.

`highway-test.env`의 설정은 다음과 같습니다.

```makefile
TUNNEL_LANE_PRIORITY_ENABLED=true
TUNNEL_MAX_SPEED_KPH=25.0
```

터널 25km/h는 상한이며 기존 곡률/신호/전체 최고속도보다 높여서 강제하지 않습니다.
차선 확보 중에는 최대 7.2km/h로 제한하고, 1초 이상 차선을 확보하지 못하면 정지합니다.
CAM1이 죽어 있으면 터널에서 계속 직진하도록 우회하지 않습니다.
추가 진단 토픽 `/control/tunnel_lane_status`에서 `tunnel_camera_priority`, `camera_weight`,
`lane_valid`, `blackout`을 확인할 수 있습니다.

위치는 기존 GPS 초기화 + IMU + Competition 전방속도 EKF입니다.
카메라는 조향을 보정하며, EKF 위치 오차 자체를 지도에 재정합하는 기능은 아닙니다.
초기 GPS 없이 임의 지도 위치에서 시작하거나 음영에서 누적 오차가 없다고 보장하지 않습니다.

## 회전교차로: 유진 V6.1 판단 + 기존 제어

원본: [ohdang23/yujin_morai V6.1](https://github.com/ohdang23/yujin_morai/tree/6c7d805c993f8f2f0d73362de1898206401e0b05/roundabout_v6_1/roundabout_v6_1).
원본 `core.py`, `entry_decision.py`, `prediction.py`와 패키지 초기화 파일만 가져왔습니다.
원본 validation·단독 UDP 드라이버·별도 PID·시험 런치·기록기는 가져오지 않았습니다.

LiDAR → 기존 장애물 추적/지도 좌표 변환 → V6.1 차량 예측·진입/양보·선행차 판단 →
기존 곡률 제어기의 목표속도 제한 → 기존 신호등 중재 → 기존 UDP 송신 순서입니다.
`yujin_roundabout_gate_node.py`는 판단만 게시하고 가속·브레이크·조향을 직접 송신하지 않습니다.
회전 경로는 기존 대회 전역 경로를 따르며 임의 회피 경로를 새로 생성하지 않습니다.

적용 구간은 `config/roundabout_v61.yaml`의 경로 인덱스 1720~1900입니다.
원본 V6.1 진입 지점과 현재 전역 경로를 대조했으며,
기존 대회 MGeo `A2256W000134 → 135 → 132 → 133` 폐곡선에서 순환 차량 예측 경로를 생성했습니다.
모든 커브에서 켜는 기능이 아닙니다. 구간 밖에서는 판단 입력이 없어도 일반 주행을 제한하지 않습니다.
경로 파일 해시가 달라지면 임의로 같은 인덱스를 적용하지 않고 설정 갱신을 요구합니다.

V6.1의 진입 전 1회 정지, 충돌 시간/빈 공간 확인, 차량 예측, 선행차 간격 제어를 유지했습니다.
속도 단위는 설정 파일에서 **m/s**입니다. 현재 원본값은 접근 2.0, 합류 2.8,
순환 2.0m/s이며 절대 상한 3.4m/s입니다. 기존 제어기의 더 낮은 제한은 항상 우선합니다.
이 브랜치에서 코스 전체 속도나 기존 포트/PID를 V6.1 시험 드라이버 값으로 교체하지 않았습니다.
회전교차로 범위 안에서 LiDAR·추적·판단이 끊기면 정지합니다.
정상 수신된 차량 0개 프레임과 LiDAR 미수신을 구분합니다.
`/roundabout_v61/status`, `/control/roundabout_overlay_status`가 진단에 추가되었습니다.

```makefile
ROUNDABOUT_ENABLED=true
```

`false`로 바꾸고 재실행하면 회전교차로 판단과 신규 LiDAR 수신을 끌 수 있습니다.
같은 2001 포트를 별도 LiDAR 노드가 동시에 수신하지 않도록 합니다.
일반 도로 장애물 회피/고속도로 기능을 다른 브랜치에서 추가로 가져온 것은 아닙니다.

## 검증 범위

오프라인 검사에는 Competition 입력 계약, 기존 제어 보존, 센서 누락, 실제 메인 제어기 연결,
MGeo 범위 제한, 원본 V6.1 의사결정, 터널 직선/곡선 간이 차량 모델 검사가 포함됩니다.
회전교차로 원본 validation을 통합하는 것과 별개로 이 연결부 회귀 검사는 저장소에 포함했습니다.
출처·보존 파일 해시는 루트 `JI_UJIN_PROVENANCE.json`에 기록했습니다.

이 개발 환경은 Windows이며 실제 ROS catkin 빌드·MORAI 패킷/주행 검증은 하지 못했습니다.
오프라인 통과가 시뮬레이터에서의 차선 추종·진입 안전을 보증하지는 않습니다.
현장에서는 CAM1 수신, LiDAR 장착값, Competition 포트, 진단 토픽을 확인한 후 주행 검증합니다.
