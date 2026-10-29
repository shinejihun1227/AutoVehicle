# test/roundabout 신호·4번째 우회전 업데이트 옮기기

GPS 음영 IMU 예측까지 포함한 최신 누적 목록/ZIP은 [10개 파일 안내](20261030_TUNNEL_IMU_FILES_KO.md)를 사용합니다.
아래 6개 목록은 신호·우회전 수정만의 범위이며, 최신 전달 도구는 누적 10개를 교체합니다.

## 적용 동작

대상은 `mgeo:A2256W000318:0`으로 지정된 4번째 우회전입니다.

- 신뢰도 **0.6 초과**인 현재 경로의 우회전 허용 신호를 받으면 첫 유효 프레임에서 바로 통과합니다.
  이 교차로에서는 별도의 정차나 정차 대기, 추가 녹색 확인 시간 0.3초를 강제하지 않습니다.
  예전 개인 설정에 `SIGNAL_RIGHT_ON_GREEN=false`가 있어도 지정된 4번째 우회전에는 초록 우선을 적용합니다.
- 빨간불 등 우회전 허용 신호가 없으면 앞범퍼 목표 위치에서 완전 정지합니다.
  정지 확인 시간을 기존 1.0초에서 **0.5초**로 줄였으며, 확인 직후 우회전을 허가합니다.
  제어 주기/시간 정밀도에 따라 실제 해제는 약 0.5~0.6초입니다.
- 정지 중/대기 중에 빨강이 초록으로 바뀌면 남은 정지 확인 시간을 채우지 않고 바로 해제합니다.
- 정지 확인 후 이미 출발 허가를 받았으면 빨강이나 신호 소실로 다시 정지하지 않습니다.
  녹색 진입 후 신호 소실·늦은 빨강/노랑으로 교차로 안에 멈추는 이전 문제의 수정도 포함합니다.
- 이 우회전 출발에만 가속 페달 상승 제한을 **0.5/s → 1.0/s**로 높였습니다.
  이는 출발 가속 응답 변경이며 최고속도 강제 증가는 아닙니다.
  기존 메인 PI가 요청한 가속값, 곡률/코너 속도 상한과 장애물·위치·명령 이상 정지는 유지합니다.
- 다른 교차로의 신호 정책과 전체 속도·Competition 포트·GPS/IMU/EKF·회전교차로 기능은 그대로입니다.

진단 `/control/maneuver_status`의 `entry_policy`는 `green_priority`,
`stop_then_right_wait`, `stop_then_right_released`를 구분합니다.
`right_stop_dwell_sec=0.5`, 출발 중 `accel_rise_rate_per_sec=1.0`도 확인할 수 있습니다.
조절이 필요하면 이 노드에 `right_stop_dwell_sec`, `right_accel_rise_rate_per_sec` ROS 파라미터를 지정합니다.
기존 설정 파일에는 새 항목을 추가하지 않아도 위 기본값으로 적용됩니다.

## 어떤 파일을 덮어쓰는가

다른 PC도 기존 **test/roundabout 기반**일 때 적용합니다.
이전 진입/신뢰도 수정(`a58c67f`)이 이미 적용되어 있다면, 이번 실행 코드 변경은
아래 1번 `maneuver_fusion_node.py` 하나입니다.
이전 수정까지 함께 옮기려면 아래 **6개를 모두** 같은 상대 경로에 덮어씁니다.

```text
morai_ws/src/control/turn_signal_controller/scripts/maneuver_fusion_node.py
morai_ws/src/control/turn_signal_controller/src/turn_signal_controller/signal_association.py
morai_ws/src/detection/camera_perception/launch/camera_perception.launch
morai_ws/src/detection/camera_perception/scripts/camera_object_detection_node.py
morai_ws/src/detection/camera_perception/scripts/traffic_light_stop_node.py
morai_ws/src/detection/camera_perception/src/camera_perception/traffic_signal.py
```

`highway.env`, `highway-test.env`, 개인 IP/포트·모델 파일·지도 파일은 이 업데이트로 덮어쓰지 않습니다.
컨테이너 안의 파일만 직접 바꾸지 말고 **Ubuntu 호스트 저장소 파일부터** 갱신합니다.
그 뒤 아래 설치 명령으로 컨테이너에도 같은 파일을 반영해야 합니다.

## 인터넷/Git 사용이 가능한 PC

주행 터미널을 Ctrl+C로 종료하고, 실제 저장소 폴더 이름에 맞춰 실행합니다.

```bash
cd ~/AutoVehicle_roundabout &&
git pull --ff-only origin test/roundabout &&
cd morai_ws/docker/final_ws &&
bash run_highway.sh stop &&
bash install_curvature_signal.sh &&
bash run_highway.sh start &&
bash run_test.sh drive 2
```

## ZIP으로 다른 PC에 옮기는 경우

전달용 ZIP은 위 6개와 경로/체크섬/적용 스크립트만 포함합니다. 모델·env 파일은 없습니다.
주행을 Ctrl+C로 종료한 뒤 ZIP을 **별도의 임시 폴더**에 풀고 그 폴더에서 실행합니다.

```bash
bash apply_signal_update.sh "$HOME/AutoVehicle_roundabout"
```

적용 스크립트는 test/roundabout 저장소인지 확인하고, 기존 6개 파일을
`morai_ws/test-results/signal-backup.XXXXXX/`에 먼저 백업한 뒤 덮어씁니다.
스크립트 자체는 ROS·Docker·차량 제어를 실행하지 않습니다.
그 다음 호스트의 해당 저장소에서 컨테이너를 갱신합니다.

```bash
cd ~/AutoVehicle_roundabout/morai_ws/docker/final_ws &&
bash run_highway.sh stop &&
bash install_curvature_signal.sh &&
bash run_highway.sh start &&
bash run_test.sh drive 2
```

이미지 재빌드나 catkin 재빌드는 필요하지 않습니다. 기존 신호 지도 설정도 보존합니다.
검증은 원본 제어 콜백과 오프라인 시뮬레이션 검사이며, 실제 MORAI 주행 확인과는 구분합니다.
