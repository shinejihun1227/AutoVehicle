# test/roundabout 누적 수정: 신호·우회전·GPS 음영 IMU 주행

`ef3d0e8`의 신호/우회전 수정에 기존 new_final_ws `a5a8f7c`의 GPS 음영 예측만 추가했습니다.
초기 GPS 위치를 얻은 뒤 GPS가 끊기면 IMU 자세·각속도와 Competition 전방 속도로 위치를 예측합니다.
Competition 속도는 기존 브리지에서 m/s로 변환한 velocity.x만 읽습니다.
일반 Ego 위치·횡속도·가속도를 사용하지 않습니다. IMU 가속도 적분으로 속도를 계속 늘리지 않습니다.
GPS가 복구되면 기존 GPS 보정으로 돌아옵니다. IMU 또는 Competition 속도까지 끊기면 안전 정지는 유지합니다.
터널용 카메라 우선 조향·추가 차선 보정은 넣지 않았으며 기존 차선 보정 설정은 그대로입니다.
차량 제어·포트·신호·회전교차로 정책도 바꾸지 않았습니다.

## 누적 덮어쓰기 파일 10개

신호 소실·0.6 초과 신뢰도·4번째 우회전 수정부터 이번 GPS 음영 수정까지 포함한 목록입니다.
아래 경로는 test/roundabout 저장소 루트 기준입니다.

```text
morai_ws/src/control/turn_signal_controller/scripts/maneuver_fusion_node.py
morai_ws/src/control/turn_signal_controller/src/turn_signal_controller/signal_association.py
morai_ws/src/detection/camera_perception/launch/camera_perception.launch
morai_ws/src/detection/camera_perception/scripts/camera_object_detection_node.py
morai_ws/src/detection/camera_perception/scripts/traffic_light_stop_node.py
morai_ws/src/detection/camera_perception/src/camera_perception/traffic_signal.py
morai_ws/src/localization/ekf_local_enu/scripts/ekf_local_enu_node.py
morai_ws/src/localization/ekf_local_enu/launch/ekf_local_enu.launch
morai_ws/src/bringup/morai_bringup/launch/morai_udp_ekf_purepursuit.launch
morai_ws/src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch
```

이번 GPS 음영 기능만 추가할 경우 마지막 4개입니다.
Python 노드 하나만 복사하면 속도 입력 연결이 빠지므로 **런치 3개도 함께** 교체합니다.
`highway.env`, `highway-test.env`, 개인 포트/IP, 지도와 모델은 덮어쓰지 않습니다.

## 반영

주행을 Ctrl+C로 종료하고 Ubuntu 호스트 저장소의 위 경로를 교체합니다.
ZIP을 사용하는 경우 별도 폴더에 풀고 그 안에서 다음을 실행하면 10개를 백업 후 교체합니다.

```bash
bash apply_signal_update.sh "$HOME/AutoVehicle_roundabout"
```

### ZIP 덮어쓰기 후 Permission denied 복구

초기 배포 ZIP은 Unix 읽기 권한이 600으로 저장됐습니다. 이 파일을 sudo/Docker로
복사하고 일반 계정으로 다시 검증하면 `sha256sum: Permission denied`가 발생할 수 있습니다.
수정 ZIP은 일반 파일 644, 실행 노드와 셸 도구 755를 명시합니다.
설치기도 컨테이너로 복사할 공개 소스·모델의 읽기 권한을 정규화합니다.

이미 덮어쓴 경우 수정 ZIP을 **별도 새 폴더**에 풀고, 그 폴더에서 다음을 실행합니다.
sudo bash로 실행하지 않습니다. 필요할 때만 도구가 파일 소유권 변경을 위해 sudo를 요청합니다.

```bash
bash repair_signal_permissions.sh "$HOME/AutoVehicle_roundabout" &&
bash apply_signal_update.sh "$HOME/AutoVehicle_roundabout"
```

복구 도구는 위에 명시한 호스트 소스 10개의 소유권·권한만 복구합니다.
파일 내용, IP·포트·env, 컨테이너는 변경하지 않습니다. 교체 도구는 기존 내용을 백업합니다.
그 후 아래 컨테이너 설치 순서를 다시 완료합니다. 이전 검증이 실패했으면 설치 완료로 간주하지 않습니다.
새 ZIP의 10개 주행 소스 내용은 `28d5c0e`와 동일하며 이번 변경은 전달·설치 권한 문제만 수정합니다.

이전 파일은 대상 저장소 `morai_ws/test-results/signal-backup.XXXXXX/`에 보존됩니다.
기존 컨테이너에도 반영하려면 실행 중인 주행을 종료한 뒤 다음 순서로 갱신합니다.

```bash
cd ~/AutoVehicle_roundabout/morai_ws/docker/final_ws &&
bash run_highway.sh stop &&
bash install_curvature_signal.sh &&
bash run_highway.sh start &&
bash run_test.sh drive 2
```

Git을 쓸 수 있으면 대상 저장소에서 `git pull --ff-only origin test/roundabout` 후 동일하게 설치합니다.
이미지/catkin 재빌드는 필요 없습니다. 이 목록의 문서·시험·전달 도구 자체는 주행 필수 교체 파일이 아닙니다.
GPS 없는 상태에서 초기 지도 위치를 임의 생성하는 기능은 아니며 실제 MORAI 주행 확인은 별도로 필요합니다.
