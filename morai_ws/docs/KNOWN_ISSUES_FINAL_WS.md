# final_ws 기준 코드의 미해결 사항

기록일: 2026-09-08. 이번 저장은 현재 구현의 백업이며 대회 주행 합격본이 아니다.
아래 문제는 이전 코드 리뷰에서 확인했고, Docker 환경 구성 작업에서는 수정하지 않았다.
환경 설치 성공이나 기존 단위 테스트 통과만으로 해소되지 않는다.

| 우선순위 | 문제 | 수정·회귀 시험 조건 |
|---|---|---|
| P1 | 카메라 정지선 보정이 교차로 진입 판정 경계까지 앞당김 | 제동 목표와 실제 진입 경계를 분리. 정지선 앞에서 신호가 적색으로 바뀌면 허가 취소 |
| P1 | 제어 tick 사이 적색 관측이 다음 허용 신호에 덮일 수 있음 | 수신 즉시 허가 취소 또는 관측 순서 처리. RED→LEFT 연속 수신에서 재확인 요구 |
| P1 | 보행자 입력만 끊겨도 오래된 clear 상태를 새 Bool로 발행 | 원본 시각·유효성 전달. 해당 입력만 중단해도 정지 |
| P1 | 늦게 도착한 차선 변경 간격 정보를 새 정보로 취급 | LiDAR 원본 시각/scan ID 검증. 지연·중복 clear 거부 |
| P2 | 영상 이후 도착한 동일 시각 pose로 신호 대응을 재시도하지 않음 | pose 도착 후 보류 영상을 재평가. 센서 도착 순서 교환 시험 |

관련 구현: `turn_signal_controller/scripts/maneuver_fusion_node.py`,
`camera_perception/src/camera_perception/pedestrian_crossing.py`,
`morai_sensor_fusion/scripts/roi_sensor_safety_adapter.py`,
`lidar_perception/scripts/lidar_merge_gap_node.py`.

추가 현장 확인이 필요한 항목:

- `signal_camera.calibrated:false`: 실제 Cam4(UDP 1131)의 원본 해상도/FOV/장착 위치·각도와 투영 검증 필요. 차선 Cam1 값을 복사하지 않는다.
- 위치 좌표 정렬, 실제 지도·차량 버전, 뒤차축→앞범퍼 3.845m 가정의 일치 여부.
- 기존 mock 테스트는 실제 ROS 빌드, 추론 처리 지연, UDP 왕복, 신호등/정지선 주행을 검증하지 않는다.

현재 허용할 시험 범위는 오프라인 검사와 `enable_control:=false` 센서 관찰이다.
이는 이 launch의 제어/방향지시등 UDP 송신을 끄는 설정이지 차량을 물리적으로
정지시키는 비상정지가 아니다. MORAI에서 차량을 먼저 정지시키고 다른 제어 송신기도 종료한다.
실제 주행은 위 P1 수정·회귀 검증 및 센서 보정 이후에 진행한다.
