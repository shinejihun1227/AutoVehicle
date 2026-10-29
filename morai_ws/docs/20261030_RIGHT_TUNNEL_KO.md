# 녹색 우회전 우선 처리와 GPS 음영 주행

대상: new_final_ws / `bash run_test.sh drive 2`.

## 4번째 교차로

직전 수정은 모든 신호에서 일시정지를 요구했고, 녹색 관측 처리를 건너뛰었다.
이제 해당 경로의 확인된 녹색/우회전 허용 신호를 먼저 사용한다.
녹색에서는 정차 없이 통과하고, 적색 또는 확인할 수 없는 신호일 때만 기존
정차 목표에서 1초간 완전 정지한 뒤 우회전한다. 정차 완료 뒤 적색 관측이
매 프레임 재정지시키는 문제도 제거했다. 진입 전 녹색이 실제 적색으로 바뀌면
정차 정책을 적용한다. 다른 교차로에는 이 예외가 적용되지 않는다.
원래 카메라 신호를 상태 출력에 유지하고 `event.right_entry_policy`로
`green_priority`, `stop_then_right_wait`, `stop_then_right_released`를 구분한다.

## 터널 위치 추정

기존 위치 추정은 GPS 단절 시 IMU 가속도 적분만 사용했다. Competition 속도는
주행 속도 제어에만 연결되어 있어 위치의 이동 거리와 실제 속도가 달라질 수 있었다.
개별 실제 정지의 직접 원인은 종전 터널 로그가 없어 확정하지 않는다.

profile 2는 이제 초기 GPS 위치 + IMU 방향 + Competition `velocity.x`로
위치를 예측한다. 내부 메시지의 속도는 이미 m/s이므로 다시 3.6으로 나누지 않는다.
IMU 가속도를 중복 적분하지 않으며, Competition의 위치·가속도·측면 속도나
heading을 위치 추정 입력으로 사용하지 않는다.

GPS가 끊겨도 IMU와 Competition 속도가 들어오면 예측 위치를 계속 발행한다.
실제 이동 중 0.5초 넘게 같은 GPS 좌표만 오는 경우도 고정 위치로 잡아당기지 않는다.
무효·0/0 위경도, 비유한 값과 비현실적인 큰 GPS 점프는 제외한다.
GPS 복귀 시 GPS 위치 업데이트로 보정한다.

IMU가 끊기거나 Competition 속도가 0.5초 넘게 지연되면 신선한 위치를 발행하지
않아 기존 pose/속도 타임아웃 정지가 동작한다. GPS 패킷만으로 멈춘 위치를
정상 갱신 중인 것처럼 만들지 않는다. 실행 전에 한 번은 유효한 GPS 초기 위치가
필요하다. 터널 안에서 프로그램을 새로 켜도 위치를 자동 생성하지 않는다.

이 방식은 IMU+속도 추측항법이며 LiDAR 지도 매칭은 아니다. GPS 음영이 길면
방향·위치 오차가 누적될 수 있다. CAM1 차선 보정은 정상 영상 수신 시 보조하며,
이번 변경은 profile 2에 별도 장애물/터널 회피 기능을 추가하지 않는다.

## 업데이트

현재 drive/monitor를 Ctrl+C로 종료하고 Ubuntu 호스트에서 실행한다.

```bash
cd ~/AutoVehicle_new_final_ws
git pull --ff-only origin new_final_ws
cd morai_ws/docker/final_ws
bash run_highway.sh stop
bash install_curvature_signal.sh --config
bash run_highway.sh start
bash run_test.sh drive 2
```

기존 수신 포트·속도·정차 거리 env 설정은 유지한다.
앞범퍼 정지선 10m 전 정차는 `bash run_test.sh clearance 10`으로 저장한 뒤
주행 런치를 재시작한다. 별도 호스트 catkin 빌드는 Docker 절차에 필요하지 않다.
기존 ROS 이미지에 morai_msgs/std_msgs가 있어 Python/launch 갱신을 바로 사용한다.

별도 터미널에서 `bash run_test.sh diagnose 2`를 실행한다.
새 `estimator` 행의 모드는 GPS 보정 중 `GPS_IMU_COMPETITION`, GPS 음영 중
`IMU_COMPETITION_DEAD_RECKONING`이며 `gps_age_sec`, `gps_frozen`, `speed_mps`,
`position_std_m`을 함께 표시한다. 정지 시 `status.reason`과 이 행, IMU/ego/odom의
count 및 age를 함께 봐야 실제 수신 단절과 경로 오차를 구별할 수 있다.

실제 MORAI 주행·GPS 음영 길이에 대한 검증은 아직 수행하지 못했다.
