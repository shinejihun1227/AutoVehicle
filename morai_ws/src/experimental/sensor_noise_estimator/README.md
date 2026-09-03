# sensor_noise_estimator

MORAI에서 들어오는 원본 GPS·IMU의 정지 구간 변동을 측정하는 독립 패키지다.
인위적인 noise injector, GPS/IMU filter, EKF, Pure Pursuit 및 차량 제어 명령을
포함하지 않는다.

## 입력

기본 입력은 현재 `morai_ws`의 원본 경로와 같다.

```text
/localization/gps   nav_msgs/Odometry   GPS 좌표 변환 결과
/Imu                sensor_msgs/Imu    원본 IMU
```

선택적으로 `speed_topic`에 `nav_msgs/Odometry`를 연결하면 정지 판정에 속도를
우선 사용한다. 지정하지 않으면 gyro·가속도와 GPS 이동량으로 정지 여부를 보조
판정한다. `/Ego_topic`에는 의존하지 않는다.

## 출력

```text
/localization/noise_statistics       sensor_noise_estimator/NoiseStatistics
/localization/noise_statistics_json  std_msgs/String
```

정지 후 안정화 시간이 지나고 최소 샘플 수를 모으면 다음을 매초 갱신한다.

- GPS 중심값과 robust 표준편차
- GPS 3-sigma 범위
- gyro 정지 평균값(gyro bias 후보)
- gyro 표준편차와 3-sigma 범위
- 가속도 정지 평균값과 표준편차
- GPS/IMU 수신 주기와 최대 gap

GPS 중심값은 정확한 기준 위치가 없을 때 절대 bias가 아니라 관측 중심이다.
가속도 평균도 차체 pitch/roll과 중력 투영을 보정하지 않으면 bias가 아니라
정지 기준값으로 해석해야 한다.

## 실행

기존 주행 launch와 UDP 포트를 동시에 열지 않는다.

```bash
cd /root/morai_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch sensor_noise_estimator sensor_noise_measurement.launch \
  csv_path:=/root/morai_ws/data/measurements/raw_sensor_noise.csv
```

이미 원본 `/localization/gps`와 `/Imu`를 발행하는 주행 launch가 실행 중이면
UDP 포트를 다시 열지 않도록 `start_raw_sources:=false`를 사용한다.

```bash
roslaunch sensor_noise_estimator sensor_noise_measurement.launch \
  start_raw_sources:=false \
  speed_topic:=/localization/odometry \
  csv_path:=/root/morai_ws/data/measurements/raw_sensor_noise.csv
```

이 경우에도 estimator는 `/gps_noisy`, `/Imu_noisy`, `/gps_filtered`,
`/Imu_filtered`를 구독하지 않고 원본 토픽만 관찰한다. 기존 주행 launch에서
인위적인 노이즈를 끄려면 해당 launch를 별도로 `enable_sensor_noise:=false`로
실행한다.

EKF가 실행 중이지 않으면 `speed_topic`을 생략할 수 있다. 그러면 gyro·가속도와
GPS 이동량을 이용해 정지 여부를 보조 판정한다.

```bash
roslaunch sensor_noise_estimator sensor_noise_measurement.launch \
  start_raw_sources:=false \
  csv_path:=/root/morai_ws/data/measurements/raw_sensor_noise.csv
```

## 확인

```bash
rostopic echo /localization/noise_statistics
rostopic echo /localization/noise_statistics_json
rostopic hz /localization/noise_statistics
```

이 패키지는 측정 결과를 주행 필터에 자동 반영하지 않는다. 공식 MORAI 노이즈
범위가 공개된 후 측정값과 비교한 다음, EKF covariance나 bias 보정 정책을 별도
검토한다.
