# 전방 카메라 차선 기반 GPS/IMU 이상 대응

## 목적

`morai_udp_ekf_curvature_camera_fallback.launch`는 인위적인 GPS/IMU noise topic을
사용하지 않고, MORAI에서 들어오는 센서 상태를 감시하면서 이상 상황에서만 전방
카메라 차선 결과를 주행 명령에 연결한다.

정상 상태에서는 기존 곡률 기반 Pure Pursuit를 그대로 사용한다.

```text
raw /gps, /Imu
        ↓
GPS 변환 + raw EKF
        ↓
곡률 기반 Pure Pursuit
        ↓ /control/curvature_ctrl_cmd

raw GPS/IMU + GPS health
        ↓
sensor_quality_monitor
        ↓ /localization/sensor_quality

전방 camera image
        ↓
camera_lane_detector_robust
        ↓ /detection/lane

Pure Pursuit + sensor quality + lane
        ↓
camera_localization_fallback_controller
        ↓ /control/camera_fallback_cmd
        ↓
control_mux → /ctrl_cmd → MORAI
```

## 상태별 동작

| 상태 | 조향 | 속도 |
|---|---|---|
| `NORMAL` | Pure Pursuit 그대로 | 곡률 속도 프로파일 그대로 |
| `GPS_NOISE` | Pure Pursuit + 작은 차선 보정 | nominal 속도 유지, 상한 적용 |
| `IMU_NOISE` | Pure Pursuit + 차선 보정 | nominal 속도 유지, 상한 적용 |
| `GPS_BLACKOUT` | 차선 조향을 주 제어로 사용 | nominal 또는 EKF 속도, 상한 적용 |
| 차선 stale/낮은 confidence | fallback 사용 안 함 | 기본 정지 |

카메라 fallback은 GPS의 위치나 IMU의 yaw를 복원하는 EKF가 아니다. GPS blackout
동안 위치를 새로 추정하는 것이 아니라, 차선의 lateral offset과 heading error로
조향을 유지하는 방식이다. 따라서 카메라가 보이지 않으면 자동으로 정지하는 것이
기본값이다.

## 주행 중 이상 판정

`sensor_quality_monitor.py`는 다음을 확인한다.

- GPS 수신 timeout 및 `GpsHealth` blackout 상태
- 연속 GPS 위치의 물리적으로 큰 jump 또는 비정상 step speed
- GPS 변환 위치와 EKF 위치의 잔차
- IMU 연속 샘플 간 gyro/accel 급변
- 각 이상 상태를 일정 샘플 이상 확인한 뒤 진입
- 정상 샘플이 연속으로 들어오면 anomaly 상태 해제

새 통합 launch의 EKF에는 별도의 입력 gate도 활성화되어 있다. 한 샘플의 GPS
jump와 EKF 상태 대비 큰 GPS innovation은 `sensor_quality`가 한 주기 늦게 갱신되어도
즉시 update에서 제외한다. 이 gate는 인위적인 noise를 생성하거나 정상 GPS를
필터링하는 기능이 아니라, 물리적으로 불가능한 측정이 상태를 오염시키는 것을
막는 장치다.

GPS/IMU 공식 노이즈 범위를 아직 모르므로 threshold는 노이즈를 가정해 넣은 값이
아니다. 오검출 방지를 위한 초기 이상 감지 gate이며, MORAI 공식 범위와 실제 로그를
확보한 뒤 조정해야 한다.

## 실행

먼저 테스트 모드:

```bash
cd /root/morai_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

catkin_make --pkg morai_perception_msgs stability_stack morai_camera_perception
source devel/setup.bash

roslaunch stability_stack morai_udp_ekf_curvature_camera_fallback.launch \
  path_file:=/root/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  front_camera_topic:=/camera/front/image/compressed \
  enable_control:=false
```

실제 MORAI에 명령을 보내는 단계는 토픽과 차선 방향을 먼저 확인한 뒤 실행한다.

```bash
roslaunch stability_stack morai_udp_ekf_curvature_camera_fallback.launch \
  path_file:=/root/morai_ws/data/routes/2026_molit_comp_global_path.txt \
  morai_host_ip:=192.168.0.148 \
  front_camera_topic:=/camera/front/image/compressed \
  enable_control:=true
```

`enable_control:=true`는 `/ctrl_cmd`를 MORAI로 송신한다. 카메라 영상이 실제로
다른 토픽으로 들어오면 `front_camera_topic`을 바꾼다.

## 확인 토픽

```bash
rostopic echo /localization/sensor_quality
rostopic echo /stability/camera_fallback_status
rostopic echo /detection/lane
rostopic hz /detection/lane
```

디버그 영상을 지원하는 경우 다음 토픽을 `rqt_image_view`에서 확인한다.

```bash
rqt_image_view /detection/lane_debug/compressed
```

왼쪽/오른쪽 차선이 정상적으로 보이고 `LaneDetection.confidence`가 0.65 이상일
때만 fallback에 사용한다. 조향 방향이 반대이면 launch의 `lane_sign`을 `-1.0`으로
바꾸기 전에 저속·`enable_control:=false` 상태에서 먼저 확인한다.

## 현재 한계

- 단안 카메라의 정확한 meter 좌표는 camera intrinsic/extrinsic calibration 없이는
  보장할 수 없다.
- 현재 차선 검출은 색상·ROI·Hough 기반이므로 그림자, 차선 삭제, 교차로, 심한
  곡선에서는 실패할 수 있다.
- 카메라만으로 장애물 안전을 보장하지 않는다. 실제 제어 시에는 검증된 LiDAR 또는
  장애물 safety topic을 `require_fresh_safety:=true`로 연결해야 한다.
- GPS blackout 동안의 주행 가능 시간은 IMU drift와 차선 가시성에 의해 제한된다.
- 이 코드는 주행 안정성 실험용이며, “완벽한” 자율주행을 보장하는 안전 인증 코드는
  아니다.
