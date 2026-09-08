# GPS blackout과 전방 카메라 안정성 실험 스택

기존 주행 런치를 직접 대체하지 않고 다음 흐름을 선택적으로 실행한다.

```text
/localization/gps ──────────────────────> EKF
/Imu              ──────────────────────> EKF
/gps              ─> gps_blackout_detector ─> /localization/gps_health

Pure Pursuit /control/ctrl_cmd
  └─ camera_stability_controller ─> /control/camera_stable_cmd
                                      └─ gps_blackout_stability_controller
                                         └─ control_mux ─> /ctrl_cmd
```

## 실행

```bash
cd ~/morai_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch stability_stack stability_stack.launch \
  camera_stability_enabled:=false \
  enable_control:=false
```

## 확인 토픽

```bash
rostopic echo /localization/gps_health
rostopic echo /stability/gps_blackout_status
rostopic echo /stability/gps_blackout_control_status
rostopic echo /stability/camera_status
rostopic echo /control/mux_status
```

## 적용 순서

1. 카메라 보정=false로 원본 GPS/IMU baseline 확인
2. GPS blackout detector의 health 상태 확인
3. 전방 카메라 lane sign과 조향 방향을 저속에서 확인
4. `camera_stability_enabled=true`를 켜고 steering rate를 제한
5. 마지막에 `enable_control=true`로 실제 MORAI를 저속 검증

GPS blackout 처리에서는 GPS 패킷이 일정 시간 들어오지 않거나 `status=0`이면
`GPS_BLACKOUT`이 된다. 유효한 GPS가 다시 들어와도 기본 5개 샘플이 연속으로
확인될 때까지 `GPS_RECOVERING`으로 유지한다. blackout 중에는 EKF가 IMU
prediction을 계속 수행하고, `gps_blackout_stability_controller`가 속도를
기본 1.0 m/s로 제한하며 조향·속도 변화율을 제한한다. 복구 중에는 기본 1.5 m/s로
제한하고, `GPS_OK`가 되면 Pure Pursuit 명령의 정상 속도 제한으로 돌아간다.

GPS blackout만으로 정지시키려면 다음 인자를 켤 수 있지만, 기본값은 장애물
`safety_stop`과 구분하기 위해 false다.

```bash
stop_on_gps_blackout:=true
```

카메라 안정화 보정은 calibration 전에는 반드시 false로 둔다. 현재 lane detector는
고전적 threshold baseline이며, `lateral_sign`과 `heading_sign`은 실제 차량의 좌우
조향 부호를 확인한 뒤 조정해야 한다.

이 스택은 MORAI 원본 센서 토픽을 그대로 사용하며, 인위적인 센서 noise injection이나
noise filter를 수행하지 않는다. GPS 패킷 수신 중단과 recovery만 별도로 감시한다.
