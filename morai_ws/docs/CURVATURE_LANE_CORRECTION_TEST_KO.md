# 곡률 기반 주행과 차선 보정 비교 시험

현재 통합 주행의 기본 조향은 곡률 기반 Pure Pursuit다. `enable_lane_correction`을 켜면
정상 GPS 품질에서 최신 차선 오차를 곡률 조향에 제한된 가중치로 혼합한다. GPS blackout이나
센서 품질 저하에서는 기존 안전 fallback이 차선 조향을 단독으로 사용한다. 따라서 정상 주행
보정과 blackout fallback을 같은 경로에서 중복 적용하지 않는다.

## 실행 전

모든 기존 `roslaunch` 터미널에서 `Ctrl+C`를 누르고, MORAI에서는 `Ego Controller=AV-ExternalCtrl`,
`Status Initialization=OFF`, 기어 `D`를 확인한다. Ubuntu에서 ROS master와 workspace를 준비한다.

```bash
source /opt/ros/noetic/setup.bash
export MORAI_WS="$HOME/AutoVehicle/morai_ws"
source "$MORAI_WS/devel/setup.bash"
export ROS_MASTER_URI=http://192.168.0.200:11311
export ROS_IP=192.168.0.200
unset ROS_HOSTNAME
```

실제 Bash에서 여러 줄 명령을 입력할 때 줄 끝은 역슬래시 한 개(`\`)만 사용한다.

## 통합 시험

아래는 카메라 차선·YOLO는 켜고 ROI LiDAR는 끈 상태다. MORAI 제어 명령은
`192.168.0.148:9093`으로 전송하고 Ubuntu 송신 포트는 `9094`다.
`final_ws_bringup.launch`에서는 이 주소를 `morai_host_ip`로 지정하며, 하위
`morai_udp_drive_bridge`의 `control_remote_ip`로 그대로 전달된다. 브리지만 직접
실행할 때는 `control_remote_ip:=192.168.0.148`을 사용한다.

### 1. 곡률 기반 기준선

```bash
roslaunch morai_bringup final_ws_bringup.launch \
workspace_path:="$MORAI_WS" \
path_file:="$MORAI_WS/data/routes/2026_molit_comp_global_path.txt" \
enable_control:=true \
morai_host_ip:=192.168.0.148 \
ego_status_port:=1911 \
control_remote_port:=9093 \
control_source_port:=9094 \
enable_roi_camera:=true \
enable_roi_lidar:=false \
roi_enable_lane:=true \
roi_enable_yolo:=true \
enable_lane_fallback:=true \
enable_lane_correction:=false \
enable_maneuver_fusion:=true \
enable_turn_signal:=false \
test_without_turn_signals:=true \
max_speed_kph:=5.0 \
fallback_speed_cap_kph:=3.0
```

정상 GPS 품질에서 `/control/camera_fallback_status`가 `normal_nominal`이면 곡률 명령이 그대로
사용된다. 이 상태에서 차선 보정은 꺼져 있다.

### 2. 곡률 + 정상 차선 보정

1번 명령을 다시 실행하되 다음 세 값을 바꾼다.

```bash
enable_lane_correction:=true
lane_correction_weight:=0.15
max_speed_kph:=5.0
```

차선 보정이 적용되면 상태에 다음 항목이 표시된다.

```json
"mode": "normal_lane_corrected",
"lane_correction_applied": true
```

### 3. 속도 단계 상승

조향 방향과 차선 중심이 안정적인 것을 확인한 뒤에만 다음 순서로 속도를 올린다.

```text
5 → 10 → 20 → 30 km/h
```

blackout fallback 속도는 `fallback_speed_cap_kph`로 별도 제한된다. 이 값은 정상 GPS 상태의
최대 속도와 다르다.

## 모니터링 명령

```bash
rostopic echo -n 1 /control/camera_fallback_status
rostopic echo -n 1 /control/maneuver_status
rostopic echo -n 1 /control/mux_status
rostopic echo -n 1 /control/ctrl_cmd
rostopic echo -n 1 /control/camera_fallback_cmd
rostopic echo -n 1 /ctrl_cmd
```

곡률 출력과 차선 출력을 비교한다.

```bash
rostopic echo /experimental/curvature_steering
rostopic echo /experimental/curvature_value
rostopic echo /detection/lane
```

`/control/maneuver_cmd`부터 `brake: 1.0`이면 신호·정지선·위치·센서 safety gate의 원인을
`/control/maneuver_status`의 `reason`에서 확인한다. `/control/maneuver_cmd`는 가속인데
`/ctrl_cmd`만 브레이크면 `control_mux` 또는 안전 토픽을 확인한다.

## 조정 순서

1. `enable_lane_correction:=false`로 곡률 기준선을 먼저 확인한다.
2. `lane_correction_weight`를 `0.15 → 0.25 → 0.35`로 조금씩 올린다. 좌우 흔들림이 커지면
   직전 값으로 되돌린다.
3. 차선 중심 오차가 남으면 `lane_lateral_gain`을 10%씩 조정한다.
4. 차체 방향 오차가 늦게 따라가면 `lane_heading_gain`을 10%씩 조정한다.
5. 영상 노이즈가 크면 `lane_preview_distance_m`을 늘리고, 반응이 늦으면 줄인다.
   기본 차선 인식 기준은 `7 m`와 `14 m`다.
6. 조향 방향이 반대일 때만 `lane_sign:=-1.0`을 시험한다.

한 번에 하나의 파라미터만 바꾸고, 각 시험 시작 전에 이전 launch를 종료한다. `enable_lane_correction`
은 launch 시작 시 읽으므로 실행 중에 변경되지 않는다.

곡률 조향을 조정할 때는 다음 인자를 한 번에 하나씩 바꾼다. `lookahead_min_m`와
`curvature_preview_distance_m`을 줄이면 굽은 구간에 빨리 반응하지만 흔들림이 커질 수
있다. `lookahead_curvature_gain`을 키우면 큰 곡률에서 lookahead가 더 짧아진다.
`steering_feedforward_weight`는 경로 곡률을 미리 반영하는 비율이고,
`max_steering_rate_rad_s`는 조향 변화 속도 제한이다. `max_steering_rad`의 기본값
`0.6981317008`은 40도다. 이 인자들은 `final_ws_bringup.launch`와
`final_ws_native_no_lamps.launch`에서 직접 지정할 수 있다.
