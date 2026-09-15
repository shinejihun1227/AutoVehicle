#!/usr/bin/env bash
# Launch recipes only: all driving and safety calculations remain in ROS nodes.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-show}"
if [[ $# -gt 1 || ! "$ACTION" =~ ^(show|monitor|drive|diagnose)$ ]]; then
  echo 'Usage: bash run_test.sh {show|monitor|drive|diagnose}; edit highway-test.env for parameters.' >&2
  exit 2
fi
if [[ ! -f "$SCRIPT_DIR/highway.env" || ! -f "$SCRIPT_DIR/highway-test.env" ]]; then
  echo 'Create highway.env and copy highway-test.env.example to highway-test.env first.' >&2
  exit 2
fi
source "$SCRIPT_DIR/highway.env"
# The versioned example supplies defaults for fields added by later updates.
source "$SCRIPT_DIR/highway-test.env.example"
source "$SCRIPT_DIR/highway-test.env"
if [[ "$ACTION" == diagnose ]]; then
  DOCKER=(docker --context default)
  if ! "${DOCKER[@]}" info >/dev/null 2>&1; then DOCKER=(sudo docker --context default); fi
  exec "${DOCKER[@]}" exec "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
    timeout --signal=INT --kill-after=2s 15s python \
    /opt/AutoVehicle/morai_ws/docker/final_ws/diagnose_highway.py
fi
CONTROL=false
[[ "$ACTION" != drive ]] || CONTROL=true

COMMON_ARGS=(
  "workspace_path:=/opt/AutoVehicle/morai_ws"
  "path_file:=/opt/AutoVehicle/morai_ws/data/routes/2026_molit_comp_global_path.txt"
  "gps_port:=$GPS_PORT" "imu_port:=$IMU_PORT" "ego_status_port:=$EGO_STATUS_PORT"
  "morai_host_ip:=$MORAI_IP"
  "control_remote_port:=$CONTROL_REMOTE_PORT" "control_source_port:=$CONTROL_SOURCE_PORT"
  "max_speed_kph:=$MAX_SPEED_KPH" "lateral_accel_limit_mps2:=$LATERAL_ACCEL_LIMIT_MPS2"
  "max_accel_mps2:=$MAX_ACCEL_MPS2" "max_decel_mps2:=$MAX_DECEL_MPS2"
  "speed_kp:=$SPEED_KP" "speed_ki:=$SPEED_KI"
  "speed_integral_limit_kph_s:=$SPEED_INTEGRAL_LIMIT_KPH_S"
  "speed_error_deadband_kph:=$SPEED_ERROR_DEADBAND_KPH"
  "lookahead_min_m:=$LOOKAHEAD_MIN_M" "lookahead_gain:=$LOOKAHEAD_GAIN"
  "lookahead_max_m:=$LOOKAHEAD_MAX_M" "lookahead_tight_min_m:=$LOOKAHEAD_TIGHT_MIN_M"
  "lookahead_curvature_gain:=$LOOKAHEAD_CURVATURE_GAIN"
  "steering_feedforward_weight:=$STEERING_FEEDFORWARD_WEIGHT"
  "max_steering_rate_rad_s:=$MAX_STEERING_RATE_RAD_S"
)
PROFILE_ARGS=()
case "$TEST_PROFILE" in
  curvature)
    LAUNCH=morai_udp_ekf_purepursuit.launch
    PROFILE_ARGS=("use_curvature_speed_planner:=true" "command_topic:=/ctrl_cmd")
    ;;
  curvature_signal)
    LAUNCH=final_ws_curvature_signal.launch
    PROFILE_ARGS=(
      "signal_config_file:=$SIGNAL_CONFIG_FILE"
      "lane_info_port:=$LANE_INFO_PORT" "yolo_port:=$YOLO_PORT"
      "lane_info_device:=$LANE_INFO_DEVICE" "lane_info_every:=$LANE_INFO_EVERY"
      "lane_min_confidence:=$LANE_MIN_CONFIDENCE" "lane_info_timeout_s:=$LANE_INFO_TIMEOUT_S"
      "stopline_front_reference_offset_m:=$STOPLINE_FRONT_REFERENCE_OFFSET_M"
      "stopline_hold_distance_m:=$STOPLINE_HOLD_DISTANCE_M"
      "stopline_planning_decel_mps2:=$STOPLINE_PLANNING_DECEL_MPS2"
      "right_on_green:=$SIGNAL_RIGHT_ON_GREEN"
    )
    ;;
  full|obstacle|merge)
    LAUNCH=final_ws_highway_bringup.launch
    YOLO=false; HIGHWAY=true
    [[ "$TEST_PROFILE" != full ]] || YOLO=true
    [[ "$TEST_PROFILE" != obstacle ]] || HIGHWAY=false
    PROFILE_ARGS=(
      "enable_yolo:=$YOLO" "enable_stopline_control:=$YOLO"
      "enable_highway_lane_change:=$HIGHWAY"
      "enable_lane_info_publisher:=true" "enforce_camera_lane_bounds_on_bypass:=true"
      "lane_info_device:=$LANE_INFO_DEVICE" "lane_info_every:=$LANE_INFO_EVERY"
      "lane_info_port:=$LANE_INFO_PORT" "yolo_port:=$YOLO_PORT"
      "roi_lidar_host_port:=$ROI_LIDAR_HOST_PORT" "roi_lidar_port:=$ROI_LIDAR_PORT"
      "lane_min_confidence:=$LANE_MIN_CONFIDENCE" "lane_info_timeout_s:=$LANE_INFO_TIMEOUT_S"
      "front_min_gap_m:=$FRONT_MIN_GAP_M" "rear_min_gap_m:=$REAR_MIN_GAP_M"
      "time_headway_s:=$TIME_HEADWAY_S" "min_ttc_s:=$MIN_TTC_S"
      "max_lane_changes:=$MAX_LANE_CHANGES" "evaluation_speed_mps:=$EVALUATION_SPEED_MPS"
      "stopline_front_reference_offset_m:=$STOPLINE_FRONT_REFERENCE_OFFSET_M"
      "lidar_x_m:=$LIDAR_X_M" "lidar_y_m:=$LIDAR_Y_M" "lidar_z_m:=$LIDAR_Z_M"
      "lidar_yaw_deg:=$LIDAR_YAW_DEG"
    )
    ;;
  *) echo "Unknown TEST_PROFILE: $TEST_PROFILE" >&2; exit 2 ;;
esac

echo "Profile=$TEST_PROFILE  action=$ACTION  max_speed_kph=$MAX_SPEED_KPH"
echo "Ubuntu=$UBUNTU_IP  MORAI=$MORAI_IP  container=$CONTAINER_NAME"
if [[ "$ACTION" == show ]]; then
  echo "Launch=$LAUNCH (show does not contact Docker or send control)"
  printf '%s\n' "${COMMON_ARGS[@]}" "${PROFILE_ARGS[@]}"
  exit 0
fi

if [[ "$TEST_PROFILE" != curvature && "$TEST_PROFILE" != curvature_signal ]]; then
  exec bash "$SCRIPT_DIR/run_highway.sh" "$ACTION" "${COMMON_ARGS[@]}" "${PROFILE_ARGS[@]}"
fi
DISPLAY_ARGS=()
if [[ "$TEST_PROFILE" == curvature_signal ]]; then
  echo 'Original route + curvature + route-associated stopline/signals; no LiDAR/avoidance/merge/lane steering.'
  DISPLAY_ARGS=(xvfb-run -a)
else
  echo 'Curvature-only: original route; camera/LiDAR/signal stops are not launched.'
fi
DOCKER=(docker --context default)
if ! "${DOCKER[@]}" info >/dev/null 2>&1; then DOCKER=(sudo docker --context default); fi
exec "${DOCKER[@]}" exec -it "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
  "${DISPLAY_ARGS[@]}" roslaunch morai_bringup "$LAUNCH" "enable_control:=$CONTROL" \
  "${COMMON_ARGS[@]}" "${PROFILE_ARGS[@]}"
