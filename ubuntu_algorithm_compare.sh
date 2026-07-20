#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-$HOME/AutoVehicle}"
CATKIN_WS="${CATKIN_WS:-$HOME/morai_stanley_ws}"
WAYPOINT_FILE="${WAYPOINT_FILE:-$HOME/morai_recorded_paths/01.csv}"
TARGET_SPEED_MPS="${TARGET_SPEED_MPS:-4.0}"
MAX_SPEED_MPS="${MAX_SPEED_MPS:-$TARGET_SPEED_MPS}"

usage() {
  cat <<'EOF'
Usage:
  ./ubuntu_algorithm_compare.sh install <pp|app|stanley|hybrid>
  ./ubuntu_algorithm_compare.sh record <pp|app|stanley|hybrid>
  ./ubuntu_algorithm_compare.sh drive <pp|app|stanley|hybrid>
  ./ubuntu_algorithm_compare.sh topics <pp|app|stanley|hybrid>

Environment overrides:
  REPO_DIR=$HOME/AutoVehicle
  CATKIN_WS=$HOME/morai_stanley_ws
  WAYPOINT_FILE=$HOME/morai_recorded_paths/01.csv
  TARGET_SPEED_MPS=4.0
  MAX_SPEED_MPS=4.0
EOF
}

source_ros() {
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  # shellcheck disable=SC1091
  source "$CATKIN_WS/devel/setup.bash"
}

copy_packages() {
  local ws_dir="$1"
  shift
  mkdir -p "$CATKIN_WS/src"
  cd "$CATKIN_WS"
  for pkg in "$@"; do
    rm -rf "src/$pkg"
    cp -r "$REPO_DIR/$ws_dir/src/$pkg" "src/"
  done
  chmod +x src/*/scripts/*.py
  source /opt/ros/noetic/setup.bash
  catkin_make
  source devel/setup.bash
  rospack profile
}

install_algorithm() {
  case "$1" in
    pp)
      copy_packages ros_ws yeoms_base_bridge yeoms_bringup yeoms_path_recorder yeoms_pure_pursuit
      ;;
    app)
      copy_packages ros_ws2 yeoms2_base_bridge yeoms2_bringup yeoms2_path_recorder yeoms2_adaptive_pure_pursuit
      ;;
    stanley)
      copy_packages ros_ws3 yeoms3_base_bridge yeoms3_bringup yeoms3_path_recorder yeoms3_stanley
      ;;
    hybrid)
      copy_packages ros_ws4 yeoms4_base_bridge yeoms4_bringup yeoms4_path_recorder yeoms4_hybrid_controller
      ;;
    *)
      usage
      exit 2
      ;;
  esac
}

record_algorithm() {
  source_ros
  mkdir -p "$(dirname "$WAYPOINT_FILE")"
  case "$1" in
    pp) roslaunch yeoms_bringup record_gps_path.launch output_file:="$WAYPOINT_FILE" ;;
    app) roslaunch yeoms2_bringup record_gps_path.launch output_file:="$WAYPOINT_FILE" ;;
    stanley) roslaunch yeoms3_bringup record_gps_path.launch output_file:="$WAYPOINT_FILE" ;;
    hybrid) roslaunch yeoms4_bringup record_gps_path.launch output_file:="$WAYPOINT_FILE" ;;
    *) usage; exit 2 ;;
  esac
}

drive_algorithm() {
  source_ros
  case "$1" in
    pp)
      roslaunch yeoms_bringup drive_pure_pursuit.launch \
        waypoint_file:="$WAYPOINT_FILE" \
        target_speed_mps:="$TARGET_SPEED_MPS" \
        max_speed_mps:="$MAX_SPEED_MPS"
      ;;
    app)
      roslaunch yeoms2_bringup drive_adaptive_pure_pursuit.launch \
        waypoint_file:="$WAYPOINT_FILE" \
        target_speed_mps:="$TARGET_SPEED_MPS" \
        max_speed_mps:="$MAX_SPEED_MPS" \
        min_curve_speed_mps:=2.5 \
        min_lookahead_m:=3.0 \
        max_lookahead_m:=14.0
      ;;
    stanley)
      roslaunch yeoms3_bringup drive_stanley.launch \
        waypoint_file:="$WAYPOINT_FILE" \
        target_speed_mps:="$TARGET_SPEED_MPS" \
        max_speed_mps:="$MAX_SPEED_MPS" \
        stanley_gain:=0.35 \
        softening_gain:=2.0 \
        min_curve_speed_mps:=2.5
      ;;
    hybrid)
      roslaunch yeoms4_bringup drive_hybrid.launch \
        waypoint_file:="$WAYPOINT_FILE" \
        target_speed_mps:="$TARGET_SPEED_MPS" \
        max_speed_mps:="$MAX_SPEED_MPS" \
        hybrid_stanley_weight:=0.45 \
        stanley_gain:=0.35 \
        softening_gain:=2.0 \
        min_curve_speed_mps:=2.5 \
        min_lookahead_m:=3.0 \
        max_lookahead_m:=12.0
      ;;
    *)
      usage
      exit 2
      ;;
  esac
}

topics_algorithm() {
  case "$1" in
    pp)
      cat <<'EOF'
rostopic echo /control/pure_pursuit_steering_rad
rostopic echo /control/pure_pursuit_raw_steering_rad
rostopic echo /control/curve_speed_limit_mps
EOF
      ;;
    app)
      cat <<'EOF'
rostopic echo /control/adaptive_pp_steering_rad
rostopic echo /control/adaptive_pp_raw_steering_rad
rostopic echo /control/adaptive_pp_cross_track_error_m
rostopic echo /control/adaptive_lookahead_distance_m
EOF
      ;;
    stanley)
      cat <<'EOF'
rostopic echo /control/stanley_steering_rad
rostopic echo /control/stanley_raw_steering_rad
rostopic echo /control/stanley_heading_error_rad
rostopic echo /control/stanley_cross_track_error_m
EOF
      ;;
    hybrid)
      cat <<'EOF'
rostopic echo /control/hybrid_steering_rad
rostopic echo /control/hybrid_pure_pursuit_raw_steering_rad
rostopic echo /control/hybrid_stanley_raw_steering_rad
rostopic echo /control/hybrid_stanley_weight
rostopic echo /control/hybrid_cross_track_error_m
EOF
      ;;
    *)
      usage
      exit 2
      ;;
  esac
}

if [[ $# -lt 2 ]]; then
  usage
  exit 2
fi

action="$1"
algorithm="$2"

case "$action" in
  install) install_algorithm "$algorithm" ;;
  record) record_algorithm "$algorithm" ;;
  drive) drive_algorithm "$algorithm" ;;
  topics) topics_algorithm "$algorithm" ;;
  *) usage; exit 2 ;;
esac
