#!/usr/bin/env bash
# Update fixed-route controller, signal launch and dashboard in a stopped container.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --config ) ]]; then
  echo 'Usage: bash install_curvature_signal.sh [--config]' >&2
  exit 2
fi
[[ -f "$SCRIPT_DIR/highway.env" ]] || { echo 'Missing highway.env' >&2; exit 2; }
source "$SCRIPT_DIR/highway.env"
: "${CONTAINER_NAME:?Set CONTAINER_NAME in highway.env}"
DOCKER=(docker --context default)
if ! "${DOCKER[@]}" info >/dev/null 2>&1; then DOCKER=(sudo docker --context default); fi
RUNNING="$("${DOCKER[@]}" inspect --format '{{.State.Running}}' "$CONTAINER_NAME")"
[[ "$RUNNING" == false ]] || {
  echo 'Stop the launch, then bash run_highway.sh stop before updating. Restart after installation.' >&2
  exit 2
}
DEST=/opt/AutoVehicle/morai_ws
CAM=src/detection/camera_perception
CURVE=src/experimental/curvature_speed_purepursuit
FILES=(
  src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch
  "$CURVE/scripts/curvature_speed_purepursuit_node.py"
  "$CURVE/CMakeLists.txt"
  "$CURVE/package.xml"
  "$CURVE/src/curvature_speed_purepursuit/planner.py"
  "$CURVE/test/test_node_startup.py"
  src/control/purepursuit_mgeo/src/purepursuit_mgeo/longitudinal_controller.py
  src/control/turn_signal_controller/scripts/maneuver_fusion_node.py
  "$CAM/launch/camera_perception.launch"
  "$CAM/post_processing/real_lane_node.py"
  "$CAM/scripts/camera_object_detection_node.py"
  "$CAM/scripts/camera_debug_dashboard.py"
  "$CAM/src/camera_perception/debug_images.py"
  "$CAM/src/camera_perception/debug_dashboard.py"
  "$CAM/CMakeLists.txt"
  "$CAM/package.xml"
  "$CAM/test/test_debug_dashboard.py"
  "$CAM/test/test_camera_timestamps.py"
  docker/final_ws/check_highway_launch.py
)
for file in "${FILES[@]}" "$CAM/web/camera_dashboard.html" config/curvature_signal.yaml; do
  [[ -f "$WS/$file" ]] || { echo "Missing host file: $WS/$file" >&2; exit 2; }
done
BACKUP="$HOME/morai-update-backups/$CONTAINER_NAME-camera-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP"
for file in "${FILES[@]}" "$CAM/web/camera_dashboard.html" config/curvature_signal.yaml; do
  mkdir -p "$BACKUP/$(dirname -- "$file")"
  "${DOCKER[@]}" cp "$CONTAINER_NAME:$DEST/$file" "$BACKUP/$file" 2>/dev/null || true
done
# The new executable can run directly from scripts/ in the existing catkin source workspace.
chmod +x "$WS/$CAM/scripts/camera_debug_dashboard.py" "$WS/$CURVE/scripts/curvature_speed_purepursuit_node.py" \
  "$WS/src/control/turn_signal_controller/scripts/maneuver_fusion_node.py"
for file in "${FILES[@]}"; do
  "${DOCKER[@]}" cp "$WS/$file" "$CONTAINER_NAME:$DEST/$file"
done
"${DOCKER[@]}" cp "$WS/$CAM/web" "$CONTAINER_NAME:$DEST/$CAM/"
CONFIG=/opt/AutoVehicle/morai_ws/config/curvature_signal.yaml
# Preserve a previously calibrated container file unless --config is explicit.
# docker cp works with stopped containers too; no control or ROS node is started.
if [[ "${1:-}" == --config ]]; then
  "${DOCKER[@]}" cp "$WS/config/curvature_signal.yaml" "$CONTAINER_NAME:$CONFIG"
elif "${DOCKER[@]}" cp "$CONTAINER_NAME:$CONFIG" - >/dev/null 2>&1; then
  echo 'Existing signal camera config preserved; use --config to copy the host file.'
else
  "${DOCKER[@]}" cp "$WS/config/curvature_signal.yaml" "$CONTAINER_NAME:$CONFIG"
fi
echo "Installed fixed curvature controller, signal fusion node and curvature_signal launch in $CONTAINER_NAME. No driving process was started."
echo "Camera dashboard installed. Backup: $BACKUP"
echo 'bash run_highway.sh start, then bash run_test.sh monitor 2 or drive 2.'
echo 'Ubuntu browser: http://127.0.0.1:8765'
