#!/usr/bin/env bash
# Update Python/launch files in an existing, STOPPED highway container.
# No ROS messages, model, calibration, environment or image layers are changed.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
[[ $# -eq 0 ]] || { echo 'Usage: bash install_avoidance_transport.sh' >&2; exit 2; }
[[ -f "$SCRIPT_DIR/highway.env" ]] || { echo 'Missing highway.env' >&2; exit 2; }
source "$SCRIPT_DIR/highway.env"
: "${CONTAINER_NAME:?Set CONTAINER_NAME in highway.env}"
DOCKER=(docker --context default)
if ! "${DOCKER[@]}" info >/dev/null 2>&1; then DOCKER=(sudo docker --context default); fi
RUNNING="$("${DOCKER[@]}" inspect --format '{{.State.Running}}' "$CONTAINER_NAME")"
[[ "$RUNNING" == false ]] || {
  echo 'Stop the driving launch with Ctrl+C, then bash run_highway.sh stop before updating.' >&2
  exit 2
}
DEST=/opt/AutoVehicle/morai_ws
EXISTING=(
  src/control/purepursuit_mgeo/scripts/avoidance_frenet_debug_node.py
  src/control/purepursuit_mgeo/scripts/bypass_lane_guard_node.py
  src/control/purepursuit_mgeo/scripts/avoidance_path_manager_node.py
  src/control/purepursuit_mgeo/scripts/highway_lane_strategy_node.py
  src/experimental/curvature_speed_purepursuit/scripts/adaptive_curvature_purepursuit_node.py
  src/bringup/morai_bringup/launch/final_ws_highway_bringup.launch
  src/control/purepursuit_mgeo/test/test_highway_safety.py
  src/experimental/curvature_speed_purepursuit/test/test_adaptive_path.py
)
NEW=(
  src/control/purepursuit_mgeo/src/purepursuit_mgeo/plan_transport.py
  src/control/purepursuit_mgeo/test/test_plan_transport.py
  docker/final_ws/diagnose_highway.py
)
for file in "${EXISTING[@]}" "${NEW[@]}"; do
  [[ -f "$WS/$file" ]] || { echo "Missing host file: $WS/$file" >&2; exit 2; }
done
BACKUP="$HOME/morai-update-backups/$CONTAINER_NAME-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP"
# Verify the expected old workspace exists and back it up before the first write.
for file in "${EXISTING[@]}"; do
  mkdir -p "$BACKUP/$(dirname -- "$file")"
  "${DOCKER[@]}" cp "$CONTAINER_NAME:$DEST/$file" "$BACKUP/$file"
done
for file in "${NEW[@]}"; do
  mkdir -p "$BACKUP/$(dirname -- "$file")"
  "${DOCKER[@]}" cp "$CONTAINER_NAME:$DEST/$file" "$BACKUP/$file" 2>/dev/null || true
done
for file in "${EXISTING[@]}" "${NEW[@]}"; do
  "${DOCKER[@]}" cp "$WS/$file" "$CONTAINER_NAME:$DEST/$file"
done
echo "Installed atomic avoidance/merge transport in $CONTAINER_NAME. Backup: $BACKUP"
echo 'Container is still stopped. Start it, run the offline regressions, then monitor.'
