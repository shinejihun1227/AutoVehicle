#!/usr/bin/env bash
# Add only the new launch/config to an existing final_ws container. No rebuild.
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
"${DOCKER[@]}" container inspect "$CONTAINER_NAME" >/dev/null
"${DOCKER[@]}" cp "$WS/src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch" \
  "$CONTAINER_NAME:/opt/AutoVehicle/morai_ws/src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch"
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
echo "Installed curvature_signal launch in $CONTAINER_NAME. No driving process was started."
echo 'Set TEST_PROFILE=curvature_signal, then bash run_test.sh show / monitor / drive.'
