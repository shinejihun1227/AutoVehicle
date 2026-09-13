#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
if [[ -f "$SCRIPT_DIR/highway.env" ]]; then source "$SCRIPT_DIR/highway.env"; fi
UBUNTU_IP="${UBUNTU_IP:-192.168.0.185}"
MORAI_IP="${MORAI_IP:-192.168.0.148}"
CONTAINER_NAME="${CONTAINER_NAME:-morai-highway}"
IMAGE_NAME="${IMAGE_NAME:-morai-final:highway}"
TORCH_FLAVOR="${TORCH_FLAVOR:-cpu}"
# This two-PC setup uses the Ubuntu host Engine, including its LAN and GPU.
# A Docker Desktop context points at a different daemon with different containers.
DOCKER=(docker --context default)
if ! "${DOCKER[@]}" info >/dev/null 2>&1; then DOCKER=(sudo docker --context default); fi
MODE="${1:-help}"
[[ $# -eq 0 ]] || shift
case "$MODE" in
  build)
    MSGS_SHA="${MORAI_MSGS_REF:-$(git ls-remote https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git refs/heads/beta_drive | awk 'NR==1 {print $1}')}"
    [[ "$MSGS_SHA" =~ ^[0-9a-fA-F]{40}$ ]] || { echo 'Cannot resolve beta_drive SHA'; exit 1; }
    "${DOCKER[@]}" build -f "$SCRIPT_DIR/Dockerfile" -t "$IMAGE_NAME" \
      --build-arg "MORAI_MSGS_REF=$MSGS_SHA" --build-arg "TORCH_FLAVOR=$TORCH_FLAVOR" \
      --build-arg "CODE_REVISION=$(git -C "$WS" rev-parse HEAD)" "$WS"
    ;;
  start)
    if "${DOCKER[@]}" container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
      "${DOCKER[@]}" start "$CONTAINER_NAME"
      echo 'Existing container started. Changed image/IP/GPU settings require a new CONTAINER_NAME.'
      exit 0
    fi
    GPU_ARGS=()
    if [[ "$TORCH_FLAVOR" == cu121 ]]; then GPU_ARGS=(--gpus all); fi
    "${DOCKER[@]}" run -d --init --name "$CONTAINER_NAME" --network host --shm-size 1g \
      "${GPU_ARGS[@]}" -e "ROS_IP=$UBUNTU_IP" -e "ROS_MASTER_URI=http://$UBUNTU_IP:11311" \
      -e "MORAI_IP=$MORAI_IP" -v morai-highway-logs:/root/.ros \
      "$IMAGE_NAME" sleep infinity
    ;;
  shell)
    "${DOCKER[@]}" exec -it "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint bash --noprofile --norc
    ;;
  monitor|drive)
    CONTROL=false; [[ "$MODE" != drive ]] || CONTROL=true
    # YOLO's OpenCV display runs on a virtual screen; no xhost or host DISPLAY needed.
    "${DOCKER[@]}" exec -it "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
      xvfb-run -a roslaunch morai_bringup final_ws_highway_bringup.launch \
      "morai_host_ip:=$MORAI_IP" "enable_control:=$CONTROL" "$@"
    ;;
  test)
    "${DOCKER[@]}" exec "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
      python /opt/AutoVehicle/morai_ws/docker/final_ws/run_regression.py
    "${DOCKER[@]}" exec "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
      python /opt/AutoVehicle/morai_ws/docker/final_ws/check_highway_launch.py
    "${DOCKER[@]}" exec "$CONTAINER_NAME" /usr/local/bin/morai-entrypoint \
      python /opt/AutoVehicle/morai_ws/docker/final_ws/smoke_highway_model.py
    ;;
  stop)
    "${DOCKER[@]}" stop --time 10 "$CONTAINER_NAME"
    ;;
  *)
    echo 'Usage: bash run_highway.sh {build|start|shell|monitor|drive|test|stop} [roslaunch name:=value ...]'
    echo 'Use monitor first; drive explicitly enables MORAI UDP control.'
    ;;
esac
