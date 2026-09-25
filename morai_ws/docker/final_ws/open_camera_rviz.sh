#!/usr/bin/env bash
# A separate, disposable GUI container uses the current driving image.
# No launch file, camera UDP receiver, model inference or controller is started.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/highway.env"
: "${CONTAINER_NAME:?Set CONTAINER_NAME in highway.env}"
: "${UBUNTU_IP:?Set UBUNTU_IP in highway.env}"
[[ -n "${DISPLAY:-}" ]] || {
  echo 'DISPLAY is empty. Run from the Ubuntu desktop terminal, without sudo.' >&2; exit 2;
}
command -v xauth >/dev/null 2>&1 || {
  echo 'Install the host GUI helper first: sudo apt-get install xauth' >&2; exit 2;
}
[[ -d /tmp/.X11-unix ]] || {
  echo 'No X11/XWayland socket. Open an Ubuntu desktop session first.' >&2; exit 2;
}
DOCKER=(docker --context default)
if ! "${DOCKER[@]}" info >/dev/null 2>&1; then DOCKER=(sudo docker --context default); fi
[[ "$("${DOCKER[@]}" inspect --format '{{.State.Running}}' "$CONTAINER_NAME")" == true ]] || {
  echo 'Start the driving container and monitor 2 or drive 2 first.' >&2; exit 2;
}
# Use the exact existing image ID; no image download/build or changes to the driving container.
IMAGE_ID="$("${DOCKER[@]}" inspect --format '{{.Image}}' "$CONTAINER_NAME")"
ROS_MASTER="$("${DOCKER[@]}" exec "$CONTAINER_NAME" printenv ROS_MASTER_URI)"
[[ -n "$ROS_MASTER" ]] || { echo 'Driving container has no ROS_MASTER_URI.' >&2; exit 2; }
AUTH_FILE="$(mktemp /tmp/morai-camera-rviz-xauth.XXXXXXXX)"
trap 'rm -f -- "$AUTH_FILE"' EXIT
chmod 600 "$AUTH_FILE"
COOKIE="$(xauth nlist "$DISPLAY")"
[[ -n "$COOKIE" ]] || {
  echo 'No desktop Xauthority cookie. Run without sudo in the logged-in Ubuntu desktop terminal.' >&2; exit 2;
}
# FamilyWild permits the same display cookie in the viewer's separate hostname.
printf '%s\n' "$COOKIE" | sed 's/^..../ffff/' | xauth -f "$AUTH_FILE" nmerge -
unset COOKIE
echo "RViz CAM1 + CAM4; ROS master=$ROS_MASTER"
echo 'Close RViz to stop only this viewer. The driving launch stays running.'
"${DOCKER[@]}" run --rm --init --network host \
  --user "$(id -u):$(id -g)" --cap-drop ALL --security-opt no-new-privileges \
  -e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/camera-viewer.xauth \
  -e ROS_HOME=/tmp/ros -e XDG_CONFIG_HOME=/tmp/camera-config -e XDG_CACHE_HOME=/tmp/camera-cache \
  -e "ROS_IP=$UBUNTU_IP" -e "ROS_MASTER_URI=$ROS_MASTER" \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_QPA_PLATFORM=xcb \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=$AUTH_FILE,dst=/tmp/camera-viewer.xauth,readonly" \
  --mount "type=bind,src=$SCRIPT_DIR/cameras.rviz,dst=/tmp/cameras.rviz,readonly" \
  --entrypoint /bin/bash "$IMAGE_ID" -c \
  'source /opt/ros/noetic/setup.bash && exec rosrun rviz rviz -d /tmp/cameras.rviz'
