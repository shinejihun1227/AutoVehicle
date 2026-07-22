#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ROS_WS="$SCRIPT_DIR/../ros_ws_lidar"
CAMERA_PORT="${CAMERA_PORT:-1101}"
LIDAR_PORT="${LIDAR_PORT:-2001}"
BRIDGE_PID=""
CAMERA_PID=""

cleanup() {
  if [ -n "$CAMERA_PID" ]; then
    kill "$CAMERA_PID" 2>/dev/null || true
    wait "$CAMERA_PID" 2>/dev/null || true
  fi
  if [ -n "$BRIDGE_PID" ]; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash

if [ ! -f "$ROS_WS/devel/setup.bash" ]; then
  echo "LiDAR ROS workspace is not built. Building it now..."
  (cd "$ROS_WS" && catkin_make)
fi
source "$ROS_WS/devel/setup.bash"

python3 "$SCRIPT_DIR/camera_monitor.py" \
  --bind-ip 0.0.0.0 \
  --port "$CAMERA_PORT" \
  --display \
  --save-dir /tmp/morai_front \
  > /tmp/morai_camera_monitor.log 2>&1 &
CAMERA_PID=$!

roslaunch yeoms_lidar_bringup lidar_udp_only.launch \
  lidar_port:="$LIDAR_PORT" \
  > /tmp/morai_lidar_roslaunch.log 2>&1 &
BRIDGE_PID=$!

sleep 3
echo "Starting RViz and camera display."
echo "Camera port: $CAMERA_PORT"
echo "LiDAR topic: /sensors/lidar/points"
echo "Camera log: /tmp/morai_camera_monitor.log"
echo "LiDAR ROS log: /tmp/morai_lidar_roslaunch.log"
rviz -d "$SCRIPT_DIR/lidar_manual.rviz"
