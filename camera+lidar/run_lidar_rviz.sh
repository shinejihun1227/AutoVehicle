#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
ROS_WS="$PROJECT_DIR/ros_ws_lidar"
LIDAR_PORT="${LIDAR_PORT:-2001}"
BRIDGE_PID=""

cleanup() {
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

roslaunch yeoms_lidar_bringup lidar_udp_only.launch \
  lidar_port:="$LIDAR_PORT" \
  > /tmp/morai_lidar_roslaunch.log 2>&1 &
BRIDGE_PID=$!

sleep 3
echo "Starting RViz. LiDAR topic: /sensors/lidar/points"
echo "ROS log: /tmp/morai_lidar_roslaunch.log"
rviz -d "$SCRIPT_DIR/lidar_manual.rviz"
