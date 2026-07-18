#!/usr/bin/env bash
set -u

topics=(
  "/sensors/gps/fix"
  "/sensors/imu/data"
  "/sensors/lidar/front/points"
  "/morai/vehicle_status"
  "/control/ctrl_cmd"
)

for topic in "${topics[@]}"; do
  echo "== $topic =="
  if rostopic list | grep -Fx "$topic" >/dev/null 2>&1; then
    timeout 3s rostopic hz "$topic" || true
  else
    echo "[MISS] topic not found"
  fi
done

