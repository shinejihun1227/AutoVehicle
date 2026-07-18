#!/usr/bin/env bash
set -u

failures=0

check_command() {
  local name="$1"
  if command -v "$name" >/dev/null 2>&1; then
    echo "[OK] command: $name"
  else
    echo "[FAIL] command missing: $name"
    failures=$((failures + 1))
  fi
}

check_env() {
  local name="$1"
  if [ -n "${!name:-}" ]; then
    echo "[OK] env: $name=${!name}"
  else
    echo "[WARN] env missing: $name"
  fi
}

echo "== MORAI competition preflight check =="

check_command roscore
check_command rostopic
check_command roslaunch
check_command rosbag

check_env ROS_MASTER_URI
check_env ROS_IP
check_env ROS_HOSTNAME
check_env MORAI_COMPETITION_WS

if [ -f "$HOME/morai_competition_ws/devel/setup.bash" ]; then
  echo "[OK] catkin workspace setup found"
else
  echo "[FAIL] catkin workspace setup missing: $HOME/morai_competition_ws/devel/setup.bash"
  failures=$((failures + 1))
fi

echo "== Network interfaces =="
ip -brief addr || true

if [ "$failures" -eq 0 ]; then
  echo "Preflight result: PASS"
else
  echo "Preflight result: FAIL ($failures issue(s))"
fi

exit "$failures"

