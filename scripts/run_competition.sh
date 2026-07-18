#!/usr/bin/env bash
set -euo pipefail

WS="${MORAI_COMPETITION_WS:-$HOME/morai_competition_ws}"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

mkdir -p "${MORAI_COMPETITION_LOG_DIR:-$HOME/morai_competition_logs}"

roslaunch morai_competition_bringup competition.launch "$@"

