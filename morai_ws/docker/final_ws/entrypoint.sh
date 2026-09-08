#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/morai-venv/bin/activate
source /opt/AutoVehicle/morai_ws/devel/setup.bash
exec "$@"
