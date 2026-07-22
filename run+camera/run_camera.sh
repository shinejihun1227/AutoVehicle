#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "\${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "\${SCRIPT_DIR}/.." && pwd)"
cd "\${PROJECT_DIR}"

MORAI_PC_IP="\${MORAI_PC_IP:-192.168.0.151}"
CAMERA_PORT="\${CAMERA_PORT:-1101}"
ROI_PATH="\${ROI_PATH:-\${HOME}/ROI/src/path_planning/data/2026_molit_comp_global_path.txt}"

source /opt/ros/noetic/setup.bash
source "\${HOME}/ROI/devel/setup.bash"

exec python3 "run+camera/morai_pure_pursuit_camera_udp.py" \\
  --bind-ip "\${ALGORITHM_BIND_IP:-0.0.0.0}" \\
  --control-ip "\${MORAI_PC_IP}" \\
  --camera-port "\${CAMERA_PORT}" \\
  --path "\${ROI_PATH}" \\
  "$@"
