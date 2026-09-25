#!/usr/bin/env bash
# Host browser only. Never starts ROS, a camera receiver, or vehicle control.
set -euo pipefail
URL=http://127.0.0.1:8765/
if [[ "${1:-}" == --wait ]]; then
  for ((attempt=0; attempt<30; attempt++)); do
    if curl --silent --fail --max-time 1 "${URL}api/status" >/dev/null 2>&1; then break; fi
    sleep 1
  done
fi
echo "CAM1 + CAM4: $URL"
if command -v xdg-open >/dev/null 2>&1 && [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  xdg-open "$URL" >/dev/null 2>&1 || true
fi
