#!/usr/bin/env bash
# Host browser only. Never starts ROS, a camera receiver, or vehicle control.
set -euo pipefail
URL=http://127.0.0.1:8765/
command -v curl >/dev/null 2>&1 || {
  echo "curl is missing. Open $URL manually; or install curl on the Ubuntu host." >&2; exit 2;
}
if [[ "${1:-}" == --wait ]]; then
  for ((attempt=0; attempt<30; attempt++)); do
    if curl --silent --fail --max-time 1 "${URL}api/status" >/dev/null 2>&1; then break; fi
    sleep 1
  done
fi
if ! curl --silent --fail --max-time 2 "${URL}api/status" >/dev/null 2>&1; then
  echo "Camera dashboard is not responding at $URL" >&2
  echo 'Keep monitor 2 or drive 2 running. Check camera_debug_dashboard errors or apply the camera update.' >&2
  exit 1
fi
echo "CAM1 + CAM4: $URL"
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || {
  echo "No desktop session. Open $URL manually on Ubuntu." >&2; exit 2;
}
[[ "$EUID" != 0 ]] || {
  echo "Run 'bash run_test.sh view' WITHOUT sudo in a new Ubuntu desktop terminal." >&2; exit 2;
}
if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$URL" && exit 0
elif command -v gio >/dev/null 2>&1; then
  gio open "$URL" && exit 0
fi
echo "Browser could not be opened. Enter $URL in the Ubuntu browser address bar." >&2
exit 1
