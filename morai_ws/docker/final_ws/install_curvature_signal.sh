#!/usr/bin/env bash
# Install the complete profile 2 source dependency set into a stopped container.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --config ) ]]; then
  echo 'Usage: bash install_curvature_signal.sh [--config]' >&2
  exit 2
fi
[[ -f "$SCRIPT_DIR/highway.env" ]] || { echo 'Missing highway.env' >&2; exit 2; }
source "$SCRIPT_DIR/highway.env"
source "$SCRIPT_DIR/curvature_runtime_files.sh"
: "${CONTAINER_NAME:?Set CONTAINER_NAME in highway.env}"
DOCKER=(docker --context default)
if ! "${DOCKER[@]}" info >/dev/null 2>&1; then DOCKER=(sudo docker --context default); fi
RUNNING="$("${DOCKER[@]}" inspect --format '{{.State.Running}}' "$CONTAINER_NAME")"
[[ "$RUNNING" == false ]] || {
  echo 'Stop the launch, then bash run_highway.sh stop before updating. Restart after installation.' >&2
  exit 2
}
DEST=/opt/AutoVehicle/morai_ws
CAM=src/detection/camera_perception
FILE_LIST="$(curvature_runtime_files "$WS")"
mapfile -t FILES <<< "$FILE_LIST"
for file in "${FILES[@]}" config/curvature_signal.yaml; do
  [[ -f "$WS/$file" ]] || { echo "Missing host file: $WS/$file" >&2; exit 2; }
done
printf '%s\n' \
  '1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a  '"$WS/$CAM/models/yolov8s.pt" \
  '6812d43beda5ab6ff18881198f34a49f79d641efbf2768220b34c74dd11aa9f5  '"$WS/$CAM/models/best0917.pt" \
  | sha256sum --check --status || { echo 'CAM4 checkpoint checksum mismatch.' >&2; exit 2; }
BACKUP="$(mktemp -d "$HOME/morai-curvature-backup.XXXXXX")"
STAGE="$(mktemp -d "${TMPDIR:-/tmp}/morai-curvature-update.XXXXXX")"
trap 'rm -rf -- "$STAGE"' EXIT
REVISION="$(git -C "$WS" rev-parse HEAD)"
INSTALL_STATE=docker/final_ws/curvature-runtime-install.txt
MANIFEST=docker/final_ws/curvature-runtime.sha256
# Back up every existing file first. Missing files in an older image are expected.
for file in "${FILES[@]}" config/curvature_signal.yaml "$INSTALL_STATE" "$MANIFEST"; do
  mkdir -p "$BACKUP/$(dirname -- "$file")"
  if ! "${DOCKER[@]}" cp "$CONTAINER_NAME:$DEST/$file" "$BACKUP/$file" 2>"$STAGE/backup-error"; then
    if ! grep -Eq 'Could not find|No such file' "$STAGE/backup-error"; then
      cat "$STAGE/backup-error" >&2
      echo "Backup failed for $file; installation has not started." >&2
      exit 2
    fi
  fi
done
for file in "${FILES[@]}"; do
  mkdir -p "$STAGE/payload/$(dirname -- "$file")"
  cp -p -- "$WS/$file" "$STAGE/payload/$file"
done
# Source executables can run directly even before catkin generates wrappers.
find "$STAGE/payload/src" -path '*/scripts/*.py' -type f -exec chmod +x {} +
curvature_runtime_manifest "$WS" > "$STAGE/manifest"
cp -- "$STAGE/manifest" "$STAGE/payload/$MANIFEST"
printf 'state=pending\nrevision=%s\n' "$REVISION" > "$STAGE/state"
# This marker makes an interrupted update visible to run_test.sh.
"${DOCKER[@]}" cp "$STAGE/state" "$CONTAINER_NAME:$DEST/$INSTALL_STATE"
"${DOCKER[@]}" cp "$STAGE/payload/." "$CONTAINER_NAME:$DEST/"
CONFIG=config/curvature_signal.yaml
if [[ "${1:-}" == --config || ! -f "$BACKUP/$CONFIG" ]]; then
  "${DOCKER[@]}" cp "$WS/$CONFIG" "$CONTAINER_NAME:$DEST/$CONFIG"
  EXPECTED_CONFIG="$WS/$CONFIG"
  echo 'Installed selected route junction configuration.'
else
  EXPECTED_CONFIG="$BACKUP/$CONFIG"
  echo 'Existing signal configuration preserved (backup includes its exact contents).'
  if ! cmp -s "$WS/$CONFIG" "$BACKUP/$CONFIG"; then
    echo 'CONFIG_DIFF: container differs from repository; use --config for the repository junction settings.'
  fi
fi
# Read installed bytes back while the container remains stopped. Checking only
# the new launch/node misses stale imported helpers in a partially updated image.
mkdir -p "$STAGE/verify/config"
"${DOCKER[@]}" cp "$CONTAINER_NAME:$DEST/$CONFIG" "$STAGE/verify/$CONFIG"
cmp -- "$EXPECTED_CONFIG" "$STAGE/verify/$CONFIG"
# Record the exact active config too, including an intentionally preserved one.
(cd -- "$STAGE/verify" && sha256sum -- "$CONFIG") >> "$STAGE/manifest"
for file in "${FILES[@]}"; do
  mkdir -p "$STAGE/verify/$(dirname -- "$file")"
  "${DOCKER[@]}" cp "$CONTAINER_NAME:$DEST/$file" "$STAGE/verify/$file"
done
(cd -- "$STAGE/verify" && sha256sum --check --quiet "$STAGE/manifest")
"${DOCKER[@]}" cp "$STAGE/manifest" "$CONTAINER_NAME:$DEST/$MANIFEST"
printf 'state=ready\nrevision=%s\ninstalled_at=%s\nfiles=%s\n' \
  "$REVISION" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$((${#FILES[@]} + 1))" > "$STAGE/state"
"${DOCKER[@]}" cp "$STAGE/state" "$CONTAINER_NAME:$DEST/$INSTALL_STATE"
echo "Installed and verified ${#FILES[@]} runtime files and the active config in $CONTAINER_NAME (revision $REVISION)."
echo "Backup: $BACKUP"
echo 'No ROS or driving process was started. Start the container, then use show 2 and monitor 2 or drive 2.'
echo 'Ubuntu browser: http://127.0.0.1:8765'
