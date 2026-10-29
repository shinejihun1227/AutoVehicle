#!/usr/bin/env bash
# Repair only the ten transferred host sources; never start Docker or driving.
set -euo pipefail
[[ $# == 1 ]] || { echo 'Usage: bash repair_signal_permissions.sh /path/to/AutoVehicle_roundabout' >&2; exit 2; }
BUNDLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TARGET_DIR="$(cd -- "$1" && pwd -P)"
[[ "$(git -C "$TARGET_DIR" branch --show-current)" == test/roundabout ]] || {
  echo 'Select the test/roundabout checkout.' >&2; exit 2;
}
[[ "$(cd -- "$(git -C "$TARGET_DIR" rev-parse --show-toplevel)" && pwd -P)" == "$TARGET_DIR" ]] || {
  echo 'Pass the repository root.' >&2; exit 2;
}
mapfile -t FILES < "$BUNDLE_DIR/signal_update_files.txt"
[[ ${#FILES[@]} == 10 ]] || { echo 'Unexpected update file list' >&2; exit 2; }
# Validate the complete set before changing ownership or permissions.
for file in "${FILES[@]}"; do
  [[ "$file" == morai_ws/src/* && "$file" != *..* && -f "$TARGET_DIR/$file" && ! -L "$TARGET_DIR/$file" ]] || {
    echo "Missing/unexpected target: $file" >&2; exit 2;
  }
  [[ "$(realpath -- "$TARGET_DIR/$file")" == "$TARGET_DIR/"* ]] || {
    echo "Target leaves repository: $file" >&2; exit 2;
  }
done
[[ $(id -u) != 0 ]] || { echo 'Run with bash as your Ubuntu user, not sudo bash.' >&2; exit 2; }
OWNER="$(id -u):$(id -g)"
for file in "${FILES[@]}"; do
  if [[ ! -O "$TARGET_DIR/$file" ]]; then sudo chown -- "$OWNER" "$TARGET_DIR/$file"; fi
  chmod 644 -- "$TARGET_DIR/$file"
  if [[ "$file" == */scripts/*.py ]]; then chmod 755 -- "$TARGET_DIR/$file"; fi
done
echo 'Repaired permissions for ten host sources. File contents and local env settings were not changed.'
echo 'Stop the container, then rerun install_curvature_signal.sh as your Ubuntu user.'
