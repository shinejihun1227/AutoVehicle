#!/usr/bin/env bash
# Run from an extracted signal-update bundle; copy sources only, never drive.
set -euo pipefail
[[ $# == 1 ]] || { echo 'Usage: bash apply_signal_update.sh /path/to/AutoVehicle_roundabout' >&2; exit 2; }
BUNDLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TARGET_DIR="$(cd -- "$1" && pwd -P)"
[[ "$(git -C "$TARGET_DIR" branch --show-current)" == test/roundabout ]] || {
  echo 'Select a test/roundabout checkout; other branches are not modified.' >&2; exit 2;
}
[[ "$(cd -- "$(git -C "$TARGET_DIR" rev-parse --show-toplevel)" && pwd -P)" == "$TARGET_DIR" ]] || {
  echo 'Pass the repository root, not morai_ws or docker/final_ws.' >&2; exit 2;
}
cd -- "$BUNDLE_DIR"
sha256sum --check SHA256SUMS
mapfile -t FILES < signal_update_files.txt
[[ ${#FILES[@]} == 10 ]] || { echo 'Unexpected update file list' >&2; exit 2; }
for file in "${FILES[@]}"; do
  [[ "$file" == morai_ws/src/* && "$file" != *..* && -f "$TARGET_DIR/$file" ]] || {
    echo "Missing/unexpected target: $file" >&2; exit 2;
  }
  [[ "$(realpath -- "$TARGET_DIR/$file")" == "$TARGET_DIR/"* ]] || {
    echo "Target leaves repository: $file" >&2; exit 2;
  }
done
mkdir -p "$TARGET_DIR/morai_ws/test-results"
BACKUP_DIR="$(mktemp -d "$TARGET_DIR/morai_ws/test-results/signal-backup.XXXXXX")"
for file in "${FILES[@]}"; do
  mkdir -p "$BACKUP_DIR/$(dirname -- "$file")"
  cp -p -- "$TARGET_DIR/$file" "$BACKUP_DIR/$file"
done
for file in "${FILES[@]}"; do
  cp -- "$BUNDLE_DIR/$file" "$TARGET_DIR/$file"
  chmod 644 -- "$TARGET_DIR/$file"
  if [[ "$file" == */scripts/*.py ]]; then chmod 755 -- "$TARGET_DIR/$file"; fi
done
echo "Updated ten host source files. Backup: $BACKUP_DIR"
echo 'Stop the driving launch, then stop the container and run install_curvature_signal.sh before restarting.'
