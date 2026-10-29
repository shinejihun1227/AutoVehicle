#!/usr/bin/env bash
# Host-only environment preparation. Never starts ROS, Docker or vehicle control.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE="${1:-$HOME/AutoVehicle_new_final_ws/morai_ws/docker/final_ws}"
LIDAR_PRESET="${2:-preserve}"
[[ "$LIDAR_PRESET" == preserve || "$LIDAR_PRESET" == --lidar-bdt ]] || {
  echo 'Usage: bash prepare_ji_ujin.sh [old-final_ws-directory] [--lidar-bdt]' >&2; exit 2;
}
for name in highway.env highway-test.env; do
  if [[ ! -f "$SCRIPT_DIR/$name" ]]; then
    if [[ -f "$SOURCE/$name" ]]; then
      cp -- "$SOURCE/$name" "$SCRIPT_DIR/$name"
    else
      cp -- "$SCRIPT_DIR/$name.example" "$SCRIPT_DIR/$name"
      echo "No saved $name at source; using the new_final_ws example defaults."
    fi
  fi
  cp -p -- "$SCRIPT_DIR/$name" "$SCRIPT_DIR/$name.before-ji-ujin-$(date +%Y%m%d-%H%M%S)-$$"
done
python3 - "$SCRIPT_DIR" "$LIDAR_PRESET" <<'PY'
from pathlib import Path
import re
import sys
directory=Path(sys.argv[1])
def update(name, fields):
    path=directory/name
    text=path.read_text()
    for key,value in fields.items():
        pattern=r'(?m)^'+re.escape(key)+r'=.*$'
        replacement=key+'='+value
        if re.search(pattern,text):text=re.sub(pattern,lambda _:replacement,text)
        else:text=text.rstrip()+'\n'+replacement+'\n'
    path.write_text(text,encoding='utf-8')
update('highway.env',{'CONTAINER_NAME':'morai-ji-ujin-net10','IMAGE_NAME':'morai-ji-ujin:latest'})
fields={'LANE_INFO_DEVICE':'auto','ROUNDABOUT_ENABLED':'true',
        'TUNNEL_LANE_PRIORITY_ENABLED':'true','TUNNEL_MAX_SPEED_KPH':'25.0'}
if sys.argv[2]=='--lidar-bdt':
    fields.update(LIDAR_X_M='1.22',LIDAR_Y_M='0.0',LIDAR_Z_M='1.23',LIDAR_YAW_DEG='0.0')
update('highway-test.env',fields)
print('Prepared separate ji_ujin container/image; saved IPs, ports, speed and stop clearance preserved.')
print('LiDAR mount: '+('BDT preset x=1.22 y=0 z=1.23 yaw=0 applied.' if sys.argv[2]=='--lidar-bdt' else 'saved values preserved; compare with MORAI sensor settings.'))
PY
echo 'Next: python3 fetch_highway_model.py; bash run_highway.sh build'
