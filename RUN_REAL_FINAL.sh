#!/usr/bin/env bash
# Ubuntu host entry point. Each drive remains an explicit user action.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd -- "$ROOT/morai_ws/docker/final_ws"
for name in highway.env highway-test.env; do
  if [[ ! -f "$name" ]]; then cp -- "$name.example" "$name"; fi
done
case "${1:-show}" in
  setup) echo "Settings: $PWD/highway.env and $PWD/highway-test.env" ;;
  build)
    python3 fetch_highway_model.py
    bash run_highway.sh build
    ;;
  install)
    python3 fetch_highway_model.py
    bash run_highway.sh start
    bash run_highway.sh stop
    bash install_curvature_signal.sh
    bash run_highway.sh start
    ;;
  tune) bash run_test.sh tune real_final ;;
  network) python3 configure_real_final_network.py "${2:?Use personal or competition}" ;;
  speed) bash run_test.sh speed "${2:?Specify km/h}" ;;
  show|monitor|drive|diagnose) bash run_test.sh "${1:-show}" 2 ;;
  stop) bash run_highway.sh stop ;;
  *) echo 'Usage: bash RUN_REAL_FINAL.sh {setup|build|install|tune|show|monitor|drive|diagnose|stop|speed KMH|network personal/competition}' >&2; exit 2 ;;
esac
