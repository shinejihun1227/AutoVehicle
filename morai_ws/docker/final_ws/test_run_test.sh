#!/usr/bin/env bash
# Exercise the host recipes with a recording Docker stub; no ROS or UDP runs.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TMP_ROOT="$(cd -- "${TMPDIR:-/tmp}" && pwd -P)"
FIXTURE="$(mktemp -d "$TMP_ROOT/morai-recipes.XXXXXXXX")"
cleanup() {
  [[ "$FIXTURE" == "$TMP_ROOT"/morai-recipes.* &&
     "$(cd -- "$FIXTURE" && pwd -P)" == "$FIXTURE" ]] || return 1
  rm -rf -- "$FIXTURE"
}
trap cleanup EXIT
cp "$SCRIPT_DIR/"{run_test.sh,run_highway.sh,highway-test.env.example} "$FIXTURE/"
cp "$SCRIPT_DIR/highway.env.example" "$FIXTURE/highway.env"
cp "$SCRIPT_DIR/highway-test.env.example" "$FIXTURE/highway-test.env"
cp "$FIXTURE/highway-test.env" "$FIXTURE/original.env"
mkdir "$FIXTURE/bin"
export DOCKER_LOG="$FIXTURE/docker.log"
cat > "$FIXTURE/bin/docker" <<'STUB'
#!/usr/bin/env bash
printf '%s\n' "$@" >> "$DOCKER_LOG"
STUB
chmod +x "$FIXTURE/bin/docker"
export PATH="$FIXTURE/bin:$PATH"
fail() { echo "FAIL: $*" >&2; exit 1; }
has() { grep -qxF -- "$1" "$2" || fail "Missing $1"; }
absent() { if grep -qF -- "$1" "$2"; then fail "Unexpected $1"; fi; }

for n in 1 2 3 4 5; do
  bash "$FIXTURE/run_test.sh" show "$n" > "$FIXTURE/show-$n"
done
[[ ! -f "$DOCKER_LOG" ]] || fail 'show contacted Docker'
cmp "$FIXTURE/original.env" "$FIXTURE/highway-test.env"
has 'use_curvature_speed_planner:=true' "$FIXTURE/show-1"
absent 'enable_yolo' "$FIXTURE/show-1"
grep -q '^Launch=final_ws_curvature_signal.launch ' "$FIXTURE/show-2"
absent 'enable_obstacle_avoidance' "$FIXTURE/show-2"
for n in 3 4 5; do
  has 'enable_yolo:=true' "$FIXTURE/show-$n"
  has 'enable_stopline_control:=true' "$FIXTURE/show-$n"
done
has 'enable_obstacle_avoidance:=true' "$FIXTURE/show-3"
has 'enable_highway_lane_change:=false' "$FIXTURE/show-3"
has 'enable_obstacle_avoidance:=false' "$FIXTURE/show-4"
has 'enable_highway_lane_change:=true' "$FIXTURE/show-4"
has 'enable_obstacle_avoidance:=true' "$FIXTURE/show-5"
has 'enable_highway_lane_change:=true' "$FIXTURE/show-5"

# Exercise actual Docker argv generation, including the run_highway delegation.
for action in monitor drive; do
  control=false; [[ "$action" != drive ]] || control=true
  for n in 1 2 3 4 5; do
    : > "$DOCKER_LOG"
    bash "$FIXTURE/run_test.sh" "$action" "$n" > /dev/null
    has "enable_control:=$control" "$DOCKER_LOG"
    has 'morai_host_ip:=192.168.0.147' "$DOCKER_LOG"
    has 'max_speed_kph:=5.0' "$DOCKER_LOG"
    case "$n" in
      1) has 'morai_udp_ekf_purepursuit.launch' "$DOCKER_LOG" ;;
      2) has 'final_ws_curvature_signal.launch' "$DOCKER_LOG" ;;
      3|4|5) has 'final_ws_highway_bringup.launch' "$DOCKER_LOG" ;;
    esac
  done
done

for old in curvature curvature_signal full obstacle merge; do
  sed -i "s/^TEST_PROFILE=.*/TEST_PROFILE=$old/" "$FIXTURE/highway-test.env"
  bash "$FIXTURE/run_test.sh" show > "$FIXTURE/legacy"
  grep -q "^Profile=$old " "$FIXTURE/legacy"
done
bash "$FIXTURE/run_test.sh" speed 12.5 > /dev/null
for n in 1 2 3 4 5; do
  bash "$FIXTURE/run_test.sh" show "$n" > "$FIXTURE/speed"
  has 'max_speed_kph:=12.5' "$FIXTURE/speed"
done
has 'LATERAL_ACCEL_LIMIT_MPS2=1.0' "$FIXTURE/highway-test.env"
has 'MAX_SPEED_KPH=5.0' "$FIXTURE/highway-test.env.bak"
for bad in 0 -1 NaN '10;echo bad' ''; do
  if bash "$FIXTURE/run_test.sh" speed "$bad" > /dev/null 2>&1; then fail "Accepted speed $bad"; fi
done
if bash "$FIXTURE/run_test.sh" drive 6 > /dev/null 2>&1; then fail 'Accepted case 6'; fi
: > "$DOCKER_LOG"
bash "$FIXTURE/run_test.sh" request-merge > /dev/null
has 'rostopic' "$DOCKER_LOG"
has '/planning/highway_lane_change_request' "$DOCKER_LOG"
has 'data: true' "$DOCKER_LOG"
echo 'RUN_TEST_RECIPES_PASS (Docker mocked; no driving)'
