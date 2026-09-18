#!/usr/bin/env bash

set -Eeuo pipefail

MODE="${1:-tcp}"
ROS_SETUP="/opt/ros/noetic/setup.bash"

case "$MODE" in
  tcp)
    LAUNCH_FILE="netgps.launch"
    ;;
  udp)
    LAUNCH_FILE="phone_gps_test.launch"
    ;;
  -h|--help)
    echo "Usage: start_phone_gps.sh [tcp|udp]"
    exit 0
    ;;
  *)
    echo "[ERROR] unsupported GPS mode: $MODE (expected tcp or udp)" >&2
    exit 2
    ;;
esac

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
  exit 2
fi

PREVIOUS_PACKAGE_PATH="$(rospack find phone_gps_bridge 2>/dev/null || true)"

set +u
source "$ROS_SETUP"
set -u

source_overlay() {
  local candidate
  local dynamic_workspace=""
  if [[ -n "$PREVIOUS_PACKAGE_PATH" ]]; then
    dynamic_workspace="$(cd "$PREVIOUS_PACKAGE_PATH/../.." 2>/dev/null && pwd || true)"
  fi
  for candidate in \
    /home/guest/catkin_ws/devel_isolated/phone_gps_bridge/setup.bash \
    /home/guest/catkin_ws/devel_isolated/setup.bash \
    "${GPS_TRON_SETUP:-}" \
    /root/catkin_ws/devel_isolated/phone_gps_bridge/setup.bash \
    /root/catkin_ws/devel_isolated/setup.bash \
    "${dynamic_workspace:+${dynamic_workspace}/devel_isolated/phone_gps_bridge/setup.bash}" \
    "${dynamic_workspace:+${dynamic_workspace}/devel_isolated/setup.bash}"; do
    [[ -n "$candidate" && -f "$candidate" ]] || continue
    set +u
    source "$candidate"
    set -u
    echo "[OK] sourced overlay: $candidate"
    return 0
  done
  return 1
}

source_overlay || true
if ! rospack find phone_gps_bridge >/dev/null 2>&1; then
  echo "[ERROR] phone_gps_bridge is not visible in the sourced overlays" >&2
  exit 3
fi

echo "[INFO] starting phone GPS mode=$MODE with $LAUNCH_FILE"
echo "[INFO] verify in another terminal: rostopic hz /gps/fix"

exec roslaunch phone_gps_bridge "$LAUNCH_FILE"
