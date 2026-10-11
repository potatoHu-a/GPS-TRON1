#!/usr/bin/env bash

set -Eeuo pipefail

ROS_SETUP="/opt/ros/noetic/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
  exit 2
fi

PREVIOUS_PACKAGE_PATH="$(rospack find tron_open_space_nav 2>/dev/null || true)"

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
    /home/guest/catkin_ws/devel_isolated/tron_open_space_nav/setup.bash \
    /home/guest/catkin_ws/devel_isolated/setup.bash \
    "${GPS_TRON_SETUP:-}" \
    /root/catkin_ws/devel_isolated/tron_open_space_nav/setup.bash \
    /root/catkin_ws/devel_isolated/setup.bash \
    "${dynamic_workspace:+${dynamic_workspace}/devel_isolated/tron_open_space_nav/setup.bash}" \
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
if ! rospack find tron_open_space_nav >/dev/null 2>&1; then
  echo "[ERROR] tron_open_space_nav is not visible in the sourced overlays" >&2
  exit 3
fi

echo "[INFO] starting GPS navigation with dry_run:=true"
echo "[WARN] gps_global_nav.launch may start /lidar_obstacle_avoid."
echo "       Start collision safety next; it will stop the legacy node automatically."

exec roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false \
  use_fake_fastlio:=false \
  dry_run:=true
