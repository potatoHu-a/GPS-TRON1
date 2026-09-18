#!/usr/bin/env bash

set -Eeuo pipefail

ROS_SETUP="/opt/ros/noetic/setup.bash"
ROBOT_MASTER_IP="10.192.1.3"
DEFAULT_MASTER_URI="http://${ROBOT_MASTER_IP}:11311"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
  exit 2
fi

PREVIOUS_PACKAGE_PATH="$(rospack find tron_open_space_nav 2>/dev/null || true)"

set +u
source "$ROS_SETUP"
set -u

source_first_visible_overlay() {
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

source_first_visible_overlay || true

if ! PACKAGE_PATH="$(rospack find tron_open_space_nav 2>/dev/null)"; then
  echo "[ERROR] tron_open_space_nav is not visible in the sourced overlays" >&2
  echo "        Set GPS_TRON_SETUP to the applicable setup.bash path." >&2
  exit 3
fi

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  export ROS_MASTER_URI="$DEFAULT_MASTER_URI"
fi
unset ROS_HOSTNAME

if [[ -z "${ROS_IP:-}" ]]; then
  if ! command -v ip >/dev/null 2>&1; then
    echo "[ERROR] ip command is unavailable; export ROS_IP manually" >&2
    exit 4
  fi
  ROS_IP="$(ip route get "$ROBOT_MASTER_IP" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
  if [[ -z "$ROS_IP" ]]; then
    echo "[ERROR] cannot determine the local IPv4 route to $ROBOT_MASTER_IP" >&2
    echo "        Export ROS_IP manually and run the script again." >&2
    exit 4
  fi
  export ROS_IP
fi

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 5
fi

if [[ -d "$PACKAGE_PATH/maps/wuhan_tiles" ]]; then
  TILE_ROOT="$PACKAGE_PATH/maps/wuhan_tiles"
elif [[ -d "$PACKAGE_PATH/maps" ]]; then
  TILE_ROOT="$PACKAGE_PATH/maps"
else
  echo "[ERROR] no tile directory found under $PACKAGE_PATH/maps" >&2
  exit 6
fi

echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[INFO] ROS_IP=$ROS_IP"
echo "[INFO] tile_root=$TILE_ROOT"

exec roslaunch tron_open_space_nav mapviz_ground_station.launch \
  tile_root:="$TILE_ROOT" \
  launch_mapviz:=true
