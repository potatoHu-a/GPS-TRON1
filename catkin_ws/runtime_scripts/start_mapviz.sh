#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

robot_master_ip="10.192.1.3"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://${robot_master_ip}:11311}"
unset ROS_HOSTNAME

if [[ -z "${ROS_IP:-}" ]]; then
  if ! command -v ip >/dev/null 2>&1; then
    echo "[ERROR] ip command is unavailable; export ROS_IP manually" >&2
    exit 4
  fi
  ROS_IP="$(ip route get "$robot_master_ip" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
  if [[ -z "$ROS_IP" ]]; then
    echo "[ERROR] cannot determine the local IPv4 route to $robot_master_ip" >&2
    echo "        Export ROS_IP manually and run this script again." >&2
    exit 4
  fi
  export ROS_IP
fi

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 5
fi

package_path="$(rospack find tron_open_space_nav)"
if [[ -d "$package_path/maps/wuhan_tiles" ]]; then
  tile_root="$package_path/maps/wuhan_tiles"
elif [[ -d "$package_path/maps" ]]; then
  tile_root="$package_path/maps"
else
  echo "[ERROR] no tile directory found below $package_path/maps" >&2
  exit 6
fi

echo "[INFO] TRON_WS=$TRON_WS"
echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[INFO] ROS_IP=$ROS_IP"
echo "[INFO] tile_root=$tile_root"
exec roslaunch tron_open_space_nav mapviz_ground_station.launch \
  tile_root:="$tile_root" \
  launch_mapviz:=true
