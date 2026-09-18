#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"

mode="${1:-tcp}"
if (($# > 1)); then
  echo "Usage: $0 [tcp|udp]" >&2
  exit 2
fi

case "$mode" in
  tcp)
    launch_file="netgps.launch"
    ;;
  udp)
    launch_file="phone_gps_test.launch"
    ;;
  -h|--help)
    echo "Usage: $0 [tcp|udp]"
    exit 0
    ;;
  *)
    echo "[ERROR] unsupported GPS mode: $mode (expected tcp or udp)" >&2
    exit 2
    ;;
esac

rospack find phone_gps_bridge >/dev/null
echo "[INFO] starting phone GPS mode=$mode"
echo "[INFO] verify in another terminal: rostopic hz /gps/fix"
exec roslaunch phone_gps_bridge "$launch_file"
