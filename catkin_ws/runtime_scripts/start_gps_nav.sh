#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"
rospack find tron_open_space_nav >/dev/null

echo "[INFO] starting GPS navigation with dry_run:=true"
echo "[WARN] gps_global_nav.launch may start /lidar_obstacle_avoid."
echo "       Start collision safety next; it will stop the legacy node."

exec roslaunch tron_open_space_nav gps_global_nav.launch \
  use_fake_gps:=false \
  use_fake_fastlio:=false \
  dry_run:=true
