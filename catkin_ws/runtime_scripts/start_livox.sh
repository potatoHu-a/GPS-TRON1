#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"

if ! livox_package_path="$(rospack find livox_ros_driver2 2>/dev/null)"; then
  echo "[ERROR] livox_ros_driver2 is not visible in the runtime environment" >&2
  echo "        Set LIVOX_WS to the workspace containing its local_setup.bash." >&2
  exit 4
fi

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 4
fi

node_list="$(timeout 5 rosnode list)"
existing_nodes=()
for node_name in /livox_lidar_publisher2 /livox_driver; do
  if grep -Fxq "$node_name" <<<"$node_list"; then
    existing_nodes+=("$node_name")
  fi
done
for node_name in /laserMapping /fast_lio /fast_lio_localization_sc_qn_node /gps_fastlio_fusion; do
  if grep -Fxq "$node_name" <<<"$node_list"; then
    echo "[INFO] existing separate node detected: $node_name (left untouched)"
  fi
done
if ((${#existing_nodes[@]} > 0)); then
  echo "[INFO] Livox node(s) already running: ${existing_nodes[*]}"
  echo "       Livox driver will not be started again."
  exit 0
fi

launch_file="$livox_package_path/launch_ROS1/msg_MID360.launch"
if [[ ! -f "$launch_file" ]]; then
  echo "[ERROR] official Livox launch not found: $launch_file" >&2
  exit 5
fi

echo "[INFO] starting Livox driver only: $launch_file"
echo "[INFO] FAST-LIO, GPS fusion and navigation are not started by this script"
exec roslaunch "$launch_file" \
  rviz_enable:=false \
  rosbag_enable:=false \
  msg_frame_id:="${LIVOX_FRAME:-livox_frame}" \
  publish_freq:="${LIVOX_PUBLISH_FREQUENCY:-10.0}"
