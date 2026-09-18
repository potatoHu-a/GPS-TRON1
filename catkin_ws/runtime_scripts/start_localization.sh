#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"

if ! localization_package_path="$(rospack find fast_lio_localization_sc_qn 2>/dev/null)"; then
  echo "[ERROR] fast_lio_localization_sc_qn is not visible in the runtime environment" >&2
  echo "        Set FASTLIO_WS to the workspace containing its local_setup.bash." >&2
  exit 4
fi

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 4
fi

node_list="$(timeout 5 rosnode list)"
existing_localization_nodes=()
for node_name in /laserMapping /fast_lio /fast_lio_localization_sc_qn_node; do
  if grep -Fxq "$node_name" <<<"$node_list"; then
    existing_localization_nodes+=("$node_name")
  fi
done
if ((${#existing_localization_nodes[@]} > 0)); then
  echo "[INFO] localization node(s) already running: ${existing_localization_nodes[*]}"
  echo "       FAST-LIO localization will not be started again."
  if grep -Fxq /gps_fastlio_fusion <<<"$node_list"; then
    echo "[INFO] /gps_fastlio_fusion is also already running; no duplicate is started."
  fi
  exit 0
fi

if grep -Fxq /gps_fastlio_fusion <<<"$node_list"; then
  echo "[INFO] /gps_fastlio_fusion already exists; it is left untouched."
  echo "       GPS fusion remains owned by the GPS navigation launch."
fi

launch_file="$localization_package_path/launch/run.launch"
if [[ ! -f "$launch_file" ]]; then
  echo "[ERROR] FAST-LIO localization launch not found: $launch_file" >&2
  exit 5
fi

echo "[INFO] starting FAST-LIO localization only: $launch_file"
echo "[INFO] Livox driver must be started separately with start_livox.sh"
echo "[INFO] GPS fusion remains owned by start_gps_nav.sh"
exec roslaunch "$launch_file" \
  rviz:="${FASTLIO_RVIZ:-false}" \
  lidar:="${LIDAR_MODEL:-livox_mid360}" \
  odom_topic:="${FASTLIO_ODOM_TOPIC:-/Odometry}" \
  lidar_topic:="${FASTLIO_REGISTERED_CLOUD_TOPIC:-/cloud_registered}"
