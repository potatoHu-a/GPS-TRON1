#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

export DISPLAY="${DISPLAY:-:1}"

robot_master_ip=10.192.1.3
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://${robot_master_ip}:11311}"
unset ROS_HOSTNAME

if [[ -z "${ROS_IP:-}" ]]; then
  ROS_IP="$(ip route get "$robot_master_ip" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
  if [[ -z "$ROS_IP" ]]; then
    echo "[ERROR] cannot determine ROS_IP; export it manually" >&2
    exit 4
  fi
  export ROS_IP
fi

if ! timeout 5 rosnode list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 5
fi

cloud_topic=""
for candidate in /livox/lidar_filter1 /cloud_registered /livox/lidar; do
  topic_type="$(timeout 3 rostopic type "$candidate" 2>/dev/null || true)"
  if [[ "$topic_type" == "sensor_msgs/PointCloud2" ]]; then
    cloud_topic="$candidate"
    break
  fi
done

if [[ -z "$cloud_topic" ]]; then
  cloud_topic=/livox/lidar_filter1
  echo "[WARN] no PointCloud2 topic is available yet; waiting on $cloud_topic" >&2
else
  echo "[OK] PointCloud2 topic=$cloud_topic"
fi

package_path="$(rospack find tron_open_space_nav)"
config_path="$package_path/config/rviz/rviz_tron_navigation_debug.rviz"
echo "[INFO] RViz config=$config_path"

exec rosrun rviz rviz -d "$config_path" \
  /livox/lidar_filter1:="$cloud_topic"
