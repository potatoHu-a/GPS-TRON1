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

origin_type="$(timeout 3 rostopic type /local_xy_origin 2>/dev/null || true)"
if [[ "$origin_type" == "geometry_msgs/PoseStamped" ]]; then
  launch_origin=false
  echo "[OK] reusing existing /local_xy_origin"
else
  launch_origin=true
  echo "[INFO] /local_xy_origin is absent; starting the standard Mapviz origin initializer"
fi

select_path_topic() {
  local candidate
  for candidate in "$@"; do
    if [[ "$(timeout 3 rostopic type "$candidate" 2>/dev/null || true)" == "nav_msgs/Path" ]]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

actual_path="$(select_path_topic /open_nav/path /open_nav/tracker_path /waypoint_tracker/path || true)"
if [[ -z "$actual_path" ]]; then
  actual_path=/open_nav/path
  echo "[WARN] no actual Path topic is available yet; waiting on $actual_path" >&2
else
  echo "[OK] actual path topic=$actual_path"
fi

planned_path="$(select_path_topic /open_nav/mission_path /open_nav/tracker_path /waypoint_tracker/path || true)"
if [[ -z "$planned_path" ]]; then
  planned_path=/open_nav/mission_path
  echo "[WARN] no planned Path topic is available yet; waiting on $planned_path" >&2
else
  echo "[OK] planned path topic=$planned_path"
fi

package_path="$(rospack find tron_open_space_nav)"
config_path="$package_path/config/rviz/mapviz_navigation_test.mvc"
echo "[INFO] Mapviz config=$config_path"

exec roslaunch tron_open_space_nav mapviz_navigation_test.launch \
  config:="$config_path" \
  actual_path_topic:="$actual_path" \
  planned_path_topic:="$planned_path" \
  launch_origin:="$launch_origin"
