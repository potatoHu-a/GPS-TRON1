#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export TRON_WS="${TRON_WS:-/root/catkin_ws}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[ERROR] ROS Noetic setup not found" >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash

if [[ ! -f "$TRON_WS/devel_isolated/setup.bash" ]]; then
  echo "[ERROR] workspace setup not found: $TRON_WS/devel_isolated/setup.bash" >&2
  exit 2
fi
source "$TRON_WS/devel_isolated/setup.bash"
source "$SCRIPT_DIR/setup_runtime_env.sh"

if ! "$SCRIPT_DIR/check_heading_runtime.sh"; then
  echo "[ERROR] dashboard/heading runtime is stale; rebuild before starting Dashboard." >&2
  exit 5
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"
export ROS_IP="${ROS_IP:-10.192.1.194}"
unset ROS_HOSTNAME
export DISPLAY="${DISPLAY:-:1}"
# Some containers expose an X display without the NVIDIA DRM userspace
# driver. Use Mesa software rendering in that case; this affects only the
# Dashboard GUI and avoids misleading libGL startup errors.
if ! command -v nvidia-smi >/dev/null 2>&1; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
fi

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] robot ROS master is not reachable at $ROS_MASTER_URI" >&2
  echo "        Dashboard does not start roscore." >&2
  exit 3
fi
if ! rospack find tron_nav_dashboard >/dev/null 2>&1; then
  echo "[ERROR] tron_nav_dashboard is not visible; build it with catkin_make_isolated" >&2
  exit 4
fi

echo "================================"
echo "TRON1 NAVIGATION DASHBOARD"
echo "================================"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"
echo "DISPLAY=$DISPLAY"
echo "Mode: READ ONLY"
echo "================================"

exec roslaunch tron_nav_dashboard navigation_dashboard.launch
