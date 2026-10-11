#!/usr/bin/env bash

set -Eeuo pipefail

if [[ -n "${TRON_WS:-}" ]]; then
  workspace="$TRON_WS"
elif [[ -d /root/catkin_ws ]]; then
  workspace=/root/catkin_ws
elif [[ -d /home/guest/catkin_ws ]]; then
  workspace=/home/guest/catkin_ws
else
  echo "[ERROR] catkin workspace not found; set TRON_WS" >&2
  exit 2
fi

ros_setup=/opt/ros/noetic/setup.bash
workspace_setup="$workspace/devel_isolated/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
  echo "[ERROR] ROS setup not found: $ros_setup" >&2
  exit 2
fi
if [[ ! -f "$workspace_setup" ]]; then
  echo "[ERROR] isolated workspace setup not found: $workspace_setup" >&2
  exit 2
fi

set +u
source "$ros_setup"
source "$workspace_setup"
set -u

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export DISPLAY="${DISPLAY:-:1}"

if ! timeout 5 rosnode list >/dev/null 2>&1; then
  echo "[ERROR] local ROS master not available at http://127.0.0.1:11311" >&2
  echo "[INFO] start roscore first" >&2
  exit 3
fi

fix_type="$(timeout 5 rostopic type /gps/fix 2>/dev/null || true)"
if [[ "$fix_type" == "sensor_msgs/NavSatFix" ]]; then
  echo "[OK] /gps/fix available"
elif [[ -n "$fix_type" ]]; then
  echo "[WARN] /gps/fix has unexpected type: $fix_type" >&2
  echo "[INFO] expected sensor_msgs/NavSatFix" >&2
else
  echo "[WARN] /gps/fix is not available yet" >&2
  echo "[INFO] start the GNSS receiver first" >&2
fi

package_path="$(rospack find tron_open_space_nav)"
config_path="$package_path/config/rviz/mapviz_gnss_test.mvc"
tile_root="$package_path/maps/wuhan_tiles"
if [[ ! -f "$config_path" ]]; then
  echo "[ERROR] Mapviz config not found: $config_path" >&2
  exit 4
fi
if [[ ! -d "$tile_root" ]]; then
  echo "[ERROR] Wuhan tile directory not found: $tile_root" >&2
  exit 4
fi

echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[INFO] ROS_IP=$ROS_IP"
echo "[INFO] DISPLAY=$DISPLAY"
echo "[INFO] config=$config_path"
echo "[INFO] Mapviz fixed/target frame=wgs84"
echo "[INFO] internal geographic projection frame=gnss_local"

exec roslaunch tron_open_space_nav mapviz_gnss_test.launch \
  config:="$config_path"
