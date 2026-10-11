#!/usr/bin/env bash

set -Eeuo pipefail

ROS_SETUP="/opt/ros/noetic/setup.bash"
DEFAULT_MASTER_URI="http://10.192.1.3:11311"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
  exit 2
fi

PREVIOUS_PACKAGE_PATH="$(rospack find tron_local_collision_safety 2>/dev/null || true)"

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
    /home/guest/catkin_ws/devel_isolated/tron_local_collision_safety/setup.bash \
    /home/guest/catkin_ws/devel_isolated/setup.bash \
    "${GPS_TRON_SETUP:-}" \
    /root/catkin_ws/devel_isolated/tron_local_collision_safety/setup.bash \
    /root/catkin_ws/devel_isolated/setup.bash \
    "${dynamic_workspace:+${dynamic_workspace}/devel_isolated/tron_local_collision_safety/setup.bash}" \
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

if ! PACKAGE_PATH="$(rospack find tron_local_collision_safety 2>/dev/null)"; then
  echo "[ERROR] tron_local_collision_safety is not visible in the sourced overlays" >&2
  echo "        Set GPS_TRON_SETUP to the applicable setup.bash path." >&2
  exit 3
fi

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  export ROS_MASTER_URI="$DEFAULT_MASTER_URI"
fi
unset ROS_HOSTNAME

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 4
fi

rosrun tron_local_collision_safety stop_legacy_obstacle.sh

extract_publishers() {
  awk '
    /^Publishers:/ { active=1; next }
    /^Subscribers:/ { active=0 }
    active && /^[[:space:]]*\*/ {
      line=$0
      sub(/^[[:space:]]*\*[[:space:]]*/, "", line)
      sub(/[[:space:]].*$/, "", line)
      print line
    }
  '
}

CMD_INFO="$(timeout 3 rostopic info /open_nav/cmd_vel 2>&1 || true)"
mapfile -t CMD_PUBLISHERS < <(extract_publishers <<<"$CMD_INFO")
UNKNOWN_PUBLISHERS=()
for publisher_name in "${CMD_PUBLISHERS[@]}"; do
  [[ "$publisher_name" == "/lidar_obstacle_avoid" ]] && continue
  UNKNOWN_PUBLISHERS+=("$publisher_name")
done

if ((${#UNKNOWN_PUBLISHERS[@]} > 0)); then
  echo "[ERROR] /open_nav/cmd_vel already has publisher(s): ${UNKNOWN_PUBLISHERS[*]}" >&2
  echo "        Stop the conflicting publisher before starting collision safety." >&2
  exit 5
fi

CONFIG_FILE="$PACKAGE_PATH/config/test_mode.yaml"
if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "[ERROR] test mode config not found: $CONFIG_FILE" >&2
  exit 6
fi

echo "[INFO] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[INFO] starting collision safety with: $CONFIG_FILE"
echo "[INFO] after startup, verify:"
echo "       rostopic info /open_nav/cmd_vel"
echo "       rostopic echo /open_nav/obstacle_status"

exec roslaunch tron_local_collision_safety collision_safety.launch \
  config_file:="$CONFIG_FILE"
