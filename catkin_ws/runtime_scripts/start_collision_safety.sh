#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"
unset ROS_HOSTNAME

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 4
fi

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

wait_count=0
while true; do
  cloud_info="$(timeout 3 rostopic info /livox/lidar_filter1 2>&1 || true)"
  mapfile -t cloud_publishers < <(extract_publishers <<<"$cloud_info")
  if printf '%s\n' "${cloud_publishers[@]}" \
      | grep -Fxq /fast_lio_localization_sc_qn_node; then
    echo "[OK] /fast_lio_localization_sc_qn_node publishes /livox/lidar_filter1"
    break
  fi

  if ((wait_count % 5 == 0)); then
    echo "[INFO] waiting for fast_lio_localization_sc_qn_node to provide /livox/lidar_filter1"
    if ((${#cloud_publishers[@]} > 0)); then
      echo "       current publisher(s): ${cloud_publishers[*]}"
    fi
  fi
  wait_count=$((wait_count + 1))
  sleep 1
done

node_list="$(timeout 5 rosnode list)"
if grep -Fxq /lidar_obstacle_avoid <<<"$node_list"; then
  echo "[INFO] stopping /lidar_obstacle_avoid"
  rosnode kill /lidar_obstacle_avoid
  for _attempt in {1..30}; do
    sleep 0.1
    node_list="$(timeout 2 rosnode list)"
    if ! grep -Fxq /lidar_obstacle_avoid <<<"$node_list"; then
      break
    fi
  done
  if grep -Fxq /lidar_obstacle_avoid <<<"$node_list"; then
    echo "[ERROR] /lidar_obstacle_avoid is still running after 3 seconds" >&2
    exit 5
  fi
fi
echo "legacy lidar_obstacle_avoid is stopped"

cmd_info="$(timeout 3 rostopic info /open_nav/cmd_vel 2>&1 || true)"
mapfile -t cmd_publishers < <(extract_publishers <<<"$cmd_info")
if ((${#cmd_publishers[@]} > 0)); then
  echo "[ERROR] /open_nav/cmd_vel already has publisher(s): ${cmd_publishers[*]}" >&2
  echo "        Stop the conflicting publisher before starting collision safety." >&2
  exit 6
fi

package_path="$(rospack find tron_local_collision_safety)"
config_file="$package_path/config/test_mode.yaml"
if [[ ! -f "$config_file" ]]; then
  echo "[ERROR] test mode config not found: $config_file" >&2
  exit 7
fi

echo "[INFO] starting collision safety with $config_file"
echo "[INFO] verify: rostopic info /open_nav/cmd_vel"
echo "[INFO] verify: rostopic echo /open_nav/obstacle_status"
exec roslaunch tron_local_collision_safety collision_safety.launch \
  config_file:="$config_file"
