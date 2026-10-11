#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  exit 2
fi

failures=0

check_rate() {
  local topic="$1"
  local topic_type rate_output
  topic_type="$(timeout 3 rostopic type "$topic" 2>/dev/null || true)"
  if [[ -z "$topic_type" ]]; then
    echo "[WARN] $topic: unavailable"
    failures=$((failures + 1))
    return
  fi
  echo "[INFO] $topic type=$topic_type"
  rate_output="$(timeout 8 rostopic hz -w 3 "$topic" 2>&1 || true)"
  sed "s/^/       /" <<<"$rate_output"
  if ! grep -q "average rate" <<<"$rate_output"; then
    echo "[WARN] $topic: unable to measure frequency"
    failures=$((failures + 1))
  fi
}

echo "=== Topic rates ==="
for topic in \
  /livox/lidar /livox/imu /Odometry /livox/lidar_filter1 \
  /gps/fix /open_nav/odom /open_nav/local_plan; do
  check_rate "$topic"
done

echo "=== Safety message ==="
status_type="$(timeout 3 rostopic type /open_nav/obstacle_status 2>/dev/null || true)"
if [[ "$status_type" == "tron_local_collision_safety/ObstacleStatus" ]]; then
  echo "[OK] /open_nav/obstacle_status type=$status_type"
else
  echo "[ERROR] unexpected /open_nav/obstacle_status type: ${status_type:-unavailable}" >&2
  failures=$((failures + 1))
fi

echo "=== Collision nodes ==="
node_list="$(timeout 5 rosnode list)"
printf '%s\n' "$node_list" | grep -E 'collision|lidar_obstacle' || true
if grep -Fxq /collision_safety <<<"$node_list"; then
  echo "[OK] /collision_safety is running"
else
  echo "[ERROR] /collision_safety is not running" >&2
  failures=$((failures + 1))
fi
if grep -Fxq /lidar_obstacle_avoid <<<"$node_list"; then
  echo "[ERROR] legacy /lidar_obstacle_avoid must not run in full bringup" >&2
  failures=$((failures + 1))
else
  echo "[OK] legacy /lidar_obstacle_avoid is absent"
fi

echo "=== cmd_vel ownership ==="
cmd_info="$(timeout 3 rostopic info /open_nav/cmd_vel 2>&1 || true)"
cmd_publishers="$(awk '
  /^Publishers:/ { active=1; next }
  /^Subscribers:/ { active=0 }
  active && /^[[:space:]]*\*/ {
    line=$0
    sub(/^[[:space:]]*\*[[:space:]]*/, "", line)
    sub(/[[:space:]].*$/, "", line)
    print line
  }
' <<<"$cmd_info")"
if [[ "$cmd_publishers" == "/collision_safety" ]]; then
  echo "[OK] /collision_safety is the only /open_nav/cmd_vel publisher"
else
  echo "[ERROR] unexpected /open_nav/cmd_vel publisher(s):" >&2
  sed 's/^/        /' <<<"${cmd_publishers:-none}" >&2
  failures=$((failures + 1))
fi

echo "=== Controller mode ==="
if dry_run="$(rosparam get /tron_controller_bridge/dry_run 2>/dev/null)"; then
  echo "[OK] /tron_controller_bridge/dry_run=$dry_run"
else
  echo "[ERROR] /tron_controller_bridge/dry_run is unavailable" >&2
  failures=$((failures + 1))
fi

echo "=== Shadow planner ==="
if ! grep -Fxq /local_trajectory_planner <<<"$node_list"; then
  echo "[ERROR] /local_trajectory_planner is not running" >&2
  failures=$((failures + 1))
  planner_info=""
else
  planner_info="$(timeout 3 rosnode info /local_trajectory_planner 2>&1 || true)"
fi
if [[ -n "$planner_info" ]] && grep -Eq '/open_nav/cmd_vel(_raw)?' <<<"$planner_info"; then
  echo "[ERROR] local trajectory planner is connected to cmd_vel" >&2
  failures=$((failures + 1))
elif [[ -n "$planner_info" ]]; then
  echo "[OK] local trajectory planner has no cmd_vel interface"
fi

if ((failures > 0)); then
  echo "[WARN] full bringup check completed with $failures issue(s)" >&2
  exit 1
fi
echo "[OK] full bringup check passed"
