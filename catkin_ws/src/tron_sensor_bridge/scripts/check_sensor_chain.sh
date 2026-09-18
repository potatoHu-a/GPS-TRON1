#!/usr/bin/env bash

set -uo pipefail

SAMPLE_TIMEOUT="${SENSOR_TOPIC_TIMEOUT:-4}"
LIVOX_MSG_TYPE="${LIVOX_MSG_TYPE:-livox_ros_driver2/CustomMsg}"
WARNINGS=0

ok() {
  printf '[OK] %s\n' "$*"
}

warn() {
  printf '[WARN] %s\n' "$*" >&2
  WARNINGS=$((WARNINGS + 1))
}

if ! command -v rostopic >/dev/null 2>&1; then
  warn "rostopic is unavailable; source ROS and the sensor overlays first"
  exit 2
fi

if ! timeout 3 rosnode list >/dev/null 2>&1; then
  warn "ROS master is unreachable: ${ROS_MASTER_URI:-unset}"
  exit 2
fi

check_topic() {
  local topic="$1"
  local expected_type="$2"
  local actual_type
  local info
  local publisher

  actual_type="$(timeout 3 rostopic type "$topic" 2>/dev/null || true)"
  if [[ -z "$actual_type" ]]; then
    warn "$topic is not registered"
    return
  fi

  if [[ "$actual_type" == "$expected_type" ]]; then
    ok "$topic type=$actual_type"
  else
    warn "$topic type=$actual_type expected=$expected_type"
  fi

  info="$(timeout 3 rostopic info "$topic" 2>/dev/null || true)"
  publisher="$(awk '/^Publishers:/{getline; sub(/^[[:space:]]*\*[[:space:]]*/, ""); print; exit}' <<<"$info")"
  if [[ -z "$publisher" || "$publisher" == "None" ]]; then
    warn "$topic has no publisher"
  else
    ok "$topic publisher=$publisher"
  fi

  if timeout "$SAMPLE_TIMEOUT" rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
    ok "$topic received a message"
  else
    warn "$topic is registered but no message arrived within ${SAMPLE_TIMEOUT}s"
  fi
}

printf 'Sensor chain check (master=%s)\n' "${ROS_MASTER_URI:-unset}"
check_topic /livox/lidar "$LIVOX_MSG_TYPE"
check_topic /livox/imu sensor_msgs/Imu
check_topic /Odometry nav_msgs/Odometry
check_topic /cloud_registered sensor_msgs/PointCloud2
check_topic /livox/lidar_filter1 sensor_msgs/PointCloud2

if ((WARNINGS)); then
  printf '[WARN] sensor chain completed with %d warning(s)\n' "$WARNINGS" >&2
  exit 1
fi

ok "sensor chain is complete"
