#!/usr/bin/env bash

# Read-only runtime inspection for the GPS-TRON sensor/navigation/safety chain.
set -uo pipefail

HZ_SAMPLE_SECONDS="${RUNTIME_HZ_SECONDS:-3}"
WARNINGS=0

section() {
  printf '\n==== %s ====\n' "$1"
}

ok() {
  printf '[OK] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1" >&2
  WARNINGS=$((WARNINGS + 1))
}

fatal() {
  printf '[ERROR] %s\n' "$1" >&2
  exit 2
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fatal "required command not found: $1"
}

extract_publishers() {
  awk '
    /^Publishers:/ { in_publishers=1; next }
    /^Subscribers:/ { in_publishers=0 }
    in_publishers && /^[[:space:]]*\*/ {
      line=$0
      sub(/^[[:space:]]*\*[[:space:]]*/, "", line)
      sub(/[[:space:]].*$/, "", line)
      print line
    }
  '
}

section "ROS environment"
printf 'ROS_MASTER_URI=%s\n' "${ROS_MASTER_URI:-<unset>}"
printf 'ROS_IP=%s\n' "${ROS_IP:-<unset>}"
printf 'ROS_HOSTNAME=%s\n' "${ROS_HOSTNAME:-<unset>}"

for command_name in rosnode rostopic rosrun timeout stdbuf; do
  require_command "$command_name"
done

if ! NODE_LIST="$(timeout 5 rosnode list 2>&1)"; then
  fatal "cannot contact ROS master: ${NODE_LIST}"
fi

section "Required nodes"
REQUIRED_NODES=(
  /livox_lidar_publisher2
  /laserMapping
  /fast_lio_localization_sc_qn_node
  /gps_fastlio_fusion
  /waypoint_tracker
  /collision_safety
  /tron_controller_bridge
)

for node_name in "${REQUIRED_NODES[@]}"; do
  if grep -Fxq "$node_name" <<<"$NODE_LIST"; then
    ok "node ${node_name}"
  else
    warn "node missing: ${node_name}"
  fi
done

check_topic() {
  local topic="$1"
  local topic_type topic_info publisher_text hz_output average_rate

  if ! topic_type="$(timeout 3 rostopic type "$topic" 2>/dev/null)"; then
    warn "topic missing: ${topic}"
    return
  fi

  topic_info="$(timeout 3 rostopic info "$topic" 2>&1 || true)"
  publisher_text="$(extract_publishers <<<"$topic_info" | paste -sd ',' -)"
  if [[ -z "$publisher_text" ]]; then
    publisher_text="<none>"
    warn "topic has no publisher: ${topic}"
  fi

  hz_output="$(
    timeout "${HZ_SAMPLE_SECONDS}" \
      stdbuf -oL -eL rostopic hz -w 3 "$topic" 2>&1 || true
  )"
  average_rate="$(
    awk -F': ' '/average rate:/ { rate=$2 } END { print rate }' <<<"$hz_output"
  )"
  if [[ -z "$average_rate" ]]; then
    average_rate="<no samples in ${HZ_SAMPLE_SECONDS}s>"
    warn "unable to measure topic rate: ${topic}"
  else
    average_rate="${average_rate} Hz"
  fi

  printf '%s\n' "$topic"
  printf '  type: %s\n' "$topic_type"
  printf '  hz: %s\n' "$average_rate"
  printf '  publishers: %s\n' "$publisher_text"
}

section "Required topics"
REQUIRED_TOPICS=(
  /livox/lidar
  /livox/lidar_filter1
  /Odometry
  /gps/fix
  /open_nav/odom
  /open_nav/cmd_vel_raw
  /open_nav/cmd_vel
  /open_nav/obstacle_status
)

for topic_name in "${REQUIRED_TOPICS[@]}"; do
  check_topic "$topic_name"
done

check_tf() {
  local parent_frame="$1"
  local child_frame="$2"
  local tf_output translation rotation

  tf_output="$(
    timeout 3 stdbuf -oL -eL \
      rosrun tf tf_echo "$parent_frame" "$child_frame" 10 2>&1 || true
  )"
  translation="$(
    sed -n 's/^[[:space:]-]*Translation:/Translation:/p' <<<"$tf_output" | head -n 1
  )"
  rotation="$(
    sed -n \
      's/^[[:space:]-]*Rotation: in Quaternion/Rotation: in Quaternion/p' \
      <<<"$tf_output" | head -n 1
  )"

  printf '%s -> %s\n' "$parent_frame" "$child_frame"
  if [[ -n "$translation" && -n "$rotation" ]]; then
    printf '  %s\n' "$translation"
    printf '  %s\n' "$rotation"
  else
    warn "TF unavailable: ${parent_frame} -> ${child_frame}"
    printf '  result: %s\n' "$(tail -n 1 <<<"$tf_output")"
  fi
}

section "Required TF transforms"
check_tf map open_base
check_tf body livox_frame

section "cmd_vel arbitration"
CMD_INFO="$(timeout 3 rostopic info /open_nav/cmd_vel 2>&1 || true)"
mapfile -t CMD_PUBLISHERS < <(extract_publishers <<<"$CMD_INFO")

if ((${#CMD_PUBLISHERS[@]} == 1)) && [[ "${CMD_PUBLISHERS[0]}" == "/collision_safety" ]]; then
  ok "/open_nav/cmd_vel has exactly one publisher: /collision_safety"
else
  if ((${#CMD_PUBLISHERS[@]} == 0)); then
    warn "/open_nav/cmd_vel has no publisher"
  else
    warn "/open_nav/cmd_vel publishers: ${CMD_PUBLISHERS[*]}"
  fi
fi

for publisher_name in "${CMD_PUBLISHERS[@]}"; do
  if [[ "$publisher_name" == *lidar_obstacle_avoid* ]]; then
    printf '\n*******************************************************\n' >&2
    printf '[SAFETY WARNING] legacy lidar_obstacle_avoid is publishing cmd_vel\n' >&2
    printf 'Do not enable robot motion with multiple cmd_vel publishers.\n' >&2
    printf '*******************************************************\n' >&2
    WARNINGS=$((WARNINGS + 1))
  fi
done

section "Summary"
if ((WARNINGS == 0)); then
  ok "runtime inspection passed without warnings"
else
  warn "runtime inspection completed with ${WARNINGS} warning(s)"
fi
