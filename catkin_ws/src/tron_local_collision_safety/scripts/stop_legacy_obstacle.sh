#!/usr/bin/env bash

set -uo pipefail

ROS_SETUP="/opt/ros/noetic/setup.bash"
LEGACY_NODE="/lidar_obstacle_avoid"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
  exit 2
fi

set +u
source "$ROS_SETUP"
set -u

if ! NODE_LIST="$(timeout 5 rosnode list 2>&1)"; then
  echo "[ERROR] cannot contact ROS master: $NODE_LIST" >&2
  exit 2
fi

if ! grep -Fxq "$LEGACY_NODE" <<<"$NODE_LIST"; then
  echo "legacy lidar_obstacle_avoid is stopped"
  exit 0
fi

echo "[INFO] stopping $LEGACY_NODE"
if ! rosnode kill "$LEGACY_NODE"; then
  echo "[ERROR] failed to request shutdown of $LEGACY_NODE" >&2
  exit 3
fi

for _attempt in {1..30}; do
  sleep 0.1
  if ! NODE_LIST="$(timeout 2 rosnode list 2>&1)"; then
    echo "[ERROR] lost connection to ROS master while checking $LEGACY_NODE" >&2
    exit 2
  fi
  if ! grep -Fxq "$LEGACY_NODE" <<<"$NODE_LIST"; then
    echo "legacy lidar_obstacle_avoid is stopped"
    exit 0
  fi
done

echo "[ERROR] $LEGACY_NODE is still running after 3 seconds" >&2
exit 4
