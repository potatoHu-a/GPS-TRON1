#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${TRON_SENSOR_WORKSPACE_CONFIG:-}"
CHECK_ONLY=false
ROSLAUNCH_ARGS=()

usage() {
  cat <<'EOF'
Usage: start_sensor.sh [--config PATH] [--check-only] [ROSLAUNCH_ARG:=VALUE ...]

Sources the configured ROS overlays and starts official_sensor.launch.
The external workspaces must be mounted into this runtime; no official source is copied.
EOF
}

while (($#)); do
  case "$1" in
    --config)
      if (($# < 2)); then
        echo "[ERROR] --config requires a path" >&2
        exit 2
      fi
      CONFIG_FILE="$2"
      shift 2
      ;;
    --check-only)
      CHECK_ONLY=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      ROSLAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

find_default_config() {
  local candidate
  for candidate in \
    "${SCRIPT_DIR}/../config/workspace.yaml" \
    "${SCRIPT_DIR}/../../share/tron_sensor_bridge/config/workspace.yaml"; do
    if [[ -f "$candidate" ]]; then
      readlink -f "$candidate"
      return 0
    fi
  done
  return 1
}

if [[ -z "$CONFIG_FILE" ]]; then
  if ! CONFIG_FILE="$(find_default_config)"; then
    echo "[ERROR] workspace.yaml not found; pass --config PATH" >&2
    exit 2
  fi
fi

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "[ERROR] workspace config not found: $CONFIG_FILE" >&2
  exit 2
fi
CONFIG_FILE="$(readlink -f "$CONFIG_FILE")"

mapfile -t SETTINGS < <(python3 - "$CONFIG_FILE" <<'PY'
import os
import sys

import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}

def required(mapping, *keys):
    value = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            raise KeyError(".".join(keys))
        value = value[key]
    if value is None or str(value).strip() == "":
        raise ValueError("empty value: " + ".".join(keys))
    return os.path.expanduser(os.path.expandvars(str(value)))

def boolean(mapping, *keys):
    value = required(mapping, *keys)
    return "true" if value.lower() in ("1", "true", "yes", "on") else "false"

try:
    print(required(config, "ros", "setup_file"))
    print(required(config, "workspaces", "livox", "setup_file"))
    print(required(config, "workspaces", "fastlio", "setup_file"))
    print(required(config, "workspaces", "gps_tron", "setup_file"))
    print(boolean(config, "launch", "start_livox"))
    print(boolean(config, "launch", "start_fastlio"))
    print(required(config, "launch", "lidar_model"))
    print(boolean(config, "launch", "rviz"))
    print(required(config, "launch", "livox_frame"))
    print(required(config, "launch", "livox_publish_frequency"))
except (KeyError, ValueError, TypeError) as exc:
    print("invalid workspace config: {}".format(exc), file=sys.stderr)
    sys.exit(2)
PY
)

if ((${#SETTINGS[@]} != 10)); then
  echo "[ERROR] unable to read required settings from $CONFIG_FILE" >&2
  exit 2
fi

ROS_SETUP="${SETTINGS[0]}"
LIVOX_SETUP="${SETTINGS[1]}"
FASTLIO_SETUP="${SETTINGS[2]}"
GPS_TRON_SETUP="${SETTINGS[3]}"
START_LIVOX="${SETTINGS[4]}"
START_FASTLIO="${SETTINGS[5]}"
LIDAR_MODEL="${SETTINGS[6]}"
RVIZ="${SETTINGS[7]}"
LIVOX_FRAME="${SETTINGS[8]}"
LIVOX_PUBLISH_FREQUENCY="${SETTINGS[9]}"

require_setup() {
  local label="$1"
  local setup_file="$2"
  if [[ ! -f "$setup_file" ]]; then
    echo "[ERROR] ${label} setup is not visible: $setup_file" >&2
    echo "        Mount the external workspace at the configured path or update $CONFIG_FILE" >&2
    return 1
  fi
}

setup_missing=0
require_setup "ROS" "$ROS_SETUP" || setup_missing=1
require_setup "Livox workspace" "$LIVOX_SETUP" || setup_missing=1
require_setup "FAST-LIO workspace" "$FASTLIO_SETUP" || setup_missing=1
require_setup "GPS-TRON workspace" "$GPS_TRON_SETUP" || setup_missing=1
if ((setup_missing)); then
  exit 2
fi

# Catkin setup files normally remove previous overlays. --extend preserves each
# already-sourced workspace while adding the next one.
set +u
source "$ROS_SETUP"
CATKIN_SETUP_UTIL_ARGS=--extend source "$LIVOX_SETUP"
CATKIN_SETUP_UTIL_ARGS=--extend source "$FASTLIO_SETUP"
CATKIN_SETUP_UTIL_ARGS=--extend source "$GPS_TRON_SETUP"
unset CATKIN_SETUP_UTIL_ARGS
set -u

required_packages=(
  livox_ros_driver2
  fast_lio
  fast_lio_localization_sc_qn
  pointcloud_to_laserscan
  tron_sensor_bridge
)

missing=0
for package in "${required_packages[@]}"; do
  if package_path="$(rospack find "$package" 2>/dev/null)"; then
    echo "[OK] package ${package}: ${package_path}"
  else
    echo "[ERROR] package not visible after sourcing overlays: ${package}" >&2
    missing=1
  fi
done
if ((missing)); then
  exit 3
fi

LIVOX_LAUNCH="$(rospack find livox_ros_driver2)/launch_ROS1/msg_MID360.launch"
FASTLIO_LAUNCH="$(rospack find fast_lio_localization_sc_qn)/launch/run.launch"
for launch_file in "$LIVOX_LAUNCH" "$FASTLIO_LAUNCH"; do
  if [[ ! -f "$launch_file" ]]; then
    echo "[ERROR] required launch file not found: $launch_file" >&2
    exit 3
  fi
  echo "[OK] launch: $launch_file"
done

if [[ "$CHECK_ONLY" == true ]]; then
  echo "[OK] sensor overlay preflight passed; launch was not started"
  exit 0
fi

echo "[INFO] starting official sensor chain with config: $CONFIG_FILE"
exec roslaunch tron_sensor_bridge official_sensor.launch \
  start_livox:="$START_LIVOX" \
  start_fastlio:="$START_FASTLIO" \
  lidar_model:="$LIDAR_MODEL" \
  rviz:="$RVIZ" \
  livox_frame:="$LIVOX_FRAME" \
  livox_publish_frequency:="$LIVOX_PUBLISH_FREQUENCY" \
  "${ROSLAUNCH_ARGS[@]}"
