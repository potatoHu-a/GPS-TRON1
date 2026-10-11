#!/usr/bin/env bash

set -Ee -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRON_WS="${TRON_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
export TRON_WS

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[ERROR] ROS Noetic setup not found: /opt/ros/noetic/setup.bash" >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash

if [[ ! -f "$TRON_WS/devel_isolated/setup.bash" ]]; then
  echo "[ERROR] workspace setup not found: $TRON_WS/devel_isolated/setup.bash" >&2
  echo "        Build with catkin_make_isolated first." >&2
  exit 2
fi
source "$TRON_WS/devel_isolated/setup.bash"

# Reassemble all isolated package environments after loading the generated
# top-level setup, whose chain may omit packages built later with --pkg.
source "$SCRIPT_DIR/setup_runtime_env.sh"
set -u

if ! "$SCRIPT_DIR/check_heading_runtime.sh"; then
  echo "[ERROR] generated heading interfaces are stale; bringup aborted." >&2
  exit 7
fi

mode="${1:-test}"
if (($# > 1)); then
  echo "Usage: $0 [test|real]" >&2
  exit 2
fi
case "$mode" in
  test)
    dry_run=true
    control_mode=TEST
    ;;
  real)
    dry_run=false
    control_mode=REAL
    ;;
  -h|--help)
    echo "Usage: $0 [test|real]"
    exit 0
    ;;
  *)
    echo "[ERROR] invalid mode: $mode (expected test or real)" >&2
    exit 2
    ;;
esac

enable_livox="${ENABLE_LIVOX:-true}"
enable_localization="${ENABLE_LOCALIZATION:-true}"
enable_gnss="${ENABLE_GNSS:-true}"
enable_navigation="${ENABLE_NAVIGATION:-true}"
enable_local_planner="${ENABLE_LOCAL_PLANNER:-true}"
enable_collision_safety="${ENABLE_COLLISION_SAFETY:-true}"

validate_bool() {
  case "$2" in
    true|false) ;;
    *) echo "[ERROR] $1 must be true or false, got: $2" >&2; exit 2 ;;
  esac
}
validate_bool ENABLE_LIVOX "$enable_livox"
validate_bool ENABLE_LOCALIZATION "$enable_localization"
validate_bool ENABLE_GNSS "$enable_gnss"
validate_bool ENABLE_NAVIGATION "$enable_navigation"
validate_bool ENABLE_LOCAL_PLANNER "$enable_local_planner"
validate_bool ENABLE_COLLISION_SAFETY "$enable_collision_safety"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"
if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[ERROR] ROS master is not reachable at $ROS_MASTER_URI" >&2
  echo "        This script does not start another roscore." >&2
  exit 3
fi

primary_device="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C84345474-if00"
fallback_device="/dev/ttyACM0"
gnss_device="${GNSS_SERIAL_DEVICE:-}"
if [[ "$enable_gnss" == true ]]; then
  if [[ -z "$gnss_device" ]]; then
    if [[ -e "$primary_device" ]]; then
      gnss_device="$primary_device"
    elif [[ -e "$fallback_device" ]]; then
      gnss_device="$fallback_device"
      echo "[WARN] preferred GNSS device is absent; using $fallback_device" >&2
    else
      echo "[ERROR] no South GNSS serial device found" >&2
      echo "        checked: $primary_device" >&2
      echo "        checked: $fallback_device" >&2
      exit 4
    fi
  fi
  if [[ ! -e "$gnss_device" ]]; then
    echo "[ERROR] GNSS serial device does not exist: $gnss_device" >&2
    exit 4
  fi
  if [[ ! -r "$gnss_device" || ! -w "$gnss_device" ]]; then
    echo "[ERROR] GNSS device is not readable and writable: $gnss_device" >&2
    ls -l "$gnss_device" >&2 || true
    echo "        Configure dialout/udev, or for the first test run manually:" >&2
    echo "        sudo chmod 666 $gnss_device" >&2
    exit 4
  fi
else
  gnss_device="${GNSS_SERIAL_DEVICE:-$primary_device}"
fi

node_list="$(timeout 5 rosnode list)"
conflicts=()
expected_nodes=()
[[ "$enable_livox" == true ]] && expected_nodes+=(/livox_lidar_publisher2 /livox_driver)
[[ "$enable_localization" == true ]] && expected_nodes+=(/laserMapping /fast_lio_localization_sc_qn_node)
[[ "$enable_gnss" == true ]] && expected_nodes+=(/serial_gnss_receiver)
[[ "$enable_navigation" == true ]] && expected_nodes+=(/gps_fastlio_fusion /waypoint_tracker /tron_controller_bridge)
[[ "$enable_local_planner" == true ]] && expected_nodes+=(/local_trajectory_planner)
[[ "$enable_collision_safety" == true ]] && expected_nodes+=(/collision_safety /lidar_obstacle_avoid)
for node_name in "${expected_nodes[@]}"; do
  if grep -Fxq "$node_name" <<<"$node_list"; then
    conflicts+=("$node_name")
  fi
done
if ((${#conflicts[@]} > 0)); then
  echo "[ERROR] bringup component(s) already running: ${conflicts[*]}" >&2
  echo "        Stop the old launch before starting the unified bringup." >&2
  exit 5
fi

required_packages=(tron_open_space_nav)
if [[ "$enable_livox" == true ]]; then
  required_packages+=(livox_ros_driver2 tron_sensor_bridge)
fi
if [[ "$enable_localization" == true ]]; then
  required_packages+=(fast_lio_localization_sc_qn tron_sensor_bridge)
fi
if [[ "$enable_gnss" == true ]]; then
  required_packages+=(phone_gps_bridge)
fi
if [[ "$enable_local_planner" == true ]]; then
  required_packages+=(tron_local_trajectory_planner)
fi
if [[ "$enable_collision_safety" == true ]]; then
  required_packages+=(tron_local_collision_safety)
fi
for package_name in "${required_packages[@]}"; do
  if ! rospack find "$package_name" >/dev/null 2>&1; then
    echo "[ERROR] required package is not visible: $package_name" >&2
    exit 6
  fi
done

on_off() {
  [[ "$1" == true ]] && echo ON || echo OFF
}
planner_mode=OFF
[[ "$enable_local_planner" == true ]] && planner_mode=SHADOW

cat <<EOF
================================
TRON1 FULL BRINGUP
================================
Livox: $(on_off "$enable_livox")
Localization: $(on_off "$enable_localization")
South GNSS: $(on_off "$enable_gnss")
GPS Navigation: $(on_off "$enable_navigation")
Local Planner: $planner_mode
Collision Safety: $(on_off "$enable_collision_safety")

Control mode: $control_mode
dry_run: $dry_run
GNSS device: $gnss_device
ROS master: $ROS_MASTER_URI
================================
EOF

exec roslaunch tron_open_space_nav tron_full_bringup.launch \
  dry_run:="$dry_run" \
  enable_livox:="$enable_livox" \
  enable_localization:="$enable_localization" \
  enable_gnss:="$enable_gnss" \
  enable_navigation:="$enable_navigation" \
  enable_local_planner:="$enable_local_planner" \
  enable_collision_safety:="$enable_collision_safety" \
  gnss_serial_device:="$gnss_device"
