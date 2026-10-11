#!/usr/bin/env bash

set -Eeuo pipefail

MODE="${1:-tcp}"
ROS_SETUP="/opt/ros/noetic/setup.bash"

case "$MODE" in
  tcp)
    LAUNCH_FILE="netgps.launch"
    ;;
  udp)
    LAUNCH_FILE="phone_gps_test.launch"
    ;;
  serial)
    LAUNCH_FILE="serial_gnss.launch"
    ;;
  -h|--help)
    echo "Usage: start_phone_gps.sh [tcp|udp|serial]"
    exit 0
    ;;
  *)
    echo "[ERROR] unsupported GPS mode: $MODE (expected tcp, udp, or serial)" >&2
    exit 2
    ;;
esac

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
  exit 2
fi

PREVIOUS_PACKAGE_PATH="$(rospack find phone_gps_bridge 2>/dev/null || true)"

set +u
source "$ROS_SETUP"
set -u

source_overlay() {
  local candidate
  local dynamic_workspace=""
  if [[ -n "$PREVIOUS_PACKAGE_PATH" ]]; then
    dynamic_workspace="$(cd "$PREVIOUS_PACKAGE_PATH/../.." 2>/dev/null && pwd || true)"
  fi
  for candidate in \
    /home/guest/catkin_ws/devel_isolated/phone_gps_bridge/setup.bash \
    /home/guest/catkin_ws/devel_isolated/setup.bash \
    "${GPS_TRON_SETUP:-}" \
    /root/catkin_ws/devel_isolated/phone_gps_bridge/setup.bash \
    /root/catkin_ws/devel_isolated/setup.bash \
    "${dynamic_workspace:+${dynamic_workspace}/devel_isolated/phone_gps_bridge/setup.bash}" \
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

source_overlay || true
if ! rospack find phone_gps_bridge >/dev/null 2>&1; then
  echo "[ERROR] phone_gps_bridge is not visible in the sourced overlays" >&2
  exit 3
fi

receiver_nodes=(/netgps_tcp_receiver /gps_udp_receiver /serial_gnss_receiver)
node_list="$(timeout 5 rosnode list 2>/dev/null || true)"
for receiver_node in "${receiver_nodes[@]}"; do
  if grep -Fxq "$receiver_node" <<<"$node_list"; then
    echo "[ERROR] GPS receiver already running: $receiver_node" >&2
    exit 4
  fi
done

launch_args=()
if [[ "$MODE" == "serial" ]]; then
  device="${GNSS_SERIAL_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C84345474-if00}"
  baudrate="${GNSS_SERIAL_BAUDRATE:-115200}"
  if [[ ! -e "$device" ]]; then
    echo "[ERROR] GNSS serial device is not visible: $device" >&2
    exit 5
  fi
  if [[ ! -r "$device" || ! -w "$device" ]]; then
    ls -l "$device" >&2 || true
    echo "[ERROR] serial permission denied; user=$(id -un) groups=$(id -Gn)" >&2
    echo "        Add the user to dialout; this script will not change permissions." >&2
    exit 6
  fi
  launch_args+=("device:=$device" "baudrate:=$baudrate")
  echo "[INFO] starting GNSS mode=serial"
  echo "[INFO] device=$device"
  echo "[INFO] baudrate=$baudrate"
else
  echo "[INFO] starting phone GPS mode=$MODE with $LAUNCH_FILE"
fi
echo "[INFO] verify in another terminal: rostopic hz /gps/fix"

exec roslaunch phone_gps_bridge "$LAUNCH_FILE" "${launch_args[@]}"
