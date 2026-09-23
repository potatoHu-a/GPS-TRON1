#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/setup_runtime_env.sh"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://10.192.1.3:11311}"

mode="${1:-tcp}"
if (($# > 1)); then
  echo "Usage: $0 [tcp|udp|serial]" >&2
  exit 2
fi

case "$mode" in
  tcp)
    launch_file="netgps.launch"
    ;;
  udp)
    launch_file="phone_gps_test.launch"
    ;;
  serial)
    launch_file="serial_gnss.launch"
    ;;
  -h|--help)
    echo "Usage: $0 [tcp|udp|serial]"
    exit 0
    ;;
  *)
    echo "[ERROR] unsupported GPS mode: $mode (expected tcp, udp, or serial)" >&2
    exit 2
    ;;
esac

rospack find phone_gps_bridge >/dev/null

receiver_nodes=(/netgps_tcp_receiver /gps_udp_receiver /serial_gnss_receiver)
node_list="$(timeout 5 rosnode list 2>/dev/null || true)"
for receiver_node in "${receiver_nodes[@]}"; do
  if grep -Fxq "$receiver_node" <<<"$node_list"; then
    echo "[ERROR] GPS receiver already running: $receiver_node" >&2
    echo "        Stop it before selecting another input mode." >&2
    exit 3
  fi
done

launch_args=()
if [[ "$mode" == "serial" ]]; then
  device="${GNSS_SERIAL_DEVICE:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C84345474-if00}"
  baudrate="${GNSS_SERIAL_BAUDRATE:-115200}"
  if [[ ! -e "$device" ]]; then
    echo "[ERROR] GNSS serial device is not visible: $device" >&2
    echo "        Pass the USB device into this container or set GNSS_SERIAL_DEVICE." >&2
    exit 4
  fi
  if [[ ! -r "$device" || ! -w "$device" ]]; then
    echo "[ERROR] GNSS serial device is not readable and writable: $device" >&2
    ls -l "$device" >&2 || true
    echo "        current user: $(id -un)" >&2
    echo "        groups: $(id -Gn)" >&2
    echo "        Add the user to dialout, then log out and back in:" >&2
    echo "        sudo usermod -aG dialout $(id -un)" >&2
    exit 5
  fi
  launch_args+=("device:=$device" "baudrate:=$baudrate")
  echo "[INFO] starting GNSS mode=serial"
  echo "[INFO] device=$device"
  echo "[INFO] baudrate=$baudrate"
else
  echo "[INFO] starting phone GPS mode=$mode"
fi
echo "[INFO] verify in another terminal: rostopic hz /gps/fix"
exec roslaunch phone_gps_bridge "$launch_file" "${launch_args[@]}"
