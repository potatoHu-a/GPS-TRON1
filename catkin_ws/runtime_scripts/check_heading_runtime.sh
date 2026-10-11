#!/usr/bin/env bash

set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${TRON_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/noetic/setup.bash
source "$SCRIPT_DIR/setup_runtime_env.sh"

required=(tron_open_space_nav tron_nav_dashboard)
for pkg in "${required[@]}"; do
  rospack find "$pkg" >/dev/null || {
    echo "[ERROR] package is not visible: $pkg" >&2
    exit 1
  }
done

rossrv show tron_open_space_nav/SetHeadingAlignment >/dev/null || {
  echo "[ERROR] generated SetHeadingAlignment service is stale or missing." >&2
  echo "        Rebuild tron_open_space_nav with catkin_make_isolated." >&2
  exit 1
}
rosmsg show tron_open_space_nav/HeadingStatus >/dev/null || {
  echo "[ERROR] generated HeadingStatus message is stale or missing." >&2
  exit 1
}

python3 - <<'PY'
from tron_open_space_nav.srv import SetHeadingAlignment, SetHeadingAlignmentResponse
from tron_open_space_nav.msg import HeadingStatus
from tron_nav_dashboard.dashboard_geometry import arrow_delta_screen
print("[OK] SetHeadingAlignment:", SetHeadingAlignment.__module__)
print("[OK] HeadingStatus:", HeadingStatus.__module__)
print("[OK] dashboard_geometry:", arrow_delta_screen.__module__)
PY

echo "[OK] heading runtime interfaces are current"
