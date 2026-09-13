#!/usr/bin/env bash
# Pre-flight checks for tron_open_space_nav outdoor stack.
set -u

OK=0
ERR=0

pass() { echo "[OK]   $1"; OK=$((OK + 1)); }
fail() { echo "[ERROR] $1"; ERR=$((ERR + 1)); }

check_topic() {
  local topic="$1"
  local typ="$2"
  if timeout 3 rostopic echo "$topic" -n 1 >/dev/null 2>&1; then
    pass "topic $topic ($typ) publishing"
  else
    fail "topic $topic ($typ) no data"
  fi
}

check_tf() {
  local parent="$1"
  local child="$2"
  if timeout 3 rosrun tf tf_echo "$parent" "$child" 2>/dev/null | head -5 | grep -q "At time"; then
    pass "TF $parent -> $child"
  else
    fail "TF $parent -> $child missing or stale"
  fi
}

echo "=== tron_open_space_nav system check ==="

if timeout 2 rostopic list >/dev/null 2>&1; then
  pass "ROS master reachable"
else
  fail "ROS master not reachable (check ROS_MASTER_URI)"
  echo "Summary: OK=$OK ERROR=$ERR"
  exit 1
fi

check_topic "/gps/fix" "sensor_msgs/NavSatFix"
check_topic "/Odometry" "nav_msgs/Odometry"
check_topic "/open_nav/odom" "nav_msgs/Odometry"
check_topic "/open_nav/cmd_vel" "geometry_msgs/Twist"

check_tf "earth" "map"
check_tf "map" "open_base"

if rosnode list 2>/dev/null | grep -q "gps_global_converter"; then
  pass "node gps_global_converter running"
else
  fail "node gps_global_converter not running"
fi

if rosnode list 2>/dev/null | grep -q "waypoint_tracker"; then
  pass "node waypoint_tracker running"
else
  fail "node waypoint_tracker not running"
fi

if rosnode list 2>/dev/null | grep -qi "rviz"; then
  fail "rviz running on robot (should run on dev PC only)"
else
  pass "no rviz on robot"
fi

echo "=== Summary: OK=$OK ERROR=$ERR ==="
if [ "$ERR" -gt 0 ]; then
  exit 1
fi
exit 0
