#!/usr/bin/env bash
# Synthetic heading-alignment check without the physical robot.
# Usage (inside GPS-TRON1 container):
#   source /opt/ros/noetic/setup.bash
#   source /root/catkin_ws/devel_isolated/tron_open_space_nav/setup.bash
#   ./runtime_scripts/test_heading_alignment_ros.sh

set -euo pipefail

WS_ROOT="${WS_ROOT:-/root/catkin_ws}"
source /opt/ros/noetic/setup.bash
if [[ -f "${WS_ROOT}/devel_isolated/tron_open_space_nav/setup.bash" ]]; then
  source "${WS_ROOT}/devel_isolated/tron_open_space_nav/setup.bash"
fi

if ! rostopic list >/dev/null 2>&1; then
  echo "ROS master unavailable. Start roscore or connect to robot master first." >&2
  exit 1
fi

if ! rosnode list | grep -qx "/gps_fastlio_fusion"; then
  echo "gps_fastlio_fusion is not running; this test will not start it automatically." >&2
  exit 1
fi

rosparam set /tron_open_space_nav/origin_ready true
rosparam set /tron_open_space_nav/origin_lat 30.5928
rosparam set /tron_open_space_nav/origin_lon 114.3055
rosparam set /tron_open_space_nav/origin_alt 30.0

echo "Initial heading status (must be UNALIGNED after a fresh fusion start):"
timeout 3 rostopic echo /open_nav/heading/status -n 1 || true

python3 - <<'PY'
import math
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

rospy.init_node("heading_alignment_test_publisher", anonymous=True)
pub = rospy.Publisher("/Odometry", Odometry, queue_size=1)

def publish_yaw(yaw_deg):
    yaw = math.radians(yaw_deg)
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "open_odom"
    msg.child_frame_id = "open_base"
    msg.pose.pose.position = Point(0.0, 0.0, 0.0)
    msg.pose.pose.orientation = Quaternion(
        x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5)
    )
    msg.twist.twist = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
    pub.publish(msg)

for _ in range(5):
    publish_yaw(30.0)
    rospy.sleep(0.2)
PY

echo "Calling /open_nav/heading/set_alignment with offset=50deg..."
rosservice call /open_nav/heading/set_alignment "{yaw_offset: 0.872664626, reference_distance: 25.0, source: 'MANUAL'}"

echo "Heading status:"
timeout 3 rostopic echo /open_nav/heading/status -n 1 || true

echo "Fused odom yaw after alignment:"
timeout 3 rostopic echo /open_nav/odom -n 1 | grep -A8 -E "orientation:|position:" || true

python3 - <<'PY'
import math
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

rospy.init_node("heading_alignment_test_publisher2", anonymous=True)
pub = rospy.Publisher("/Odometry", Odometry, queue_size=1)

yaw = math.radians(50.0)
msg = Odometry()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = "open_odom"
msg.child_frame_id = "open_base"
msg.pose.pose.position = Point(0.0, 0.0, 0.0)
msg.pose.pose.orientation = Quaternion(
    x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5)
)
msg.twist.twist = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
for _ in range(5):
    pub.publish(msg)
    rospy.sleep(0.2)
PY

echo "Fused odom yaw after FAST-LIO yaw=50deg:"
timeout 3 rostopic echo /open_nav/odom -n 1 | grep -A8 -E "orientation:|position:" || true

echo "Clearing alignment..."
rosservice call /open_nav/heading/clear_alignment "{}"
timeout 3 rostopic echo /open_nav/heading/status -n 1 || true

echo "Done. Verify aligned=true, offset~50deg, global yaw~80deg then ~100deg after yaw=50deg."
