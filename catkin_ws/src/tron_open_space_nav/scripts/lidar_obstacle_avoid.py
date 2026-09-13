#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import math

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from tron_open_space_nav.msg import ObstacleStatus


STATE_CLEAR = "CLEAR"
STATE_SLOW = "SLOW"
STATE_AVOID_LEFT = "AVOID_LEFT"
STATE_AVOID_RIGHT = "AVOID_RIGHT"
STATE_BLOCKED = "BLOCKED"
STATE_SENSOR_TIMEOUT = "SENSOR_TIMEOUT"


class RegionStats(object):
    def __init__(self, max_distance):
        self.min_distance = max_distance
        self.point_count = 0
        self.occupancy = 0.0
        self.points = []


class LidarObstacleAvoid(object):
    """Local obstacle layer between waypoint tracking and robot bridge."""

    def __init__(self):
        self.input_topic = rospy.get_param("~input_cmd_topic", "/open_nav/cmd_vel_raw")
        self.output_topic = rospy.get_param("~output_cmd_topic", "/open_nav/cmd_vel")
        self.status_topic = rospy.get_param("~status_topic", "/open_nav/obstacle_status")
        self.marker_topic = rospy.get_param("~marker_topic", "/open_nav/obstacle_markers")
        self.nav_mode_topic = rospy.get_param("~nav_mode_topic", "/open_nav/nav_mode")

        self.use_scan = rospy.get_param("~use_scan", False)
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cloud_topic = rospy.get_param("~pointcloud_topic", "/livox/lidar")
        self.base_frame = rospy.get_param("~base_frame", "open_base")

        self.min_range = float(rospy.get_param("~check_min_range", 0.25))
        self.max_range = float(rospy.get_param("~check_max_range", 4.0))
        self.roi_min_x = float(rospy.get_param("~roi_min_x", 0.15))
        self.roi_max_x = float(rospy.get_param("~roi_max_x", self.max_range))
        self.roi_min_y = float(rospy.get_param("~roi_min_y", -1.4))
        self.roi_max_y = float(rospy.get_param("~roi_max_y", 1.4))
        self.roi_min_z = float(rospy.get_param("~roi_min_z", -0.35))
        self.roi_max_z = float(rospy.get_param("~roi_max_z", 1.4))
        self.ground_z_max = float(rospy.get_param("~ground_z_max", 0.08))

        self.robot_width = float(rospy.get_param("~robot_width", 0.70))
        self.safety_margin = float(rospy.get_param("~safety_margin", 0.15))
        self.side_region_width = float(rospy.get_param("~side_region_width", 0.55))
        self.min_points = int(rospy.get_param("~min_points", 5))

        self.slow_distance = float(rospy.get_param("~slow_distance", 2.0))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.8))
        self.slowdown_factor = float(
            rospy.get_param("~slowdown_factor", rospy.get_param("~max_slowdown_factor", 0.35))
        )
        self.avoid_linear_scale = float(rospy.get_param("~avoid_linear_scale", 0.65))
        self.avoid_angular_vel = float(rospy.get_param("~avoid_angular_vel", 0.28))
        self.min_effective_linear_vel = float(rospy.get_param("~min_effective_linear_vel", 0.25))
        self.max_output_angular_vel = float(rospy.get_param("~max_output_angular_vel", 0.4))

        self.confirm_frames = max(1, int(rospy.get_param("~confirm_frames", 2)))
        self.clear_frames = max(1, int(rospy.get_param("~clear_frames", 3)))
        self.direction_hold_time = float(rospy.get_param("~direction_hold_time", 1.5))
        self.switch_margin = float(rospy.get_param("~switch_margin", 0.35))

        self.pointcloud_timeout = float(rospy.get_param("~pointcloud_timeout", 0.8))
        self.fail_safe_stop = bool(rospy.get_param("~fail_safe_stop", True))
        self.debug_markers = bool(rospy.get_param("~debug_markers", True))

        self.half_width = self.robot_width * 0.5 + self.safety_margin
        self.left_y_min = self.half_width
        self.left_y_max = self.half_width + self.side_region_width
        self.right_y_min = -self.half_width - self.side_region_width
        self.right_y_max = -self.half_width

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.state = STATE_SENSOR_TIMEOUT
        self.pending_state = None
        self.pending_count = 0
        self.avoid_direction = None
        self.avoid_since = rospy.Time(0)
        self.nav_mode = "flat"
        self.last_cmd = Twist()
        self.last_cloud_time = rospy.Time(0)
        self.last_cloud_frame = ""
        self.regions = self._empty_regions()

        self.cmd_pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, ObstacleStatus, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.Subscriber(self.input_topic, Twist, self.cmd_callback, queue_size=10)
        rospy.Subscriber(self.nav_mode_topic, String, self.nav_mode_callback, queue_size=5)
        if self.use_scan:
            rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=5)
        else:
            rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=5)

        self.safety_timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.loginfo(
            "[lidar_obstacle_avoid] input=%s output=%s source=%s base_frame=%s",
            self.input_topic,
            self.output_topic,
            self.scan_topic if self.use_scan else self.cloud_topic,
            self.base_frame,
        )

    def _empty_regions(self):
        return {
            "front": RegionStats(self.roi_max_x),
            "left": RegionStats(self.roi_max_x),
            "right": RegionStats(self.roi_max_x),
        }

    def nav_mode_callback(self, msg):
        self.nav_mode = msg.data.strip() if msg.data else "flat"

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.cmd_pub.publish(self._adjust(msg))
        self._publish_status()

    def timer_callback(self, _event):
        if self._sensor_timed_out():
            self._commit_state(STATE_SENSOR_TIMEOUT)
            if self.fail_safe_stop:
                self.cmd_pub.publish(Twist())
            rospy.logwarn_throttle(
                2.0,
                "[obstacle_avoid] sensor timeout: no valid obstacle data for %.2fs",
                self._pointcloud_age(),
            )
        self._publish_status()
        if self.debug_markers:
            self._publish_markers()

    def scan_callback(self, msg):
        points = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            points.append((r * math.cos(angle), r * math.sin(angle), 0.0))
        self._process_points(points, msg.header.frame_id, msg.header.stamp, apply_ground_filter=False)

    def cloud_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self._process_points(points, msg.header.frame_id, msg.header.stamp, apply_ground_filter=True)

    def _process_points(self, points, source_frame, stamp, apply_ground_filter):
        transform = None
        if source_frame and source_frame != self.base_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(0.05),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "[obstacle_avoid] waiting for TF %s -> %s: %s",
                    source_frame,
                    self.base_frame,
                    exc,
                )
                return

        regions = self._empty_regions()
        obstacle_points = []
        for p in points:
            try:
                x, y, z = float(p[0]), float(p[1]), float(p[2])
            except (TypeError, ValueError, IndexError):
                continue
            if any(math.isnan(v) or math.isinf(v) for v in (x, y, z)):
                continue
            if transform is not None:
                x, y, z = self._transform_point(x, y, z, transform.transform)

            dist_xy = math.hypot(x, y)
            if dist_xy < self.min_range or dist_xy > self.max_range:
                continue
            if x < self.roi_min_x or x > self.roi_max_x:
                continue
            if y < self.roi_min_y or y > self.roi_max_y:
                continue
            if z < self.roi_min_z or z > self.roi_max_z:
                continue
            if apply_ground_filter and z <= self.ground_z_max:
                continue

            if abs(y) <= self.half_width:
                self._add_point(regions["front"], x, y, z)
                obstacle_points.append((x, y, z))
            elif self.left_y_min < y <= self.left_y_max:
                self._add_point(regions["left"], x, y, z)
                obstacle_points.append((x, y, z))
            elif self.right_y_min <= y < self.right_y_max:
                self._add_point(regions["right"], x, y, z)
                obstacle_points.append((x, y, z))

        self._compute_occupancy(regions)
        self.regions = regions
        self.last_cloud_time = stamp if stamp and stamp != rospy.Time(0) else rospy.Time.now()
        self.last_cloud_frame = source_frame

        self._commit_state(self._desired_state())
        self._publish_status()
        if self.debug_markers:
            self._publish_markers(obstacle_points)

    def _add_point(self, region, x, y, z):
        region.point_count += 1
        region.min_distance = min(region.min_distance, x)
        if len(region.points) < 250:
            region.points.append((x, y, z))

    def _compute_occupancy(self, regions):
        x_span = max(self.roi_max_x - self.roi_min_x, 1e-3)
        center_area = max(x_span * (2.0 * self.half_width), 1e-3)
        side_area = max(x_span * self.side_region_width, 1e-3)
        regions["front"].occupancy = regions["front"].point_count / center_area
        regions["left"].occupancy = regions["left"].point_count / side_area
        regions["right"].occupancy = regions["right"].point_count / side_area

    def _transform_point(self, x, y, z, transform):
        q = transform.rotation
        tx = transform.translation.x
        ty = transform.translation.y
        tz = transform.translation.z

        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz

        rx = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        ry = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        rz = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z
        return rx + tx, ry + ty, rz + tz

    def _desired_state(self):
        if self.nav_mode == "stair_execution":
            return STATE_CLEAR
        if self._sensor_timed_out():
            return STATE_SENSOR_TIMEOUT

        front = self.regions["front"]
        if front.point_count < self.min_points or front.min_distance >= self.slow_distance:
            return STATE_CLEAR
        if front.min_distance > self.stop_distance:
            return STATE_SLOW

        left_clearance = self._region_clearance("left")
        right_clearance = self._region_clearance("right")
        left_safe = self._side_safe("left")
        right_safe = self._side_safe("right")

        if self.state in (STATE_AVOID_LEFT, STATE_AVOID_RIGHT):
            held_state = self._held_direction_state(left_safe, right_safe, left_clearance, right_clearance)
            if held_state:
                return held_state

        if left_safe and right_safe:
            if left_clearance >= right_clearance + self.switch_margin:
                return STATE_AVOID_LEFT
            if right_clearance >= left_clearance + self.switch_margin:
                return STATE_AVOID_RIGHT
            return STATE_AVOID_LEFT if left_clearance >= right_clearance else STATE_AVOID_RIGHT
        if left_safe:
            return STATE_AVOID_LEFT
        if right_safe:
            return STATE_AVOID_RIGHT
        return STATE_BLOCKED

    def _held_direction_state(self, left_safe, right_safe, left_clearance, right_clearance):
        if self.avoid_since == rospy.Time(0):
            return None
        held = (rospy.Time.now() - self.avoid_since).to_sec() < self.direction_hold_time
        if self.state == STATE_AVOID_LEFT and left_safe:
            if held or left_clearance + self.switch_margin >= right_clearance:
                return STATE_AVOID_LEFT
        if self.state == STATE_AVOID_RIGHT and right_safe:
            if held or right_clearance + self.switch_margin >= left_clearance:
                return STATE_AVOID_RIGHT
        return None

    def _region_clearance(self, name):
        region = self.regions[name]
        if region.point_count < self.min_points:
            return self.roi_max_x
        return region.min_distance

    def _side_safe(self, name):
        region = self.regions[name]
        if region.point_count < self.min_points:
            return True
        return region.min_distance > self.stop_distance

    def _commit_state(self, desired_state):
        if desired_state == self.state:
            self.pending_state = None
            self.pending_count = 0
            return

        needed = self.clear_frames if desired_state == STATE_CLEAR else self.confirm_frames
        if desired_state != self.pending_state:
            self.pending_state = desired_state
            self.pending_count = 1
        else:
            self.pending_count += 1

        if self.pending_count < needed:
            return

        old_state = self.state
        self.state = desired_state
        self.pending_state = None
        self.pending_count = 0
        if self.state in (STATE_AVOID_LEFT, STATE_AVOID_RIGHT):
            self.avoid_direction = "left" if self.state == STATE_AVOID_LEFT else "right"
            self.avoid_since = rospy.Time.now()
        rospy.loginfo("[obstacle_avoid] %s -> %s", old_state, self.state)

    def _adjust(self, cmd):
        self._commit_state(self._desired_state())

        out = self._copy_twist(cmd)
        if self.nav_mode == "stair_execution":
            return out

        if cmd.linear.x <= 0.0:
            if self.state == STATE_SENSOR_TIMEOUT and self.fail_safe_stop:
                return Twist()
            return out

        if self.state == STATE_CLEAR:
            return out
        if self.state == STATE_SLOW:
            out.linear.x = cmd.linear.x * self._slow_scale()
            return out
        if self.state == STATE_AVOID_LEFT:
            out.linear.x = self._avoid_linear(cmd.linear.x)
            out.angular.z = self._clamp(
                cmd.angular.z + self.avoid_angular_vel,
                -self.max_output_angular_vel,
                self.max_output_angular_vel,
            )
            return out
        if self.state == STATE_AVOID_RIGHT:
            out.linear.x = self._avoid_linear(cmd.linear.x)
            out.angular.z = self._clamp(
                cmd.angular.z - self.avoid_angular_vel,
                -self.max_output_angular_vel,
                self.max_output_angular_vel,
            )
            return out
        if self.state in (STATE_BLOCKED, STATE_SENSOR_TIMEOUT):
            return Twist()
        return out

    def _copy_twist(self, cmd):
        out = Twist()
        out.linear = copy.copy(cmd.linear)
        out.angular = copy.copy(cmd.angular)
        return out

    def _avoid_linear(self, raw_x):
        scaled = raw_x * self.avoid_linear_scale
        return min(raw_x, max(self.min_effective_linear_vel, scaled))

    def _slow_scale(self):
        front_d = self.regions["front"].min_distance
        span = max(self.slow_distance - self.stop_distance, 1e-3)
        scale = (front_d - self.stop_distance) / span
        return self._clamp(max(scale, self.slowdown_factor), self.slowdown_factor, 1.0)

    def _sensor_timed_out(self):
        if self.last_cloud_time == rospy.Time(0):
            return True
        return self._pointcloud_age() > self.pointcloud_timeout

    def _pointcloud_age(self):
        if self.last_cloud_time == rospy.Time(0):
            return float("inf")
        return max(0.0, (rospy.Time.now() - self.last_cloud_time).to_sec())

    def _publish_status(self):
        msg = ObstacleStatus()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.state = self.state
        msg.front_distance = self.regions["front"].min_distance
        msg.left_clearance = self._region_clearance("left")
        msg.right_clearance = self._region_clearance("right")
        msg.front_points = self.regions["front"].point_count
        msg.left_points = self.regions["left"].point_count
        msg.right_points = self.regions["right"].point_count
        msg.front_occupancy = self.regions["front"].occupancy
        msg.left_occupancy = self.regions["left"].occupancy
        msg.right_occupancy = self.regions["right"].occupancy
        msg.pointcloud_age = self._pointcloud_age()
        msg.obstacle_detected = self.regions["front"].point_count >= self.min_points
        msg.fail_safe_active = self.state == STATE_SENSOR_TIMEOUT and self.fail_safe_stop
        self.status_pub.publish(msg)

    def _publish_markers(self, obstacle_points=None):
        obstacle_points = obstacle_points if obstacle_points is not None else self._all_region_points()
        markers = MarkerArray()
        stamp = rospy.Time.now()
        markers.markers.append(
            self._box_marker(0, stamp, self.roi_min_x, self.roi_max_x, -self.half_width, self.half_width, 0.08, 0.15, 0.55)
        )
        markers.markers.append(
            self._box_marker(1, stamp, self.roi_min_x, self.roi_max_x, self.left_y_min, self.left_y_max, 0.15, 0.65, 0.25)
        )
        markers.markers.append(
            self._box_marker(2, stamp, self.roi_min_x, self.roi_max_x, self.right_y_min, self.right_y_max, 0.65, 0.18, 0.18)
        )
        markers.markers.append(self._points_marker(3, stamp, obstacle_points))
        markers.markers.append(self._text_marker(4, stamp))
        self.marker_pub.publish(markers)

    def _all_region_points(self):
        points = []
        for region in self.regions.values():
            points.extend(region.points)
        return points

    def _box_marker(self, marker_id, stamp, x_min, x_max, y_min, y_max, r, g, b):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = stamp
        marker.ns = "obstacle_regions"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (x_min + x_max) * 0.5
        marker.pose.position.y = (y_min + y_max) * 0.5
        marker.pose.position.z = 0.02
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(x_max - x_min, 0.01)
        marker.scale.y = max(y_max - y_min, 0.01)
        marker.scale.z = 0.04
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.18
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def _points_marker(self, marker_id, stamp, points):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = stamp
        marker.ns = "obstacle_points"
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.b = 0.05
        marker.color.a = 0.85
        marker.lifetime = rospy.Duration(0.5)
        for x, y, z in points[:500]:
            marker.points.append(Point(x=x, y=y, z=z))
        return marker

    def _text_marker(self, marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = stamp
        marker.ns = "obstacle_state"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = max(0.6, self.roi_min_x)
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.roi_max_z + 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = self.state
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def _clamp(self, value, lo, hi):
        return max(lo, min(hi, value))


def main():
    rospy.init_node("lidar_obstacle_avoid")
    LidarObstacleAvoid()
    rospy.spin()


if __name__ == "__main__":
    main()
