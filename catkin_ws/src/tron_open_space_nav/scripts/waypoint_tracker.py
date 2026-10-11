#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys

import rospy
import yaml
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import geodetic_to_enu, normalize_angle

try:
    from tron_open_space_nav.msg import MissionStatus
except ImportError:
    MissionStatus = None


class WaypointTracker(object):
    """GPS / map / RViz waypoints + Pure Pursuit -> /open_nav/cmd_vel_raw."""

    PARAM_NS = "/tron_open_space_nav"
    SEQUENTIAL_MODES = ("single_goal", "multi_goal", "rviz_topic")

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/open_nav/odom")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/open_nav/cmd_vel_raw")
        self.loop = rospy.get_param("~loop", True)
        self.lookahead = rospy.get_param("~lookahead_distance", 1.0)
        self.goal_tol = rospy.get_param("~goal_tolerance", 0.5)
        self.min_v = rospy.get_param("~min_linear_vel", 0.4)
        self.max_v = rospy.get_param("~max_linear_vel", 0.5)
        self.min_effective_forward_v = rospy.get_param("~min_effective_forward_vel", 0.5)
        self.max_w = rospy.get_param("~max_angular_vel", 0.4)
        self.rate_hz = rospy.get_param("~control_rate", 20.0)
        self.use_map_waypoints = rospy.get_param("~use_map_waypoints", False)
        self.debug = rospy.get_param("~debug", True)

        source = rospy.get_param("~waypoint_source", "manual_yaml")
        if source == "rviz_topic":
            rospy.logwarn(
                "[waypoint_tracker] waypoint_source=rviz_topic is deprecated; use multi_goal"
            )
            source = "multi_goal"
        self.waypoint_source = source

        self.single_goal_topic = rospy.get_param(
            "~single_goal_topic", "/move_base_simple/goal"
        )
        self.waypoints_topic = rospy.get_param("~waypoints_topic", "/open_nav/waypoints")
        self.mission_status_topic = rospy.get_param(
            "~mission_status_topic", "/open_nav/mission_status"
        )
        pkg_config = rospy.get_param("~config_dir", "")
        if not pkg_config:
            pkg_config = os.path.join(os.path.dirname(SCRIPT_DIR), "config")
        default_gps_yaml = os.path.join(pkg_config, "gps_waypoints.yaml")
        self.gps_waypoints_file = rospy.get_param("~gps_waypoints_file", default_gps_yaml)

        self.waypoints = self._load_waypoints()
        self.map_waypoints = rospy.get_param("~map_waypoints", [])
        self.path = []
        self.path_ready = False
        self.mission_index = 0
        self.mission_state = "idle"
        self.nav_enabled = True

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.have_pose = False

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        if MissionStatus is not None:
            self.status_pub = rospy.Publisher(
                self.mission_status_topic, MissionStatus, queue_size=1, latch=True
            )
        else:
            self.status_pub = None
            rospy.logwarn("[waypoint_tracker] MissionStatus msg not built yet")

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(
            "/open_nav/mission/navigate_enable",
            Bool,
            self._nav_enable_callback,
            queue_size=1,
        )

        if self.waypoint_source == "single_goal":
            rospy.Subscriber(
                self.single_goal_topic,
                PoseStamped,
                self.single_goal_callback,
                queue_size=10,
            )
        elif self.waypoint_source == "multi_goal":
            rospy.Subscriber(
                self.waypoints_topic,
                PoseArray,
                self.waypoints_callback,
                queue_size=1,
            )

        rospy.Timer(rospy.Duration(1.0), self._try_build_path, oneshot=False)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.control_loop)
        self._publish_status()

        rospy.loginfo(
            "[waypoint_tracker] source=%s loop=%s lookahead=%.2f tol=%.2f",
            self.waypoint_source,
            self.loop,
            self.lookahead,
            self.goal_tol,
        )

    def _publish_status(self, state=None):
        if self.status_pub is None:
            return
        if state is not None:
            self.mission_state = state

        msg = MissionStatus()
        msg.current_index = self.mission_index
        msg.total = len(self.path)
        msg.state = self.mission_state
        msg.distance_to_goal = 0.0
        if self.path_ready and self.mission_index < len(self.path):
            tx, ty = self.path[self.mission_index]
            msg.distance_to_goal = float(self._dist_to(tx, ty))
        self.status_pub.publish(msg)

    def _nav_enable_callback(self, msg):
        self.nav_enabled = msg.data
        if not self.nav_enabled:
            self.cmd_pub.publish(Twist())
            if self.path_ready and self.mission_state == "navigating":
                self._publish_status("paused")
            return
        if self.path_ready and self.mission_state in ("paused", "ready", "stopped"):
            self._publish_status("navigating")

    def _load_waypoints_from_yaml(self, path):
        if not os.path.isfile(path):
            return []
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return data.get("waypoints", []) or []
        except Exception as exc:
            rospy.logwarn("[waypoint_tracker] failed to load %s: %s", path, exc)
            return []

    def _load_waypoints(self):
        if self.waypoint_source in self.SEQUENTIAL_MODES:
            return []

        if rospy.has_param("~waypoints"):
            return rospy.get_param("~waypoints")

        gps_wps = self._load_waypoints_from_yaml(self.gps_waypoints_file)
        if gps_wps:
            rospy.loginfo(
                "[waypoint_tracker] loaded %d waypoints from %s",
                len(gps_wps),
                self.gps_waypoints_file,
            )
            return gps_wps
        return []

    def _set_path(self, path_points, state="navigating"):
        self.path = path_points
        self.path_ready = len(self.path) > 0
        self.mission_index = 0
        self._publish_status(state)
        rospy.loginfo("[waypoint_tracker] path set: %d points, state=%s", len(self.path), state)

    def single_goal_callback(self, msg):
        pt = (msg.pose.position.x, msg.pose.position.y)
        self._set_path([pt], state="navigating")

    def waypoints_callback(self, msg):
        if not msg.poses:
            self._set_path([], state="idle")
            return
        path = [(p.position.x, p.position.y) for p in msg.poses]
        state = "navigating" if self.nav_enabled else "ready"
        self._set_path(path, state=state)

    def _try_build_path(self, _event):
        if self.waypoint_source in self.SEQUENTIAL_MODES:
            return

        if self.path_ready:
            gps_wps = self._load_waypoints_from_yaml(self.gps_waypoints_file)
            if gps_wps and len(gps_wps) != len(self.waypoints):
                self.waypoints = gps_wps
                self.path_ready = False
            return

        if self.use_map_waypoints and self.map_waypoints:
            path = [(float(wp["x"]), float(wp["y"])) for wp in self.map_waypoints]
            if self.loop and len(path) > 1:
                path.append(path[0])
            self._set_path(path, state="navigating")
            return

        if not rospy.get_param(self.PARAM_NS + "/origin_ready", False):
            return
        if not self.waypoints:
            return

        lat0 = rospy.get_param(self.PARAM_NS + "/origin_lat")
        lon0 = rospy.get_param(self.PARAM_NS + "/origin_lon")
        alt0 = rospy.get_param(self.PARAM_NS + "/origin_alt")

        path = []
        for wp in self.waypoints:
            east, north, _up = geodetic_to_enu(wp["lat"], wp["lon"], alt0, lat0, lon0, alt0)
            path.append((east, north))

        if self.loop and len(path) > 1:
            path.append(path[0])

        self.waypoints = self.waypoints
        self._set_path(path, state="navigating")

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = math.atan2(siny, cosy)
        self.have_pose = True

    def _dist_to(self, px, py):
        return math.hypot(px - self.pose_x, py - self.pose_y)

    def _advance_sequential_if_reached(self):
        if self.mission_index >= len(self.path):
            if self.mission_state != "completed":
                self._publish_status("completed")
            return False

        tx, ty = self.path[self.mission_index]
        if self._dist_to(tx, ty) >= self.goal_tol:
            return True

        reached = self.mission_index + 1
        self.mission_index += 1
        if self.mission_index >= len(self.path):
            self._publish_status("completed")
            rospy.loginfo("[waypoint_tracker] mission completed (%d waypoints)", len(self.path))
            return False

        self._publish_status("navigating")
        rospy.loginfo(
            "[waypoint_tracker] reached P%d, advancing to P%d",
            reached,
            self.mission_index + 1,
        )
        return True

    def _nearest_index(self):
        best_i = self.mission_index
        best_d = float("inf")
        n = len(self.path)
        search_end = min(n, self.mission_index + n // 2 + 2) if self.loop else n
        for i in range(self.mission_index, search_end):
            idx = i % n if self.loop else i
            px, py = self.path[idx]
            d = self._dist_to(px, py)
            if d < best_d:
                best_d = d
                best_i = idx
        self.mission_index = best_i
        return best_i

    def _lookahead_point(self):
        if not self.path:
            return None

        if self.waypoint_source in self.SEQUENTIAL_MODES:
            if self.mission_index >= len(self.path):
                return None
            return self.path[self.mission_index]

        self._nearest_index()
        n = len(self.path)
        for step in range(n):
            idx = (self.mission_index + step) % n if self.loop else min(self.mission_index + step, n - 1)
            px, py = self.path[idx]
            d = self._dist_to(px, py)
            if d >= self.lookahead:
                return px, py
            if not self.loop and idx == n - 1:
                return px, py
        return self.path[-1]

    def _compute_cmd(self, tx, ty, dist):
        target_heading = math.atan2(ty - self.pose_y, tx - self.pose_x)
        heading_err = normalize_angle(target_heading - self.pose_yaw)
        cmd = Twist()

        if (
            self.waypoint_source not in self.SEQUENTIAL_MODES
            and not self.loop
            and dist < self.goal_tol
        ):
            return cmd

        speed = min(self.max_v, 0.25 * dist)
        speed = max(self.min_v, min(self.max_v, speed))

        if self.lookahead > 0.1:
            curvature = 2.0 * math.sin(heading_err) / self.lookahead
        else:
            curvature = 0.0

        turn_scale = max(0.35, 1.0 - abs(heading_err) / math.pi)
        cmd.linear.x = speed * turn_scale
        if abs(heading_err) < (0.5 * math.pi) and cmd.linear.x > 1e-3:
            cmd.linear.x = min(self.max_v, max(self.min_effective_forward_v, cmd.linear.x))
            cmd.angular.z = max(-self.max_w, min(self.max_w, cmd.linear.x * curvature))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_w if heading_err > 0.0 else -self.max_w
        return cmd

    def control_loop(self, _event):
        if not self.path_ready or not self.have_pose:
            return
        if not self.nav_enabled:
            self.cmd_pub.publish(Twist())
            return

        if self.waypoint_source in self.SEQUENTIAL_MODES:
            if self.mission_state == "completed":
                self.cmd_pub.publish(Twist())
                return
            if not self._advance_sequential_if_reached():
                if self.mission_state == "completed":
                    self.cmd_pub.publish(Twist())
                return

        target = self._lookahead_point()
        if target is None:
            self.cmd_pub.publish(Twist())
            return

        tx, ty = target
        dist = self._dist_to(tx, ty)
        if dist < 1e-3:
            return

        cmd = self._compute_cmd(tx, ty, dist)

        if self.debug:
            heading_err = normalize_angle(
                math.atan2(ty - self.pose_y, tx - self.pose_x) - self.pose_yaw
            )
            rospy.loginfo(
                "[waypoint_tracker] idx=%d/%d state=%s robot=(%.2f,%.2f) "
                "goal=(%.2f,%.2f) dist=%.2f err=%.2f cmd=(%.2f,%.2f)",
                self.mission_index,
                len(self.path),
                self.mission_state,
                self.pose_x,
                self.pose_y,
                tx,
                ty,
                dist,
                heading_err,
                cmd.linear.x,
                cmd.angular.z,
            )

        self.cmd_pub.publish(cmd)


def main():
    rospy.init_node("waypoint_tracker")
    WaypointTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
