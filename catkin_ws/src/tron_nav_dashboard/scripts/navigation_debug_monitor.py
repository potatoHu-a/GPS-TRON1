#!/usr/bin/env python3

import math
import threading

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from tron_local_collision_safety.msg import ObstacleStatus


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def finite_sqrt(value):
    return math.sqrt(value) if math.isfinite(value) and value >= 0.0 else float("nan")


class NavigationDebugMonitor(object):
    """Read-only aggregation of localization, navigation and safety diagnostics."""

    def __init__(self):
        self.lock = threading.RLock()
        self.messages = {}
        self.received = {}
        self.course_min_speed = float(rospy.get_param("~course_min_speed", 0.3))
        self.publish_rate = float(rospy.get_param("~publish_rate", 8.0))
        self.publisher = rospy.Publisher(
            "/open_nav/navigation_debug", DiagnosticArray, queue_size=5
        )

        self._subscribe("gps", "/gps/fix", NavSatFix)
        self._subscribe("course", "/gps/course", Float64)
        self._subscribe("fastlio", "/Odometry", Odometry)
        self._subscribe("fused", "/open_nav/odom", Odometry)
        self._subscribe("gps_filtered", "/open_nav/gps_filtered_pose", PoseStamped)
        self._subscribe("mission", "/open_nav/mission_path", Path)
        self._subscribe("local_plan", "/open_nav/local_plan", Path)
        self._subscribe("cmd_raw", "/open_nav/cmd_vel_raw", Twist)
        self._subscribe("cmd_final", "/open_nav/cmd_vel", Twist)
        self._subscribe("safety", "/open_nav/obstacle_status", ObstacleStatus)
        rospy.Subscriber(
            "/open_nav/local_planner/status",
            DiagnosticArray,
            lambda msg: self._store("planner", msg),
            queue_size=5,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._timer_callback
        )
        rospy.loginfo(
            "[navigation_debug_monitor] read-only diagnostics -> /open_nav/navigation_debug"
        )

    def _subscribe(self, key, topic, message_type):
        rospy.Subscriber(
            topic, message_type, lambda msg, k=key: self._store(k, msg), queue_size=20
        )

    def _store(self, key, message):
        with self.lock:
            self.messages[key] = message
            self.received[key] = rospy.Time.now()

    def _snapshot(self):
        with self.lock:
            return dict(self.messages), dict(self.received)

    @staticmethod
    def _age(now, received, key):
        stamp = received.get(key)
        return -1.0 if stamp is None else max(0.0, (now - stamp).to_sec())

    @staticmethod
    def _format(value, precision=3):
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, float):
            if not math.isfinite(value):
                return "nan"
            return ("{:.%df}" % precision).format(value)
        return str(value)

    def _status(self, name, level, message, values):
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = "TRON1"
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key=key, value=self._format(value))
            for key, value in values.items()
        ]
        return status

    @staticmethod
    def _planner_values(planner_message):
        if planner_message is None or not planner_message.status:
            return {}
        selected = next(
            (
                status
                for status in planner_message.status
                if status.name == "local_trajectory_planner"
            ),
            planner_message.status[0],
        )
        values = {item.key: item.value for item in selected.values}
        values["diagnostic_level"] = str(selected.level)
        values["diagnostic_message"] = selected.message
        return values

    @staticmethod
    def _mission_metrics(fused, mission):
        empty = {
            "target_x": float("nan"),
            "target_y": float("nan"),
            "desired_bearing_deg": float("nan"),
            "heading_error_deg": float("nan"),
            "cross_track_error": float("nan"),
            "distance_to_goal": float("nan"),
            "target_index": -1,
        }
        if fused is None or mission is None or not mission.poses:
            return empty

        robot_x = fused.pose.pose.position.x
        robot_y = fused.pose.pose.position.y
        robot_yaw = yaw_from_quaternion(fused.pose.pose.orientation)
        points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in mission.poses
        ]
        nearest_index = min(
            range(len(points)),
            key=lambda index: math.hypot(
                points[index][0] - robot_x, points[index][1] - robot_y
            ),
        )
        target_index = min(nearest_index + 1, len(points) - 1)
        target_x, target_y = points[target_index]
        desired = math.atan2(target_y - robot_y, target_x - robot_x)

        cross_track = math.hypot(points[0][0] - robot_x, points[0][1] - robot_y)
        for start, end in zip(points[:-1], points[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-9:
                distance = math.hypot(start[0] - robot_x, start[1] - robot_y)
            else:
                fraction = max(
                    0.0,
                    min(
                        1.0,
                        ((robot_x - start[0]) * dx + (robot_y - start[1]) * dy)
                        / length_sq,
                    ),
                )
                projection_x = start[0] + fraction * dx
                projection_y = start[1] + fraction * dy
                distance = math.hypot(robot_x - projection_x, robot_y - projection_y)
            cross_track = min(cross_track, distance)

        goal_x, goal_y = points[-1]
        return {
            "target_x": target_x,
            "target_y": target_y,
            "desired_bearing_deg": math.degrees(desired),
            "heading_error_deg": math.degrees(normalize_angle(desired - robot_yaw)),
            "cross_track_error": cross_track,
            "distance_to_goal": math.hypot(goal_x - robot_x, goal_y - robot_y),
            "target_index": target_index,
        }

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        messages, received = self._snapshot()
        gps = messages.get("gps")
        fastlio = messages.get("fastlio")
        fused = messages.get("fused")
        mission = messages.get("mission")
        safety = messages.get("safety")
        cmd_raw = messages.get("cmd_raw")
        cmd_final = messages.get("cmd_final")
        course = messages.get("course")
        planner = self._planner_values(messages.get("planner"))
        mission_metrics = self._mission_metrics(fused, mission)

        gps_age = self._age(now, received, "gps")
        fix_ok = gps is not None and gps.status.status >= NavSatStatus.STATUS_FIX
        covariance = [float("nan")] * 3
        if gps is not None and len(gps.position_covariance) >= 9:
            covariance = [
                gps.position_covariance[0],
                gps.position_covariance[4],
                gps.position_covariance[8],
            ]
        gnss_values = {
            "fix_status": "FIX" if fix_ok else "NO_FIX",
            "latitude": float("nan") if gps is None else gps.latitude,
            "longitude": float("nan") if gps is None else gps.longitude,
            "altitude": float("nan") if gps is None else gps.altitude,
            "covariance_east": covariance[0],
            "covariance_north": covariance[1],
            "covariance_up": covariance[2],
            "sigma_east": finite_sqrt(covariance[0]),
            "sigma_north": finite_sqrt(covariance[1]),
            "sigma_up": finite_sqrt(covariance[2]),
            "message_age": gps_age,
        }

        fast_yaw = (
            float("nan")
            if fastlio is None
            else math.degrees(yaw_from_quaternion(fastlio.pose.pose.orientation))
        )
        fused_yaw = (
            float("nan")
            if fused is None
            else math.degrees(yaw_from_quaternion(fused.pose.pose.orientation))
        )
        fast_speed = (
            float("nan")
            if fastlio is None
            else math.hypot(fastlio.twist.twist.linear.x, fastlio.twist.twist.linear.y)
        )
        fused_speed = (
            float("nan")
            if fused is None
            else math.hypot(fused.twist.twist.linear.x, fused.twist.twist.linear.y)
        )
        course_valid = (
            course is not None
            and math.isfinite(fused_speed)
            and fused_speed >= self.course_min_speed
            and self._age(now, received, "course") <= 2.0
        )
        localization_values = {
            "fastlio_x": float("nan") if fastlio is None else fastlio.pose.pose.position.x,
            "fastlio_y": float("nan") if fastlio is None else fastlio.pose.pose.position.y,
            "fastlio_yaw_deg": fast_yaw,
            "fused_x": float("nan") if fused is None else fused.pose.pose.position.x,
            "fused_y": float("nan") if fused is None else fused.pose.pose.position.y,
            "fused_yaw_deg": fused_yaw,
            "fastlio_speed": fast_speed,
            "fused_speed": fused_speed,
            "fastlio_age": self._age(now, received, "fastlio"),
            "fused_age": self._age(now, received, "fused"),
            "gnss_course_deg": float("nan") if course is None else course.data,
            "gnss_course_valid": course_valid,
            "yaw_alignment_explicit": False,
        }

        navigation_values = dict(mission_metrics)
        navigation_values["mission_points"] = 0 if mission is None else len(mission.poses)
        local_plan = messages.get("local_plan")
        navigation_values["local_plan_points"] = (
            0 if local_plan is None else len(local_plan.poses)
        )

        control_values = {
            "cmd_raw_linear": float("nan") if cmd_raw is None else cmd_raw.linear.x,
            "cmd_raw_angular": float("nan") if cmd_raw is None else cmd_raw.angular.z,
            "cmd_final_linear": float("nan") if cmd_final is None else cmd_final.linear.x,
            "cmd_final_angular": float("nan") if cmd_final is None else cmd_final.angular.z,
            "cmd_raw_age": self._age(now, received, "cmd_raw"),
            "cmd_final_age": self._age(now, received, "cmd_final"),
            "controller_dry_run": rospy.get_param(
                "/tron_controller_bridge/dry_run", True
            ),
        }
        safety_values = {
            "state": "UNKNOWN" if safety is None else safety.state,
            "front_distance": float("nan") if safety is None else safety.front_distance,
            "collision_risk": float("nan") if safety is None else safety.collision_risk,
            "pointcloud_age": float("nan") if safety is None else safety.pointcloud_age,
            "message_age": self._age(now, received, "safety"),
        }
        planner_values = {
            "mode": planner.get("mode", "SHADOW"),
            "state": planner.get("state", "WAITING"),
            "total_candidates": planner.get("candidate_count", "0"),
            "valid_candidates": planner.get("valid_candidate_count", "0"),
            "best_v": planner.get("best_linear_velocity", "nan"),
            "best_w": planner.get("best_angular_velocity", "nan"),
            "best_clearance": planner.get("best_clearance", "nan"),
            "best_score": planner.get("best_score", "nan"),
            "message_age": self._age(now, received, "planner"),
        }

        array = DiagnosticArray()
        array.header.stamp = now
        array.status = [
            self._status(
                "navigation_debug/GNSS",
                DiagnosticStatus.OK if fix_ok and gps_age <= 2.0 else DiagnosticStatus.WARN,
                "FIX" if fix_ok else "NO_FIX",
                gnss_values,
            ),
            self._status(
                "navigation_debug/Localization",
                DiagnosticStatus.OK
                if 0.0 <= self._age(now, received, "fused") <= 0.5
                else DiagnosticStatus.WARN,
                "fused pose and yaw comparison",
                localization_values,
            ),
            self._status(
                "navigation_debug/Navigation",
                DiagnosticStatus.OK if mission is not None and mission.poses else DiagnosticStatus.WARN,
                "mission geometry",
                navigation_values,
            ),
            self._status(
                "navigation_debug/Control",
                DiagnosticStatus.OK,
                "read-only command comparison",
                control_values,
            ),
            self._status(
                "navigation_debug/Safety",
                DiagnosticStatus.OK if safety is not None else DiagnosticStatus.WARN,
                safety_values["state"],
                safety_values,
            ),
            self._status(
                "navigation_debug/Planner",
                DiagnosticStatus.OK if planner.get("state") == "OK" else DiagnosticStatus.WARN,
                planner.get("diagnostic_message", "waiting for planner"),
                planner_values,
            ),
        ]
        self.publisher.publish(array)


def main():
    rospy.init_node("navigation_debug_monitor")
    NavigationDebugMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
