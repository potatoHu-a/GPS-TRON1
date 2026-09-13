#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Patrol mission layer — services only; waypoint truth lives in collector."""

import os
import sys

import rospy
import yaml
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import enu_to_geodetic

try:
    from tron_open_space_nav.msg import MissionStatus
    from tron_open_space_nav.srv import (
        MissionPatrolStart,
        MissionPatrolStartResponse,
        DeleteWaypoint,
        MoveWaypoint,
    )
except ImportError:
    MissionStatus = None
    MissionPatrolStart = None
    DeleteWaypoint = None
    MoveWaypoint = None

PARAM_NS = "/tron_open_space_nav"


class PatrolMissionManager(object):
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.patrol_mode = rospy.get_param("~patrol_mode", "once")
        self.loop_count = int(rospy.get_param("~loop_count", 0))
        self.save_path = rospy.get_param("~save_path", "patrol_waypoints.yaml")
        if not os.path.isabs(self.save_path):
            pkg_config = os.path.join(os.path.dirname(SCRIPT_DIR), "config")
            self.save_path = os.path.join(pkg_config, self.save_path)

        self.nav_enable_topic = rospy.get_param(
            "~navigate_enable_topic", "/open_nav/mission/navigate_enable"
        )
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/open_nav/cmd_vel_raw")
        self.tracker_status_topic = rospy.get_param(
            "~tracker_status_topic", "/open_nav/tracker_status"
        )
        self.collector_ns = "/gps_multi_waypoint_collector"

        self.state = "IDLE"
        self.current_loop = 0
        self._pingpong_forward = True
        self._confirmed_poses = []
        self._pending_count = 0
        self._tracker_status = None

        self.nav_enable_pub = rospy.Publisher(
            self.nav_enable_topic, Bool, queue_size=1, latch=True
        )
        self.selected_pub = rospy.Publisher(
            "/open_nav/selected_waypoint", Int32, queue_size=1, latch=True
        )
        self.tracker_waypoints_pub = rospy.Publisher(
            "/open_nav/tracker_waypoints", PoseArray, queue_size=1, latch=True
        )
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        if MissionStatus is not None:
            self.status_pub = rospy.Publisher(
                "/open_nav/mission_status", MissionStatus, queue_size=1, latch=True
            )
        else:
            self.status_pub = None

        rospy.Subscriber("/open_nav/mission_path", Path, self._mission_path_cb, queue_size=1)
        rospy.Subscriber("/open_nav/waypoints", Path, self._pending_waypoints_cb, queue_size=1)
        if MissionStatus is not None:
            rospy.Subscriber(
                self.tracker_status_topic, MissionStatus, self._tracker_status_cb, queue_size=1
            )

        rospy.Service("/open_nav/mission/send", Trigger, self.send_service)
        rospy.Service("/open_nav/mission/start_patrol", MissionPatrolStart, self.start_patrol_service)
        rospy.Service("/open_nav/mission/pause", Empty, self.pause_service)
        rospy.Service("/open_nav/mission/resume", Empty, self.resume_service)
        rospy.Service("/open_nav/mission/stop", Empty, self.stop_service)
        rospy.Service("/open_nav/mission/clear", Empty, self.clear_service)
        rospy.Service("/open_nav/mission/save", Trigger, self.save_service)
        rospy.Service("/open_nav/mission/load", Trigger, self.load_service)
        if DeleteWaypoint is not None:
            rospy.Service(
                "/open_nav/mission/delete_waypoint", DeleteWaypoint, self.delete_waypoint_service
            )
        if MoveWaypoint is not None:
            rospy.Service(
                "/open_nav/mission/move_waypoint", MoveWaypoint, self.move_waypoint_service
            )

        self.nav_enable_pub.publish(Bool(data=False))
        rospy.Timer(rospy.Duration(0.5), self._status_timer, oneshot=False)
        self._publish_status()
        rospy.loginfo("[patrol_mission_manager] ready state=%s", self.state)

    def _editable(self):
        return self.state not in ("RUNNING", "PAUSED")

    def _mission_path_cb(self, msg):
        self._confirmed_poses = [ps.pose for ps in msg.poses]

    def _pending_waypoints_cb(self, msg):
        self._pending_count = len(msg.poses)

    def _tracker_status_cb(self, msg):
        prev_state = self._tracker_status.state if self._tracker_status else ""
        self._tracker_status = msg
        if (
            self.state == "RUNNING"
            and msg.state == "completed"
            and prev_state != "completed"
        ):
            self._on_leg_completed()

    def _call_trigger(self, name):
        rospy.wait_for_service(name, timeout=5.0)
        return rospy.ServiceProxy(name, Trigger)()

    def _call_empty(self, name):
        rospy.wait_for_service(name, timeout=5.0)
        return rospy.ServiceProxy(name, Empty)()

    def _publish_tracker_waypoints(self, poses):
        pa = PoseArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = self.map_frame
        pa.poses = list(poses)
        self.tracker_waypoints_pub.publish(pa)
        rospy.loginfo("[patrol_mission_manager] tracker waypoints: %d", len(poses))

    def send_service(self, _req):
        try:
            self.nav_enable_pub.publish(Bool(data=False))
            rospy.sleep(0.05)
            resp = self._call_trigger(self.collector_ns + "/confirm_mission")
            if resp.success:
                self.state = "READY"
                self.current_loop = 0
                self._pingpong_forward = True
                self._publish_status()
            return resp
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def start_patrol_service(self, req):
        if not self._confirmed_poses:
            return MissionPatrolStartResponse(
                success=False, message="no confirmed mission; call send first"
            )
        if req.patrol_mode:
            self.patrol_mode = req.patrol_mode
        self.loop_count = int(req.loop_count)
        self.current_loop = 0
        self._pingpong_forward = True
        self._publish_patrol_leg(forward=True)
        return MissionPatrolStartResponse(
            success=True,
            message="patrol started mode=%s" % self.patrol_mode,
        )

    def _leg_poses(self, forward):
        if not self._confirmed_poses:
            return []
        if forward or len(self._confirmed_poses) <= 1:
            return list(self._confirmed_poses)
        return list(reversed(self._confirmed_poses))[1:]

    def _publish_patrol_leg(self, forward):
        poses = self._leg_poses(forward)
        if not poses:
            return
        self._publish_tracker_waypoints(poses)
        self.nav_enable_pub.publish(Bool(data=True))
        self.state = "RUNNING"
        self._publish_status()

    def _on_leg_completed(self):
        if self.patrol_mode == "once":
            self.state = "COMPLETED"
            self.nav_enable_pub.publish(Bool(data=False))
            self._publish_zero_cmd()
            self._publish_status()
            return
        self.current_loop += 1
        if self.loop_count > 0 and self.current_loop >= self.loop_count:
            self.state = "COMPLETED"
            self.nav_enable_pub.publish(Bool(data=False))
            self._publish_zero_cmd()
            self._publish_status()
            return
        if self.patrol_mode == "pingpong":
            self._pingpong_forward = not self._pingpong_forward
            self._publish_patrol_leg(forward=self._pingpong_forward)
        else:
            self._publish_patrol_leg(forward=True)

    def pause_service(self, _req):
        self.nav_enable_pub.publish(Bool(data=False))
        self._publish_zero_cmd()
        if self.state == "RUNNING":
            self.state = "PAUSED"
        self._publish_status()
        return EmptyResponse()

    def resume_service(self, _req):
        if self.state in ("PAUSED", "STOPPED", "READY") and self._confirmed_poses:
            self.state = "RUNNING"
            self.nav_enable_pub.publish(Bool(data=True))
        self._publish_status()
        return EmptyResponse()

    def stop_service(self, _req):
        self.nav_enable_pub.publish(Bool(data=False))
        self._publish_zero_cmd()
        if self.state in ("RUNNING", "PAUSED", "READY"):
            self.state = "STOPPED"
        self._publish_status()
        return EmptyResponse()

    def clear_service(self, _req):
        self.nav_enable_pub.publish(Bool(data=False))
        self._publish_zero_cmd()
        try:
            self._call_empty(self.collector_ns + "/clear")
        except Exception as exc:
            rospy.logwarn("[patrol_mission_manager] clear: %s", exc)
        empty_pa = PoseArray()
        empty_pa.header.stamp = rospy.Time.now()
        empty_pa.header.frame_id = self.map_frame
        self.tracker_waypoints_pub.publish(empty_pa)
        self._confirmed_poses = []
        self._pending_count = 0
        self.state = "IDLE"
        self.current_loop = 0
        self._publish_status()
        return EmptyResponse()

    def delete_waypoint_service(self, req):
        if not self._editable():
            return DeleteWaypointResponse(
                success=False, message="cannot modify mission while running"
            )
        try:
            rospy.wait_for_service(self.collector_ns + "/delete_waypoint", timeout=5.0)
            proxy = rospy.ServiceProxy(self.collector_ns + "/delete_waypoint", DeleteWaypoint)
            return proxy(req)
        except Exception as exc:
            return DeleteWaypointResponse(success=False, message=str(exc))

    def move_waypoint_service(self, req):
        if not self._editable():
            return MoveWaypointResponse(
                success=False, message="cannot modify mission while running"
            )
        try:
            rospy.wait_for_service(self.collector_ns + "/move_waypoint", timeout=5.0)
            proxy = rospy.ServiceProxy(self.collector_ns + "/move_waypoint", MoveWaypoint)
            return proxy(req)
        except Exception as exc:
            return MoveWaypointResponse(success=False, message=str(exc))

    def _origin_ready(self):
        return rospy.get_param(PARAM_NS + "/origin_ready", False)

    def save_service(self, _req):
        try:
            pending = rospy.wait_for_message("/open_nav/waypoints", Path, timeout=2.0)
            rows = []
            lat0 = lon0 = alt0 = 0.0
            if self._origin_ready():
                lat0 = rospy.get_param(PARAM_NS + "/origin_lat")
                lon0 = rospy.get_param(PARAM_NS + "/origin_lon")
                alt0 = rospy.get_param(PARAM_NS + "/origin_alt")
            for ps in pending.poses:
                lat, lon = 0.0, 0.0
                if self._origin_ready():
                    lat, lon, _a = enu_to_geodetic(
                        ps.pose.position.x,
                        ps.pose.position.y,
                        ps.pose.position.z,
                        lat0,
                        lon0,
                        alt0,
                    )
                rows.append({
                    "x": float(ps.pose.position.x),
                    "y": float(ps.pose.position.y),
                    "latitude": float(lat),
                    "longitude": float(lon),
                })
            payload = {
                "frame_id": self.map_frame,
                "patrol_mode": self.patrol_mode,
                "loop_count": self.loop_count,
                "waypoints": rows,
            }
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(payload, f, default_flow_style=False, allow_unicode=True)
            return TriggerResponse(success=True, message="saved to " + self.save_path)
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def load_service(self, _req):
        if not self._editable():
            return TriggerResponse(success=False, message="cannot load while running")
        try:
            if not os.path.isfile(self.save_path):
                return TriggerResponse(success=False, message="file not found")
            with open(self.save_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            self.patrol_mode = data.get("patrol_mode", self.patrol_mode)
            self.loop_count = int(data.get("loop_count", self.loop_count))
            collector_save = rospy.get_param(
                self.collector_ns + "/save_path",
                os.path.join(os.path.dirname(SCRIPT_DIR), "config", "gps_waypoints.yaml"),
            )
            os.makedirs(os.path.dirname(collector_save), exist_ok=True)
            with open(collector_save, "w", encoding="utf-8") as f:
                yaml.safe_dump({"waypoints": data.get("waypoints", [])}, f)
            resp = self._call_trigger(self.collector_ns + "/load_pending")
            self.nav_enable_pub.publish(Bool(data=False))
            self.state = "IDLE"
            self._confirmed_poses = []
            self._publish_status()
            return resp
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def _publish_zero_cmd(self):
        z = Twist()
        for _ in range(5):
            self.cmd_pub.publish(z)

    def _publish_status(self):
        if self.status_pub is None:
            return
        msg = MissionStatus()
        msg.state = self.state
        msg.patrol_mode = self.patrol_mode
        msg.current_loop = self.current_loop
        msg.loop_count = self.loop_count

        if self.state in ("READY", "RUNNING", "PAUSED", "COMPLETED", "STOPPED"):
            msg.total = len(self._confirmed_poses)
        else:
            msg.total = self._pending_count

        if self._tracker_status is not None and self.state in ("RUNNING", "PAUSED", "COMPLETED"):
            msg.current_index = self._tracker_status.current_index
            msg.distance_to_goal = self._tracker_status.distance_to_goal
            if self._tracker_status.total > 0:
                msg.total = self._tracker_status.total
        else:
            msg.current_index = 0
            msg.distance_to_goal = 0.0

        idx = msg.current_index + 1 if msg.total > 0 else 0
        msg.status_text = "%s | P%d/%d | loop %d | dist=%.1fm" % (
            self.state,
            idx,
            msg.total,
            self.current_loop,
            msg.distance_to_goal,
        )
        self.status_pub.publish(msg)

    def _status_timer(self, _event):
        if self.state in ("PAUSED", "STOPPED"):
            self._publish_zero_cmd()
        self._publish_status()


def main():
    rospy.init_node("patrol_mission_manager")
    PatrolMissionManager()
    rospy.spin()


if __name__ == "__main__":
    main()
