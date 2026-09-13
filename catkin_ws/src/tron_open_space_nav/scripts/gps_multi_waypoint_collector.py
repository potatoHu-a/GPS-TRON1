#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-waypoint collector — single source of truth for pending / confirmed waypoints.

Topic graph (NO feedback loops):
  IN:  /open_nav/goal_add          -> append pending
  IN:  /multi_navi_goals/mission   -> replace pending (RViz)
  OUT: /open_nav/waypoints         nav_msgs/Path   pending preview
  OUT: /open_nav/mission_path      nav_msgs/Path   ONLY after send/confirm
  OUT: /open_nav/tracker_waypoints geometry_msgs/PoseArray  ONLY via patrol start
  OUT: /open_nav/waypoint_markers  visualization
  OUT: /open_nav/waypoint_info     WaypointInfoArray

Does NOT subscribe to /open_nav/mission_path or its own outputs.
"""

import copy
import math
import os
import sys

import rospy
import yaml
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

try:
    from tron_open_space_nav.msg import MissionStatus, WaypointInfo, WaypointInfoArray
    from tron_open_space_nav.srv import DeleteWaypoint, DeleteWaypointResponse
    from tron_open_space_nav.srv import MoveWaypoint, MoveWaypointResponse
except ImportError:
    MissionStatus = None
    WaypointInfo = None
    WaypointInfoArray = None
    DeleteWaypoint = None
    MoveWaypoint = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import enu_to_geodetic, geodetic_to_enu


class GpsMultiWaypointCollector(object):
    PARAM_NS = "/tron_open_space_nav"
    NS_POINTS = "gps_waypoints"
    NS_LABELS = "gps_waypoint_labels"
    NS_LINES = "gps_waypoint_lines"

    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.mission_topic = rospy.get_param("~mission_topic", "/multi_navi_goals/mission")
        self.mission_path_topic = rospy.get_param(
            "~mission_path_topic", "/open_nav/mission_path"
        )
        self.waypoints_topic = rospy.get_param("~waypoints_topic", "/open_nav/waypoints")
        self.tracker_waypoints_topic = rospy.get_param(
            "~tracker_waypoints_topic", "/open_nav/tracker_waypoints"
        )
        self.accumulate_topic = rospy.get_param("~accumulate_topic", "/open_nav/goal_add")
        self.duplicate_click_distance = float(
            rospy.get_param("~duplicate_click_distance", 0.3)
        )
        self.auto_save = rospy.get_param("~auto_save_waypoints", False)
        self.auto_load = rospy.get_param("~auto_load_waypoints", False)

        pkg_config = os.path.join(os.path.dirname(SCRIPT_DIR), "config")
        self.save_path = rospy.get_param(
            "~save_path", os.path.join(pkg_config, "gps_waypoints.yaml")
        )

        self.pending_waypoints = []
        self.confirmed_waypoints = []
        self._confirmed = False
        self._current_index = 0
        self._selected_index = -1
        self._prev_marker_count = 0
        self._mission_state = "IDLE"

        self.waypoints_pub = rospy.Publisher(
            self.waypoints_topic, Path, queue_size=1, latch=True
        )
        self.tracker_waypoints_pub = rospy.Publisher(
            self.tracker_waypoints_topic, PoseArray, queue_size=1, latch=True
        )
        self.mission_path_pub = rospy.Publisher(
            self.mission_path_topic, Path, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "/open_nav/waypoint_markers", MarkerArray, queue_size=1, latch=True
        )
        if WaypointInfoArray is not None:
            self.waypoint_info_pub = rospy.Publisher(
                "/open_nav/waypoint_info", WaypointInfoArray, queue_size=1, latch=True
            )
        else:
            self.waypoint_info_pub = None
            rospy.logerr(
                "[waypoint_collector] WaypointInfoArray msg not available — "
                "/open_nav/waypoint_info disabled; rebuild tron_open_space_nav"
            )

        rospy.Subscriber(self.mission_topic, PoseArray, self.mission_callback, queue_size=1)
        rospy.Subscriber(
            self.accumulate_topic, PoseStamped, self.accumulate_callback, queue_size=10
        )
        rospy.Subscriber("/open_nav/selected_waypoint", Int32, self._selected_cb, queue_size=1)
        if MissionStatus is not None:
            rospy.Subscriber(
                "/open_nav/mission_status", MissionStatus, self._mission_status_cb, queue_size=1
            )

        rospy.Service("~save", Trigger, self.save_service)
        rospy.Service("~clear", Empty, self.clear_service)
        rospy.Service("~publish", Trigger, self.publish_service)
        rospy.Service("~send_mission", Trigger, self.send_mission_service)
        rospy.Service("~confirm_mission", Trigger, self.confirm_mission_service)
        if DeleteWaypoint is not None:
            rospy.Service("~delete_waypoint", DeleteWaypoint, self.delete_waypoint_service)
        if MoveWaypoint is not None:
            rospy.Service("~move_waypoint", MoveWaypoint, self.move_waypoint_service)
        rospy.Service("~load_pending", Trigger, self.load_pending_service)

        if self.auto_load:
            self._load_pending_from_file()
        else:
            self._force_clear_all()

        rospy.loginfo(
            "[waypoint_collector] auto_save=%s auto_load=%s (no mission_path feedback)",
            self.auto_save,
            self.auto_load,
        )

    def _mission_status_cb(self, msg):
        prev_index = self._current_index
        prev_state = self._mission_state
        self._mission_state = msg.state.upper() if msg.state else "IDLE"
        if msg.patrol_mode or msg.state.upper() in (
            "RUNNING", "READY", "PAUSED", "NAVIGATING", "COMPLETED", "STOPPED"
        ):
            self._current_index = msg.current_index
        if self.pending_waypoints and (
            prev_index != self._current_index or prev_state != self._mission_state
        ):
            self._publish_waypoint_info(self.pending_waypoints)

    def _selected_cb(self, msg):
        self._selected_index = int(msg.data)
        self._publish_pending_preview()

    def _editable(self):
        return self._mission_state not in ("RUNNING", "PAUSED")

    def _wait_origin(self):
        return rospy.get_param(self.PARAM_NS + "/origin_ready", False)

    def _pose_to_gps(self, pose):
        if not self._wait_origin():
            return None, None
        lat0 = rospy.get_param(self.PARAM_NS + "/origin_lat")
        lon0 = rospy.get_param(self.PARAM_NS + "/origin_lon")
        alt0 = rospy.get_param(self.PARAM_NS + "/origin_alt")
        lat, lon, _alt = enu_to_geodetic(
            pose.position.x, pose.position.y, pose.position.z, lat0, lon0, alt0
        )
        return float(lat), float(lon)

    def _is_duplicate_click(self, pose):
        if not self.pending_waypoints:
            return False
        last = self.pending_waypoints[-1]
        return (
            math.hypot(pose.position.x - last.position.x, pose.position.y - last.position.y)
            < self.duplicate_click_distance
        )

    def accumulate_callback(self, msg):
        if not self._editable():
            rospy.logwarn("[waypoint_collector] ignore click while mission %s", self._mission_state)
            return
        if self._is_duplicate_click(msg.pose):
            rospy.loginfo("[waypoint_collector] duplicate click ignored")
            return

        self._invalidate_confirmed_if_stale()
        self.pending_waypoints.append(copy.deepcopy(msg.pose))
        n = len(self.pending_waypoints)
        p = self.pending_waypoints[-1]
        rospy.loginfo(
            "[waypoint_collector] added P%d x=%.2f y=%.2f total=%d",
            n, p.position.x, p.position.y, n,
        )
        self._log_pending_list()
        self._publish_pending_preview()

    def mission_callback(self, msg):
        """RViz multi goal: replace pending list, do NOT auto-save or confirm."""
        if not self._editable():
            rospy.logwarn("[waypoint_collector] ignore RViz mission while %s", self._mission_state)
            return
        self._invalidate_confirmed_if_stale()
        self.pending_waypoints = copy.deepcopy(list(msg.poses))
        rospy.loginfo(
            "[waypoint_collector] RViz mission set %d pending waypoints",
            len(self.pending_waypoints),
        )
        self._log_pending_list()
        self._publish_pending_preview()

    def confirm_mission_service(self, _req):
        return self._confirm_mission()

    def send_mission_service(self, _req):
        return self._confirm_mission()

    def _confirm_mission(self):
        if not self.pending_waypoints:
            return TriggerResponse(success=False, message="no pending waypoints")
        self.confirmed_waypoints = copy.deepcopy(self.pending_waypoints)
        self._confirmed = True
        self._publish_path(self.mission_path_pub, self.confirmed_waypoints)
        rospy.loginfo(
            "[waypoint_collector] mission confirmed: %d waypoints -> %s",
            len(self.confirmed_waypoints),
            self.mission_path_topic,
        )
        self._publish_pending_preview()
        return TriggerResponse(
            success=True,
            message="confirmed %d waypoints" % len(self.confirmed_waypoints),
        )

    def publish_tracker_waypoints(self, poses):
        """Called by patrol_mission_manager on start_patrol only."""
        pa = PoseArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = self.map_frame
        pa.poses = list(poses)
        self.tracker_waypoints_pub.publish(pa)
        rospy.loginfo(
            "[waypoint_collector] tracker waypoints published: %d",
            len(poses),
        )

    def delete_waypoint_service(self, req):
        if not self._editable():
            return DeleteWaypointResponse(
                success=False, message="cannot modify mission while running"
            )
        idx = int(req.index)
        if idx >= len(self.pending_waypoints):
            return DeleteWaypointResponse(success=False, message="index out of range")
        removed = self.pending_waypoints.pop(idx)
        rospy.loginfo(
            "[waypoint_collector] deleted P%d (%.2f, %.2f)",
            idx + 1, removed.position.x, removed.position.y,
        )
        self._invalidate_confirmed_if_stale()
        self._publish_pending_preview()
        return DeleteWaypointResponse(success=True, message="deleted index %d" % idx)

    def move_waypoint_service(self, req):
        if not self._editable():
            return MoveWaypointResponse(
                success=False, message="cannot modify mission while running"
            )
        src = int(req.from_index)
        dst = int(req.to_index)
        n = len(self.pending_waypoints)
        if src >= n or dst >= n:
            return MoveWaypointResponse(success=False, message="index out of range")
        item = self.pending_waypoints.pop(src)
        self.pending_waypoints.insert(dst, item)
        self._invalidate_confirmed_if_stale()
        self._publish_pending_preview()
        return MoveWaypointResponse(success=True, message="moved %d -> %d" % (src, dst))

    def clear_service(self, _req):
        self.pending_waypoints = []
        self.confirmed_waypoints = []
        self._confirmed = False
        self._current_index = 0
        self._selected_index = -1
        self._force_clear_all()
        return EmptyResponse()

    def save_service(self, _req):
        try:
            self._save_file()
            return TriggerResponse(success=True, message="saved to " + self.save_path)
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def load_pending_service(self, _req):
        ok = self._load_pending_from_file()
        if ok:
            return TriggerResponse(success=True, message="loaded pending from " + self.save_path)
        return TriggerResponse(success=False, message="load failed")

    def publish_service(self, _req):
        self._publish_pending_preview()
        return TriggerResponse(
            success=True, message="published %d pending" % len(self.pending_waypoints)
        )

    def _save_file(self):
        rows = []
        for pose in self.pending_waypoints:
            lat, lon = self._pose_to_gps(pose)
            rows.append({
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "latitude": lat if lat is not None else 0.0,
                "longitude": lon if lon is not None else 0.0,
            })
        payload = {
            "frame_id": self.map_frame,
            "waypoints": rows,
        }
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        with open(self.save_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, allow_unicode=True)
        rospy.loginfo("[waypoint_collector] saved %d -> %s", len(rows), self.save_path)

    def _load_pending_from_file(self):
        if not os.path.isfile(self.save_path):
            return False
        try:
            with open(self.save_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            rows = data.get("waypoints", []) or []
            self.pending_waypoints = self._poses_from_rows(rows)
            self._confirmed = False
            self._publish_pending_preview()
            return True
        except Exception as exc:
            rospy.logwarn("[waypoint_collector] load failed: %s", exc)
            return False

    def _poses_from_rows(self, rows):
        poses = []
        lat0 = lon0 = alt0 = 0.0
        have_origin = self._wait_origin()
        if have_origin:
            lat0 = rospy.get_param(self.PARAM_NS + "/origin_lat")
            lon0 = rospy.get_param(self.PARAM_NS + "/origin_lon")
            alt0 = rospy.get_param(self.PARAM_NS + "/origin_alt")
        for row in rows:
            lat = row.get("latitude", 0.0)
            lon = row.get("longitude", 0.0)
            if have_origin and lat and lon:
                east, north, up = geodetic_to_enu(lat, lon, alt0, lat0, lon0, alt0)
                x, y, z = east, north, up
            else:
                x = float(row.get("x", 0.0))
                y = float(row.get("y", 0.0))
                z = 0.0
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.w = 1.0
            poses.append(p)
        return poses

    def _log_pending_list(self):
        lines = ["[waypoint_collector] total_pending=%d" % len(self.pending_waypoints)]
        for i, pose in enumerate(self.pending_waypoints):
            lines.append("  P%d x=%.2f y=%.2f" % (i + 1, pose.position.x, pose.position.y))
        rospy.loginfo("\n".join(lines))

    def _force_clear_all(self):
        empty_path = Path()
        empty_path.header.stamp = rospy.Time.now()
        empty_path.header.frame_id = self.map_frame
        self.waypoints_pub.publish(empty_path)
        self.mission_path_pub.publish(empty_path)
        empty_pa = PoseArray()
        empty_pa.header = empty_path.header
        self.tracker_waypoints_pub.publish(empty_pa)
        delete_all = Marker(action=Marker.DELETEALL)
        self.marker_pub.publish(MarkerArray(markers=[delete_all]))
        if self.waypoint_info_pub is not None:
            info = WaypointInfoArray()
            info.header = empty_path.header
            self.waypoint_info_pub.publish(info)
        self._prev_marker_count = 0
        rospy.loginfo("[waypoint_collector] startup clear: empty waypoints/markers/path")

    def _invalidate_confirmed_if_stale(self):
        """Pending edits after send clear latched mission_path."""
        if self._confirmed or self.confirmed_waypoints:
            self.confirmed_waypoints = []
            self._confirmed = False
            empty = Path()
            empty.header.stamp = rospy.Time.now()
            empty.header.frame_id = self.map_frame
            self.mission_path_pub.publish(empty)

    def _publish_pending_preview(self):
        """Publish pending preview; clears mission_path if pending changed after send."""
        poses = list(self.pending_waypoints)
        self._publish_path(self.waypoints_pub, poses)
        self._publish_markers(poses)
        self._publish_waypoint_info(poses)

    def _publish_path(self, publisher, poses):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.map_frame
        for pose in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose = pose
            path.poses.append(ps)
        publisher.publish(path)

    def _waypoint_status(self, index):
        if self._mission_state not in ("RUNNING", "PAUSED", "COMPLETED"):
            return "PENDING"
        if index < self._current_index:
            return "DONE"
        if index == self._current_index:
            return "ACTIVE"
        return "PENDING"

    def _publish_waypoint_info(self, poses):
        if self.waypoint_info_pub is None or WaypointInfo is None:
            rospy.logwarn_throttle(
                10.0,
                "[waypoint_info] skip publish — WaypointInfo msg unavailable (rebuild package)",
            )
            return
        msg = WaypointInfoArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        for i, pose in enumerate(poses):
            lat, lon = self._pose_to_gps(pose)
            wi = WaypointInfo()
            wi.index = i
            wi.name = "P%d" % (i + 1)
            wi.x = pose.position.x
            wi.y = pose.position.y
            wi.latitude = lat if lat is not None else 0.0
            wi.longitude = lon if lon is not None else 0.0
            wi.status = self._waypoint_status(i)
            msg.waypoints.append(wi)
        self.waypoint_info_pub.publish(msg)
        lines = ["[waypoint_info] publishing %d waypoints" % len(msg.waypoints)]
        for wi in msg.waypoints:
            lines.append(
                "  %s x=%.2f y=%.2f lat=%.6f lon=%.6f status=%s"
                % (wi.name, wi.x, wi.y, wi.latitude, wi.longitude, wi.status)
            )
        rospy.loginfo("\n".join(lines))

    def _delete_stale_markers(self, new_count):
        if new_count >= self._prev_marker_count:
            return
        ma = MarkerArray()
        stamp = rospy.Time.now()
        for i in range(new_count, self._prev_marker_count):
            for ns in (self.NS_POINTS, self.NS_LABELS):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = stamp
                m.ns = ns
                m.id = i
                m.action = Marker.DELETE
                ma.markers.append(m)
        if new_count <= 1:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = self.NS_LINES
            m.id = 0
            m.action = Marker.DELETE
            ma.markers.append(m)
        if ma.markers:
            self.marker_pub.publish(ma)

    def _publish_markers(self, poses):
        n = len(poses)
        self._delete_stale_markers(n)
        ma = MarkerArray()
        stamp = rospy.Time.now()
        for i, pose in enumerate(poses):
            scale = 0.9 if i == self._selected_index else 0.6
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = self.NS_POINTS
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.r = 1.0
            m.color.g = 0.8 if i == self._selected_index else 0.4
            m.color.b = 0.0
            m.color.a = 0.95
            ma.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = self.NS_LABELS
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose = copy.deepcopy(pose)
            t.pose.position.z += 0.8
            t.scale.z = 0.5
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text = "P%d" % (i + 1)
            ma.markers.append(t)

        if n > 1:
            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = stamp
            line.ns = self.NS_LINES
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.15
            line.color.r = 0.2
            line.color.g = 0.8
            line.color.b = 1.0
            line.color.a = 0.9
            for pose in poses:
                pt = Point(x=pose.position.x, y=pose.position.y, z=pose.position.z + 0.2)
                line.points.append(pt)
            ma.markers.append(line)

        self.marker_pub.publish(ma)
        self._prev_marker_count = n
        rospy.loginfo(
            "[waypoint_collector] markers: points=%d labels=%d lines=%s",
            n, n, "1" if n > 1 else "0",
        )


def main():
    rospy.init_node("gps_multi_waypoint_collector")
    GpsMultiWaypointCollector()
    rospy.spin()


if __name__ == "__main__":
    main()
