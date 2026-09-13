#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Single-goal collector for outdoor GPS navigation.

Subscribes:
  /move_base_simple/goal   (RViz 2D Nav Goal, one point at a time)

Optionally saves lat/lon to gps_waypoints.yaml and publishes visualization markers.
Waypoint tracking in single_goal mode is handled directly by waypoint_tracker.
"""

import os
import sys

import rospy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import enu_to_geodetic, geodetic_to_enu


class GpsWaypointCollector(object):
    PARAM_NS = "/tron_open_space_nav"

    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        pkg_config = rospy.get_param("~config_dir", "")
        if not pkg_config:
            pkg_config = os.path.join(os.path.dirname(SCRIPT_DIR), "config")
        self.save_path = rospy.get_param(
            "~save_path", os.path.join(pkg_config, "gps_waypoints.yaml")
        )
        self.auto_save = rospy.get_param("~auto_save", False)

        self.last_waypoint = None

        self.marker_pub = rospy.Publisher(
            "/open_nav/single_goal_marker", MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=10)
        rospy.Service("~save_last", Trigger, self.save_last_service)

        rospy.loginfo(
            "[gps_waypoint_collector] single goal topic=%s auto_save=%s",
            self.goal_topic,
            self.auto_save,
        )

    def _wait_origin(self):
        return rospy.get_param(self.PARAM_NS + "/origin_ready", False)

    def goal_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id not in ("", self.map_frame, "map"):
            rospy.logwarn_throttle(
                5.0,
                "[gps_waypoint_collector] goal frame=%s expected %s",
                msg.header.frame_id,
                self.map_frame,
            )

        if not self._wait_origin():
            rospy.logwarn("[gps_waypoint_collector] GPS origin not ready")
            return

        lat0 = rospy.get_param(self.PARAM_NS + "/origin_lat")
        lon0 = rospy.get_param(self.PARAM_NS + "/origin_lon")
        alt0 = rospy.get_param(self.PARAM_NS + "/origin_alt")

        lat, lon, _alt = enu_to_geodetic(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            lat0,
            lon0,
            alt0,
        )
        self.last_waypoint = {"lat": float(lat), "lon": float(lon)}

        rospy.loginfo(
            "[gps_waypoint_collector] single goal map=(%.2f,%.2f) gps=(%.8f,%.8f)",
            msg.pose.position.x,
            msg.pose.position.y,
            lat,
            lon,
        )

        if self.auto_save:
            self._save_file([self.last_waypoint])

        self._publish_marker(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _save_file(self, waypoints):
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        payload = {"waypoints": waypoints}
        with open(self.save_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, allow_unicode=True)

    def save_last_service(self, _req):
        if self.last_waypoint is None:
            return TriggerResponse(success=False, message="no goal received yet")
        try:
            self._save_file([self.last_waypoint])
            return TriggerResponse(success=True, message="saved to " + self.save_path)
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def _publish_marker(self, x, y, z):
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "single_goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.8
        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.95
        ma.markers.append(m)
        self.marker_pub.publish(ma)


def main():
    rospy.init_node("gps_waypoint_collector")
    GpsWaypointCollector()
    rospy.spin()


if __name__ == "__main__":
    main()
