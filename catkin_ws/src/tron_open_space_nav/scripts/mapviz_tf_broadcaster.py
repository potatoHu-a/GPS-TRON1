#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fallback Mapviz origin publisher (use swri initialize_origin.py by default).

Publishes latched /local_xy_origin and the map->map__identity TF that
swri_transform_util expects. Does NOT publish wgs84->map.

Pose convention (swri_transform_util / Mapviz):
  header.frame_id = map
  position.x = longitude
  position.y = latitude
  position.z = altitude
  orientation = identity (heading 0)
"""

import os
import sys

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import is_valid_navsat


class MapvizOriginPublisher(object):
    PARAM_NS = "/tron_open_space_nav"

    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.origin_topic = rospy.get_param("~origin_topic", "/local_xy_origin")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.allow_gps_fallback = rospy.get_param("~allow_gps_fallback", True)

        self._published = False
        self._frame_identity = self.map_frame + "__identity"
        self._tf_broadcaster = tf.TransformBroadcaster()
        self.origin_pub = rospy.Publisher(
            self.origin_topic, PoseStamped, queue_size=1, latch=True
        )

        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_callback, queue_size=5)
        rospy.Timer(rospy.Duration(0.5), self._try_existing_origin, oneshot=False)
        rospy.Timer(rospy.Duration(1.0), self._broadcast_identity_tf, oneshot=False)

        rospy.loginfo(
            "[mapviz_origin] waiting for origin on %s (frame=%s)",
            self.origin_topic,
            self.map_frame,
        )

    def _try_existing_origin(self, _event):
        if self._published:
            return
        if not rospy.get_param(self.PARAM_NS + "/origin_ready", False):
            return
        lat = rospy.get_param(self.PARAM_NS + "/origin_lat")
        lon = rospy.get_param(self.PARAM_NS + "/origin_lon")
        alt = rospy.get_param(self.PARAM_NS + "/origin_alt")
        self._publish_origin(lat, lon, alt, "existing_param")

    def fix_callback(self, msg):
        if self._published or not self.allow_gps_fallback:
            return
        if rospy.get_param(self.PARAM_NS + "/origin_ready", False):
            return
        if not is_valid_navsat(msg):
            return
        self._publish_origin(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            "first_gps_fallback",
        )

    def _broadcast_identity_tf(self, _event):
        self._tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            self._frame_identity,
            self.map_frame,
        )

    def _publish_origin(self, lat, lon, alt, source):
        if self._published:
            return
        self._published = True

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(lon)
        msg.pose.position.y = float(lat)
        msg.pose.position.z = float(alt)
        msg.pose.orientation.w = 1.0

        self.origin_pub.publish(msg)
        rospy.loginfo(
            "[mapviz_origin] local_xy_origin published: "
            "frame=%s lat=%.8f lon=%.8f alt=%.3f (source=%s)",
            self.map_frame,
            lat,
            lon,
            alt,
            source,
        )


def main():
    rospy.init_node("mapviz_origin_publisher")
    MapvizOriginPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
