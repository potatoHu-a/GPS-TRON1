#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wait for Mapviz prerequisites, then exec mapviz.

Avoids first-launch grey screen when Tile Map is created before
/local_xy_origin and Wgs84Transformer are ready.
"""

import os
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import is_valid_navsat


def is_valid_local_xy_origin(msg, map_frame):
    if msg.header.frame_id != map_frame:
        return False
    lon = msg.pose.position.x
    lat = msg.pose.position.y
    if lat == 0.0 and lon == 0.0:
        return False
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    return True


class MapvizReadyLauncher(object):
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.target_frame = rospy.get_param("~target_frame", "open_base")
        self.origin_topic = rospy.get_param("~origin_topic", "/local_xy_origin")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.config_path = rospy.get_param("~config_path")
        self.mapviz_bin = rospy.get_param(
            "~mapviz_bin", "/opt/ros/noetic/lib/mapviz/mapviz"
        )
        self.post_ready_delay = float(rospy.get_param("~post_ready_delay", 1.0))

        self.local_xy_origin_ok = False
        self.gps_fix_ok = False
        self._launched = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        rospy.Subscriber(
            self.origin_topic, PoseStamped, self._origin_callback, queue_size=1
        )
        rospy.Subscriber(self.fix_topic, NavSatFix, self._gps_callback, queue_size=5)
        rospy.Timer(rospy.Duration(1.0), self._status_loop, oneshot=False)

        rospy.loginfo(
            "[mapviz_launcher] waiting for origin, gps, and TF %s->%s",
            self.map_frame,
            self.target_frame,
        )

    def _origin_callback(self, msg):
        if is_valid_local_xy_origin(msg, self.map_frame):
            self.local_xy_origin_ok = True

    def _gps_callback(self, msg):
        if is_valid_navsat(msg):
            self.gps_fix_ok = True

    def _poll_origin(self):
        if self.local_xy_origin_ok:
            return
        try:
            msg = rospy.wait_for_message(
                self.origin_topic, PoseStamped, timeout=0.05
            )
            self._origin_callback(msg)
        except rospy.ROSException:
            pass

    def _map_to_target_ok(self):
        try:
            return self._tf_buffer.can_transform(
                self.map_frame,
                self.target_frame,
                rospy.Time(0),
                rospy.Duration(0.1),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return False

    def _status_loop(self, _event):
        if self._launched:
            return

        self._poll_origin()
        map_to_open_base = self._map_to_target_ok()

        rospy.loginfo(
            "Waiting:\n"
            "local_xy_origin=%s\n"
            "gps_fix=%s\n"
            "map_to_open_base=%s",
            str(self.local_xy_origin_ok).lower(),
            str(self.gps_fix_ok).lower(),
            str(map_to_open_base).lower(),
        )

        if not (
            self.local_xy_origin_ok and self.gps_fix_ok and map_to_open_base
        ):
            return

        self._launched = True
        rospy.loginfo(
            "[mapviz_launcher] Geographic origin ready, launching Mapviz."
        )
        time.sleep(self.post_ready_delay)

        if not os.path.isfile(self.config_path):
            rospy.logfatal(
                "[mapviz_launcher] config not found: %s", self.config_path
            )
            sys.exit(1)
        if not os.path.isfile(self.mapviz_bin):
            rospy.logfatal(
                "[mapviz_launcher] mapviz binary not found: %s", self.mapviz_bin
            )
            sys.exit(1)

        args = [self.mapviz_bin, "-d", self.config_path]
        rospy.loginfo("[mapviz_launcher] exec: %s", " ".join(args))
        os.execv(self.mapviz_bin, args)


def main():
    rospy.init_node("mapviz_launcher")
    MapvizReadyLauncher()
    rospy.spin()


if __name__ == "__main__":
    main()
