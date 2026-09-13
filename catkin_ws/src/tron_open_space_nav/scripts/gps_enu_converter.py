#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import NavSatFix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import geodetic_to_enu, is_valid_navsat, yaw_to_quaternion


class GpsEnuConverter(object):
    """Step 2: /gps/fix -> /open_nav/pose + TF open_map -> gps_link."""

    PARAM_NS = "/tron_open_space_nav"

    def __init__(self):
        self.origin_mode = rospy.get_param("~origin_mode", "auto")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.pose_topic = rospy.get_param("~pose_topic", "/open_nav/pose")
        self.map_frame = rospy.get_param("~map_frame", "open_map")
        self.gps_frame = rospy.get_param("~gps_frame", "gps_link")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.origin_lat = rospy.get_param("~origin_lat", 0.0)
        self.origin_lon = rospy.get_param("~origin_lon", 0.0)
        self.origin_alt = rospy.get_param("~origin_alt", 0.0)
        self.origin_ready = self.origin_mode == "manual" and not (
            self.origin_lat == 0.0 and self.origin_lon == 0.0
        )

        if self.origin_ready:
            self._store_origin(self.origin_lat, self.origin_lon, self.origin_alt, "manual")

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.sub = rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_callback, queue_size=20)

        rospy.loginfo(
            "[gps_enu_converter] origin_mode=%s map_frame=%s",
            self.origin_mode,
            self.map_frame,
        )

    def _store_origin(self, lat, lon, alt, source):
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_alt = alt
        self.origin_ready = True
        rospy.set_param(self.PARAM_NS + "/origin_lat", lat)
        rospy.set_param(self.PARAM_NS + "/origin_lon", lon)
        rospy.set_param(self.PARAM_NS + "/origin_alt", alt)
        rospy.set_param(self.PARAM_NS + "/origin_ready", True)
        rospy.loginfo(
            "[gps_enu_converter] origin set (%s): lat=%.8f lon=%.8f alt=%.3f",
            source,
            lat,
            lon,
            alt,
        )

    def fix_callback(self, msg):
        if not is_valid_navsat(msg):
            return

        if not self.origin_ready:
            if self.origin_mode != "auto":
                return
            self._store_origin(msg.latitude, msg.longitude, msg.altitude, "first GPS")

        east, north, up = geodetic_to_enu(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            self.origin_lat,
            self.origin_lon,
            self.origin_alt,
        )

        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = up
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.gps_frame
            t.transform.translation.x = east
            t.transform.translation.y = north
            t.transform.translation.z = up
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node("gps_enu_converter")
    GpsEnuConverter()
    rospy.spin()


if __name__ == "__main__":
    main()
