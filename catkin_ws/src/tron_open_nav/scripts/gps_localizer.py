#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def geodetic_to_enu(lat_deg, lon_deg, alt_m, lat0_deg, lon0_deg, alt0_m):
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
    dx = x - x0
    dy = y - y0
    dz = z - z0

    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return east, north, up


class GpsLocalizer(object):
    def __init__(self):
        self.origin_mode = rospy.get_param("~origin_mode", rospy.get_param("/origin_mode", "auto"))
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.pose_topic = rospy.get_param("~pose_topic", "/gps/pose")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.gps_frame = rospy.get_param("~gps_frame", "gps_link")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.origin_ready = False

        if self.origin_mode == "manual":
            self._load_manual_origin()

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.origin_lat_pub = rospy.Publisher("~origin_lat", Float64, queue_size=1, latch=True)
        self.origin_lon_pub = rospy.Publisher("~origin_lon", Float64, queue_size=1, latch=True)
        self.origin_alt_pub = rospy.Publisher("~origin_alt", Float64, queue_size=1, latch=True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.sub = rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_callback, queue_size=10)

        rospy.loginfo(
            "gps_localizer started: origin_mode=%s fix_topic=%s pose_topic=%s",
            self.origin_mode,
            self.fix_topic,
            self.pose_topic,
        )

    def _load_manual_origin(self):
        self.origin_lat = rospy.get_param("/origin_lat", rospy.get_param("~origin_lat", 0.0))
        self.origin_lon = rospy.get_param("/origin_lon", rospy.get_param("~origin_lon", 0.0))
        self.origin_alt = rospy.get_param("/origin_alt", rospy.get_param("~origin_alt", 0.0))
        if self.origin_lat == 0.0 and self.origin_lon == 0.0:
            rospy.logwarn("manual origin_mode but origin_lat/origin_lon are 0; waiting for valid params")
            return
        self._set_origin(self.origin_lat, self.origin_lon, self.origin_alt, source="manual config")

    def _set_origin(self, lat, lon, alt, source="GPS"):
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_alt = alt
        self.origin_ready = True

        rospy.set_param("/tron_open_nav/origin_lat", lat)
        rospy.set_param("/tron_open_nav/origin_lon", lon)
        rospy.set_param("/tron_open_nav/origin_alt", alt)
        rospy.set_param("/tron_open_nav/origin_ready", True)

        self.origin_lat_pub.publish(Float64(data=lat))
        self.origin_lon_pub.publish(Float64(data=lon))
        self.origin_alt_pub.publish(Float64(data=alt))

        rospy.loginfo(
            "GPS origin set from %s: lat=%.8f lon=%.8f alt=%.3f",
            source,
            lat,
            lon,
            alt,
        )

    @staticmethod
    def _is_valid_fix(msg):
        if msg.status.status < 0:
            return False
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return False
        return True

    def fix_callback(self, msg):
        if not self._is_valid_fix(msg):
            return

        if not self.origin_ready:
            if self.origin_mode == "auto":
                self._set_origin(msg.latitude, msg.longitude, msg.altitude, source="first valid GPS")
            else:
                return

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
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.gps_frame
            t.transform.translation.x = east
            t.transform.translation.y = north
            t.transform.translation.z = up
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node("gps_localizer")
    GpsLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
