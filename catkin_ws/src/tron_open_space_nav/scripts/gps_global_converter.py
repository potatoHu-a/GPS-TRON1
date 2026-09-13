#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WGS84 GPS -> global ENU pose in true-north aligned ``map`` frame.

TF chain (global layer):
  earth -> map -> (open_odom -> open_base via gps_fastlio_fusion)

Publishes /global_pose (filtered ENU), /open_nav/gps_raw_pose, /open_nav/gps_filtered_pose.
"""

import os
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import NavSatFix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import (
    geodetic_to_enu,
    geodetic_to_utm,
    haversine_distance_m,
    is_valid_navsat,
    median_value,
    navsat_horizontal_sigma,
    reject_origin_outlier,
    yaw_to_quaternion,
)


class GpsGlobalConverter(object):
    PARAM_NS = "/tron_open_space_nav"

    def __init__(self):
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.pose_topic = rospy.get_param("~pose_topic", "/global_pose")
        self.raw_pose_topic = rospy.get_param("~raw_pose_topic", "/open_nav/gps_raw_pose")
        self.filtered_pose_topic = rospy.get_param(
            "~filtered_pose_topic", "/open_nav/gps_filtered_pose"
        )
        self.earth_frame = rospy.get_param("~earth_frame", "earth")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.origin_mode = rospy.get_param("~origin_mode", "auto")
        self.origin_lat = rospy.get_param("~origin_lat", 0.0)
        self.origin_lon = rospy.get_param("~origin_lon", 0.0)
        self.origin_alt = rospy.get_param("~origin_alt", 0.0)
        self.origin_samples = int(rospy.get_param("~origin_samples", 20))
        self.origin_outlier_m = float(rospy.get_param("~origin_outlier_m", 15.0))
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.use_utm = rospy.get_param("~use_utm", False)

        self.gps_max_sigma = float(rospy.get_param("~gps_max_sigma", 8.0))
        self.gps_max_speed = float(rospy.get_param("~gps_max_speed", 3.0))
        self.gps_ema_alpha = float(rospy.get_param("~gps_ema_alpha", 0.3))
        self.enable_gps_filter = rospy.get_param("~enable_gps_filter", True)

        self.origin_ready = self.origin_mode == "manual" and not (
            self.origin_lat == 0.0 and self.origin_lon == 0.0
        )
        if self.origin_ready:
            self._store_origin(self.origin_lat, self.origin_lon, self.origin_alt, "manual")

        self._origin_buffer = []
        self._last_fix = None
        self._last_fix_time = None
        self._rejected_count = 0
        self._filtered_east = None
        self._filtered_north = None
        self._filtered_up = None

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.raw_pose_pub = rospy.Publisher(self.raw_pose_topic, PoseStamped, queue_size=10)
        self.filtered_pose_pub = rospy.Publisher(
            self.filtered_pose_topic, PoseStamped, queue_size=10
        )
        self.static_broadcaster = (
            tf2_ros.StaticTransformBroadcaster() if self.publish_tf else None
        )
        self._earth_map_sent = False

        try:
            import pyproj  # noqa: F401
            self.has_pyproj = True
        except ImportError:
            self.has_pyproj = False
            if self.use_utm:
                rospy.logwarn(
                    "[gps_global_converter] pyproj not installed; falling back to ENU. "
                    "Install: pip3 install pyproj"
                )
                self.use_utm = False

        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_callback, queue_size=20)
        rospy.loginfo(
            "[gps_global_converter] fix=%s pose=%s origin_samples=%d ema=%.2f",
            self.fix_topic,
            self.pose_topic,
            self.origin_samples,
            self.gps_ema_alpha,
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
        rospy.set_param(self.PARAM_NS + "/global_origin_lat", lat)
        rospy.set_param(self.PARAM_NS + "/global_origin_lon", lon)
        rospy.set_param(self.PARAM_NS + "/global_origin_alt", alt)
        rospy.set_param(self.PARAM_NS + "/global_origin_ready", True)
        rospy.set_param(self.PARAM_NS + "/map_frame", self.map_frame)

        rospy.loginfo(
            "[gps_global_converter] origin set (%s): lat=%.8f lon=%.8f alt=%.3f",
            source,
            lat,
            lon,
            alt,
        )
        self._publish_earth_map_tf()

    def _try_lock_origin_from_buffer(self):
        if len(self._origin_buffer) < self.origin_samples:
            return False
        lats = [s[0] for s in self._origin_buffer]
        lons = [s[1] for s in self._origin_buffer]
        alts = [s[2] for s in self._origin_buffer]
        lat = median_value(lats)
        lon = median_value(lons)
        alt = median_value(alts)
        self._store_origin(lat, lon, alt, "median_%d_samples" % len(self._origin_buffer))
        self._origin_buffer = []
        return True

    def _publish_earth_map_tf(self):
        if not self.publish_tf or self.static_broadcaster is None or self._earth_map_sent:
            return

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.earth_frame
        t.child_frame_id = self.map_frame
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)
        self._earth_map_sent = True
        rospy.loginfo(
            "[gps_global_converter] static TF %s -> %s (ENU true-north at origin)",
            self.earth_frame,
            self.map_frame,
        )

    def _position_from_fix(self, msg):
        if self.use_utm and self.has_pyproj:
            utm = geodetic_to_utm(msg.latitude, msg.longitude, msg.altitude)
            if utm is not None:
                east, north, up, zone, hemi = utm
                if not rospy.has_param(self.PARAM_NS + "/utm_zone"):
                    rospy.set_param(self.PARAM_NS + "/utm_zone", zone)
                    rospy.set_param(self.PARAM_NS + "/utm_hemisphere", hemi)
                if self.origin_ready:
                    e0, n0, u0, _, _ = geodetic_to_utm(
                        self.origin_lat, self.origin_lon, self.origin_alt
                    )
                    return east - e0, north - n0, up - u0
                return east, north, up

        if not self.origin_ready:
            return None
        east, north, up = geodetic_to_enu(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            self.origin_lat,
            self.origin_lon,
            self.origin_alt,
        )
        return east, north, up

    def _reject_noisy_fix(self, msg):
        sigma = navsat_horizontal_sigma(msg)
        if sigma is not None and sigma > self.gps_max_sigma:
            self._rejected_count += 1
            rospy.logwarn_throttle(
                5.0,
                "[gps_filter] rejected noisy fix sigma=%.2f > %.2f",
                sigma,
                self.gps_max_sigma,
            )
            return True

        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        if self._last_fix is not None and self._last_fix_time is not None:
            dt = (stamp - self._last_fix_time).to_sec()
            if dt > 0.05:
                dist = haversine_distance_m(
                    self._last_fix.latitude,
                    self._last_fix.longitude,
                    msg.latitude,
                    msg.longitude,
                )
                speed = dist / dt
                if speed > self.gps_max_speed:
                    self._rejected_count += 1
                    rospy.logwarn_throttle(
                        5.0,
                        "[gps_filter] rejected jump speed=%.2f m/s > %.2f",
                        speed,
                        self.gps_max_speed,
                    )
                    return True

        self._last_fix = msg
        self._last_fix_time = stamp
        return False

    def _apply_ema(self, east, north, up):
        if not self.enable_gps_filter:
            return east, north, up
        alpha = self.gps_ema_alpha
        if self._filtered_east is None:
            self._filtered_east = east
            self._filtered_north = north
            self._filtered_up = up
        else:
            self._filtered_east = alpha * east + (1.0 - alpha) * self._filtered_east
            self._filtered_north = alpha * north + (1.0 - alpha) * self._filtered_north
            self._filtered_up = alpha * up + (1.0 - alpha) * self._filtered_up
        return self._filtered_east, self._filtered_north, self._filtered_up

    def _make_pose(self, stamp, east, north, up):
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
        return pose

    def fix_callback(self, msg):
        if not is_valid_navsat(msg):
            return

        if rospy.get_param(self.PARAM_NS + "/origin_ready", False) and not self.origin_ready:
            self.origin_ready = True
            self.origin_lat = rospy.get_param(self.PARAM_NS + "/origin_lat")
            self.origin_lon = rospy.get_param(self.PARAM_NS + "/origin_lon")
            self.origin_alt = rospy.get_param(self.PARAM_NS + "/origin_alt")
            self._publish_earth_map_tf()

        if not self.origin_ready:
            if self.origin_mode != "auto":
                return
            if reject_origin_outlier(
                msg.latitude, msg.longitude, self._origin_buffer, self.origin_outlier_m
            ):
                return
            self._origin_buffer.append((msg.latitude, msg.longitude, msg.altitude))
            rospy.loginfo_throttle(
                2.0,
                "[gps_global_converter] collecting origin %d/%d",
                len(self._origin_buffer),
                self.origin_samples,
            )
            if not self._try_lock_origin_from_buffer():
                return

        if self._reject_noisy_fix(msg):
            return

        pos = self._position_from_fix(msg)
        if pos is None:
            return
        east, north, up = pos

        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        raw_pose = self._make_pose(stamp, east, north, up)
        self.raw_pose_pub.publish(raw_pose)

        fe, fn, fu = self._apply_ema(east, north, up)
        filtered_pose = self._make_pose(stamp, fe, fn, fu)
        self.filtered_pose_pub.publish(filtered_pose)
        self.pose_pub.publish(filtered_pose)

        rospy.set_param(self.PARAM_NS + "/gps_rejected_count", self._rejected_count)


def main():
    rospy.init_node("gps_global_converter")
    GpsGlobalConverter()
    rospy.spin()


if __name__ == "__main__":
    main()
