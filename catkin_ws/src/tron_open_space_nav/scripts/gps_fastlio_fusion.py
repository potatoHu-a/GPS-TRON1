#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fuse low-rate GPS (map absolute) with high-rate FAST-LIO (/Odometry).

Strategy:
  open_nav_position = fastlio_local_in_map + slowly_updated gps_offset (map->open_odom)

FAST-LIO provides short-term continuous displacement; GPS applies low-rate, rate-limited
offset correction. GPS yaw is not modified here.
"""

import math
import os
import sys
from collections import deque

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import (
    geodetic_to_enu,
    haversine_distance_m,
    is_valid_navsat,
    median_value,
    navsat_horizontal_sigma,
    normalize_angle,
    quaternion_to_yaw,
    yaw_to_quaternion,
)

try:
    from tron_open_space_nav.msg import LocalizationStatus
except ImportError:
    LocalizationStatus = None


class GpsFastlioFusion(object):
    PARAM_NS = "/tron_open_space_nav"

    def __init__(self):
        self.gps_topic = rospy.get_param("~gps_fix_topic", "/gps/fix")
        self.gps_pose_topic = rospy.get_param("~gps_pose_topic", "/open_nav/gps_filtered_pose")
        self.use_filtered_pose = rospy.get_param("~use_filtered_gps_pose", True)
        self.odom_in_topic = rospy.get_param("~fastlio_odom_topic", "/Odometry")
        self.odom_out_topic = rospy.get_param("~odom_topic", "/open_nav/odom")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "open_odom")
        self.base_frame = rospy.get_param("~base_frame", "open_base")

        self.gps_min_interval = float(rospy.get_param("~gps_update_min_interval", 0.5))
        self.gps_correction_gain = float(rospy.get_param("~gps_correction_gain", 0.12))
        self.gps_correction_max_step = float(
            rospy.get_param("~gps_correction_max_step", 0.5)
        )
        self.gps_max_sigma = float(rospy.get_param("~gps_max_sigma", 8.0))
        self.gps_max_speed = float(rospy.get_param("~gps_max_speed", 3.0))
        self.stationary_speed = float(rospy.get_param("~stationary_speed", 0.05))
        self.stationary_window = int(rospy.get_param("~stationary_window", 20))
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_status = rospy.get_param("~publish_localization_status", True)

        self.origin_ready = False
        self.last_gps_time = rospy.Time(0)
        self._last_gps_fix = None
        self._last_gps_fix_time = None
        self._gps_rejected = 0
        self._stationary_gps_e = deque(maxlen=self.stationary_window)
        self._stationary_gps_n = deque(maxlen=self.stationary_window)

        # map -> open_odom offset (GPS long-term correction)
        self.map_odom_x = 0.0
        self.map_odom_y = 0.0
        self.map_odom_yaw = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_wz = 0.0
        self.have_odom = False

        self.gps_x = 0.0
        self.gps_y = 0.0
        self.gps_z = 0.0
        self._last_gps_sigma = 0.0

        self.odom_pub = rospy.Publisher(self.odom_out_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        if LocalizationStatus is not None and self.publish_status:
            self.status_pub = rospy.Publisher(
                "/open_nav/fusion_status", LocalizationStatus, queue_size=1, latch=False
            )
        else:
            self.status_pub = None

        if self.use_filtered_pose:
            rospy.Subscriber(
                self.gps_pose_topic, PoseStamped, self.gps_pose_callback, queue_size=10
            )
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_fix_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_in_topic, Odometry, self.odom_callback, queue_size=50)

        rospy.loginfo(
            "[gps_fastlio_fusion] gps_pose=%s gain=%.3f max_step=%.2f odom_in=%s",
            self.gps_pose_topic if self.use_filtered_pose else self.gps_topic,
            self.gps_correction_gain,
            self.gps_correction_max_step,
            self.odom_in_topic,
        )

    def _wait_origin(self):
        if self.origin_ready:
            return True
        if rospy.get_param(self.PARAM_NS + "/origin_ready", False):
            self.origin_ready = True
            return True
        return False

    def _compose_map_pose(self):
        cos_m = math.cos(self.map_odom_yaw)
        sin_m = math.sin(self.map_odom_yaw)
        map_x = self.map_odom_x + cos_m * self.odom_x - sin_m * self.odom_y
        map_y = self.map_odom_y + sin_m * self.odom_x + cos_m * self.odom_y
        map_yaw = normalize_angle(self.map_odom_yaw + self.odom_yaw)
        return map_x, map_y, map_yaw

    def _reject_gps_fix(self, msg):
        sigma = navsat_horizontal_sigma(msg)
        self._last_gps_sigma = sigma if sigma is not None else 0.0
        if sigma is not None and sigma > self.gps_max_sigma:
            self._gps_rejected += 1
            rospy.logwarn_throttle(
                5.0,
                "[gps_filter] fusion rejected noisy fix sigma=%.2f",
                sigma,
            )
            return True

        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        if self._last_gps_fix is not None and self._last_gps_fix_time is not None:
            dt = (stamp - self._last_gps_fix_time).to_sec()
            if dt > 0.05:
                dist = haversine_distance_m(
                    self._last_gps_fix.latitude,
                    self._last_gps_fix.longitude,
                    msg.latitude,
                    msg.longitude,
                )
                speed = dist / dt
                odom_speed = abs(self.odom_vx)
                if speed > self.gps_max_speed and odom_speed < self.gps_max_speed * 0.5:
                    self._gps_rejected += 1
                    rospy.logwarn_throttle(
                        5.0,
                        "[gps_filter] fusion rejected jump speed=%.2f odom=%.2f",
                        speed,
                        odom_speed,
                    )
                    return True

        self._last_gps_fix = msg
        self._last_gps_fix_time = stamp
        return False

    def _apply_gps_map_position(self, east, north, up, stamp):
        if not self.have_odom:
            return

        now = stamp
        if (now - self.last_gps_time).to_sec() < self.gps_min_interval:
            return
        self.last_gps_time = now

        self.gps_x = east
        self.gps_y = north
        self.gps_z = up

        robot_x, robot_y, _robot_yaw = self._compose_map_pose()
        err_x = east - robot_x
        err_y = north - robot_y

        speed = math.hypot(self.odom_vx, 0.0)
        if speed < self.stationary_speed:
            self._stationary_gps_e.append(east)
            self._stationary_gps_n.append(north)
            if len(self._stationary_gps_e) >= max(5, self.stationary_window // 2):
                med_e = median_value(list(self._stationary_gps_e))
                med_n = median_value(list(self._stationary_gps_n))
                err_x = med_e - robot_x
                err_y = med_n - robot_y

        step_x = max(
            -self.gps_correction_max_step,
            min(self.gps_correction_max_step, self.gps_correction_gain * err_x),
        )
        step_y = max(
            -self.gps_correction_max_step,
            min(self.gps_correction_max_step, self.gps_correction_gain * err_y),
        )
        self.map_odom_x += step_x
        self.map_odom_y += step_y

        self._publish(now)

    def gps_pose_callback(self, msg):
        if not self._wait_origin() or not self.have_odom:
            return
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            return
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        self._apply_gps_map_position(
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, stamp
        )

    def gps_fix_callback(self, msg):
        if self.use_filtered_pose:
            return
        if not is_valid_navsat(msg) or not self._wait_origin() or not self.have_odom:
            return
        if self._reject_gps_fix(msg):
            return

        lat0 = rospy.get_param(self.PARAM_NS + "/origin_lat")
        lon0 = rospy.get_param(self.PARAM_NS + "/origin_lon")
        alt0 = rospy.get_param(self.PARAM_NS + "/origin_alt")
        east, north, up = geodetic_to_enu(
            msg.latitude, msg.longitude, msg.altitude, lat0, lon0, alt0
        )
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        self._apply_gps_map_position(east, north, up, stamp)

    def odom_callback(self, msg):
        if not self._wait_origin():
            return

        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        self.have_odom = True

        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        self._publish(stamp)

    def _publish_status(self, stamp):
        if self.status_pub is None:
            return
        msg = LocalizationStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.gps_sigma = float(self._last_gps_sigma)
        msg.gps_fix_valid = True
        msg.gps_rejected_count = self._gps_rejected
        msg.gps_correction_offset_x = float(self.map_odom_x)
        msg.gps_correction_offset_y = float(self.map_odom_y)
        msg.note = "fusion_offset"
        self.status_pub.publish(msg)

    def _publish(self, stamp):
        map_x, map_y, map_yaw = self._compose_map_pose()
        qx, qy, qz, qw = yaw_to_quaternion(map_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = map_x
        odom.pose.pose.position.y = map_y
        odom.pose.pose.position.z = self.gps_z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.odom_vx
        odom.twist.twist.angular.z = self.odom_wz
        self.odom_pub.publish(odom)
        self._publish_status(stamp)

        if self.tf_broadcaster is None:
            return

        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = stamp
        t_map_odom.header.frame_id = self.map_frame
        t_map_odom.child_frame_id = self.odom_frame
        t_map_odom.transform.translation.x = self.map_odom_x
        t_map_odom.transform.translation.y = self.map_odom_y
        t_map_odom.transform.translation.z = 0.0
        qmx, qmy, qmz, qmw = yaw_to_quaternion(self.map_odom_yaw)
        t_map_odom.transform.rotation.x = qmx
        t_map_odom.transform.rotation.y = qmy
        t_map_odom.transform.rotation.z = qmz
        t_map_odom.transform.rotation.w = qmw
        self.tf_broadcaster.sendTransform(t_map_odom)

        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = stamp
        t_odom_base.header.frame_id = self.odom_frame
        t_odom_base.child_frame_id = self.base_frame
        t_odom_base.transform.translation.x = self.odom_x
        t_odom_base.transform.translation.y = self.odom_y
        t_odom_base.transform.translation.z = 0.0
        qox, qoy, qoz, qow = yaw_to_quaternion(self.odom_yaw)
        t_odom_base.transform.rotation.x = qox
        t_odom_base.transform.rotation.y = qoy
        t_odom_base.transform.rotation.z = qoz
        t_odom_base.transform.rotation.w = qow
        self.tf_broadcaster.sendTransform(t_odom_base)


def main():
    rospy.init_node("gps_fastlio_fusion")
    GpsFastlioFusion()
    rospy.spin()


if __name__ == "__main__":
    main()
