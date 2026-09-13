#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, TriggerResponse


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


class GpsGoalPublisher(object):
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.goal_input_topic = rospy.get_param("~goal_input_topic", "/gps/goal")
        self.default_alt = rospy.get_param("~default_alt", 0.0)
        self.goal_yaw = rospy.get_param("~goal_yaw", 0.0)

        self.goal_lat = rospy.get_param("~goal_latitude", 0.0)
        self.goal_lon = rospy.get_param("~goal_longitude", 0.0)
        self.publish_on_start = rospy.get_param("~publish_on_start", False)

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.goal_sub = rospy.Subscriber(self.goal_input_topic, NavSatFix, self.goal_input_callback, queue_size=10)
        self.publish_srv = rospy.Service("~publish_goal", Trigger, self.publish_goal_service)

        rospy.loginfo(
            "gps_goal_publisher started: goal_topic=%s goal_input_topic=%s",
            self.goal_topic,
            self.goal_input_topic,
        )

        if self.publish_on_start and self.goal_lat != 0.0 and self.goal_lon != 0.0:
            rospy.Timer(rospy.Duration(1.0), self._startup_publish_once, oneshot=True)

    def _startup_publish_once(self, _event):
        try:
            self.publish_goal(self.goal_lat, self.goal_lon, self.default_alt)
        except rospy.ROSException as exc:
            rospy.logwarn("startup goal publish skipped: %s", exc)

    def _wait_for_origin(self, timeout=30.0):
        if rospy.get_param("/tron_open_nav/origin_ready", False):
            self.origin_lat = rospy.get_param("/tron_open_nav/origin_lat")
            self.origin_lon = rospy.get_param("/tron_open_nav/origin_lon")
            self.origin_alt = rospy.get_param("/tron_open_nav/origin_alt")
            return True

        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.get_param("/tron_open_nav/origin_ready", False):
                self.origin_lat = rospy.get_param("/tron_open_nav/origin_lat")
                self.origin_lon = rospy.get_param("/tron_open_nav/origin_lon")
                self.origin_alt = rospy.get_param("/tron_open_nav/origin_alt")
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()
        return False

    def publish_goal(self, lat, lon, alt=None):
        if alt is None:
            alt = self.default_alt

        if not self._wait_for_origin():
            raise rospy.ROSException("GPS origin not ready; start gps_localizer first")

        east, north, up = geodetic_to_enu(
            lat,
            lon,
            alt,
            self.origin_lat,
            self.origin_lon,
            self.origin_alt,
        )

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = east
        goal.pose.position.y = north
        goal.pose.position.z = up

        half_yaw = self.goal_yaw * 0.5
        goal.pose.orientation.z = math.sin(half_yaw)
        goal.pose.orientation.w = math.cos(half_yaw)

        self.goal_pub.publish(goal)
        rospy.loginfo(
            "Published move_base goal: gps=(%.8f, %.8f) map=(%.3f, %.3f, %.3f) yaw=%.2f",
            lat,
            lon,
            east,
            north,
            up,
            self.goal_yaw,
        )
        return goal

    def goal_input_callback(self, msg):
        if msg.status.status < 0:
            rospy.logwarn("Ignored invalid GPS goal input")
            return
        try:
            alt = msg.altitude if msg.altitude != 0.0 else self.default_alt
            self.publish_goal(msg.latitude, msg.longitude, alt)
        except rospy.ROSException as exc:
            rospy.logwarn("Failed to publish GPS goal: %s", exc)

    def publish_goal_service(self, _req):
        try:
            if self.goal_lat == 0.0 and self.goal_lon == 0.0:
                return TriggerResponse(success=False, message="goal_latitude/goal_longitude not set")
            self.publish_goal(self.goal_lat, self.goal_lon, self.default_alt)
            return TriggerResponse(success=True, message="move_base goal published")
        except rospy.ROSException as exc:
            return TriggerResponse(success=False, message=str(exc))


def main():
    rospy.init_node("gps_goal_publisher")
    GpsGoalPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
