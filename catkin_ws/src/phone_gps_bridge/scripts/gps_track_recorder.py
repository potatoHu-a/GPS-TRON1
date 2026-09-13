#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS = 6378137.0


def latlon_to_enu(lat, lon, lat0, lon0):
    """Local ENU approximation for short distances."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    cos_lat0 = math.cos(math.radians(lat0))
    east = EARTH_RADIUS * dlon * cos_lat0
    north = EARTH_RADIUS * dlat
    return east, north


class GpsTrackRecorder(object):
    def __init__(self):
        self.origin_lat = None
        self.origin_lon = None
        self.pub = rospy.Publisher("/gps/local_point", PointStamped, queue_size=10)
        rospy.Subscriber("/gps/fix", NavSatFix, self.callback)

    def callback(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            rospy.loginfo(
                "Origin set: lat=%.6f lon=%.6f",
                self.origin_lat,
                self.origin_lon,
            )

        east, north = latlon_to_enu(
            msg.latitude, msg.longitude,
            self.origin_lat, self.origin_lon,
        )
        dist = math.hypot(east, north)

        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "gps_local"
        point.point.x = east
        point.point.y = north
        point.point.z = msg.altitude
        self.pub.publish(point)

        rospy.loginfo(
            "lat=%.6f lon=%.6f dist=%.2fm East=%.2f North=%.2f",
            msg.latitude,
            msg.longitude,
            dist,
            east,
            north,
        )


def main():
    rospy.init_node("gps_track_recorder")
    GpsTrackRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
