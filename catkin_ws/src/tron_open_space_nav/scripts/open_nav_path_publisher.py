#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Accumulate /open_nav/odom poses into nav_msgs/Path in the map frame.

For Mapviz ground station visualization (replaces /gps/path in gps_local).
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class OpenNavPathPublisher(object):
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/open_nav/odom")
        self.path_topic = rospy.get_param("~path_topic", "/open_nav/path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.max_points = int(rospy.get_param("~max_points", 5000))

        self._path = Path()
        self._path.header.frame_id = self.map_frame

        self._path_pub = rospy.Publisher(
            self.path_topic, Path, queue_size=10, latch=True
        )
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)

        rospy.loginfo(
            "[open_nav_path] %s -> %s frame=%s max_points=%d",
            self.odom_topic,
            self.path_topic,
            self.map_frame,
            self.max_points,
        )

    def odom_callback(self, msg):
        if msg.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                10.0,
                "[open_nav_path] expected frame_id=%s, got %s",
                self.map_frame,
                msg.header.frame_id,
            )

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose = msg.pose.pose

        self._path.header.stamp = msg.header.stamp
        self._path.poses.append(pose)

        if len(self._path.poses) > self.max_points:
            overflow = len(self._path.poses) - self.max_points
            del self._path.poses[:overflow]

        self._path_pub.publish(self._path)


def main():
    rospy.init_node("open_nav_path_publisher")
    OpenNavPathPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
