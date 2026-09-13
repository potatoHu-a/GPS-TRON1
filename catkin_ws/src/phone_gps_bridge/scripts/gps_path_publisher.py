#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class GpsPathPublisher(object):
    def __init__(self):
        self.path = Path()
        self.path.header.frame_id = "gps_local"
        self.path_pub = rospy.Publisher("/gps/path", Path, queue_size=10)
        self.marker_pub = rospy.Publisher("/gps/marker", Marker, queue_size=10)
        rospy.Subscriber("/gps/local_point", PointStamped, self.callback)

    def callback(self, msg):
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(
            _point_to_pose_stamped(msg)
        )
        self.path_pub.publish(self.path)

        marker = Marker()
        marker.header = msg.header
        marker.ns = "gps_current"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def _point_to_pose_stamped(point_stamped):
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header = point_stamped.header
    pose.pose.position = point_stamped.point
    pose.pose.orientation.w = 1.0
    return pose


def main():
    rospy.init_node("gps_path_publisher")
    GpsPathPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
