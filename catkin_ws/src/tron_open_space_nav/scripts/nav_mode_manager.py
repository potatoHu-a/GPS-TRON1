#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Step 6 (reserved): stair / terrain mode manager interface."""

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class NavModeManager(object):
    STAIR_MIN_CM = 8.0
    STAIR_MAX_CM = 20.0

    def __init__(self):
        self.cloud_topic = rospy.get_param("~pointcloud_topic", "/livox/lidar")
        self.mode_topic = rospy.get_param("~mode_topic", "/open_nav/nav_mode")
        self.current_mode = "flat"

        self.mode_pub = rospy.Publisher(self.mode_topic, String, queue_size=1, latch=True)
        self.mode_pub.publish(String(data=self.current_mode))
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=5)

        rospy.loginfo("[nav_mode_manager] reserved interface ready mode=%s", self.current_mode)

    def cloud_callback(self, _msg):
        # Reserved: future height-change detection (8-20cm => stair_mode)
        pass


def main():
    rospy.init_node("nav_mode_manager")
    NavModeManager()
    rospy.spin()


if __name__ == "__main__":
    main()
