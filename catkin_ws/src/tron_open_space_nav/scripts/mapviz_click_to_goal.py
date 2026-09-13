#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert Mapviz point clicks to map-frame goals for gps_multi_waypoint_collector.
"""

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped


class MapvizClickToGoal(object):
    def __init__(self):
        self.click_topic = rospy.get_param("~click_topic", "/mapviz/waypoint_click")
        self.goal_topic = rospy.get_param("~goal_topic", "/open_nav/goal_add")
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.click_topic, PointStamped, self.click_callback, queue_size=10)

        rospy.loginfo(
            "[mapviz_click_to_goal] %s -> %s frame=%s",
            self.click_topic,
            self.goal_topic,
            self.map_frame,
        )

    def click_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                5.0,
                "[mapviz_click_to_goal] expected frame %s, got %s",
                self.map_frame,
                msg.header.frame_id,
            )

        goal = PoseStamped()
        goal.header.stamp = msg.header.stamp or rospy.Time.now()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        rospy.loginfo(
            "[mapviz_click_to_goal] P? map=(%.2f, %.2f)",
            goal.pose.position.x,
            goal.pose.position.y,
        )


def main():
    rospy.init_node("mapviz_click_to_goal")
    MapvizClickToGoal()
    rospy.spin()


if __name__ == "__main__":
    main()
