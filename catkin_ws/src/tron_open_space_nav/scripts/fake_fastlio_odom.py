#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulate wheel odometry on a closed circular path for Docker closed-loop testing.

Publishes /Odometry in the independent frame chain:
  open_odom -> open_base
"""

import math

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class FakeFastlioOdom(object):
    def __init__(self):
        self.topic = rospy.get_param("~odom_topic", "/Odometry")
        self.frame_id = rospy.get_param("~frame_id", "open_odom")
        self.child_frame = rospy.get_param("~child_frame_id", "open_base")
        self.radius = rospy.get_param("~radius", 5.0)
        self.linear_speed = rospy.get_param("~linear_speed", 0.35)
        self.rate_hz = rospy.get_param("~rate", 20.0)

        self.theta = 0.0
        self.pub = rospy.Publisher(self.topic, Odometry, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)

        omega = self.linear_speed / max(self.radius, 0.1)
        rospy.loginfo(
            "[fake_fastlio_odom] topic=%s frame=%s->%s R=%.1fm v=%.2f w=%.3f",
            self.topic,
            self.frame_id,
            self.child_frame,
            self.radius,
            self.linear_speed,
            omega,
        )

    def tick(self, _event):
        dt = 1.0 / self.rate_hz
        omega = self.linear_speed / max(self.radius, 0.1)

        self.theta = normalize_angle(self.theta + omega * dt)

        x = self.radius * math.cos(self.theta)
        y = self.radius * math.sin(self.theta)

        dx = -self.radius * omega * math.sin(self.theta)
        dy = self.radius * omega * math.cos(self.theta)
        yaw = math.atan2(dy, dx)

        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame
        msg.pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=yaw_to_quat(yaw),
        )
        msg.twist.twist = Twist(
            linear=Vector3(x=self.linear_speed, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=omega),
        )
        self.pub.publish(msg)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def main():
    rospy.init_node("fake_fastlio_odom")
    FakeFastlioOdom()
    rospy.spin()


if __name__ == "__main__":
    main()
