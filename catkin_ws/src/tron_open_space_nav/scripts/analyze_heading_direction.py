#!/usr/bin/env python3
"""Summarize recorded raw/normalized/fused motion directions from a rosbag."""

import argparse
import math
import os
import sys

import rosbag

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from fastlio_frame import convert_fastlio_frd_to_flu, quaternion_yaw  # noqa: E402


def bearing(first, last):
    return math.degrees(math.atan2(last[1] - first[1], last[0] - first[0]))


def distance(first, last):
    return math.hypot(last[0] - first[0], last[1] - first[1])


def endpoint(values):
    return None if len(values) < 2 else (values[0], values[-1])


def describe(label, values):
    pair = endpoint(values)
    if pair is None:
        print("{}: N/A (fewer than two samples)".format(label))
        return
    first, last = pair
    print(
        "{}: count={} delta=({:+.3f},{:+.3f}) distance={:.3f}m bearing={:+.2f}deg".format(
            label,
            len(values),
            last[0] - first[0],
            last[1] - first[1],
            distance(first, last),
            bearing(first, last),
        )
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()
    raw = []
    normalized = []
    fused = []
    gps = []
    raw_yaw = []
    normalized_yaw = []
    imu_z = []
    imu_frames = set()

    with rosbag.Bag(args.bag) as bag:
        topics = bag.get_type_and_topic_info().topics
        for topic, msg, _stamp in bag.read_messages(
            topics=["/Odometry", "/open_nav/odom", "/open_nav/gps_filtered_pose", "/livox/imu"]
        ):
            if topic == "/Odometry":
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                source_q = (q.x, q.y, q.z, q.w)
                nav_p, nav_q = convert_fastlio_frd_to_flu(
                    (p.x, p.y, p.z), source_q
                )
                raw.append((p.x, p.y))
                normalized.append(nav_p[:2])
                raw_yaw.append(math.degrees(quaternion_yaw(source_q)))
                normalized_yaw.append(math.degrees(quaternion_yaw(nav_q)))
            elif topic == "/open_nav/odom":
                fused.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
            elif topic == "/open_nav/gps_filtered_pose":
                gps.append((msg.pose.position.x, msg.pose.position.y))
            elif topic == "/livox/imu":
                imu_z.append(msg.angular_velocity.z)
                imu_frames.add(msg.header.frame_id or "<empty>")

    print("bag: {}".format(os.path.abspath(args.bag)))
    describe("FAST-LIO raw FRD", raw)
    describe("FAST-LIO normalized FLU", normalized)
    describe("GPS ENU", gps)
    describe("fused /open_nav/odom", fused)
    if raw_yaw:
        print(
            "yaw: raw first/last={:+.2f}/{:+.2f}deg normalized={:+.2f}/{:+.2f}deg".format(
                raw_yaw[0], raw_yaw[-1], normalized_yaw[0], normalized_yaw[-1]
            )
        )
    if imu_z:
        print(
            "IMU: count={} frame={} angular_velocity.z range={:+.4f}..{:+.4f}; "
            "interpret only after applying that frame's TF to body FLU".format(
                len(imu_z), ",".join(sorted(imu_frames)), min(imu_z), max(imu_z)
            )
        )
    else:
        print("IMU: N/A (/livox/imu is not recorded)")
    missing = [topic for topic in ("/Odometry", "/open_nav/odom", "/open_nav/gps_filtered_pose", "/livox/imu") if topic not in topics]
    if missing:
        print("missing topics: {}".format(", ".join(missing)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
