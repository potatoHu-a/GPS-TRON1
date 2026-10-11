#!/usr/bin/env python3
"""Offline tests for the TRON1 FAST-LIO FRD to ROS FLU conversion."""

import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from fastlio_frame import (  # noqa: E402
    convert_fastlio_frd_to_flu,
    convert_fastlio_twist_frd_to_flu,
    quaternion_yaw,
)


def yaw_quaternion(degrees):
    half = 0.5 * math.radians(degrees)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def near(actual, expected, tolerance=1.0e-6):
    return abs(actual - expected) <= tolerance


def check(name, condition, detail=""):
    print("{} {}{}".format("OK" if condition else "FAIL", name,
                            " " + detail if detail else ""))
    return condition


def main():
    passed = True

    # FRD positive yaw is physically clockwise. The same physical pose in ROS
    # FLU must therefore have negative (clockwise) REP-103 yaw.
    position, orientation = convert_fastlio_frd_to_flu(
        (1.0, 2.0, 3.0), yaw_quaternion(30.0)
    )
    passed &= check("position_basis_change", position == (1.0, -2.0, -3.0), str(position))
    passed &= check(
        "physical_clockwise_is_ros_negative",
        near(math.degrees(quaternion_yaw(orientation)), -30.0),
        "yaw={:.3f}".format(math.degrees(quaternion_yaw(orientation))),
    )

    _, orientation = convert_fastlio_frd_to_flu(
        (0.0, 0.0, 0.0), yaw_quaternion(-30.0)
    )
    passed &= check(
        "physical_counterclockwise_is_ros_positive",
        near(math.degrees(quaternion_yaw(orientation)), 30.0),
        "yaw={:.3f}".format(math.degrees(quaternion_yaw(orientation))),
    )

    forward, angular = convert_fastlio_twist_frd_to_flu(
        (1.0, 0.0, 0.0), (0.0, 0.0, 0.4)
    )
    passed &= check("forward_remains_positive_x", forward == (1.0, 0.0, 0.0))
    passed &= check("yaw_rate_uses_same_basis", angular == (0.0, 0.0, -0.4))

    left, _ = convert_fastlio_twist_frd_to_flu(
        (0.0, -1.0, 0.0), (0.0, 0.0, 0.0)
    )
    passed &= check("physical_left_is_ros_positive_y", left == (0.0, 1.0, 0.0))

    if passed:
        print("ALL TESTS PASSED")
        return 0
    print("SOME TESTS FAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
