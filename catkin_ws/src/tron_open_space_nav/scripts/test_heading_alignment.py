#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for heading alignment math (run: python3 test_heading_alignment.py)."""

import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from geodetic_utils import normalize_angle  # noqa: E402


def compose_map_pose(map_odom_x, map_odom_y, map_odom_yaw, odom_x, odom_y, odom_yaw,
                     heading_aligned, yaw_alignment_offset):
    rotation_yaw = map_odom_yaw
    if heading_aligned:
        rotation_yaw = normalize_angle(rotation_yaw + yaw_alignment_offset)
    cos_m = math.cos(rotation_yaw)
    sin_m = math.sin(rotation_yaw)
    map_x = map_odom_x + cos_m * odom_x - sin_m * odom_y
    map_y = map_odom_y + sin_m * odom_x + cos_m * odom_y
    map_yaw = normalize_angle(rotation_yaw + odom_yaw)
    return map_x, map_y, map_yaw


def compute_offset(global_heading, fastlio_yaw):
    return normalize_angle(global_heading - fastlio_yaw)


def deg(angle):
    return math.degrees(angle)


def assert_near(name, actual, expected, tol=1e-6):
    ok = abs(actual - expected) <= tol
    status = "OK" if ok else "FAIL"
    print("{} {}: actual={:.6f} expected={:.6f}".format(status, name, actual, expected))
    return ok


def test_offset_and_global_yaw():
    # Inputs here are already normalized ROS FLU yaw values. Raw FAST-LIO FRD
    # conversion is covered independently by test_fastlio_frame.py.
    normalized_fastlio_30 = math.radians(30.0)
    global_80 = math.radians(80.0)
    offset = compute_offset(global_80, normalized_fastlio_30)
    ok = assert_near("offset_30_to_80", offset, math.radians(50.0))

    _, _, global_yaw = compose_map_pose(
        0.0, 0.0, 0.0, 0.0, 0.0, normalized_fastlio_30, True, offset
    )
    ok &= assert_near("global_yaw_after_align", global_yaw, global_80)

    _, _, global_yaw = compose_map_pose(
        0.0, 0.0, 0.0, 0.0, 0.0, math.radians(50.0), True, offset
    )
    ok &= assert_near("global_yaw_fastlio_50", global_yaw, math.radians(100.0))

    _, _, global_yaw = compose_map_pose(
        0.0, 0.0, 0.0, 0.0, 0.0, math.radians(10.0), True, offset
    )
    ok &= assert_near("global_yaw_fastlio_10", global_yaw, math.radians(60.0))
    return ok


def test_wrap_179():
    fastlio_yaw = math.radians(179.0)
    global_heading = math.radians(-179.0)
    offset = compute_offset(global_heading, fastlio_yaw)
    _, _, global_yaw = compose_map_pose(
        0.0, 0.0, 0.0, 0.0, 0.0, fastlio_yaw, True, offset
    )
    ok = assert_near("wrap_global_yaw", global_yaw, global_heading)
    ok &= assert_near("offset_wrap", offset, math.radians(2.0), tol=1e-5)
    return ok


def test_position_rotation_uses_same_yaw():
    offset = math.radians(50.0)
    odom_x, odom_y = 1.0, 0.0
    map_x, map_y, map_yaw = compose_map_pose(
        0.0, 0.0, 0.0, odom_x, odom_y, math.radians(30.0), True, offset
    )
    expected_x = math.cos(math.radians(50.0))
    expected_y = math.sin(math.radians(50.0))
    ok = assert_near("rotated_x", map_x, expected_x)
    ok &= assert_near("rotated_y", map_y, expected_y)
    ok &= assert_near("rotated_yaw", map_yaw, math.radians(80.0))
    return ok


def test_alignment_keeps_current_position_continuous():
    old_rotation = math.radians(0.0)
    local_x, local_y = 10.0, 5.0
    old_tx, old_ty = 100.0, 200.0
    before_x = old_tx + math.cos(old_rotation) * local_x - math.sin(old_rotation) * local_y
    before_y = old_ty + math.sin(old_rotation) * local_x + math.cos(old_rotation) * local_y
    new_rotation = math.radians(50.0)
    new_tx = before_x - (math.cos(new_rotation) * local_x - math.sin(new_rotation) * local_y)
    new_ty = before_y - (math.sin(new_rotation) * local_x + math.cos(new_rotation) * local_y)
    after_x = new_tx + math.cos(new_rotation) * local_x - math.sin(new_rotation) * local_y
    after_y = new_ty + math.sin(new_rotation) * local_x + math.cos(new_rotation) * local_y
    ok = assert_near("position_continuity_x", after_x, before_x)
    ok &= assert_near("position_continuity_y", after_y, before_y)
    next_x = new_tx + math.cos(new_rotation) * (local_x + 1.0) - math.sin(new_rotation) * local_y
    next_y = new_ty + math.sin(new_rotation) * (local_x + 1.0) + math.cos(new_rotation) * local_y
    ok &= assert_near("position_increment_x", next_x - after_x, math.cos(new_rotation))
    ok &= assert_near("position_increment_y", next_y - after_y, math.sin(new_rotation))
    return ok


def main():
    tests = [
        test_offset_and_global_yaw,
        test_wrap_179,
        test_position_rotation_uses_same_yaw,
        test_alignment_keeps_current_position_continuous,
    ]
    results = [test() for test in tests]
    if all(results):
        print("ALL TESTS PASSED")
        return 0
    print("SOME TESTS FAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
