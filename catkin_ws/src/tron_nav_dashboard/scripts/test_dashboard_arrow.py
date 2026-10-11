#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for dashboard map arrow rendering (run: python3 test_dashboard_arrow.py)."""

import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(SCRIPT_DIR), "src"))

from tron_nav_dashboard.dashboard_geometry import arrow_tip_xy  # noqa: E402


def assert_direction(name, point, yaw_deg, expected_dx_sign, expected_dy_sign):
    tip_x, tip_y = arrow_tip_xy(point[0], point[1], math.radians(yaw_deg), 100.0)
    delta_x = tip_x - point[0]
    delta_y = tip_y - point[1]
    def sign(value):
        if abs(value) < 1.0e-6:
            return 0.0
        return 1.0 if value > 0.0 else -1.0

    ok = sign(delta_x) == expected_dx_sign and sign(delta_y) == expected_dy_sign
    status = "OK" if ok else "FAIL"
    print(
        "{} {}: dx={:.3f} dy={:.3f} expected_sign=({:+.0f},{:+.0f})".format(
            status, name, delta_x, delta_y, expected_dx_sign, expected_dy_sign
        )
    )
    return ok


def test_cardinal_directions():
    point = (0.0, 0.0)
    ok = assert_direction("east_0", point, 0.0, 1.0, 0.0)
    ok &= assert_direction("north_90", point, 90.0, 0.0, -1.0)
    ok &= assert_direction("west_180", point, 180.0, -1.0, 0.0)
    ok &= assert_direction("south_-90", point, -90.0, 0.0, 1.0)
    return ok


def test_counterclockwise_rotation():
    point = (0.0, 0.0)
    tip0 = arrow_tip_xy(point[0], point[1], math.radians(0.0), 100.0)
    tip30 = arrow_tip_xy(point[0], point[1], math.radians(30.0), 100.0)
    cross = tip0[0] * tip30[1] - tip0[1] * tip30[0]
    ok = cross < 0.0
    status = "OK" if ok else "FAIL"
    print("{} ccw_0_to_30: cross={:.3f}".format(status, cross))
    return ok


def main():
    results = [test_cardinal_directions(), test_counterclockwise_rotation()]
    if all(results):
        print("ALL TESTS PASSED")
        return 0
    print("SOME TESTS FAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
