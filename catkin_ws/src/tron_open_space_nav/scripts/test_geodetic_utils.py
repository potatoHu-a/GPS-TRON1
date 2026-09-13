#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for WGS84 ENU distance scale (run: python3 test_geodetic_utils.py)."""

import math
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from geodetic_utils import enu_to_geodetic, geodetic_to_enu, haversine_distance_m  # noqa: E402


def _assert_near(name, actual, expected, tol_ratio=0.01):
    err = abs(actual - expected)
    ratio = err / max(expected, 1e-6)
    ok = ratio <= tol_ratio
    status = "OK" if ok else "FAIL"
    print(
        "%s %s: actual=%.4f expected=%.4f err=%.4f ratio=%.4f"
        % (status, name, actual, expected, err, ratio)
    )
    return ok


def test_enu_vs_haversine():
    # Wuhan area ~30.5N, 114.3E
    lat0, lon0, alt0 = 30.5928, 114.3055, 30.0
    cases = [
        (20.0, 0.0, "20m north"),
        (0.0, 20.0, "20m east"),
        (15.0, 10.0, "diagonal"),
        (-30.0, 25.0, "mixed"),
    ]
    all_ok = True
    for de, dn, label in cases:
        expected = math.hypot(de, dn)
        lat1, lon1, _alt1 = enu_to_geodetic(de, dn, 0.0, lat0, lon0, alt0)
        haversine = haversine_distance_m(lat0, lon0, lat1, lon1)
        east, north, _up = geodetic_to_enu(lat1, lon1, alt0, lat0, lon0, alt0)
        enu = math.hypot(east, north)

        all_ok &= _assert_near("haversine " + label, haversine, expected, 0.01)
        all_ok &= _assert_near("ENU " + label, enu, expected, 0.01)
    return all_ok


def test_known_geodesic_pair():
    """Two points ~19.9m apart (handheld GPS field sanity)."""
    lat0, lon0 = 30.592800, 114.305500
    lat1, lon1 = 30.592979, 114.305500
    haversine = haversine_distance_m(lat0, lon0, lat1, lon1)
    east, north, _ = geodetic_to_enu(lat1, lon1, 0.0, lat0, lon0, 0.0)
    enu = math.hypot(east, north)
    ok = _assert_near("north ~20m haversine", haversine, 19.9, 0.02)
    ok &= _assert_near("north ~20m ENU", enu, 19.9, 0.01)
    return ok


def main():
    print("=== geodetic_utils unit tests ===")
    ok = test_enu_vs_haversine() and test_known_geodesic_pair()
    print("=== RESULT: %s ===" % ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
