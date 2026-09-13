#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Localization diagnostics: path length + net displacement + correction breakdown.

Path totals are reference only (GPS jitter inflates cumulative distance).
For straight-line field tests, use net displacement vs measured_distance.
"""

import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from geodetic_utils import haversine_distance_m, is_valid_navsat, navsat_horizontal_sigma

try:
    from tron_open_space_nav.msg import LocalizationStatus
    from tron_open_space_nav.srv import (
        ResetLocalizationDistance,
        ResetLocalizationDistanceResponse,
        SetMeasuredDistance,
        SetMeasuredDistanceResponse,
    )
except ImportError:
    LocalizationStatus = None
    ResetLocalizationDistance = None
    SetMeasuredDistance = None


class PathTracker(object):
    """Incremental path length since reset."""

    def __init__(self):
        self.total = 0.0
        self._last = None

    def reset(self):
        self.total = 0.0
        self._last = None

    def update_xy(self, x, y):
        if self._last is not None:
            self.total += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)

    def update_latlon(self, lat, lon):
        if self._last is not None:
            self.total += haversine_distance_m(self._last[0], self._last[1], lat, lon)
        self._last = (lat, lon)


class DeadbandPathTracker(object):
    """Path length with minimum segment length (diagnostics only)."""

    def __init__(self, deadband_m):
        self.deadband = deadband_m
        self.total = 0.0
        self._last = None

    def reset(self):
        self.total = 0.0
        self._last = None

    def update_latlon(self, lat, lon):
        if self._last is not None:
            seg = haversine_distance_m(self._last[0], self._last[1], lat, lon)
            if seg >= self.deadband:
                self.total += seg
                self._last = (lat, lon)
        else:
            self._last = (lat, lon)


class GpsDisplacementTracker(object):
    def __init__(self):
        self.origin = None
        self.current = None

    def reset(self):
        self.origin = None
        self.current = None

    def update(self, lat, lon):
        if self.origin is None:
            self.origin = (lat, lon)
        self.current = (lat, lon)

    def displacement(self):
        if self.origin is None or self.current is None:
            return 0.0
        return haversine_distance_m(
            self.origin[0], self.origin[1], self.current[0], self.current[1]
        )


class XyDisplacementTracker(object):
    def __init__(self):
        self.origin = None
        self.current = None

    def reset(self):
        self.origin = None
        self.current = None

    def update(self, x, y):
        if self.origin is None:
            self.origin = (x, y)
        self.current = (x, y)

    def displacement(self):
        if self.origin is None or self.current is None:
            return 0.0
        return math.hypot(
            self.current[0] - self.origin[0], self.current[1] - self.origin[1]
        )

    def delta_xy(self):
        if self.origin is None or self.current is None:
            return 0.0, 0.0
        return self.current[0] - self.origin[0], self.current[1] - self.origin[1]


class LocalizationDiagnostics(object):
    def __init__(self):
        self.measured_distance = float(rospy.get_param("~measured_distance", 0.0))
        self.report_interval = float(rospy.get_param("~report_interval", 1.0))
        self.gps_distance_deadband = float(rospy.get_param("~gps_distance_deadband", 1.0))

        # Path length (reference)
        self.gps_raw_path = PathTracker()
        self.gps_filtered_path = PathTracker()
        self.global_path = PathTracker()
        self.fastlio_path = PathTracker()
        self.open_nav_path = PathTracker()
        self.gps_deadband_path = DeadbandPathTracker(self.gps_distance_deadband)
        self.gps_correction_path = PathTracker()
        self.fastlio_motion_path = PathTracker()

        # Net displacement
        self.gps_raw_disp = GpsDisplacementTracker()
        self.gps_filtered_disp = XyDisplacementTracker()
        self.global_disp = XyDisplacementTracker()
        self.fastlio_disp = XyDisplacementTracker()
        self.open_nav_disp = XyDisplacementTracker()

        self._gps_sigma = 0.0
        self._gps_valid = False
        self._gps_rejected = 0
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._last_offset = None

        if LocalizationStatus is not None:
            self.status_pub = rospy.Publisher(
                "/open_nav/localization_status", LocalizationStatus, queue_size=1, latch=False
            )
        else:
            self.status_pub = None

        rospy.Subscriber("/gps/fix", NavSatFix, self._gps_cb, queue_size=20)
        rospy.Subscriber(
            "/open_nav/gps_filtered_pose", PoseStamped, self._gps_filtered_cb, queue_size=20
        )
        rospy.Subscriber("/global_pose", PoseStamped, self._global_cb, queue_size=20)
        rospy.Subscriber("/Odometry", Odometry, self._fastlio_cb, queue_size=50)
        rospy.Subscriber("/open_nav/odom", Odometry, self._open_nav_cb, queue_size=50)
        if LocalizationStatus is not None:
            rospy.Subscriber(
                "/open_nav/fusion_status", LocalizationStatus, self._fusion_cb, queue_size=10
            )

        if ResetLocalizationDistance is not None:
            rospy.Service(
                "/open_nav/localization/reset_distance",
                ResetLocalizationDistance,
                self._reset_service,
            )
        if SetMeasuredDistance is not None:
            rospy.Service(
                "/open_nav/localization/set_measured",
                SetMeasuredDistance,
                self._set_measured_service,
            )

        rospy.Timer(rospy.Duration(self.report_interval), self._report_timer, oneshot=False)
        rospy.loginfo("[localization_diag] ready; call /open_nav/localization/reset_distance")

    def _fusion_cb(self, msg):
        if msg.note != "fusion_offset":
            return
        ox = msg.gps_correction_offset_x
        oy = msg.gps_correction_offset_y
        if self._last_offset is not None:
            self.gps_correction_path.update_xy(ox, oy)
        else:
            self.gps_correction_path._last = (ox, oy)
        self._last_offset = (ox, oy)
        self._offset_x = ox
        self._offset_y = oy
        self._gps_rejected = max(self._gps_rejected, msg.gps_rejected_count)
        if msg.gps_sigma > 0.0:
            self._gps_sigma = msg.gps_sigma

    def _gps_cb(self, msg):
        if not is_valid_navsat(msg):
            self._gps_valid = False
            return
        self._gps_valid = True
        sigma = navsat_horizontal_sigma(msg)
        if sigma is not None:
            self._gps_sigma = sigma
        self.gps_raw_path.update_latlon(msg.latitude, msg.longitude)
        self.gps_deadband_path.update_latlon(msg.latitude, msg.longitude)
        self.gps_raw_disp.update(msg.latitude, msg.longitude)

    def _gps_filtered_cb(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        self.gps_filtered_path.update_xy(x, y)
        self.gps_filtered_disp.update(x, y)

    def _global_cb(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        self.global_path.update_xy(x, y)
        self.global_disp.update(x, y)

    def _fastlio_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.fastlio_path.update_xy(x, y)
        self.fastlio_motion_path.update_xy(x, y)
        self.fastlio_disp.update(x, y)

    def _open_nav_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.open_nav_path.update_xy(x, y)
        self.open_nav_disp.update(x, y)

    def _reset_all(self):
        for t in (
            self.gps_raw_path, self.gps_filtered_path, self.global_path,
            self.fastlio_path, self.open_nav_path, self.gps_correction_path,
            self.fastlio_motion_path,
        ):
            t.reset()
        self.gps_deadband_path.reset()
        for t in (
            self.gps_raw_disp, self.gps_filtered_disp, self.global_disp,
            self.fastlio_disp, self.open_nav_disp,
        ):
            t.reset()
        self._last_offset = None

    def _reset_service(self, _req):
        self._reset_all()
        rospy.loginfo("[localization_diag] distance counters reset")
        return ResetLocalizationDistanceResponse(success=True, message="distance counters reset")

    def _disp_ratio(self, displacement):
        if self.measured_distance <= 1e-3:
            return 0.0
        return displacement / self.measured_distance

    def _print_summary(self):
        m = self.measured_distance
        lines = [
            "=== localization summary (straight-line test) ===",
            "Measured real distance: %.2f m" % m,
            "",
            "PATH LENGTH (reference, GPS jitter inflates):",
            "  gps_raw:      %.2f m" % self.gps_raw_path.total,
            "  gps_deadband: %.2f m" % self.gps_deadband_path.total,
            "  global:       %.2f m" % self.global_path.total,
            "  fastlio:      %.2f m" % self.fastlio_path.total,
            "  open_nav:     %.2f m" % self.open_nav_path.total,
            "",
            "NET DISPLACEMENT (use for scale judgment):",
            "  gps_raw:      %.2f m" % self.gps_raw_disp.displacement(),
            "  gps_filtered: %.2f m" % self.gps_filtered_disp.displacement(),
            "  global:       %.2f m" % self.global_disp.displacement(),
            "  fastlio:      %.2f m" % self.fastlio_disp.displacement(),
            "  open_nav:     %.2f m" % self.open_nav_disp.displacement(),
            "",
            "CORRECTION:",
            "  gps_correction_path: %.2f m" % self.gps_correction_path.total,
            "  fastlio_motion_path: %.2f m" % self.fastlio_motion_path.total,
        ]
        if m > 1.0:
            lines.append("")
            lines.append("Displacement / measured:")
            for label, disp in (
                ("gps_raw", self.gps_raw_disp.displacement()),
                ("global", self.global_disp.displacement()),
                ("fastlio", self.fastlio_disp.displacement()),
                ("open_nav", self.open_nav_disp.displacement()),
            ):
                lines.append(
                    "  %s: %.2f m (ratio %.3f, err %+.2f m)"
                    % (label, disp, self._disp_ratio(disp), disp - m)
                )
        rospy.loginfo("\n".join(lines))

    def _set_measured_service(self, req):
        self.measured_distance = float(req.distance_m)
        self._print_summary()
        return SetMeasuredDistanceResponse(
            success=True,
            message="measured_distance=%.2f" % self.measured_distance,
        )

    def _report_timer(self, _event):
        fdx, fdy = self.fastlio_disp.delta_xy()
        gdx, gdy = self.global_disp.delta_xy()
        odx, ody = self.open_nav_disp.delta_xy()

        rospy.loginfo(
            "[localization_diag]\n"
            "PATH LENGTH:\n"
            "  gps_raw=%.2f gps_deadband=%.2f global=%.2f fastlio=%.2f open_nav=%.2f\n"
            "NET DISPLACEMENT:\n"
            "  gps=%.2f global=%.2f fastlio=%.2f open_nav=%.2f\n"
            "CORRECTION:\n"
            "  gps_correction_distance=%.2f fastlio_motion=%.2f",
            self.gps_raw_path.total,
            self.gps_deadband_path.total,
            self.global_path.total,
            self.fastlio_path.total,
            self.open_nav_path.total,
            self.gps_raw_disp.displacement(),
            self.global_disp.displacement(),
            self.fastlio_disp.displacement(),
            self.open_nav_disp.displacement(),
            self.gps_correction_path.total,
            self.fastlio_motion_path.total,
        )

        if self.measured_distance > 1.0:
            rospy.loginfo_throttle(
                5.0,
                "[localization_diag] displacement/measured: "
                "global=%.3f fastlio=%.3f open_nav=%.3f",
                self._disp_ratio(self.global_disp.displacement()),
                self._disp_ratio(self.fastlio_disp.displacement()),
                self._disp_ratio(self.open_nav_disp.displacement()),
            )

        if self.status_pub is None:
            return

        msg = LocalizationStatus()
        msg.header.stamp = rospy.Time.now()
        msg.gps_sigma = float(self._gps_sigma)
        msg.gps_fix_valid = self._gps_valid
        msg.gps_rejected_count = self._gps_rejected
        msg.gps_raw_distance = float(self.gps_raw_path.total)
        msg.gps_filtered_distance = float(self.gps_filtered_path.total)
        msg.global_pose_distance = float(self.global_path.total)
        msg.fastlio_distance = float(self.fastlio_path.total)
        msg.open_nav_distance = float(self.open_nav_path.total)
        msg.gps_correction_offset_x = float(self._offset_x)
        msg.gps_correction_offset_y = float(self._offset_y)
        msg.measured_distance = float(self.measured_distance)
        msg.gps_raw_displacement = float(self.gps_raw_disp.displacement())
        msg.gps_filtered_displacement = float(self.gps_filtered_disp.displacement())
        msg.global_pose_displacement = float(self.global_disp.displacement())
        msg.fastlio_displacement = float(self.fastlio_disp.displacement())
        msg.open_nav_displacement = float(self.open_nav_disp.displacement())
        msg.gps_deadband_distance = float(self.gps_deadband_path.total)
        msg.gps_correction_distance = float(self.gps_correction_path.total)
        msg.fastlio_motion_distance = float(self.fastlio_motion_path.total)
        msg.open_nav_path_distance = float(self.open_nav_path.total)
        msg.fastlio_dx = float(fdx)
        msg.fastlio_dy = float(fdy)
        msg.global_dx = float(gdx)
        msg.global_dy = float(gdy)
        msg.open_nav_dx = float(odx)
        msg.open_nav_dy = float(ody)
        gps_d = self.gps_raw_disp.displacement()
        msg.ratio_global_gps = (
            self.global_disp.displacement() / gps_d if gps_d > 1.0 else 0.0
        )
        msg.ratio_fastlio_gps = (
            self.fastlio_disp.displacement() / gps_d if gps_d > 1.0 else 0.0
        )
        msg.ratio_open_nav_gps = (
            self.open_nav_disp.displacement() / gps_d if gps_d > 1.0 else 0.0
        )
        msg.note = "use net displacement for straight-line tests"
        self.status_pub.publish(msg)


def main():
    rospy.init_node("localization_diagnostics")
    LocalizationDiagnostics()
    rospy.spin()


if __name__ == "__main__":
    main()
